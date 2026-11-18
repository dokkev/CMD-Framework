# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Closed four-bar loop utilities for Newton and Isaac Lab.

The CoDe hand models each finger's crossed four-bar as a spanning tree (actuated ``pip_servo`` at ``A'``,
passive joints ``a`` at ``A`` and ``b`` at ``B``) plus a non-tree loop cut at ``B'``. Running that kinematics
accurately in simulation requires two runtime hooks and one diagnostic:

1. **Loop equality stiffening on MuJoCo-Warp** (:func:`stiffen_mujoco_loops`, :func:`stiffen_env_mujoco_loops`):
   MuJoCo's default ``CONNECT`` constraint (``solref[0] = 0.02 s``) stretches by ~0.3 mm under load, which
   translates to ~2.3 deg of PIP error across the short ``B-B'`` link. Tightening ``eq_solref`` (respecting
   MuJoCo's ``2 * step_dt`` stability floor) and ``eq_solimp`` brings the error down to hundredths of a degree.
2. **Closed-tree joint state synchronization** (:class:`FourBarTables`, :func:`sync_four_bar_joints_on_reset`):
   resetting only the actuated servo joints leaves ``a`` and ``b`` open so the cut snaps shut on the first step.
   Interpolating the exported ``four_bar`` closure table sets ``a`` and ``b`` consistently across environments.
3. **Body-pose closure diagnostics** (:func:`loop_residual_mm`, :func:`measure_finger_pip_deg`): solver-agnostic
   checks computed from world body poses rather than solver-internal joint coordinates.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np
import torch

from code_isaaclab.assets.description import Description, quat_xyzw_to_matrix, rotation_about


def resolve_newton_solver(sim_or_manager: Any = None) -> Any | None:
    """Extract the underlying ``newton.solvers.SolverBase`` instance from an Isaac Lab sim or manager."""
    candidates: list[Any] = []
    if sim_or_manager is not None:
        candidates.append(sim_or_manager)
        pm = getattr(sim_or_manager, "physics_manager", None)
        if pm is not None:
            candidates.extend([pm, getattr(pm, "_solver", None)])
    for owner in candidates:
        if owner is None:
            continue
        if hasattr(owner, "mjw_model") or hasattr(owner, "mj_model"):
            return owner
        inner = getattr(owner, "solver", None)
        if inner is not None:
            return inner
    try:
        from isaaclab_newton.physics import NewtonManager
    except ImportError:
        return None
    return getattr(NewtonManager, "_solver", None)


def stiffen_mujoco_loops(
    solver: Any,
    step_dt: float,
    *,
    timeconst: float | None = None,
    min_timeconst: float = 0.004,
    solimp: tuple[float, float] = (0.99, 0.999),
) -> bool:
    """Tighten MuJoCo / MuJoCo-Warp equality constraints for loop closure.

    Returns ``True`` when equality constraints were updated, or ``False`` when ``solver`` is not a MuJoCo
    solver or the model has no equality constraints (e.g. an open-chain hand or a VBD/Kamino backend).
    """
    mjw = getattr(solver, "mjw_model", None)
    if mjw is None or not hasattr(mjw, "eq_solref") or not hasattr(mjw, "eq_solimp"):
        return False
    solref_arr = mjw.eq_solref.numpy()
    if solref_arr.size == 0:
        return False
    tc = float(timeconst) if timeconst is not None else max(float(min_timeconst), 2.0 * float(step_dt))
    solimp_arr = mjw.eq_solimp.numpy()
    solref_arr[..., 0] = tc
    solimp_arr[..., 0], solimp_arr[..., 1] = float(solimp[0]), float(solimp[1])
    mjw.eq_solref.assign(solref_arr)
    mjw.eq_solimp.assign(solimp_arr)
    return True


def stiffen_env_mujoco_loops(
    env: Any,
    env_ids: torch.Tensor | None,
    min_timeconst: float = 0.004,
    solimp: tuple[float, float] = (0.99, 0.999),
) -> None:
    """Isaac Lab startup event hook tightening MuJoCo-Warp loop constraints when present."""
    del env_ids
    sim = env.sim
    substeps = max(int(getattr(sim.cfg.physics, "num_substeps", 1) or 1), 1)
    solver = resolve_newton_solver(sim)
    stiffen_mujoco_loops(solver, float(sim.cfg.dt) / substeps, min_timeconst=min_timeconst, solimp=solimp)


def _interp_1d(x: torch.Tensor, xp: torch.Tensor, fp: torch.Tensor) -> torch.Tensor:
    """Piecewise-linear 1D interpolation on torch tensors (clamped outside ``[xp[0], xp[-1]]``).

    ``xp`` must be 1D and strictly increasing; ``x`` may have arbitrary shape on the same device.
    """
    x_contig = x.contiguous()
    idx = torch.bucketize(x_contig, xp)
    hi = torch.clamp(idx, 1, xp.numel() - 1)
    lo = hi - 1
    x0, x1 = xp[lo], xp[hi]
    y0, y1 = fp[lo], fp[hi]
    denom = torch.where((x1 - x0).abs() > 1e-12, x1 - x0, torch.ones_like(x0))
    t = torch.clamp((x_contig - x0) / denom, 0.0, 1.0)
    return y0 + t * (y1 - y0)


@dataclass(frozen=True)
class _DigitTable:
    digit: str
    servo_idx: int
    a_idx: int
    b_idx: int
    pip_rad: torch.Tensor
    ap_rad: torch.Tensor
    a_rad: torch.Tensor
    b_rad: torch.Tensor


class FourBarTables:
    """Vectorized torch lookup tables mapping between PIP, servo (``A'``), and passive (``A``, ``B``) joints."""

    def __init__(
        self,
        tables: Mapping[str, Mapping[str, Sequence[float]]],
        joint_names: Sequence[str],
        *,
        device: torch.device | str = "cpu",
        dtype: torch.dtype = torch.float32,
    ) -> None:
        index = {name: i for i, name in enumerate(joint_names)}
        entries: list[_DigitTable] = []
        for digit, raw in tables.items():
            pip = np.radians(np.asarray(raw["pip_deg"], dtype=np.float64))
            ap = np.radians(np.asarray(raw["ap_deg"], dtype=np.float64))
            a = np.radians(np.asarray(raw["a_deg"], dtype=np.float64))
            b = np.radians(np.asarray(raw["b_deg"], dtype=np.float64))
            # Ensure strictly ascending abscissas for both PIP- and servo-indexed interpolation.
            if ap[0] > ap[-1]:
                pip, ap, a, b = pip[::-1], ap[::-1], a[::-1], b[::-1]
            entries.append(
                _DigitTable(
                    digit=digit,
                    servo_idx=index[f"{digit}_pip_servo"],
                    a_idx=index[f"{digit}_a"],
                    b_idx=index[f"{digit}_b"],
                    pip_rad=torch.tensor(pip.copy(), device=device, dtype=dtype),
                    ap_rad=torch.tensor(ap.copy(), device=device, dtype=dtype),
                    a_rad=torch.tensor(a.copy(), device=device, dtype=dtype),
                    b_rad=torch.tensor(b.copy(), device=device, dtype=dtype),
                )
            )
        self._entries = tuple(entries)
        self.passive_indices: tuple[int, ...] = tuple(idx for e in self._entries for idx in (e.a_idx, e.b_idx))

    @classmethod
    def from_description(
        cls,
        desc: Description,
        joint_names: Sequence[str] | None = None,
        *,
        device: torch.device | str = "cpu",
        dtype: torch.dtype = torch.float32,
    ) -> FourBarTables:
        names = [j["name"] for j in desc.joints] if joint_names is None else joint_names
        return cls(desc.four_bar, names, device=device, dtype=dtype)

    def to(self, device: torch.device | str, dtype: torch.dtype | None = None) -> FourBarTables:
        moved = FourBarTables.__new__(FourBarTables)
        moved._entries = tuple(
            _DigitTable(
                digit=e.digit,
                servo_idx=e.servo_idx,
                a_idx=e.a_idx,
                b_idx=e.b_idx,
                pip_rad=e.pip_rad.to(device=device, dtype=dtype or e.pip_rad.dtype),
                ap_rad=e.ap_rad.to(device=device, dtype=dtype or e.ap_rad.dtype),
                a_rad=e.a_rad.to(device=device, dtype=dtype or e.a_rad.dtype),
                b_rad=e.b_rad.to(device=device, dtype=dtype or e.b_rad.dtype),
            )
            for e in self._entries
        )
        moved.passive_indices = self.passive_indices
        return moved

    def _aligned(self, tensor: torch.Tensor) -> FourBarTables:
        if not self._entries:
            return self
        first = self._entries[0].ap_rad
        if first.device == tensor.device and first.dtype == tensor.dtype:
            return self
        return self.to(tensor.device, tensor.dtype)

    def close_from_servo(self, joint_pos: torch.Tensor) -> torch.Tensor:
        """Return a copy of ``joint_pos`` ``(..., num_joints)`` with passive ``a`` and ``b`` closed to ``pip_servo``."""
        out = joint_pos.clone()
        for e in self._aligned(out)._entries:
            ap = out[..., e.servo_idx]
            out[..., e.a_idx] = _interp_1d(ap, e.ap_rad, e.a_rad)
            out[..., e.b_idx] = _interp_1d(ap, e.ap_rad, e.b_rad)
        return out

    def close_from_pip(
        self, joint_pos: torch.Tensor, pip_rad: float | Mapping[str, float | torch.Tensor]
    ) -> torch.Tensor:
        """Return a copy of ``joint_pos`` with ``pip_servo``, ``a``, and ``b`` set for target ``pip_rad`` [rad]."""
        out = joint_pos.clone()
        for e in self._aligned(out)._entries:
            raw = pip_rad[e.digit] if isinstance(pip_rad, Mapping) else pip_rad
            target = torch.as_tensor(raw, device=out.device, dtype=out.dtype).expand_as(out[..., e.servo_idx])
            # Sort order of pip_rad may be ascending or descending depending on ap_rad sign; ensure ascending xp.
            xp = e.pip_rad
            if xp[0] > xp[-1]:
                xp, ap, a, b = xp.flip(0), e.ap_rad.flip(0), e.a_rad.flip(0), e.b_rad.flip(0)
            else:
                ap, a, b = e.ap_rad, e.a_rad, e.b_rad
            out[..., e.servo_idx] = _interp_1d(target, xp, ap)
            out[..., e.a_idx] = _interp_1d(target, xp, a)
            out[..., e.b_idx] = _interp_1d(target, xp, b)
        return out


_TABLE_CACHE: dict[tuple[int, tuple[str, ...], torch.device, torch.dtype], FourBarTables] = {}


def sync_four_bar_joints_on_reset(
    env: Any,
    env_ids: torch.Tensor | None,
    tables: dict[str, dict[str, list[float]]],
    asset_name: str = "robot",
) -> None:
    """Isaac Lab reset event hook closing passive four-bar joints to match the reset servo joint positions."""
    if not tables:
        return
    robot = env.scene[asset_name]
    pos_view = robot.data.joint_pos
    vel_view = robot.data.joint_vel
    q_all = pos_view if isinstance(pos_view, torch.Tensor) else pos_view.torch
    qd_all = vel_view if isinstance(vel_view, torch.Tensor) else vel_view.torch

    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=q_all.device)
    key = (id(tables), tuple(robot.joint_names), q_all.device, q_all.dtype)
    helper = _TABLE_CACHE.get(key)
    if helper is None:
        helper = _TABLE_CACHE[key] = FourBarTables(tables, robot.joint_names, device=q_all.device, dtype=q_all.dtype)

    closed_q = helper.close_from_servo(q_all[env_ids])
    closed_qd = qd_all[env_ids].clone()
    if helper.passive_indices:
        closed_qd[:, list(helper.passive_indices)] = 0.0
    robot.write_joint_state_to_sim(closed_q, closed_qd, env_ids=env_ids)


def loop_residual_mm(
    desc: Description,
    body_names: Sequence[str],
    body_pos: np.ndarray,
    body_quat_xyzw: np.ndarray,
) -> float:
    """Worst loop-cut position residual [mm] from body poses ``(..., num_bodies, 3)`` and ``(..., num_bodies, 4)``."""
    if not desc.loops:
        return 0.0
    pos = np.asarray(body_pos, dtype=float)
    quat = np.asarray(body_quat_xyzw, dtype=float)
    if pos.ndim == 2:
        pos, quat = pos[None], quat[None]
    index = {name: i for i, name in enumerate(body_names)}
    worst = 0.0
    for cut in desc.loops:
        anchor = np.asarray(cut["anchor"], dtype=float)
        p_idx, c_idx = index[cut["parent"]], index[cut["child"]]
        p_off = anchor - np.asarray(desc.body(cut["parent"])["origin"], dtype=float)
        c_off = anchor - np.asarray(desc.body(cut["child"])["origin"], dtype=float)
        for env in range(pos.shape[0]):
            p_world = pos[env, p_idx] + quat_xyzw_to_matrix(quat[env, p_idx]) @ p_off
            c_world = pos[env, c_idx] + quat_xyzw_to_matrix(quat[env, c_idx]) @ c_off
            worst = max(worst, 1e3 * float(np.linalg.norm(p_world - c_world)))
    return worst


def measure_finger_pip_deg(
    desc: Description,
    digit: str,
    body_names: Sequence[str],
    body_quat_xyzw: np.ndarray,
) -> float:
    """Distal-to-proximal flexion angle [deg] of ``digit`` measured directly from single-env body quaternions."""
    index = {name: i for i, name in enumerate(body_names)}
    axis = np.asarray(desc.joint(f"{digit}_pip_servo")["axis"], dtype=float)
    r_prox = quat_xyzw_to_matrix(body_quat_xyzw[index[f"{digit}_proximal"]])
    r_dist = quat_xyzw_to_matrix(body_quat_xyzw[index[f"{digit}_distal"]])
    return rotation_about(r_prox, r_dist, axis)


__all__ = [
    "FourBarTables",
    "loop_residual_mm",
    "measure_finger_pip_deg",
    "resolve_newton_solver",
    "stiffen_env_mujoco_loops",
    "stiffen_mujoco_loops",
    "sync_four_bar_joints_on_reset",
]
