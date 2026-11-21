# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Flexion-space control of the CoDe hand: a policy commands and observes PIP flexion q, each hand turns it into
its own servo joint angle theta_H(q) (Study 3, ``design/SSoT/research-objective.md`` Section 8).

- :class:`FlexionTables`: the exported four-bar tables (``four_bar[digit]``: ``pip_deg``, ``ap_deg``) as torch lookups
  for q -> theta, theta -> q and the transmission ratio dtheta/dq (the same tables and rules as
  :mod:`code_isaaclab.joint_map`, which checks them against the design kinematics).
- :class:`FlexionEMAAction`: upstream's EMA joint-position-to-limits action with the PIP servo entries in flexion:
  an action in [-1, 1] is rescaled to the flexion limits, smoothed in flexion, clamped, and sent as theta_H(q). With
  ``flexion_stiffness`` (and ``flexion_damping``) set, the PIP servos' implicit gains are rescheduled every env step
  to K_theta = K_q* / (dtheta/dq)^2 (D_theta = D_q* / (dtheta/dq)^2) at the current flexion, so every hand has the
  same flexion stiffness and damping; the gains are written to the solver through the articulation, which notifies
  Newton of the change.
- :func:`flexion_joint_pos_normalized`, :func:`flexion_joint_vel`: the actuated joints' positions and velocities with
  the PIP entries in flexion (q normalized by the flexion limits; dq/dt = dtheta/dt / (dtheta/dq)).

Other actuated joints (abduction, MCP) pass through unchanged.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import MISSING

import torch

import isaaclab.utils.math as math_utils
from isaaclab.envs.mdp.actions.actions_cfg import EMAJointPositionToLimitsActionCfg
from isaaclab.envs.mdp.actions.joint_actions_to_limits import EMAJointPositionToLimitsAction
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from code_isaaclab.loops import _interp_1d

SERVO_SUFFIX = "_pip_servo"


class FlexionTables:
    """Per PIP servo joint (by name): flexion q and servo joint theta samples [rad], and dtheta/dq."""

    def __init__(self, tables: Mapping[str, Mapping[str, Sequence[float]]], device="cpu", dtype=torch.float32):
        self._q, self._theta, self._ratio = {}, {}, {}
        for digit, raw in tables.items():
            q = torch.tensor([math.radians(v) for v in raw["pip_deg"]], dtype=torch.float64)
            theta = torch.tensor([math.radians(v) for v in raw["ap_deg"]], dtype=torch.float64)
            order = torch.argsort(q)
            q, theta = q[order], theta[order]
            steps = theta[1:] - theta[:-1]
            if not (bool((steps > 0).all()) or bool((steps < 0).all())):
                raise ValueError(f"{digit}: theta(q) is not monotonic; flexion control needs its inverse")
            ratio = torch.empty_like(q)
            ratio[1:-1] = (theta[2:] - theta[:-2]) / (q[2:] - q[:-2])
            ratio[0] = (theta[1] - theta[0]) / (q[1] - q[0])
            ratio[-1] = (theta[-1] - theta[-2]) / (q[-1] - q[-2])
            name = f"{digit}{SERVO_SUFFIX}"
            self._q[name] = q.to(device=device, dtype=dtype)
            self._theta[name] = theta.to(device=device, dtype=dtype)
            self._ratio[name] = ratio.to(device=device, dtype=dtype)

    @property
    def joints(self) -> tuple[str, ...]:
        return tuple(self._q)

    def servo(self, name: str, q: torch.Tensor) -> torch.Tensor:
        return _interp_1d(q, self._q[name].to(q), self._theta[name].to(q))

    def flexion(self, name: str, theta: torch.Tensor) -> torch.Tensor:
        xp, fp = self._theta[name].to(theta), self._q[name].to(theta)
        if xp[0] > xp[-1]:
            xp, fp = xp.flip(0), fp.flip(0)
        return _interp_1d(theta, xp, fp)

    def ratio(self, name: str, q: torch.Tensor) -> torch.Tensor:
        return _interp_1d(q, self._q[name].to(q), self._ratio[name].to(q))


def _pip_columns(joint_names: Sequence[str], tables: FlexionTables) -> list[tuple[int, str]]:
    return [(k, n) for k, n in enumerate(joint_names) if n in tables.joints]


class FlexionEMAAction(EMAJointPositionToLimitsAction):
    """EMA joint-position-to-limits action with the PIP servo entries commanded in flexion."""

    cfg: FlexionEMAActionCfg

    def __init__(self, cfg: FlexionEMAActionCfg, env):
        super().__init__(cfg, env)
        self._tables = FlexionTables(cfg.four_bar_tables, device=self.device)
        self._pip = _pip_columns(self._joint_names, self._tables)
        if not self._pip:
            raise ValueError("FlexionEMAAction: none of the action's joints is a PIP servo with a four-bar table")
        lo, hi = (math.radians(v) for v in cfg.flexion_limits_deg)
        self._cmd_limits = self._asset.data.soft_joint_pos_limits.torch[:, self._joint_ids].clone()
        for k, _ in self._pip:
            self._cmd_limits[:, k, 0], self._cmd_limits[:, k, 1] = lo, hi
        self._pip_cols = torch.tensor([k for k, _ in self._pip], device=self.device)
        self._pip_joint_ids = [self._asset.find_joints(n)[0][0] for _, n in self._pip]
        self._flexion_cmd = torch.zeros(self.num_envs, len(self._pip), device=self.device)

    def _to_flexion(self, joint_values: torch.Tensor) -> torch.Tensor:
        out = joint_values.clone()
        for k, n in self._pip:
            out[:, k] = self._tables.flexion(n, joint_values[:, k])
        return out

    def _to_servo(self, command: torch.Tensor) -> torch.Tensor:
        out = command.clone()
        for k, n in self._pip:
            out[:, k] = self._tables.servo(n, command[:, k])
        return out

    @property
    def flexion_command(self) -> torch.Tensor:
        """The last commanded flexion of each PIP joint [rad], ``(num_envs, num_pip)``."""
        return self._flexion_cmd

    def reset(self, env_ids=None) -> None:
        super().reset(env_ids)  # sets the previous targets to the current joint positions (servo angles)
        ids = slice(None) if env_ids is None else env_ids
        self._prev_applied_actions[ids] = self._to_flexion(self._prev_applied_actions[ids])

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        command = self._raw_actions * self._scale
        if self.cfg.clip is not None:
            command = torch.clamp(command, min=self._clip[:, :, 0], max=self._clip[:, :, 1])
        command = math_utils.unscale_transform(command.clamp(-1.0, 1.0), self._cmd_limits[..., 0], self._cmd_limits[..., 1])
        command = self._alpha * command + (1.0 - self._alpha) * self._prev_applied_actions
        command = torch.clamp(command, self._cmd_limits[..., 0], self._cmd_limits[..., 1])
        self._prev_applied_actions[:] = command
        self._flexion_cmd[:] = command[:, self._pip_cols]
        self._processed_actions[:] = self._to_servo(command)
        if self.cfg.flexion_stiffness is not None or self.cfg.flexion_damping is not None:
            self._schedule_gains()

    def _schedule_gains(self) -> None:
        theta = self._asset.data.joint_pos.torch[:, self._pip_joint_ids]
        ratio_sq = torch.stack([self._tables.ratio(n, self._tables.flexion(n, theta[:, i])) for i, (_, n) in
                                enumerate(self._pip)], dim=1) ** 2
        if self.cfg.flexion_stiffness is not None:
            self._asset.write_joint_stiffness_to_sim_index(
                stiffness=(self.cfg.flexion_stiffness / ratio_sq).contiguous(), joint_ids=self._pip_joint_ids)
        if self.cfg.flexion_damping is not None:
            self._asset.write_joint_damping_to_sim_index(
                damping=(self.cfg.flexion_damping / ratio_sq).contiguous(), joint_ids=self._pip_joint_ids)


@configclass
class FlexionEMAActionCfg(EMAJointPositionToLimitsActionCfg):
    """EMA joint-position action with the PIP servos commanded in flexion (see :class:`FlexionEMAAction`)."""

    class_type: type = FlexionEMAAction
    four_bar_tables: dict = MISSING
    """The description's ``four_bar`` (per digit: ``pip_deg``, ``ap_deg``, ...)."""
    flexion_limits_deg: tuple[float, float] = (0.0, 120.0)
    """The flexion range an action in [-1, 1] spans [deg]."""
    flexion_stiffness: float | None = None
    """K_q* [N m/rad]: reschedule the PIP servos' stiffness to K_q* / (dtheta/dq)^2 (None: keep the servo gains)."""
    flexion_damping: float | None = None
    """D_q* [N m s/rad]: reschedule the PIP servos' damping to D_q* / (dtheta/dq)^2 (None: keep)."""


def flexion_joint_pos_normalized(env, asset_cfg: SceneEntityCfg, four_bar_tables: dict,
                                 flexion_limits_deg: tuple[float, float] = (0.0, 120.0)) -> torch.Tensor:
    """Actuated joint positions normalized to [-1, 1], the PIP servo entries as flexion over the flexion limits."""
    asset = env.scene[asset_cfg.name]
    names = [asset.joint_names[i] for i in asset_cfg.joint_ids] if not isinstance(asset_cfg.joint_ids, slice) \
        else list(asset.joint_names)
    tables = FlexionTables(four_bar_tables, device=env.device)
    pos = asset.data.joint_pos.torch[:, asset_cfg.joint_ids].clone()
    lo = asset.data.soft_joint_pos_limits.torch[:, asset_cfg.joint_ids, 0].clone()
    hi = asset.data.soft_joint_pos_limits.torch[:, asset_cfg.joint_ids, 1].clone()
    for k, n in _pip_columns(names, tables):
        pos[:, k] = tables.flexion(n, pos[:, k])
        lo[:, k], hi[:, k] = (math.radians(v) for v in flexion_limits_deg)
    return math_utils.scale_transform(pos, lo, hi)


def flexion_joint_vel(env, asset_cfg: SceneEntityCfg, four_bar_tables: dict) -> torch.Tensor:
    """Actuated joint velocities, the PIP servo entries as flexion rate dq/dt = (dtheta/dt) / (dtheta/dq)."""
    asset = env.scene[asset_cfg.name]
    names = [asset.joint_names[i] for i in asset_cfg.joint_ids] if not isinstance(asset_cfg.joint_ids, slice) \
        else list(asset.joint_names)
    tables = FlexionTables(four_bar_tables, device=env.device)
    pos = asset.data.joint_pos.torch[:, asset_cfg.joint_ids]
    vel = asset.data.joint_vel.torch[:, asset_cfg.joint_ids].clone()
    for k, n in _pip_columns(names, tables):
        vel[:, k] = vel[:, k] / tables.ratio(n, tables.flexion(n, pos[:, k]))
    return vel


__all__ = ["FlexionEMAAction", "FlexionEMAActionCfg", "FlexionTables", "flexion_joint_pos_normalized", "flexion_joint_vel"]
