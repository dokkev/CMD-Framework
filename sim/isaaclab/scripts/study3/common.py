# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Shared pieces of the Study 3 pilot scripts: run manifests, the diagnostic sensors, and per-step probes.

Nothing here changes the task: the sensors added to a scene read contacts and do not act on the dynamics.
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import json
import os
import platform
import subprocess
import time
from pathlib import Path
from typing import Any

import numpy as np

SIM_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = SIM_ROOT.parents[1]
LOG_ROOT = SIM_ROOT / "logs" / "study3_pilot"
TASKS = {"flat": "CoDe-Reorient-Cube-CoDeFlat", "crowned": "CoDe-Reorient-Cube-CoDeCrowned"}
"""The pilot tasks (overlapping goals, unsmoothed actions)."""
MAIN_TASKS = {v: f"{t}-Main" for v, t in TASKS.items()}
"""Study 3A main experiment tasks (code_env_cfg.CoDeReorientMainEnvCfg)."""
PROTOCOLS = {"pilot": TASKS, "main": MAIN_TASKS}
PAD_BODIES = [f"{d}_{s}" for d in ("index", "middle", "ring") for s in ("proximal", "distal")]


def run_id(prefix: str) -> str:
    """A unique, sortable identifier (results are never overwritten)."""
    return f"{prefix}_{_dt.datetime.now().strftime('%Y%m%d-%H%M%S')}_{os.getpid()}"


def _git(*args: str) -> str:
    try:
        return subprocess.run(["git", *args], cwd=REPO_ROOT, capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def versions() -> dict[str, str]:
    found = {"python": platform.python_version()}
    for name in ("isaaclab", "newton", "warp", "torch", "rsl_rl", "mujoco_warp"):
        try:
            from importlib.metadata import version

            found[name] = version(name if name != "rsl_rl" else "rsl-rl-lib")
        except Exception:  # noqa: BLE001 (any missing distribution)
            found[name] = "n/a"
    return found


def gpu() -> dict[str, Any]:
    try:
        import torch

        return {
            "device": torch.cuda.get_device_name(0),
            "max_memory_allocated_gb": round(torch.cuda.max_memory_allocated() / 1e9, 3),
            "max_memory_reserved_gb": round(torch.cuda.max_memory_reserved() / 1e9, 3),
        }
    except Exception:  # noqa: BLE001
        return {"device": "n/a"}


def manifest(**extra: Any) -> dict[str, Any]:
    """Git state, versions, GPU and the Study 3 assets, plus ``extra``."""
    from code_isaaclab.hands.code_variants import variant_manifest

    status = _git("status", "--porcelain")
    return {
        "created": _dt.datetime.now().isoformat(timespec="seconds"),
        "git_commit": _git("rev-parse", "HEAD"),
        "git_dirty": bool(status) and status != "unknown",
        "git_dirty_files": status.splitlines()[:50] if status != "unknown" else [],
        "versions": versions(),
        "gpu": gpu(),
        "assets": {v: variant_manifest(v) for v in TASKS},
        **extra,
    }


def write_json(path: Path, data: Any) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError(f"{path} exists: results are immutable, use a new run id")
    path.write_text(json.dumps(data, indent=2, default=str) + "\n", encoding="utf-8")
    return path


def file_sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


SENSORLESS_PHYSICS = ("newton_vbd",)
"""Backends whose Newton solver has no contact reporting (``SolverVBD.update_contacts`` is not implemented)."""


def hand_bodies() -> list[str]:
    from code_isaaclab.assets.description import Description
    from code_isaaclab.hands.code_variants import STUDY3_DESCRIPTION

    return [b["name"] for b in Description.load(STUDY3_DESCRIPTION).bodies]


def add_sensors(env_cfg, physics: str = "newton_mjwarp") -> list[str]:
    """Contact sensors on the cube (against each hand body) and on every hand body (against anything); none on
    a backend that cannot report contacts.

    Returns the hand bodies, in the order of the cube sensor's filter columns."""
    from isaaclab.sensors import ContactSensorCfg

    bodies = hand_bodies()
    if physics in SENSORLESS_PHYSICS:
        return bodies
    env_cfg.scene.cube_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/object",
        filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{name}" for name in bodies],
        track_contact_points=True,
    )
    env_cfg.scene.hand_contacts = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*")
    return bodies


def torch_of(value):
    import torch

    if isinstance(value, torch.Tensor):
        return value
    if hasattr(value, "torch"):
        return value.torch
    import warp as wp

    return wp.to_torch(value)


class Probe:
    """Per-step physical diagnostics of a CoDe reorientation env (all environments)."""

    def __init__(self, env, bodies: list[str]):
        import torch

        from code_isaaclab.assets.description import Description
        from code_isaaclab.hands.code_variants import STUDY3_DESCRIPTION
        from code_isaaclab.loops import FourBarTables

        self.env = env.unwrapped
        self.robot = self.env.scene["robot"]
        self.cube = self.env.scene["object"]
        self.command = self.env.command_manager.get_term("object_pose")
        self.desc = Description.load(STUDY3_DESCRIPTION)
        self.bodies = bodies
        self.actuated = self.robot.find_joints(self.env.cfg.actuated_joint_names, preserve_order=True)[0]
        self.tables = FourBarTables.from_description(self.desc, self.robot.joint_names, device=self.env.device)
        self.cube_sensor = self.env.scene.sensors.get("cube_contacts")
        self.hand_sensor = self.env.scene.sensors.get("hand_contacts")
        self.robot_body_names = list(self.robot.body_names)
        self.cut = []
        for loop in self.desc.loops:
            p, c = self.robot_body_names.index(loop["parent"]), self.robot_body_names.index(loop["child"])
            anchor = np.asarray(loop["anchor"], dtype=float)
            self.cut.append(
                (
                    p,
                    c,
                    torch.tensor(anchor - self.desc.body(loop["parent"])["origin"], device=self.env.device),
                    torch.tensor(anchor - self.desc.body(loop["child"])["origin"], device=self.env.device),
                )
            )
        self.pad_columns = [self.bodies.index(name) for name in PAD_BODIES]

    def hold_action(self):
        """The action whose targets are the hand's default joint positions (the extended pose)."""
        limits = torch_of(self.robot.data.soft_joint_pos_limits)[:, self.actuated]
        default = torch_of(self.robot.data.default_joint_pos)[:, self.actuated]
        return 2.0 * (default - limits[..., 0]) / (limits[..., 1] - limits[..., 0]) - 1.0

    def step_metrics(self) -> dict[str, Any]:
        """Per-env tensors of one step's diagnostics."""
        import torch

        import isaaclab.utils.math as math_utils

        q = torch_of(self.robot.data.joint_pos)
        target = torch_of(self.robot.data.joint_pos_target)
        pos = torch_of(self.robot.data.body_pos_w)
        quat = torch_of(self.robot.data.body_quat_w)
        residual = torch.zeros(self.env.num_envs, device=self.env.device)
        for p, c, off_p, off_c in self.cut:
            wp_ = pos[:, p] + math_utils.quat_apply(quat[:, p], off_p.float().expand(len(pos), 3))
            wc = pos[:, c] + math_utils.quat_apply(quat[:, c], off_c.float().expand(len(pos), 3))
            residual = torch.maximum(residual, (wp_ - wc).norm(dim=-1))
        closed = self.tables.close_from_servo(q)
        passive = list(self.tables.passive_indices)
        cube_pos = torch_of(self.cube.data.root_pos_w) - self.env.scene.env_origins
        cube_quat = torch_of(self.cube.data.root_quat_w)
        out = {
            "tracking_rad": (q[:, self.actuated] - target[:, self.actuated]).abs().mean(dim=1),
            "loop_residual_mm": 1e3 * residual,
            "passive_sync_rad": (q[:, passive] - closed[:, passive]).abs().max(dim=1).values,
            "cube_pos": cube_pos,
            "cube_quat": cube_quat,
            "orientation_error": torch_of(self.command.metrics["orientation_error"]).clone(),
            "finite": torch.isfinite(q).all(dim=1) & torch.isfinite(cube_pos).all(dim=1),
        }
        if self.cube_sensor is not None:
            matrix = torch_of(self.cube_sensor.data.force_matrix_w)[:, 0]  # (envs, hand bodies, 3)
            normal = matrix.norm(dim=-1)
            out["pad_force_n"] = normal[:, self.pad_columns]
            out["hand_force_on_cube_n"] = normal.sum(dim=1)
            out["nonpad_force_on_cube_n"] = normal.sum(dim=1) - normal[:, self.pad_columns].sum(dim=1)
            points = self.cube_sensor.data.contact_pos_w
            if points is not None:
                out["pad_contact_pos_w"] = torch_of(points)[:, 0][:, self.pad_columns] - self.env.scene.env_origins[
                    :, None
                ]
        if self.hand_sensor is not None and self.cube_sensor is not None:
            net = torch_of(self.hand_sensor.data.net_forces_w)  # (envs, sensor bodies, 3), the hand's own bodies
            names = list(self.hand_sensor.body_names)
            from_cube = torch.zeros_like(net)
            matrix = torch_of(self.cube_sensor.data.force_matrix_w)[:, 0]
            for column, name in enumerate(self.bodies):
                if name in names:
                    from_cube[:, names.index(name)] = -matrix[:, column]
            out["stray_contact_n"] = (net - from_cube).norm(dim=-1).max(dim=1).values
        return out


def timer() -> float:
    return time.perf_counter()


__all__ = [
    "LOG_ROOT",
    "PAD_BODIES",
    "Probe",
    "SENSORLESS_PHYSICS",
    "hand_bodies",
    "TASKS",
    "add_sensors",
    "file_sha256",
    "manifest",
    "run_id",
    "timer",
    "torch_of",
    "versions",
    "write_json",
]
