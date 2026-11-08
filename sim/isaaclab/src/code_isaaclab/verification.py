# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Backend-agnostic rollout metrics for hand tasks, used to compare physics backends (sim-to-sim checks).

The same seeded action sequence (or later a trained policy) is rolled out once per backend, each in its own
process (Newton keeps one solver per process). Each run writes the summary of :class:`RolloutRecorder`, and
:func:`compare` sets two summaries side by side. The metrics read only the scene's ``robot`` and ``object``
entities and the ``object_pose`` command, so they work for any registered hand.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import torch


def _torch(value: Any) -> torch.Tensor:
    """Asset data as a torch tensor (Newton-backed data may be a Warp array)."""
    if isinstance(value, torch.Tensor):
        return value
    import warp as wp

    return wp.to_torch(value)


class RolloutRecorder:
    """Accumulates per-step metrics of one rollout, averaged over environments."""

    def __init__(self, env) -> None:
        self.env = env.unwrapped
        self.robot = self.env.scene["robot"]
        self.object = self.env.scene["object"]
        self.joint_ids = self.robot.find_joints(self.env.cfg.actuated_joint_names)[0]
        self.command = self.env.command_manager.get_term("object_pose")
        # body-space check, independent of how a solver reports joint coordinates
        self.tip_ids = self.robot.find_bodies(self.env.cfg.fingertip_body_names)[0]
        self.tip_start = _torch(self.robot.data.body_pos_w)[:, self.tip_ids].clone()
        self.rows: list[dict[str, float]] = []
        self.resets = 0

    def record(self, dones: torch.Tensor) -> None:
        q = _torch(self.robot.data.joint_pos)[:, self.joint_ids]
        q_target = _torch(self.robot.data.joint_pos_target)[:, self.joint_ids]
        z = _torch(self.object.data.root_pos_w)[:, 2] - self.env.scene.env_origins[:, 2]
        tips = _torch(self.robot.data.body_pos_w)[:, self.tip_ids]
        row = {
            "joint_tracking_error_rad": float((q - q_target).abs().mean()),
            "object_height_m": float(z.mean()),
            "object_height_min_m": float(z.min()),
            "fingertip_displacement_m": float((tips - self.tip_start).norm(dim=-1).mean()),
            "finite": float(torch.isfinite(q).all() and torch.isfinite(z).all()),
        }
        for name, value in getattr(self.command, "metrics", {}).items():
            row[f"command_{name}"] = float(_torch(value).float().mean())
        self.resets += int(_torch(dones).sum())
        self.rows.append(row)

    def summary(self, **meta: Any) -> dict[str, Any]:
        keys = self.rows[0].keys() if self.rows else []
        mean = {k: sum(r[k] for r in self.rows) / len(self.rows) for k in keys}
        return {
            **meta,
            "steps": len(self.rows),
            "num_envs": self.env.num_envs,
            "resets": self.resets,
            "all_finite": all(r["finite"] == 1.0 for r in self.rows),
            "mean": mean,
            "final": self.rows[-1] if self.rows else {},
        }


def compare(paths: list[Path]) -> str:
    """A table of the mean metrics of several rollout summaries (one column per backend)."""
    runs = [json.loads(Path(p).read_text()) for p in paths]
    keys = sorted({k for run in runs for k in run["mean"]})
    header = ["metric"] + [f"{run['policy']} / {run['physics']}" for run in runs]
    lines = [" | ".join(header), " | ".join("---" for _ in header)]
    for meta in ("steps", "resets", "all_finite"):
        lines.append(" | ".join([meta] + [str(run[meta]) for run in runs]))
    for key in keys:
        cells = [f"{run['mean'].get(key, math.nan):.4g}" for run in runs]
        lines.append(" | ".join([key] + cells))
    return "\n".join(lines)


__all__ = ["RolloutRecorder", "compare"]
