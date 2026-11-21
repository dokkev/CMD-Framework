# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""The CoDe hand's goal distribution: rotations about one axis, on top of upstream's reorientation command.

Upstream draws every goal uniformly over SO(3) (:class:`ReorientCommand`). The CoDe hand's three fingers are
parallel and flex in one plane, so most of SO(3) is out of its reach without regrasping. The Study 3 pilot
therefore asks for rotations of the cube about one fixed world axis: each goal is the identity turned by an
angle drawn uniformly from ``[-max_angle, max_angle]``, at least ``min_step`` away from the previous goal (so a
new goal is never already reached). Everything else (success threshold, resample on success, position
command, metrics) is upstream's.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import torch

import isaaclab.utils.math as math_utils
from isaaclab.utils import configclass

from isaaclab_tasks.core.reorient.mdp.commands import ReorientCommand
from isaaclab_tasks.core.reorient.mdp.commands_cfg import ReorientCommandCfg


class AxisReorientCommand(ReorientCommand):
    """Orientation goals about one world axis (see the module docstring)."""

    cfg: AxisReorientCommandCfg

    def __init__(self, cfg: AxisReorientCommandCfg, env):
        super().__init__(cfg, env)
        self.goal_angle = torch.zeros(self.num_envs, device=self.device)
        self._axis = torch.tensor(cfg.axis, dtype=torch.float, device=self.device)

    def _resample_command(self, env_ids: Sequence[int]):
        self._success.record_goal_reached(env_ids)
        if isinstance(env_ids, slice):
            env_ids = torch.arange(self.num_envs, device=self.device)[env_ids]
        env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        if env_ids.numel() == 0:
            return
        previous = self.goal_angle[env_ids]
        angle = torch.empty_like(previous)
        pending = torch.ones_like(previous, dtype=torch.bool)
        for _ in range(64):  # rejection: a goal at least ``min_step`` from the previous one
            draw = (2.0 * torch.rand_like(previous) - 1.0) * self.cfg.max_angle
            angle = torch.where(pending, draw, angle)
            pending = (angle - previous).abs() < self.cfg.min_step
            if not bool(pending.any()):
                break
        # the few still pending after the loop: the farther end of the range
        angle = torch.where(pending, -torch.sign(previous + 1e-6) * self.cfg.max_angle, angle)
        self.goal_angle[env_ids] = angle
        quat = math_utils.quat_from_angle_axis(angle, self._axis.expand(len(env_ids), 3))
        self.quat_command_w[env_ids] = math_utils.quat_unique(quat) if self.cfg.make_quat_unique else quat

    def reset(self, env_ids=None):
        # a new episode starts from the cube's reset orientation (identity): its first goal is drawn from 0
        ids = slice(None) if env_ids is None else env_ids
        self.goal_angle[ids] = 0.0
        return super().reset(env_ids)


@configclass
class AxisReorientCommandCfg(ReorientCommandCfg):
    class_type: type = AxisReorientCommand

    axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    """World axis the goals turn the cube about."""
    max_angle: float = math.radians(60.0)
    """Largest goal angle from the cube's reset orientation [rad]."""
    min_step: float = math.radians(30.0)
    """Least angle between consecutive goals [rad] (above the success threshold)."""


__all__ = ["AxisReorientCommand", "AxisReorientCommandCfg"]
