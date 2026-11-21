# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""The CoDe hand's goal distribution: rotations about one axis, on top of upstream's reorientation command.

Upstream draws every goal uniformly over SO(3) (:class:`ReorientCommand`). The CoDe hand's three fingers are
parallel and flex in one plane, so most of SO(3) is out of its reach without regrasping. The Study 3 pilot
therefore asks for rotations of the cube about one fixed world axis: each goal is the identity turned by an
angle drawn uniformly from ``[-max_angle, max_angle]``, at least ``min_step`` away from the previous goal.
Everything else (success threshold eps, resample on the first step below it, position command, metrics) is
upstream's.

A goal is reached on any step whose orientation error is below eps, and the next one is drawn at once. Only
``min_step > 2 eps`` guarantees that a new goal is never already reached: the cube, within eps of the goal just
reached, is then more than eps from the next, and has to turn at least ``min_step - 2 eps`` for it. With
``min_step <= 2 eps`` the two goals' success regions overlap (``scripts/study3/goal_audit.py`` measures how
often that is used).

With ``sequence_seed`` set (evaluation), every environment's goals come from a fixed sequence drawn once from
that seed, the same in every run: the k-th goal of environment i does not depend on when the policy reached the
earlier ones, so two policies, or two interfaces, are scored on the same goals one for one.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import torch

import isaaclab.utils.math as math_utils
from isaaclab.utils import configclass

from isaaclab_tasks.core.reorient.mdp.commands import ReorientCommand
from isaaclab_tasks.core.reorient.mdp.commands_cfg import ReorientCommandCfg


def draw_goals(previous: torch.Tensor, max_angle: float, min_step: float, rand) -> torch.Tensor:
    """One goal angle per entry of ``previous``: uniform in ``[-max_angle, max_angle]`` and at least ``min_step``
    from it, by rejection; ``rand(t)`` returns uniform [0, 1) samples shaped like ``t``. The few still rejected
    after 64 draws take the farther end of the range."""
    angle = torch.empty_like(previous)
    pending = torch.ones_like(previous, dtype=torch.bool)
    for _ in range(64):
        angle = torch.where(pending, (2.0 * rand(previous) - 1.0) * max_angle, angle)
        pending = (angle - previous).abs() < min_step
        if not bool(pending.any()):
            break
    return torch.where(pending, -torch.sign(previous + 1e-6) * max_angle, angle)


def goal_sequence(num_envs: int, length: int, max_angle: float, min_step: float, seed: int) -> torch.Tensor:
    """``(num_envs, length)`` goal angles, each row a sequence from 0 by :func:`draw_goals`, from ``seed`` alone
    (CPU generator, so the same on every device)."""
    gen = torch.Generator().manual_seed(int(seed))
    rand = lambda t: torch.rand(t.shape, generator=gen, dtype=t.dtype)  # noqa: E731
    out = torch.empty(num_envs, length)
    previous = torch.zeros(num_envs)
    for k in range(length):
        previous = out[:, k] = draw_goals(previous, max_angle, min_step, rand)
    return out


class AxisReorientCommand(ReorientCommand):
    """Orientation goals about one world axis (see the module docstring)."""

    cfg: AxisReorientCommandCfg

    def __init__(self, cfg: AxisReorientCommandCfg, env):
        # before the parent, whose initial reset may already draw goals
        self._sequence = self._index = None
        self.goal_angle = torch.zeros(env.num_envs, device=env.device)
        if cfg.sequence_seed is not None:
            self._sequence = goal_sequence(
                env.num_envs, cfg.sequence_length, cfg.max_angle, cfg.min_step, cfg.sequence_seed
            ).to(env.device)
            self._index = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        super().__init__(cfg, env)
        self._axis = torch.tensor(cfg.axis, dtype=torch.float, device=self.device)

    def _resample_command(self, env_ids: Sequence[int]):
        self._success.record_goal_reached(env_ids)
        if isinstance(env_ids, slice):
            env_ids = torch.arange(self.num_envs, device=self.device)[env_ids]
        env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        if env_ids.numel() == 0:
            return
        if self._sequence is not None:
            angle = self._sequence[env_ids, self._index[env_ids] % self._sequence.shape[1]]
            self._index[env_ids] += 1
        else:
            angle = draw_goals(self.goal_angle[env_ids], self.cfg.max_angle, self.cfg.min_step, torch.rand_like)
        self.goal_angle[env_ids] = angle
        quat = math_utils.quat_from_angle_axis(angle, self._axis.expand(len(env_ids), 3))
        self.quat_command_w[env_ids] = math_utils.quat_unique(quat) if self.cfg.make_quat_unique else quat

    def reset(self, env_ids=None):
        # a new episode starts from the cube's reset orientation (identity): its first goal is drawn from 0
        ids = slice(None) if env_ids is None else env_ids
        self.goal_angle[ids] = 0.0
        if self._index is not None:
            self._index[ids] = 0
        return super().reset(env_ids)


@configclass
class AxisReorientCommandCfg(ReorientCommandCfg):
    class_type: type = AxisReorientCommand

    axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    """World axis the goals turn the cube about."""
    max_angle: float = math.radians(60.0)
    """Largest goal angle from the cube's reset orientation [rad]."""
    min_step: float = math.radians(30.0)
    """Least angle between consecutive goals [rad]. Goals cannot be reached for free only if it exceeds twice the
    success threshold."""
    sequence_seed: int | None = None
    """Evaluation: draw every environment's goals from a fixed sequence of this seed (None: draw at random)."""
    sequence_length: int = 512
    """Goals per environment in the fixed sequence (it wraps around beyond)."""


__all__ = ["AxisReorientCommand", "AxisReorientCommandCfg", "draw_goals", "goal_sequence"]
