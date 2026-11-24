# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""MDP terms of in-hand cylinder rotation: the cylinder turned continuously about its own axis (Hora-style).

- :class:`DefaultOffsetFlexionAction`: :class:`~code_isaaclab.flexion_control.FlexionEMAAction` with the action
  centred on the hand's default (cradle) pose instead of the middle of the joint limits, so a zero action holds
  the cylinder; the PIP entries are commanded in flexion.
- The cylinder's axis and its spin about that axis, its position relative to its nominal resting position, the
  rewards built on them, and the drop termination.

The cylinder's axis is its body ``y`` (``CylinderCfg(axis="Y")``), signed so that it points along world ``+y`` (the
cylinder is symmetric end to end, so the spin about the axis is measured the same whichever end is which).
"""

from __future__ import annotations

from collections.abc import Sequence

import torch

import isaaclab.utils.math as math_utils
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from code_isaaclab.flexion_control import FlexionEMAAction, FlexionEMAActionCfg

_OBJECT = SceneEntityCfg("object")


def _t(value) -> torch.Tensor:
    return value if isinstance(value, torch.Tensor) else value.torch


# --- action -------------------------------------------------------------------------------------------------------


class DefaultOffsetFlexionAction(FlexionEMAAction):
    """Joint targets ``default + range * action`` (PIP in flexion), smoothed by an EMA and clamped to the limits.

    ``range`` is :attr:`DefaultOffsetFlexionActionCfg.action_range` times each joint's half range (the flexion range
    for the PIP servos), so an action of zero commands the default joint pose."""

    cfg: DefaultOffsetFlexionActionCfg

    def __init__(self, cfg: DefaultOffsetFlexionActionCfg, env):
        super().__init__(cfg, env)
        default = _t(self._asset.data.default_joint_pos)[:, self._joint_ids].clone()
        self._center = self._to_flexion(default).clamp(self._cmd_limits[..., 0], self._cmd_limits[..., 1])
        self._half_range = cfg.action_range * 0.5 * (self._cmd_limits[..., 1] - self._cmd_limits[..., 0])

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        command = self._center + self._half_range * actions.clamp(-1.0, 1.0)
        command = self._alpha * command + (1.0 - self._alpha) * self._prev_applied_actions
        command = torch.clamp(command, self._cmd_limits[..., 0], self._cmd_limits[..., 1])
        self._prev_applied_actions[:] = command
        self._flexion_cmd[:] = command[:, self._pip_cols]
        self._processed_actions[:] = self._to_servo(command)
        if self.cfg.flexion_stiffness is not None or self.cfg.flexion_damping is not None:
            self._schedule_gains()


@configclass
class DefaultOffsetFlexionActionCfg(FlexionEMAActionCfg):
    class_type: type = DefaultOffsetFlexionAction
    action_range: float = 0.6
    """Reach of a unit action from the default pose, as a fraction of each joint's half range."""


# --- object state -------------------------------------------------------------------------------------------------


def cylinder_axis_w(env, asset_cfg: SceneEntityCfg = _OBJECT) -> torch.Tensor:
    """The cylinder's axis (body ``y``) in the world, signed to point along world ``+y``; ``(num_envs, 3)``."""
    quat = _t(env.scene[asset_cfg.name].data.root_quat_w)
    y = torch.zeros(quat.shape[0], 3, device=quat.device)
    y[:, 1] = 1.0
    axis = math_utils.quat_apply(quat, y)
    sign = torch.where(axis[:, 1:2] < 0.0, -1.0, 1.0)
    return axis * sign


def axis_spin(env, asset_cfg: SceneEntityCfg = _OBJECT) -> torch.Tensor:
    """The cylinder's angular velocity about its own (signed) axis [rad/s]; ``(num_envs, 1)``."""
    omega = _t(env.scene[asset_cfg.name].data.root_ang_vel_w)
    return (omega * cylinder_axis_w(env, asset_cfg)).sum(dim=1, keepdim=True)


def nominal_pos_w(env, asset_cfg: SceneEntityCfg = _OBJECT) -> torch.Tensor:
    """The cylinder's nominal resting position (its configured initial position) in the world; ``(num_envs, 3)``."""
    asset = env.scene[asset_cfg.name]
    return _t(asset.data.default_root_pose)[:, :3] + env.scene.env_origins


def object_pos_from_nominal(env, asset_cfg: SceneEntityCfg = _OBJECT) -> torch.Tensor:
    """The cylinder's position relative to its nominal resting position [m]; ``(num_envs, 3)``."""
    return _t(env.scene[asset_cfg.name].data.root_pos_w) - nominal_pos_w(env, asset_cfg)


# --- the proximal pad-face frame (transfer-ready terms) -----------------------------------------------------------
# The env cfg carries ``face_origin`` (3, env frame), ``face_rot`` (9, row-major, columns = the frame's axes in the
# world), ``rest_face`` (3, the cylinder's rest place in the frame) and ``distal_face_points`` (per finger, a point of the
# distal pad face in the distal body's frame); see ``env_cfg.face_frame``.


def _face(env) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    cache = getattr(env, "_rotate_face_cache", None)
    if cache is None:
        cfg = env.cfg
        cache = (torch.tensor(cfg.face_origin, device=env.device, dtype=torch.float32),
                 torch.tensor(cfg.face_rot, device=env.device, dtype=torch.float32).reshape(3, 3),
                 torch.tensor(cfg.rest_face, device=env.device, dtype=torch.float32))
        env._rotate_face_cache = cache
    return cache


def to_face(env, points_w: torch.Tensor) -> torch.Tensor:
    """World points ``(num_envs, ..., 3)`` in each environment's pad-face frame."""
    origin, rot, _ = _face(env)
    shape = (-1,) + (1,) * (points_w.dim() - 2) + (3,)
    return (points_w - env.scene.env_origins.reshape(shape) - origin) @ rot  # row vector times R = R^T p


def object_pos_face(env, asset_cfg: SceneEntityCfg = _OBJECT) -> torch.Tensor:
    """The cylinder's centre in the pad-face frame [m]; ``(num_envs, 3)``."""
    return to_face(env, _t(env.scene[asset_cfg.name].data.root_pos_w))


def object_rest_distance(env, asset_cfg: SceneEntityCfg = _OBJECT) -> torch.Tensor:
    """Distance of the cylinder from its rest place, measured in the pad-face frame [m]."""
    return (object_pos_face(env, asset_cfg) - _face(env)[2]).norm(dim=1)


def object_dropped_face(env, max_distance: float, asset_cfg: SceneEntityCfg = _OBJECT) -> torch.Tensor:
    """The cylinder more than ``max_distance`` from its rest place (pad-face frame)."""
    return object_rest_distance(env, asset_cfg) > max_distance


def distal_pad_points_face(env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Each finger's distal pad-face point (``cfg.distal_face_points``, bodies ``asset_cfg.body_names`` in the same order)
    in the pad-face frame [m]; ``(num_envs, 3 * fingers)``."""
    asset = env.scene[asset_cfg.name]
    local = getattr(env, "_rotate_distal_points", None)
    if local is None:
        local = env._rotate_distal_points = torch.tensor(env.cfg.distal_face_points, device=env.device)
    pos = _t(asset.data.body_link_pos_w)[:, asset_cfg.body_ids]
    quat = _t(asset.data.body_link_quat_w)[:, asset_cfg.body_ids]
    points = pos + math_utils.quat_apply(quat, local.expand(pos.shape[0], -1, -1))
    return to_face(env, points).reshape(env.num_envs, -1)


# --- rewards ------------------------------------------------------------------------------------------------------


def rotation_reward(env, direction: float, clip_min: float, clip_max: float,
                    asset_cfg: SceneEntityCfg = _OBJECT) -> torch.Tensor:
    """``clip(direction * spin, clip_min, clip_max)`` [rad/s]: spin in the commanded direction, clipped."""
    return (direction * axis_spin(env, asset_cfg)[:, 0]).clamp(clip_min, clip_max)


def object_lin_vel_l1(env, asset_cfg: SceneEntityCfg = _OBJECT) -> torch.Tensor:
    """L1 norm of the cylinder's linear velocity [m/s]."""
    return _t(env.scene[asset_cfg.name].data.root_lin_vel_w).abs().sum(dim=1)


def object_pos_deviation(env, asset_cfg: SceneEntityCfg = _OBJECT) -> torch.Tensor:
    """Distance of the cylinder from its nominal resting position [m]."""
    return object_pos_from_nominal(env, asset_cfg).norm(dim=1)


def off_axis_ang_vel(env, asset_cfg: SceneEntityCfg = _OBJECT) -> torch.Tensor:
    """Norm of the cylinder's angular velocity perpendicular to its own axis [rad/s] (wobble, tumbling)."""
    omega = _t(env.scene[asset_cfg.name].data.root_ang_vel_w)
    axis = cylinder_axis_w(env, asset_cfg)
    return (omega - (omega * axis).sum(dim=1, keepdim=True) * axis).norm(dim=1)


def axis_tilt(env, asset_cfg: SceneEntityCfg = _OBJECT) -> torch.Tensor:
    """``1 - |a . y|``: 0 while the cylinder's axis stays along world ``y`` (across the fingers)."""
    return 1.0 - cylinder_axis_w(env, asset_cfg)[:, 1]


# --- terminations -------------------------------------------------------------------------------------------------


def object_dropped(env, max_distance: float, asset_cfg: SceneEntityCfg = _OBJECT) -> torch.Tensor:
    """The cylinder more than ``max_distance`` from its nominal resting position (fallen or pushed out of the hand)."""
    return object_pos_deviation(env, asset_cfg) > max_distance


def reset_object_pose(env, env_ids: Sequence[int] | torch.Tensor, position_noise_m: float,
                      asset_cfg: SceneEntityCfg = _OBJECT) -> None:
    """The cylinder at its nominal pose, ``position_noise_m`` of uniform noise in ``x`` and ``y``, turned about its
    own axis by a uniform random angle (it is symmetric about the axis), at rest."""
    asset = env.scene[asset_cfg.name]
    if env_ids is None or isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids if isinstance(env_ids, slice) else slice(None)]
    ids = torch.as_tensor(env_ids, device=env.device, dtype=torch.long)
    pose = _t(asset.data.default_root_pose)[ids].clone()
    pose[:, :3] += env.scene.env_origins[ids]
    noise = (2.0 * torch.rand(len(ids), 2, device=env.device) - 1.0) * position_noise_m
    pose[:, 0:2] += noise
    angle = (2.0 * torch.rand(len(ids), device=env.device) - 1.0) * torch.pi
    y = torch.zeros(len(ids), 3, device=env.device)
    y[:, 1] = 1.0
    pose[:, 3:7] = math_utils.quat_mul(pose[:, 3:7], math_utils.quat_from_angle_axis(angle, y))
    asset.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)
    asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(ids), 6, device=env.device), env_ids=ids)


__all__ = [
    "DefaultOffsetFlexionAction",
    "DefaultOffsetFlexionActionCfg",
    "axis_spin",
    "axis_tilt",
    "cylinder_axis_w",
    "nominal_pos_w",
    "distal_pad_points_face",
    "object_dropped",
    "object_dropped_face",
    "object_lin_vel_l1",
    "object_pos_deviation",
    "object_pos_face",
    "object_pos_from_nominal",
    "object_rest_distance",
    "off_axis_ang_vel",
    "reset_object_pose",
    "rotation_reward",
    "to_face",
]
