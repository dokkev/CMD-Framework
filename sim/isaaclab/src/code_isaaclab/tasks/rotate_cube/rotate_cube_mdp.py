# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""MDP terms of in-hand cube rotation about world z (:mod:`.env_cfg`). Sources: Hora (Qi et al. 2022,
``haozhiqi/hora``), LEAP Hand's Isaac Lab task, RotateIt (Qi et al. 2023), PenSpin (Wang et al. 2024); terms marked
"project" are this task's own.

- action: :class:`RelativeFlexionAction` (Hora's relative targets, the PIP in flexion);
- reset: :class:`GraspCacheReset` (Hora / LEAP: a grasp drawn from a cache of stable grasps);
- motion: :class:`ObjectMotion` computes the cube's finite-difference motion once per step for the rotation reward and
  the terms that read it (it runs first);
- rewards, terminations, observations and Hora's stage-1 extras (work penalty, random force, privileged physics).
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import torch

import isaaclab.utils.math as math_utils
from isaaclab.managers import ManagerTermBase, SceneEntityCfg
from isaaclab.utils import configclass

from code_isaaclab.flexion_control import FlexionEMAAction, FlexionEMAActionCfg


def _t(value) -> torch.Tensor:
    return value if isinstance(value, torch.Tensor) else value.torch


# --- action -------------------------------------------------------------------------------------------------------


class RelativeFlexionAction(FlexionEMAAction):
    """Relative joint targets (Hora): ``target += step * clip(action, -1, 1)``, clamped to the command limits (PIP in
    flexion). The previous target restarts at the measured pose on reset (``FlexionEMAAction.reset``)."""

    cfg: RelativeFlexionActionCfg

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        command = self._prev_applied_actions + self.cfg.step * actions.clamp(-1.0, 1.0)
        command = torch.clamp(command, self._cmd_limits[..., 0], self._cmd_limits[..., 1])
        self._prev_applied_actions[:] = command
        self._flexion_cmd[:] = command[:, self._pip_cols]
        self._processed_actions[:] = self._to_servo(command)
        if self.cfg.flexion_stiffness is not None or self.cfg.flexion_damping is not None:
            self._schedule_gains()

    def reset(self, env_ids=None) -> None:
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        reset = grasp_term(self._env)
        self._prev_applied_actions[ids] = reset.grasp_target[ids]
        self._flexion_cmd[ids] = reset.grasp_target[ids][:, self._pip_cols]
        if self.cfg.gain_noise > 0.0:
            if not hasattr(self, "_gain_scale"):
                self._gain_scale = torch.ones(self.num_envs, len(self._pip), 2, device=self.device)
            g = self.cfg.gain_noise
            self._gain_scale[ids] = 1.0 + (2.0 * torch.rand_like(self._gain_scale[ids]) - 1.0) * g

    def _schedule_gains(self) -> None:
        theta = self._asset.data.joint_pos.torch[:, self._pip_joint_ids]
        ratio_sq = (
            torch.stack(
                [self._tables.ratio(n, self._tables.flexion(n, theta[:, i])) for i, (_, n) in enumerate(self._pip)],
                dim=1,
            )
            ** 2
        )
        scale = getattr(self, "_gain_scale", None)
        ks = 1.0 if scale is None else scale[..., 0]
        ds = 1.0 if scale is None else scale[..., 1]
        if self.cfg.flexion_stiffness is not None:
            self._asset.write_joint_stiffness_to_sim_index(
                stiffness=(ks * self.cfg.flexion_stiffness / ratio_sq).contiguous(), joint_ids=self._pip_joint_ids
            )
        if self.cfg.flexion_damping is not None:
            self._asset.write_joint_damping_to_sim_index(
                damping=(ds * self.cfg.flexion_damping / ratio_sq).contiguous(), joint_ids=self._pip_joint_ids
            )

    @property
    def targets(self) -> torch.Tensor:
        """The current joint targets (PIP in flexion), ``(num_envs, num_actuated)``."""
        return self._prev_applied_actions


@configclass
class RelativeFlexionActionCfg(FlexionEMAActionCfg):
    class_type: type = RelativeFlexionAction
    step: float = 1.0 / 24.0
    """Largest target change per control step [rad] (Hora's 1/24)."""
    gain_noise: float = 0.0
    """Per-episode scale of the scheduled PIP stiffness and damping, U(1 - g, 1 + g) (Hora's PD-gain randomisation;
    the other servos' gains are randomised by the ``randomize_actuator_gains`` event)."""


# --- grasp cache reset --------------------------------------------------------------------------------------------


class GraspCacheReset(ManagerTermBase):
    """Reset event: a grasp drawn uniformly from ``cache`` (``q`` and optional commanded ``target`` for every joint,
    plus the cube's env-frame position and quaternion), or, with ``cache`` None (generation), the default (cradle) pose
    with ``joint_noise`` rad of uniform noise on every actuated joint (PIP in flexion, the four-bar closed from the
    tables) and the cube ``lift_m`` above
    its rest, turned about world z by a uniform random yaw. Records each grasp (``grasp_q`` actuated, PIP in
    flexion; ``grasp_pos`` env frame) for the rewards and terminations."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        p = cfg.params
        self.robot, self.obj = env.scene[p.get("robot", "robot")], env.scene[p.get("object", "object")]
        n, dev = env.num_envs, env.device
        self.grasp_q = torch.zeros(n, len(p["actuated"]), device=dev)
        self.grasp_target = torch.zeros_like(self.grasp_q)
        self.grasp_cache_index = torch.full((n,), -1, dtype=torch.long, device=dev)
        self.grasp_pos = _t(self.obj.data.default_root_pose)[:, :3].clone()
        self.cache = None
        self._actuated = [self.robot.joint_names.index(j) for j in p["actuated"]]
        if p.get("cache"):
            path = Path(p["cache"])
            if not path.is_file():
                raise FileNotFoundError(
                    f"No grasp cache at {path}. Generate it with scripts/rotate_cube/gen_grasps.py."
                )
            data = torch.load(path, map_location="cpu", weights_only=True)
            if data.get("hand", env.cfg.hand_key) != env.cfg.hand_key:
                raise ValueError(f"Grasp cache {path} is for hand {data['hand']}, expected {env.cfg.hand_key}")
            from .grasp import CUBE_SIZE_M, GRASP_MCP_DEG, GRASP_PIP_DEG

            for key, expected in (
                ("cube_size_m", CUBE_SIZE_M),
                ("grasp_mcp_deg", GRASP_MCP_DEG),
                ("grasp_pip_deg", GRASP_PIP_DEG),
            ):
                if data.get(key, expected) != expected:
                    raise ValueError(f"Grasp cache {path} has {key}={data[key]}, expected {expected}")
            if list(data["joint_names"]) != list(self.robot.joint_names):
                raise ValueError(f"Grasp cache {path} joints differ from the robot's")
            q, pos, quat = (data[k] for k in ("q", "pos", "quat"))
            n_grasps = len(q)
            if (
                n_grasps == 0
                or q.shape != (n_grasps, len(self.robot.joint_names))
                or pos.shape != (n_grasps, 3)
                or quat.shape != (n_grasps, 4)
            ):
                raise ValueError(f"Grasp cache {path} must contain matching nonempty q, pos and quat arrays")
            if not all(torch.isfinite(values).all() for values in (q, pos, quat)):
                raise ValueError(f"Grasp cache {path} contains non-finite states")
            target = data.get("target", q)
            if target.shape != q.shape or not torch.isfinite(target).all():
                raise ValueError(f"Grasp cache {path} target must match q and contain only finite values")
            self.cache = {
                k: values.to(device=dev, dtype=torch.float32)
                for k, values in (("q", q), ("pos", pos), ("quat", quat), ("target", target))
            }
        from code_isaaclab.flexion_control import FlexionTables
        from code_isaaclab.loops import FourBarTables

        self._four_bar = FourBarTables(p["four_bar_tables"], self.robot.joint_names, device=dev)
        self._flex = FlexionTables(p["four_bar_tables"], device=dev)

    def _to_flexion(self, q_act: torch.Tensor) -> torch.Tensor:
        out = q_act.clone()
        for k, name in enumerate(self.cfg.params["actuated"]):
            if name.endswith("_pip_servo"):
                out[:, k] = self._flex.flexion(name, q_act[:, k])
        return out

    def __call__(
        self, env, env_ids, actuated: list, four_bar_tables: dict, joint_noise: float, lift_m: float, cache: str | None
    ):
        if env_ids is None or isinstance(env_ids, slice):
            ids = torch.arange(env.num_envs, device=env.device)[env_ids if isinstance(env_ids, slice) else slice(None)]
        else:
            ids = torch.as_tensor(env_ids, device=env.device, dtype=torch.long)
        k = len(ids)
        origins = env.scene.env_origins[ids]
        if self.cache is not None:
            pick = torch.randint(0, self.cache["q"].shape[0], (k,), device=env.device)
            self.grasp_cache_index[ids] = pick
            q = self.cache["q"][pick]
            target = self.cache["target"][pick][:, self._actuated]
            pose = torch.cat((self.cache["pos"][pick] + origins, self.cache["quat"][pick]), dim=1)
        else:
            self.grasp_cache_index[ids] = -1
            p = self.cfg.params
            q = _t(self.robot.data.default_joint_pos)[ids].clone()
            noise = (2.0 * torch.rand(k, len(self._actuated), device=env.device) - 1.0) * p["joint_noise"]
            lo, hi = (_t(self.robot.data.soft_joint_pos_limits)[ids][:, self._actuated, i] for i in (0, 1))
            for c, j in enumerate(self._actuated):
                name = self.robot.joint_names[j]
                if name.endswith("_pip_servo"):  # perturb the flexion, then close the four-bar
                    flex = (self._flex.flexion(name, q[:, j]) + noise[:, c]).clamp(0.0, 2.094)
                    q[:, j] = self._flex.servo(name, flex)
                else:
                    q[:, j] = (q[:, j] + noise[:, c]).clamp(lo[:, c], hi[:, c])
            q = self._four_bar.close_from_servo(q)
            pose = _t(self.obj.data.default_root_pose)[ids].clone()
            pose[:, :3] += origins
            pose[:, 2] += p["lift_m"]
            yaw = (2.0 * torch.rand(k, device=env.device) - 1.0) * torch.pi
            z = torch.zeros(k, 3, device=env.device)
            z[:, 2] = 1.0
            pose[:, 3:7] = math_utils.quat_mul(math_utils.quat_from_angle_axis(yaw, z), pose[:, 3:7])
            target = q[:, self._actuated]
        self.robot.write_joint_position_to_sim_index(position=q, env_ids=ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=ids)
        self.robot.set_joint_position_target_index(target=q, env_ids=ids)
        self.obj.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)
        self.obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros(k, 6, device=env.device), env_ids=ids)
        self.grasp_q[ids] = self._to_flexion(q[:, self._actuated])
        self.grasp_target[ids] = self._to_flexion(target)
        self.grasp_pos[ids] = pose[:, :3] - origins


def grasp_term(env) -> GraspCacheReset:
    return env.event_manager.get_term_cfg("reset_grasp").func


# --- object motion ------------------------------------------------------------------------------------------------


class ObjectMotion(ManagerTermBase):
    """Reward term: ``clip(omega_z, lo, hi)`` (``params``), the cube's yaw rate about world z from the quaternion finite
    difference over one control step (Hora). Keeps the previous pose, the linear velocity from the position finite
    difference (:func:`object_lin_vel_fd_l1` reads it) and each episode's net yaw, logged on reset as
    ``Metrics/net_yaw_rad`` and ``Metrics/omega_z_mean`` (net yaw over the episode's duration)."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        full_credit = cfg.params.get("full_credit_m", 0.0)
        zero_credit = cfg.params.get("zero_credit_m", 0.0)
        if (full_credit, zero_credit) != (0.0, 0.0) and not 0.0 <= full_credit < zero_credit:
            raise ValueError("Rotation credit distances must satisfy 0 <= full_credit_m < zero_credit_m")
        self.obj = env.scene["object"]
        self.prev_quat = _t(self.obj.data.root_quat_w).clone()
        self.prev_pos = _t(self.obj.data.root_pos_w).clone()
        self.omega_z = torch.zeros(env.num_envs, device=env.device)
        self.omega = torch.zeros(env.num_envs, 3, device=env.device)
        self.lin_vel = torch.zeros(env.num_envs, 3, device=env.device)
        self.net_yaw = torch.zeros(env.num_envs, device=env.device)
        self.steps = torch.zeros(env.num_envs, device=env.device)

    def reset(self, env_ids: Sequence[int] | None = None):
        ids = slice(None) if env_ids is None else env_ids
        log = self._env.extras.setdefault("log", {})
        if self.net_yaw[ids].numel() > 0 and bool((self.steps[ids] > 0).any()):
            log["Metrics/net_yaw_rad"] = float(self.net_yaw[ids].mean())
            secs = (self.steps[ids] * self._env.step_dt).clamp_min(self._env.step_dt)
            log["Metrics/omega_z_mean"] = float((self.net_yaw[ids] / secs).mean())
        self.prev_quat[ids] = _t(self.obj.data.root_quat_w)[ids]
        self.prev_pos[ids] = _t(self.obj.data.root_pos_w)[ids]
        self.net_yaw[ids] = 0.0
        self.steps[ids] = 0.0
        self.omega_z[ids] = 0.0
        self.omega[ids] = 0.0
        self.lin_vel[ids] = 0.0

    def __call__(
        self,
        env,
        clip_min: float,
        clip_max: float,
        full_credit_m: float = 0.0,
        zero_credit_m: float = 0.0,
    ) -> torch.Tensor:
        quat, pos = _t(self.obj.data.root_quat_w), _t(self.obj.data.root_pos_w)
        dq = math_utils.axis_angle_from_quat(math_utils.quat_mul(quat, math_utils.quat_conjugate(self.prev_quat)))
        self.omega = dq / env.step_dt
        self.omega_z = self.omega[:, 2]
        self.lin_vel = (pos - self.prev_pos) / env.step_dt
        self.net_yaw += dq[:, 2]
        self.steps += 1.0
        self.prev_quat[:] = quat
        self.prev_pos[:] = pos
        reward = self.omega_z.clamp(clip_min, clip_max)
        if zero_credit_m > 0.0:
            distance = (pos - env.scene.env_origins - grasp_term(env).grasp_pos).norm(dim=1)
            credit = ((zero_credit_m - distance) / (zero_credit_m - full_credit_m)).clamp(0.0, 1.0)
            reward = torch.where(reward > 0.0, credit * reward, reward)
        return reward


def rotate_over_speed(env, omega_max: float) -> torch.Tensor:
    """``max(omega_z - omega_max, 0)`` (PenSpin's ``rotate_penalty``; it uses ``angvelPenaltyThres`` 1.0 with scale
    -0.3), ``omega_z`` the rotation term's finite-difference yaw rate (it runs first)."""
    return (env.reward_manager.get_term_cfg("rotation").func.omega_z - omega_max).clamp_min(0.0)


def object_grasp_distance(env) -> torch.Tensor:
    """``|p - p_grasp|`` [m], the cube's distance from the episode's grasp position (LEAP Isaac Lab's ``dist_rew``,
    scale -10)."""
    pos = _t(env.scene["object"].data.root_pos_w) - env.scene.env_origins
    return (pos - grasp_term(env).grasp_pos).norm(dim=1)


def off_axis_ang_vel_l1(env) -> torch.Tensor:
    """``|omega x k|_1`` with ``k`` world z, i.e. ``|omega_x| + |omega_y|`` (RotateIt's rotation penalty: turning off
    the rotation axis, here tilting), from the rotation term's finite-difference angular velocity."""
    w = env.reward_manager.get_term_cfg("rotation").func.omega
    return w[:, 0].abs() + w[:, 1].abs()


def object_lin_vel_fd_l1(env) -> torch.Tensor:
    """``|dp/dt|_1`` of the cube over the last control step (computed by the rotation term, which runs first)."""
    return env.reward_manager.get_term_cfg("rotation").func.lin_vel.abs().sum(dim=1)


def servo_torque_l2(env, tau_max: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """``sum (0.5 tau / tau_max)^2`` over the actuated servo joints, ``tau`` the PD law the solver applies with the
    current (scheduled) gains, ``K (theta* - theta) - D theta_dot``, clipped to ``+-tau_max`` (Hora's torque penalty
    with its 0.5 N m clip mapped to the XM335 stall torque)."""
    robot = env.scene[asset_cfg.name]
    ids = asset_cfg.joint_ids
    k, d = _t(robot.data.joint_stiffness)[:, ids], _t(robot.data.joint_damping)[:, ids]
    tau = (
        k * (_t(robot.data.joint_pos_target)[:, ids] - _t(robot.data.joint_pos)[:, ids])
        - d * _t(robot.data.joint_vel)[:, ids]
    )
    return ((0.5 * tau.clamp(-tau_max, tau_max) / tau_max) ** 2).sum(dim=1)


def work_l2(env, tau_max: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Hora's work penalty ``(sum tau~ q_dot)^2`` with ``tau~ = 0.5 tau / tau_max`` (:func:`servo_torque_l2`'s torque)
    and the actuated servo joints' velocities."""
    robot = env.scene[asset_cfg.name]
    ids = asset_cfg.joint_ids
    k, d = _t(robot.data.joint_stiffness)[:, ids], _t(robot.data.joint_damping)[:, ids]
    qd = _t(robot.data.joint_vel)[:, ids]
    tau = k * (_t(robot.data.joint_pos_target)[:, ids] - _t(robot.data.joint_pos)[:, ids]) - d * qd
    return ((0.5 * tau.clamp(-tau_max, tau_max) / tau_max) * qd).sum(dim=1) ** 2


class ObjectRandomForce(ManagerTermBase):
    """Interval event, every control step: Hora's random object force. The force decays by ``decay`` per
    ``decay_interval`` s; with probability ``prob`` per step a new one is drawn, ``N(0, 1) * mass * scale`` per world
    axis, and applied at the cube's centre of mass (world frame) until the next change; zeroed on reset."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.obj = env.scene["object"]
        self.forces = torch.zeros(env.num_envs, 3, device=env.device)

    def reset(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        self.forces[ids] = 0.0
        self._write(torch.arange(self.num_envs, device=self.device)[ids] if isinstance(ids, slice) else ids)

    def _write(self, ids):
        ids = torch.as_tensor(ids, device=self.device, dtype=torch.long)
        if ids.numel() == 0:
            return
        self.obj.permanent_wrench_composer.set_forces_and_torques_index(
            forces=self.forces[ids][:, None],
            torques=torch.zeros_like(self.forces[ids])[:, None],
            body_ids=[0],
            env_ids=ids,
            is_global=True,
        )

    def __call__(self, env, env_ids, scale: float, prob: float, decay: float, decay_interval: float):
        if env_ids is None or isinstance(env_ids, slice):
            ids = torch.arange(env.num_envs, device=env.device)
        else:
            ids = torch.as_tensor(env_ids, device=env.device, dtype=torch.long)
        self.forces[ids] *= decay ** (env.step_dt / decay_interval)
        mass = _t(self.obj.data.body_mass)[ids].reshape(len(ids), -1)[:, :1]
        new = torch.rand(len(ids), device=env.device) < prob
        draw = torch.randn(len(ids), 3, device=env.device) * mass * scale
        self.forces[ids] = torch.where(new[:, None], draw, self.forces[ids])
        self._write(ids)


def object_physics_priv(env) -> torch.Tensor:
    """Hora's privileged physical properties of the cube: mass [kg], friction (its shape's coefficient), centre-of-mass
    offset in its body frame (x 100, cm) and scale (1: the cube's size is not randomised); ``(num_envs, 6)``."""
    obj = env.scene["object"]
    mass = _t(obj.data.body_mass).reshape(env.num_envs, -1)[:, :1]
    model = env.sim.physics_manager.get_model()
    import warp as wp

    mu = wp.to_torch(obj._root_view.get_attribute("shape_material_mu", model)[:, 0]).reshape(env.num_envs, -1)[:, :1]
    com = _t(obj.data.body_com_pos_b).reshape(env.num_envs, -1)[:, :3]
    return torch.cat((mass, mu, 100.0 * com, torch.ones_like(mass)), dim=1)


# --- terminations -------------------------------------------------------------------------------------------------


def object_off_grasp(env, max_distance: float) -> torch.Tensor:
    """The cube more than ``max_distance`` from the episode's grasp position."""
    pos = _t(env.scene["object"].data.root_pos_w) - env.scene.env_origins
    return (pos - grasp_term(env).grasp_pos).norm(dim=1) > max_distance


class ObjectTilted(ManagerTermBase):
    """The cube tilted: its body axis that pointed up at reset now has ``z < min_dot`` (a turn about world z keeps it
    at 1; LEAP's flip test)."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.obj = env.scene["object"]
        self.up_body = torch.zeros(env.num_envs, 3, device=env.device)
        self.up_body[:, 2] = 1.0

    def reset(self, env_ids: Sequence[int] | None = None):
        ids = slice(None) if env_ids is None else env_ids
        z = torch.zeros_like(self.up_body[ids])
        z[:, 2] = 1.0
        self.up_body[ids] = math_utils.quat_apply_inverse(_t(self.obj.data.root_quat_w)[ids], z)

    def up_dot(self) -> torch.Tensor:
        return math_utils.quat_apply(_t(self.obj.data.root_quat_w), self.up_body)[:, 2]

    def __call__(self, env, min_dot: float) -> torch.Tensor:
        return self.up_dot() < min_dot


def tilt_deviation(env, free_tilt_dot: float) -> torch.Tensor:
    """``max(free_tilt_dot - up_z, 0)``: tilt beyond the free angle from the reset pose, as a cosine deficit (project
    term)."""
    tilted = env.termination_manager.get_term_cfg("object_tilted").func
    return (free_tilt_dot - tilted.up_dot()).clamp_min(0.0)


class RotationMilestones(ManagerTermBase):
    """Project term: 1 for each new ``angle_step_rad`` of progress, paid once, while the cube is inside the envelope
    (``max_distance`` from the grasp, up axis ``z >= min_dot``). Progress is the smaller of the projected heading of the
    cube's reset x axis and the integrated world-z rotation (:class:`ObjectMotion`); re-crossing pays nothing. Disabled
    (no state) at weight 0. Also tracks whether the episode left the envelope and the halfway values for
    :func:`retained_turns_at_timeout`."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        if cfg.params["angle_step_rad"] <= 0:
            raise ValueError("angle_step_rad must be positive")
        self.enabled = cfg.weight != 0.0
        if self.enabled:
            self.obj = env.scene["object"]
            self.forward_body = torch.zeros(env.num_envs, 3, device=env.device)
            self.prev_forward = torch.zeros_like(self.forward_body)
            self.heading = torch.zeros(env.num_envs, device=env.device)
            self.credited = torch.zeros_like(self.heading)
            self.progress = torch.zeros_like(self.heading)
            self.halfway_heading = torch.zeros_like(self.heading)
            self.halfway_yaw = torch.zeros_like(self.heading)
            self.within_envelope = torch.ones_like(self.heading, dtype=torch.bool)

    def reset(self, env_ids: Sequence[int] | None = None):
        if not self.enabled:
            return
        ids = slice(None) if env_ids is None else env_ids
        forward = torch.zeros_like(self.forward_body[ids])
        forward[:, 0] = 1.0
        self.forward_body[ids] = math_utils.quat_apply_inverse(_t(self.obj.data.root_quat_w)[ids], forward)
        self.prev_forward[ids] = forward
        self.heading[ids] = 0.0
        self.credited[ids] = 0.0
        self.progress[ids] = 0.0
        self.halfway_heading[ids] = 0.0
        self.halfway_yaw[ids] = 0.0
        self.within_envelope[ids] = True

    def __call__(self, env, angle_step_rad: float, max_distance: float, min_dot: float) -> torch.Tensor:
        forward = math_utils.quat_apply(_t(self.obj.data.root_quat_w), self.forward_body)
        cross_z = self.prev_forward[:, 0] * forward[:, 1] - self.prev_forward[:, 1] * forward[:, 0]
        dot_xy = (self.prev_forward[:, :2] * forward[:, :2]).sum(dim=1)
        self.heading += torch.atan2(cross_z, dot_xy)
        self.prev_forward[:] = forward
        motion = env.reward_manager.get_term_cfg("rotation").func
        halfway = motion.steps == env.max_episode_length // 2
        self.halfway_heading[:] = torch.where(halfway, self.heading, self.halfway_heading)
        self.halfway_yaw[:] = torch.where(halfway, motion.net_yaw, self.halfway_yaw)
        self.progress[:] = torch.minimum(self.heading, motion.net_yaw)
        reached = (self.progress / angle_step_rad).floor()
        tilted = env.termination_manager.get_term_cfg("object_tilted").func
        retained = (object_grasp_distance(env) <= max_distance) & (tilted.up_dot() >= min_dot)
        self.within_envelope &= retained
        earned = torch.where(
            retained & ~env.termination_manager.terminated, (reached - self.credited).clamp_min(0), 0.0
        )
        self.credited += earned
        return earned


def retained_turns_at_timeout(env, second_half_weight: float = 0.0) -> torch.Tensor:
    """Project term: progress in turns at a clean timeout (no termination, never left the milestone envelope), plus
    ``second_half_weight`` x the forward progress made in the episode's second half (capped by the total)."""
    milestones = env.reward_manager.get_term_cfg("milestones").func
    if not milestones.enabled:
        raise ValueError("retained_turns_at_timeout requires the milestones reward to be enabled")
    tm = env.termination_manager
    valid = tm.time_outs & ~tm.terminated & milestones.within_envelope
    progress = milestones.progress.clamp_min(0.0)
    if second_half_weight:
        motion = env.reward_manager.get_term_cfg("rotation").func
        late = torch.minimum(
            milestones.heading - milestones.halfway_heading,
            motion.net_yaw - milestones.halfway_yaw,
        ).clamp_min(0.0)
        late = torch.where(motion.steps > env.max_episode_length // 2, late, 0.0)
        progress = progress + second_half_weight * torch.minimum(late, progress)
    return torch.where(valid, progress / (2.0 * torch.pi), 0.0)


# --- observations -------------------------------------------------------------------------------------------------


def gait_phase(env, period_s: float) -> torch.Tensor:
    """``(sin, cos)`` of the episode clock with period ``period_s``: a timing input for repeating release and regrasp
    (project observation)."""
    angle = env.episode_length_buf * (2.0 * torch.pi * env.step_dt / period_s)
    return torch.stack((angle.sin(), angle.cos()), dim=-1)


def next_rotation_goal(env) -> torch.Tensor:
    """The next uncredited milestone minus the progress, in milestones, from the projected heading and from the
    integrated rotation, clipped to [-2, 4] (project observation)."""
    if not hasattr(env, "reward_manager"):  # observation manager shape inference during startup
        return torch.ones(env.num_envs, 2, device=env.device)
    cfg = env.reward_manager.get_term_cfg("milestones")
    milestones = cfg.func
    if not milestones.enabled:
        raise ValueError("The rotation goal observation requires a nonzero milestones reward weight")
    step = cfg.params["angle_step_rad"]
    goal = milestones.credited + 1.0
    net_yaw = env.reward_manager.get_term_cfg("rotation").func.net_yaw
    return (goal[:, None] - torch.stack((milestones.heading, net_yaw), dim=1) / step).clamp(-2.0, 4.0)


def actuated_targets_normalized(env) -> torch.Tensor:
    """The current joint targets (PIP in flexion), normalised to [-1, 1] by the command limits."""
    term = env.action_manager.get_term("joint_pos")
    lo, hi = term._cmd_limits[..., 0], term._cmd_limits[..., 1]
    return math_utils.scale_transform(term.targets, lo, hi)


def joint_target_limit_cost(
    env,
    lower_joints: Sequence[str] = (),
    upper_joints: Sequence[str] = (),
    free_fraction: float = 0.85,
    min_progress_rad: float = 0.0,
    ramp_progress_rad: float = 0.0,
    max_distance_m: float = 0.0,
    min_up_dot: float = -1.0,
) -> torch.Tensor:
    """Saturated joint targets (project term): the mean over ``lower_joints`` (targets near their lower limit) and
    ``upper_joints`` (near the upper) of ``(max(|t| - free_fraction, 0) / (1 - free_fraction))^2``, ``t`` normalised.

    The progress gate can leave the initial grasp and first rotation unpenalized. ``ramp_progress_rad`` smooths its
    onset after ``min_progress_rad``; progress is the minimum of projected heading and integrated world-z rotation.
    If ``max_distance_m`` is positive, the cost also requires the cube to remain inside that distance and tilt envelope.
    """
    if not 0.0 <= free_fraction < 1.0:
        raise ValueError("free_fraction must lie in [0, 1)")
    if min_progress_rad < 0.0 or ramp_progress_rad < 0.0 or max_distance_m < 0.0:
        raise ValueError("progress gate values and max_distance_m must be nonnegative")
    if not -1.0 <= min_up_dot <= 1.0:
        raise ValueError("min_up_dot must lie in [-1, 1]")
    names = env.action_manager.get_term("joint_pos")._joint_names
    targets = actuated_targets_normalized(env)
    lower_names, upper_names = list(lower_joints), list(upper_joints)
    if not lower_names and not upper_names:
        raise ValueError("at least one lower or upper joint must be configured")
    costs = [(-targets[:, names.index(joint)] - free_fraction).clamp_min(0.0).square() for joint in lower_names]
    costs.extend((targets[:, names.index(joint)] - free_fraction).clamp_min(0.0).square() for joint in upper_names)
    cost = torch.stack(costs, dim=0).mean(dim=0) / (1.0 - free_fraction) ** 2
    if min_progress_rad > 0.0 or ramp_progress_rad > 0.0 or max_distance_m > 0.0:
        milestones = env.reward_manager.get_term_cfg("milestones").func
        if not milestones.enabled:
            raise ValueError("a progress-gated joint limit cost requires the milestones term")
        if ramp_progress_rad > 0.0:
            gate = ((milestones.progress - min_progress_rad) / ramp_progress_rad).clamp(0.0, 1.0)
        else:
            gate = (milestones.progress >= min_progress_rad).to(cost.dtype)
        if max_distance_m > 0.0:
            tilted = env.termination_manager.get_term_cfg("object_tilted").func
            retained = (object_grasp_distance(env) <= max_distance_m) & (tilted.up_dot() >= min_up_dot)
            gate *= retained.to(cost.dtype)
        cost *= gate
    return cost


def _face(env) -> tuple[torch.Tensor, torch.Tensor]:
    """The proximal pad-face frame (``env.cfg.face_origin``, ``face_rot``; :func:`.grasp.face_frame`) as tensors."""
    cache = getattr(env, "_rotate_face_cache", None)
    if cache is None:
        cache = (
            torch.tensor(env.cfg.face_origin, device=env.device, dtype=torch.float32),
            torch.tensor(env.cfg.face_rot, device=env.device, dtype=torch.float32).reshape(3, 3),
        )
        env._rotate_face_cache = cache
    return cache


def object_pos_from_grasp_face(env) -> torch.Tensor:
    """The cube's position relative to its grasp position, in the proximal pad-face frame's axes [m]."""
    pos = _t(env.scene["object"].data.root_pos_w) - env.scene.env_origins
    return (pos - grasp_term(env).grasp_pos) @ _face(env)[1]


def object_ang_vel_w(env) -> torch.Tensor:
    """The cube's angular velocity [rad/s] (world), as the simulator reports it (privileged, Hora)."""
    return _t(env.scene["object"].data.root_ang_vel_w)


__all__ = [
    "GraspCacheReset",
    "ObjectMotion",
    "ObjectRandomForce",
    "ObjectTilted",
    "RelativeFlexionAction",
    "RelativeFlexionActionCfg",
    "RotationMilestones",
    "actuated_targets_normalized",
    "gait_phase",
    "grasp_term",
    "joint_target_limit_cost",
    "next_rotation_goal",
    "object_ang_vel_w",
    "object_grasp_distance",
    "object_lin_vel_fd_l1",
    "object_off_grasp",
    "object_physics_priv",
    "object_pos_from_grasp_face",
    "off_axis_ang_vel_l1",
    "retained_turns_at_timeout",
    "rotate_over_speed",
    "servo_torque_l2",
    "tilt_deviation",
    "work_l2",
]
