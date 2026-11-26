# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Terms of the finger walk (``docs/experimental/rl_pip_only.md``, attempt 7b'): the cylinder's place along the
proximal pad face and the alternating targets, observations, rewards, terminations and the reset."""

from __future__ import annotations

import torch

from isaaclab.managers import ManagerTermBase, SceneEntityCfg

from ..finger_relay.mdp import _finite, action_rate_l2, tip_touch  # noqa: F401
from ..rotate_cube.rotate_cube_mdp import _t
from ..rotate_cube_omnireset.mdp import recover_worlds


class WalkGoal(ManagerTermBase):
    """Reward term returning 0 that keeps the cylinder's state in the face frame (``x`` along the face toward the
    tips, ``z`` its height) and the target: it alternates between ``targets_m`` (the first drawn per episode), and
    advances once ``|x - target| < success_m`` for ``hold_steps`` steps. ``progress`` is this step's reduction of the
    distance to the current target [m] (the net walk toward targets)."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        n, dev = env.num_envs, env.device
        p = cfg.params
        self.obj = env.scene["object"]
        self.origin = torch.tensor(p["face_origin"], device=dev)
        self.rot = torch.tensor(p["face_rot"], device=dev).reshape(3, 3)
        self.targets = torch.tensor(p["targets_m"], device=dev)
        z = torch.zeros(n, device=dev)
        self.x, self.zh, self.y, self.err, self.progress, self.walked = (z.clone() for _ in range(6))
        self.k = torch.zeros(n, dtype=torch.long, device=dev)
        self.hold = torch.zeros(n, dtype=torch.long, device=dev)
        self.goals_reached = torch.zeros(n, dtype=torch.long, device=dev)
        self.steps = torch.zeros(n, dtype=torch.long, device=dev)
        self.advanced = torch.zeros(n, dtype=torch.bool, device=dev)

    def face(self) -> torch.Tensor:
        p = _t(self.obj.data.root_pos_w) - self._env.scene.env_origins - self.origin
        return _finite(p @ self.rot)  # (n, 3): x, y, z in the face frame

    def target(self) -> torch.Tensor:
        return self.targets[self.k % len(self.targets)]

    def reset(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        log = self._env.extras.setdefault("log", {})
        done = self.steps[ids] > 0
        if bool(done.any()):
            log["Metrics/goals_per_episode"] = float(self.goals_reached[ids][done].float().mean())
            log["Metrics/walked_mm"] = float(1e3 * self.walked[ids][done].mean())
        f = self.face()
        self.k[ids] = torch.randint(0, len(self.targets), (len(f[ids]),), device=f.device)
        self.x[ids], self.y[ids], self.zh[ids] = f[ids, 0], f[ids, 1], f[ids, 2]
        self.err[ids] = (self.target() - self.x)[ids]
        self.progress[ids] = 0.0
        self.walked[ids] = 0.0
        self.hold[ids] = 0
        self.goals_reached[ids] = 0
        self.steps[ids] = 0

    def __call__(self, env, face_origin, face_rot, targets_m, success_m: float, hold_steps: int) -> torch.Tensor:
        f = self.face()
        self.x[:], self.y[:], self.zh[:] = f[:, 0], f[:, 1], f[:, 2]
        err = self.target() - self.x
        self.progress[:] = self.err.abs() - err.abs()
        self.walked += self.progress
        self.err[:] = err
        self.steps += 1
        ok = err.abs() < success_m
        self.hold[:] = torch.where(ok, self.hold + 1, 0)
        advance = self.hold >= hold_steps
        self.advanced[:] = advance
        self.k += advance.long()
        self.goals_reached += advance.long()
        self.hold[advance] = 0
        self.err[:] = self.target() - self.x  # the next target's error after an advance
        return torch.zeros(env.num_envs, device=env.device)


def walk_goal(env) -> WalkGoal:
    return env.reward_manager.get_term_cfg("walk_goal").func


def walk_progress(env, clip_m: float) -> torch.Tensor:
    if not hasattr(env, "reward_manager"):
        return torch.zeros(env.num_envs, device=env.device)
    return walk_goal(env).progress.clamp(-clip_m, clip_m)


def goal_bonus(env) -> torch.Tensor:
    return walk_goal(env).advanced.float() / env.step_dt


def walk_obs(env) -> torch.Tensor:
    """The cylinder for the actor (closed loop): its face-frame x and height [cm], x rate [m/s], and the target error
    [cm], ``(num_envs, 4)``."""
    if not hasattr(env, "reward_manager"):
        return torch.zeros(env.num_envs, 4, device=env.device)
    g = walk_goal(env)
    v = _finite(_t(g.obj.data.root_lin_vel_w) @ g.rot)[:, 0]
    return torch.stack((g.x * 100, g.zh * 100, v.clamp(-1, 1), (g.err * 100).clamp(-10, 10)), dim=1)


def walk_direction(env) -> torch.Tensor:
    """The sign of the target error only (+1 toward the tips' target, -1 toward the MCP's), ``(num_envs, 1)``: the
    touch+direction actor's one piece of task information (no position, no magnitude)."""
    if not hasattr(env, "reward_manager"):
        return torch.zeros(env.num_envs, 1, device=env.device)
    return torch.sign(walk_goal(env).err)[:, None]


def walk_priv(env) -> torch.Tensor:
    """The critic's extra state: the cylinder's y [cm] and angular rate about its axis (rad/s over 10)."""
    if not hasattr(env, "reward_manager"):
        return torch.zeros(env.num_envs, 2, device=env.device)
    g = walk_goal(env)
    w = _finite(_t(g.obj.data.root_ang_vel_w) @ g.rot)[:, 1]
    return torch.stack((g.y * 100, (w / 10).clamp(-1, 1)), dim=1)


def cylinder_off(env, x_max_m: float, z_min_m: float, y_max_m: float) -> torch.Tensor:
    """The cylinder off the fingers: beyond the tips, below the face or off the side."""
    if not hasattr(env, "reward_manager"):
        return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    g = walk_goal(env)
    f = g.face()
    return (f[:, 0] > x_max_m) | (f[:, 2] < z_min_m) | (f[:, 1].abs() > y_max_m)


class NonFinite(ManagerTermBase):
    """Termination: a non-finite hand or cylinder state; the reset recovers flagged worlds (``recover_worlds``)."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.last_bad = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    def __call__(self, env) -> torch.Tensor:
        r, o = env.scene["robot"], env.scene["object"]
        states = (_t(r.data.joint_pos), _t(r.data.joint_vel), _t(o.data.root_pose_w), _t(o.data.root_vel_w))
        bad = ~torch.stack([torch.isfinite(x).all(dim=1) for x in states], dim=1).all(dim=1)
        self.last_bad = bad.clone()
        return bad


def reset_hand_and_cylinder(env, env_ids, robot_cfg: SceneEntityCfg, start_poses: dict, face_origin, face_rot,
                            x_range_m, z_m: float):
    """The hand at its default pose with the acting fingers at random start flexions (``start_poses``, as the relay's
    reset), the cylinder at rest on the proximal pads, its face-frame x uniform in ``x_range_m``, height ``z_m``,
    its axis across the fingers."""
    if env_ids is None or isinstance(env_ids, slice):
        ids = torch.arange(env.num_envs, device=env.device)
    else:
        ids = torch.as_tensor(env_ids, device=env.device, dtype=torch.long)
    if hasattr(env, "termination_manager") and "non_finite" in env.termination_manager.active_terms:
        bad = env.termination_manager.get_term_cfg("non_finite").func.last_bad[ids]
        if bad.any():
            recover_worlds(env, ids[bad])
    robot, obj = env.scene[robot_cfg.name], env.scene["object"]
    q = _t(robot.data.default_joint_pos)[ids].clone()
    for spec in start_poses.values():
        cols = [robot.joint_names.index(j) for j in spec["joints"]]
        table = torch.tensor(spec["rad"], device=env.device)
        q[:, cols] = table[torch.randint(0, len(table), (len(ids),), device=env.device)]
    robot.write_joint_position_to_sim_index(position=q, env_ids=ids)
    robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=ids)
    robot.set_joint_position_target_index(target=q, env_ids=ids)
    o = torch.tensor(face_origin, device=env.device)
    rot = torch.tensor(face_rot, device=env.device).reshape(3, 3)
    x = x_range_m[0] + (x_range_m[1] - x_range_m[0]) * torch.rand(len(ids), device=env.device)
    local = torch.stack((x, torch.zeros_like(x), torch.full_like(x, z_m)), dim=1)
    pos = env.scene.env_origins[ids] + o + local @ rot.T
    quat = _t(obj.data.default_root_pose)[ids, 3:7].clone()
    obj.write_root_pose_to_sim_index(root_pose=torch.cat((pos, quat), dim=1), env_ids=ids)
    obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(ids), 6, device=env.device), env_ids=ids)
