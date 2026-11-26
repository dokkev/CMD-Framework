# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Terms of the finger-relay dial (``docs/experimental/rl_pip_only.md``): the dial's state and chained goals, the
observations, rewards, the reset and the non-finite guard."""

from __future__ import annotations

import math

import torch

from isaaclab.managers import ManagerTermBase, SceneEntityCfg

from ..rotate_cube.rotate_cube_mdp import _t
from ..rotate_cube_omnireset.mdp import recover_worlds


def _finite(x: torch.Tensor) -> torch.Tensor:
    return torch.nan_to_num(x, nan=0.0, posinf=0.0, neginf=0.0)


class DialGoal(ManagerTermBase):
    """Reward term returning 0 that keeps the dial's state (it runs first): the angle ``theta`` of the spin joint
    since the episode's start, its change this step, and the chained goal: the goal starts ``goal_step`` ahead and
    advances by ``goal_step`` once ``theta`` is within ``success_rad`` of it for ``hold_steps`` steps."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        n, dev = env.num_envs, env.device
        self.dial = env.scene["dial"]
        self.spin = self.dial.find_joints("spin_joint")[0][0]
        z = torch.zeros(n, device=dev)
        self.theta, self.start, self.delta, self.goal = z.clone(), z.clone(), z.clone(), z.clone()
        self.hold = torch.zeros(n, dtype=torch.long, device=dev)
        self.goals_reached = torch.zeros(n, dtype=torch.long, device=dev)
        self.steps = torch.zeros(n, dtype=torch.long, device=dev)
        self.advanced = torch.zeros(n, dtype=torch.bool, device=dev)

    def angle(self) -> torch.Tensor:
        return _finite(_t(self.dial.data.joint_pos)[:, self.spin])

    def reset(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        log = self._env.extras.setdefault("log", {})
        done = self.steps[ids] > 0
        if bool(done.any()):
            log["Metrics/goals_per_episode"] = float(self.goals_reached[ids][done].float().mean())
            log["Metrics/dial_rotation_deg"] = float(torch.rad2deg(self.theta[ids][done]).mean())
        self.start[ids] = self.angle()[ids]
        self.theta[ids] = 0.0
        self.delta[ids] = 0.0
        self.goal[ids] = self.cfg.params["goal_step"]
        self.hold[ids] = 0
        self.goals_reached[ids] = 0
        self.steps[ids] = 0

    def __call__(self, env, goal_step: float, success_rad: float, hold_steps: int) -> torch.Tensor:
        theta = self.angle() - self.start
        self.delta[:] = theta - self.theta
        self.theta[:] = theta
        self.steps += 1
        ok = (self.theta - self.goal).abs() < success_rad
        self.hold[:] = torch.where(ok, self.hold + 1, 0)
        advance = self.hold >= hold_steps
        self.advanced[:] = advance
        self.goal += torch.where(advance, goal_step, 0.0)
        self.goals_reached += advance.long()
        self.hold[advance] = 0
        return torch.zeros(env.num_envs, device=env.device)


def dial_goal(env) -> DialGoal:
    return env.reward_manager.get_term_cfg("dial_goal").func


def dial_progress(env, clip_rad: float) -> torch.Tensor:
    """The dial's turn this step toward the goal direction [rad], clipped."""
    if not hasattr(env, "reward_manager"):
        return torch.zeros(env.num_envs, device=env.device)
    return dial_goal(env).delta.clamp(-clip_rad, clip_rad)


def goal_bonus(env) -> torch.Tensor:
    """``1 / step_dt`` on the step a goal is reached (the reward manager multiplies by the step duration)."""
    return dial_goal(env).advanced.float() / env.step_dt


def action_rate_l2(env) -> torch.Tensor:
    return _finite(torch.sum(torch.square(env.action_manager.action - env.action_manager.prev_action), dim=1))


def dial_obs(env) -> torch.Tensor:
    """The dial for the actor (closed loop): sin and cos of its angle since the start, the goal error clipped to +-pi
    (over pi), and its rate (rad/s over 10), ``(num_envs, 4)``."""
    if not hasattr(env, "reward_manager"):
        return torch.zeros(env.num_envs, 4, device=env.device)
    g = dial_goal(env)
    w = _finite(_t(g.dial.data.joint_vel)[:, g.spin])
    err = (g.goal - g.theta).clamp(-math.pi, math.pi) / math.pi
    return torch.stack((torch.sin(g.theta), torch.cos(g.theta), err, (w / 10.0).clamp(-1, 1)), dim=1)


def dial_priv(env) -> torch.Tensor:
    """The critic's extra dial state: the slider's travel [cm] and the goal error [rad, clipped to +-pi]."""
    if not hasattr(env, "reward_manager"):
        return torch.zeros(env.num_envs, 2, device=env.device)
    g = dial_goal(env)
    slide = _finite(_t(g.dial.data.joint_pos)[:, g.dial.find_joints("slide_joint")[0][0]])
    return torch.stack((slide * 100.0, (g.goal - g.theta).clamp(-math.pi, math.pi)), dim=1)


class DelayedNoisy:
    """A delayed, noisy reading of per-environment values (attempt 8, ``docs/experimental/rl_pip_only.md``): each
    episode draws a delay of ``delay_steps`` (uniform integer, ends included; 3-6 control steps = 100-200 ms at 30 Hz);
    a reading is the true value that many steps ago (the episode's first value until then) plus uniform noise in
    +-``amp`` per component, drawn every step. ``rate`` is the reading's change over one step divided by ``dt``."""

    def __init__(self, env, dim: int, amp, delay_steps):
        n, dev = env.num_envs, env.device
        self.env, self.lo, self.hi = env, int(delay_steps[0]), int(delay_steps[1])
        self.amp = torch.as_tensor(amp, device=dev, dtype=torch.float32)
        self.buf = torch.zeros(self.hi + 1, n, dim, device=dev)
        self.delay = torch.randint(self.lo, self.hi + 1, (n,), device=dev)
        self.head = 0
        self.last = None
        self.prev = torch.zeros(n, dim, device=dev)

    def read(self, value: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        env = self.env
        self.head = (self.head + 1) % len(self.buf)
        self.buf[self.head] = value
        steps = getattr(env, "episode_length_buf", None)
        new = torch.ones(env.num_envs, dtype=torch.bool, device=env.device) if (steps is None or self.last is None) \
            else steps == 0
        if new.any():
            self.buf[:, new] = value[new]
            self.delay[new] = torch.randint(self.lo, self.hi + 1, (int(new.sum()),), device=env.device)
        idx = (self.head - self.delay) % len(self.buf)
        out = self.buf[idx, torch.arange(env.num_envs, device=env.device)]
        out = out + (2 * torch.rand_like(out) - 1) * self.amp
        prev = torch.where(new[:, None], out, self.prev)
        rate = (out - prev) / env.step_dt
        self.prev, self.last = out.clone(), out
        return out, rate


def _sensor(env, key: str, dim: int, amp, delay_steps) -> DelayedNoisy:
    s = getattr(env, key, None)
    if s is None or s.env is not env:
        s = DelayedNoisy(env, dim, amp, delay_steps)
        setattr(env, key, s)
    return s


def dial_obs_noisy(env, angle_amp_rad: float, delay_steps) -> torch.Tensor:
    """:func:`dial_obs` from a delayed, noisy dial angle (attempt 8): sin, cos, the goal error from the reading, and the
    reading's rate."""
    if not hasattr(env, "reward_manager"):
        return torch.zeros(env.num_envs, 4, device=env.device)
    g = dial_goal(env)
    th, w = _sensor(env, "_code_dial_sensor", 1, [angle_amp_rad], delay_steps).read(g.theta[:, None])
    th, w = th[:, 0], w[:, 0]
    err = (g.goal - th).clamp(-math.pi, math.pi) / math.pi
    return torch.stack((torch.sin(th), torch.cos(th), err, (w / 10.0).clamp(-1, 1)), dim=1)


def tip_touch(env, max_force_n: float, threshold_n: float, force_noise: float, flip_prob: float) -> torch.Tensor:
    """Per-pad contact (as ``rotate_cube_omnireset.mdp.pad_touch``) from the ``tip_touch`` sensor: per filtered link
    the force magnitude with a uniform gain error, clipped and normalised, then a touch flag flipped with
    ``flip_prob``."""
    f = _t(env.scene.sensors["tip_touch"].data.force_matrix_w).reshape(env.num_envs, -1, 3).norm(dim=-1)
    gain = 1.0 + force_noise * (2 * torch.rand_like(f) - 1)
    force = (f * gain).clamp(0.0, max_force_n) / max_force_n
    flag = (f > threshold_n) ^ (torch.rand_like(f) < flip_prob)
    return _finite(torch.cat((force, flag.float()), dim=1))


class NonFinite(ManagerTermBase):
    """Termination: a non-finite hand or dial joint state; the reset recovers flagged worlds (``recover_worlds``)."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.last_bad = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        self.events = 0

    def __call__(self, env) -> torch.Tensor:
        states = [_t(env.scene[a].data.joint_pos) for a in ("robot", "dial")]
        states += [_t(env.scene[a].data.joint_vel) for a in ("robot", "dial")]
        bad = ~torch.stack([torch.isfinite(x).all(dim=1) for x in states], dim=1).all(dim=1)
        self.events += int(bad.sum())
        env.extras.setdefault("log", {})["Metrics/non_finite_events"] = float(self.events)
        self.last_bad = bad.clone()
        return bad


def reset_hand_and_dial(env, env_ids, robot_cfg: SceneEntityCfg, dial_cfg: SceneEntityCfg, start_poses: dict):
    """The hand at its default pose with its default targets, except the acting fingers at a random start flexion from
    ``start_poses`` (``{finger: {"joints": [...], "rad": [[...] per grid flexion]}}``); the dial at rest (slider 0,
    angle 0)."""
    if env_ids is None or isinstance(env_ids, slice):
        ids = torch.arange(env.num_envs, device=env.device)
    else:
        ids = torch.as_tensor(env_ids, device=env.device, dtype=torch.long)
    if hasattr(env, "termination_manager") and "non_finite" in env.termination_manager.active_terms:
        bad = env.termination_manager.get_term_cfg("non_finite").func.last_bad[ids]
        if bad.any():
            recover_worlds(env, ids[bad])
    robot, dial = env.scene[robot_cfg.name], env.scene[dial_cfg.name]
    q = _t(robot.data.default_joint_pos)[ids].clone()
    for spec in start_poses.values():
        cols = [robot.joint_names.index(j) for j in spec["joints"]]
        table = torch.tensor(spec["rad"], device=env.device)
        q[:, cols] = table[torch.randint(0, len(table), (len(ids),), device=env.device)]
    robot.write_joint_position_to_sim_index(position=q, env_ids=ids)
    robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=ids)
    robot.set_joint_position_target_index(target=q, env_ids=ids)
    d = _t(dial.data.default_joint_pos)[ids].clone()
    dial.write_joint_position_to_sim_index(position=d, env_ids=ids)
    dial.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(d), env_ids=ids)
    dial.set_joint_position_target_index(target=torch.zeros_like(d), env_ids=ids)
