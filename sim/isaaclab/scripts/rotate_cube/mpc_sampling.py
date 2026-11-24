# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Sampling-based MPC (MPPI / predictive sampling) for cube rotation on the RL task's simulator: a go/no-go pilot.

Run from sim/isaaclab::

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run python scripts/rotate_cube/mpc_sampling.py --verify --num_samples 8
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run python scripts/rotate_cube/mpc_sampling.py --num_samples 64 --grasps 0 1 2 3 4

The model is the RL task's (``CoDe-Rotate-Cube-O``: hand USD, servo actuators, PIP flexion gains, loops, MuJoCo-Warp
settings, ``physics=newton_mjwarp``) in ``N + 1`` worlds stacked on one origin (``env_spacing`` 0, so a state copied
between worlds is the same state). World 0 is the system; at each 30 Hz control step:

1. every world is restored to world 0's state (Newton ``State``/``Control`` arrays, every per-world ``mjw_data`` array,
   which holds MuJoCo's warm start and the qpos/qvel kept between the solver's sparse syncs, ``update_data_interval``
   2, and the action term's target buffers);
2. ``N`` action sequences (the RL action space: ``target += a / 24``, ``a`` in [-1, 1] per actuated joint) are drawn
   around the previous plan, shifted by one step, as Gaussian noise on ``--knots`` knots linearly interpolated over the
   horizon; sample 0 is the shifted plan itself (world 0 runs it);
3. all are rolled out ``--horizon`` control steps and scored by the RL reward terms (``env_cfg.RewardsCfg``):
   ``2.5 clip(omega_z, +-0.6)`` (faded from 20 to 50 mm of displacement), ``-0.3 |v|_1``, ``-0.1`` torque,
   ``-0.15 (|omega_x| + |omega_y|)``, ``-4 |p - p_grasp|``, ``-0.3 max(omega_z - 0.8, 0)``, ``-1`` tilt deficit, plus
   ``--drop_cost`` per step outside the 40 mm / 60 deg envelope and ``--action_cost |a|^2``;
4. the plan becomes the MPPI-weighted mean of the samples (``--temperature``, relative to the cost spread; 0: the best
   sample, predictive sampling); every world is restored again and steps the plan's first action, and world 0's new
   state is the next snapshot.

``--verify`` checks the state copy first: worlds copied from one state and given the same actions stay identical, and a
restored world replays its earlier trajectory bit for bit while the other worlds run different actions.

Episodes last ``--episode_s`` (20) s from held-out grasps (``logs/rotate_cube/heldout_crown1_final4``) and are scored as
``eval_policy.py`` scores them (integrated world-z rotation, heading of the reset x axis, displacement <= 40 mm and tilt
<= 60 deg). Results go to ``logs/rotate_cube/mpc/<stamp>_N<N>_H<H>.json`` (and world 0's trajectories to ``.npz``).
``--video`` records world 0 with the Newton GL viewer, offscreen (run without ``DISPLAY``/``WAYLAND_DISPLAY``).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

SIM_ROOT = Path(__file__).resolve().parents[2]
HELDOUT = SIM_ROOT / "logs" / "rotate_cube" / "heldout_crown1_final4"
os.environ.setdefault("CODE_ROTATE_CUBE_GRASPS", str(HELDOUT))  # read when the task config is built
if "--video" in sys.argv:  # offscreen EGL: pyglet must not see a display before newton.viewer is imported
    for _var in ("DISPLAY", "WAYLAND_DISPLAY"):
        os.environ.pop(_var, None)
    import pyglet

    pyglet.options["headless"] = True

import numpy as np  # noqa: E402
import torch  # noqa: E402

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

from isaaclab_tasks.utils import resolve_task_config  # noqa: E402

import code_isaaclab.tasks  # noqa: E402, F401

ENVELOPE_M, ENVELOPE_TILT_DEG = 0.04, 60.0
REWARD = {
    "rotation": 2.5, "rotation_clip": 0.6, "full_credit_m": 0.02, "zero_credit_m": 0.05, "lin_vel": -0.3,
    "torque": -0.1, "off_axis": -0.15, "distance": -4.0, "over_speed": -0.3, "omega_max": 0.8, "tilt": -1.0,
    "free_tilt_dot": 0.94,
}  # fmt: skip
"""The RL reward terms (``env_cfg.RewardsCfg``, s15) that MPC scores; the episodic ones (milestones, timeout, failure,
target-limit regrasp) are replaced by the drop cost."""


class WorldCopier:
    """Snapshot of world 0 and its broadcast to every world: Newton state and control (per-world blocks of the
    flattened arrays), every ``mjw_data`` (and ``efc``) array with a leading world axis, and the action term's buffers."""

    def __init__(self, env):
        import warp as wp

        self.env = env
        mgr = env.sim.physics_manager
        self.mgr = mgr
        model = mgr.get_model()
        self.W = model.world_count
        if self.W != env.num_envs:
            raise RuntimeError(f"{self.W} Newton worlds for {env.num_envs} environments")
        self.solver = mgr._solver
        self.wp = wp
        self.term = env.action_manager.get_term("joint_pos")

    def _arrays(self):
        """(name, torch view of shape (W, -1)) of everything per world; gathered again when the backend has swapped
        its state buffers."""
        backend = self.mgr.backend
        key = (id(backend.state_0), id(backend.state_1), id(backend.control), id(self.solver.mjw_data))
        if getattr(self, "_key", None) != key:
            self._cache, self._key = self._gather(), key
        return self._cache

    def _gather(self):
        wp, W = self.wp, self.W
        out = []
        backend = self.mgr.backend
        for label, obj in (("state_0", backend.state_0), ("state_1", backend.state_1), ("control", backend.control)):
            if obj is None:
                continue
            for name, value in vars(obj).items():
                if isinstance(value, wp.array) and value.size > 0 and value.shape[0] % W == 0:
                    out.append((f"{label}.{name}", wp.to_torch(value).reshape(W, -1)))
        d = self.solver.mjw_data
        for label, obj in (("mjw_data", d), ("mjw_data.efc", d.efc)):
            for name in dir(obj):
                if name.startswith("_"):
                    continue
                value = getattr(obj, name)
                if isinstance(value, wp.array) and value.ndim >= 1 and value.shape[0] == W and value.size > 0:
                    out.append((f"{label}.{name}", wp.to_torch(value).reshape(W, -1)))
        t = self.term
        for name in ("_raw_actions", "_processed_actions", "_prev_applied_actions", "_flexion_cmd"):
            out.append((f"action.{name}", getattr(t, name)))
        return out

    def joint_q(self):
        return self.wp.to_torch(self.mgr.backend.state_0.joint_q).reshape(self.W, -1)

    def snapshot(self, world: int = 0) -> dict:
        return {name: view[world].clone() for name, view in self._arrays()}

    def restore(self, snap: dict) -> None:
        for name, view in self._arrays():
            view[:] = snap[name]

    def broadcast(self, world: int = 0) -> None:
        self.restore(self.snapshot(world))

    def max_spread(self) -> tuple[float, str]:
        """Largest |world k - world 0| over the float arrays (0 when every world holds the same state)."""
        worst, where = 0.0, ""
        for name, view in self._arrays():
            if not view.is_floating_point() or name.startswith("mjw_data.efc") or "contact" in name:
                continue
            diff = float((view - view[:1]).abs().nan_to_num(0.0).max()) if view.numel() else 0.0
            if diff > worst:
                worst, where = diff, name
        return worst, where


class Plant:
    """The RL environment driven below the manager loop: actions in, control steps out, the cube's state read back."""

    def __init__(self, env):
        from code_isaaclab.tasks.rotate_cube.rotate_cube_mdp import _t

        self.env, self._t = env, _t
        self.robot, self.obj = env.scene["robot"], env.scene["object"]
        self.term = env.action_manager.get_term("joint_pos")
        self.act_ids = [self.robot.joint_names.index(n) for n in self.term._joint_names]
        self.steps_per_call = env.cfg.decimation if env._physics_handles_decimation else 1

    @property
    def action_dim(self) -> int:
        return self.term.action_dim

    def step(self, actions: torch.Tensor) -> None:
        env = self.env
        env.action_manager.process_action(actions)
        for _ in range(env.cfg.decimation // self.steps_per_call):
            env._sim_step_counter += self.steps_per_call
            env.action_manager.apply_action()
            env.scene.write_data_to_sim()
            env.sim.step(render=False)
            env.scene.update(dt=env.physics_dt * self.steps_per_call)

    def cube(self):
        t = self._t
        return t(self.obj.data.root_pos_w).clone(), t(self.obj.data.root_quat_w).clone()

    def torque_l2(self, tau_max: float = 1.03) -> torch.Tensor:
        t, r, ids = self._t, self.robot, self.act_ids
        k, d = t(r.data.joint_stiffness)[:, ids], t(r.data.joint_damping)[:, ids]
        tau = k * (t(r.data.joint_pos_target)[:, ids] - t(r.data.joint_pos)[:, ids]) - d * t(r.data.joint_vel)[:, ids]
        return ((0.5 * tau.clamp(-tau_max, tau_max) / tau_max) ** 2).sum(dim=1)

    def set_grasp(self, cache: dict, index: int) -> dict:
        """Every world at held-out grasp ``index`` (as ``GraspCacheReset`` writes it), the action targets at its
        commanded targets; returns the grasp's cube position (env frame) and its up and x axes in the body frame."""
        from isaaclab.utils import math as math_utils

        from code_isaaclab.tasks.rotate_cube.rotate_cube_mdp import grasp_term

        env, n = self.env, self.env.num_envs
        ids = torch.arange(n, device=env.device)
        g = grasp_term(env)
        q = cache["q"][index].to(env.device).expand(n, -1).contiguous()
        target = cache["target"][index].to(env.device).expand(n, -1).contiguous()
        pose = torch.cat((cache["pos"][index], cache["quat"][index])).to(env.device).expand(n, -1).clone()
        pose[:, :3] += env.scene.env_origins
        self.robot.write_joint_position_to_sim_index(position=q, env_ids=ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=ids)
        self.robot.set_joint_position_target_index(target=q, env_ids=ids)
        self.obj.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)
        self.obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=env.device), env_ids=ids)
        g.grasp_q[:] = g._to_flexion(q[:, g._actuated])
        g.grasp_target[:] = g._to_flexion(target[:, g._actuated])
        g.grasp_pos[:] = pose[:, :3] - env.scene.env_origins
        env.sim.physics_manager._reset_solver_internals(None)
        self.term.reset()  # targets restart at the grasp's commanded targets
        env.sim.forward()
        env.scene.update(dt=0.0)
        quat = pose[:1, 3:7]
        up = math_utils.quat_apply_inverse(quat, quat.new_tensor([[0.0, 0.0, 1.0]]))
        x = math_utils.quat_apply_inverse(quat, quat.new_tensor([[1.0, 0.0, 0.0]]))
        return {"pos": pose[0, :3].clone(), "up_body": up, "x_body": x}


def delta_rotation(q1: torch.Tensor, q0: torch.Tensor) -> torch.Tensor:
    """Rotation vector of ``q1 q0^-1`` (world frame), Isaac Lab's quaternion convention."""
    from isaaclab.utils import math as math_utils

    return math_utils.axis_angle_from_quat(math_utils.quat_mul(q1, math_utils.quat_conjugate(q0)))


def rotate(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    from isaaclab.utils import math as math_utils

    return math_utils.quat_apply(q, v.expand(q.shape[0], -1))


def step_reward(plant, grasp, pos0, quat0, pos1, quat1, dt):
    """The RL reward terms for one control step of every world, and whether each is outside the envelope."""
    R = REWARD
    origins = plant.env.scene.env_origins
    dq = delta_rotation(quat1, quat0)
    omega = dq / dt
    lin = (pos1 - pos0) / dt
    dist = (pos1 - origins - grasp["pos"]).norm(dim=1)
    credit = ((R["zero_credit_m"] - dist) / (R["zero_credit_m"] - R["full_credit_m"])).clamp(0.0, 1.0)
    rot = omega[:, 2].clamp(-R["rotation_clip"], R["rotation_clip"])
    rot = torch.where(rot > 0, credit * rot, rot)
    up = rotate(quat1, grasp["up_body"])[:, 2]
    r = (
        R["rotation"] * rot
        + R["lin_vel"] * lin.abs().sum(dim=1)
        + R["torque"] * plant.torque_l2()
        + R["off_axis"] * (omega[:, 0].abs() + omega[:, 1].abs())
        + R["distance"] * dist
        + R["over_speed"] * (omega[:, 2] - R["omega_max"]).clamp_min(0.0)
        + R["tilt"] * (R["free_tilt_dot"] - up).clamp_min(0.0)
    )
    outside = (dist > ENVELOPE_M) | (up < math.cos(math.radians(ENVELOPE_TILT_DEG)))
    return r, outside, dq, dist, up


def interp_knots(knots: torch.Tensor, horizon: int) -> torch.Tensor:
    """(S, K, A) knots, linearly interpolated to (S, H, A) over the horizon (K = 1: zero-order hold)."""
    S, K, A = knots.shape
    if K == 1:
        return knots.expand(S, horizon, A)
    s = torch.linspace(0.0, K - 1, horizon, device=knots.device)
    lo = s.floor().long().clamp(max=K - 2)
    w = (s - lo)[None, :, None]
    return knots[:, lo] * (1 - w) + knots[:, lo + 1] * w


def verify(plant, copier, cache, steps: int = 20) -> dict:
    """State copy checks (see the module docstring)."""
    env = plant.env
    W, A = env.num_envs, plant.action_dim
    g = torch.Generator(device=env.device).manual_seed(0)
    plant.set_grasp(cache, 0)
    for _ in range(5):  # leave the reset state so the solver's warm start and sparse sync matter
        plant.step(torch.zeros(W, A, device=env.device))
    copier.broadcast(0)
    spread0 = copier.max_spread()
    snap = copier.snapshot(0)
    same = (torch.rand(steps, 1, A, generator=g, device=env.device) * 2 - 1).expand(steps, W, A).contiguous()
    traj = []
    for k in range(steps):
        plant.step(same[k])
        traj.append(plant.cube()[0][0].clone())
    spread_same = copier.max_spread()
    pos_same, quat_same = plant.cube()
    cube_spread = float((pos_same - pos_same[:1]).abs().max())
    # restore, world 0 replays the same actions, the others run different ones
    copier.restore(snap)
    other = torch.rand(steps, W, A, generator=g, device=env.device) * 2 - 1
    other[:, 0] = same[:, 0]
    replay_err = 0.0
    for k in range(steps):
        plant.step(other[k])
        replay_err = max(replay_err, float((plant.cube()[0][0] - traj[k]).abs().max()))
    pos_other, _ = plant.cube()
    out = {
        "max_spread_after_copy": spread0,
        "max_spread_after_same_actions": spread_same,
        "cube_pos_spread_same_actions_m": cube_spread,
        "world0_replay_max_err_m": replay_err,
        "cube_pos_spread_different_actions_m": float((pos_other - pos_other[:1]).abs().max()),
        "cube_travel_m": float((traj[-1] - traj[0]).norm()),
    }
    print(json.dumps(out, indent=2))
    return out


def run_episode(plant, copier, cache, index, args, log_every=30, record=None):
    env = plant.env
    W, A, H, dev = env.num_envs, plant.action_dim, args.horizon, env.device
    S = W
    dt = env.step_dt
    grasp = plant.set_grasp(cache, index)
    gen = torch.Generator(device=dev).manual_seed(args.seed + index)
    plan = torch.zeros(args.knots, A, device=dev)  # knots of the nominal plan
    snap = copier.snapshot(0)
    steps = int(round(args.episode_s / dt))
    yaw = heading = 0.0
    max_disp, min_up = 0.0, 1.0
    first_exit = None
    pos_prev, quat_prev = (v[:1] for v in plant.cube())
    x0 = rotate(quat_prev, grasp["x_body"])[0]
    trace = []
    wall, rollout_s = [], []
    discount = args.discount ** torch.arange(H, device=dev, dtype=torch.float32)
    sigma = args.sigma
    for t in range(steps):
        t0 = time.perf_counter()
        # samples: noise on the knots around the nominal plan; sample 0 is the plan itself
        noise = torch.randn(S, args.knots, A, generator=gen, device=dev) * sigma
        noise[0] = 0.0
        knots = (plan[None] + noise).clamp(-1.0, 1.0)
        seq = interp_knots(knots, H)
        cost = torch.zeros(S, device=dev)
        failed = torch.zeros(S, dtype=torch.bool, device=dev)
        copier.restore(snap)
        torch.cuda.synchronize()
        t1 = time.perf_counter()
        p0, q0 = plant.cube()
        for h in range(H):
            plant.step(seq[:, h])
            p1, q1 = plant.cube()
            r, outside, *_ = step_reward(plant, grasp, p0, q0, p1, q1, dt)
            failed |= outside
            cost += discount[h] * (-r + args.drop_cost * failed.float() + args.action_cost * seq[:, h].square().sum(1))
            p0, q0 = p1, q1
        cost = cost.nan_to_num(1e6)
        torch.cuda.synchronize()
        rollout_s.append(time.perf_counter() - t1)
        if args.temperature > 0:
            spread = (cost.max() - cost.min()).clamp_min(1e-6)
            w = torch.softmax(-(cost - cost.min()) / (args.temperature * spread), dim=0)
            plan = (w[:, None, None] * knots).sum(0)
        else:
            plan = knots[int(cost.argmin())]
        first = interp_knots(plan[None], H)[0, 0]
        # the real step: every world from the snapshot with the chosen action; world 0 is the system
        copier.restore(snap)
        plant.step(first.expand(W, A).contiguous())
        snap = copier.snapshot(0)
        if record is not None:
            record.append(copier.joint_q()[0].clone())
        # shift the plan by one control step (knots span the horizon)
        shift = 1.0 / max(H - 1, 1) * (args.knots - 1)
        s = torch.arange(args.knots, device=dev, dtype=torch.float32) + shift
        lo = s.floor().long().clamp(max=args.knots - 1)
        hi = (lo + 1).clamp(max=args.knots - 1)
        w_ = (s - lo).clamp(0, 1)[:, None]
        plan = plan[lo] * (1 - w_) + plan[hi] * w_
        torch.cuda.synchronize()
        wall.append(time.perf_counter() - t0)
        # metrics, as eval_policy.py
        pos, quat = (v[:1] for v in plant.cube())
        _, outside, dq, dist, up = step_reward(plant, grasp, pos_prev, quat_prev, pos, quat, dt)
        yaw += float(dq[0, 2])
        x1 = rotate(quat, grasp["x_body"])[0]
        xp = rotate(quat_prev, grasp["x_body"])[0]
        heading += math.atan2(float(xp[0] * x1[1] - xp[1] * x1[0]), float(xp[0] * x1[0] + xp[1] * x1[1]))
        max_disp, min_up = max(max_disp, float(dist[0])), min(min_up, float(up[0]))
        if first_exit is None and bool(outside[0]):
            first_exit = t
        pos_prev, quat_prev = pos, quat
        trace.append([t * dt + dt, yaw, heading, float(dist[0]), float(up[0]), float(cost.min()), float(cost.mean())])
        if t % log_every == 0:
            print(f"  grasp {index} t={t * dt:5.2f}s yaw={math.degrees(yaw):7.1f} deg disp={1e3 * float(dist[0]):5.1f} mm "
                  f"up={float(up[0]):.3f} cost[min,mean]=[{float(cost.min()):.2f},{float(cost.mean()):.2f}] "
                  f"{1e3 * wall[-1]:.0f} ms/step ({1e3 * rollout_s[-1]:.0f} rollout)", flush=True)
        if float(dist[0]) > args.abort_m or not math.isfinite(yaw):
            break
    del x0
    turn = min(yaw, heading)
    retained = max_disp <= ENVELOPE_M and math.degrees(math.acos(max(-1.0, min(1.0, min_up)))) <= ENVELOPE_TILT_DEG
    rot_in_env = None
    tr = np.array(trace)
    if first_exit is not None:
        rot_in_env = float(np.degrees(min(tr[first_exit - 1, 1], tr[first_exit - 1, 2]))) if first_exit > 0 else 0.0
    return {
        "grasp_index": index,
        "duration_s": len(trace) * dt,
        "signed_rotation_deg": math.degrees(yaw),
        "projected_heading_change_deg": math.degrees(heading),
        "max_displacement_mm": 1e3 * max_disp,
        "peak_tilt_deg": math.degrees(math.acos(max(-1.0, min(1.0, min_up)))),
        "retained_40mm_60deg": retained,
        "rotation_before_leaving_envelope_deg": rot_in_env,
        "quarter_turn": retained and turn >= math.pi / 2,
        "half_turn": retained and turn >= math.pi,
        "three_quarter_turn": retained and turn >= 1.5 * math.pi,
        "full_turn": retained and turn >= 2 * math.pi,
        "wall_s_per_control_step_mean": float(np.mean(wall)),
        "wall_s_per_control_step_p90": float(np.percentile(wall, 90)),
        "real_time_factor": dt / float(np.mean(wall)),
        "rollout_s_per_control_step_mean": float(np.mean(rollout_s)),
    }, tr


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--task", default="CoDe-Rotate-Cube-O")
    parser.add_argument("--num_samples", type=int, default=64, help="N rollout worlds (world 0 is extra)")
    parser.add_argument("--horizon", type=int, default=12, help="control steps (30 Hz): 12 is 0.4 s")
    parser.add_argument("--knots", type=int, default=4)
    parser.add_argument("--sigma", type=float, default=0.5, help="knot noise, action units")
    parser.add_argument("--temperature", type=float, default=0.05, help="MPPI, relative to the cost spread; 0: best")
    parser.add_argument("--discount", type=float, default=1.0)
    parser.add_argument("--drop_cost", type=float, default=10.0, help="per step outside the 40 mm / 60 deg envelope")
    parser.add_argument("--action_cost", type=float, default=0.01)
    parser.add_argument("--distance_weight", type=float, default=REWARD["distance"],
                        help="weight of |p - p_grasp| per step (RL: -4)")
    parser.add_argument("--tilt_weight", type=float, default=REWARD["tilt"],
                        help="weight of the tilt deficit beyond about 20 deg per step (RL: -1)")
    parser.add_argument("--grasps", type=int, nargs="+", default=[0, 1, 2, 3, 4], help="held-out cache indices")
    parser.add_argument("--episode_s", type=float, default=20.0)
    parser.add_argument("--abort_m", type=float, default=0.07, help="end an episode once the cube is this far off")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--verify", action="store_true", help="only check the state copy")
    parser.add_argument("--video", action="store_true", help="record world 0 (Newton GL, offscreen)")
    parser.add_argument("--output", type=Path)
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args = parser.parse_args(argv)
    W = args.num_samples + 1
    REWARD["distance"] = args.distance_weight
    REWARD["tilt"] = args.tilt_weight
    env_cfg, _ = resolve_task_config(args.task, None, overrides=["physics=newton_mjwarp"])
    env_cfg.scene.num_envs = W
    env_cfg.scene.env_spacing = 0.0  # every world on one origin: copied states mean the same thing in every world
    env_cfg.terminations.object_off_grasp = None
    env_cfg.terminations.object_tilted = None
    env_cfg.episode_length_s = 1e6
    env_cfg.observations.policy.enable_corruption = False
    if args.deterministic:  # Isaac Lab's launcher flag: Warp run-to-run determinism (needs MuJoCo-Warp sensors off)
        env_cfg.sim.physics.deterministic_mode = "run_to_run"
        env_cfg.sim.physics.solver_cfg.disable_sensors = True
    args.device = env_cfg.sim.device
    cache_path = Path(os.environ["CODE_ROTATE_CUBE_GRASPS"]) / "O.pt"
    cache = torch.load(cache_path, map_location="cpu", weights_only=True)

    import gymnasium as gym

    with launch_simulation(env_cfg, args), torch.inference_mode():
        env = gym.make(args.task, cfg=env_cfg).unwrapped
        env.reset(seed=args.seed)
        plant, copier = Plant(env), WorldCopier(env)
        print(f"worlds {W}, action dim {plant.action_dim}, control dt {env.step_dt:.4f} s, decimation "
              f"{env.cfg.decimation}, GPU alloc {torch.cuda.memory_allocated() / 2**20:.0f} MiB", flush=True)
        check = verify(plant, copier, cache)
        if args.verify:
            env.close()
            return 0
        out_dir = SIM_ROOT / "logs" / "rotate_cube" / "mpc"
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        out = args.output or out_dir / f"{stamp}_N{args.num_samples}_H{args.horizon}.json"
        episodes, traces = [], {}
        for index in args.grasps:
            record = [] if args.video and index == args.grasps[0] else None
            result, tr = run_episode(plant, copier, cache, index, args, record=record)
            if record:
                render_video(env, record, out.with_name(out.stem + f"_grasp{index}.mp4"))
            episodes.append(result)
            traces[f"grasp{index}"] = tr
            print(json.dumps(result), flush=True)
            summary = summarize(episodes)
            record = {
                "settings": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()
                             if k in ("task", "num_samples", "horizon", "knots", "sigma", "temperature", "discount",
                                      "drop_cost", "action_cost", "grasps", "episode_s", "abort_m", "seed",
                                      "deterministic")},
                "control_dt_s": env.step_dt,
                "horizon_s": args.horizon * env.step_dt,
                "reward_weights": REWARD,
                "grasp_cache": str(cache_path),
                "state_copy_check": check,
                "gpu_max_allocated_mib": torch.cuda.max_memory_allocated() / 2**20,
                "summary": summary,
                "episodes": episodes,
                "rl_reference_s15": {"signed_rotation_deg_mean": 154.4, "quarter": 0.525, "half": 0.243,
                                     "three_quarter": 0.072, "full": 0.0117, "source": "report/study4/in-hand.md"},
            }
            out.write_text(json.dumps(record, indent=2) + "\n")
            np.savez_compressed(out.with_suffix(".npz"), columns=np.array(
                ["t_s", "yaw_rad", "heading_rad", "displacement_m", "up_dot", "cost_min", "cost_mean"]), **traces)
        print(json.dumps(summarize(episodes), indent=2))
        print(out)
        env.close()
    return 0


def render_video(env, joint_q: list, path: Path, fps: int = 30) -> None:
    """World 0's recorded joint coordinates, replayed through forward kinematics in an offscreen Newton GL viewer."""
    import imageio
    import newton
    import warp as wp
    from newton.viewer import ViewerGL

    from code_isaaclab.tasks.rotate_cube.camera import CAMERA_EYE_ENV, CAMERA_LOOKAT_ENV

    model = env.sim.physics_manager.get_model()
    state = model.state()
    W = model.world_count
    q_all = wp.to_torch(state.joint_q).reshape(W, -1)
    viewer = ViewerGL(width=960, height=540, headless=True)
    viewer.set_model(model)
    viewer.set_visible_worlds([0])
    viewer.show_collision = True  # the pads and the cube as simulated
    eye, look = np.array(CAMERA_EYE_ENV), np.array(CAMERA_LOOKAT_ENV)
    d = look - eye
    pitch = math.degrees(math.asin(d[2] / np.linalg.norm(d)))
    yaw = math.degrees(math.atan2(d[1], d[0]))
    viewer.set_camera(wp.vec3(*eye.tolist()), pitch, yaw)
    with imageio.get_writer(path, fps=fps, codec="libx264", quality=7) as writer:
        for k, q in enumerate(joint_q):
            q_all[:] = q
            newton.eval_fk(model, state.joint_q, state.joint_qd, state)
            viewer.begin_frame(k / fps)
            viewer.log_state(state)
            viewer.end_frame()
            writer.append_data(viewer.get_frame().numpy())
    viewer.close()
    print(path, flush=True)


def summarize(episodes):
    n = len(episodes)
    return {
        "episodes": n,
        "signed_rotation_deg_mean": float(np.mean([e["signed_rotation_deg"] for e in episodes])),
        "signed_rotation_deg": [round(e["signed_rotation_deg"], 1) for e in episodes],
        **{k: sum(e[k] for e in episodes) for k in ("quarter_turn", "half_turn", "three_quarter_turn", "full_turn")},
        "retained": sum(e["retained_40mm_60deg"] for e in episodes),
        "wall_s_per_control_step_mean": float(np.mean([e["wall_s_per_control_step_mean"] for e in episodes])),
        "real_time_factor_mean": float(np.mean([e["real_time_factor"] for e in episodes])),
    }


if __name__ == "__main__":
    sys.exit(main())
