# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Servo speeds and abnormal terminations of a policy on its training resets.

    uv run --extra rsl-rl python scripts/rotate_cube/servo_speed_probe.py --checkpoint <model.pt> --out <json> \\
        [--task CoDe-Rotate-Cube-O-OmniReset] [--num_envs 128] [--steps 600] physics=newton_mjwarp

Rolls the deterministic policy out on the task's own resets and reports, over every control step and actuated servo:
the p50/p95/p99/max of |q_dot| (servo joint, rad/s; the XM335 no-load speed is 5.55 rad/s, which the implicit servo
model does not enforce in MuJoCo-Warp), per joint, the share of samples above 1x and 2x that speed, and how the episodes
ended (time-out, abnormal joint speed, non-finite state, cube out of reach), by reset type.
"""

from __future__ import annotations

import argparse
import collections
import json
import sys
from importlib.metadata import version


def main() -> int:
    from isaaclab.app import add_launcher_args, launch_simulation

    from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli

    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--task", default="CoDe-Rotate-Cube-O-OmniReset")
    p.add_argument("--num_envs", type=int, default=128)
    p.add_argument("--steps", type=int, default=600)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--out", required=True)
    add_launcher_args(p)
    p.set_defaults(device=None, visualizer=[])
    args, hydra = setup_preset_cli(p, None)
    sys.argv = [sys.argv[0]] + hydra

    import gymnasium as gym
    import torch

    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.tasks.rotate_cube_omnireset.mdp import reset_term

    cfg, agent = resolve_task_config(args.task, "rsl_rl_cfg_entry_point")
    speeds = []
    speed_term = cfg.terminations.abnormal_robot
    inner = speed_term.func

    def recorded(env, factor: float = 2.0):  # runs every step before resets: the speeds that trip it are seen
        robot = env.scene["robot"]
        ids = [robot.joint_names.index(n) for n in env.action_manager.get_term("joint_pos")._joint_names]
        speeds.append(robot.data.joint_vel.torch[:, ids].abs().clone())
        return inner(env, factor=factor)

    speed_term.func = recorded
    cfg.scene.num_envs = args.num_envs
    cfg.seed = agent.seed = args.seed
    agent = handle_deprecated_rsl_rl_cfg(agent, version("rsl-rl-lib"))
    args.device = cfg.sim.device
    with launch_simulation(cfg, args), torch.inference_mode():
        env = gym.make(args.task, cfg=cfg)
        u = env.unwrapped
        wrapped = RslRlVecEnvWrapper(env, clip_actions=agent.clip_actions)
        runner = create_rsl_rl_runner(wrapped, agent)
        runner.load(args.checkpoint, load_cfg={"actor": True}, map_location=wrapped.device)
        policy = runner.get_inference_policy(device=wrapped.device)
        robot = u.scene["robot"]
        names = list(u.action_manager.get_term("joint_pos")._joint_names)
        servos = [robot.joint_names.index(n) for n in names]
        limit = float(robot.data.joint_vel_limits.torch[0, servos].min())
        ends = collections.Counter()
        term = reset_term(u)
        obs = wrapped.get_observations()
        for _ in range(args.steps):
            task_id = term.task_id.clone()
            obs, _, dones, _ = wrapped.step(policy(obs))
            tm = u.termination_manager
            for e in dones.nonzero().flatten().tolist():
                kind = next((k for k in tm.active_terms if bool(tm.get_term(k)[e])), "?")
                ends[(term.names[int(task_id[e])], kind)] += 1
        s = torch.stack(speeds[-args.steps:])  # (steps, envs, servos), before each step's resets
        flat = s.reshape(-1)
        q = torch.tensor([0.5, 0.95, 0.99], device=flat.device)
        out = {
            "checkpoint": args.checkpoint, "task": args.task, "num_envs": args.num_envs, "steps": args.steps,
            "seed": args.seed, "servo_speed_limit_rad_s": limit,
            "servo_speed_rad_s_p50_p95_p99": flat.quantile(q).tolist() if flat.numel() < 2**24 else
            flat[torch.randperm(flat.numel(), device=flat.device)[: 2**24 - 1]].quantile(q).tolist(),
            "servo_speed_rad_s_max": float(flat.max()),
            "share_above_1x_limit": float((flat > limit).float().mean()),
            "share_above_2x_limit": float((flat > 2 * limit).float().mean()),
            "per_joint_p95_max": {n: [float(s[..., k].reshape(-1).quantile(0.95)), float(s[..., k].max())]
                                  for k, n in enumerate(names)},
            "episode_ends": {f"{t}/{k}": c for (t, k), c in sorted(ends.items())},
        }  # fmt: skip
        totals = collections.Counter()
        for (_, k), c in ends.items():
            totals[k] += c
        out["episode_end_shares"] = {k: c / max(sum(totals.values()), 1) for k, c in totals.items()}
        with open(args.out, "w") as f:
            json.dump(out, f, indent=1)
        print(json.dumps({k: out[k] for k in ("servo_speed_rad_s_p50_p95_p99", "servo_speed_rad_s_max",
                                              "share_above_1x_limit", "share_above_2x_limit",
                                              "episode_end_shares")}, indent=1))  # fmt: skip
        env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
