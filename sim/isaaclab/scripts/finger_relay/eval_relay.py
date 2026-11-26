# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Evaluate finger-relay dial policies (attempt 7a', ``docs/experimental/rl_pip_only.md``): one 10 s episode per
environment from the rest pose, the deterministic policy; per checkpoint the dial's mean, median and p10/p90 rotation
[deg] at the end, goals per episode, the fraction of episodes that pass 90, 180 and 360 deg, the touch statistics
(share of steps each distal pad touches, and how often both touch at once), and the number of contact make/break events
per pad (gaiting).

    uv run --extra rsl-rl python scripts/finger_relay/eval_relay.py --task CoDe-Finger-Relay-O-Eval \\
        --checkpoint <model.pt> [--checkpoint ...] --num_envs 256 --eval_seed 94000 [--output <json>] physics=newton_mjwarp

Pad friction comes from ``CODE_CONTACT_PAD_MU`` (the interface swap). Writes one JSON (a list when several checkpoints).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from importlib.metadata import version
from pathlib import Path


def main() -> int:
    from isaaclab.app import add_launcher_args, launch_simulation

    from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli

    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--task", required=True)
    p.add_argument("--checkpoint", type=Path, action="append", required=True)
    p.add_argument("--num_envs", type=int, default=256)
    p.add_argument("--eval_seed", type=int, default=94000)
    p.add_argument("--output", type=Path)
    add_launcher_args(p)
    p.set_defaults(device=None, visualizer=[])
    args, hydra = setup_preset_cli(p, None)
    sys.argv = [sys.argv[0]] + hydra
    if args.output and args.output.exists():
        p.error(f"destination already exists: {args.output}")

    import gymnasium as gym
    import torch

    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.tasks.finger_relay.mdp import dial_goal

    cfg, agent = resolve_task_config(args.task, "rsl_rl_cfg_entry_point")
    cfg.scene.num_envs = args.num_envs
    cfg.seed = agent.seed = args.eval_seed
    agent = handle_deprecated_rsl_rl_cfg(agent, version("rsl-rl-lib"))
    args.device = cfg.sim.device
    results = []
    with launch_simulation(cfg, args), torch.inference_mode():
        env = gym.make(args.task, cfg=cfg)
        u = env.unwrapped
        wrapped = RslRlVecEnvWrapper(env)
        runner = create_rsl_rl_runner(wrapped, agent)
        sensor = u.scene.sensors["tip_touch"]
        steps = int(wrapped.max_episode_length) - 1
        for ck in args.checkpoint:
            runner.load(str(ck), load_cfg={"actor": True}, map_location=wrapped.device)
            policy = runner.get_inference_policy(device=wrapped.device)
            env.reset(seed=args.eval_seed)
            obs = wrapped.get_observations()
            n = u.num_envs
            touch_steps = torch.zeros(n, 2, device=u.device)
            both = torch.zeros(n, device=u.device)
            makes = torch.zeros(n, 2, device=u.device)
            prev = torch.zeros(n, 2, dtype=torch.bool, device=u.device)
            for _ in range(steps):
                obs, _, _, _ = wrapped.step(policy(obs))
                f = sensor.data.force_matrix_w.torch.reshape(n, -1, 3).norm(dim=-1) > 0.05
                touch_steps += f.float()
                both += (f[:, 0] & f[:, 1]).float()
                makes += (f & ~prev).float()
                prev = f
            g = dial_goal(u)
            rot = torch.rad2deg(g.theta)
            q = torch.quantile(rot, torch.tensor([0.1, 0.5, 0.9], device=rot.device)).tolist()
            res = {
                "checkpoint": str(ck), "task": args.task, "num_envs": n, "eval_seed": args.eval_seed,
                "pad_mu": float(os.environ.get("CODE_CONTACT_PAD_MU", "1.3")), "steps": steps,
                "rotation_deg_mean": float(rot.mean()), "rotation_deg_p10_p50_p90": q,
                "goals_per_episode": float(g.goals_reached.float().mean()),
                "past_90_rate": float((rot >= 90).float().mean()), "past_180_rate": float((rot >= 180).float().mean()),
                "past_360_rate": float((rot >= 360).float().mean()),
                "touch_share_index_middle": (touch_steps / steps).mean(0).tolist(),
                "both_touch_share": float((both / steps).mean()),
                "contact_makes_index_middle": makes.mean(0).tolist(),
            }
            results.append(res)
            print(json.dumps(res))
        env.close()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results[0] if len(results) == 1 else results, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
