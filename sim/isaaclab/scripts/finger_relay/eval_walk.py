# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Evaluate finger-walk policies (attempt 7b', ``docs/experimental/rl_pip_only.md``): one 10 s episode per environment
from a random rest start, the deterministic policy; per checkpoint the net distance walked toward the alternating
targets [mm] (mean, p10/p50/p90; an episode that drops the cylinder keeps what it walked before), goals per episode,
the share of episodes that drop it, the touch share of each distal pad and the contact make events per pad (gaiting).

    uv run --extra rsl-rl python scripts/finger_relay/eval_walk.py --task CoDe-Finger-Walk-O-Eval \\
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
    from code_isaaclab.tasks.finger_walk.mdp import walk_goal

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
            g = walk_goal(u)
            touch_steps = torch.zeros(n, 3, device=u.device)
            makes = torch.zeros(n, 3, device=u.device)
            prev = torch.zeros(n, 3, dtype=torch.bool, device=u.device)
            alive = torch.ones(n, dtype=torch.bool, device=u.device)
            walked = torch.zeros(n, device=u.device)
            goals = torch.zeros(n, device=u.device)
            lived = torch.zeros(n, device=u.device)
            for _ in range(steps):
                obs, _, dones, _ = wrapped.step(policy(obs))
                f = sensor.data.force_matrix_w.torch.reshape(n, -1, 3).norm(dim=-1) > 0.05
                a = alive[:, None].float()
                touch_steps += f.float() * a
                makes += (f & ~prev).float() * a
                prev = f
                ended = dones.bool() & alive
                # the reward term's running totals reset with the episode: take them on the step before
                walked = torch.where(alive & ~ended, g.walked, walked)
                goals = torch.where(alive & ~ended, g.goals_reached.float(), goals)
                lived += alive.float()
                alive &= ~ended
            mm = 1e3 * walked
            q = torch.quantile(mm, torch.tensor([0.1, 0.5, 0.9], device=mm.device)).tolist()
            res = {
                "checkpoint": str(ck), "task": args.task, "num_envs": n, "eval_seed": args.eval_seed,
                "pad_mu": float(os.environ.get("CODE_CONTACT_PAD_MU", "1.3")), "steps": steps,
                "walked_mm_mean": float(mm.mean()), "walked_mm_p10_p50_p90": q,
                "goals_per_episode": float(goals.mean()), "dropped_rate": float((~alive).float().mean()),
                "touch_share_index_middle_ring": (touch_steps / lived[:, None].clamp_min(1)).mean(0).tolist(),
                "contact_makes_index_middle_ring": makes.mean(0).tolist(),
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
