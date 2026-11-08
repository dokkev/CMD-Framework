# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Roll out a hand task on one physics backend and save its metrics; or compare saved runs.

    uv run python scripts/rollout_metrics.py --task CoDe-Reorient-Cube-Allegro --policy random \
        --num_envs 16 --max_steps 300 --out logs/verify/mjwarp.json physics=newton_mjwarp
    uv run python scripts/rollout_metrics.py ... --out logs/verify/vbd.json physics=newton_vbd
    uv run python scripts/rollout_metrics.py --compare logs/verify/mjwarp.json logs/verify/vbd.json

The action sequence is seeded, so the same command on two backends replays identical actions.
Run one backend per process. The environment follows Isaac Lab's checkpoint-free agent workflow.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import sys
from pathlib import Path

import warp as wp

wp.config.enable_backward = False

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

SEED = 42


def _parse(argv):
    from isaaclab.app import add_launcher_args

    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--task", type=str)
    parser.add_argument("--policy", choices=("zero", "random"), default="random")
    parser.add_argument("--num_envs", type=int, default=16)
    parser.add_argument("--max_steps", type=int, default=300)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--compare", nargs="+", type=Path, help="print a table of saved runs and exit")
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args
    return args, hydra_args


def main(argv=None) -> int:
    args, hydra_args = _parse(argv)
    from code_isaaclab.verification import RolloutRecorder, compare

    if args.compare:
        print(compare(args.compare))
        return 0
    if not (args.task and args.out):
        raise SystemExit("--task and --out are required for a rollout")

    from isaaclab.app import launch_simulation
    from isaaclab.utils import validate

    from isaaclab_rl.entrypoints.simple_agents import create_random_action_policy, create_zero_action_policy

    from isaaclab_tasks.utils import resolve_task_config

    import code_isaaclab.tasks  # noqa: F401  (registers the project tasks)

    torch.manual_seed(SEED)
    env_cfg, _ = resolve_task_config(args.task, "")
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.seed = SEED
    args.device = env_cfg.sim.device
    validate(env_cfg)
    physics = next((t.split("=", 1)[1] for t in hydra_args if t.startswith("physics=")), "default")
    solver = type(env_cfg.sim.physics.solver_cfg).__name__
    solver_settings = {
        k: getattr(env_cfg.sim.physics.solver_cfg, k)
        for k in ("iterations", "integrator", "njmax")
        if hasattr(env_cfg.sim.physics.solver_cfg, k)
    }
    solver_settings["num_substeps"] = getattr(env_cfg.sim.physics, "num_substeps", None)

    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make(args.task, cfg=env_cfg)
        cleanup.callback(env.close)
        env.reset()
        policy = create_zero_action_policy(env) if args.policy == "zero" else create_random_action_policy(env)
        recorder = RolloutRecorder(env)
        for _ in range(args.max_steps):
            with torch.inference_mode():
                _, _, terminated, truncated, _ = env.step(policy())
                recorder.record(terminated | truncated)
        summary = recorder.summary(
            task=args.task,
            physics=physics,
            solver=solver,
            policy=args.policy,
            seed=SEED,
            solver_settings=solver_settings,
            overrides=hydra_args,
        )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summary, indent=2) + "\n")
    print(
        json.dumps({k: summary[k] for k in ("physics", "solver", "solver_settings", "steps", "resets", "all_finite")})
    )
    print(json.dumps(summary["mean"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
