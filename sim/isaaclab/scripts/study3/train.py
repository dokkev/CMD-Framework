# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3: train one PPO policy on one contact interface with the project's standard RSL-RL entry point.

    uv run --extra rsl-rl python scripts/study3/train.py --variant flat --seed 0 --iterations 400 [--num_envs 128]

It runs ``isaaclab train --rl_library rsl_rl`` unchanged (MuJoCo-Warp, the task's PPO runner config) in a child
process, under the run name ``<variant>_s<seed>``, and writes ``study3_manifest.json`` next to the checkpoints:
the git state, versions, assets, the task, seed, iterations, environment count, transitions (iterations x
steps per environment x environments), wall time, GPU memory, and the exit status. Every variant uses the same
runner config; only ``--variant`` changes the task.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

EXPERIMENT = "study3_code_reorient"
STEPS_PER_ENV = 24  # AllegroHandManagerPPORunnerCfg.num_steps_per_env


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--variant", choices=tuple(common.TASKS), required=True)
    parser.add_argument("--protocol", choices=tuple(common.PROTOCOLS), default="pilot")
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--iterations", type=int, required=True)
    parser.add_argument("--num_envs", type=int, default=128)
    parser.add_argument("--physics", default="newton_mjwarp")
    parser.add_argument("--label", default="", help="suffix of the run name (e.g. smoke)")
    parser.add_argument("--override", action="append", default=[],
                        help="Hydra override of the task, e.g. env.actions.joint_pos.alpha=0.1 (repeatable; recorded)")
    args = parser.parse_args(argv)
    task = common.PROTOCOLS[args.protocol][args.variant]
    run_name = f"{args.variant}_s{args.seed}" + (f"_{args.label}" if args.label else "")
    stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    log_dir = common.SIM_ROOT / "logs" / "rsl_rl" / EXPERIMENT / f"{stamp}_{run_name}"
    command = [
        "uv", "run", "--extra", "rsl-rl", "isaaclab", "train", "--rl_library", "rsl_rl",
        "--task", task, "--num_envs", str(args.num_envs), "--seed", str(args.seed),
        "--max_iterations", str(args.iterations), "--run_name", run_name, "--run_timestamp", stamp,
        f"physics={args.physics}", *args.override,
    ]  # fmt: skip
    started = time.time()
    env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD="1")
    with open(common.LOG_ROOT.parent / f"train_{stamp}_{run_name}.log", "w") as log:
        status = subprocess.run(command, cwd=common.SIM_ROOT, env=env, stdout=log, stderr=subprocess.STDOUT).returncode
    wall = time.time() - started
    log_text = (common.LOG_ROOT.parent / f"train_{stamp}_{run_name}.log").read_text(errors="replace")
    overflows = log_text.count("overflow")
    checkpoints = sorted(log_dir.glob("model_*.pt"), key=lambda p: int(p.stem.split("_")[1])) if log_dir.exists() else []
    record = common.manifest(
        task=task,
        variant=args.variant,
        protocol=args.protocol,
        seed=args.seed,
        physics=args.physics,
        servo_mode=os.environ.get("CODE_STUDY3_SERVO", "implicit"),
        iterations=args.iterations,
        overrides=args.override,
        num_envs=args.num_envs,
        steps_per_env=STEPS_PER_ENV,
        transitions=args.iterations * STEPS_PER_ENV * args.num_envs,
        command=command,
        log_dir=str(log_dir),
        checkpoints=[p.name for p in checkpoints],
        final_checkpoint_sha256=common.file_sha256(checkpoints[-1]) if checkpoints else None,
        wall_s=wall,
        exit_status=status,
        solver_overflow_messages=overflows,
        status="passed" if status == 0 and checkpoints and overflows == 0 else "failed",
    )
    target = log_dir if log_dir.exists() else common.LOG_ROOT / "train_failed" / f"{stamp}_{run_name}"
    common.write_json(target / "study3_manifest.json", record)
    print(json.dumps({k: record[k] for k in ("task", "seed", "transitions", "wall_s", "status", "log_dir")}))
    return 0 if record["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
