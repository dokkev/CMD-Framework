# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Train in-hand cylinder rotation (``CoDe-Rotate-Cylinder-O``) with the project's standard RSL-RL entry point.

    uv run --extra rsl-rl python scripts/rotate/train.py --seed 0 --iterations 1000 --num_envs 4096 [--label first]

Runs ``isaaclab train --rl_library rsl_rl`` unchanged (MuJoCo-Warp, the task's runner config) in a child process,
under the run name ``O_s<seed>[_<label>]``, logs to ``logs/train_<stamp>_<run>.log`` and writes ``rotate_manifest.json``
next to the checkpoints (``logs/rsl_rl/code_rotate_cylinder_o/<stamp>_<run>/``): git state, versions, the task, seed,
iterations, environments, transitions, wall time, exit status and solver overflow messages.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "study3_measurement"))
import common  # noqa: E402  (run records: git state, versions, immutable result files)

TASK = "CoDe-Rotate-Cylinder-O"
EXPERIMENT = "code_rotate_cylinder_o"
STEPS_PER_ENV = 24  # AllegroHandManagerPPORunnerCfg.num_steps_per_env


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--task", default=TASK, help="CoDe-Rotate-Cylinder-O or CoDe-Rotate-Cylinder-O-Smooth")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--iterations", type=int, required=True)
    parser.add_argument("--num_envs", type=int, default=4096)
    parser.add_argument("--physics", default="newton_mjwarp")
    parser.add_argument("--label", default="")
    parser.add_argument("--override", action="append", default=[], help="Hydra override of the task (repeatable)")
    args = parser.parse_args(argv)
    run_name = f"O_s{args.seed}" + (f"_{args.label}" if args.label else "")
    stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    log_dir = common.SIM_ROOT / "logs" / "rsl_rl" / EXPERIMENT / f"{stamp}_{run_name}"
    command = [
        "uv", "run", "--extra", "rsl-rl", "isaaclab", "train", "--rl_library", "rsl_rl",
        "--task", args.task, "--num_envs", str(args.num_envs), "--seed", str(args.seed),
        "--max_iterations", str(args.iterations), "--run_name", run_name, "--run_timestamp", stamp,
        f"physics={args.physics}", *args.override,
    ]  # fmt: skip
    log_path = common.SIM_ROOT / "logs" / f"train_{stamp}_rotate_{run_name}.log"
    started = time.time()
    env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD="1")
    with open(log_path, "w") as log:
        status = subprocess.run(command, cwd=common.SIM_ROOT, env=env, stdout=log, stderr=subprocess.STDOUT).returncode
    wall = time.time() - started
    text = log_path.read_text(errors="replace")
    checkpoints = sorted(log_dir.glob("model_*.pt"), key=lambda p: int(p.stem.split("_")[1])) if log_dir.exists() else []
    record = common.manifest(
        task=args.task, seed=args.seed, physics=args.physics, iterations=args.iterations, num_envs=args.num_envs,
        steps_per_env=STEPS_PER_ENV, transitions=args.iterations * STEPS_PER_ENV * args.num_envs,
        overrides=args.override, command=command, log=str(log_path), log_dir=str(log_dir),
        checkpoints=[p.name for p in checkpoints], wall_s=wall, exit_status=status,
        solver_overflow_messages=text.count("overflow"),
        status="passed" if status == 0 and checkpoints else "failed",
    )
    target = log_dir if log_dir.exists() else common.SIM_ROOT / "logs" / "rsl_rl" / EXPERIMENT / "failed" / f"{stamp}_{run_name}"
    common.write_json(target / "rotate_manifest.json", record)
    print(json.dumps({k: record[k] for k in ("task", "seed", "transitions", "wall_s", "status", "log_dir")}))
    return 0 if record["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
