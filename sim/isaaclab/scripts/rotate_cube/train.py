# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Train cube rotation (``CoDe-Rotate-Cube-O`` by default), logged to Weights & Biases, with a manifest per run.

    uv run python scripts/rotate_cube/train.py --iterations 200 --num_envs 1024 --seed 16 --no_video \\
        --checkpoint <model.pt> --fresh_optimizer [--label <name>] [--override <hydra override> ...]

(Formerly ``scripts/reorient_o/train.py``; manifests of runs up to s15 name that path.) Runs ``isaaclab train
--rl_library rsl_rl`` in a child process with ``--logger wandb --log_project_name code-hand-inhand`` and, unless
``--no_video``, ``--video --viz newton_gl``: the Newton GL viewer records a ``--video_length`` step clip every
``--video_interval`` environment steps into the run's ``videos/train`` (offscreen, 4 worlds at 960 x 540,
``rotate_cube.camera``). The child blocks this process; launch it detached (``nohup ... &``) for long runs.

The run is named ``<stamp>_<prefix>_<hand>_s<seed>[_<label>]`` (rsl_rl names the W&B run after the log directory).
Writes the console log to ``logs/train_<stamp>_<run>.log`` and ``reorient_manifest.json`` next to the checkpoints
(``logs/rsl_rl/<experiment>/<stamp>_<run>/``): git state, versions, the command, the overrides, the task's environment
switches (``CODE_ROTATE_CUBE_*``), the W&B run URL and the checkpoints. W&B credentials come from ``~/.netrc``; nothing
here reads or prints them.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import secrets
import string
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "study3_measurement"))
import common  # noqa: E402  (run records: git state, versions, immutable result files)

CAMERA_PY = HERE.parents[1] / "src" / "code_isaaclab" / "tasks" / "rotate_cube" / "camera.py"
_spec = importlib.util.spec_from_file_location("rotate_cube_camera", CAMERA_PY)  # by path: the package starts Isaac Lab
_camera = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_camera)
env0_camera = _camera.env0_camera

EXPERIMENT = "code_rotate_cube"
PROJECT = "code-hand-inhand"
TASK_ENV_KEYS = (
    "CODE_ROTATE_CUBE_GRASPS", "CODE_ROTATE_CUBE_CROWN_MM", "CODE_ROTATE_CUBE_CROWN_LAYOUT",
    "CODE_ROTATE_CUBE_PHASE_PERIOD_S", "CODE_ROTATE_CUBE_GOAL_OBS", "CODE_ROTATE_CUBE_HORA",
    "CODE_ROTATE_CUBE_GRU", "CODE_ROTATE_CUBE_RESET_POOL",
)
"""Recorded in the manifest when set (unset: the task's defaults, see ``tasks/rotate_cube/env_cfg.py``)."""


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--task", default="CoDe-Rotate-Cube-O")
    parser.add_argument("--experiment", default=EXPERIMENT, help="the runner config's experiment_name")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--iterations", type=int, required=True, help="PPO updates to run, including after a restart")
    parser.add_argument("--num_envs", type=int, default=4096)
    parser.add_argument("--video_interval", type=int, default=1800, help="environment steps between clips")
    parser.add_argument("--video_length", type=int, default=300)
    parser.add_argument("--no_video", action="store_true")
    parser.add_argument("--physics", default="newton_mjwarp")
    parser.add_argument("--label", default="")
    parser.add_argument("--checkpoint", type=Path, help="local checkpoint to continue training from")
    parser.add_argument("--fresh_optimizer", action="store_true", help="restart the PPO optimizer at the checkpoint")
    parser.add_argument("--logger", choices=("wandb", "tensorboard"), default="wandb")
    parser.add_argument("--override", action="append", default=[], help="Hydra override (repeatable)")
    args = parser.parse_args(argv)
    checkpoint = ["--checkpoint", str(args.checkpoint.resolve(strict=True))] if args.checkpoint else []
    optimizer = ["--no-resume_optimizer_state"] if args.fresh_optimizer else []
    hand = "_".join(args.task.split("-")[3:])  # O, R, U
    prefix = "_".join(args.task.split("-")[1:3]).lower()  # rotate_cube
    experiment = args.experiment
    run_name = f"{prefix}_{hand}_s{args.seed}" + (f"_{args.label}" if args.label else "")
    stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    log_dir = common.SIM_ROOT / "logs" / "rsl_rl" / experiment / f"{stamp}_{run_name}"
    eye, lookat = env0_camera(args.num_envs)
    video = [] if args.no_video else [
        "--video", "--video_length", str(args.video_length), "--video_interval", str(args.video_interval),
        "--viz", "newton_gl",
        f"env.sim.default_visualizer_cfg.eye=[{','.join(map(str, eye))}]",
        f"env.sim.default_visualizer_cfg.lookat=[{','.join(map(str, lookat))}]",
    ]  # fmt: skip
    command = [
        "uv", "run", "--extra", "rsl-rl", "--extra", "video", "--extra", "wandb", "isaaclab", "train",
        "--rl_library", "rsl_rl", "--task", args.task, "--num_envs", str(args.num_envs), "--seed", str(args.seed),
        "--max_iterations", str(args.iterations), "--experiment_name", experiment,
        "--run_name", run_name, "--run_timestamp", stamp,
        "--logger", args.logger, "--log_project_name", PROJECT, *checkpoint, *optimizer,
        *video, f"physics={args.physics}", *args.override,
    ]  # fmt: skip
    log_path = common.SIM_ROOT / "logs" / f"train_{stamp}_{run_name}.log"
    started = time.time()
    # Isaac Lab pins the run id with wandb.util.generate_id() unless WANDB_RUN_ID is set; wandb 0.31 removed it, so the
    # id is drawn here (the same alphabet and length: 8 lowercase letters and digits)
    run_id = "".join(secrets.choice(string.ascii_lowercase + string.digits) for _ in range(8))
    env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD="1", WANDB_RUN_ID=run_id)
    # The recording viewer is offscreen (headless): pyglet picks its X11 path whenever a display is set and then rejects
    # its own headless canvas, so the child runs without one and uses offscreen EGL.
    for var in ("DISPLAY", "WAYLAND_DISPLAY"):
        env.pop(var, None)
    with open(log_path, "w") as log:
        status = subprocess.run(command, cwd=common.SIM_ROOT, env=env, stdout=log, stderr=subprocess.STDOUT).returncode
    wall = time.time() - started
    text = log_path.read_text(errors="replace")
    agent_params = log_dir / "params" / "agent.yaml"
    rollout_steps = None
    if agent_params.exists():
        rollout_steps = re.search(r"^num_steps_per_env: (\d+)$", agent_params.read_text(), re.MULTILINE)
    counts = re.findall(r"^\s*Total steps: (\d+)", text, re.MULTILINE)
    steps_per_env = int(rollout_steps[1]) if rollout_steps else None
    transitions = int(counts[-1]) if counts else None
    requested_transitions = args.iterations * args.num_envs * steps_per_env if steps_per_env else None
    urls = sorted(set(re.findall(r"View run .* at: (https://\S+/runs/\w+)", text)))
    checkpoints = sorted(log_dir.glob("model_*.pt"), key=lambda p: int(p.stem.split("_")[1]))
    videos = sorted(str(p.relative_to(log_dir)) for p in log_dir.rglob("*.mp4")) if log_dir.exists() else []
    run_status = "failed"
    if status == 0 and checkpoints:
        run_status = "passed" if transitions is not None and transitions == requested_transitions else "incomplete"
    record = common.manifest(
        task=args.task, seed=args.seed, physics=args.physics, iterations=args.iterations, num_envs=args.num_envs,
        steps_per_env=steps_per_env, transitions=transitions, requested_transitions=requested_transitions,
        overrides=args.override, command=command, log=str(log_path), log_dir=str(log_dir),
        task_settings={key: env[key] for key in TASK_ENV_KEYS if key in env},
        wandb_run_id=run_id if args.logger == "wandb" else None,
        wandb_run_urls=urls,
        videos=videos, checkpoints=[p.name for p in checkpoints], wall_s=wall, exit_status=status,
        solver_overflow_messages=text.count("overflow"),
        status=run_status,
    )
    target = log_dir if log_dir.exists() else log_dir.parent / "failed" / log_dir.name
    common.write_json(target / "reorient_manifest.json", record)
    print(json.dumps({k: record[k] for k in ("task", "seed", "transitions", "wall_s", "status", "log_dir",
                                             "wandb_run_urls", "videos")}))
    return 0 if record["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
