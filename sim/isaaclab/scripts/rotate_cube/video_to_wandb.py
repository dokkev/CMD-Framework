# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Record a play clip of a trained checkpoint and add it to that run's W&B page.

    uv run python scripts/rotate_cube/video_to_wandb.py --checkpoint <run dir>/model_<it>.pt [--held_out]

Runs ``isaaclab play --video --viz newton_gl`` (4 worlds, the training camera of ``rotate_cube.camera``) on the
checkpoint, then logs the newest mp4 under ``videos/play`` of the run directory to the W&B run named in the run's
``reorient_manifest.json`` (resumed by id), as ``video/play`` with the iteration in the caption and ``play_iteration``
in the summary. ``--held_out`` uses the held-out grasp set of the evaluation. For runs trained with ``--no_video``;
the run must have finished (W&B cannot resume a run another process holds). Credentials come from ``~/.netrc``.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import subprocess
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SIM_ROOT = HERE.parents[1]
CAMERA_PY = SIM_ROOT / "src" / "code_isaaclab" / "tasks" / "rotate_cube" / "camera.py"
_spec = importlib.util.spec_from_file_location("rotate_cube_camera", CAMERA_PY)  # by path: the package starts Isaac Lab
_camera = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_camera)
HELD_OUT = SIM_ROOT / "logs" / "rotate_cube" / "heldout_crown1_final4"
NUM_ENVS = 4


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", type=Path, required=True)
    p.add_argument("--task", default="CoDe-Rotate-Cube-O")
    p.add_argument("--video_length", type=int, default=600, help="steps (30 Hz policy: 20 s)")
    p.add_argument("--held_out", action="store_true")
    p.add_argument("--physics", default="newton_mjwarp")
    a = p.parse_args()
    ckpt = a.checkpoint.resolve()
    run_dir = ckpt.parent
    manifest = json.loads((run_dir / "reorient_manifest.json").read_text())
    url = manifest["wandb_run_urls"][0]
    entity, project, run_id = re.search(r"/([^/]+)/([^/]+)/runs/([^/?#]+)", url).groups()
    iteration = int(re.search(r"model_(\d+)\.pt", ckpt.name).group(1))

    eye, lookat = _camera.env0_camera(NUM_ENVS)
    env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD="1")
    for var in ("DISPLAY", "WAYLAND_DISPLAY"):  # offscreen EGL, as in train.py: pyglet rejects headless under X11
        env.pop(var, None)
    for key, value in (manifest.get("task_settings") or {}).items():
        env[key] = str(value)
    if a.held_out:
        env["CODE_ROTATE_CUBE_GRASPS"] = str(HELD_OUT)
    started = time.time()
    command = [
        "uv", "run", "--extra", "rsl-rl", "--extra", "video", "isaaclab", "play", "--rl_library", "rsl_rl",
        "--task", a.task, "--num_envs", str(NUM_ENVS), "--checkpoint", str(ckpt),
        "--video", "--video_length", str(a.video_length), "--viz", "newton_gl",
        f"env.sim.default_visualizer_cfg.eye=[{','.join(map(str, eye))}]",
        f"env.sim.default_visualizer_cfg.lookat=[{','.join(map(str, lookat))}]",
        f"physics={a.physics}",
    ]  # fmt: skip
    status = subprocess.run(command, cwd=SIM_ROOT, env=env).returncode
    clips = [c for c in run_dir.rglob("*.mp4") if "play" in c.parts and c.stat().st_mtime >= started]
    if not clips:
        print(f"no new play clip under {run_dir} (play exit {status})")
        return 1
    clip = max(clips, key=lambda c: c.stat().st_mtime)

    import wandb

    run = wandb.init(entity=entity, project=project, id=run_id, resume="must", dir=str(SIM_ROOT / "logs" / "wandb"))
    grasps = "held-out grasps" if a.held_out else "training grasps"
    run.log({"video/play": wandb.Video(str(clip), format="mp4", caption=f"iteration {iteration}, {grasps}")})
    run.summary["play_iteration"] = iteration
    run.finish()
    print(f"{clip} -> {url}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
