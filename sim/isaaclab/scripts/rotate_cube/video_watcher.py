# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Play clips of checkpoints while a run trains, logged to a W&B run of their own.

    uv run --extra wandb python scripts/rotate_cube/video_watcher.py --runs 'logs/rsl_rl/<experiment>/*_<label>*' \\
        [--task CoDe-Rotate-Cube-O-OmniReset-Eval] [--every 100] [--until_file <driver.log>:DRIVER_DONE]

``video_to_wandb.py`` adds a clip to a finished training run; a run still training holds its W&B run, so this job
logs to ``<run dir name>_videos`` instead (same project, ``code-hand-inhand``), grouped with the training run (the
training run's group is set to the same name through the W&B API). For every run directory matching ``--runs``, each
``model_<it>.pt`` with ``it`` a multiple of ``--every`` (and each run's last checkpoint once its manifest exists) is
played with ``isaaclab play`` (4 worlds, the training camera, ``--video_length`` steps, held-out grasps:
``CODE_ROTATE_CUBE_GRASPS``) and logged as ``video/play`` at step ``it`` with the iteration in the caption. It waits
while the GPU has less than ``--min_free_mib`` free, so the training run is not disturbed. Done checkpoints are
recorded in ``<run dir>/videos/watcher.json``. It exits when ``--until_file`` contains its marker and nothing is left.
W&B credentials come from ``~/.netrc``; nothing here reads or prints them.
"""

from __future__ import annotations

import argparse
import glob
import importlib.util
import json
import os
import re
import secrets
import string
import subprocess
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SIM_ROOT = HERE.parents[1]
CAMERA_PY = SIM_ROOT / "src" / "code_isaaclab" / "tasks" / "rotate_cube" / "camera.py"
_spec = importlib.util.spec_from_file_location("rotate_cube_camera", CAMERA_PY)  # by path: the package starts Isaac Lab
_camera = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_camera)
NUM_ENVS = 4
PROJECT = "code-hand-inhand"


def free_mib() -> int:
    out = subprocess.run(["nvidia-smi", "--query-gpu=memory.free", "--format=csv,noheader,nounits"],
                         capture_output=True, text=True).stdout  # fmt: skip
    return int(out.split()[0]) if out.strip() else 0


def training_run(run_dir: Path) -> tuple[str, str, str] | None:
    """``(entity, project, run id)`` of the training run, from its console log (``train.py``)."""
    log = SIM_ROOT / "logs" / f"train_{run_dir.name}.log"
    if not log.exists():
        return None
    m = re.search(r"https://\S+/([^/\s]+)/([^/\s]+)/runs/(\w+)", log.read_text(errors="replace"))
    return m.groups() if m else None


def play(run_dir: Path, ckpt: Path, task: str, length: int, held_out: str, physics: str) -> Path | None:
    eye, lookat = _camera.env0_camera(NUM_ENVS)
    env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD="1", CODE_ROTATE_CUBE_GRASPS=held_out)
    for var in ("DISPLAY", "WAYLAND_DISPLAY"):
        env.pop(var, None)
    started = time.time()
    command = [
        "uv", "run", "--extra", "rsl-rl", "--extra", "video", "isaaclab", "play", "--rl_library", "rsl_rl",
        "--task", task, "--num_envs", str(NUM_ENVS), "--checkpoint", str(ckpt),
        "--video", "--video_length", str(length), "--viz", "newton_gl",
        f"env.sim.default_visualizer_cfg.eye=[{','.join(map(str, eye))}]",
        f"env.sim.default_visualizer_cfg.lookat=[{','.join(map(str, lookat))}]",
        f"physics={physics}",
    ]  # fmt: skip
    with open(run_dir / "videos" / "watcher_play.log", "a") as log:
        status = subprocess.run(command, cwd=SIM_ROOT, env=env, stdout=log, stderr=subprocess.STDOUT).returncode
    clips = [c for c in run_dir.rglob("*.mp4") if "play" in c.parts and c.stat().st_mtime >= started]
    if status != 0 or not clips:
        print(f"play failed for {ckpt} (exit {status})", flush=True)
        return None
    return max(clips, key=lambda c: c.stat().st_mtime)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--runs", required=True, help="glob of training run directories")
    p.add_argument("--task", default="CoDe-Rotate-Cube-O-OmniReset-Eval")
    p.add_argument("--every", type=int, default=100)
    p.add_argument("--video_length", type=int, default=600, help="steps (30 Hz policy: 20 s)")
    p.add_argument("--held_out", default=str(SIM_ROOT / "logs" / "rotate_cube" / "heldout_omnireset_v1"))
    p.add_argument("--physics", default="newton_mjwarp")
    p.add_argument("--min_free_mib", type=int, default=2500)
    p.add_argument("--until_file", default="", help="<path>:<marker>: exit once the file contains the marker")
    p.add_argument("--poll_s", type=float, default=60.0)
    p.add_argument("--group", default=None,
                   help="W&B group of the training and video runs (default: the training run's directory name)")
    a = p.parse_args()

    import wandb

    api = wandb.Api()
    while True:
        pending = 0
        for run_dir in sorted(Path(d) for d in glob.glob(str(SIM_ROOT / a.runs)) if Path(d).is_dir()):
            ids = training_run(run_dir)
            if ids is None:
                continue
            entity, project, run_id = ids
            (run_dir / "videos").mkdir(exist_ok=True)
            state_path = run_dir / "videos" / "watcher.json"
            state = json.loads(state_path.read_text()) if state_path.exists() else {"done": []}
            ckpts = {int(m.group(1)): c for c in run_dir.glob("model_*.pt")
                     if (m := re.match(r"model_(\d+)\.pt", c.name))}
            finished = (run_dir / "reorient_manifest.json").exists()
            wanted = [it for it in ckpts if it % a.every == 0 or (finished and it == max(ckpts))]
            todo = sorted(it for it in wanted if it not in state["done"])
            pending += len(todo) + (0 if finished else 1)
            for it in todo:
                while free_mib() < a.min_free_mib:
                    time.sleep(a.poll_s)
                time.sleep(5.0)  # the checkpoint file may still be being written
                clip = play(run_dir, ckpts[it], a.task, a.video_length, a.held_out, a.physics)
                if clip is None:
                    state["done"].append(it)  # do not retry forever
                    state_path.write_text(json.dumps(state, indent=1))
                    continue
                group = a.group or run_dir.name
                try:  # group the training run with its video run
                    tr = api.run(f"{entity}/{project}/{run_id}")
                    if tr.group != group:
                        tr.group = group
                        tr.update()
                except Exception as e:  # noqa: BLE001 (a busy run may refuse the update; the videos still go up)
                    print(f"could not set the group of {run_id}: {type(e).__name__}", flush=True)
                if not state.get("video_run_id"):  # 8 lowercase letters and digits, as train.py draws run ids
                    state["video_run_id"] = "".join(
                        secrets.choice(string.ascii_lowercase + string.digits) for _ in range(8))
                run = wandb.init(entity=entity, project=PROJECT, id=state["video_run_id"], resume="allow",
                                 name=f"{run_dir.name}_videos", group=group, job_type="videos",
                                 dir=str(SIM_ROOT / "logs" / "wandb"),
                                 config={"training_run": f"{entity}/{project}/{run_id}", "task": a.task,
                                         "held_out": a.held_out})  # fmt: skip
                run.log({"video/play": wandb.Video(str(clip), format="mp4", caption=f"iteration {it}, held-out grasps"),
                         "iteration": it}, step=it)  # fmt: skip
                state["video_run_url"] = run.url
                run.finish()
                state["done"].append(it)
                state_path.write_text(json.dumps(state, indent=1))
                print(f"{run_dir.name} iteration {it}: {clip.name} -> {state['video_run_url']}", flush=True)
        if a.until_file:
            path, marker = a.until_file.rsplit(":", 1)
            if pending == 0 and Path(path).exists() and marker in Path(path).read_text():
                return 0
        time.sleep(a.poll_s)


if __name__ == "__main__":
    raise SystemExit(main())
