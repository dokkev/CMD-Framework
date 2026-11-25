# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Play a checkpoint on any task and log the clip to a new W&B run.

    uv run --extra wandb python scripts/rotate_cube/play_to_wandb.py --checkpoint <model.pt> --task <task> \\
        --name <run name> --group <group> [--video_length 600] [--held_out <grasp dir>]

``video_to_wandb.py`` adds a clip to the checkpoint's own training run; this logs to a run of its own (project
``code-hand-inhand``), for policies evaluated on tasks they were not trained on (the transfer study). The play itself is
``video_watcher.play`` (4 worlds, the training camera, held-out grasps). Environment variables (e.g.
``CODE_CONTACT_PAD_MU``) pass through to the play. W&B credentials come from ``~/.netrc``.
"""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("video_watcher", HERE / "video_watcher.py")
_vw = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_vw)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--checkpoint", type=Path, required=True)
    p.add_argument("--task", required=True)
    p.add_argument("--name", required=True)
    p.add_argument("--group", required=True)
    p.add_argument("--video_length", type=int, default=600)
    p.add_argument("--held_out", default=str(_vw.SIM_ROOT / "logs" / "rotate_cube" / "heldout_omnireset_v1"))
    p.add_argument("--physics", default="newton_mjwarp")
    a = p.parse_args()
    ckpt = a.checkpoint.resolve()
    (ckpt.parent / "videos").mkdir(exist_ok=True)
    clip = _vw.play(ckpt.parent, ckpt, a.task, a.video_length, a.held_out, a.physics)
    if clip is None:
        return 1
    import wandb

    run = wandb.init(project=_vw.PROJECT, name=a.name, group=a.group, job_type="videos",
                     dir=str(_vw.SIM_ROOT / "logs" / "wandb"),
                     config={"checkpoint": str(ckpt), "task": a.task, "held_out": a.held_out})  # fmt: skip
    run.log({"video/play": wandb.Video(str(clip), format="mp4", caption=f"{a.name}: {a.task}")})
    print(f"{clip} -> {run.url}")
    run.finish()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
