# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3: mirror finished training runs to Weights & Biases, for viewing only.

    uv run --no-project --with wandb --with tensorboard python scripts/study3/wandb_sync.py [--project code-study3]

The local files stay the record (manifests, event files, episode CSVs); nothing here reads back from W&B or
changes a run directory (W&B's local files go to logs/study3_pilot/wandb). For every passed, non-smoke run under logs/rsl_rl/study3_code_reorient it uploads:
- the TensorBoard scalars, at their training iteration;
- a subset of study3_manifest.json (task, variant, seed, overrides, transitions, git commit, versions,
  checkpoint hash) without local paths or command lines;
- the evaluation summaries of that policy (logs/study3_pilot/eval for 3A, eval_ema for 3B'), the first
  evaluation of each (test surface, solver) pair, as summary fields ``eval/<test>/<solver>/<metric>``.

Runs are grouped ``3A`` (no overrides) or ``3B_ema`` (with overrides). Synced runs are listed in
logs/study3_pilot/wandb_synced.json and skipped next time, so the script can be rerun as runs finish.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

SIM_ROOT = Path(__file__).resolve().parents[2]
RUNS = SIM_ROOT / "logs" / "rsl_rl" / "study3_code_reorient"
PILOT = SIM_ROOT / "logs" / "study3_pilot"
LEDGER = PILOT / "wandb_synced.json"
EVAL_ROOTS = {"3A": PILOT / "eval", "3B_ema": PILOT / "eval_ema", "3A_main": SIM_ROOT / "logs" / "study3_main" / "eval"}
CONFIG_KEYS = ("task", "variant", "seed", "physics", "servo_mode", "iterations", "overrides", "num_envs",
               "steps_per_env", "transitions", "git_commit", "git_dirty", "versions", "final_checkpoint_sha256",
               "wall_s", "solver_overflow_messages", "status")
METRICS = ("success_first_goal", "goals_reached_mean", "drop_rate", "valid_fraction", "length_mean")


def evaluations(root: Path, run_dir: Path) -> dict:
    """First evaluation of this policy per (test surface, solver), keyed by the evaluation run's timestamp."""
    found = {}
    for path in sorted(root.glob("*/summary.json")):
        data = json.loads(path.read_text())
        if Path(data["train_run"]).name != run_dir.name:
            continue
        key = (data["test_variant"], data["physics"])
        stamp = data["run"].split("_")[-2]
        if key not in found or stamp < found[key][0]:
            found[key] = (stamp, data)
    return {f"eval/{t}/{p.removeprefix('newton_')}/{m}": d[m] for (t, p), (_, d) in found.items() for m in METRICS}


def scalars(run_dir: Path) -> dict[int, dict[str, float]]:
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

    acc = EventAccumulator(str(next(run_dir.glob("events.out.tfevents.*"))), size_guidance={"scalars": 0})
    acc.Reload()
    steps: dict[int, dict[str, float]] = {}
    for tag in acc.Tags()["scalars"]:
        for event in acc.Scalars(tag):
            steps.setdefault(event.step, {})[tag] = event.value
    return steps


def main(argv=None) -> int:
    import wandb

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--project", default="code-study3")
    parser.add_argument("--group", choices=tuple(EVAL_ROOTS), help="sync only this group (e.g. once its evaluations exist)")
    parser.add_argument("--dry_run", action="store_true")
    args = parser.parse_args(argv)
    done = json.loads(LEDGER.read_text()) if LEDGER.exists() else {}
    for manifest_path in sorted(RUNS.glob("*/study3_manifest.json")):
        run_dir, manifest = manifest_path.parent, json.loads(manifest_path.read_text())
        if "smoke" in run_dir.name or manifest.get("status") != "passed" or run_dir.name in done:
            continue
        group = ("3A_main" if manifest.get("protocol") == "main"
                 else "3B_ema" if manifest.get("overrides") else "3A")
        if args.group and group != args.group:
            continue
        evals = evaluations(EVAL_ROOTS[group], run_dir)
        if args.dry_run:
            print(run_dir.name, group, len(evals))
            continue
        run = wandb.init(project=args.project, name=run_dir.name.split("_", 2)[2], group=group,
                         job_type=manifest["variant"], tags=[group, manifest["variant"]],
                         config={k: manifest.get(k) for k in CONFIG_KEYS} | {"run_dir": run_dir.name},
                         reinit="finish_previous", dir=str(PILOT),
                         # no host, account e-mail, paths or system stats; the git commit is in the config
                         settings=wandb.Settings(x_disable_meta=True, disable_git=True, save_code=False,
                                                 x_disable_stats=True))
        for step, values in sorted(scalars(run_dir).items()):
            run.log(values, step=step)
        run.summary.update(evals)
        done[run_dir.name] = run.url
        run.finish()
        LEDGER.write_text(json.dumps(done, indent=1) + "\n")
        print(run_dir.name, group, run.url)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
