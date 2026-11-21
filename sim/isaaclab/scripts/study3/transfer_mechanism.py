# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3A: why flat-trained policies lose first-goal success on crowned pads (a test of one hypothesis).

    uv run python scripts/study3/transfer_mechanism.py [--root logs/study3_pilot/eval_traces]

Hypothesis (from the prescribed abduction before training: the same finger motion turns the cube 5-13 times
more on crowned pads): a policy tuned on flat pads turns the cube too far on crowned ones. Prediction: on
crowned pads its failed episodes overshoot the first goal (the cube passes beyond it by more than the 0.2 rad
success band) more often, and undershoot no more often, than on flat pads; and the cube turns further in the
first second.

For each flat-trained policy, every episode of its two evaluations (same initial states, ``--trace_all``) is
classified by the cube's progress toward the first goal before it is reached (yaw times the goal's sign):
overshoot (peak progress beyond goal + 0.2 rad), undershoot (peak short of goal - 0.2 rad), or neither. Paired
over seeds with exact sign-flip tests. Writes ``report/study3/tables/transfer_mechanism.csv``.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

BAND = 0.2


def classify(root: Path) -> list[dict]:
    rows = []
    for summary_path in sorted(root.glob("*/summary.json")):
        s = json.loads(summary_path.read_text())
        if s["train_variant"] != "flat":
            continue
        npz = np.load(summary_path.parent / "trajectories.npz")
        yaw = npz["yaw"]  # (steps, episodes), NaN after the episode ended
        with (summary_path.parent / "episodes.csv").open() as handle:
            episodes = list(csv.DictReader(handle))
        first_second = int(round(1.0 / float(npz["dt"])))
        counts = {"overshoot": 0, "undershoot": 0, "neither": 0}
        early_turn, n_fail = [], 0
        for i, e in enumerate(episodes):
            goal = float(e["first_goal_rad"])
            stop = int(float(e["steps_to_first_goal"])) if e["success_first_goal"] == "1" else int(e["length"])
            progress = yaw[:max(stop, 1), i] * np.sign(goal)
            progress = progress[np.isfinite(progress)]
            early = yaw[:first_second, i]
            early = early[np.isfinite(early)]
            if len(early):
                early_turn.append(float(np.max(np.abs(early))))
            if e["success_first_goal"] == "1" or not len(progress):
                continue
            n_fail += 1
            peak = float(progress.max())
            key = "overshoot" if peak > abs(goal) + BAND else "undershoot" if peak < abs(goal) - BAND else "neither"
            counts[key] += 1
        n = len(episodes)
        rows.append({"seed": s["train_seed"], "test_variant": s["test_variant"], "episodes": n,
                     "success_first_goal": s["success_first_goal"], "failures": n_fail,
                     "overshoot_rate": counts["overshoot"] / n, "undershoot_rate": counts["undershoot"] / n,
                     "neither_rate": counts["neither"] / n,
                     "max_abs_yaw_first_second_rad": float(np.mean(early_turn)), "eval_run": s["run"]})
    return rows


def sign_flip_p(values: list[float]) -> float:
    observed = abs(np.mean(values))
    flips = [abs(np.mean([v * s for v, s in zip(values, signs)])) for signs in itertools.product((1, -1), repeat=len(values))]
    return float(np.mean([f >= observed - 1e-12 for f in flips]))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--root", type=Path, default=common.LOG_ROOT / "eval_traces")
    args = parser.parse_args(argv)
    rows = classify(args.root)
    out = common.REPO_ROOT / "report" / "study3" / "tables" / "transfer_mechanism.csv"
    with out.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    by = {(r["seed"], r["test_variant"]): r for r in rows}
    seeds = sorted({r["seed"] for r in rows})
    print(f"{'seed':>4s} {'surface':8s} {'success':>8s} {'overshoot':>10s} {'undershoot':>11s} {'neither':>8s} {'yaw 1 s':>8s}")
    for seed in seeds:
        for v in ("flat", "crowned"):
            r = by[(seed, v)]
            print(f"{seed:>4d} {v:8s} {r['success_first_goal']:8.3f} {r['overshoot_rate']:10.3f} {r['undershoot_rate']:11.3f} "
                  f"{r['neither_rate']:8.3f} {r['max_abs_yaw_first_second_rad']:8.3f}")
    for key in ("success_first_goal", "overshoot_rate", "undershoot_rate", "max_abs_yaw_first_second_rad"):
        diffs = [by[(s, "crowned")][key] - by[(s, "flat")][key] for s in seeds]
        print(f"crowned - flat {key:30s} {np.mean(diffs):+.3f} +- {np.std(diffs, ddof=1):.3f}  "
              f"({sum(d > 0 for d in diffs)}/{len(diffs)} positive, sign-flip p {sign_flip_p(diffs):.4f})")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
