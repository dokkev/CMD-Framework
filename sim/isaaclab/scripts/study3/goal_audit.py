# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3: does a goal count measure rotation? Audit of every goal reached in the traced episodes.

    python3 scripts/study3/goal_audit.py [--root logs/study3_pilot/eval_ema] [--threshold_rad 0.2]

A goal is reached whenever the orientation error falls below the threshold eps, on any step, and the next goal
is drawn at once at least ``min_step`` from the reached one (no dwell). When ``min_step < 2 eps`` the two goals'
success regions overlap, so the cube can sit inside the next goal's region at the moment it is drawn and
reach it with no rotation. From the traced episodes (``trajectories.npz``: cube yaw and goal angle per step)
this reports, per evaluation cell and over all goals reached:

- ``step_deg``: |next goal - reached goal|;
- ``inside_at_switch``: the cube's yaw already within eps of the next goal when it is drawn (necessary for a
  free goal; the orientation error also counts tilt, so this is an upper bound on free goals);
- ``next_within_2``: the next goal reached within 2 steps of the previous one;
- ``rotation_deg``: |yaw change| between consecutive reached goals (the first from the start yaw);
- ``required_deg``: max(0, |next goal - yaw at switch| - eps), the least rotation the next goal needs.

Writes report/study3/tables/goal_audit.csv. Reads saved traces only.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

SIM_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = SIM_ROOT.parent.parent


def events(yaw: np.ndarray, goal: np.ndarray, eps: float) -> list[dict]:
    """Goals reached in one traced episode, from its per-step yaw and goal angle [rad]."""
    out = []
    live = ~np.isnan(yaw)
    last_t, last_yaw = None, None
    for t in range(1, len(goal)):
        if not (live[t] and live[t - 1]) or goal[t] == goal[t - 1]:
            continue
        reached, nxt, y = goal[t - 1], goal[t], yaw[t]
        out.append({
            "t": t,
            "step_deg": np.degrees(abs(nxt - reached)),
            "inside_at_switch": abs(nxt - y) < eps,
            "rotation_deg": np.degrees(abs(y - (last_yaw if last_yaw is not None else yaw[0]))),
            "gap_steps": t - (last_t if last_t is not None else 0),
            "required_deg": np.degrees(max(0.0, abs(nxt - y) - eps)),
        })
        last_t, last_yaw = t, y
    for a, b in zip(out, out[1:]):
        a["next_within_2"] = b["gap_steps"] <= 2
    if out:
        out[-1]["next_within_2"] = False
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", default="logs/study3_pilot/eval_ema")
    parser.add_argument("--threshold_rad", type=float, default=0.2)
    parser.add_argument("--physics", default="newton_mjwarp")
    parser.add_argument("--out", default=str(REPO_ROOT / "report" / "study3" / "tables" / "goal_audit.csv"))
    args = parser.parse_args(argv)
    rows = []
    for summary in sorted((SIM_ROOT / args.root).glob("*/summary.json")):
        s = json.loads(summary.read_text())
        if s["physics"] != args.physics:
            continue
        tr = np.load(summary.parent / "trajectories.npz")
        ev = [e for k in range(tr["yaw"].shape[1]) for e in events(tr["yaw"][:, k], tr["goal"][:, k], args.threshold_rad)]
        if not ev:
            continue
        col = lambda k: np.array([float(e[k]) for e in ev])  # noqa: E731
        rows.append({
            "cell": s["run"].rsplit("_", 2)[0], "train_variant": s["train_variant"], "train_seed": s["train_seed"],
            "test_variant": s["test_variant"], "physics": s["physics"], "traced_episodes": tr["yaw"].shape[1],
            "goals": len(ev), "goals_per_episode": len(ev) / tr["yaw"].shape[1],
            "step_deg_min": col("step_deg").min(), "inside_at_switch": col("inside_at_switch").mean(),
            "next_within_2": col("next_within_2").mean(), "gap_steps_median": np.median(col("gap_steps")),
            "rotation_deg_median": np.median(col("rotation_deg")), "rotation_lt_2deg": (col("rotation_deg") < 2).mean(),
            "rotation_lt_8_5deg": (col("rotation_deg") < 8.54).mean(),
            "required_deg_median": np.median(col("required_deg")),
        })
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    keys = ("goals_per_episode", "inside_at_switch", "next_within_2", "gap_steps_median", "rotation_deg_median",
            "rotation_lt_2deg", "rotation_lt_8_5deg", "required_deg_median")
    print("cell".ljust(34), *[k[:12].rjust(12) for k in keys])
    for r in rows:
        print(r["cell"][:34].ljust(34), *[f"{float(r[k]):12.2f}" for k in keys])
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
