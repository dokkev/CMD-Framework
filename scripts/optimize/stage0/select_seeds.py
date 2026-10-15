#!/usr/bin/env python3
"""Verify Stage-0 Pareto candidates on a fine PIP grid and mark the seeds.

    python scripts/optimize/stage0/select_seeds.py output/stage0/run_001 output/stage0/run_002

Each candidate is re-evaluated with ``--step-deg`` PIP samples (the optimizer
uses a coarse grid, so a condition can fail between samples). The verified,
mutually non-dominated candidates are written with a ``seed_role``
(``docs/optimization/stage0.md``):

- ``best_transmission``: the smallest ``k_max``;
- ``best_length``: the largest ``OA + BC``;
- ``best_curl``: the largest ``phi_safe_max``;
- ``representative``: the largest ``OA + BC`` within ``--k-tolerance`` of the
  best ``k_max`` (a marker only, no filter);
- ``balanced``: the closest to the ideal point with each objective scaled to
  ``[0, 1]`` over the verified front (the compromise of all three);
- ``pareto``: every other verified non-dominated candidate.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import sys
from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.stage0 import VARIABLE_NAMES, decode, evaluate  # noqa: E402
from core.optimization.stage0.run import load_config, objective_1, objectives  # noqa: E402


DEFAULT_OUTPUT = REPOSITORY_ROOT / "output" / "stage0" / "seeds.csv"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path, help="Stage-0 run directories")
    parser.add_argument("--step-deg", type=float, default=0.05)
    parser.add_argument("--dip-tolerance-deg", type=float, default=0.1,
                        help="DIP and driven-link backoff allowed in verification")
    parser.add_argument("--k-tolerance", type=float, default=0.05,
                        help="relative k_max excess over the best for the representative seed")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)

    rows = []
    print("run                 pareto  verified")
    for run in args.runs:
        config = dataclasses.replace(
            load_config(run), pip_step_rad=math.radians(args.step_deg),
            dip_backoff_tolerance_rad=math.radians(args.dip_tolerance_deg),
        )
        with (run / "pareto.csv").open(encoding="utf-8", newline="") as stream:
            candidates = list(csv.DictReader(stream))
        verified = 0
        for row in candidates:
            x = [float(row[name]) for name in VARIABLE_NAMES]
            e = evaluate(decode(x), config)
            if not e.feasible:
                continue
            verified += 1
            rows.append({
                "run": f"{run.parent.name}/{run.name}", "candidate": row["candidate"],
                **dict(zip(VARIABLE_NAMES, x)),
                "objectives": objectives(e, config),
                "objective_1": objective_1(e, config),
                "torque_ratio_max": e.torque_ratio_max,
                "structural_length_mm": e.structural_length_mm,
                "phi_safe_deg": math.degrees(e.phi_safe_rad),
                "usable_pip_deg": math.degrees(e.usable_pip_rad),
                "stop_reason": e.stop_reason,
                "min_margin_deg": math.degrees(e.min_margin_rad),
                "servo_travel_deg": math.degrees(e.servo_travel_rad),
                "lever_max_mm": e.lever_max_mm,
                "ab_mm": e.ab_mm,
                "curl_tip_x_mm": e.curl_tip_x_mm,
                "servo_lowest_z_mm": e.servo_lowest_z_mm,
            })
        print(f"{run.parent.name + '/' + run.name:19s} {len(candidates):6d}  {verified:8d}")

    # Keep the verified candidates no other verified candidate dominates.
    def dominates(first, second):
        return all(a <= b for a, b in zip(first, second)) and first != second

    rows = [row for row in rows
            if not any(dominates(other["objectives"], row["objectives"]) for other in rows)]
    for row in rows:
        row["seed_role"] = "pareto"
    if rows:
        best_k = min(row["objective_1"] for row in rows)
        scores = np.array([row["objectives"] for row in rows])
        low, high = scores.min(axis=0), scores.max(axis=0)
        scaled = (scores - low) / np.where(high > low, high - low, 1.0)
        roles = {
            "balanced": rows[int(np.argmin(np.linalg.norm(scaled, axis=1)))],
            "representative": max(
                (row for row in rows if row["objective_1"] <= best_k * (1.0 + args.k_tolerance)),
                key=lambda row: row["structural_length_mm"]),
            "best_curl": max(rows, key=lambda row: (row["phi_safe_deg"], -row["objective_1"])),
            "best_length": max(rows, key=lambda row: (row["structural_length_mm"], -row["objective_1"])),
            "best_transmission": min(rows, key=lambda row: row["objective_1"]),
        }
        for role, row in roles.items():  # later roles take precedence on one design
            row["seed_role"] = role
    order = {"best_transmission": 0, "best_length": 1, "best_curl": 2, "representative": 3, "balanced": 4,
             "pareto": 5}
    rows.sort(key=lambda row: (order[row["seed_role"]], row["objective_1"]))
    for row in rows:
        row.pop("objectives")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        fields = ["seed_role", *[key for key in (rows[0] if rows else {"run": 0}) if key != "seed_role"]]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"\n{len(rows)} verified non-dominated seeds -> {args.output}")
    for row in rows[:8]:
        print(
            f"  {row['seed_role']:17s} {row['run']} #{row['candidate']:>3s}: "
            f"k {row['torque_ratio_max']:.2f}, OA+BC {row['structural_length_mm']:.1f} mm, "
            f"phi_safe {row['phi_safe_deg']:.0f} deg ({row['stop_reason']}), "
            f"margin {row['min_margin_deg']:.0f} deg, servo travel {row['servo_travel_deg']:.0f} deg"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
