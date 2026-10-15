#!/usr/bin/env python3
"""Verify finger study Pareto candidates on a fine PIP grid and rank them.

    python scripts/optimize/select_finger.py outputs/finger/run_001 outputs/finger/run_002

Each candidate is re-evaluated with ``--step-deg`` PIP samples (the optimizer
uses a coarse grid, so a constraint can be violated between samples).
Verified candidates are ranked by the study priorities: the first objective
(worst transmission quality, or worst lever ``r`` for ``objective="lever"``
runs), then the smallest AB among those within the tolerance of its best.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from cmd.optimization.finger import VARIABLE_NAMES, decode, evaluate  # noqa: E402
from cmd.optimization.finger.run import load_config  # noqa: E402


DEFAULT_OUTPUT = REPOSITORY_ROOT / "outputs" / "finger" / "shortlist.csv"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path, help="finger study run directories")
    parser.add_argument("--step-deg", type=float, default=0.05)
    parser.add_argument("--dip-tolerance-deg", type=float, default=0.1,
                        help="DIP and driven-link backoff allowed in verification")
    parser.add_argument("--quality-tolerance", type=float, default=0.02,
                        help="quality shortfall from the best still preferred for a smaller AB")
    parser.add_argument("--lever-tolerance", type=float, default=0.05,
                        help="relative lever excess over the best still preferred for a smaller AB")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)

    rows = []
    objectives = set()
    pareto_studies = set()
    print("run                 pareto  verified")
    for run in args.runs:
        config = dataclasses.replace(
            load_config(run), pip_step_rad=math.radians(args.step_deg),
            dip_backoff_tolerance_rad=math.radians(args.dip_tolerance_deg),
        )
        objectives.add(config.objective)
        pareto_studies.add(config.minimize_ab)
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
                "run": f"{run.parent.name}/{run.name}", "candidate": row["candidate"], **dict(zip(VARIABLE_NAMES, x)),
                "worst_transmission_quality": e.worst_transmission_quality,
                "worst_margin_deg": math.degrees(math.asin(e.worst_transmission_quality)),
                "ab_mm": e.ab_mm,
                "curl_pip_deg": math.degrees(e.curl_pip_rad),
                "curl_dip_deg": math.degrees(e.curl_dip_rad),
                "curl_gap_mm": e.curl_gap_mm,
                "curl_tip_x_mm": e.curl_tip_x_mm,
                "link_clearance_mm": e.link_clearance_mm,
                "servo_lowest_z_mm": e.servo_lowest_z_mm,
                "lever_max_mm": e.lever_max_mm,
                "torque_ratio_max": e.torque_ratio_max,
                "servo_travel_deg": math.degrees(e.servo_travel_rad),
            })
        print(f"{run.parent.name + '/' + run.name:19s} {len(candidates):6d}  {verified:8d}")

    if len(objectives) > 1:
        raise ValueError(f"runs mix objectives {sorted(objectives)}")
    if objectives & {"lever", "torque_ratio"}:
        column = "lever_max_mm" if objectives == {"lever"} else "torque_ratio_max"

        def score(row):  # smaller is better
            return row[column]
        best = min((score(row) for row in rows), default=0.0)
        within = best * (1.0 + args.lever_tolerance)
    else:
        def score(row):
            return -row["worst_transmission_quality"]
        best = min((score(row) for row in rows), default=0.0)
        within = best + args.quality_tolerance
    if objectives == {"torque_ratio"} and pareto_studies == {False}:
        # The torque ratio is the only objective; AB only breaks ties.
        rows.sort(key=lambda row: (score(row), row["ab_mm"]))
    else:
        rows.sort(key=lambda row: (score(row) > within,
                                   row["ab_mm"] if score(row) <= within else score(row)))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["run", "candidate"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"\n{len(rows)} verified candidates -> {args.output}")
    for row in rows[:8]:
        print(
            f"  {row['run']} #{row['candidate']:>3s}: quality {row['worst_transmission_quality']:.3f} "
            f"(margin {row['worst_margin_deg']:.0f} deg), AB {row['ab_mm']:.1f} mm, "
            f"lever {row['lever_max_mm']:.1f} mm, k {row['torque_ratio_max']:.2f}, "
            f"servo travel {row['servo_travel_deg']:.0f} deg, "
            f"curl PIP {row['curl_pip_deg']:.0f} / DIP {row['curl_dip_deg']:.0f} deg, "
            f"gap {row['curl_gap_mm']:.1f} mm, link {row['link_clearance_mm']:+.1f} mm, "
            f"servo lowest z {row['servo_lowest_z_mm']:+.1f} mm"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
