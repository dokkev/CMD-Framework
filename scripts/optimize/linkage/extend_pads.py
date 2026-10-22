#!/usr/bin/env python3
"""Lengthen the proximal pads of selected linkage seeds past A', for free.

    python scripts/optimize/linkage/extend_pads.py output/linkage/production_v1/portfolio_001/seeds.csv

Each representative seed (``--all``: every seed) gets the longest
``app_offset_mm`` (A''_x - A'_x) that keeps it feasible with the same k_max and
phi_safe, verified on the fine grid (``core/optimization/linkage/extend.py``);
``--extra-clearance-mm`` keeps every pad that much beyond the body clearance.
Writes ``seeds_extended.csv`` next to the input.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

import csv  # noqa: E402

from core.optimization.linkage import MECHANISM  # noqa: E402
from core.optimization.linkage.extend import longest_proximal_pad  # noqa: E402
from core.optimization.study import load_run, write_rows  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("seeds", type=Path, help="seeds.csv of select_seeds.py or run.py --portfolio")
    parser.add_argument("--all", action="store_true", help="every seed, not only the representative ones")
    parser.add_argument("--extra-clearance-mm", type=float, default=0.0)
    parser.add_argument("--step-deg", type=float, default=0.05, help="motion step it is verified on")
    parser.add_argument("--runs-root", type=Path,
                        help="where the seeds' runs are (default: two levels above the CSV's folder)")
    parser.add_argument("--output", type=Path, help="default: seeds_extended.csv next to the input")
    args = parser.parse_args(argv)

    root = args.runs_root or args.seeds.resolve().parent.parent.parent
    with args.seeds.open(encoding="utf-8", newline="") as stream:
        rows = [row for row in csv.DictReader(stream) if args.all or row["seed_role"] != "pareto"]
    configs: dict[str, object] = {}
    extended_rows = []
    print(f"{'role':28s} {'run':22s} {'#':>4s}  {'added':>6s}  {'proximal pad':>16s}  "
          f"{'pad clr':>15s}  k_max   phi_safe")
    for row in rows:
        if row["run"] not in configs:
            run = load_run(root / row["run"])
            if run.mechanism is not MECHANISM:
                parser.error(f"{row['run']} is not a linkage run")
            configs[row["run"]] = MECHANISM.refine(run.config, math.radians(args.step_deg))
        config = configs[row["run"]]
        before = MECHANISM.evaluate(MECHANISM.read_design(row), config)
        result = longest_proximal_pad(MECHANISM.read_design(row), config,
                                      extra_clearance_mm=args.extra_clearance_mm)
        e = result.evaluation
        extended_rows.append({**row, **dict(zip(MECHANISM.variable_names, result.x)),
                              **MECHANISM.report(e), "added_offset_mm": result.added_mm})
        print(f"{row['seed_role']:28s} {row['run']:22s} {row['candidate']:>4s}  {result.added_mm:5.1f}  "
              f"{before.proximal_support_length_mm:6.1f} -> {e.proximal_support_length_mm:5.1f}  "
              f"{before.pad_clearance_mm:5.2f} -> {e.pad_clearance_mm:5.2f}  "
              f"{e.torque_ratio_max:5.2f}  {math.degrees(e.phi_safe_rad):6.1f}°")
    output = args.output or args.seeds.with_name("seeds_extended.csv")
    write_rows(output, extended_rows, ("seed_role", "run", "candidate", "added_offset_mm"))
    print(f"-> {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
