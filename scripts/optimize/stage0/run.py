#!/usr/bin/env python3
"""Run the Stage-0 NSGA-II linkage seed study and save its feasible Pareto set.

See ``docs/optimization/stage0.md``. Objectives: the worst torque ratio
``k_max``, ``-(OA + BC)`` and ``180 deg - phi_safe_max``.

Edit ``config.yaml`` next to this script (or pass another with ``--config``),
then run, e.g.

    python scripts/optimize/stage0/run.py --seed 0

Each run writes ``run.json`` and ``pareto.csv`` to a new ``output/stage0/run_NNN``;
``select_seeds.py`` verifies them and marks the seeds.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import subprocess
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.stage0 import read_design  # noqa: E402
from core.optimization.stage0.config import load_yaml  # noqa: E402
from core.optimization.stage0.run import run_study, save_result  # noqa: E402


OUTPUT_ROOT = REPOSITORY_ROOT / "output" / "stage0"
DEFAULT_CONFIG = Path(__file__).with_name("config.yaml")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                        help="study settings (default: %(default)s)")
    parser.add_argument("--seed", type=int, help="NSGA-II seed (default: the config's)")
    parser.add_argument("--warm-start", type=Path, nargs="+", default=(), metavar="CSV",
                        help="pareto.csv or seeds.csv files whose designs seed up to half the population")
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    args = parser.parse_args(argv)
    config = load_yaml(args.config)
    if args.seed is not None:
        config = dataclasses.replace(config, seed=args.seed)

    initial = [
        read_design(row)
        for path in args.warm_start
        for row in csv.DictReader(path.open(encoding="utf-8", newline=""))
    ]
    result = run_study(config, initial_designs=initial)
    output = save_result(result, _next_run_directory(args.output_root), source_git_commit=_git_commit())
    print(f"Evaluations: {result.evaluation_count}")
    print(f"Feasible Pareto candidates: {len(result.pareto)}")
    for index, candidate in enumerate(result.pareto[:8], start=1):
        e = candidate.evaluation
        print(
            f"  {index}: k {e.torque_ratio_max:.2f}, OA+BC {e.structural_length_mm:.1f} mm, "
            f"phi_safe {math.degrees(e.phi_safe_rad):.0f} deg ({e.stop_reason}), "
            f"margin {math.degrees(e.min_margin_rad):.0f} deg, "
            f"servo travel {math.degrees(e.servo_travel_rad):.0f} deg"
        )
    print(f"Output: {output}")
    return 0


def _next_run_directory(root: Path) -> Path:
    index = 1
    while (root / f"run_{index:03d}").exists():
        index += 1
    return root / f"run_{index:03d}"


def _git_commit() -> str | None:
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPOSITORY_ROOT,
            check=True, capture_output=True, text=True, timeout=3.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return completed.stdout.strip() or None


if __name__ == "__main__":
    raise SystemExit(main())
