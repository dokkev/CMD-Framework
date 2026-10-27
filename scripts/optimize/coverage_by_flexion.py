#!/usr/bin/env python3
"""The most effective pad coverage each mechanism keeps up to each flexion.

    python scripts/optimize/coverage_by_flexion.py --linkage output/linkage/flex90 output/linkage/finger90_100 \\
        --revolute output/revolute/flex90 --output output/comparison/coverage_by_flexion.csv

Every saved candidate of the given studies (their ``run_*``) is re-evaluated
with the required flexion set to each target ``phi`` (``--from`` to 180 deg,
``--step``), on the verification grid: a pad's effective length is then the
face the links leave free on ``[0, phi]``, and the design must keep the
motion safe to ``phi``. The table is each mechanism's best feasible design
per target: a lower bound of what an optimization at that target would
reach. All studies must share one goal but for the required flexion and the
search settings.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import multiprocessing
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.study import load_run  # noqa: E402

_JOB: dict = {}


def _goal(config):
    return getattr(config, "goal", config)


def _with_flexion(mechanism, config, phi_rad):
    """``config`` requiring ``phi_rad``, on the verification grid."""

    if hasattr(config, "goal"):
        config = dataclasses.replace(config, goal=dataclasses.replace(config.goal, required_rom_rad=phi_rad))
    else:
        config = dataclasses.replace(config, required_rom_rad=phi_rad)
    return mechanism.refine(config, _goal(config).verify_step_rad)


def _evaluate(task):
    name, label, x = task
    mechanism, configs = _JOB[name]
    rows = []
    for phi_deg, config in configs:
        e = mechanism.evaluate(x, config)
        if e is not None and e.feasible:
            rows.append((phi_deg, e.pad_coverage, e.proximal_effective_mm, e.distal_effective_mm,
                         e.torque_ratio_max, label))
    return name, x, rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--linkage", type=Path, nargs="+", required=True, help="linkage study directories")
    parser.add_argument("--revolute", type=Path, nargs="+", required=True, help="2R study directories")
    parser.add_argument("--from", dest="start", type=float, default=90.0)
    parser.add_argument("--step", type=float, default=10.0)
    parser.add_argument("--workers", type=int, default=16)
    parser.add_argument("--output", type=Path, default=REPOSITORY_ROOT / "output" / "comparison" / "coverage_by_flexion.csv")
    args = parser.parse_args(argv)

    targets = [args.start + k * args.step for k in range(int((180.0 - args.start) / args.step + 1e-9) + 1)]
    tasks, reference = [], None
    for name, studies in (("linkage", args.linkage), ("revolute", args.revolute)):
        runs = [load_run(d) for study in studies for d in sorted(study.glob("run_*"))]
        mechanism, config = runs[0].mechanism, runs[0].config
        for run in runs:  # one goal, the required flexion and search settings aside
            if _comparable(_goal(run.config)) != _comparable(_goal(config)):
                parser.error(f"{run.directory} was run on another goal")
        goal = _comparable(_goal(config))
        if reference is not None and goal != reference:
            parser.error("the linkage and the 2R studies were run on different goals")
        reference = goal
        _JOB[name] = (mechanism, [(phi, _with_flexion(mechanism, config, math.radians(phi))) for phi in targets])
        seen = set()
        for run in runs:
            for row in run.candidates():
                x = tuple(mechanism.read_design(row))
                if x not in seen:
                    seen.add(x)
                    tasks.append((name, f"{run.directory.parent.name}/{run.directory.name}#{row['candidate']}", x))
    with multiprocessing.get_context("fork").Pool(args.workers) as pool:
        results = pool.map(_evaluate, tasks, chunksize=8)

    best: dict[tuple[str, float], tuple] = {}
    for name, x, rows in results:
        for phi, coverage, proximal, distal, k, label in rows:
            key = (name, phi)
            if key not in best or coverage > best[key][0]:
                best[key] = (coverage, proximal, distal, k, label, x)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("mechanism", "flexion_deg", "pad_coverage", "proximal_effective_mm", "distal_effective_mm",
                         "torque_ratio_max", "design", "x"))
        for (name, phi), (coverage, proximal, distal, k, label, x) in sorted(best.items()):
            writer.writerow((name, phi, coverage, proximal, distal, k, label, " ".join(map(str, x))))
    counts = {name: sum(task[0] == name for task in tasks) for name in _JOB}
    print(f"candidates: {counts}")
    print(f"{'flexion':>8s}  {'linkage (P + D mm, k_max)':>34s}  {'serial 2R (P + D mm)':>26s}")
    for phi in targets:
        cells = []
        for name in ("linkage", "revolute"):
            if (name, phi) in best:
                coverage, proximal, distal, k, _, _ = best[(name, phi)]
                extra = f", k {k:.2f}" if name == "linkage" else ""
                cells.append(f"{coverage:.3f} ({proximal:4.1f} + {distal:4.1f}{extra})")
            else:
                cells.append("-")
        print(f"{phi:7.0f}°  {cells[0]:>34s}  {cells[1]:>26s}")
    print(f"saved {args.output}")
    return 0


GOAL_FIELDS = ("packaging", "extended_length_bounds_mm", "proximal_length_bounds_mm",
               "minimum_proximal_support_mm", "minimum_distal_support_mm", "transmission_angle_range_rad", "hyperextension_rad", "mcp_rotation_checked",
               "mcp_flexion_max_rad", "pip_servo_tilt_rad")
"""What a study's goal is made of: the required flexion and the search settings aside."""


def _comparable(goal):
    return tuple(getattr(goal, name) for name in GOAL_FIELDS)


if __name__ == "__main__":
    raise SystemExit(main())
