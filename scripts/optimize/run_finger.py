#!/usr/bin/env python3
"""Run the 2-DoF finger NSGA-II study and save its feasible Pareto set.

Edit ``CONFIG`` below, then run, e.g.

    python scripts/optimize/run_finger.py --curl-gap-mm 10 --seed 0

Each run writes ``run.json`` and ``pareto.csv`` to a new ``outputs/finger/run_NNN``.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import subprocess
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from cmd.optimization.finger import FingerStudyConfig  # noqa: E402
from cmd.optimization.finger.run import run_study, save_result  # noqa: E402


OUTPUT_ROOT = REPOSITORY_ROOT / "outputs" / "finger"

CONFIG = FingerStudyConfig(
    bounds={
        # OA >= 36 mm keeps the PIP servo clear of the MCP servo.
        "a_x_mm": (36.0, 60.0),
        "b_x_mm": (55.0, 90.0),
        "ap_x_mm": (20.0, 100.0),
        # A' and B' sit dorsal of the pad line, at most 20 mm (finger thickness).
        "ap_z_mm": (0.0, 20.0),
        "bp_x_mm": (40.0, 110.0),
        "bp_z_mm": (0.0, 20.0),
    },
    # Human index distal segment; also the fingertip length.
    bc_length_mm=24.0,
    # Evaluate 10 deg into hyperextension and 10 deg past the curl pose (stoppers).
    range_margin_rad=math.radians(10.0),
    minimum_transmission_margin_rad=math.radians(30.0),
    minimum_pin_spacing_mm=9.0,
    extended_length_bounds_mm=(90.0, 110.0),
    # OA-to-BC line distance at the curl pose (the contact interfaces); override
    # with --curl-gap-mm.
    curl_gap_mm=10.0,
    curl_gap_tolerance_mm=0.5,
    # The undriven link may pass through the PIP servo footprint for now; a
    # curved link is to route around it.
    link_clearance=False,
    # The PIP servo at A' sits dorsal of the OA contact interface.
    oa_interface_clearance_mm=0.0,
    population_size=200,
    generations=200,
    seed=0,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--curl-gap-mm", type=float, default=CONFIG.curl_gap_mm)
    parser.add_argument("--seed", type=int, default=CONFIG.seed)
    parser.add_argument("--oa-interface-mm", type=float, default=CONFIG.oa_interface_clearance_mm,
                        help="PIP servo clearance dorsal of OA")
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    args = parser.parse_args(argv)
    config = dataclasses.replace(
        CONFIG, curl_gap_mm=args.curl_gap_mm, seed=args.seed,
        oa_interface_clearance_mm=args.oa_interface_mm,
    )

    result = run_study(config)
    output = save_result(result, _next_run_directory(args.output_root), source_git_commit=_git_commit())
    print(f"Evaluations: {result.evaluation_count}")
    print(f"Feasible Pareto candidates: {len(result.pareto)}")
    for index, candidate in enumerate(result.pareto[:8], start=1):
        e = candidate.evaluation
        print(
            f"  {index}: quality {e.worst_transmission_quality:.3f}, AB {e.ab_mm:.1f} mm, "
            f"curl at PIP {math.degrees(e.curl_pip_rad):.0f} / DIP {math.degrees(e.curl_dip_rad):.0f} deg, "
            f"gap {e.curl_gap_mm:.1f} mm"
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
