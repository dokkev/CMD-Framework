#!/usr/bin/env python3
"""Run the Stage-0 NSGA-II linkage seed study and save its feasible Pareto set.

See ``docs/optimization/stage0.md``. Objectives: the worst torque ratio
``k_max``, ``-(OA + BC)`` and ``180 deg - phi_safe_max``.

Edit ``CONFIG`` below (the OA and BC ranges are its ``a_x_mm`` and
``bc_length_mm`` bounds), then run, e.g.

    python scripts/optimize/run_stage0.py --seed 0

Each run writes ``run.json`` and ``pareto.csv`` to a new ``outputs/stage0/run_NNN``;
``select_stage0.py`` verifies them and marks the seeds.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import subprocess
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from cmd.morphology.linkage.packaging import Packaging  # noqa: E402
from cmd.optimization.stage0 import VARIABLE_NAMES, FingerStudyConfig  # noqa: E402
from cmd.optimization.stage0.run import run_study, save_result  # noqa: E402


OUTPUT_ROOT = REPOSITORY_ROOT / "outputs" / "stage0"

CONFIG = FingerStudyConfig(
    bounds={
        # OA is free; the servo clearance constraints keep the two servos apart.
        "a_x_mm": (5.0, 120.0),
        "b_x_mm": (40.0, 100.0),
        # BC stays parallel to OA but may sit above or below it.
        "b_z_mm": (-10.0, 10.0),
        "ap_x_mm": (20.0, 100.0),
        # A' sits dorsal of OA, at most 20 mm.
        "ap_z_mm": (0.0, 20.0),
        # B' is free in practice.
        "bp_x_mm": (-50.0, 200.0),
        "bp_z_mm": (-60.0, 80.0),
        # The PIP servo body may tilt dorsally, at most 45 deg.
        "servo_tilt_deg": (0.0, 45.0),
        # BC, the middle + distal phalanx.
        "bc_length_mm": (30.0, 50.0),
    },
    # Objective 1: the worst servo torque per unit distal-link moment.
    objective="torque_ratio",
    # The distal link must turn at least 150 deg relative to OA, safely.
    minimum_flexion_rad=math.radians(150.0),
    # Safe 10 deg into hyperextension; the usable range ends 10 deg of PIP
    # before the first unsafe pose (room for stoppers).
    range_margin_rad=math.radians(10.0),
    minimum_transmission_margin_rad=math.radians(30.0),
    # Half the 0.1 deg verification tolerance: optima sit on the DIP
    # monotonicity boundary, so the coarse search keeps some slack.
    dip_backoff_tolerance_rad=math.radians(0.05),
    # XM335 servos at MCP (O) and PIP (A'), 3 x 4.5 mm pin bushings (McMaster
    # 2706T72) in 1 mm machined aluminium walls, 1 mm clearances; pass other
    # hardware here.
    packaging=Packaging(),
    # |OC| in the fully extended pose.
    extended_length_bounds_mm=(80.0, 100.0),
    # BC <= OA: the curled tip stays near O.
    bc_to_oa_ratio_max=1.0,
    # Held until Stage-1 (reported, not enforced): the tip C ahead of O at the
    # curl pose and the fingertip clearing the MCP servo.
    curl_tip_min_x_mm=0.0,
    enforce_curl_tip=False,
    mcp_flexion_max_rad=math.radians(90.0),
    enforce_fingertip_mcp=False,
    # The middle link's plates straddle the PIP servo in other layers.
    link_clearance=False,
    # The PIP servo at A' sits dorsal of the OA contact interface.
    oa_interface_clearance_mm=0.0,
    population_size=200,
    generations=200,
    seed=0,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seed", type=int, default=CONFIG.seed)
    parser.add_argument("--oa-interface-mm", type=float, default=CONFIG.oa_interface_clearance_mm,
                        help="PIP servo clearance dorsal of OA")
    parser.add_argument("--objective", choices=("transmission", "lever", "torque_ratio"),
                        default=CONFIG.objective)
    parser.add_argument("--servo-travel-max-deg", type=float,
                        help="most PIP servo rotation from extension to curl (default: unbounded)")
    parser.add_argument("--reference-point", choices=("C", "B"), default=CONFIG.reference_point,
                        help="point for the length and curl-tip constraints")
    parser.add_argument("--length-mm", type=float, nargs=2, metavar=("MIN", "MAX"),
                        default=CONFIG.extended_length_bounds_mm,
                        help="bounds on |O ref| in the extended pose")
    parser.add_argument("--min-flexion-deg", type=float,
                        default=math.degrees(CONFIG.minimum_flexion_rad),
                        help="least phi_safe_max (default: %(default)s)")
    parser.add_argument("--bc-to-oa-max", type=float, default=CONFIG.bc_to_oa_ratio_max,
                        help="most BC / OA (default: %(default)s)")
    parser.add_argument("--population", type=int, default=CONFIG.population_size)
    parser.add_argument("--generations", type=int, default=CONFIG.generations)
    parser.add_argument("--warm-start", type=Path, nargs="+", default=(), metavar="CSV",
                        help="pareto.csv or seeds.csv files whose designs seed the population")
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    args = parser.parse_args(argv)
    config = dataclasses.replace(
        CONFIG, seed=args.seed,
        population_size=args.population, generations=args.generations,
        objective=args.objective,
        reference_point=args.reference_point,
        extended_length_bounds_mm=tuple(args.length_mm),
        servo_travel_max_rad=(math.radians(args.servo_travel_max_deg)
                              if args.servo_travel_max_deg is not None else None),
        oa_interface_clearance_mm=args.oa_interface_mm,
        bc_to_oa_ratio_max=args.bc_to_oa_max,
        minimum_flexion_rad=math.radians(args.min_flexion_deg),
    )

    initial = [
        [float(row[name]) for name in VARIABLE_NAMES]
        for path in args.warm_start
        for row in csv.DictReader(path.open(encoding="utf-8", newline=""))
    ][:config.population_size]
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
