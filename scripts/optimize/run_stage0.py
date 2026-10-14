#!/usr/bin/env python3
"""Run the Stage-0 NSGA-II linkage study and save its feasible Pareto set.

Edit ``CONFIG`` below, then run ``python scripts/optimize/run_stage0.py``.
Each run writes ``run.json`` and ``pareto.csv`` to a new ``outputs/stage0/run_NNN``.
"""

from __future__ import annotations

import math
import subprocess
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from cmd.experimental.linkage import LinkageWorkspaceCriteria  # noqa: E402
from cmd.experimental.stage0 import Stage0Config  # noqa: E402
from cmd.experimental.stage0.run import run_stage0, save_stage0_result  # noqa: E402


OUTPUT_ROOT = REPOSITORY_ROOT / "outputs" / "stage0"

# TODO: replace these placeholder values with the CLEAR Hand design requirements.
CONFIG = Stage0Config(
    bounds={
        "op_from_o_x_mm": (14.5, 30.0),
        "op_from_o_z_mm": (3.8, 27.5),
        # Only the O' bounds are physical; the rest are wide enough not to bind
        # and rely on the length and pin-spacing constraints.
        "a_from_o_x_mm": (20.0, 50.0),
        "a_from_o_z_mm": (-10.0, 30.0),
        "ap_from_o_x_mm": (15.0, 45.0),
        "b_from_a_x_mm": (10.0, 40.0),
        "bp_from_a_x_mm": (-10.0, 20.0),
        "bc_length_mm": (20.0, 35.0),
        "cp_from_b_x_mm": (-25.0, 5.0),
        "cp_from_b_z_mm": (-10.0, 20.0),
    },
    # Designs are swept to their own motion limit, at most this MCP flexion;
    # 0.05 rad samples catch late PIP/DIP stalls that a coarser grid misses.
    search_horizon_rad=1.5,
    sample_count=31,
    criteria=LinkageWorkspaceCriteria(
        # Initial hard minimum; 40 deg is the stricter comparison level.
        minimum_transmission_margin_rad=math.radians(30.0),
        q2_minimum_motion_ratio=0.1,
        q3_minimum_motion_ratio=0.1,
        q2_maximum_motion_ratio=3.0,
        q3_maximum_motion_ratio=3.0,
    ),
    extended_length_bounds_mm=(80.0, 110.0),
    # MR63 (3x6x2.5) bearings in aluminium: 6 mm OD + 3 mm web between pockets.
    minimum_pin_spacing_mm=9.0,
    # At the grasp pose the pad faces OA' from at least this palmar gap (link thickness).
    minimum_grasp_gap_mm=15.0,
    # Safe operating range: grasp MCP + 20 deg must keep both loops closing with
    # healthy transmission and no q2/q3 reversal.
    minimum_post_grasp_travel_rad=math.radians(20.0),
    population_size=200,
    generations=200,
    seed=0,
)


def main() -> int:
    result = run_stage0(CONFIG)
    output = save_stage0_result(
        result, _next_run_directory(OUTPUT_ROOT), source_git_commit=_git_commit()
    )
    print(f"Evaluations: {result.evaluation_count}")
    print(f"Feasible Pareto candidates: {len(result.pareto)}")
    for index, candidate in enumerate(result.pareto[:5], start=1):
        evaluation = candidate.evaluation
        print(
            f"  {index}: worst transmission quality "
            f"{evaluation.worst_transmission_quality:.3f}, "
            f"facing error {math.degrees(evaluation.facing_error_rad):.1f} deg, "
            f"grasp at MCP {math.degrees(evaluation.grasp_mcp_rad):.0f} deg, "
            f"post-grasp travel {math.degrees(evaluation.post_grasp_travel_rad):.0f} deg"
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
