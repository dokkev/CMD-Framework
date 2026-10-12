#!/usr/bin/env python3
"""Run the Stage-0 NSGA-II linkage study and save its feasible Pareto set.

Edit ``CONFIG`` below, then run ``python scripts/optimize/run_stage0.py``.
Each run writes ``run.json`` and ``pareto.csv`` to a new ``outputs/stage0/run_NNN``.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from cmd.morphology.linkage import BASELINE_LINKAGE_V0, LinkageWorkspaceCriteria  # noqa: E402
from cmd.optimization.stage0 import Stage0Config, bounds_around  # noqa: E402
from cmd.optimization.stage0.run import run_stage0, save_stage0_result  # noqa: E402


OUTPUT_ROOT = REPOSITORY_ROOT / "outputs" / "stage0"

# TODO: replace these placeholder values with the CLEAR Hand design requirements.
CONFIG = Stage0Config(
    bounds=bounds_around(BASELINE_LINKAGE_V0, half_width_mm=5.0),
    core_input_workspace_rad=0.6,
    required_input_workspace_rad=0.9,
    search_horizon_rad=1.5,
    sample_count=11,
    criteria=LinkageWorkspaceCriteria(
        minimum_transmission_margin_rad=0.3,
        q2_minimum_motion_ratio=0.1,
        q3_minimum_motion_ratio=0.1,
        q2_maximum_motion_ratio=3.0,
        q3_maximum_motion_ratio=3.0,
    ),
    q2_minimum_endpoint_flexion_rad=0.3,
    q3_minimum_endpoint_flexion_rad=0.3,
    minimum_anchor_dorsal_offset_mm=0.0,
    population_size=100,
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
            f"ratio variation {evaluation.ratio_variation:.3f}"
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
