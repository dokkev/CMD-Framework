#!/usr/bin/env python3
"""Run a JSON-defined CLEAR Stage-0 linkage optimization study.

Run from any directory with:

    python scripts/optimize/optimize_linkages.py

Edit ``STUDY_PATH`` below to select a different study file. If it does not yet
exist, the first run writes an incomplete template there and exits. Solver
overrides can be supplied to :func:`run_study` or set in ``SOLVER_OVERRIDES``.
"""

from __future__ import annotations

import json
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from cmd.optimization.linkage.diagnostics import summarize_linkage_stage0  # noqa: E402
from cmd.optimization.linkage.stage0 import (  # noqa: E402
    LinkageStage0Result,
    export_linkage_stage0_results,
    run_linkage_stage0,
)
from cmd.optimization.linkage.study import (  # noqa: E402
    load_linkage_stage0_study,
    save_linkage_stage0_study,
    write_linkage_stage0_study_template,
)


STUDY_PATH = REPOSITORY_ROOT / "studies" / "linkage_stage0.json"
OUTPUT_ROOT = REPOSITORY_ROOT / "outputs" / "linkage_stage0"
SOLVER_OVERRIDES: dict[str, int] = {}


def main() -> int:
    if not STUDY_PATH.exists():
        write_linkage_stage0_study_template(STUDY_PATH)
        print(f"Wrote incomplete study template: {STUDY_PATH}")
        print("Fill in the bounds and study settings, then run this script again.")
        return 0

    result, output = run_study(
        STUDY_PATH,
        OUTPUT_ROOT,
        solver_overrides=SOLVER_OVERRIDES,
    )
    _print_summary(
        result.study_name or "(unnamed)",
        result,
        summarize_linkage_stage0(result),
        output,
    )
    return 0


def run_study(
    study_path: str | Path,
    output_root: str | Path,
    *,
    solver_overrides: dict[str, int] | None = None,
) -> tuple[LinkageStage0Result, Path]:
    """Load and execute one study, allocating a new non-overwriting run folder."""

    study = load_linkage_stage0_study(study_path)
    overrides = dict(solver_overrides or {})
    allowed_overrides = {"population_size", "generations", "seed"}
    unexpected = set(overrides) - allowed_overrides
    if unexpected:
        raise ValueError(f"unsupported solver overrides: {sorted(unexpected)!r}")
    resolved_study = replace(
        study,
        solver_config=replace(study.solver_config, **overrides),
    )
    output = _next_output_directory(Path(output_root))

    result = run_linkage_stage0(
        resolved_study.variables,
        resolved_study.constraint_config,
        resolved_study.sweep_config,
        resolved_study.solver_config,
        include_baseline_seed=resolved_study.include_baseline_seed,
        study_name=resolved_study.name,
    )
    save_linkage_stage0_study(resolved_study, output / "study.resolved.json")
    export_linkage_stage0_results(
        result,
        output,
        source_git_commit=_source_git_commit(),
    )
    diagnostics = summarize_linkage_stage0(result)
    with (output / "diagnostics.json").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        json.dump(diagnostics, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return result, output


def _next_output_directory(output_root: Path) -> Path:
    if output_root.exists() and not output_root.is_dir():
        raise ValueError(f"output root exists and is not a directory: {output_root}")
    run_number = 1
    while True:
        output = output_root / f"run_{run_number:03d}"
        if not output.exists():
            return output
        run_number += 1


def _source_git_commit() -> str | None:
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=REPOSITORY_ROOT,
            check=True,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    revision = completed.stdout.strip()
    return revision or None


def _print_summary(
    study_name: str,
    result: LinkageStage0Result,
    diagnostics: dict[str, object],
    output: Path,
) -> None:
    final_count = diagnostics["final_population_count"]
    feasible_count = diagnostics["feasible_count"]
    feasible_fraction = diagnostics["feasible_fraction"]
    print("Stage-0 linkage optimization complete")
    print(f"Study: {study_name}")
    print(f"Evaluations: {result.solver_result.evaluation_count}")
    print(f"Final population: {final_count}")
    print(
        f"Feasible: {feasible_count} / {final_count} "
        f"({100.0 * float(feasible_fraction):.1f}%)"
    )
    print(f"Pareto candidates: {diagnostics['pareto_candidate_count']}")

    constraints = diagnostics["constraints"]
    assert isinstance(constraints, dict)
    most_violated = sorted(
        (
            (name, summary)
            for name, summary in constraints.items()
            if summary["violated_candidate_count"] > 0
        ),
        key=lambda item: (
            -item[1]["violation_fraction"],
            -item[1]["violated_candidate_count"],
            item[0],
        ),
    )[:3]
    print("Most frequently violated constraints:")
    if most_violated:
        for name, summary in most_violated:
            print(
                f"  {name}: {100.0 * summary['violation_fraction']:.1f}% "
                f"({summary['violated_candidate_count']}/{final_count})"
            )
    else:
        print("  none")
    print("Output:")
    print(f"  {output}")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        raise SystemExit(
            "This runner takes no command-line arguments; configure the study "
            "JSON and script constants instead."
        )
    raise SystemExit(main())
