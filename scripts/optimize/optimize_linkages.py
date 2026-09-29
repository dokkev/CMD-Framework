#!/usr/bin/env python3
"""Run a JSON-defined Stage-0 linkage study without command-line options."""

from __future__ import annotations

import json
import math
import subprocess
import sys
from collections.abc import Mapping
from dataclasses import replace
from numbers import Real
from pathlib import Path
from typing import Any

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from cmd.morphology.linkage import LinkageWorkspaceCriteria  # noqa: E402
from cmd.optimization import BoxVariable  # noqa: E402
from cmd.optimization.linkage import (  # noqa: E402
    LinkageBaselineParameterization,
    LinkageStage0Config,
    LinkageStage0Result,
    run_linkage_stage0,
)
from cmd.optimization.linkage.study import LinkageStage0Study  # noqa: E402
from cmd.optimization.solvers.pymoo import NSGA2Config  # noqa: E402
from scripts.optimize.linkage_report import (  # noqa: E402
    export_linkage_stage0_results,
    summarize_linkage_stage0,
)


STUDY_PATH = REPOSITORY_ROOT / "studies" / "linkage_stage0.json"
OUTPUT_ROOT = REPOSITORY_ROOT / "outputs" / "linkage_stage0"
SOLVER_OVERRIDES: dict[str, object] = {}


def main() -> int:
    if not STUDY_PATH.exists():
        write_linkage_stage0_study_template(STUDY_PATH)
        print(f"Wrote incomplete study template: {STUDY_PATH}")
        print("Fill in the bounds and study settings, then run this script again.")
        return 0
    result, output = run_study(STUDY_PATH, OUTPUT_ROOT, solver_overrides=SOLVER_OVERRIDES)
    _print_summary(result.study_name or "(unnamed)", result, summarize_linkage_stage0(result), output)
    return 0


def run_study(
    study_path: str | Path,
    output_root: str | Path,
    *,
    solver_overrides: Mapping[str, object] | None = None,
) -> tuple[LinkageStage0Result, Path]:
    """Load, validate, execute and export a study to a fresh numbered folder."""
    study = load_linkage_stage0_study(study_path)
    overrides = dict(solver_overrides or {})
    allowed = set(NSGA2Config.__dataclass_fields__)
    unexpected = set(overrides) - allowed
    if unexpected:
        raise ValueError(f"unsupported solver overrides: {sorted(unexpected)!r}")
    resolved = replace(study, solver=replace(study.solver, **overrides))
    result = run_linkage_stage0(
        resolved.variables,
        resolved.stage0,
        resolved.solver,
        include_baseline_seed=resolved.include_baseline_seed,
        study_name=resolved.name,
    )
    output = _next_output_directory(Path(output_root))
    output.mkdir(parents=True, exist_ok=False)
    save_linkage_stage0_study(resolved, output / "study.resolved.json")
    export_linkage_stage0_results(result, output, source_git_commit=_source_git_commit())
    diagnostics = summarize_linkage_stage0(result)
    with (output / "diagnostics.json").open("w", encoding="utf-8", newline="") as stream:
        json.dump(diagnostics, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return result, output


def study_to_dict(study: LinkageStage0Study) -> dict[str, object]:
    criteria = study.stage0.workspace_criteria
    config = study.stage0
    return {
        "name": study.name,
        "variables": [
            {"name": variable.name, "lower": variable.lower, "upper": variable.upper}
            for variable in study.variables
        ],
        "workspace": {
            "core_input_workspace_rad": config.core_input_workspace_rad,
            "minimum_required_input_workspace_rad": config.minimum_required_input_workspace_rad,
            "search_horizon_rad": config.search_horizon_rad,
            "sample_count": config.sample_count,
        },
        "criteria": {
            name: getattr(criteria, name)
            for name in (
                "minimum_transmission_margin_rad", "q2_minimum_motion_ratio",
                "q3_minimum_motion_ratio", "q2_maximum_motion_ratio",
                "q3_maximum_motion_ratio",
            )
        },
        "constraints": {
            "minimum_anchor_dorsal_offset_mm": config.minimum_anchor_dorsal_offset_mm,
            "q2_minimum_endpoint_flexion_rad": config.q2_minimum_endpoint_flexion_rad,
            "q3_minimum_endpoint_flexion_rad": config.q3_minimum_endpoint_flexion_rad,
        },
        "solver": {
            "population_size": study.solver.population_size,
            "generations": study.solver.generations,
            "seed": study.solver.seed,
            "eliminate_duplicates": study.solver.eliminate_duplicates,
            "nonfinite_objective_penalty": study.solver.nonfinite_objective_penalty,
        },
        "include_baseline_seed": study.include_baseline_seed,
    }


def study_from_dict(data: object) -> LinkageStage0Study:
    root = _mapping(data, "study")
    _require_keys(root, "study", (
        "name", "variables", "workspace", "criteria", "constraints", "solver",
        "include_baseline_seed",
    ))
    raw_variables = root["variables"]
    if not isinstance(raw_variables, list):
        raise ValueError("study.variables must be an ordered JSON list")
    variables = []
    for index, raw in enumerate(raw_variables):
        path = f"study.variables[{index}]"
        item = _mapping(raw, path)
        _require_keys(item, path, ("name", "lower", "upper"))
        if not isinstance(item["name"], str):
            raise ValueError(f"{path}.name must be a string")
        variables.append(BoxVariable(
            item["name"], _number(item["lower"], f"{path}.lower"),
            _number(item["upper"], f"{path}.upper"),
        ))

    workspace = _mapping(root["workspace"], "study.workspace")
    workspace_keys = (
        "core_input_workspace_rad", "minimum_required_input_workspace_rad",
        "search_horizon_rad", "sample_count",
    )
    _require_keys(workspace, "study.workspace", workspace_keys)
    criteria_data = _mapping(root["criteria"], "study.criteria")
    criteria_keys = (
        "minimum_transmission_margin_rad", "q2_minimum_motion_ratio",
        "q3_minimum_motion_ratio", "q2_maximum_motion_ratio",
        "q3_maximum_motion_ratio",
    )
    _require_keys(criteria_data, "study.criteria", criteria_keys)
    constraint_data = _mapping(root["constraints"], "study.constraints")
    constraint_keys = (
        "minimum_anchor_dorsal_offset_mm", "q2_minimum_endpoint_flexion_rad",
        "q3_minimum_endpoint_flexion_rad",
    )
    _require_keys(constraint_data, "study.constraints", constraint_keys)
    solver_data = _mapping(root["solver"], "study.solver")
    solver_keys = (
        "population_size", "generations", "seed", "eliminate_duplicates",
        "nonfinite_objective_penalty",
    )
    _require_keys(solver_data, "study.solver", solver_keys)
    for key in ("population_size", "generations", "seed"):
        if isinstance(solver_data[key], bool) or not isinstance(solver_data[key], int):
            raise ValueError(f"study.solver.{key} must be an integer")
    if not isinstance(solver_data["eliminate_duplicates"], bool):
        raise ValueError("study.solver.eliminate_duplicates must be a bool")
    if not isinstance(root["name"], str):
        raise ValueError("study.name must be a string")
    if not isinstance(root["include_baseline_seed"], bool):
        raise ValueError("study.include_baseline_seed must be a bool")

    criteria = LinkageWorkspaceCriteria(**{
        key: _number(criteria_data[key], f"study.criteria.{key}")
        for key in criteria_keys
    })
    stage0 = LinkageStage0Config(
        core_input_workspace_rad=_number(workspace["core_input_workspace_rad"], "study.workspace.core_input_workspace_rad"),
        minimum_required_input_workspace_rad=_number(workspace["minimum_required_input_workspace_rad"], "study.workspace.minimum_required_input_workspace_rad"),
        search_horizon_rad=_number(workspace["search_horizon_rad"], "study.workspace.search_horizon_rad"),
        sample_count=_integer(workspace["sample_count"], "study.workspace.sample_count"),
        minimum_anchor_dorsal_offset_mm=_number(constraint_data["minimum_anchor_dorsal_offset_mm"], "study.constraints.minimum_anchor_dorsal_offset_mm"),
        q2_minimum_endpoint_flexion_rad=_number(constraint_data["q2_minimum_endpoint_flexion_rad"], "study.constraints.q2_minimum_endpoint_flexion_rad"),
        q3_minimum_endpoint_flexion_rad=_number(constraint_data["q3_minimum_endpoint_flexion_rad"], "study.constraints.q3_minimum_endpoint_flexion_rad"),
        workspace_criteria=criteria,
    )
    solver = NSGA2Config(
        population_size=solver_data["population_size"],
        generations=solver_data["generations"],
        seed=solver_data["seed"],
        eliminate_duplicates=solver_data["eliminate_duplicates"],
        nonfinite_objective_penalty=_number(solver_data["nonfinite_objective_penalty"], "study.solver.nonfinite_objective_penalty"),
    )
    return LinkageStage0Study(
        name=root["name"], variables=tuple(variables), stage0=stage0,
        solver=solver, include_baseline_seed=root["include_baseline_seed"],
    )


def load_linkage_stage0_study(path: str | Path) -> LinkageStage0Study:
    with Path(path).open(encoding="utf-8") as stream:
        value = json.load(stream, parse_constant=_reject_json_constant)
    return study_from_dict(value)


def save_linkage_stage0_study(study: LinkageStage0Study, path: str | Path) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8", newline="") as stream:
        json.dump(study_to_dict(study), stream, indent=2, allow_nan=False)
        stream.write("\n")
    return destination


def study_template_to_dict() -> dict[str, object]:
    """Build an intentionally incomplete template without guessed bounds."""
    return {
        "name": "linkage_stage0_template",
        "variables": [
            {"name": name, "lower": None, "upper": None}
            for name in LinkageBaselineParameterization().variable_names
        ],
        "workspace": {
            "core_input_workspace_rad": None,
            "minimum_required_input_workspace_rad": None,
            "search_horizon_rad": None,
            "sample_count": None,
        },
        "criteria": {
            "minimum_transmission_margin_rad": None,
            "q2_minimum_motion_ratio": None,
            "q3_minimum_motion_ratio": None,
            "q2_maximum_motion_ratio": None,
            "q3_maximum_motion_ratio": None,
        },
        "constraints": {
            "minimum_anchor_dorsal_offset_mm": None,
            "q2_minimum_endpoint_flexion_rad": None,
            "q3_minimum_endpoint_flexion_rad": None,
        },
        "solver": {
            "population_size": None, "generations": None, "seed": None,
            "eliminate_duplicates": True, "nonfinite_objective_penalty": 1.0e12,
        },
        "include_baseline_seed": True,
    }


def write_linkage_stage0_study_template(path: str | Path) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x", encoding="utf-8", newline="") as stream:
        json.dump(study_template_to_dict(), stream, indent=2, allow_nan=False)
        stream.write("\n")
    return destination


def _next_output_directory(output_root: Path) -> Path:
    if output_root.exists() and not output_root.is_dir():
        raise ValueError(f"output root exists and is not a directory: {output_root}")
    index = 1
    while (candidate := output_root / f"run_{index:03d}").exists():
        index += 1
    return candidate


def _source_git_commit() -> str | None:
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPOSITORY_ROOT,
            check=True, capture_output=True, text=True, timeout=3.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return completed.stdout.strip() or None


def _print_summary(study_name, result, diagnostics, output) -> None:
    count = diagnostics["final_population_count"]
    feasible = diagnostics["feasible_count"]
    fraction = diagnostics["feasible_fraction"]
    print("Stage-0 linkage optimization complete")
    print(f"Study: {study_name}")
    print(f"Evaluations: {result.solver_result.evaluation_count}")
    print(f"Final population: {count}")
    print(f"Feasible: {feasible} / {count} ({100.0 * float(fraction):.1f}%)")
    print(f"Pareto candidates: {diagnostics['pareto_candidate_count']}")
    constraints = diagnostics["constraints"]
    most_violated = sorted(
        ((name, summary) for name, summary in constraints.items() if summary["violated_candidate_count"]),
        key=lambda pair: (-pair[1]["violation_fraction"], -pair[1]["violated_candidate_count"], pair[0]),
    )[:3]
    print("Most frequently violated constraints:")
    for name, summary in most_violated:
        print(f"  {name}: {100.0 * summary['violation_fraction']:.1f}% ({summary['violated_candidate_count']}/{count})")
    if not most_violated:
        print("  none")
    print("Output:")
    print(f"  {output}")


def _mapping(value: object, path: str) -> dict[str, Any]:
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{path} must be a JSON object with string keys")
    return value


def _require_keys(value: dict[str, Any], path: str, expected: tuple[str, ...]) -> None:
    missing = tuple(key for key in expected if key not in value)
    unexpected = tuple(sorted(set(value) - set(expected)))
    if missing or unexpected:
        raise ValueError(f"{path} keys mismatch; missing={missing!r}, unexpected={unexpected!r}")


def _number(value: object, path: str) -> float:
    if value is None:
        raise ValueError(f"{path} is null; complete the study template before running")
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{path} must be a finite JSON number")
    return float(value)


def _integer(value: object, path: str) -> int:
    if value is None:
        raise ValueError(f"{path} is null; complete the study template before running")
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{path} must be a JSON integer")
    return value


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"JSON constant {value!r} is not allowed; use finite numbers")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        raise SystemExit("This runner takes no command-line arguments; configure the study JSON and script constants.")
    raise SystemExit(main())
