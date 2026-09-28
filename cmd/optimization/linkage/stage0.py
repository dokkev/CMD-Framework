"""Caller-bounded Stage-0 linkage runs and standard-library result export."""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from cmd.morphology import BASELINE_LINKAGE_V0
from cmd.optimization import BoxVariable, OptimizationProblem, ProblemEvaluation
from cmd.optimization.solvers.pymoo import NSGA2Config, NSGA2RunResult, run_nsga2

from .baseline import LinkageBaselineConstraintConfig, build_linkage_baseline_problem
from .evaluator import LinkageEvaluation, LinkageSweepConfig
from .parameterization import LinkageBaselineParameterization
from .workspace import evaluate_linkage_workspace


@dataclass(frozen=True)
class LinkageStage0Result:
    """Stage-0 configuration, composed problem, and evaluated NSGA-II output."""

    problem: OptimizationProblem
    constraint_config: LinkageBaselineConstraintConfig
    sweep_config: LinkageSweepConfig
    solver_result: NSGA2RunResult


def run_linkage_stage0(
    variables: Sequence[BoxVariable],
    constraint_config: LinkageBaselineConstraintConfig,
    sweep_config: LinkageSweepConfig,
    solver_config: NSGA2Config,
    *,
    include_baseline_seed: bool = True,
) -> LinkageStage0Result:
    """Build and run the existing Stage-0 formulation within caller bounds."""

    if not isinstance(include_baseline_seed, bool):
        raise ValueError("include_baseline_seed must be a bool")
    problem = build_linkage_baseline_problem(
        variables,
        constraint_config,
        sweep_config,
    )
    prototype_x = LinkageBaselineParameterization().encode(BASELINE_LINKAGE_V0)
    initial_designs: tuple[tuple[float, ...], ...] = ()
    if include_baseline_seed:
        for variable, value in zip(problem.variables, prototype_x):
            if variable.violation(value) > 0.0:
                raise ValueError(
                    "projected BASELINE_LINKAGE_V0 is outside caller-supplied "
                    f"bounds for {variable.name!r}: {value} not in "
                    f"[{variable.lower}, {variable.upper}]"
                )
        initial_designs = (prototype_x,)

    solver_result = run_nsga2(
        problem,
        prototype_x,
        solver_config,
        initial_designs=initial_designs,
    )
    return LinkageStage0Result(
        problem=problem,
        constraint_config=constraint_config,
        sweep_config=sweep_config,
        solver_result=solver_result,
    )


def export_linkage_stage0_results(
    result: LinkageStage0Result,
    output_directory: str | Path,
) -> tuple[Path, Path, Path]:
    """Write run metadata, Pareto candidates, and final population as JSON/CSV."""

    if not isinstance(result, LinkageStage0Result):
        raise TypeError("result must be a LinkageStage0Result")
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    run_path = output / "run.json"
    pareto_path = output / "pareto.csv"
    population_path = output / "final_population.csv"

    problem = result.problem
    solver = result.solver_result
    constraints = tuple(
        ("equality", constraint.name) for constraint in problem.equalities
    ) + tuple(("inequality", constraint.name) for constraint in problem.inequalities)
    metadata = {
        "solver": "pymoo_nsga2",
        "pymoo_version": solver.pymoo_version,
        "population_size": solver.config.population_size,
        "generations": solver.config.generations,
        "seed": solver.config.seed,
        "eliminate_duplicates": solver.config.eliminate_duplicates,
        "nonfinite_objective_penalty": solver.config.nonfinite_objective_penalty,
        "termination": solver.termination,
        "generation_count": solver.generation_count,
        "evaluation_count": solver.evaluation_count,
        "search_horizon_rad": result.sweep_config.search_horizon_rad,
        "sample_count": result.sweep_config.sample_count,
        "core_input_workspace_rad": result.constraint_config.core_input_workspace_rad,
        "minimum_required_input_workspace_rad": (
            result.constraint_config.minimum_required_input_workspace_rad
        ),
        "minimum_anchor_dorsal_offset_mm": (
            result.constraint_config.minimum_anchor_dorsal_offset_mm
        ),
        "q2_minimum_endpoint_flexion_rad": (
            result.constraint_config.q2_minimum_endpoint_flexion_rad
        ),
        "q3_minimum_endpoint_flexion_rad": (
            result.constraint_config.q3_minimum_endpoint_flexion_rad
        ),
        "workspace_criteria": {
            "minimum_transmission_margin_rad": (
                result.constraint_config.workspace_criteria.minimum_transmission_margin_rad
            ),
            "q2_minimum_motion_ratio": (
                result.constraint_config.workspace_criteria.q2_minimum_motion_ratio
            ),
            "q3_minimum_motion_ratio": (
                result.constraint_config.workspace_criteria.q3_minimum_motion_ratio
            ),
            "q2_maximum_motion_ratio": (
                result.constraint_config.workspace_criteria.q2_maximum_motion_ratio
            ),
            "q3_maximum_motion_ratio": (
                result.constraint_config.workspace_criteria.q3_maximum_motion_ratio
            ),
        },
        "variables": [
            {"name": variable.name, "lower": variable.lower, "upper": variable.upper}
            for variable in problem.variables
        ],
        "bounds_note": "Caller-supplied search bounds; physical validity is not implied.",
        "objectives": [
            {"name": objective.name, "sense": objective.sense}
            for objective in problem.objectives
        ],
        "constraints": [
            {"kind": kind, "name": name} for kind, name in constraints
        ],
        "pareto_candidate_count": len(solver.pareto_candidates),
        "final_population_candidate_count": len(solver.final_population),
        "feasible_final_population_candidate_count": sum(
            evaluation.feasible for evaluation in solver.final_population
        ),
    }
    with run_path.open("w", newline="", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, allow_nan=False)
        stream.write("\n")

    constraint_columns = tuple(
        f"constraint_{name}_max_violation" for _, name in constraints
    )
    fieldnames = (
        "candidate_id",
        *(variable.name for variable in problem.variables),
        *(objective.name for objective in problem.objectives),
        "closure_input_workspace_rad",
        "usable_input_workspace_rad",
        "distal_path_length_mm",
        "distal_x_excursion_mm",
        "dorsal_z_excursion_mm",
        "first_unusable_index",
        "limiting_reasons",
        "feasible",
        "overall_max_constraint_violation",
        "overall_sum_positive_constraint_violation",
        *constraint_columns,
    )
    _write_candidate_csv(
        pareto_path,
        fieldnames,
        "pareto",
        solver.pareto_candidates,
        result,
        constraints,
    )
    _write_candidate_csv(
        population_path,
        fieldnames,
        "final",
        solver.final_population,
        result,
        constraints,
    )
    return run_path, pareto_path, population_path


def _write_candidate_csv(
    path: Path,
    fieldnames: tuple[str, ...],
    candidate_prefix: str,
    evaluations: tuple[ProblemEvaluation, ...],
    result: LinkageStage0Result,
    constraints: tuple[tuple[str, str], ...],
) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="raise")
        writer.writeheader()
        for index, evaluation in enumerate(evaluations, start=1):
            writer.writerow(
                _candidate_row(
                    f"{candidate_prefix}_{index:04d}",
                    evaluation,
                    result,
                    constraints,
                )
            )


def _candidate_row(
    candidate_id: str,
    evaluation: ProblemEvaluation,
    result: LinkageStage0Result,
    constraints: tuple[tuple[str, str], ...],
) -> dict[str, object]:
    domain = evaluation.domain_evaluation
    if not isinstance(domain, LinkageEvaluation):
        raise TypeError("Stage-0 candidate domain_evaluation must be LinkageEvaluation")
    workspace = evaluate_linkage_workspace(
        domain,
        result.constraint_config.workspace_criteria,
    )
    box_violations = list(evaluation.box_violations.values())
    equality_violations = [
        item.maximum_violation for item in evaluation.equality_results
    ]
    inequality_violations = [
        item.maximum_violation for item in evaluation.inequality_results
    ]
    all_maximum_violations = (
        box_violations + equality_violations + inequality_violations
    )
    positive_violation_magnitudes = (
        box_violations
        + equality_violations
        + [
            max(residual, 0.0)
            for item in evaluation.inequality_results
            for residual in item.residuals
        ]
    )
    row: dict[str, object] = {
        "candidate_id": candidate_id,
        **evaluation.variable_values,
        **evaluation.raw_objective_values,
        "closure_input_workspace_rad": workspace.closure_input_workspace_rad,
        "usable_input_workspace_rad": workspace.usable_input_workspace_rad,
        "distal_path_length_mm": workspace.distal_path_length_mm,
        "distal_x_excursion_mm": workspace.distal_x_excursion_mm,
        "dorsal_z_excursion_mm": workspace.dorsal_z_excursion_mm,
        "first_unusable_index": (
            "" if workspace.first_unusable_index is None else workspace.first_unusable_index
        ),
        "limiting_reasons": ";".join(workspace.limiting_reasons),
        "feasible": evaluation.feasible,
        "overall_max_constraint_violation": max(all_maximum_violations, default=0.0),
        "overall_sum_positive_constraint_violation": math.fsum(
            value for value in positive_violation_magnitudes if value > 0.0
        ),
    }
    named_results = {
        ("equality", item.name): item for item in evaluation.equality_results
    }
    for item in evaluation.inequality_results:
        named_results[("inequality", item.name)] = item
    for kind, name in constraints:
        row[f"constraint_{name}_max_violation"] = named_results[(kind, name)].maximum_violation
    return row


__all__ = [
    "LinkageStage0Result",
    "export_linkage_stage0_results",
    "run_linkage_stage0",
]
