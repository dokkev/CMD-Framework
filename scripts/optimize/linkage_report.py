"""CSV export and descriptive reporting for Stage-0 experiment runs."""

from __future__ import annotations

import csv
import json
import math
import statistics
from collections.abc import Iterable
from pathlib import Path

from cmd.optimization import ProblemEvaluation
from cmd.optimization.linkage import LinkageStage0Evaluation, LinkageStage0Result


_REASON_ORDER = (
    "kinematic_failure", "transmission_loop_1", "transmission_loop_2",
    "q2_minimum_motion_ratio", "q3_minimum_motion_ratio",
    "q2_maximum_motion_ratio", "q3_maximum_motion_ratio",
    "q2_motion_ratio_unavailable", "q3_motion_ratio_unavailable", "search_horizon",
)


def summarize_linkage_stage0(result: LinkageStage0Result) -> dict[str, object]:
    """Summarize final-population feasibility, objectives, and workspace."""
    population = result.solver_result.final_population
    count = len(population)
    denominator = count or 1
    feasible = sum(item.feasible for item in population)
    constraint_keys = tuple(
        ("equality", item.name) for item in result.problem.equalities
    ) + tuple(("inequality", item.name) for item in result.problem.inequalities)
    constraint_values = {key: [] for key in constraint_keys}
    box_values = {variable.name: [] for variable in result.problem.variables}
    objective_values = {objective.name: [] for objective in result.problem.objectives}
    closure: list[float] = []
    usable: list[float] = []
    reason_counts = {reason: 0 for reason in _REASON_ORDER}

    for evaluation in population:
        for name, value in evaluation.box_violations.items():
            box_values[name].append(value)
        for item in evaluation.equality_results:
            constraint_values[("equality", item.name)].append(item.maximum_violation)
        for item in evaluation.inequality_results:
            constraint_values[("inequality", item.name)].append(item.maximum_violation)
        for name in objective_values:
            objective_values[name].append(evaluation.raw_objective_values[name])
        domain = _stage0_evaluation(evaluation)
        closure.append(domain.workspace.closure_input_workspace_rad)
        usable.append(domain.workspace.usable_input_workspace_rad)
        for reason in dict.fromkeys(domain.workspace.limiting_reasons):
            reason_counts[reason] = reason_counts.get(reason, 0) + 1

    constraints = {
        name: _violation_summary(constraint_values[(kind, name)], denominator)
        for kind, name in constraint_keys
    }
    box_summaries = {
        name: _violation_summary(values, denominator) for name, values in box_values.items()
    }
    reason_order = _REASON_ORDER + tuple(sorted(set(reason_counts) - set(_REASON_ORDER)))
    return {
        "final_population_count": count,
        "feasible_count": feasible,
        "feasible_fraction": feasible / denominator if count else 0.0,
        "pareto_candidate_count": len(result.solver_result.pareto_candidates),
        "constraints": constraints,
        "box_variables": box_summaries,
        "workspace": {
            "closure_input_workspace_rad": _finite_summary(closure),
            "usable_input_workspace_rad": _finite_summary(usable),
        },
        "objectives": {name: _objective_summary(values) for name, values in objective_values.items()},
        "limiting_reasons": {
            name: {
                "candidate_count": reason_counts[name],
                "fraction": reason_counts[name] / denominator if count else 0.0,
            }
            for name in reason_order
        },
    }


def export_linkage_stage0_results(
    result: LinkageStage0Result,
    output_directory: str | Path,
    *,
    source_git_commit: str | None = None,
) -> tuple[Path, Path, Path]:
    """Write ``run.json`` and candidate CSVs; the run script writes diagnostics."""
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    run_path, pareto_path, population_path = (
        output / "run.json", output / "pareto.csv", output / "final_population.csv"
    )
    problem, solver, config = result.problem, result.solver_result, result.stage0_config
    constraints = tuple(("equality", c.name) for c in problem.equalities) + tuple(
        ("inequality", c.name) for c in problem.inequalities
    )
    metadata = {
        "solver": "pymoo_nsga2",
        "study_name": result.study_name,
        "include_baseline_seed": result.include_baseline_seed,
        "initialization": {
            "baseline_seed_included": result.include_baseline_seed,
            "remaining_population_method": "lhs",
        },
        "source_git_commit": source_git_commit,
        "pymoo_version": solver.pymoo_version,
        "population_size": solver.config.population_size,
        "generations": solver.config.generations,
        "seed": solver.config.seed,
        "eliminate_duplicates": solver.config.eliminate_duplicates,
        "nonfinite_objective_penalty": solver.config.nonfinite_objective_penalty,
        "termination": solver.termination,
        "generation_count": solver.generation_count,
        "evaluation_count": solver.evaluation_count,
        "search_horizon_rad": config.search_horizon_rad,
        "sample_count": config.sample_count,
        "core_input_workspace_rad": config.core_input_workspace_rad,
        "minimum_required_input_workspace_rad": config.minimum_required_input_workspace_rad,
        "minimum_anchor_dorsal_offset_mm": config.minimum_anchor_dorsal_offset_mm,
        "q2_minimum_endpoint_flexion_rad": config.q2_minimum_endpoint_flexion_rad,
        "q3_minimum_endpoint_flexion_rad": config.q3_minimum_endpoint_flexion_rad,
        "workspace_criteria": {
            name: getattr(config.workspace_criteria, name)
            for name in (
                "minimum_transmission_margin_rad", "q2_minimum_motion_ratio",
                "q3_minimum_motion_ratio", "q2_maximum_motion_ratio",
                "q3_maximum_motion_ratio",
            )
        },
        "variables": [
            {"name": v.name, "lower": v.lower, "upper": v.upper} for v in problem.variables
        ],
        "bounds_note": "Caller-supplied search bounds; physical validity is not implied.",
        "objectives": [{"name": o.name, "sense": o.sense} for o in problem.objectives],
        "constraints": [{"kind": kind, "name": name} for kind, name in constraints],
        "pareto_candidate_count": len(solver.pareto_candidates),
        "final_population_candidate_count": len(solver.final_population),
        "feasible_final_population_candidate_count": sum(e.feasible for e in solver.final_population),
    }
    with run_path.open("w", encoding="utf-8", newline="") as stream:
        json.dump(metadata, stream, indent=2, allow_nan=False)
        stream.write("\n")

    fields = (
        "candidate_id", *(v.name for v in problem.variables),
        *(o.name for o in problem.objectives), "closure_input_workspace_rad",
        "usable_input_workspace_rad", "distal_path_length_mm", "distal_x_excursion_mm",
        "dorsal_z_excursion_mm", "first_unusable_index", "limiting_reasons",
        "kinematic_failure_reason", "feasible", "overall_max_constraint_violation",
        "overall_sum_positive_constraint_violation",
        *(f"constraint_{name}_max_violation" for _, name in constraints),
    )
    _write_candidates(pareto_path, fields, "pareto", solver.pareto_candidates, result, constraints)
    _write_candidates(population_path, fields, "final", solver.final_population, result, constraints)
    return run_path, pareto_path, population_path


def _write_candidates(path, fields, prefix, evaluations, result, constraints) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="raise")
        writer.writeheader()
        for index, evaluation in enumerate(evaluations, start=1):
            writer.writerow(_candidate_row(f"{prefix}_{index:04d}", evaluation, result, constraints))


def _candidate_row(candidate_id, evaluation: ProblemEvaluation, result, constraints):
    domain = _stage0_evaluation(evaluation)
    workspace = domain.workspace
    box = list(evaluation.box_violations.values())
    named = {("equality", item.name): item for item in evaluation.equality_results}
    named.update({("inequality", item.name): item for item in evaluation.inequality_results})
    maxima = box + [item.maximum_violation for item in evaluation.equality_results]
    maxima += [item.maximum_violation for item in evaluation.inequality_results]
    positives = box + [item.maximum_violation for item in evaluation.equality_results]
    positives += [max(value, 0.0) for item in evaluation.inequality_results for value in item.residuals]
    row = {
        "candidate_id": candidate_id,
        **evaluation.variable_values,
        **evaluation.raw_objective_values,
        "closure_input_workspace_rad": workspace.closure_input_workspace_rad,
        "usable_input_workspace_rad": workspace.usable_input_workspace_rad,
        "distal_path_length_mm": workspace.distal_path_length_mm,
        "distal_x_excursion_mm": workspace.distal_x_excursion_mm,
        "dorsal_z_excursion_mm": workspace.dorsal_z_excursion_mm,
        "first_unusable_index": "" if workspace.first_unusable_index is None else workspace.first_unusable_index,
        "limiting_reasons": ";".join(workspace.limiting_reasons),
        "kinematic_failure_reason": domain.motion.failure_reason or "",
        "feasible": evaluation.feasible,
        "overall_max_constraint_violation": max(maxima, default=0.0),
        "overall_sum_positive_constraint_violation": math.fsum(v for v in positives if v > 0.0),
    }
    row.update({
        f"constraint_{name}_max_violation": named[(kind, name)].maximum_violation
        for kind, name in constraints
    })
    return row


def _stage0_evaluation(evaluation: ProblemEvaluation) -> LinkageStage0Evaluation:
    domain = evaluation.domain_evaluation
    if not isinstance(domain, LinkageStage0Evaluation):
        raise TypeError("Stage-0 result must contain LinkageStage0Evaluation")
    return domain


def _violation_summary(values: list[float], denominator: int) -> dict[str, float | int | None]:
    positive = [value for value in values if value > 0.0]
    return {
        "violated_candidate_count": len(positive),
        "violation_fraction": len(positive) / denominator if values else 0.0,
        "mean_positive_max_violation": math.fsum(positive) / len(positive) if positive else None,
        "maximum_violation": max(values, default=0.0),
    }


def _finite_summary(values: Iterable[float]) -> dict[str, float | None]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return {"min": None, "median": None, "max": None}
    return {"min": min(finite), "median": statistics.median(finite), "max": max(finite)}


def _objective_summary(values: list[float]) -> dict[str, float | int | None]:
    finite = [value for value in values if math.isfinite(value)]
    return {
        "finite_count": len(finite),
        "nonfinite_count": len(values) - len(finite),
        **_finite_summary(finite),
    }


__all__ = ["export_linkage_stage0_results", "summarize_linkage_stage0"]
