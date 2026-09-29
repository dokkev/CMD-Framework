"""Descriptive diagnostics for a completed Stage-0 linkage run."""

from __future__ import annotations

import math
import statistics
from collections.abc import Iterable

from .evaluator import LinkageEvaluation
from .stage0 import LinkageStage0Result
from .workspace import evaluate_linkage_workspace


_WORKSPACE_REASON_ORDER = (
    "kinematic_failure",
    "transmission_loop_1",
    "transmission_loop_2",
    "q2_minimum_motion_ratio",
    "q3_minimum_motion_ratio",
    "q2_maximum_motion_ratio",
    "q3_maximum_motion_ratio",
    "q2_motion_ratio_unavailable",
    "q3_motion_ratio_unavailable",
    "search_horizon",
)


def summarize_linkage_stage0(result: LinkageStage0Result) -> dict[str, object]:
    """Return JSON-ready population, constraint, objective, and workspace summaries."""

    if not isinstance(result, LinkageStage0Result):
        raise TypeError("result must be a LinkageStage0Result")
    population = result.solver_result.final_population
    population_count = len(population)
    feasible_count = sum(evaluation.feasible for evaluation in population)
    denominator = population_count or 1

    constraint_keys = tuple(
        ("equality", constraint.name) for constraint in result.problem.equalities
    ) + tuple(
        ("inequality", constraint.name) for constraint in result.problem.inequalities
    )
    constraint_values: dict[tuple[str, str], list[float]] = {
        key: [] for key in constraint_keys
    }
    box_values = {
        variable.name: [] for variable in result.problem.variables
    }
    objective_values = {
        objective.name: [] for objective in result.problem.objectives
    }
    closure_workspaces: list[float] = []
    usable_workspaces: list[float] = []
    limiting_reason_counts = {reason: 0 for reason in _WORKSPACE_REASON_ORDER}

    for evaluation in population:
        for name, value in evaluation.box_violations.items():
            box_values[name].append(value)
        for item in evaluation.equality_results:
            constraint_values[("equality", item.name)].append(item.maximum_violation)
        for item in evaluation.inequality_results:
            constraint_values[("inequality", item.name)].append(item.maximum_violation)
        for name in objective_values:
            objective_values[name].append(evaluation.raw_objective_values[name])

        domain = evaluation.domain_evaluation
        if not isinstance(domain, LinkageEvaluation):
            raise TypeError("Stage-0 evaluation must contain a LinkageEvaluation")
        workspace = evaluate_linkage_workspace(
            domain,
            result.constraint_config.workspace_criteria,
        )
        closure_workspaces.append(workspace.closure_input_workspace_rad)
        usable_workspaces.append(workspace.usable_input_workspace_rad)
        for reason in dict.fromkeys(workspace.limiting_reasons):
            limiting_reason_counts.setdefault(reason, 0)
            limiting_reason_counts[reason] += 1

    constraint_summaries = {
        name: _violation_summary(constraint_values[(kind, name)], denominator)
        for kind, name in constraint_keys
    }
    box_summaries = {
        name: _violation_summary(values, denominator)
        for name, values in box_values.items()
    }
    reason_order = _WORKSPACE_REASON_ORDER + tuple(
        sorted(set(limiting_reason_counts) - set(_WORKSPACE_REASON_ORDER))
    )

    return {
        "final_population_count": population_count,
        "feasible_count": feasible_count,
        "feasible_fraction": feasible_count / denominator if population_count else 0.0,
        "pareto_candidate_count": len(result.solver_result.pareto_candidates),
        "constraints": constraint_summaries,
        "box_variables": box_summaries,
        "workspace": {
            "closure_input_workspace_rad": _finite_summary(closure_workspaces),
            "usable_input_workspace_rad": _finite_summary(usable_workspaces),
        },
        "objectives": {
            name: _objective_summary(values)
            for name, values in objective_values.items()
        },
        "limiting_reasons": {
            reason: {
                "candidate_count": limiting_reason_counts[reason],
                "fraction": limiting_reason_counts[reason] / denominator
                if population_count
                else 0.0,
            }
            for reason in reason_order
        },
    }


def _violation_summary(values: list[float], denominator: int) -> dict[str, float | int | None]:
    positive = [value for value in values if value > 0.0]
    return {
        "violated_candidate_count": len(positive),
        "violation_fraction": len(positive) / denominator if values else 0.0,
        "mean_positive_max_violation": (
            math.fsum(positive) / len(positive) if positive else None
        ),
        "maximum_violation": max(values, default=0.0),
    }


def _finite_summary(values: Iterable[float]) -> dict[str, float | None]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return {"min": None, "median": None, "max": None}
    return {
        "min": min(finite),
        "median": statistics.median(finite),
        "max": max(finite),
    }


def _objective_summary(values: list[float]) -> dict[str, float | int | None]:
    finite = [value for value in values if math.isfinite(value)]
    summary = _finite_summary(finite)
    return {
        "finite_count": len(finite),
        "nonfinite_count": len(values) - len(finite),
        "min": summary["min"],
        "median": summary["median"],
        "max": summary["max"],
    }


__all__ = ["summarize_linkage_stage0"]
