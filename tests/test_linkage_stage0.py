import csv
from dataclasses import replace
import json
import math

import numpy as np
import pytest

from cmd.morphology import BASELINE_LINKAGE_V0
from cmd.optimization import BoxVariable
from cmd.optimization.linkage import (
    LinkageBaselineConstraintConfig,
    LinkageBaselineParameterization,
    LinkageSweepConfig,
    LinkageSweepEvaluator,
    LinkageWorkspaceCriteria,
    build_linkage_baseline_problem,
)
from cmd.optimization.linkage.stage0 import (
    export_linkage_stage0_results,
    run_linkage_stage0,
)
from cmd.optimization.solvers.pymoo import (
    NSGA2Config,
    NSGA2RunResult,
    PymooProblemAdapter,
)


def _seed_and_test_bounds():
    parameterization = LinkageBaselineParameterization()
    seed = parameterization.encode(BASELINE_LINKAGE_V0)
    # These V0 +/- 1 bounds are test fixtures only, not physical design limits.
    variables = tuple(
        BoxVariable(name, value - 1.0, value + 1.0)
        for name, value in zip(parameterization.variable_names, seed)
    )
    return parameterization, seed, variables


def _constraint_config(*, minimum_anchor_offset=0.0):
    return LinkageBaselineConstraintConfig(
        core_input_workspace_rad=0.4,
        minimum_required_input_workspace_rad=0.4,
        minimum_anchor_dorsal_offset_mm=minimum_anchor_offset,
        q2_minimum_endpoint_flexion_rad=0.0,
        q3_minimum_endpoint_flexion_rad=0.0,
        workspace_criteria=LinkageWorkspaceCriteria(
            minimum_transmission_margin_rad=0.0,
            q2_minimum_motion_ratio=0.0,
            q3_minimum_motion_ratio=0.0,
            q2_maximum_motion_ratio=100.0,
            q3_maximum_motion_ratio=100.0,
        ),
    )


def _sweep_config():
    return LinkageSweepConfig(search_horizon_rad=0.8, sample_count=5)


def test_stage0_keeps_xz_variables_and_canonical_objective_order() -> None:
    parameterization, _, variables = _seed_and_test_bounds()
    problem = build_linkage_baseline_problem(
        variables,
        _constraint_config(),
        _sweep_config(),
    )

    assert problem.parameterization.variable_names == parameterization.variable_names
    assert all(name.endswith(("_x_mm", "_z_mm", "bc_length_mm")) for name in parameterization.variable_names)
    assert tuple(objective.name for objective in problem.objectives) == (
        "worst_case_transmission_quality",
        "coordination_ratio_variation",
        "distal_endpoint_excursion_mm",
    )


def test_stage0_rejects_nonpositive_bc_lower_bound() -> None:
    _, seed, variables = _seed_and_test_bounds()
    invalid = list(variables)
    invalid[9] = BoxVariable("bc_length_mm", 0.0, seed[9] + 1.0)

    with pytest.raises(ValueError, match="bc_length_mm lower bound.*greater than zero"):
        build_linkage_baseline_problem(
            invalid,
            _constraint_config(),
            _sweep_config(),
        )


def test_stage0_seed_is_projected_and_rejected_if_outside_caller_bounds(monkeypatch) -> None:
    parameterization, seed, variables = _seed_and_test_bounds()
    captured = {}

    def capture_run(problem, prototype_x, config, initial_designs=()):
        captured["prototype"] = tuple(prototype_x)
        captured["initial_designs"] = tuple(tuple(item) for item in initial_designs)
        return NSGA2RunResult(config, "test", "test termination", 0, 0, (), ())

    monkeypatch.setattr("cmd.optimization.linkage.stage0.run_nsga2", capture_run)
    run_linkage_stage0(
        variables,
        _constraint_config(),
        _sweep_config(),
        NSGA2Config(population_size=4, generations=1, seed=3),
    )
    assert captured["prototype"] == pytest.approx(seed)
    assert len(captured["initial_designs"]) == 1
    assert captured["initial_designs"][0] == pytest.approx(seed)

    outside = list(variables)
    outside[0] = BoxVariable(variables[0].name, seed[0] + 0.1, seed[0] + 1.1)
    with pytest.raises(ValueError, match="projected BASELINE_LINKAGE_V0.*outside"):
        run_linkage_stage0(
            outside,
            _constraint_config(),
            _sweep_config(),
            NSGA2Config(population_size=4, generations=1, seed=3),
        )
    assert parameterization.encode(BASELINE_LINKAGE_V0) == pytest.approx(seed)


def test_stage0_can_exclude_baseline_seed_and_records_the_choice(monkeypatch) -> None:
    _, _, variables = _seed_and_test_bounds()
    captured = {}

    def capture_run(problem, prototype_x, config, initial_designs=()):
        captured["initial_designs"] = tuple(initial_designs)
        return NSGA2RunResult(config, "test", "test termination", 0, 0, (), ())

    monkeypatch.setattr("cmd.optimization.linkage.stage0.run_nsga2", capture_run)
    result = run_linkage_stage0(
        variables,
        _constraint_config(),
        _sweep_config(),
        NSGA2Config(population_size=4, generations=1, seed=3),
        include_baseline_seed=False,
        study_name="test_without_seed",
    )

    assert captured["initial_designs"] == ()
    assert result.study_name == "test_without_seed"
    assert result.include_baseline_seed is False


def test_incomplete_core_uses_finite_solver_transport_and_finite_constraints(monkeypatch) -> None:
    parameterization, seed, variables = _seed_and_test_bounds()
    problem = build_linkage_baseline_problem(variables, _constraint_config(), _sweep_config())
    parameters = parameterization.decode(dict(zip(parameterization.variable_names, seed)))
    complete = LinkageSweepEvaluator(_sweep_config(), 0.4).evaluate(parameters)
    incomplete = replace(
        complete,
        poses=(),
        unwrapped_q2_rad=(),
        unwrapped_q3_rad=(),
        q2_flexion_rad=(),
        q3_flexion_rad=(),
        dq2_flexion_dq1_flexion=(),
        dq3_flexion_dq1_flexion=(),
        transmission_angle_loop_1_rad=(),
        transmission_angle_loop_2_rad=(),
        transmission_quality_loop_1=(),
        transmission_quality_loop_2=(),
        complete_workspace=False,
        solved_sample_count=0,
        failure_index=0,
        failure_reason="synthetic incomplete core",
    )
    monkeypatch.setattr(LinkageSweepEvaluator, "evaluate", lambda self, _: incomplete)
    adapter = PymooProblemAdapter(problem, seed, nonfinite_objective_penalty=1.0e10)
    out = {}

    adapter._evaluate(np.asarray(seed), out)

    assert adapter.last_evaluation.raw_objective_values[
        "coordination_ratio_variation"
    ] == math.inf
    assert out["F"][1] == 1.0e10
    assert np.isfinite(out["F"]).all()
    assert np.isfinite(out["G"]).all()


def test_tiny_stage0_run_and_export_support_empty_feasible_pareto_set(tmp_path) -> None:
    _, _, variables = _seed_and_test_bounds()
    result = run_linkage_stage0(
        variables,
        _constraint_config(minimum_anchor_offset=1000.0),
        _sweep_config(),
        NSGA2Config(population_size=6, generations=2, seed=11),
    )

    assert result.solver_result.final_population
    assert len(result.solver_result.final_population) == 6
    assert not result.solver_result.pareto_candidates
    assert all(not item.feasible for item in result.solver_result.final_population)

    first_candidate = result.solver_result.final_population[0]
    failed_domain = replace(
        first_candidate.domain_evaluation,
        failure_reason="synthetic closure diagnostic",
    )
    first_candidate = replace(first_candidate, domain_evaluation=failed_domain)
    solver_result = replace(
        result.solver_result,
        final_population=(first_candidate, *result.solver_result.final_population[1:]),
    )
    result = replace(result, solver_result=solver_result)

    run_path, pareto_path, population_path = export_linkage_stage0_results(
        result,
        tmp_path,
        source_git_commit="test-commit",
    )
    metadata = json.loads(run_path.read_text(encoding="utf-8"))
    with pareto_path.open(newline="", encoding="utf-8") as stream:
        pareto_rows = list(csv.DictReader(stream))
    with pareto_path.open(newline="", encoding="utf-8") as stream:
        pareto_headers = next(csv.reader(stream))
    with population_path.open(newline="", encoding="utf-8") as stream:
        final_rows = list(csv.DictReader(stream))

    assert metadata["solver"] == "pymoo_nsga2"
    assert metadata["study_name"] is None
    assert metadata["include_baseline_seed"] is True
    assert metadata["initialization"] == {
        "baseline_seed_included": True,
        "remaining_population_method": "lhs",
    }
    assert metadata["source_git_commit"] == "test-commit"
    assert metadata["pymoo_version"]
    assert metadata["pareto_candidate_count"] == 0
    assert metadata["final_population_candidate_count"] == 6
    assert metadata["feasible_final_population_candidate_count"] == 0
    assert metadata["workspace_criteria"]["q2_maximum_motion_ratio"] == 100.0
    assert "coordination_ratio_variation" in pareto_headers
    assert pareto_rows == []
    assert len(final_rows) == 6
    assert "overall_max_constraint_violation" in final_rows[0]
    assert "kinematic_failure_reason" in final_rows[0]
    assert final_rows[0]["kinematic_failure_reason"] == "synthetic closure diagnostic"
    for row, evaluation in zip(final_rows, result.solver_result.final_population):
        expected_sum = math.fsum(
            [*evaluation.box_violations.values()]
            + [item.maximum_violation for item in evaluation.equality_results]
            + [
                max(residual, 0.0)
                for item in evaluation.inequality_results
                for residual in item.residuals
            ]
        )
        assert float(row["overall_sum_positive_constraint_violation"]) == pytest.approx(
            expected_sum
        )
