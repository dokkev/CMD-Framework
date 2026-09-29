import csv
from dataclasses import replace
import json
import math
import subprocess
import sys

import numpy as np
import pytest

from cmd.morphology import BASELINE_LINKAGE_V0, LinkageMotionEvaluator
from cmd.morphology.linkage import evaluate_linkage_workspace
from cmd.optimization import BoxVariable
from cmd.optimization.linkage import (
    LinkageBaselineParameterization,
    LinkageStage0Config,
    LinkageStage0Evaluation,
    Stage0Evaluator,
    build_linkage_stage0_problem,
    run_linkage_stage0,
)
from cmd.optimization.solvers.pymoo import NSGA2Config, NSGA2RunResult, PymooProblemAdapter
from scripts.optimize.linkage_report import export_linkage_stage0_results


def _seed_and_test_bounds():
    parameterization = LinkageBaselineParameterization()
    seed = parameterization.encode(BASELINE_LINKAGE_V0)
    # TEST FIXTURE ONLY: artificial V0-centered bounds, not physical design limits.
    variables = tuple(
        BoxVariable(name, value - 1.0, value + 1.0)
        for name, value in zip(parameterization.variable_names, seed)
    )
    return parameterization, seed, variables


def _config(*, minimum_anchor_offset=0.0):
    from cmd.morphology import LinkageWorkspaceCriteria

    return LinkageStage0Config(
        core_input_workspace_rad=0.4,
        minimum_required_input_workspace_rad=0.4,
        search_horizon_rad=0.8,
        sample_count=5,
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


def test_stage0_keeps_xz_variables_and_canonical_objective_order():
    parameterization, _, variables = _seed_and_test_bounds()
    problem = build_linkage_stage0_problem(variables, _config())
    assert problem.parameterization.variable_names == parameterization.variable_names
    assert tuple(obj.name for obj in problem.objectives) == (
        "worst_case_transmission_quality",
        "coordination_ratio_variation",
        "distal_endpoint_excursion_mm",
    )
    assert problem.evaluator.evaluate(BASELINE_LINKAGE_V0.parameters).motion.input_flexion_rad == pytest.approx(
        (0.0, 0.2, 0.4, 0.6, 0.8)
    )


def test_reusable_motion_accepts_explicit_nonuniform_samples():
    motion = LinkageMotionEvaluator((0.0, 0.13, 0.41)).evaluate(BASELINE_LINKAGE_V0.parameters)
    assert motion.input_flexion_rad == (0.0, 0.13, 0.41)
    assert len(motion.dq2_flexion_dq1_flexion) == 2
    assert len(motion.dq3_flexion_dq1_flexion) == 2


def test_optimization_linkage_package_import_does_not_load_pymoo():
    completed = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import cmd.optimization.linkage; "
            "assert not any(name == 'pymoo' or name.startswith('pymoo.') for name in sys.modules)",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0


def test_stage0_rejects_nonpositive_bc_lower_bound():
    _, seed, variables = _seed_and_test_bounds()
    invalid = list(variables)
    invalid[9] = BoxVariable("bc_length_mm", 0.0, seed[9] + 1.0)
    with pytest.raises(ValueError, match="bc_length_mm lower bound.*greater than zero"):
        build_linkage_stage0_problem(invalid, _config())


def test_stage0_seed_is_projected_and_rejected_outside_caller_bounds(monkeypatch):
    parameterization, seed, variables = _seed_and_test_bounds()
    captured = {}

    def capture_run(problem, prototype_x, config, initial_designs=()):
        captured["prototype"] = tuple(prototype_x)
        captured["initial_designs"] = tuple(tuple(item) for item in initial_designs)
        return NSGA2RunResult(config, "test", "termination", 0, 0, (), ())

    monkeypatch.setattr("cmd.optimization.linkage.stage0.run_nsga2", capture_run)
    result = run_linkage_stage0(variables, _config(), NSGA2Config(4, 1, 3))
    assert captured["prototype"] == pytest.approx(seed)
    assert captured["initial_designs"] == (pytest.approx(seed),)
    assert result.include_baseline_seed

    outside = list(variables)
    outside[0] = BoxVariable(variables[0].name, seed[0] + 0.1, seed[0] + 1.1)
    with pytest.raises(ValueError, match="BASELINE_LINKAGE_V0.*outside"):
        run_linkage_stage0(outside, _config(), NSGA2Config(4, 1, 3))
    assert parameterization.encode(BASELINE_LINKAGE_V0) == pytest.approx(seed)


def test_stage0_can_exclude_baseline_seed_and_records_choice(monkeypatch):
    _, _, variables = _seed_and_test_bounds()
    captured = {}

    def capture_run(problem, prototype_x, config, initial_designs=()):
        captured["initial_designs"] = tuple(initial_designs)
        return NSGA2RunResult(config, "test", "termination", 0, 0, (), ())

    monkeypatch.setattr("cmd.optimization.linkage.stage0.run_nsga2", capture_run)
    result = run_linkage_stage0(
        variables, _config(), NSGA2Config(4, 1, 3),
        include_baseline_seed=False, study_name="test_without_seed",
    )
    assert captured["initial_designs"] == ()
    assert result.study_name == "test_without_seed"
    assert result.include_baseline_seed is False


def test_incomplete_core_uses_finite_solver_transport_and_finite_constraints(monkeypatch):
    parameterization, seed, variables = _seed_and_test_bounds()
    problem = build_linkage_stage0_problem(variables, _config())
    parameters = parameterization.decode(dict(zip(parameterization.variable_names, seed)))
    complete = problem.evaluator.evaluate(parameters)
    motion = replace(
        complete.motion,
        poses=(), unwrapped_q2_rad=(), unwrapped_q3_rad=(),
        q2_flexion_rad=(), q3_flexion_rad=(),
        dq2_flexion_dq1_flexion=(), dq3_flexion_dq1_flexion=(),
        transmission_angle_loop_1_rad=(), transmission_angle_loop_2_rad=(),
        transmission_quality_loop_1=(), transmission_quality_loop_2=(),
        complete_sweep=False, solved_sample_count=0, failure_index=0,
        failure_reason="synthetic incomplete core",
    )
    incomplete = LinkageStage0Evaluation(
        motion,
        evaluate_linkage_workspace(motion, _config().workspace_criteria),
        complete.core_sample_count,
    )
    monkeypatch.setattr(Stage0Evaluator, "evaluate", lambda self, _: incomplete)
    adapter = PymooProblemAdapter(problem, seed, nonfinite_objective_penalty=1.0e10)
    out = {}
    adapter._evaluate(np.asarray(seed), out)
    assert adapter.last_evaluation.raw_objective_values["coordination_ratio_variation"] == math.inf
    assert out["F"][1] == 1.0e10
    assert np.isfinite(out["F"]).all() and np.isfinite(out["G"]).all()


def test_tiny_run_and_export_support_empty_feasible_pareto(tmp_path):
    _, _, variables = _seed_and_test_bounds()
    result = run_linkage_stage0(
        variables, _config(minimum_anchor_offset=1000.0), NSGA2Config(6, 2, 11)
    )
    assert result.solver_result.final_population
    assert len(result.solver_result.final_population) == 6
    assert not result.solver_result.pareto_candidates
    assert all(not item.feasible for item in result.solver_result.final_population)

    candidate = result.solver_result.final_population[0]
    failed = replace(candidate.domain_evaluation.motion, failure_reason="synthetic closure diagnostic")
    domain = replace(candidate.domain_evaluation, motion=failed)
    population = (replace(candidate, domain_evaluation=domain), *result.solver_result.final_population[1:])
    result = replace(result, solver_result=replace(result.solver_result, final_population=population))
    run_path, pareto_path, population_path = export_linkage_stage0_results(
        result, tmp_path, source_git_commit="test-commit"
    )
    metadata = json.loads(run_path.read_text(encoding="utf-8"))
    with pareto_path.open(newline="", encoding="utf-8") as stream:
        pareto_rows = list(csv.DictReader(stream))
    with population_path.open(newline="", encoding="utf-8") as stream:
        final_rows = list(csv.DictReader(stream))
    assert metadata["solver"] == "pymoo_nsga2"
    assert metadata["study_name"] is None
    assert metadata["include_baseline_seed"] is True
    assert metadata["initialization"] == {
        "baseline_seed_included": True, "remaining_population_method": "lhs"
    }
    assert metadata["source_git_commit"] == "test-commit"
    assert pareto_rows == []
    assert len(final_rows) == 6
    assert final_rows[0]["kinematic_failure_reason"] == "synthetic closure diagnostic"
    assert "constraint_minimum_anchor_dorsal_offset_max_violation" in final_rows[0]
