import json
import math
from dataclasses import replace

from cmd.morphology import (
    BASELINE_LINKAGE_V0,
    LinkageWorkspaceCriteria,
    evaluate_linkage_workspace,
)
from cmd.optimization import BoxVariable
from cmd.optimization.linkage import (
    LinkageBaselineParameterization,
    LinkageStage0Config,
    LinkageStage0Evaluation,
    LinkageStage0Result,
    build_linkage_stage0_problem,
)
from cmd.optimization.solvers.pymoo import NSGA2Config, NSGA2RunResult
from scripts.optimize.linkage_report import summarize_linkage_stage0


def _synthetic_result() -> LinkageStage0Result:
    parameterization = LinkageBaselineParameterization()
    seed = parameterization.encode(BASELINE_LINKAGE_V0)
    # TEST FIXTURE ONLY: artificial bounds around V0 are not physical design limits.
    variables = tuple(
        BoxVariable(name, value - 1.0, value + 1.0)
        for name, value in zip(parameterization.variable_names, seed)
    )
    criteria = LinkageWorkspaceCriteria(
        minimum_transmission_margin_rad=0.0,
        q2_minimum_motion_ratio=0.0,
        q3_minimum_motion_ratio=0.0,
        q2_maximum_motion_ratio=100.0,
        q3_maximum_motion_ratio=100.0,
    )
    stage0_config = LinkageStage0Config(
        core_input_workspace_rad=0.4,
        minimum_required_input_workspace_rad=0.4,
        search_horizon_rad=0.8,
        sample_count=5,
        minimum_anchor_dorsal_offset_mm=0.0,
        q2_minimum_endpoint_flexion_rad=0.0,
        q3_minimum_endpoint_flexion_rad=0.0,
        workspace_criteria=criteria,
    )
    problem = build_linkage_stage0_problem(variables, stage0_config)
    base = problem.evaluate(seed)

    passing_constraints = tuple(
        replace(item, maximum_violation=0.0, satisfied=True)
        for item in base.inequality_results
    )
    passing = replace(base, inequality_results=passing_constraints)
    anchor_index = next(
        index
        for index, item in enumerate(passing.inequality_results)
        if item.name == "minimum_anchor_dorsal_offset"
    )

    def with_anchor_violation(evaluation, violation):
        constraints = list(evaluation.inequality_results)
        constraints[anchor_index] = replace(
            constraints[anchor_index],
            maximum_violation=violation,
            satisfied=violation == 0.0,
        )
        return replace(evaluation, inequality_results=tuple(constraints))

    violating_two = with_anchor_violation(passing, 2.0)
    violating_four = with_anchor_violation(passing, 4.0)
    domain = violating_two.domain_evaluation
    incomplete_motion = replace(
        domain.motion,
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
        complete_sweep=False,
        solved_sample_count=0,
        failure_index=0,
        failure_reason="synthetic circle closure failure",
    )
    incomplete_domain = replace(
        domain,
        motion=incomplete_motion,
        workspace=evaluate_linkage_workspace(incomplete_motion, stage0_config.workspace_criteria),
    )
    incomplete = replace(
        violating_two,
        domain_evaluation=incomplete_domain,
        raw_objective_values={
            **violating_two.raw_objective_values,
            "coordination_ratio_variation": math.inf,
        },
        minimization_objective_values={
            **violating_two.minimization_objective_values,
            "coordination_ratio_variation": math.inf,
        },
    )
    box_violations = dict(violating_four.box_violations)
    first_variable = variables[0].name
    box_violations[first_variable] = 0.25
    violating_four = replace(violating_four, box_violations=box_violations)

    solver_result = NSGA2RunResult(
        config=NSGA2Config(population_size=3, generations=1, seed=4),
        pymoo_version="test",
        termination="test",
        generation_count=1,
        evaluation_count=3,
        pareto_candidates=(passing,),
        final_population=(passing, incomplete, violating_four),
    )
    return LinkageStage0Result(
        problem=problem,
        stage0_config=stage0_config,
        solver_result=solver_result,
        study_name="diagnostics_test",
        include_baseline_seed=True,
    )


def test_stage0_diagnostics_summarize_population_without_samplewise_overcount() -> None:
    diagnostics = summarize_linkage_stage0(_synthetic_result())
    anchor = diagnostics["constraints"]["minimum_anchor_dorsal_offset"]
    box = next(iter(diagnostics["box_variables"].values()))
    objective = diagnostics["objectives"]["coordination_ratio_variation"]
    closure = diagnostics["workspace"]["closure_input_workspace_rad"]
    usable = diagnostics["workspace"]["usable_input_workspace_rad"]

    assert diagnostics["final_population_count"] == 3
    assert diagnostics["feasible_count"] == 1
    assert diagnostics["feasible_fraction"] == 1.0 / 3.0
    assert diagnostics["pareto_candidate_count"] == 1
    assert anchor["violated_candidate_count"] == 2
    assert anchor["violation_fraction"] == 2.0 / 3.0
    assert anchor["mean_positive_max_violation"] == 3.0
    assert anchor["maximum_violation"] == 4.0
    assert box["violated_candidate_count"] == 1
    assert box["mean_positive_max_violation"] == 0.25
    assert box["maximum_violation"] == 0.25
    assert objective["finite_count"] == 2
    assert objective["nonfinite_count"] == 1
    assert objective["min"] == objective["median"] == objective["max"]
    assert closure["min"] == 0.0
    assert closure["max"] > closure["min"]
    assert closure["min"] <= closure["median"] <= closure["max"]
    assert usable["min"] == 0.0
    assert usable["min"] <= usable["median"] <= usable["max"]
    assert diagnostics["limiting_reasons"]["kinematic_failure"] == {
        "candidate_count": 1,
        "fraction": 1.0 / 3.0,
    }

    json.dumps(diagnostics, allow_nan=False)
