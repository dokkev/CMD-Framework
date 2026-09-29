from dataclasses import replace
import math
import pytest

from cmd.morphology import (
    BASELINE_LINKAGE_V0, FingerLinkage, LinkageMotionEvaluator, LinkageWorkspaceCriteria,
    PointXZ, evaluate_linkage_workspace,
)
from cmd.morphology.linkage_kinematics import KinematicClosureError, LinkageKinematics
from cmd.morphology.linkage_motion import differentiate_flexion_samples, unwrap_principal_angles
from cmd.optimization import BoxVariable
from cmd.optimization.constraint import inequality_result
from cmd.optimization.linkage.constraints import (
    CoreWorkspaceKinematicValidity,
    MaximumMotionRatio,
    MinimumAnchorDorsalOffset,
    MinimumFlexionAtInput,
    MinimumTransmissionMargin,
    MinimumUsableInputWorkspace,
    MonotonicFlexion,
)
from cmd.optimization.linkage.objectives import (
    CoordinationRatioVariation,
    DistalEndpointExcursion,
    WorstCaseTransmissionQuality,
)
from cmd.optimization.linkage.parameterization import LinkageBaselineParameterization
from cmd.optimization.linkage.stage0 import (
    LinkageStage0Config,
    LinkageStage0Evaluation,
    Stage0Evaluator,
    build_linkage_stage0_problem,
    stage0_input_samples,
)


def baseline_seed() -> tuple[LinkageBaselineParameterization, tuple[float, ...]]:
    parameterization = LinkageBaselineParameterization()
    return parameterization, parameterization.encode(BASELINE_LINKAGE_V0)


def baseline_evaluation(
    *,
    search_horizon_rad: float = 0.8,
    core_input_workspace_rad: float = 0.4,
    sample_count: int = 5,
):
    parameterization, encoded = baseline_seed()
    parameters = parameterization.decode(
        dict(zip(parameterization.variable_names, encoded))
    )
    config = baseline_constraint_config(
        core_input_workspace_rad=core_input_workspace_rad,
        search_horizon_rad=search_horizon_rad,
        sample_count=sample_count,
    )
    return LinkageMotionEvaluator(stage0_input_samples(config)).evaluate(parameters)


def permissive_criteria(
    *,
    minimum_transmission_margin_rad: float = 0.0,
    q2_minimum_motion_ratio: float = 0.0,
    q3_minimum_motion_ratio: float = 0.0,
    q2_maximum_motion_ratio: float = 100.0,
    q3_maximum_motion_ratio: float = 100.0,
) -> LinkageWorkspaceCriteria:
    return LinkageWorkspaceCriteria(
        minimum_transmission_margin_rad=minimum_transmission_margin_rad,
        q2_minimum_motion_ratio=q2_minimum_motion_ratio,
        q3_minimum_motion_ratio=q3_minimum_motion_ratio,
        q2_maximum_motion_ratio=q2_maximum_motion_ratio,
        q3_maximum_motion_ratio=q3_maximum_motion_ratio,
    )


def baseline_constraint_config(
    *,
    core_input_workspace_rad: float = 0.4,
    minimum_required_input_workspace_rad: float | None = None,
    criteria: LinkageWorkspaceCriteria | None = None,
    search_horizon_rad: float = 0.8,
    sample_count: int = 5,
) -> LinkageStage0Config:
    return LinkageStage0Config(
        core_input_workspace_rad=core_input_workspace_rad,
        minimum_required_input_workspace_rad=(
            core_input_workspace_rad
            if minimum_required_input_workspace_rad is None
            else minimum_required_input_workspace_rad
        ),
        search_horizon_rad=search_horizon_rad,
        sample_count=sample_count,
        minimum_anchor_dorsal_offset_mm=0.0,
        q2_minimum_endpoint_flexion_rad=0.0,
        q3_minimum_endpoint_flexion_rad=0.0,
        workspace_criteria=criteria or permissive_criteria(),
    )


def baseline_problem(
    *,
    core_input_workspace_rad: float = 0.4,
    minimum_required_input_workspace_rad: float | None = None,
    search_horizon_rad: float = 0.8,
    sample_count: int = 5,
    criteria: LinkageWorkspaceCriteria | None = None,
):
    parameterization, encoded = baseline_seed()
    variables = tuple(
        BoxVariable(name, value - 1.0, value + 1.0)
        for name, value in zip(parameterization.variable_names, encoded)
    )
    config = baseline_constraint_config(
        core_input_workspace_rad=core_input_workspace_rad,
        minimum_required_input_workspace_rad=minimum_required_input_workspace_rad,
        criteria=criteria,
        search_horizon_rad=search_horizon_rad,
        sample_count=sample_count,
    )
    problem = build_linkage_stage0_problem(variables, config)
    return problem, encoded


def healthy_workspace_metrics(evaluation):
    count = evaluation.solved_sample_count
    return replace(
        evaluation,
        dq2_flexion_dq1_flexion=(1.0,) * max(0, count - 1),
        dq3_flexion_dq1_flexion=(1.0,) * max(0, count - 1),
        transmission_angle_loop_1_rad=(math.pi / 2.0,) * count,
        transmission_angle_loop_2_rad=(math.pi / 2.0,) * count,
    )


def stage0_context(motion, criteria=None):
    criteria = criteria or permissive_criteria()
    return LinkageStage0Evaluation(motion, evaluate_linkage_workspace(motion, criteria))


def patch_solver_to_fail_after(monkeypatch, success_limit: int) -> None:
    original_solve = LinkageKinematics.solve
    counts: dict[int, int] = {}

    def fail_after_limit(self, q1_rad):
        solver_id = id(self)
        solved = counts.get(solver_id, 0)
        if solved >= success_limit:
            raise KinematicClosureError("synthetic post-core closure failure")
        counts[solver_id] = solved + 1
        return original_solve(self, q1_rad)

    monkeypatch.setattr(LinkageKinematics, "solve", fail_after_limit)


def test_parameterization_preserves_twelve_dimensional_exact_manifold() -> None:
    parameterization, encoded = baseline_seed()
    assert parameterization.variable_names == (
        "o_prime_from_o_x_mm",
        "o_prime_from_o_z_mm",
        "a_from_o_x_mm",
        "a_from_o_z_mm",
        "a_prime_from_o_x_mm",
        "a_prime_from_o_z_mm",
        "b_from_o_x_mm",
        "b_prime_from_o_x_mm",
        "b_prime_from_o_z_mm",
        "bc_length_mm",
        "c_prime_from_o_x_mm",
        "c_prime_from_o_z_mm",
    )
    parameters = parameterization.decode(
        dict(zip(parameterization.variable_names, encoded))
    )
    linkage = FingerLinkage(parameters)
    reference = linkage.reference_positions

    assert len(encoded) == 12
    assert reference.b.z_mm == 0.0
    assert reference.c.z_mm == 0.0
    assert reference.c.x_mm - reference.b.x_mm == pytest.approx(
        parameters.bc_length_mm
    )
    assert reference.c.x_mm > 0.0


def test_baseline_encoding_projects_only_tiny_b_z_residual_without_mutation() -> None:
    parameterization, encoded = baseline_seed()
    original = BASELINE_LINKAGE_V0.reference_positions
    decoded = parameterization.decode(
        dict(zip(parameterization.variable_names, encoded))
    )
    projected = FingerLinkage(decoded).reference_positions

    assert 0.0 < parameterization.projection_error_mm(BASELINE_LINKAGE_V0) < 0.002
    assert BASELINE_LINKAGE_V0.reference_positions == original
    for name in ("o", "op", "a", "ap", "bp", "cp"):
        assert getattr(projected, name).distance_to(getattr(original, name)) < 1e-12
    assert projected.b.z_mm == 0.0
    assert projected.c.z_mm == 0.0


@pytest.mark.parametrize("value", [0.0, -0.1, math.nan, math.inf])
def test_search_horizon_must_be_finite_and_positive(value) -> None:
    with pytest.raises(ValueError):
        baseline_constraint_config(search_horizon_rad=value)


@pytest.mark.parametrize("value", [0.0, -0.1, math.nan, math.inf])
def test_core_workspace_must_be_finite_and_positive(value) -> None:
    with pytest.raises(ValueError):
        baseline_constraint_config(core_input_workspace_rad=value)


def test_search_horizon_must_exceed_core_workspace() -> None:
    with pytest.raises(ValueError, match="below search_horizon"):
        baseline_problem(core_input_workspace_rad=0.8, search_horizon_rad=0.8)
    with pytest.raises(ValueError, match="below search_horizon"):
        baseline_problem(core_input_workspace_rad=0.9, search_horizon_rad=0.8)
    with pytest.raises(ValueError, match="must not exceed search_horizon"):
        baseline_problem(minimum_required_input_workspace_rad=0.9)


def test_workspace_criteria_validate_finite_physical_thresholds() -> None:
    with pytest.raises(ValueError):
        permissive_criteria(minimum_transmission_margin_rad=math.nan)
    with pytest.raises(ValueError):
        permissive_criteria(q2_minimum_motion_ratio=-0.1)
    with pytest.raises(ValueError, match="greater than or equal"):
        permissive_criteria(q3_minimum_motion_ratio=2.0, q3_maximum_motion_ratio=1.0)


def test_core_endpoint_is_inserted_exactly_when_uniform_grid_misses_it() -> None:
    evaluation = baseline_evaluation(core_input_workspace_rad=0.35, sample_count=5)

    assert evaluation.input_flexion_rad == pytest.approx(
        (0.0, 0.2, 0.35, 0.4, 0.6, 0.8)
    )
    assert sum(value == 0.35 for value in evaluation.input_flexion_rad) == 1
    assert evaluation.requested_sample_count == 6


def test_baseline_sweep_uses_positive_s_and_canonical_reference_branch() -> None:
    evaluation = baseline_evaluation(sample_count=5)

    assert evaluation.complete_workspace
    assert evaluation.input_flexion_rad[0] == 0.0
    assert evaluation.input_flexion_rad[-1] == 0.8
    assert evaluation.q1_rad == pytest.approx(
        tuple(-value for value in evaluation.input_flexion_rad)
    )
    assert evaluation.poses[0].q1_rad == 0.0
    assert evaluation.q2_flexion_rad[0] == pytest.approx(0.0, abs=1e-12)
    assert evaluation.q3_flexion_rad[0] == pytest.approx(0.0, abs=1e-12)
    assert evaluation.q2_flexion_rad[-1] > 0.0
    assert evaluation.q3_flexion_rad[-1] > 0.0


def test_angle_unwrap_preserves_continuity_across_principal_branch_cut() -> None:
    unwrapped = unwrap_principal_angles((3.0, -3.0, -2.8))

    assert unwrapped == pytest.approx((3.0, 2.0 * math.pi - 3.0, 2.0 * math.pi - 2.8))
    assert all(
        abs(next_value - value) < math.pi
        for value, next_value in zip(unwrapped, unwrapped[1:])
    )


def test_closure_workspace_stops_at_last_successful_sample(monkeypatch) -> None:
    patch_solver_to_fail_after(monkeypatch, success_limit=3)
    evaluation = baseline_evaluation(sample_count=5)
    workspace = evaluate_linkage_workspace(evaluation, permissive_criteria())

    assert evaluation.solved_sample_count == 3
    assert workspace.closure_sample_count == 3
    assert workspace.closure_input_workspace_rad == pytest.approx(0.4)
    assert workspace.usable_input_workspace_rad == pytest.approx(0.4)
    assert workspace.first_unusable_index == 3
    assert workspace.limiting_reasons == ("kinematic_failure",)


def test_usable_workspace_stops_at_first_transmission_failure() -> None:
    evaluation = healthy_workspace_metrics(baseline_evaluation(sample_count=5))
    loop_1 = list(evaluation.transmission_angle_loop_1_rad)
    loop_1[2] = 0.1
    evaluation = replace(evaluation, transmission_angle_loop_1_rad=tuple(loop_1))
    criteria = permissive_criteria(minimum_transmission_margin_rad=0.2)

    workspace = evaluate_linkage_workspace(evaluation, criteria)

    assert workspace.closure_input_workspace_rad == pytest.approx(0.8)
    assert workspace.usable_input_workspace_rad == pytest.approx(0.2)
    assert workspace.first_unusable_index == 2
    assert workspace.limiting_reasons == ("transmission_loop_1",)


@pytest.mark.parametrize(
    "joint,limit_kind,expected_reason",
    [
        ("q2", "minimum", "q2_minimum_motion_ratio"),
        ("q3", "minimum", "q3_minimum_motion_ratio"),
        ("q2", "maximum", "q2_maximum_motion_ratio"),
        ("q3", "maximum", "q3_maximum_motion_ratio"),
    ],
)
def test_usable_workspace_reports_each_motion_ratio_limit(
    joint,
    limit_kind,
    expected_reason,
) -> None:
    evaluation = healthy_workspace_metrics(baseline_evaluation(sample_count=5))
    ratio_field = (
        "dq2_flexion_dq1_flexion"
        if joint == "q2"
        else "dq3_flexion_dq1_flexion"
    )
    ratios = [1.0] * (evaluation.solved_sample_count - 1)
    ratios[1] = 0.25 if limit_kind == "minimum" else 2.0
    evaluation = replace(evaluation, **{ratio_field: tuple(ratios)})

    criteria_kwargs = {
        f"{joint}_minimum_motion_ratio": 0.5 if limit_kind == "minimum" else 0.0,
        f"{joint}_maximum_motion_ratio": 1.5,
    }
    workspace = evaluate_linkage_workspace(
        evaluation,
        permissive_criteria(**criteria_kwargs),
    )

    assert workspace.usable_input_workspace_rad == pytest.approx(0.2)
    assert workspace.first_unusable_index == 2
    assert workspace.limiting_reasons == (expected_reason,)


def test_workspace_limiting_reasons_have_deterministic_order() -> None:
    evaluation = healthy_workspace_metrics(baseline_evaluation(sample_count=5))
    loop_1 = list(evaluation.transmission_angle_loop_1_rad)
    loop_2 = list(evaluation.transmission_angle_loop_2_rad)
    q2_ratio = list(evaluation.dq2_flexion_dq1_flexion)
    q3_ratio = list(evaluation.dq3_flexion_dq1_flexion)
    loop_1[2] = loop_2[2] = 0.1
    q2_ratio[1] = 0.25
    q3_ratio[1] = 2.0
    evaluation = replace(
        evaluation,
        transmission_angle_loop_1_rad=tuple(loop_1),
        transmission_angle_loop_2_rad=tuple(loop_2),
        dq2_flexion_dq1_flexion=tuple(q2_ratio),
        dq3_flexion_dq1_flexion=tuple(q3_ratio),
    )

    workspace = evaluate_linkage_workspace(
        evaluation,
        permissive_criteria(
            minimum_transmission_margin_rad=0.2,
            q2_minimum_motion_ratio=0.5,
            q3_maximum_motion_ratio=1.5,
        ),
    )

    assert workspace.limiting_reasons == (
        "transmission_loop_1",
        "transmission_loop_2",
        "q2_minimum_motion_ratio",
        "q3_maximum_motion_ratio",
    )


def test_usable_workspace_reaches_search_horizon_when_all_samples_pass() -> None:
    evaluation = healthy_workspace_metrics(baseline_evaluation(sample_count=5))
    workspace = evaluate_linkage_workspace(
        evaluation,
        permissive_criteria(
            minimum_transmission_margin_rad=0.2,
            q2_minimum_motion_ratio=0.5,
            q3_minimum_motion_ratio=0.5,
            q2_maximum_motion_ratio=1.5,
            q3_maximum_motion_ratio=1.5,
        ),
    )

    assert workspace.usable_input_workspace_rad == pytest.approx(0.8)
    assert workspace.closure_input_workspace_rad == pytest.approx(0.8)
    assert workspace.usable_sample_count == evaluation.requested_sample_count
    assert workspace.first_unusable_index is None
    assert workspace.limiting_reasons == ("search_horizon",)


def test_baseline_problem_is_core_feasible_when_closure_fails_post_core(monkeypatch) -> None:
    original_solve = LinkageKinematics.solve

    def fail_after_core(self, q1_rad):
        if q1_rad < -0.4:
            raise KinematicClosureError("post-core failure")
        return original_solve(self, q1_rad)

    monkeypatch.setattr(LinkageKinematics, "solve", fail_after_core)
    criteria = permissive_criteria()
    problem, encoded = baseline_problem(criteria=criteria)
    result = problem.evaluate(encoded)

    assert not result.domain_evaluation.motion.complete_workspace
    assert result.domain_evaluation.motion.solved_sample_count == 3
    assert result.feasible
    assert result.raw_objective_values["distal_endpoint_excursion_mm"] > 0.0
    assert result.inequality_results[0].satisfied
    assert result.inequality_results[1].satisfied


def test_failure_before_core_makes_baseline_problem_infeasible(monkeypatch) -> None:
    patch_solver_to_fail_after(monkeypatch, success_limit=2)
    problem, encoded = baseline_problem()
    result = problem.evaluate(encoded)

    assert not result.feasible
    assert result.inequality_results[0].maximum_violation == pytest.approx(1 / 3)


def test_transmission_objective_only_uses_core_quality_samples() -> None:
    evaluation = baseline_evaluation(sample_count=5)
    core_indices = (0, 1, 2)
    loop_1 = [0.8, 0.75, 0.7, 0.0, 0.0]
    loop_2 = [0.9, 0.85, 0.8, 0.0, 0.0]
    evaluation = replace(
        evaluation,
        transmission_quality_loop_1=tuple(loop_1),
        transmission_quality_loop_2=tuple(loop_2),
    )

    assert core_indices == (0, 1, 2)
    assert WorstCaseTransmissionQuality(0.4).evaluate(stage0_context(evaluation)) == pytest.approx(0.7)


def test_coordination_objective_ignores_post_core_motion() -> None:
    evaluation = baseline_evaluation(sample_count=5)
    core = 0.4
    core_indices = (0, 1, 2)
    q2_ratios = list(evaluation.dq2_flexion_dq1_flexion)
    q3_ratios = list(evaluation.dq3_flexion_dq1_flexion)
    q2_ratios[2:] = (1000.0, -1000.0)
    q3_ratios[2:] = (-2000.0, 2000.0)
    changed_post_core = replace(
        evaluation,
        dq2_flexion_dq1_flexion=tuple(q2_ratios),
        dq3_flexion_dq1_flexion=tuple(q3_ratios),
    )
    objective = CoordinationRatioVariation(core)

    assert core_indices == (0, 1, 2)
    assert objective.evaluate(stage0_context(evaluation)) == pytest.approx(
        objective.evaluate(stage0_context(changed_post_core))
    )


def test_post_core_motion_cannot_make_the_core_workspace_unusable(
    monkeypatch,
) -> None:
    base = baseline_evaluation(sample_count=5)
    q2_flexion = list(base.q2_flexion_rad)
    q2_flexion[3] = q2_flexion[2] + 1000.0
    q2_flexion[4] = q2_flexion[3]
    candidate_evaluation = replace(
        base,
        q2_flexion_rad=tuple(q2_flexion),
        dq2_flexion_dq1_flexion=differentiate_flexion_samples(
            base.input_flexion_rad,
            tuple(q2_flexion),
        ),
    )
    criteria = permissive_criteria(q2_maximum_motion_ratio=100.0)
    problem, encoded = baseline_problem(criteria=criteria)
    monkeypatch.setattr(
        LinkageMotionEvaluator,
        "evaluate",
        lambda self, parameters: candidate_evaluation,
    )

    result = problem.evaluate(encoded)
    workspace = result.domain_evaluation.workspace

    assert result.feasible
    assert result.inequality_results[0].satisfied
    assert result.inequality_results[1].satisfied
    assert workspace.usable_input_workspace_rad == pytest.approx(0.4)
    assert workspace.first_unusable_index == 3
    assert workspace.limiting_reasons == ("q2_maximum_motion_ratio",)


def test_input_angle_range_and_distal_excursion_are_independent_metrics() -> None:
    evaluation = healthy_workspace_metrics(baseline_evaluation(sample_count=5))
    criteria = permissive_criteria()
    long_input_grid = (0.0, 0.2, 0.4, 0.6, 0.8)
    short_input_grid = (0.0, 0.1, 0.2, 0.3, 0.4)
    long_path = tuple(
        PointXZ(x, 0.0)
        for x in (0.0, 0.25, 0.5, 0.75, 1.0)
    )
    short_path = tuple(
        PointXZ(x, 0.0)
        for x in (0.0, 0.5, 1.0, 1.5, 2.0)
    )

    def candidate(input_grid, c_path):
        poses = tuple(
            replace(pose, c=point)
            for pose, point in zip(evaluation.poses, c_path)
        )
        return replace(
            evaluation,
            input_flexion_rad=input_grid,
            q1_rad=tuple(-value for value in input_grid),
            poses=poses,
        )

    larger_q1_range = candidate(long_input_grid, long_path)
    smaller_q1_range = candidate(short_input_grid, short_path)
    range_workspace = evaluate_linkage_workspace(
        larger_q1_range,
        criteria,
    )
    displacement_workspace = evaluate_linkage_workspace(
        smaller_q1_range,
        criteria,
    )
    objective = DistalEndpointExcursion()

    assert range_workspace.usable_input_workspace_rad == pytest.approx(0.8)
    assert displacement_workspace.usable_input_workspace_rad == pytest.approx(0.4)
    assert range_workspace.distal_endpoint_excursion_mm == pytest.approx(1.0)
    assert displacement_workspace.distal_endpoint_excursion_mm == pytest.approx(2.0)
    assert objective.evaluate(stage0_context(larger_q1_range, criteria)) < objective.evaluate(
        stage0_context(smaller_q1_range, criteria)
    )


def test_distal_excursion_and_path_length_use_the_usable_c_prefix() -> None:
    evaluation = healthy_workspace_metrics(baseline_evaluation(sample_count=5))
    c_path = (
        PointXZ(0.0, 0.0),
        PointXZ(1.0, 0.0),
        PointXZ(1.0, 2.0),
        PointXZ(3.0, 2.0),
        PointXZ(4.0, 2.0),
    )
    evaluation = replace(
        evaluation,
        transmission_angle_loop_1_rad=(
            evaluation.transmission_angle_loop_1_rad[0],
            evaluation.transmission_angle_loop_1_rad[1],
            evaluation.transmission_angle_loop_1_rad[2],
            0.1,
            evaluation.transmission_angle_loop_1_rad[4],
        ),
        poses=tuple(
            replace(pose, c=point)
            for pose, point in zip(evaluation.poses, c_path)
        ),
    )
    workspace = evaluate_linkage_workspace(
        evaluation,
        permissive_criteria(minimum_transmission_margin_rad=0.2),
    )

    assert workspace.usable_input_workspace_rad == pytest.approx(0.4)
    assert workspace.distal_endpoint_excursion_mm == pytest.approx(math.sqrt(5.0))
    assert workspace.distal_path_length_mm == pytest.approx(3.0)
    assert workspace.distal_x_excursion_mm == pytest.approx(1.0)
    assert workspace.dorsal_z_excursion_mm == pytest.approx(2.0)


def test_minimum_usable_input_workspace_passes_and_fails() -> None:
    evaluation = healthy_workspace_metrics(baseline_evaluation(sample_count=5))
    criteria = permissive_criteria()
    passing = inequality_result(
        MinimumUsableInputWorkspace(0.6),
        stage0_context(evaluation, criteria),
    )
    failing = inequality_result(
        MinimumUsableInputWorkspace(0.9),
        stage0_context(evaluation, criteria),
    )

    assert passing.satisfied
    assert passing.residuals == pytest.approx((-0.2,))
    assert not failing.satisfied
    assert failing.residuals == pytest.approx((0.1,))


def test_minimum_required_workspace_must_cover_core() -> None:
    with pytest.raises(ValueError, match="at least the core"):
        baseline_constraint_config(
            core_input_workspace_rad=0.4,
            minimum_required_input_workspace_rad=0.3,
        )


@pytest.mark.parametrize("joint", ["q2", "q3"])
def test_core_endpoint_flexion_constraint_does_not_use_search_endpoint(joint) -> None:
    evaluation = baseline_evaluation(sample_count=5)
    core_index = evaluation.input_flexion_rad.index(0.4)
    flexion = (
        evaluation.q2_flexion_rad
        if joint == "q2"
        else evaluation.q3_flexion_rad
    )
    core_flexion = flexion[core_index]
    horizon_flexion = flexion[-1]
    threshold = (core_flexion + horizon_flexion) / 2.0

    residual = MinimumFlexionAtInput(
        joint,
        input_flexion_rad=0.4,
        minimum_flexion_rad=threshold,
    ).residual(stage0_context(evaluation))

    assert residual == pytest.approx((threshold - core_flexion,))
    assert residual[0] != pytest.approx(threshold - horizon_flexion)


def test_incomplete_core_has_finite_constraint_residuals_and_positive_validity(
    monkeypatch,
) -> None:
    patch_solver_to_fail_after(monkeypatch, success_limit=2)
    problem, encoded = baseline_problem()
    result = problem.evaluate(encoded)

    assert result.inequality_results[0].maximum_violation > 0.0
    assert isinstance(problem.inequalities[0], CoreWorkspaceKinematicValidity)
    assert result.raw_objective_values["worst_case_transmission_quality"] == 0.0
    assert all(
        math.isfinite(value) and not math.isnan(value)
        for diagnostic in result.inequality_results
        for value in diagnostic.residuals
    )


def test_baseline_problem_has_three_objectives_and_one_shared_criteria_object() -> None:
    criteria = permissive_criteria()
    problem, _ = baseline_problem(criteria=criteria)

    assert tuple(objective.name for objective in problem.objectives) == (
        "worst_case_transmission_quality",
        "coordination_ratio_variation",
        "distal_endpoint_excursion_mm",
    )
    assert problem.objectives[0].sense == "max"
    assert problem.objectives[1].sense == "min"
    assert problem.objectives[2].sense == "max"
    assert problem.equalities == ()
    assert all(
        constraint.criteria is criteria
        for constraint in problem.inequalities
        if isinstance(
            constraint,
            (
                MinimumTransmissionMargin,
                MonotonicFlexion,
                MaximumMotionRatio,
            ),
        )
    )


def test_incomplete_core_coordination_objective_retains_documented_inf() -> None:
    evaluation = baseline_evaluation(sample_count=5)
    incomplete = replace(evaluation, solved_sample_count=2, poses=evaluation.poses[:2])

    assert CoordinationRatioVariation(0.4).evaluate(stage0_context(incomplete)) == math.inf
