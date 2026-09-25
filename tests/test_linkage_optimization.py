from dataclasses import replace
import math
from statistics import pstdev

import pytest

from cmd.morphology import BASELINE_LINKAGE_V0, FingerLinkage, PointYZ
from cmd.morphology.linkage_kinematics import KinematicClosureError, LinkageKinematics
from cmd.optimization import BoxVariable
from cmd.optimization.constraint import inequality_result
from cmd.optimization.linkage import (
    CoordinationRatioVariation,
    FullWorkspaceKinematicValidity,
    LinkageBaselineConstraintConfig,
    LinkageBaselineParameterization,
    LinkageSweepConfig,
    LinkageSweepEvaluator,
    MaximumMotionRatio,
    MinimumAnchorDorsalOffset,
    MinimumEndpointFlexion,
    MinimumTransmissionMargin,
    MonotonicFlexion,
    WorstCaseTransmissionQuality,
    build_linkage_baseline_problem,
)
from cmd.optimization.linkage.evaluator import unwrap_principal_angles


def projected_baseline_evaluation(sample_count: int = 11):
    parameterization = LinkageBaselineParameterization()
    encoded = parameterization.encode(BASELINE_LINKAGE_V0)
    parameters = parameterization.decode(dict(zip(parameterization.variable_names, encoded)))
    evaluation = LinkageSweepEvaluator(
        LinkageSweepConfig(max_flexion_rad=0.8, sample_count=sample_count)
    ).evaluate(parameters)
    return parameterization, encoded, parameters, evaluation


def test_linkage_parameterization_has_twelve_reference_position_coordinates() -> None:
    parameterization = LinkageBaselineParameterization()
    assert parameterization.variable_names == (
        "o_prime_from_o_y_mm",
        "o_prime_from_o_z_mm",
        "a_from_o_y_mm",
        "a_from_o_z_mm",
        "a_prime_from_o_y_mm",
        "a_prime_from_o_z_mm",
        "b_from_o_y_mm",
        "b_prime_from_o_y_mm",
        "b_prime_from_o_z_mm",
        "bc_length_mm",
        "c_prime_from_o_y_mm",
        "c_prime_from_o_z_mm",
    )


def test_decoding_enforces_fully_extended_geometry_by_construction() -> None:
    parameterization, encoded, parameters, evaluation = projected_baseline_evaluation()
    assert len(encoded) == 12
    assert evaluation.linkage is not None
    reference = evaluation.linkage.reference_positions

    assert reference.b.z_mm == 0.0
    assert reference.c.z_mm == 0.0
    assert reference.c.y_mm - reference.b.y_mm == pytest.approx(
        parameters.bc_length_mm
    )
    assert reference.c.y_mm > 0.0
    assert evaluation.linkage is not None
    assert evaluation.linkage.parameters == parameters
    assert parameterization.variable_names[9] == "bc_length_mm"


def test_encoding_projects_only_the_tiny_baseline_b_z_residual() -> None:
    parameterization = LinkageBaselineParameterization()
    original_reference = BASELINE_LINKAGE_V0.reference_positions
    encoded = parameterization.encode(BASELINE_LINKAGE_V0)
    decoded = parameterization.decode(dict(zip(parameterization.variable_names, encoded)))
    projected = FingerLinkage(decoded).reference_positions
    projection_error = parameterization.projection_error_mm(BASELINE_LINKAGE_V0)

    assert 0.0 < projection_error < 0.002
    assert original_reference == BASELINE_LINKAGE_V0.reference_positions
    for name in ("o", "op", "a", "ap", "bp", "cp"):
        actual = getattr(projected, name)
        expected = getattr(original_reference, name)
        assert actual.y_mm == pytest.approx(expected.y_mm, abs=1e-12)
        assert actual.z_mm == pytest.approx(expected.z_mm, abs=1e-12)
    assert projected.b.z_mm == 0.0
    assert projected.c.z_mm == 0.0
    assert abs(projected.b.z_mm - original_reference.b.z_mm) <= projection_error
    assert BASELINE_LINKAGE_V0.reference_positions == original_reference


def test_default_sweep_is_complete_with_flexion_sign_and_metrics() -> None:
    evaluation = LinkageSweepEvaluator().evaluate(BASELINE_LINKAGE_V0.parameters)

    assert evaluation.complete_workspace
    assert evaluation.solved_sample_count == 101
    assert evaluation.failure_index is None
    assert evaluation.failure_reason is None
    assert len(evaluation.input_flexion_rad) == 101
    assert len(evaluation.q1_rad) == 101
    assert evaluation.q1_rad == pytest.approx(
        tuple(-value for value in evaluation.input_flexion_rad)
    )
    assert evaluation.input_flexion_rad[0] == 0.0
    assert evaluation.input_flexion_rad[-1] == pytest.approx(0.8)
    assert evaluation.poses[0].q1_rad == 0.0
    assert evaluation.linkage is not None
    reference = evaluation.linkage.reference_positions
    zero_pose = evaluation.poses[0]
    for point_name in ("o", "a", "b", "c", "op", "ap", "bp", "cp"):
        assert getattr(zero_pose, point_name).distance_to(
            getattr(reference, point_name)
        ) < 1e-10
    assert evaluation.q2_flexion_rad[0] == pytest.approx(0.0, abs=1e-12)
    assert evaluation.q3_flexion_rad[0] == pytest.approx(0.0, abs=1e-12)
    assert evaluation.q2_flexion_rad[-1] > 0.0
    assert evaluation.q3_flexion_rad[-1] > 0.0

    for values in (
        evaluation.unwrapped_q2_rad,
        evaluation.unwrapped_q3_rad,
        evaluation.dq2_flexion_dq1_flexion,
        evaluation.dq3_flexion_dq1_flexion,
    ):
        assert len(values) == 101
        assert all(math.isfinite(value) for value in values)
    for angles in (
        evaluation.transmission_angle_loop_1_rad,
        evaluation.transmission_angle_loop_2_rad,
    ):
        assert len(angles) == 101
        assert all(math.isfinite(value) and 0.0 <= value <= math.pi for value in angles)
    for qualities in (
        evaluation.transmission_quality_loop_1,
        evaluation.transmission_quality_loop_2,
    ):
        assert len(qualities) == 101
        assert all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in qualities)


def test_angle_unwrap_preserves_continuity_across_principal_branch_cut() -> None:
    unwrapped = unwrap_principal_angles((3.0, -3.0, -2.8))

    assert unwrapped == pytest.approx((3.0, 2.0 * math.pi - 3.0, 2.0 * math.pi - 2.8))
    assert all(abs(next_value - value) < math.pi for value, next_value in zip(unwrapped, unwrapped[1:]))


def test_invalid_reference_geometry_returns_incomplete_evaluation() -> None:
    invalid_parameters = replace(
        BASELINE_LINKAGE_V0.parameters,
        b_from_a=PointYZ(
            BASELINE_LINKAGE_V0.parameters.b_from_a.y_mm,
            -4.0,
        ),
    )
    evaluation = LinkageSweepEvaluator(
        LinkageSweepConfig(sample_count=5)
    ).evaluate(invalid_parameters)

    assert not evaluation.complete_workspace
    assert evaluation.linkage is None
    assert evaluation.solved_sample_count == 0
    assert evaluation.failure_index == 0
    assert "BC parallel to OC" in evaluation.failure_reason


def test_sweep_failure_keeps_partial_diagnostics(monkeypatch) -> None:
    original_solve = LinkageKinematics.solve

    def fail_after_two_samples(self, q1_rad):
        if q1_rad < -0.2:
            raise KinematicClosureError("synthetic unreachable pose")
        return original_solve(self, q1_rad)

    monkeypatch.setattr(LinkageKinematics, "solve", fail_after_two_samples)
    evaluation = LinkageSweepEvaluator(
        LinkageSweepConfig(max_flexion_rad=0.8, sample_count=5)
    ).evaluate(BASELINE_LINKAGE_V0.parameters)

    assert not evaluation.complete_workspace
    assert evaluation.solved_sample_count == 2
    assert len(evaluation.poses) == 2
    assert evaluation.failure_index == 2
    assert "synthetic unreachable pose" in evaluation.failure_reason


def test_objectives_match_sampled_metrics_and_worst_values_on_failure() -> None:
    _, _, _, evaluation = projected_baseline_evaluation()
    worst_quality = min(
        evaluation.transmission_quality_loop_1
        + evaluation.transmission_quality_loop_2
    )
    assert WorstCaseTransmissionQuality().evaluate(evaluation) == pytest.approx(
        worst_quality
    )

    ratios_2 = evaluation.dq2_flexion_dq1_flexion
    ratios_3 = evaluation.dq3_flexion_dq1_flexion
    expected_variation = (
        pstdev(ratios_2) / (abs(sum(ratios_2) / len(ratios_2)) + 1e-9)
        + pstdev(ratios_3) / (abs(sum(ratios_3) / len(ratios_3)) + 1e-9)
    )
    variation = CoordinationRatioVariation().evaluate(evaluation)
    assert variation == pytest.approx(expected_variation)
    assert variation >= 0.0

    incomplete = replace(evaluation, complete_workspace=False)
    assert WorstCaseTransmissionQuality().evaluate(incomplete) == 0.0
    assert CoordinationRatioVariation().evaluate(incomplete) == math.inf


def test_each_linkage_inequality_reports_passing_and_violating_residuals() -> None:
    _, _, _, evaluation = projected_baseline_evaluation()

    complete = inequality_result(FullWorkspaceKinematicValidity(), evaluation)
    incomplete_evaluation = replace(
        evaluation,
        complete_workspace=False,
        solved_sample_count=evaluation.solved_sample_count - 1,
    )
    incomplete = inequality_result(FullWorkspaceKinematicValidity(), incomplete_evaluation)
    assert complete.satisfied and complete.residuals == (0.0,)
    assert not incomplete.satisfied and incomplete.maximum_violation > 0.0

    dorsal_pass = inequality_result(MinimumAnchorDorsalOffset(0.0), evaluation)
    dorsal_fail = inequality_result(MinimumAnchorDorsalOffset(0.01), evaluation)
    assert dorsal_pass.residuals[0] == pytest.approx(0.0)
    assert dorsal_pass.satisfied
    assert not dorsal_fail.satisfied

    margins = tuple(
        min(angle, math.pi - angle)
        for angle in (
            evaluation.transmission_angle_loop_1_rad
            + evaluation.transmission_angle_loop_2_rad
        )
    )
    margin_pass = inequality_result(MinimumTransmissionMargin(0.0), evaluation)
    margin_fail = inequality_result(
        MinimumTransmissionMargin(min(margins) + 1e-4),
        evaluation,
    )
    assert len(margin_pass.residuals) == 2 * evaluation.requested_sample_count
    assert margin_pass.satisfied
    assert not margin_fail.satisfied

    for joint, ratios, flexion in (
        ("q2", evaluation.dq2_flexion_dq1_flexion, evaluation.q2_flexion_rad),
        ("q3", evaluation.dq3_flexion_dq1_flexion, evaluation.q3_flexion_rad),
    ):
        monotonic_pass = inequality_result(MonotonicFlexion(joint, min(ratios)), evaluation)
        monotonic_fail = inequality_result(
            MonotonicFlexion(joint, max(ratios) + 0.1),
            evaluation,
        )
        assert len(monotonic_pass.residuals) == evaluation.requested_sample_count
        assert monotonic_pass.satisfied
        assert not monotonic_fail.satisfied

        max_ratio_pass = inequality_result(
            MaximumMotionRatio(joint, max(ratios) + 0.1),
            evaluation,
        )
        max_ratio_fail = inequality_result(
            MaximumMotionRatio(joint, max(ratios) - 0.01),
            evaluation,
        )
        assert max_ratio_pass.satisfied
        assert not max_ratio_fail.satisfied

        endpoint = flexion[-1]
        endpoint_pass = inequality_result(
            MinimumEndpointFlexion(joint, endpoint - 0.1),
            evaluation,
        )
        endpoint_fail = inequality_result(
            MinimumEndpointFlexion(joint, endpoint + 0.1),
            evaluation,
        )
        assert endpoint_pass.satisfied
        assert not endpoint_fail.satisfied


def test_incomplete_sweep_violates_all_samplewise_motion_and_margin_constraints(
    monkeypatch,
) -> None:
    original_solve = LinkageKinematics.solve

    def fail_at_flexion(self, q1_rad):
        if q1_rad < -0.2:
            raise KinematicClosureError("synthetic closure failure")
        return original_solve(self, q1_rad)

    monkeypatch.setattr(LinkageKinematics, "solve", fail_at_flexion)
    evaluation = LinkageSweepEvaluator(
        LinkageSweepConfig(sample_count=5)
    ).evaluate(BASELINE_LINKAGE_V0.parameters)
    expected_missing = evaluation.requested_sample_count - evaluation.solved_sample_count

    monotonic = MonotonicFlexion("q2").residual(evaluation)
    maximum = MaximumMotionRatio("q2", 3.0).residual(evaluation)
    margin = MinimumTransmissionMargin(0.1).residual(evaluation)
    assert len(monotonic) == evaluation.requested_sample_count
    assert len(maximum) == evaluation.requested_sample_count
    assert len(margin) == 2 * evaluation.requested_sample_count
    assert all(math.isinf(value) and value > 0.0 for value in monotonic[-expected_missing:])
    assert all(math.isinf(value) and value > 0.0 for value in maximum[-expected_missing:])
    assert all(math.isinf(value) and value > 0.0 for value in margin[-2 * expected_missing:])


def test_baseline_problem_is_caller_bounded_and_has_no_equalities() -> None:
    parameterization = LinkageBaselineParameterization()
    encoded = parameterization.encode(BASELINE_LINKAGE_V0)
    variables = tuple(
        BoxVariable(
            name,
            value - 1.0,
            value + 1.0,
        )
        for name, value in zip(parameterization.variable_names, encoded)
    )
    settings = LinkageBaselineConstraintConfig(
        minimum_anchor_dorsal_offset_mm=0.0,
        minimum_transmission_margin_rad=math.radians(30.0),
        q2_minimum_motion_ratio=0.0,
        q3_minimum_motion_ratio=0.0,
        q2_maximum_motion_ratio=3.0,
        q3_maximum_motion_ratio=3.0,
        q2_minimum_endpoint_flexion_rad=0.4,
        q3_minimum_endpoint_flexion_rad=0.2,
    )
    problem = build_linkage_baseline_problem(
        variables,
        settings,
        LinkageSweepConfig(sample_count=11),
    )
    result = problem.evaluate(encoded)

    assert len(problem.variables) == 12
    assert problem.equalities == ()
    assert tuple(result.raw_objective_values) == (
        "worst_case_transmission_quality",
        "coordination_ratio_variation",
    )
    assert tuple(item.name for item in result.inequality_results) == (
        "full_workspace_kinematic_validity",
        "minimum_anchor_dorsal_offset",
        "minimum_transmission_margin",
        "monotonic_flexion_q2",
        "monotonic_flexion_q3",
        "maximum_motion_ratio_q2",
        "maximum_motion_ratio_q3",
        "minimum_endpoint_flexion_q2",
        "minimum_endpoint_flexion_q3",
    )
