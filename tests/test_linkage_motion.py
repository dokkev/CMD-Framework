"""Sampled linkage motion and usable-workspace diagnostics."""

from dataclasses import replace
import math

import pytest

from cmd.experimental.linkage import (
    BASELINE_LINKAGE_V0, LinkageMotionEvaluator, LinkageWorkspaceCriteria, PointXZ,
    evaluate_linkage_workspace,
    KinematicClosureError,
    LinkageKinematics,
    unwrap_principal_angles,
)


SAMPLES = (0.0, 0.2, 0.4, 0.6, 0.8)
# V0 geometry with O' at (10, 10): it closes over the whole SAMPLES sweep, so
# these tests exercise motion diagnostics independently of the current seed.
SWEEP_PARAMETERS = replace(BASELINE_LINKAGE_V0.parameters, op_from_o=PointXZ(10.0, 10.0))


def baseline_evaluation(*, sample_count: int = 5):
    assert sample_count == len(SAMPLES)
    return LinkageMotionEvaluator(SAMPLES).evaluate(SWEEP_PARAMETERS)


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


def healthy_workspace_metrics(evaluation):
    count = evaluation.solved_sample_count
    return replace(
        evaluation,
        dq2_flexion_dq1_flexion=(1.0,) * max(0, count - 1),
        dq3_flexion_dq1_flexion=(1.0,) * max(0, count - 1),
        transmission_angle_loop_1_rad=(math.pi / 2.0,) * count,
        transmission_angle_loop_2_rad=(math.pi / 2.0,) * count,
    )


def patch_solver_to_fail_after(monkeypatch, success_limit: int) -> None:
    original_solve = LinkageKinematics.solve
    counts: dict[int, int] = {}

    def fail_after_limit(self, q1_rad):
        solver_id = id(self)
        solved = counts.get(solver_id, 0)
        if solved >= success_limit:
            raise KinematicClosureError("synthetic closure failure")
        counts[solver_id] = solved + 1
        return original_solve(self, q1_rad)

    monkeypatch.setattr(LinkageKinematics, "solve", fail_after_limit)


def test_baseline_sweep_uses_positive_s_and_canonical_reference_branch() -> None:
    evaluation = baseline_evaluation(sample_count=5)

    assert evaluation.complete_sweep
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
