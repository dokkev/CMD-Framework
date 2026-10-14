"""Tests for closed-form finger linkage kinematics."""

import math

import pytest

from cmd.experimental.linkage import (
    BASELINE_LINKAGE_V0,
    FingerLinkage,
    KinematicClosureError,
    LinkageKinematics,
    PointXZ,
)


def rotate(point: PointXZ, angle_rad: float) -> PointXZ:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return PointXZ(
        cosine * point.x_mm - sine * point.z_mm,
        sine * point.x_mm + cosine * point.z_mm,
    )


def translated(origin: PointXZ, offset: PointXZ, angle_rad: float) -> PointXZ:
    rotated = rotate(offset, angle_rad)
    return PointXZ(origin.x_mm + rotated.x_mm, origin.z_mm + rotated.z_mm)


def assert_point_close(actual: PointXZ, expected: PointXZ) -> None:
    assert actual.x_mm == pytest.approx(expected.x_mm, abs=1e-10)
    assert actual.z_mm == pytest.approx(expected.z_mm, abs=1e-10)


def assert_pose_matches_reference(pose, linkage: FingerLinkage) -> None:
    reference = linkage.reference_positions
    for name in ("o", "op", "a", "ap", "b", "bp", "c", "cp"):
        assert_point_close(getattr(pose, name), getattr(reference, name))
    assert pose.q1_rad == pytest.approx(0.0, abs=1e-12)
    assert pose.q2_rad == pytest.approx(0.0, abs=1e-12)
    assert pose.q3_rad == pytest.approx(0.0, abs=1e-12)


def test_zero_pose_reproduces_baseline_reference_configuration() -> None:
    linkage = BASELINE_LINKAGE_V0

    pose = LinkageKinematics(linkage).solve(0.0)

    assert_pose_matches_reference(pose, linkage)


def test_nonzero_pose_closes_both_loops_and_preserves_all_fixed_lengths() -> None:
    linkage = BASELINE_LINKAGE_V0
    pose = LinkageKinematics(linkage).solve(-0.05)
    parameters = linkage.parameters
    delta_middle = pose.q1_rad + pose.q2_rad
    delta_distal = delta_middle + pose.q3_rad

    assert pose.o.distance_to(pose.a) == pytest.approx(linkage.oa_length_mm)
    assert pose.a.distance_to(pose.b) == pytest.approx(linkage.ab_length_mm)
    assert pose.b.distance_to(pose.c) == pytest.approx(parameters.bc_length_mm)
    assert pose.op.distance_to(pose.bp) == pytest.approx(linkage.coupler_1_length_mm)
    assert pose.ap.distance_to(pose.cp) == pytest.approx(linkage.coupler_2_length_mm)

    assert_point_close(pose.a, translated(pose.o, parameters.a_from_o, pose.q1_rad))
    assert_point_close(pose.ap, translated(pose.o, parameters.ap_from_o, pose.q1_rad))
    assert_point_close(pose.b, translated(pose.a, parameters.b_from_a, delta_middle))
    assert_point_close(pose.bp, translated(pose.a, parameters.bp_from_a, delta_middle))
    assert_point_close(
        pose.cp,
        translated(pose.b, parameters.cp_from_b, delta_distal),
    )
    assert_point_close(
        pose.c,
        translated(pose.b, PointXZ(parameters.bc_length_mm, 0.0), delta_distal),
    )

    initial_ap = parameters.ap_from_o
    posed_ap = PointXZ(pose.ap.x_mm - pose.o.x_mm, pose.ap.z_mm - pose.o.z_mm)
    measured_q1 = math.atan2(
        initial_ap.x_mm * posed_ap.z_mm
        - initial_ap.z_mm * posed_ap.x_mm,
        initial_ap.x_mm * posed_ap.x_mm
        + initial_ap.z_mm * posed_ap.z_mm,
    )
    assert measured_q1 == pytest.approx(pose.q1_rad, abs=1e-10)


def test_negative_q1_moves_proximal_link_in_flexion_direction() -> None:
    linkage = BASELINE_LINKAGE_V0
    solver = LinkageKinematics(linkage)
    extended = solver.solve(0.0)
    flexed = solver.solve(-0.05)

    assert linkage.flexion_axis_xyz == (0.0, 1.0, 0.0)
    assert flexed.a.z_mm < extended.a.z_mm


def test_unreachable_closure_raises_explicit_failure() -> None:
    with pytest.raises(KinematicClosureError, match="closure"):
        LinkageKinematics(BASELINE_LINKAGE_V0).solve(-2.0)
