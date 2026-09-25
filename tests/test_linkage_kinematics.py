"""Tests for closed-form finger linkage kinematics."""

import math

import pytest

from cmd.morphology import (
    BASELINE_LINKAGE_V0,
    FingerLinkage,
    FingerLinkageParameters,
    KinematicClosureError,
    LinkageKinematics,
    PointYZ,
)


def make_noncollinear_linkage() -> FingerLinkage:
    return FingerLinkage(
        FingerLinkageParameters(
            o_prime_from_o=PointYZ(0.0, 10.0),
            a_from_o=PointYZ(10.0, 5.0),
            a_prime_from_o=PointYZ(4.0, 9.0),
            b_from_a=PointYZ(30.0, -5.0),
            b_prime_from_a=PointYZ(9.0, 8.0),
            bc_length_mm=20.0,
            c_prime_from_b=PointYZ(8.0, 6.0),
        )
    )


def rotate(point: PointYZ, angle_rad: float) -> PointYZ:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return PointYZ(
        cosine * point.y_mm - sine * point.z_mm,
        sine * point.y_mm + cosine * point.z_mm,
    )


def translated(origin: PointYZ, offset: PointYZ, angle_rad: float) -> PointYZ:
    rotated = rotate(offset, angle_rad)
    return PointYZ(origin.y_mm + rotated.y_mm, origin.z_mm + rotated.z_mm)


def assert_point_close(actual: PointYZ, expected: PointYZ) -> None:
    assert actual.y_mm == pytest.approx(expected.y_mm, abs=1e-10)
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


def test_zero_pose_reproduces_noncollinear_reference_configuration() -> None:
    linkage = make_noncollinear_linkage()
    reference = linkage.reference_positions

    assert reference.a.z_mm != pytest.approx(0.0)
    assert reference.ap.z_mm != pytest.approx(0.0)
    assert reference.bp.z_mm != pytest.approx(0.0)
    assert_pose_matches_reference(LinkageKinematics(linkage).solve(0.0), linkage)


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
    assert_point_close(pose.ap, translated(pose.o, parameters.a_prime_from_o, pose.q1_rad))
    assert_point_close(pose.b, translated(pose.a, parameters.b_from_a, delta_middle))
    assert_point_close(pose.bp, translated(pose.a, parameters.b_prime_from_a, delta_middle))
    assert_point_close(
        pose.cp,
        translated(pose.b, parameters.c_prime_from_b, delta_distal),
    )
    assert_point_close(
        pose.c,
        translated(pose.b, PointYZ(parameters.bc_length_mm, 0.0), delta_distal),
    )

    initial_a_prime = parameters.a_prime_from_o
    posed_a_prime = PointYZ(pose.ap.y_mm - pose.o.y_mm, pose.ap.z_mm - pose.o.z_mm)
    measured_q1 = math.atan2(
        initial_a_prime.y_mm * posed_a_prime.z_mm
        - initial_a_prime.z_mm * posed_a_prime.y_mm,
        initial_a_prime.y_mm * posed_a_prime.y_mm
        + initial_a_prime.z_mm * posed_a_prime.z_mm,
    )
    assert measured_q1 == pytest.approx(pose.q1_rad, abs=1e-10)


def test_assembly_branch_selection_is_deterministic() -> None:
    solver = LinkageKinematics(BASELINE_LINKAGE_V0)

    assert solver.solve(-0.05) == solver.solve(-0.05)


def test_unreachable_closure_raises_explicit_failure() -> None:
    with pytest.raises(KinematicClosureError, match="closure"):
        LinkageKinematics(BASELINE_LINKAGE_V0).solve(0.1)
