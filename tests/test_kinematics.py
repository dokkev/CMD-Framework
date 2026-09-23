import math

import pytest

from cmd.morphology import (
    AnchorYZ,
    CouplingAnchorParameters,
    FingerLinkage,
    FingerLinkageParameters,
    KinematicClosureError,
    LinkageKinematics,
    MainChainParameters,
)


def make_linkage() -> FingerLinkage:
    return FingerLinkage(
        FingerLinkageParameters(
            main_chain=MainChainParameters(
                proximal_length_mm=20.0,
                middle_length_mm=15.0,
                distal_length_mm=10.0,
            ),
            coupling=CouplingAnchorParameters(
                base_anchor=AnchorYZ(y_mm=0.0, z_mm=3.0),
                proximal_anchor=AnchorYZ(y_mm=3.0, z_mm=2.0),
                middle_anchor=AnchorYZ(y_mm=4.0, z_mm=-2.0),
                distal_anchor=AnchorYZ(y_mm=5.0, z_mm=1.0),
            ),
        )
    )


def rotate(point: AnchorYZ, angle_rad: float) -> AnchorYZ:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return AnchorYZ(
        cosine * point.y_mm - sine * point.z_mm,
        sine * point.y_mm + cosine * point.z_mm,
    )


def translated(origin: AnchorYZ, local: AnchorYZ, angle_rad: float) -> AnchorYZ:
    rotated = rotate(local, angle_rad)
    return AnchorYZ(origin.y_mm + rotated.y_mm, origin.z_mm + rotated.z_mm)


def assert_point_close(actual: AnchorYZ, expected: AnchorYZ) -> None:
    assert actual.y_mm == pytest.approx(expected.y_mm)
    assert actual.z_mm == pytest.approx(expected.z_mm)


def test_zero_pose_reproduces_reference_configuration() -> None:
    linkage = make_linkage()
    pose = LinkageKinematics(linkage).solve(0.0)
    reference = linkage.reference_positions

    assert pose.q1_rad == pytest.approx(0.0)
    assert pose.q2_rad == pytest.approx(0.0)
    assert pose.q3_rad == pytest.approx(0.0)
    for name in ("o", "a", "b", "c", "op", "ap", "bp", "cp"):
        assert_point_close(getattr(pose, name), getattr(reference, name))


def test_nonzero_pose_preserves_main_chain_and_coupler_lengths() -> None:
    linkage = make_linkage()
    pose = LinkageKinematics(linkage).solve(-0.05)
    parameters = linkage.parameters
    coupling = parameters.coupling
    theta2 = pose.q1_rad + pose.q2_rad
    theta3 = theta2 + pose.q3_rad

    assert pose.o.distance_to(pose.a) == pytest.approx(
        parameters.main_chain.proximal_length_mm
    )
    assert pose.a.distance_to(pose.b) == pytest.approx(
        parameters.main_chain.middle_length_mm
    )
    assert pose.b.distance_to(pose.c) == pytest.approx(
        parameters.main_chain.distal_length_mm
    )
    assert pose.op.distance_to(pose.bp) == pytest.approx(linkage.coupler_1_length_mm)
    assert pose.ap.distance_to(pose.cp) == pytest.approx(linkage.coupler_2_length_mm)

    assert_point_close(
        pose.ap,
        translated(pose.o, coupling.proximal_anchor, pose.q1_rad),
    )
    assert_point_close(
        pose.bp,
        translated(pose.a, coupling.middle_anchor, theta2),
    )
    assert_point_close(
        pose.cp,
        translated(pose.b, coupling.distal_anchor, theta3),
    )


def test_assembly_branch_selection_is_deterministic() -> None:
    solver = LinkageKinematics(make_linkage())

    first = solver.solve(-0.05)
    second = solver.solve(-0.05)

    assert first == second


def test_unreachable_closure_raises_explicit_failure() -> None:
    with pytest.raises(KinematicClosureError, match="closure"):
        LinkageKinematics(make_linkage()).solve(0.1)
