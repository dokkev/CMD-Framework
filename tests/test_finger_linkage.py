import math

import pytest

from cmd.morphology.linkage import (
    FingerLinkage,
    FingerLinkageParameters,
    KinematicClosureError,
    SKETCH_LINKAGE,
    PointXZ,
    servo_clearance,
)
from cmd.morphology.linkage.packaging import ServoFootprint

SKETCH = SKETCH_LINKAGE.parameters


def test_reference_pose_is_the_parameterized_geometry() -> None:
    linkage = FingerLinkage(SKETCH)
    pose = linkage.solve(0.0, 0.0)

    for name in ("o", "a", "ap", "bp", "b", "c"):
        assert getattr(pose, name).x_mm == pytest.approx(getattr(linkage, name).x_mm)
        assert getattr(pose, name).z_mm == pytest.approx(getattr(linkage, name).z_mm)
    assert pose.q_dip_rad == pytest.approx(0.0)


def test_poses_keep_every_link_length() -> None:
    linkage = FingerLinkage(SKETCH)
    reference = linkage.solve(0.0, 0.0)

    for q_mcp, q_pip in ((0.3, 0.2), (0.8, 0.6), (1.2, 1.0)):
        pose = linkage.solve(q_mcp, q_pip)
        for first, second in (("o", "a"), ("o", "ap"), ("a", "ap"), ("a", "bp"),
                              ("ap", "b"), ("bp", "b"), ("bp", "c"), ("b", "c")):
            assert getattr(pose, first).distance_to(getattr(pose, second)) == pytest.approx(
                getattr(reference, first).distance_to(getattr(reference, second))
            )


def test_dip_follows_the_pip_alone_in_the_flexion_direction() -> None:
    linkage = FingerLinkage(SKETCH)
    q_dip = [linkage.solve(0.0, math.radians(deg)).q_dip_rad for deg in (0, 15, 30, 45, 60)]

    assert all(later > earlier for earlier, later in zip(q_dip, q_dip[1:]))
    assert linkage.solve(0.9, 0.5).q_dip_rad == pytest.approx(linkage.solve(0.0, 0.5).q_dip_rad)


def test_flexion_turns_the_finger_palmar() -> None:
    pose = FingerLinkage(SKETCH).solve(0.5, 0.0)

    assert pose.a.z_mm < 0.0
    assert pose.c.z_mm < 0.0


def test_open_four_bar_is_rejected() -> None:
    open_bar = FingerLinkageParameters(
        a_x_mm=40.0, b_x_mm=100.0, ap=PointXZ(80.0, 19.3), bp=PointXZ(60.0, 19.3),
        bc_length_mm=40.0,
    )
    with pytest.raises(ValueError, match="crossed"):
        FingerLinkage(open_bar)


def angle(first: tuple[float, float], second: tuple[float, float]) -> float:
    return abs(math.atan2(first[0] * second[1] - first[1] * second[0],
                          first[0] * second[0] + first[1] * second[1]))


def test_margins_measure_the_distance_from_a_dead_point() -> None:
    pose = FingerLinkage(SKETCH).solve(0.0, 0.0)
    at_b = angle((60.0 - 100.0, 19.3), (80.0 - 100.0, 19.3))
    at_bp = angle((40.0 - 80.0, -19.3), (100.0 - 80.0, -19.3))

    assert pose.margin_at_b_rad == pytest.approx(min(at_b, math.pi - at_b))
    assert pose.margin_at_bp_rad == pytest.approx(min(at_bp, math.pi - at_bp))


def test_sweep_matches_the_pose_solver() -> None:
    linkage = FingerLinkage(SKETCH)
    q = [math.radians(degrees) for degrees in range(0, 100, 7)]
    sweep = linkage.sweep_pip(q)

    for index, q_pip in enumerate(q):
        pose = linkage.solve(0.0, q_pip)
        assert sweep.b[index] == pytest.approx((pose.b.x_mm, pose.b.z_mm))
        assert sweep.c[index] == pytest.approx((pose.c.x_mm, pose.c.z_mm))
        assert sweep.q_dip_rad[index] == pytest.approx(pose.q_dip_rad)
        assert sweep.margin_at_b_rad[index] == pytest.approx(pose.margin_at_b_rad)
        assert sweep.margin_at_bp_rad[index] == pytest.approx(pose.margin_at_bp_rad)


def test_unreachable_pip_raises() -> None:
    # This four-bar does not let the middle link turn fully: some PIP angles fail.
    linkage = FingerLinkage(FingerLinkageParameters(
        a_x_mm=40.0, b_x_mm=100.0, ap=PointXZ(45.0, 10.0), bp=PointXZ(90.0, 10.0),
        bc_length_mm=40.0,
    ))
    with pytest.raises(KinematicClosureError):
        for degrees in range(0, 360, 5):
            linkage.solve(0.0, math.radians(degrees))


def test_servo_at_a_prime_lies_back_parallel_to_oa() -> None:
    clearance = servo_clearance(FingerLinkage(SKETCH))
    footprint = ServoFootprint.of(FingerLinkage(SKETCH))

    # The rear body spans x in [60 - 25.5, 60] and z in [19.3 - 9.5, 19.3 + 9.5];
    # its corner nearest O is (34.5, 9.8).
    assert clearance.mcp_servo_mm == pytest.approx(math.hypot(34.5, 9.8) - 9.5 - 1.0)
    assert footprint.lowest_z_mm == pytest.approx(19.3 - 9.5)
    # The PIP pin A clears the shaft disk around A'.
    assert clearance.pin_mm == pytest.approx(math.hypot(20.0, 19.3) - 9.5 - 4.5)


def test_servo_near_o_and_buried_pin_are_infeasible() -> None:
    near_o = FingerLinkageParameters(
        a_x_mm=40.0, b_x_mm=90.0, ap=PointXZ(30.0, 5.0), bp=PointXZ(70.0, 19.3),
        bc_length_mm=40.0,
    )
    # The rear body reaches back to x = 4.5 mm, inside the MCP servo disk.
    assert servo_clearance(FingerLinkage(near_o)).mcp_servo_mm == pytest.approx(4.5 - 9.5 - 1.0)

    buried = FingerLinkageParameters(
        a_x_mm=40.0, b_x_mm=100.0, ap=PointXZ(30.0, 8.0), bp=PointXZ(80.0, 19.3),
        bc_length_mm=40.0,
    )
    clearance = servo_clearance(FingerLinkage(buried))
    assert clearance.pin_mm < 0.0
    assert not clearance.feasible
