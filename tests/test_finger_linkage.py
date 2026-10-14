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


def test_transmission_margin_measures_the_distance_from_a_dead_point() -> None:
    pose = FingerLinkage(SKETCH).solve(0.0, 0.0)
    to_ap = (60.0 - 100.0, 19.3)
    to_bp = (80.0 - 100.0, 19.3)
    angle = abs(math.atan2(to_ap[0] * to_bp[1] - to_ap[1] * to_bp[0],
                           to_ap[0] * to_bp[0] + to_ap[1] * to_bp[1]))

    assert pose.transmission_angle_rad == pytest.approx(angle)
    assert pose.transmission_margin_rad == pytest.approx(min(angle, math.pi - angle))


def test_unreachable_pip_raises() -> None:
    # This four-bar does not let the middle link turn fully: some PIP angles fail.
    linkage = FingerLinkage(FingerLinkageParameters(
        a_x_mm=40.0, b_x_mm=100.0, ap=PointXZ(45.0, 10.0), bp=PointXZ(90.0, 10.0),
        bc_length_mm=40.0,
    ))
    with pytest.raises(KinematicClosureError):
        for degrees in range(0, 360, 5):
            linkage.solve(0.0, math.radians(degrees))


def test_servo_clearance_of_the_sketch() -> None:
    clearance = servo_clearance(FingerLinkage(SKETCH))

    # OA = 40 mm against 25.5 mm rear + 9.5 mm MCP servo radius + 1 mm.
    assert clearance.mcp_servo_mm == pytest.approx(40.0 - 25.5 - 9.5 - 1.0)
    # A' = (60, 19.3) is outside the servo cap around A = (40, 0).
    assert clearance.ap_pin_mm == pytest.approx(math.hypot(20.0, 19.3) - 9.5 - 4.5)


def test_short_proximal_link_and_buried_pin_are_infeasible() -> None:
    short = FingerLinkageParameters(
        a_x_mm=30.0, b_x_mm=90.0, ap=PointXZ(50.0, 19.3), bp=PointXZ(70.0, 19.3),
        bc_length_mm=40.0,
    )
    assert servo_clearance(FingerLinkage(short)).mcp_servo_mm < 0.0

    buried = FingerLinkageParameters(
        a_x_mm=40.0, b_x_mm=100.0, ap=PointXZ(30.0, 8.0), bp=PointXZ(80.0, 19.3),
        bc_length_mm=40.0,
    )
    clearance = servo_clearance(FingerLinkage(buried))
    assert clearance.ap_pin_mm < 0.0
    assert not clearance.feasible
