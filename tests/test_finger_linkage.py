import math
from dataclasses import replace

import numpy as np
import pytest

from core.morphology.linkage import (
    FingerLinkage,
    FingerLinkageParameters,
    KinematicClosureError,
    SKETCH_LINKAGE,
    PointXZ,
    servo_clearance,
)
from core.morphology.linkage.packaging import PIN_RADIUS_MM, Packaging

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


def test_open_four_bar_is_allowed_and_may_cross_as_it_flexes() -> None:
    open_bar = FingerLinkage(FingerLinkageParameters(
        a_x_mm=40.0, b_x_mm=100.0, ap=PointXZ(80.0, 19.3), bp=PointXZ(60.0, 19.3),
        bc_length_mm=40.0,
    ))
    assert not open_bar.is_crossed
    assert not open_bar.solve(0.0, 0.0).is_crossed
    assert FingerLinkage(SKETCH).is_crossed


def test_dead_point_reference_is_rejected() -> None:
    # B' on the line A'-B: the four-bar has no unique branch.
    dead = FingerLinkageParameters(
        a_x_mm=40.0, b_x_mm=100.0, ap=PointXZ(60.0, 20.0), bp=PointXZ(80.0, 10.0),
        bc_length_mm=40.0,
    )
    with pytest.raises(ValueError, match="dead point"):
        FingerLinkage(dead)


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
    # This four-bar does not let the driver link turn fully: some PIP angles fail.
    linkage = FingerLinkage(FingerLinkageParameters(
        a_x_mm=40.0, b_x_mm=100.0, ap=PointXZ(45.0, 10.0), bp=PointXZ(90.0, 10.0),
        bc_length_mm=40.0,
    ))
    with pytest.raises(KinematicClosureError):
        for degrees in range(0, 360, 5):
            linkage.solve(0.0, math.radians(degrees))


def test_servo_at_a_prime_lies_back_parallel_to_oa() -> None:
    clearance = servo_clearance(FingerLinkage(SKETCH))
    footprint = Packaging().pip_footprint(FingerLinkage(SKETCH))

    # The rear body spans x in [60 - 25.5, 60] and z in [19.3 - 9.5, 19.3 + 9.5];
    # the MCP servo body spans x in [-9.5, 9.5] up to z = 25.5: the gap is
    # between its +X face and the rear face, 34.5 - 9.5.
    assert clearance.mcp_servo_mm == pytest.approx(34.5 - 9.5 - 1.0)
    assert footprint.lowest_z_mm == pytest.approx(19.3 - 9.5)
    # A = (40, 0) and A' = (60, 19.3): the 8 mm housing at A keeps 1 mm from
    # the 7 mm virtual clearance at A'.
    assert clearance.pin_mm == pytest.approx(math.hypot(20.0, 19.3) - PIN_RADIUS_MM - 3.5 - 1.0)


def test_servo_near_o_and_a_pin_at_the_shaft_are_infeasible() -> None:
    near_o = FingerLinkageParameters(
        a_x_mm=40.0, b_x_mm=90.0, ap=PointXZ(30.0, 5.0), bp=PointXZ(70.0, 19.3),
        bc_length_mm=40.0,
    )
    # The rear body reaches back to x = 4.5 mm, into the MCP servo body.
    assert servo_clearance(FingerLinkage(near_o)).mcp_servo_mm == pytest.approx(-1.0)

    # |AA'| = 5 mm: the A housing (4 mm) reaches into the clearance at A' (3.5 mm).
    crowded = FingerLinkageParameters(
        a_x_mm=40.0, b_x_mm=100.0, ap=PointXZ(43.0, 4.0), bp=PointXZ(80.0, 19.3),
        bc_length_mm=40.0,
    )
    clearance = servo_clearance(FingerLinkage(crowded))
    assert clearance.pin_mm == pytest.approx(5.0 - PIN_RADIUS_MM - 3.5 - 1.0)
    assert not clearance.feasible


def test_the_a_pin_clears_the_virtual_clearance_at_a_prime_not_the_servo_body() -> None:
    # A' = (60, 8): the untilted rear body spans x in [34.5, 60], z in [-1.5, 17.5]
    # and covers A = (40, 0), which is still clear of the 7 mm disk at A'.
    flat = FingerLinkageParameters(
        a_x_mm=40.0, b_x_mm=90.0, ap=PointXZ(60.0, 8.0), bp=PointXZ(80.0, 10.0),
        bc_length_mm=40.0,
    )
    footprint = Packaging().pip_footprint(FingerLinkage(flat))
    assert footprint.distance_to_point(np.array((40.0, 0.0))) == 0.0
    expected = math.hypot(20.0, 8.0) - PIN_RADIUS_MM - 3.5 - 1.0
    assert servo_clearance(FingerLinkage(flat)).pin_mm == pytest.approx(expected)
    # The servo's tilt does not move the disk; its diameter is the packaging's.
    tilted = FingerLinkage(replace(flat, pip_servo_tilt_rad=math.radians(45.0)))
    assert servo_clearance(tilted).pin_mm == pytest.approx(expected)
    wider = Packaging(ap_clearance_diameter_mm=10.0)
    assert servo_clearance(FingerLinkage(flat), wider).pin_mm == pytest.approx(expected - 1.5)


def test_the_dip_pin_may_cross_the_servo_body_unless_checked() -> None:
    linkage = FingerLinkage(SKETCH)
    q = [math.radians(d) for d in range(0, 91)]
    # The driver link sits in another Y layer: B' is not checked by default.
    assert servo_clearance(linkage, pip_samples_rad=q).dip_pin_mm == math.inf

    checked = Packaging(allow_collision_pip_servo_to_joint_bp=False)
    footprint = checked.pip_footprint(linkage)
    sweep = linkage.sweep_pip(q)
    nearest = min(footprint.distance_to_point(point) for point in sweep.bp)
    assert servo_clearance(linkage, checked, pip_samples_rad=q).dip_pin_mm == pytest.approx(
        nearest - PIN_RADIUS_MM - 1.0)


def test_mcp_hyperextension_turns_the_pip_servo_toward_the_mcp_servo() -> None:
    packaging = Packaging()
    footprint = packaging.pip_footprint(FingerLinkage(SKETCH))
    at_rest = packaging.mcp_servo_gap_mm(footprint, (0.0,))
    hyperextended = packaging.mcp_servo_gap_mm(footprint, (math.radians(-10.0),))
    assert hyperextended < at_rest


def test_servo_and_pip_sweeps_solve_the_same_poses() -> None:
    import numpy as np

    linkage = FingerLinkage(SKETCH)
    by_pip = linkage.sweep_pip(np.radians([-5.0, 0.0, 20.0, 45.0, 70.0]))
    by_servo = linkage.sweep_servo(by_pip.servo_rad)

    assert by_servo.q_pip_rad == pytest.approx(by_pip.q_pip_rad, abs=1e-9)
    assert by_servo.q_dip_rad == pytest.approx(by_pip.q_dip_rad, abs=1e-9)
    assert np.abs(by_servo.c - by_pip.c).max() < 1e-9
    # Flexion turns the servo one way only, from zero at extension.
    assert by_pip.servo_rad[1] == pytest.approx(0.0, abs=1e-12)
    assert np.all(np.diff(by_pip.servo_rad) * np.sign(by_pip.servo_rad[-1]) > 0.0)
