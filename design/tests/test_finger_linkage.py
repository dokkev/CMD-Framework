import math
from dataclasses import replace

import numpy as np
import pytest

from core.morphology.mechanism.linkage import (
    SKETCH_LINKAGE,
    Finger,
    KinematicClosureError,
    PointXZ,
    ServoMount,
    make_finger,
    servo_clearance,
)
from core.morphology.packaging.hardware import PIN_RADIUS_MM
from core.morphology.mechanism.linkage.packaging import (
    LinkagePackaging,
    pip_footprint,
)

SKETCH = SKETCH_LINKAGE


def test_reference_pose_is_the_parameterized_geometry() -> None:
    linkage = SKETCH
    pose = linkage.solve(0.0, 0.0)

    for name, of in (("o", linkage.origin), ("a", None), ("ap", None), ("bp", None), ("b", None), ("tip", linkage.tip)):
        point = of or getattr(linkage.joint, name)
        assert getattr(pose, name).x_mm == pytest.approx(point.x_mm)
        assert getattr(pose, name).z_mm == pytest.approx(point.z_mm)
    assert pose.q_dip_rad == pytest.approx(0.0)


def test_poses_keep_every_link_length() -> None:
    linkage = SKETCH
    reference = linkage.solve(0.0, 0.0)

    for q_mcp, q_pip in ((0.3, 0.2), (0.8, 0.6), (1.2, 1.0)):
        pose = linkage.solve(q_mcp, q_pip)
        for first, second in (("o", "a"), ("o", "ap"), ("a", "ap"), ("a", "bp"),
                              ("ap", "b"), ("bp", "b"), ("bp", "tip"), ("b", "tip")):
            assert getattr(pose, first).distance_to(getattr(pose, second)) == pytest.approx(
                getattr(reference, first).distance_to(getattr(reference, second))
            )


def test_dip_follows_the_pip_alone_in_the_flexion_direction() -> None:
    linkage = SKETCH
    q_dip = [linkage.solve(0.0, math.radians(deg)).q_dip_rad for deg in (0, 15, 30, 45, 60)]

    assert all(later > earlier for earlier, later in zip(q_dip, q_dip[1:]))
    assert linkage.solve(0.9, 0.5).q_dip_rad == pytest.approx(linkage.solve(0.0, 0.5).q_dip_rad)


def test_flexion_turns_the_finger_palmar() -> None:
    pose = SKETCH.solve(0.5, 0.0)

    assert pose.a.z_mm < 0.0
    assert pose.tip.z_mm < 0.0


def test_open_four_bar_is_allowed_and_may_cross_as_it_flexes() -> None:
    open_bar = make_finger(
        a=PointXZ(40.0, 0.0), b=PointXZ(100.0, 0.0), ap=PointXZ(80.0, 19.3), bp=PointXZ(60.0, 19.3),
        bc_length_mm=40.0,
    )
    assert not open_bar.joint.is_crossed
    assert not open_bar.solve(0.0, 0.0).is_crossed
    assert SKETCH.joint.is_crossed


def test_dead_point_reference_is_rejected() -> None:
    # B' on the line A'-B: the four-bar has no unique branch.
    dead = make_finger(
        a=PointXZ(40.0, 0.0), b=PointXZ(100.0, 0.0), ap=PointXZ(60.0, 20.0), bp=PointXZ(80.0, 10.0),
        bc_length_mm=40.0,
    )
    with pytest.raises(ValueError, match="dead point"):
        dead.joint.check()
    with pytest.raises(ValueError, match="dead point"):
        dead.sweep_servo([0.0, 0.1])


def angle(first: tuple[float, float], second: tuple[float, float]) -> float:
    return abs(math.atan2(first[0] * second[1] - first[1] * second[0],
                          first[0] * second[0] + first[1] * second[1]))


def test_margins_measure_the_distance_from_a_dead_point() -> None:
    pose = SKETCH.solve(0.0, 0.0)
    at_b = angle((60.0 - 100.0, 19.3), (80.0 - 100.0, 19.3))
    at_bp = angle((40.0 - 80.0, -19.3), (100.0 - 80.0, -19.3))

    assert pose.margin_at_b_rad == pytest.approx(min(at_b, math.pi - at_b))
    assert pose.margin_at_bp_rad == pytest.approx(min(at_bp, math.pi - at_bp))


def test_sweep_matches_the_pose_solver() -> None:
    linkage = SKETCH
    q = [math.radians(degrees) for degrees in range(0, 100, 7)]
    sweep = linkage.sweep_pip(q)

    for index, q_pip in enumerate(q):
        pose = linkage.solve(0.0, q_pip)
        assert sweep.b[index] == pytest.approx((pose.b.x_mm, pose.b.z_mm))
        assert sweep.tip[index] == pytest.approx((pose.tip.x_mm, pose.tip.z_mm))
        assert sweep.q_dip_rad[index] == pytest.approx(pose.q_dip_rad)
        assert sweep.margin_at_b_rad[index] == pytest.approx(pose.margin_at_b_rad)
        assert sweep.margin_at_bp_rad[index] == pytest.approx(pose.margin_at_bp_rad)


def test_unreachable_pip_raises() -> None:
    # This four-bar does not let the driver link turn fully: some PIP angles fail.
    linkage = make_finger(
        a=PointXZ(40.0, 0.0), b=PointXZ(100.0, 0.0), ap=PointXZ(45.0, 10.0), bp=PointXZ(90.0, 10.0),
        bc_length_mm=40.0,
    )
    with pytest.raises(KinematicClosureError):
        for degrees in range(0, 360, 5):
            linkage.solve(0.0, math.radians(degrees))


def test_servo_at_a_prime_lies_back_parallel_to_oa() -> None:
    clearance = servo_clearance(SKETCH)
    footprint = pip_footprint(LinkagePackaging(), SKETCH)

    # The rear body spans x in [60 - 25.5, 60] and z in [19.3 - 9.5, 19.3 + 9.5];
    # the MCP servo body spans x in [-9.5, 9.5] up to z = 25.5: the gap is
    # between its +X face and the rear face, 34.5 - 9.5.
    assert clearance.mcp_servo_mm == pytest.approx(34.5 - 9.5 - 1.0)
    assert footprint.lowest_z_mm == pytest.approx(19.3 - 9.5)
    # A = (40, 0) and A' = (60, 19.3): the 8 mm housing at A keeps 1 mm from
    # the 7 mm virtual clearance at A'.
    assert clearance.pin_mm == pytest.approx(math.hypot(20.0, 19.3) - PIN_RADIUS_MM - 3.5 - 1.0)


def test_servo_near_o_and_a_pin_at_the_shaft_are_infeasible() -> None:
    near_o = make_finger(
        a=PointXZ(40.0, 0.0), b=PointXZ(90.0, 0.0), ap=PointXZ(30.0, 5.0), bp=PointXZ(70.0, 19.3),
        bc_length_mm=40.0,
    )
    # The rear body reaches back to x = 4.5 mm, into the MCP servo body.
    assert servo_clearance(near_o).mcp_servo_mm == pytest.approx(-1.0)

    # |AA'| = 5 mm: the A housing (4 mm) reaches into the clearance at A' (3.5 mm).
    crowded = make_finger(
        a=PointXZ(40.0, 0.0), b=PointXZ(100.0, 0.0), ap=PointXZ(43.0, 4.0), bp=PointXZ(80.0, 19.3),
        bc_length_mm=40.0,
    )
    clearance = servo_clearance(crowded)
    assert clearance.pin_mm == pytest.approx(5.0 - PIN_RADIUS_MM - 3.5 - 1.0)
    assert not clearance.feasible


def test_the_a_pin_clears_the_virtual_clearance_at_a_prime_not_the_servo_body() -> None:
    # A' = (60, 8): the untilted rear body spans x in [34.5, 60], z in [-1.5, 17.5]
    # and covers A = (40, 0), which is still clear of the 7 mm disk at A'.
    flat = make_finger(
        a=PointXZ(40.0, 0.0), b=PointXZ(90.0, 0.0), ap=PointXZ(60.0, 8.0), bp=PointXZ(80.0, 10.0),
        bc_length_mm=40.0,
    )
    footprint = pip_footprint(LinkagePackaging(), flat)
    assert footprint.distance_to_point(np.array((40.0, 0.0))) == 0.0
    expected = math.hypot(20.0, 8.0) - PIN_RADIUS_MM - 3.5 - 1.0
    assert servo_clearance(flat).pin_mm == pytest.approx(expected)
    # The servo's tilt does not move the disk; its diameter is the packaging's.
    tilted = replace(flat, servo=ServoMount(math.radians(45.0)))
    assert servo_clearance(tilted).pin_mm == pytest.approx(expected)
    wider = LinkagePackaging(ap_clearance_diameter_mm=10.0)
    assert servo_clearance(flat, wider).pin_mm == pytest.approx(expected - 1.5)


def test_the_dip_pin_may_cross_the_servo_body_unless_checked() -> None:
    linkage = SKETCH
    q = [math.radians(d) for d in range(0, 91)]
    # The driver link sits in another Y layer: B' is not checked by default.
    assert servo_clearance(linkage, driver_samples_rad=q).dip_pin_mm == math.inf

    checked = LinkagePackaging(allow_collision_pip_servo_to_joint_bp=False)
    footprint = pip_footprint(checked, linkage)
    sweep = linkage.joint.sweep_driver(q)
    nearest = min(footprint.distance_to_point(point) for point in sweep.bp)
    assert servo_clearance(linkage, checked, driver_samples_rad=q).dip_pin_mm == pytest.approx(
        nearest - PIN_RADIUS_MM - 1.0)


def test_mcp_hyperextension_turns_the_pip_servo_toward_the_mcp_servo() -> None:
    packaging = LinkagePackaging()
    footprint = pip_footprint(packaging, SKETCH)
    at_rest = packaging.mcp_servo_gap_mm(footprint, (0.0,))
    hyperextended = packaging.mcp_servo_gap_mm(footprint, (math.radians(-10.0),))
    assert hyperextended < at_rest


def test_servo_and_pip_sweeps_solve_the_same_poses() -> None:
    import numpy as np

    linkage = SKETCH
    by_pip = linkage.sweep_pip(np.radians([-5.0, 0.0, 20.0, 45.0, 70.0]))
    by_servo = linkage.sweep_servo(by_pip.servo_rad)

    assert by_servo.q_pip_rad == pytest.approx(by_pip.q_pip_rad, abs=1e-9)
    assert by_servo.q_dip_rad == pytest.approx(by_pip.q_dip_rad, abs=1e-9)
    assert np.abs(by_servo.tip - by_pip.tip).max() < 1e-9
    # Flexion turns the servo one way only, from zero at extension.
    assert by_pip.servo_rad[1] == pytest.approx(0.0, abs=1e-12)
    assert np.all(np.diff(by_pip.servo_rad) * np.sign(by_pip.servo_rad[-1]) > 0.0)
