import math

import pytest

from core.hardware import XM335Actuator
from core.morphology.rolling_contact_joint import RollingContactJoint, RollingContactJointParameters


def xz(point) -> tuple[float, float]:
    return (point.x_mm, point.z_mm)


def test_the_radius_clears_the_servo_and_the_cable_allowance() -> None:
    assert XM335Actuator().geometry.radius_mm == 9.5
    RollingContactJointParameters(radius_mm=9.5, bc_length_mm=30.0)
    RollingContactJointParameters(radius_mm=11.5, bc_length_mm=30.0, cable_allowance_mm=2.0)
    with pytest.raises(ValueError, match="radius_mm must be at least"):
        RollingContactJointParameters(radius_mm=9.4, bc_length_mm=30.0)
    with pytest.raises(ValueError, match="radius_mm must be at least"):
        RollingContactJointParameters(radius_mm=11.0, bc_length_mm=30.0, cable_allowance_mm=2.0)
    # No maximum.
    RollingContactJointParameters(radius_mm=60.0, bc_length_mm=30.0)


def test_equal_rollers_touch_at_p_at_every_pose() -> None:
    parameters = RollingContactJointParameters(radius_mm=12.0, bc_length_mm=30.0, carrier_angle_rad=0.3)
    joint = RollingContactJoint(parameters)

    assert parameters.carrier_length_mm == pytest.approx(24.0)
    for degrees in (0.0, 30.0, 45.0, 90.0):
        pose = joint.solve(math.radians(degrees))
        assert math.dist(xz(pose.a), xz(pose.b)) == pytest.approx(2.0 * 12.0)
        # P is r from both centres, on the line between them.
        assert math.dist(xz(pose.a), xz(pose.p)) == pytest.approx(12.0)
        assert math.dist(xz(pose.b), xz(pose.p)) == pytest.approx(12.0)
        assert pose.carrier_angle_rad == pytest.approx(0.3 - math.radians(degrees))


@pytest.mark.parametrize("radius", (9.5, 15.0, 25.0))
def test_the_distal_flexion_is_twice_the_carrier_rotation(radius: float) -> None:
    joint = RollingContactJoint(RollingContactJointParameters(radius_mm=radius, bc_length_mm=30.0))

    for carrier, distal in ((0.0, 0.0), (45.0, 90.0), (90.0, 180.0)):
        pose = joint.solve(math.radians(carrier))
        assert math.degrees(pose.distal_flexion_rad) == pytest.approx(distal)
        # The distal phalanx B-C points along that flexion (+X toward -Z).
        direction = math.atan2(pose.c.z_mm - pose.b.z_mm, pose.c.x_mm - pose.b.x_mm)
        assert math.remainder(direction + math.radians(distal), 2.0 * math.pi) == pytest.approx(0.0, abs=1e-12)


def test_the_radius_changes_the_geometry_not_the_ratio() -> None:
    small = RollingContactJoint(RollingContactJointParameters(radius_mm=10.0, bc_length_mm=30.0))
    large = RollingContactJoint(RollingContactJointParameters(radius_mm=20.0, bc_length_mm=30.0))

    first, second = small.solve(math.radians(30.0)), large.solve(math.radians(30.0))
    assert xz(first.b) != pytest.approx(xz(second.b))
    assert first.distal_flexion_rad == pytest.approx(second.distal_flexion_rad)


def test_at_zero_rotation_the_distal_segment_points_along_x() -> None:
    joint = RollingContactJoint(RollingContactJointParameters(radius_mm=10.0, bc_length_mm=30.0))
    pose = joint.solve(0.0)

    assert xz(pose.a) == (0.0, 0.0)
    assert xz(pose.b) == pytest.approx((20.0, 0.0))
    assert xz(pose.p) == pytest.approx((10.0, 0.0))
    assert xz(pose.c) == pytest.approx((50.0, 0.0))
