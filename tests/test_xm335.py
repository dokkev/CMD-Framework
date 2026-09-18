import math

import pytest

from cmd.hardware import BoxEnvelope, RevoluteActuatorGeometry, XM335Actuator


def test_xm335_vendor_dimensions_and_mass() -> None:
    actuator = XM335Actuator()

    assert actuator.WIDTH_MM == pytest.approx(19.0)
    assert actuator.HEIGHT_MM == pytest.approx(35.0)
    assert actuator.DEPTH_MM == pytest.approx(22.0)
    assert actuator.OUTPUT_DISK_DIAMETER_MM == pytest.approx(16.0)
    assert actuator.MASS_G == pytest.approx(27.0)


def test_xm335_joint_frame_places_axis_from_top_and_bottom() -> None:
    actuator = XM335Actuator()
    box = actuator.nominal_housing_box

    assert actuator.AXIS_TO_TOP_MM == pytest.approx(8.0)
    assert actuator.AXIS_TO_BOTTOM_MM == pytest.approx(27.0)
    assert actuator.housing_center_mm == pytest.approx((0.0, -9.5, 0.0))
    assert box.min_corner_mm == pytest.approx((-9.5, -27.0, -11.0))
    assert box.max_corner_mm == pytest.approx((9.5, 8.0, 11.0))


def test_xm335_revolute_axis_is_plus_z() -> None:
    geometry = XM335Actuator().geometry

    assert geometry.joint_axis == pytest.approx((0.0, 0.0, 1.0))
    assert geometry.unit_joint_axis == pytest.approx((0.0, 0.0, 1.0))


def test_clearance_expands_every_face_without_moving_center() -> None:
    actuator = XM335Actuator(clearance_mm=1.5)
    box = actuator.housing_box

    assert box.center_mm == pytest.approx((0.0, -9.5, 0.0))
    assert box.size_mm == pytest.approx((22.0, 38.0, 25.0))
    assert box.min_corner_mm == pytest.approx((-11.0, -28.5, -12.5))
    assert box.max_corner_mm == pytest.approx((11.0, 9.5, 12.5))


def test_negative_or_nonfinite_clearance_is_rejected() -> None:
    with pytest.raises(ValueError, match="non-negative"):
        XM335Actuator(clearance_mm=-0.1)
    with pytest.raises(ValueError, match="finite"):
        XM335Actuator(clearance_mm=math.inf)
    with pytest.raises(ValueError, match="finite"):
        XM335Actuator(clearance_mm=True)


def test_box_envelope_validation_and_expansion() -> None:
    box = BoxEnvelope(size_mm=(2.0, 4.0, 6.0), center_mm=(1.0, 2.0, 3.0))

    assert box.half_extents_mm == pytest.approx((1.0, 2.0, 3.0))
    assert box.min_corner_mm == pytest.approx((0.0, 0.0, 0.0))
    assert box.max_corner_mm == pytest.approx((2.0, 4.0, 6.0))
    assert box.expanded(0.5).size_mm == pytest.approx((3.0, 5.0, 7.0))

    with pytest.raises(ValueError, match="strictly positive"):
        BoxEnvelope(size_mm=(1.0, 0.0, 1.0))


def test_generic_actuator_geometry_requires_valid_axis_and_boxes() -> None:
    box = BoxEnvelope(size_mm=(1.0, 1.0, 1.0))

    with pytest.raises(ValueError, match="non-zero"):
        RevoluteActuatorGeometry("test", (0.0, 0.0, 0.0), (box,))
    with pytest.raises(ValueError, match="at least one"):
        RevoluteActuatorGeometry("test", (0.0, 0.0, 1.0), ())
