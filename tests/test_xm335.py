
import pytest

from core.hardware import (
    ACTUATOR_SHAFT_AXIS_XYZ,
    XM335Actuator,
)


def test_xm335_geometry_frame_and_project_parameters() -> None:
    actuator = XM335Actuator()
    geometry = actuator.geometry

    assert geometry.radius_mm == pytest.approx(9.5)
    assert geometry.rear_extent_mm == pytest.approx(25.5)
    assert geometry.depth_mm == pytest.approx(22.0)
    assert geometry.longitudinal_extent_mm == pytest.approx(35.0)
    assert geometry.width_mm == pytest.approx(19.0)
    bounds = [value for axis_bounds in geometry.bounds_xyz_mm for value in axis_bounds]
    assert bounds == pytest.approx([-9.5, 9.5, -25.5, 9.5, -11.0, 11.0])
    assert actuator.shaft_axis_xyz == pytest.approx((0.0, 0.0, 1.0))
    assert ACTUATOR_SHAFT_AXIS_XYZ == pytest.approx((0.0, 0.0, 1.0))
    assert actuator.mechanics.gear_ratio == pytest.approx(323.04)
    assert actuator.mechanics.max_output_torque_nm == pytest.approx(1.0)
    assert actuator.mechanics.mass_g == pytest.approx(27.0)
