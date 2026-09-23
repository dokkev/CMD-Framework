from dataclasses import FrozenInstanceError

import pytest

from cmd.hardware import (
    ACTUATOR_SHAFT_AXIS_XYZ,
    Actuator,
    DShapedExtrusion,
    MechanicalParameters,
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


def test_actuator_uses_solver_independent_d_shape_and_mechanics() -> None:
    actuator = XM335Actuator()

    assert isinstance(actuator, Actuator)
    assert isinstance(actuator.geometry, DShapedExtrusion)
    assert isinstance(actuator.mechanics, MechanicalParameters)
    assert not hasattr(actuator, "box")
    assert not hasattr(actuator, "shaft")
    assert not hasattr(actuator, "center_from_bottom_mm")
    assert not hasattr(actuator, "keepout_boxes")


def test_actuator_values_are_immutable() -> None:
    actuator = XM335Actuator()

    with pytest.raises(FrozenInstanceError):
        actuator.geometry = DShapedExtrusion(1.0, 2.0, 3.0)

    with pytest.raises(FrozenInstanceError):
        actuator.geometry.radius_mm = 10.0


@pytest.mark.parametrize(
    "factory",
    [
        lambda: DShapedExtrusion(0.0, 25.5, 22.0),
        lambda: DShapedExtrusion(9.5, 0.0, 22.0),
        lambda: DShapedExtrusion(9.5, 25.5, float("inf")),
        lambda: DShapedExtrusion(10.0, 9.5, 22.0),
        lambda: MechanicalParameters(0.0, 1.0, 1.0),
        lambda: MechanicalParameters(1.0, 1.0, float("inf")),
    ],
)
def test_invalid_values_are_rejected(factory) -> None:
    with pytest.raises(ValueError):
        factory()


def test_actuator_rejects_wrong_value_types() -> None:
    with pytest.raises(TypeError, match="DShapedExtrusion"):
        Actuator(geometry=object(), mechanics=MechanicalParameters(323.04, 1.0, 27.0))
