from dataclasses import FrozenInstanceError

import pytest

from cmd.hardware import (
    ACTUATOR_SHAFT_SIDE_AXIS_XYZ,
    Actuator,
    MechanicalParameters,
    RectangularBox,
    ShaftLocation,
    XM335Actuator,
)


def test_xm335_uses_the_project_parameters() -> None:
    actuator = XM335Actuator()

    assert actuator.box.width_mm == pytest.approx(19.0)
    assert actuator.box.height_mm == pytest.approx(35.0)
    assert actuator.box.depth_mm == pytest.approx(22.0)
    assert actuator.shaft.center_from_bottom_mm == pytest.approx(25.5)
    assert actuator.shaft.axis_xyz == pytest.approx((0.0, 0.0, 1.0))
    assert ACTUATOR_SHAFT_SIDE_AXIS_XYZ == pytest.approx((1.0, 0.0, 0.0))
    assert actuator.mechanics.gear_ratio == pytest.approx(323.04)
    assert actuator.mechanics.max_output_torque_nm == pytest.approx(1.0)
    assert actuator.mechanics.mass_g == pytest.approx(27.0)


def test_actuator_is_composed_of_only_box_shaft_and_mechanics() -> None:
    actuator = XM335Actuator()

    assert isinstance(actuator, Actuator)
    assert isinstance(actuator.box, RectangularBox)
    assert isinstance(actuator.shaft, ShaftLocation)
    assert isinstance(actuator.mechanics, MechanicalParameters)
    assert not hasattr(actuator, "geometry")
    assert not hasattr(actuator, "keepout_boxes")
    assert not hasattr(actuator, "housing_box")


def test_actuator_values_are_immutable() -> None:
    actuator = XM335Actuator()

    with pytest.raises(FrozenInstanceError):
        actuator.box = RectangularBox(1.0, 1.0, 1.0)

    with pytest.raises(FrozenInstanceError):
        actuator.shaft.center_from_bottom_mm = 10.0


@pytest.mark.parametrize(
    "factory",
    [
        lambda: RectangularBox(0.0, 1.0, 1.0),
        lambda: ShaftLocation(1.0, axis_xyz=(0.0, 0.0, 0.0)),
        lambda: MechanicalParameters(0.0, 1.0, 1.0),
        lambda: MechanicalParameters(1.0, 1.0, float("inf")),
    ],
)
def test_invalid_values_are_rejected(factory) -> None:
    with pytest.raises(ValueError):
        factory()


def test_shaft_must_remain_inside_the_box_height() -> None:
    with pytest.raises(ValueError, match="below the top"):
        Actuator(
            box=RectangularBox(19.0, 35.0, 22.0),
            shaft=ShaftLocation(35.0),
            mechanics=MechanicalParameters(323.04, 1.0, 27.0),
        )
