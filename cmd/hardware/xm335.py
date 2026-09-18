"""Minimal ROBOTIS XM335 actuator parameters."""

from __future__ import annotations

from dataclasses import dataclass, field

from .actuator import (
    Actuator,
    MechanicalParameters,
    RectangularBox,
    ShaftLocation,
)


@dataclass(frozen=True)
class XM335Actuator(Actuator):
    """XM335 physical representation for CLEAR Hand layout.

    The dimensions are project parameters, not a collision or simulator
    geometry model.
    """

    box: RectangularBox = field(
        default_factory=lambda: RectangularBox(
            width_mm=19.0,
            height_mm=35.0,
            depth_mm=22.0,
        )
    )
    shaft: ShaftLocation = field(
        default_factory=lambda: ShaftLocation(
            center_from_bottom_mm=25.5,
            axis_xyz=(1.0, 0.0, 0.0),
        )
    )
    mechanics: MechanicalParameters = field(
        default_factory=lambda: MechanicalParameters(
            gear_ratio=323.04,
            max_output_torque_nm=1.0,
            mass_g=27.0,
        )
    )


__all__ = ["XM335Actuator"]
