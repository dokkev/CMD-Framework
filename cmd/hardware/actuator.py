"""Minimal solver-independent actuator representation.

The hardware model intentionally contains only the physical information needed
for hand layout and later kinematic or Newton compilation:

    actuator = rectangular box + shaft location/axis + mechanics

It does not define keep-out regions, collision primitives, simulator bodies, or
actuator transmission geometry.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real


Vector3 = tuple[float, float, float]


def _finite_real(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite real number; got {value!r}")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite real number; got {value!r}")
    return value


def _positive_real(name: str, value: object) -> float:
    value = _finite_real(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be strictly positive; got {value!r}")
    return value


def _axis_xyz(value: object) -> Vector3:
    if not isinstance(value, tuple) or len(value) != 3:
        raise TypeError("axis_xyz must be a length-3 tuple")

    axis = tuple(
        _finite_real(f"axis_xyz[{index}]", component)
        for index, component in enumerate(value)
    )
    norm = math.sqrt(sum(component * component for component in axis))
    if not math.isclose(norm, 1.0, rel_tol=1e-12, abs_tol=1e-12):
        raise ValueError(f"axis_xyz must be a unit vector; got {value!r}")
    return axis  # type: ignore[return-value]


@dataclass(frozen=True)
class RectangularBox:
    """Rectangular actuator body dimensions, all in millimetres."""

    width_mm: float
    height_mm: float
    depth_mm: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "width_mm", _positive_real("width_mm", self.width_mm))
        object.__setattr__(self, "height_mm", _positive_real("height_mm", self.height_mm))
        object.__setattr__(self, "depth_mm", _positive_real("depth_mm", self.depth_mm))


@dataclass(frozen=True)
class ShaftLocation:
    """Actuator shaft location and unit axis in the actuator frame."""

    center_from_bottom_mm: float
    axis_xyz: Vector3 = (1.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "center_from_bottom_mm",
            _positive_real("center_from_bottom_mm", self.center_from_bottom_mm),
        )
        object.__setattr__(self, "axis_xyz", _axis_xyz(self.axis_xyz))


@dataclass(frozen=True)
class MechanicalParameters:
    """Mechanical actuator properties used by later design/simulation code."""

    gear_ratio: float
    max_output_torque_nm: float
    mass_g: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "gear_ratio", _positive_real("gear_ratio", self.gear_ratio))
        object.__setattr__(
            self,
            "max_output_torque_nm",
            _positive_real("max_output_torque_nm", self.max_output_torque_nm),
        )
        object.__setattr__(self, "mass_g", _positive_real("mass_g", self.mass_g))


@dataclass(frozen=True)
class Actuator:
    """Minimal actuator value object: box, shaft, and mechanical parameters."""

    box: RectangularBox
    shaft: ShaftLocation
    mechanics: MechanicalParameters

    def __post_init__(self) -> None:
        if not isinstance(self.box, RectangularBox):
            raise TypeError("box must be a RectangularBox instance")
        if not isinstance(self.shaft, ShaftLocation):
            raise TypeError("shaft must be a ShaftLocation instance")
        if not isinstance(self.mechanics, MechanicalParameters):
            raise TypeError("mechanics must be a MechanicalParameters instance")
        if self.shaft.center_from_bottom_mm >= self.box.height_mm:
            raise ValueError(
                "shaft.center_from_bottom_mm must be below the top of the actuator box"
            )


__all__ = [
    "Actuator",
    "MechanicalParameters",
    "RectangularBox",
    "ShaftLocation",
    "Vector3",
]
