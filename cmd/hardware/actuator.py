"""Solver-independent actuator geometry and mechanical parameters.

The canonical actuator frame is shaft-centred: +X points toward the rounded
body cap, +Y is the shaft axis, and +Z is the lateral body-width direction.
All dimensions are in millimetres.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real


Vector3 = tuple[float, float, float]
ACTUATOR_SHAFT_AXIS_XYZ: Vector3 = (0.0, 1.0, 0.0)


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


@dataclass(frozen=True)
class DShapedExtrusion:
    """D-shaped XZ profile extruded along local Y, with dimensions in mm.

    The profile has a rectangular rear extent ``[-rear_extent_mm, 0]`` and a
    rounded cap of radius ``radius_mm`` in ``x >= 0``. The shaft center is the
    origin and the extrusion is centred about ``Y = 0``.
    """

    radius_mm: float
    rear_extent_mm: float
    depth_mm: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "radius_mm", _positive_real("radius_mm", self.radius_mm))
        object.__setattr__(
            self,
            "rear_extent_mm",
            _positive_real("rear_extent_mm", self.rear_extent_mm),
        )
        object.__setattr__(self, "depth_mm", _positive_real("depth_mm", self.depth_mm))
        if self.rear_extent_mm < self.radius_mm:
            raise ValueError(
                "rear_extent_mm must be at least radius_mm so the rear half "
                "of the cylinder is contained in the rectangular portion"
            )

    @property
    def longitudinal_extent_mm(self) -> float:
        """Total X extent from the rear face to the rounded tip."""

        return self.rear_extent_mm + self.radius_mm

    @property
    def width_mm(self) -> float:
        """Full lateral Z width of the D-shaped profile."""

        return 2.0 * self.radius_mm

    @property
    def bounds_xyz_mm(self) -> tuple[tuple[float, float], ...]:
        """Return ``((xmin, xmax), (ymin, ymax), (zmin, zmax))`` in mm."""

        return (
            (-self.rear_extent_mm, self.radius_mm),
            (-self.depth_mm / 2.0, self.depth_mm / 2.0),
            (-self.radius_mm, self.radius_mm),
        )


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
    """Minimal actuator value object: body geometry and mechanics."""

    geometry: DShapedExtrusion
    mechanics: MechanicalParameters

    def __post_init__(self) -> None:
        if not isinstance(self.geometry, DShapedExtrusion):
            raise TypeError("geometry must be a DShapedExtrusion instance")
        if not isinstance(self.mechanics, MechanicalParameters):
            raise TypeError("mechanics must be a MechanicalParameters instance")

    @property
    def shaft_axis_xyz(self) -> Vector3:
        """Return the canonical shaft axis in the shaft-centred actuator frame."""

        return ACTUATOR_SHAFT_AXIS_XYZ


__all__ = [
    "ACTUATOR_SHAFT_AXIS_XYZ",
    "Actuator",
    "DShapedExtrusion",
    "MechanicalParameters",
    "Vector3",
]
