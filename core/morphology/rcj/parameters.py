"""Reference geometry of the equal-radius rolling contact joint (RCJ)."""

from __future__ import annotations

from dataclasses import dataclass, field

from core._validation import require_finite, require_nonnegative, require_positive
from core.hardware import XM335Actuator
from core.morphology.linkage.parameters import PointXZ


def _xm335_radius_mm() -> float:
    return XM335Actuator().geometry.radius_mm


@dataclass(frozen=True)
class RollingContactJointParameters:
    """An RCJ of two equal rollers joined by a servo-driven rigid carrier.

    ``A`` is the proximal roller's centre, on the servo's output axis; ``B``,
    the distal roller's centre, sits ``2 r`` from it along the carrier, which
    points ``carrier_angle_rad`` from ``+X`` (toward ``+Z``) at extension. The
    distal phalanx ``B-C`` points along ``+X`` at extension. The radius must
    clear the servo and the cable routing: ``r >= servo_radius_mm +
    cable_allowance_mm``; it has no maximum.
    """

    radius_mm: float
    bc_length_mm: float
    a: PointXZ = PointXZ(0.0, 0.0)
    carrier_angle_rad: float = 0.0
    cable_allowance_mm: float = 0.0
    servo_radius_mm: float = field(default_factory=_xm335_radius_mm)

    def __post_init__(self) -> None:
        require_positive("radius_mm", self.radius_mm)
        require_positive("bc_length_mm", self.bc_length_mm)
        require_finite("carrier_angle_rad", self.carrier_angle_rad)
        require_nonnegative("cable_allowance_mm", self.cable_allowance_mm)
        require_positive("servo_radius_mm", self.servo_radius_mm)
        if not isinstance(self.a, PointXZ):
            raise TypeError("a must be a PointXZ instance")
        least = self.servo_radius_mm + self.cable_allowance_mm
        if self.radius_mm < least:
            raise ValueError(f"radius_mm must be at least servo_radius_mm + cable_allowance_mm = {least}")

    @property
    def carrier_length_mm(self) -> float:
        """``|AB| = 2 r``: the two equal rollers touch."""

        return 2.0 * self.radius_mm


__all__ = ["RollingContactJointParameters"]
