"""Clearance between the linkage and the PIP servo carried at ``A``.

The PIP XM335 rides on the proximal link with its shaft on ``A`` (along
``Y``) and its rear body pointing back along the link toward ``O``. In the
linkage plane its footprint is the D-shaped section: a disk of the actuator
radius around ``A`` and a rectangle reaching ``rear_extent_mm`` toward ``O``.
All checks are planar and in the proximal-link (reference) frame, where the
servo does not move. Each margin is in mm and is feasible when non-negative.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from ...hardware import XM335Actuator
from . import FingerLinkage, KinematicClosureError, PointXZ


PIN_CLEARANCE_MM = 4.5
"""Pin centre to servo surface: MR63 bearing radius 3 mm plus a 1.5 mm wall."""
BODY_CLEARANCE_MM = 1.0
"""Servo to servo, and servo to moving link."""
DEFAULT_PIP_SAMPLES_RAD = tuple(math.radians(degrees) for degrees in range(0, 91))
_SEGMENT_SAMPLES = 64


@dataclass(frozen=True)
class ServoClearance:
    """Signed clearance margins of one linkage design (mm, >= 0 is feasible)."""

    mcp_servo_mm: float
    """PIP servo rear face beyond the MCP servo's shaft disk."""
    ap_pin_mm: float
    """``A'`` beyond the PIP servo footprint, less the pin clearance."""
    coupler_mm: float
    """Coupler ``A'-B`` beyond the PIP servo footprint over the PIP samples."""

    @property
    def feasible(self) -> bool:
        return min(self.mcp_servo_mm, self.ap_pin_mm, self.coupler_mm) >= 0.0


def servo_clearance(
    linkage: FingerLinkage,
    actuator: XM335Actuator | None = None,
    pip_samples_rad: Sequence[float] = DEFAULT_PIP_SAMPLES_RAD,
) -> ServoClearance:
    """Return the servo clearance margins; unreachable PIP samples are skipped.

    The MCP servo is taken as the disk of its radius around ``O``; its rear
    body points away from the finger and the finger only flexes away from it.
    """

    geometry = (actuator or XM335Actuator()).geometry
    radius = geometry.radius_mm
    rear = geometry.rear_extent_mm
    a_x = linkage.parameters.a_x_mm

    def footprint_distance(point: PointXZ) -> float:
        """Distance from ``point`` to the PIP servo footprint (0 inside)."""

        to_disk = max(0.0, math.hypot(point.x_mm - a_x, point.z_mm) - radius)
        dx = max(a_x - rear - point.x_mm, 0.0, point.x_mm - a_x)
        dz = max(abs(point.z_mm) - radius, 0.0)
        return min(to_disk, math.hypot(dx, dz))

    coupler = math.inf
    for q_pip in pip_samples_rad:
        try:
            pose = linkage.solve(0.0, q_pip)
        except KinematicClosureError:
            continue
        for index in range(_SEGMENT_SAMPLES + 1):
            t = index / _SEGMENT_SAMPLES
            point = PointXZ(
                pose.ap.x_mm + t * (pose.b.x_mm - pose.ap.x_mm),
                pose.ap.z_mm + t * (pose.b.z_mm - pose.ap.z_mm),
            )
            coupler = min(coupler, footprint_distance(point))

    return ServoClearance(
        mcp_servo_mm=a_x - rear - radius - BODY_CLEARANCE_MM,
        ap_pin_mm=footprint_distance(linkage.ap) - PIN_CLEARANCE_MM,
        coupler_mm=coupler - BODY_CLEARANCE_MM,
    )


__all__ = [
    "BODY_CLEARANCE_MM",
    "DEFAULT_PIP_SAMPLES_RAD",
    "PIN_CLEARANCE_MM",
    "ServoClearance",
    "servo_clearance",
]
