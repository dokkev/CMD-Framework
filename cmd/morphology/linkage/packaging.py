"""Clearance between the linkage and the PIP servo on the proximal link.

The PIP XM335 rides on the proximal link with its shaft (along ``Y``) on
``A'`` and drives the coupler ``A'-B``; sitting dorsal of ``OA``, it leaves
the proximal contact interface free. Its rear body lies back along the link,
parallel to ``OA`` (``-X``). In the linkage plane the footprint is the
D-shaped section: a disk of the actuator radius around ``A'`` and a
rectangle reaching ``rear_extent_mm`` back from it. All checks are planar and in the proximal-link (reference) frame,
where the servo does not move. Each margin is in mm, feasible when >= 0.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from ...hardware import XM335Actuator
from . import FingerLinkage


PIN_CLEARANCE_MM = 4.5
"""Pin centre to servo surface: MR63 bearing radius 3 mm plus a 1.5 mm wall."""
BODY_CLEARANCE_MM = 1.0
"""Servo to servo, and servo to moving link."""
DEFAULT_PIP_SAMPLES_RAD = tuple(math.radians(degrees) for degrees in range(0, 91))

@dataclass(frozen=True)
class ServoClearance:
    """Signed clearance margins of one linkage design (mm, >= 0 is feasible)."""

    mcp_servo_mm: float
    """PIP servo beyond the MCP servo's shaft disk."""
    pin_mm: float
    """The PIP pin ``A`` beyond the servo's shaft disk, less the pin clearance.
    The rear body is to be routed clear of it."""
    link_mm: float
    """The middle link ``A-B'`` beyond the servo over the PIP samples."""

    @property
    def feasible(self) -> bool:
        return min(self.mcp_servo_mm, self.pin_mm, self.link_mm) >= 0.0


def servo_clearance(
    linkage: FingerLinkage,
    actuator: XM335Actuator | None = None,
    pip_samples_rad: Sequence[float] = DEFAULT_PIP_SAMPLES_RAD,
) -> ServoClearance:
    """Return the servo clearance margins over the increasing PIP samples.

    The link is checked up to the last sample that closes. The MCP servo is
    taken as the disk of its radius around ``O``; its rear body points away
    from the finger and the finger only flexes away from it.
    """

    footprint = ServoFootprint.of(linkage, actuator)
    sweep = linkage.sweep_pip(pip_samples_rad)
    a = np.array((linkage.a.x_mm, linkage.a.z_mm))
    ap = np.array((linkage.ap.x_mm, linkage.ap.z_mm))
    link = footprint.distance_to_segments(np.broadcast_to(a, sweep.bp.shape), sweep.bp)
    return ServoClearance(
        mcp_servo_mm=footprint.distance_to_point(np.zeros(2))
        - footprint.radius_mm - BODY_CLEARANCE_MM,
        pin_mm=float(np.linalg.norm(ap - a)) - footprint.radius_mm - PIN_CLEARANCE_MM,
        link_mm=(float(link.min()) if link.size else math.inf) - BODY_CLEARANCE_MM,
    )


@dataclass(frozen=True)
class ServoFootprint:
    """The PIP servo section in the proximal-link frame.

    A disk of ``radius_mm`` around the shaft ``pivot`` (``A'``) and the rectangle reaching
    ``rear_extent_mm`` from it along the unit ``rear`` direction, ``radius_mm``
    to either side.
    """

    pivot: tuple[float, float]
    rear: tuple[float, float]
    radius_mm: float
    rear_extent_mm: float

    @classmethod
    def of(
        cls, linkage: FingerLinkage, actuator: XM335Actuator | None = None
    ) -> "ServoFootprint":
        geometry = (actuator or XM335Actuator()).geometry
        return cls(
            (linkage.ap.x_mm, linkage.ap.z_mm),
            (-1.0, 0.0),
            geometry.radius_mm,
            geometry.rear_extent_mm,
        )

    @property
    def outline(self) -> np.ndarray:
        """The closed D outline in the plane, ``(n, 2)``."""

        r = self.radius_mm
        cap = [(r * math.cos(t), r * math.sin(t))
               for t in (math.pi / 2 - math.pi * k / 24 for k in range(25))]
        local = np.array(cap + [(-self.rear_extent_mm, -r), (-self.rear_extent_mm, r)])
        forward, side = self._axes()
        return np.array(self.pivot) + local[:, :1] * forward + local[:, 1:] * side

    @property
    def lowest_z_mm(self) -> float:
        """The most palmar ``z`` of the footprint."""

        return float(self.outline[:, 1].min())

    def distance_to_point(self, point: np.ndarray) -> float:
        return float(self.distance_to_segments(point[None], point[None])[0])

    def distance_to_segments(self, start: np.ndarray, end: np.ndarray) -> np.ndarray:
        """Distance from each segment ``start[i]-end[i]`` to the footprint (0 if touching)."""

        start, end = self._to_local(start), self._to_local(end)
        r, rear = self.radius_mm, self.rear_extent_mm
        disk = np.maximum(_point_segment_distance(np.zeros(2), start, end) - r, 0.0)
        corners = np.array(((-rear, -r), (0.0, -r), (0.0, r), (-rear, r)))
        to_box = np.minimum(self._box_distance(start), self._box_distance(end))
        for k in range(4):
            to_box = np.minimum(to_box, _point_segment_distance(corners[k], start, end))
            crosses = _segments_cross(start, end, corners[k], corners[(k + 1) % 4])
            to_box = np.where(crosses, 0.0, to_box)
        return np.minimum(disk, to_box)

    def _axes(self) -> tuple[np.ndarray, np.ndarray]:
        """Unit vectors of the servo frame: ``forward`` (away from the rear body) and ``side``."""

        forward = -np.array(self.rear)
        return forward, np.array((-forward[1], forward[0]))

    def _to_local(self, points: np.ndarray) -> np.ndarray:
        """Plane points in servo coordinates: pivot at 0, rear body along ``-x``."""

        forward, side = self._axes()
        delta = np.asarray(points, dtype=float) - np.array(self.pivot)
        return np.stack((delta @ forward, delta @ side), axis=-1)

    def _box_distance(self, points: np.ndarray) -> np.ndarray:
        r = self.radius_mm
        dx = np.maximum.reduce((-self.rear_extent_mm - points[:, 0], np.zeros(len(points)), points[:, 0]))
        dz = np.maximum(np.abs(points[:, 1]) - r, 0.0)
        return np.hypot(dx, dz)


def _point_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> np.ndarray:
    direction = end - start
    length_squared = np.sum(direction * direction, axis=1)
    safe = np.where(length_squared > 0.0, length_squared, 1.0)
    t = np.where(length_squared > 0.0, np.sum((point - start) * direction, axis=1) / safe, 0.0)
    nearest = start + np.clip(t, 0.0, 1.0)[:, None] * direction
    return np.linalg.norm(point - nearest, axis=1)


def _segments_cross(start: np.ndarray, end: np.ndarray, q0: np.ndarray, q1: np.ndarray) -> np.ndarray:
    def cross(o, p, q):
        return (p[..., 0] - o[..., 0]) * (q[..., 1] - o[..., 1]) - (p[..., 1] - o[..., 1]) * (q[..., 0] - o[..., 0])

    d1, d2 = cross(start, end, q0), cross(start, end, q1)
    d3, d4 = cross(q0, q1, start), cross(q0, q1, end)
    return (d1 * d2 <= 0.0) & (d3 * d4 <= 0.0)


__all__ = [
    "BODY_CLEARANCE_MM",
    "DEFAULT_PIP_SAMPLES_RAD",
    "PIN_CLEARANCE_MM",
    "ServoClearance",
    "ServoFootprint",
    "servo_clearance",
]
