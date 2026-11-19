"""Closed-form distances between planar primitives (mm).

A gap is the distance between two solids: positive apart, zero touching,
negative overlapping (for a disk: its centre's distance less its radius, so
the overlap depth when the centre is outside the other solid).
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from core._validation import require_nonnegative


def point_rect_distance_mm(box: tuple[float, float, float, float], point: Sequence[float]) -> float:
    """Distance from ``point`` ``(u, v)`` to the solid rectangle ``box`` ``(u0, u1, v0, v1)``
    (bounds in either order); 0 inside."""

    u0, u1, v0, v1 = (float(value) for value in box)
    u0, u1 = min(u0, u1), max(u0, u1)
    v0, v1 = min(v0, v1), max(v0, v1)
    u, v = float(point[0]), float(point[1])
    return math.hypot(max(u0 - u, 0.0, u - u1), max(v0 - v, 0.0, v - v1))


def rect_disk_gap_mm(box: tuple[float, float, float, float], center: Sequence[float], radius_mm: float) -> float:
    """Gap between the rectangle ``box`` ``(u0, u1, v0, v1)`` and the disk ``(center, radius_mm)``."""

    return point_rect_distance_mm(box, center) - require_nonnegative("radius_mm", radius_mm)


def disk_disk_gap_mm(first: Sequence[float], first_radius_mm: float,
                     second: Sequence[float], second_radius_mm: float) -> float:
    """Gap between two disks."""

    distance = math.hypot(float(first[0]) - float(second[0]), float(first[1]) - float(second[1]))
    return distance - (require_nonnegative("first_radius_mm", first_radius_mm)
                       + require_nonnegative("second_radius_mm", second_radius_mm))


__all__ = ["disk_disk_gap_mm", "point_rect_distance_mm", "rect_disk_gap_mm"]
