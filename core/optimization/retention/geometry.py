"""Contact modules and the planar solids they must clear.

Everything is in the proximal-segment frame (``+X`` along ``O-A``, ``+Z``
dorsal), sampled over the swept motion: ``n`` poses, so a solid fixed to the
proximal body is the same at every sample and one that moves is posed at each.
The gaps reuse the linkage packaging's (``core.morphology.linkage.packaging``).
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from core.morphology.linkage.packaging import (
    Packaging,
    ServoFootprint,
    _inside,
    _mcp_gap,
    _polygon_to_disk,
    _polygon_to_footprint,
    _polygons_gap,
    _segments_cross,
    point_segment_distance,
)


@dataclass(frozen=True)
class Module:
    """One contact module: the nominal ``a-b-d-c`` block of the linkage pads
    (``core.morphology.linkage.packaging.pad_outline``, no chamfer), its outer
    edge ``a-c`` fixed and its joint-facing vertices ``b`` (on the mounting
    line) and ``d`` (``height`` palmar of it) retained ``line_mm`` and
    ``palmar_mm`` from that edge, at most ``nominal_mm``."""

    outer_mm: float
    """Along the mounting line, of the outer edge ``a-c``."""
    toward: float
    """``+1``: the joint lies toward larger ``u`` (the proximal module); ``-1``: smaller (distal)."""
    height_mm: float
    nominal_mm: float
    line_mm: float
    palmar_mm: float

    @property
    def retention(self) -> float:
        """Area kept over the nominal area: the trapezoid over the rectangle."""

        return (self.line_mm + self.palmar_mm) / (2.0 * self.nominal_mm)

    @property
    def empty(self) -> bool:
        return self.line_mm <= 0.0 and self.palmar_mm <= 0.0

    def outline(self) -> np.ndarray:
        """``(u, v)`` in the mounting-line frame (``v`` dorsal), ``(4, 2)``, one turn sense."""

        o, h, s = self.outer_mm, self.height_mm, self.toward
        a, c = (o, 0.0), (o, -h)
        b, d = (o + s * self.line_mm, 0.0), (o + s * self.palmar_mm, -h)
        return np.array((c, d, b, a) if s > 0 else (d, c, a, b))


def place(local: np.ndarray, origin: np.ndarray, direction: np.ndarray) -> np.ndarray:
    """Points ``(m, 2)`` given along ``direction`` (``u``) and dorsal of it (``v``)
    in a frame at ``origin``, posed at each of ``(n, 2)`` origins and unit
    directions: ``(n, m, 2)``."""

    origin, e = np.atleast_2d(origin), np.atleast_2d(direction)
    n = np.stack((-e[:, 1], e[:, 0]), axis=1)
    return origin[:, None] + local[None, :, :1] * e[:, None] + local[None, :, 1:] * n[:, None]


@dataclass(frozen=True)
class Polygon:
    points: np.ndarray
    """``(n, m, 2)`` convex polygons."""


@dataclass(frozen=True)
class Disk:
    centers: np.ndarray
    """``(n, 2)``."""
    radius_mm: float


@dataclass(frozen=True)
class Segment:
    """A link's centreline, ``(n, 2)`` ends."""

    start: np.ndarray
    end: np.ndarray


@dataclass(frozen=True)
class Housing:
    """A servo housing fixed to the proximal body (the same at every sample)."""

    footprint: ServoFootprint


MCP_SERVO = "mcp_servo"
"""The MCP servo, fixed to the palm: the MCP flexion turns it about ``O`` in the
proximal frame, so its gaps are the worst over the MCP samples."""

_RANK = {Polygon: 0, Disk: 1, Segment: 2, Housing: 3, str: 4}


def gap(first, second, count: int, packaging: Packaging, mcp_samples_rad: Sequence[float]) -> np.ndarray:
    """Per-sample gap between two solids (``(count,)``, 0 or less when they touch)."""

    if _RANK[type(first)] > _RANK[type(second)]:
        first, second = second, first
    kinds = (type(first), type(second))
    if kinds == (Polygon, Polygon):
        return _polygons_gap(first.points, second.points)
    if kinds == (Polygon, Disk):
        return _polygon_to_disk(first.points, second.centers, second.radius_mm)
    if kinds == (Polygon, Segment):
        return _polygon_to_segment(first.points, second.start, second.end)
    if kinds == (Polygon, Housing):
        return _polygon_to_footprint(first.points, second.footprint)
    if kinds == (Polygon, str):
        return _mcp_gap(first.points, packaging, mcp_samples_rad)
    if kinds == (Disk, Disk):
        return np.linalg.norm(first.centers - second.centers, axis=1) - first.radius_mm - second.radius_mm
    if kinds == (Disk, Segment):
        return point_segment_distance(first.centers, second.start, second.end) - first.radius_mm
    if kinds == (Disk, Housing):
        return second.footprint.distance_to_segments(first.centers, first.centers) - first.radius_mm
    if kinds == (Disk, str):
        return _disk_to_mcp(first, packaging, mcp_samples_rad)
    if kinds == (Segment, Housing):
        return second.footprint.distance_to_segments(first.start, first.end)
    if kinds == (Segment, str):
        return _mcp_gap(np.stack((first.start, first.end), axis=1), packaging, mcp_samples_rad)
    if kinds == (Housing, str):
        return np.full(count, packaging.mcp_servo_gap_mm(first.footprint, mcp_samples_rad))
    raise ValueError(f"no gap between a {kinds[0].__name__} and a {kinds[1].__name__}")


def _polygon_to_segment(polygons: np.ndarray, start: np.ndarray, end: np.ndarray) -> np.ndarray:
    """Gap from ``(n, m, 2)`` convex polygons to segments ``(n, 2)`` (0 when they touch or cross)."""

    count, sides = polygons.shape[:2]
    p0, p1 = polygons.reshape(-1, 2), np.roll(polygons, -1, axis=1).reshape(-1, 2)
    s, e = np.repeat(start, sides, axis=0), np.repeat(end, sides, axis=0)
    distance = np.minimum.reduce((
        point_segment_distance(p0, s, e), point_segment_distance(p1, s, e),
        point_segment_distance(s, p0, p1), point_segment_distance(e, p0, p1),
    ))
    distance = np.where(_segments_cross(p0, p1, s, e), 0.0, distance).reshape(count, sides).min(axis=1)
    inside = _inside(np.stack((start, end), axis=1), polygons).any(axis=1)
    return np.where(inside, 0.0, distance)


def _disk_to_mcp(disk: Disk, packaging: Packaging, mcp_samples_rad: Sequence[float]) -> np.ndarray:
    # Turning the MCP servo by +q is turning the disk by -q about O.
    q = -np.asarray(mcp_samples_rad, dtype=float)[:, None]
    x, z = disk.centers[:, 0], disk.centers[:, 1]
    points = np.stack((np.cos(q) * x - np.sin(q) * z, np.sin(q) * x + np.cos(q) * z), axis=-1).reshape(-1, 2)
    distance = packaging.mcp_footprint().distance_to_segments(points, points)
    return distance.reshape(len(q), -1).min(axis=0) - disk.radius_mm


__all__ = ["MCP_SERVO", "Disk", "Housing", "Module", "Polygon", "Segment", "gap", "place"]
