"""Planar solids and the gaps between them: convex polygons ``(n, m, 2)`` (one
per sample), disks, segments and a servo's D-shaped section
(:class:`ServoFootprint`), and a contact pad's outline on its mounting line.
Gaps are in mm, 0 where two solids touch or overlap unless signed
(:func:`signed_gaps`)."""

from __future__ import annotations

import functools
import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from core.morphology.packaging import _kernels

if TYPE_CHECKING:
    from core.morphology.packaging.hardware import ContactPad

_APART_MM = 1e-6
"""A gap above this: the convex polygons are apart, their separation positive."""

# A D-shaped servo's cap: the unit half circle from +side round the front to -side.
_CAP_ARC = np.array([(math.cos(t), math.sin(t)) for t in (math.pi / 2 - math.pi * k / 24 for k in range(25))])


def as_points(array) -> np.ndarray:
    return np.ascontiguousarray(np.asarray(array, dtype=float).reshape(-1, 2))


def as_polygons(array) -> np.ndarray:
    array = np.asarray(array, dtype=float)
    return np.ascontiguousarray(array.reshape(-1, *array.shape[-2:]))


def turn_points(points: np.ndarray, angle_rad: float) -> np.ndarray:
    """Points ``(n, 2)`` turned by ``angle_rad`` about the origin ``O``."""

    cosine, sine = math.cos(angle_rad), math.sin(angle_rad)
    return np.stack((cosine * points[:, 0] - sine * points[:, 1],
                     sine * points[:, 0] + cosine * points[:, 1]), axis=1)


def point_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> np.ndarray:
    """Distance from ``point`` (one, or one per segment) to each segment ``start[i]-end[i]``."""

    point, start, end = np.broadcast_arrays(*(np.asarray(a, dtype=float) for a in (point, start, end)))
    return _kernels.point_segment_distance(as_points(point), as_points(start), as_points(end))


def _point_segment_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    d = b - a
    t = np.clip(np.sum((p - a) * d, axis=-1) / np.maximum(np.sum(d * d, axis=-1), 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[..., None] * d), axis=-1)


def segments_cross(start: np.ndarray, end: np.ndarray, q0: np.ndarray, q1: np.ndarray) -> np.ndarray:
    """Whether segment ``start[i]-end[i]`` crosses ``q0[i]-q1[i]``. Collinear
    segments only touch if an endpoint lies on the other, which the endpoint
    distances already find."""

    arrays = np.broadcast_arrays(*(np.asarray(a, dtype=float) for a in (start, end, q0, q1)))
    return _kernels.segments_cross(*(as_points(array) for array in arrays))


def points_inside(points: np.ndarray, polygons: np.ndarray) -> np.ndarray:
    """Whether each of ``(n, p, 2)`` points lies inside (or on) the convex ``(n, m, 2)`` polygon."""

    return _kernels.inside(as_polygons(points), as_polygons(polygons))


def polygons_gap(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """Gap between two ``(n, m, 2)`` convex polygons per sample (0 when they touch or overlap)."""

    return _kernels.polygons_gap(as_polygons(first), as_polygons(second))


def polygon_to_disk(polygons: np.ndarray, centers: np.ndarray, radius: float) -> np.ndarray:
    """Gap from ``(n, m, 2)`` polygons to disks at ``(n, 2)`` centres (negative: overlap)."""

    return _kernels.polygon_to_disk(as_polygons(polygons), as_points(centers), float(radius))


def polygon_edges(polygons: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return polygons, np.roll(polygons, -1, axis=-2)


def separation(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """Signed separation of convex ``(n, m, 2)`` and ``(n, k, 2)`` polygons per
    sample: the most any edge normal of either separates their projections.
    Positive: apart (by at least that); 0: touching; negative: overlapping,
    by that depth."""

    return _kernels.separation(as_polygons(first), as_polygons(second))


def signed_gaps(gap: np.ndarray, first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """A gap that is 0 for touching and overlapping alike, made negative by the
    overlap's depth (:func:`separation`) where the convex polygons overlap."""

    gap = np.asarray(gap, dtype=float)
    touching = np.flatnonzero(gap <= _APART_MM)
    if touching.size == gap.size:
        depth = separation(first, second)
        return np.where(depth < 0.0, depth, gap)
    # Polygons clearly apart have a separating edge (their separation is
    # positive): only the touching samples can turn negative.
    result = gap.copy()
    if touching.size:
        first, second = np.asarray(first), np.asarray(second)
        depth = separation(first[touching], second[touching])
        result[touching] = np.where(depth < 0.0, depth, gap[touching])
    return result


def segment_polygon_gaps(polygons: np.ndarray, start: np.ndarray, end: np.ndarray) -> np.ndarray:
    """Distance (mm) from each segment ``start -> end`` ``(n, 2)`` to the convex
    ``polygons`` ``(n, m, 2)``, 0 where they touch or overlap: a capsule of
    radius ``r`` about the segment clears the polygon by this less ``r``."""

    polygons = np.asarray(polygons, dtype=float)
    start, end = (np.broadcast_to(np.asarray(v, dtype=float), (len(polygons), 2)) for v in (start, end))

    def cross(o, p, q):
        return (p[:, 0] - o[:, 0]) * (q[:, 1] - o[:, 1]) - (p[:, 1] - o[:, 1]) * (q[:, 0] - o[:, 0])

    gap = np.full(len(polygons), np.inf)
    crossing = np.zeros(len(polygons), dtype=bool)
    for corner, following in zip(np.moveaxis(polygons, 1, 0), np.moveaxis(np.roll(polygons, -1, axis=1), 1, 0)):
        d1, d2 = cross(start, end, corner), cross(start, end, following)
        d3, d4 = cross(corner, following, start), cross(corner, following, end)
        crossing |= (d1 * d2 <= 0.0) & (d3 * d4 <= 0.0) & ~((d1 == 0.0) & (d2 == 0.0))
        gap = np.minimum.reduce([gap, _point_segment_distance(start, corner, following),
                                 _point_segment_distance(end, corner, following),
                                 _point_segment_distance(corner, start, end),
                                 _point_segment_distance(following, start, end)])
    inside = points_inside(np.ascontiguousarray(start[:, None]), polygons)[:, 0]
    return np.where(crossing | inside, 0.0, gap)


@dataclass(frozen=True)
class ServoFootprint:
    """A servo's section in the linkage plane, about its shaft ``pivot``.

    The rectangle from ``rear_extent_mm`` behind the shaft (along the unit
    ``rear`` direction) to ``front_mm`` ahead of it, ``radius_mm`` to either
    side, and, for a D-shaped body (``front_mm`` 0), a disk of ``cap_mm``
    (default ``radius_mm``) around the shaft. A box-shaped body (the XC330
    frame) has ``cap_mm`` 0.
    """

    pivot: tuple[float, float]
    rear: tuple[float, float]
    radius_mm: float
    rear_extent_mm: float
    front_mm: float = 0.0
    cap_mm: float | None = None

    @property
    def cap(self) -> float:
        return self.radius_mm if self.cap_mm is None else self.cap_mm

    @property
    def reach_mm(self) -> float:
        """Radius of the circle around the shaft that contains the footprint."""

        r = self.radius_mm
        return max(math.hypot(r, self.rear_extent_mm), math.hypot(r, self.front_mm), self.cap)

    @property
    def outline(self) -> np.ndarray:
        """The closed outline in the plane, ``(n, 2)``: D-shaped, or a box
        (read-only, computed once per distinct footprint)."""

        return _footprint_outline(self)

    @property
    def lowest_z_mm(self) -> float:
        """The most palmar ``z`` of the footprint."""

        return float(self.outline[:, 1].min())

    def turned(self, angle_rad: float) -> "ServoFootprint":
        """This footprint turned by ``angle_rad`` about the origin ``O``."""

        cosine, sine = math.cos(angle_rad), math.sin(angle_rad)
        x, z = self.pivot
        rx, rz = self.rear
        return ServoFootprint(
            (cosine * x - sine * z, sine * x + cosine * z),
            (cosine * rx - sine * rz, sine * rx + cosine * rz),
            self.radius_mm,
            self.rear_extent_mm,
            self.front_mm,
            self.cap_mm,
        )

    def distance_to_footprint(self, other: "ServoFootprint") -> float:
        """Gap between two footprints (0 when they touch or overlap)."""

        def edges(footprint: "ServoFootprint") -> tuple[np.ndarray, np.ndarray]:
            outline = footprint.outline
            return outline, np.roll(outline, -1, axis=0)

        return min(float(self.distance_to_segments(*edges(other)).min()),
                   float(other.distance_to_segments(*edges(self)).min()))

    def distance_to_point(self, point: np.ndarray) -> float:
        return float(self.distance_to_segments(point[None], point[None])[0])

    def distance_to_segments(self, start: np.ndarray, end: np.ndarray) -> np.ndarray:
        """Distance from each segment ``start[i]-end[i]`` to the footprint (0 if touching)."""

        forward, side = self._axes()
        return _kernels.footprint_distance(as_points(start), as_points(end), float(self.pivot[0]), float(self.pivot[1]),
                                           float(forward[0]), float(forward[1]), float(side[0]), float(side[1]),
                                           float(self.radius_mm), float(self.rear_extent_mm),
                                           float(self.front_mm), float(self.cap))

    def _axes(self) -> tuple[np.ndarray, np.ndarray]:
        """Unit vectors of the servo frame: ``forward`` (away from the rear body) and ``side``."""

        forward = -np.array(self.rear)
        return forward, np.array((-forward[1], forward[0]))


@functools.lru_cache(maxsize=4096)
def _footprint_outline(footprint: ServoFootprint) -> np.ndarray:
    r = footprint.radius_mm
    if footprint.cap > 0.0:
        local = np.vstack((footprint.cap * _CAP_ARC,
                           ((-footprint.rear_extent_mm, -r), (-footprint.rear_extent_mm, r))))
    else:
        local = np.array([(footprint.front_mm, r), (footprint.front_mm, -r), (-footprint.rear_extent_mm, -r),
                          (-footprint.rear_extent_mm, r)])
    forward, side = footprint._axes()
    outline = np.array(footprint.pivot) + local[:, :1] * forward + local[:, 1:] * side
    outline.setflags(write=False)
    return outline


def polygon_to_footprint(polygons: np.ndarray, footprint: "ServoFootprint") -> np.ndarray:
    """Gap from ``(n, m, 2)`` polygons to a servo footprint (0 when they touch or overlap)."""

    forward, side = footprint._axes()
    return _kernels.polygon_to_footprint(as_polygons(polygons), float(footprint.pivot[0]), float(footprint.pivot[1]),
                                         float(forward[0]), float(forward[1]), float(side[0]), float(side[1]),
                                         float(footprint.radius_mm), float(footprint.rear_extent_mm),
                                         float(footprint.front_mm), float(footprint.cap))


def servo_footprint(body, pivot: tuple[float, float], rear: tuple[float, float]) -> ServoFootprint:
    """A servo body (``core.hardware``: D-shaped or a box) about its shaft at
    ``pivot``, its rear along ``rear``."""

    return ServoFootprint(tuple(pivot), tuple(rear), body.radius_mm, body.rear_extent_mm,
                          body.front_extent_mm, body.cap_radius_mm)


@dataclass(frozen=True)
class PadSpan:
    """A pad's ``a-b`` along its mounting line and the room the line gives it.

    ``start_mm`` and ``end_mm`` are along the line (``x`` for the proximal pad,
    from ``B`` along ``BC`` for the distal one); ``room_mm`` runs from the
    pad's outer end to ``A''`` or ``B''``.
    """

    start_mm: float
    end_mm: float
    room_mm: float
    limit_mm: float = math.inf
    """Most length allowed by ``pad_link_ratio_max``."""

    @property
    def length_mm(self) -> float:
        return self.end_mm - self.start_mm


def pad_outline(span: PadSpan, pad: ContactPad) -> np.ndarray:
    """A pad in its mounting-line frame (``u`` along the line, ``v`` dorsal),
    ``(4, 2)``: ``a-b`` the span on the line, ``c-d`` the pad's height palmar
    of it. Counter-clockwise, convex."""

    start_mm, end_mm, t = span.start_mm, span.end_mm, pad.height_mm
    a, b, d, c = (start_mm, 0.0), (end_mm, 0.0), (end_mm, -t), (start_mm, -t)
    return np.array((c, d, b, a))


def carrier_outline(pad_outline_mm: np.ndarray, height_mm: float) -> np.ndarray:
    """The carrier plate of a pad given in its mounting-line frame (top face
    at ``v = 0``): the pad's span, ``height_mm`` dorsal of it, ``(4, 2)``."""

    start, end = float(pad_outline_mm[:, 0].min()), float(pad_outline_mm[:, 0].max())
    return np.array(((start, 0.0), (end, 0.0), (end, height_mm), (start, height_mm)))


__all__ = [
    "PadSpan",
    "ServoFootprint",
    "as_points",
    "as_polygons",
    "carrier_outline",
    "pad_outline",
    "point_segment_distance",
    "points_inside",
    "polygon_edges",
    "polygon_to_disk",
    "polygon_to_footprint",
    "servo_footprint",
    "polygons_gap",
    "segment_polygon_gaps",
    "segments_cross",
    "separation",
    "signed_gaps",
    "turn_points",
]
