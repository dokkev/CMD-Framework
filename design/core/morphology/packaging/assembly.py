"""A finger as rigid bodies of planar solids, and the collision checks between them.

A mechanism lays its parts out once per packaging (:class:`Layout`): each
:class:`Part` on a rigid body (``"palm"``, ``"proximal"``, ...) and in a set
of Y layers (names, no thickness: the model is planar), the joints (a pin and
the two bodies it joins), the pairs checked whatever the rule
(:class:`KeepOut`: two parts of one body kept apart, two pins on one link)
and the pairs the rule would check that the model leaves out
(:attr:`Layout.unchecked`). The checked pairs follow from it
(:attr:`Layout.pairs`): two parts are checked when they sit on different
bodies, share a layer and are not one joint (a pin and a body it joins). A
design gives each part its shape (:class:`Assembly`).

The kinematics poses the bodies (:class:`Poses`): each moving body's frame
(origin and unit ``x`` axis) at every sample and the joints' points; the
palm turns about ``O`` with the MCP flexion. :func:`margins` measures each
pair's gap beyond its clearance at every sample. Far pairs may report a
cheaper lower bound (a bounding-box gap) once beyond the body clearance plus
``gaps.EXACT_BAND_MM``; the palm's servo is checked over the MCP samples.
"""

from __future__ import annotations

import functools
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

import numpy as np

from core.morphology.packaging import gaps as _gaps
from core.morphology.packaging.gaps import mcp_servo_gaps, proximal_pad_mcp_gap
from core.morphology.packaging.hardware import MARGIN_TOLERANCE_MM, Packaging
from core.morphology.packaging.shapes import (
    ServoFootprint,
    point_segment_distance,
    polygon_to_disk,
    polygon_to_footprint,
    polygons_gap,
    segment_polygon_gaps,
    segments_cross,
    signed_gaps,
)

PALM = "palm"
"""The body fixed to the hand: in the proximal frame it turns by ``+q`` about ``O`` at MCP flexion ``q``."""


@dataclass(frozen=True)
class Polygon:
    """A convex polygon ``(m, 2)`` in its body's frame."""

    points: np.ndarray


@dataclass(frozen=True)
class Disk:
    """A disk on a joint's point (a pin's housing, a clearance around a shaft)."""

    at: str
    radius_mm: float


@dataclass(frozen=True)
class Bar:
    """A bar between two joints' points, a capsule of ``radius_mm`` about its centre line."""

    start: str
    end: str
    radius_mm: float = 0.0


@dataclass(frozen=True)
class Footprint:
    """A servo's D-shaped section in its body's frame."""

    footprint: ServoFootprint


Shape = Polygon | Disk | Bar | Footprint


@dataclass(frozen=True)
class Part:
    name: str
    body: str
    layers: frozenset[str]
    pad: bool = False
    """A contact pad: kept the packaging's MCP clearance from the palm's servo
    (``Packaging.mcp_pad_clearance_mm``), the body clearance from the rest."""


@dataclass(frozen=True)
class Joint:
    """A pin (a solid's name) and the two bodies it joins: the joint itself, not a collision."""

    pin: str
    bodies: tuple[str, str]


@dataclass(frozen=True)
class KeepOut:
    """Two parts checked whatever the rule (two parts of one body kept apart,
    two pins on one link): with the body clearance, or touching allowed (``touch``)."""

    first: str
    second: str
    touch: bool = False


@dataclass(frozen=True)
class Pair:
    """A checked pair, named by its parts' sorted names."""

    first: str
    second: str
    clearance: str = "body"
    """``"body"``, ``"mcp"`` (a pad and the palm's servo) or ``"touch"`` (none)."""

    @property
    def key(self) -> tuple[str, str]:
        return (self.first, self.second)


@dataclass(frozen=True)
class Layout:
    """A mechanism's parts, in the order their pairs are listed (each part
    with every later one), and how they join."""

    parts: tuple[Part, ...]
    joints: tuple[Joint, ...] = ()
    keep_outs: tuple[KeepOut, ...] = ()
    unchecked: frozenset[tuple[str, str]] = frozenset()
    """Pairs (sorted names) the rule would check that this model leaves out."""

    def part(self, name: str) -> Part:
        return next(part for part in self.parts if part.name == name)

    @property
    def pairs(self) -> tuple[Pair, ...]:
        """The pairs the rule checks: of different bodies sharing a layer and
        not one joint, less the unchecked and the keep-outs' (each part with
        every later one, in the parts' order)."""

        return _pairs(self)

    @property
    def kept(self) -> tuple[Pair, ...]:
        """The keep-outs' pairs, in their order."""

        return tuple(Pair(*sorted((k.first, k.second)), "touch" if k.touch else "body") for k in self.keep_outs)

    def pairs_with(self, names: Sequence[str]) -> tuple[Pair, ...]:
        """The rule's pairs that involve any of ``names``."""

        names = set(names)
        return tuple(pair for pair in self.pairs if pair.first in names or pair.second in names)

    def pair(self, first: str, second: str) -> Pair | None:
        """The checked pair (the rule's or a keep-out's) of two parts; ``None``: not checked."""

        key = tuple(sorted((first, second)))
        return next((pair for pair in (*self.pairs, *self.kept) if pair.key == key), None)

    def separated(self, names: Sequence[str]) -> tuple[tuple[str, str], ...]:
        """Pairs of ``names`` with parts of other bodies they share no layer
        with: they may cross in the side view, not across the finger."""

        found = []
        for part in self.parts:
            if part.name not in names:
                continue
            for other in self.parts:
                key = tuple(sorted((part.name, other.name)))
                if other.body != part.body and other.layers and not part.layers & other.layers \
                        and key not in found:
                    found.append(key)
        return tuple(found)


@dataclass(frozen=True)
class Assembly:
    """One design: its layout and each part's shape in its body's frame."""

    layout: Layout
    shapes: Mapping[str, Shape]

    def solid(self, name: str) -> tuple[Part, Shape]:
        return self.layout.part(name), self.shapes[name]


@functools.lru_cache(maxsize=256)
def _pairs(layout: Layout) -> tuple[Pair, ...]:
    names = {part.name for part in layout.parts}
    for keep in layout.keep_outs:
        if keep.first not in names or keep.second not in names:
            raise ValueError(f"no part {keep.first!r} or {keep.second!r}")
    joined = {(joint.pin, body) for joint in layout.joints for body in joint.bodies}
    pads = {part.name for part in layout.parts if part.pad}
    kept = {tuple(sorted((k.first, k.second))) for k in layout.keep_outs}
    pairs = []
    for i, part in enumerate(layout.parts):
        for other in layout.parts[i + 1:]:
            key = tuple(sorted((part.name, other.name)))
            if part.body == other.body or not part.layers & other.layers or key in layout.unchecked or key in kept:
                continue
            if (part.name, other.body) in joined or (other.name, part.body) in joined:
                continue
            palm_pad = (part.body == PALM and other.name in pads) or (other.body == PALM and part.name in pads)
            pairs.append(Pair(*key, "mcp" if palm_pad else "body"))
    return tuple(pairs)


@dataclass(frozen=True)
class Poses:
    """The bodies at ``count`` samples: each moving body's frame (``origin``
    and unit ``axis``, ``(n, 2)``; the others fixed in the proximal frame),
    the joints' points (``(n, 2)``, or ``(2,)`` fixed) and the MCP flexions
    the palm is checked over."""

    count: int
    origin: Mapping[str, np.ndarray] = field(default_factory=dict)
    axis: Mapping[str, np.ndarray] = field(default_factory=dict)
    points: Mapping[str, np.ndarray] = field(default_factory=dict)
    mcp_samples_rad: Sequence[float] = (0.0,)

    def moving(self, body: str) -> bool:
        return body in self.origin

    def polygon(self, body: str, shape: Polygon) -> np.ndarray:
        """A polygon of ``body`` at every sample, ``(n, m, 2)``."""

        local = shape.points
        if not self.moving(body):
            return np.broadcast_to(local, (self.count, *local.shape))
        origin, e = self.origin[body], self.axis[body]
        n = np.stack((-e[:, 1], e[:, 0]), axis=1)
        return origin[:, None] + local[None, :, :1] * e[:, None] + local[None, :, 1:] * n[:, None]

    def point(self, name: str) -> np.ndarray:
        return np.broadcast_to(self.points[name], (self.count, 2))


def margins(assembly: Assembly, poses: Poses, packaging: Packaging, pairs: Sequence[Pair] | None = None,
            snap: bool = True) -> dict[tuple[str, str], np.ndarray]:
    """Each pair's margin beyond its clearance at every sample (``(n,)``, mm;
    negative: too near), by its key; every checked pair by default. ``snap``:
    a margin within ``MARGIN_TOLERANCE_MM`` of 0 is 0 (a part set exactly the
    clearance away must not fail by rounding)."""

    pairs = (*assembly.layout.pairs, *assembly.layout.kept) if pairs is None else pairs
    posed = _Posed(assembly, poses)
    clearance = {"body": packaging.body_clearance_mm, "mcp": packaging.mcp_pad_clearance_mm, "touch": 0.0}
    result = {}
    for pair in pairs:
        margin = _gap(pair.first, pair.second, posed, packaging) - clearance[pair.clearance]
        result[pair.key] = np.where(np.abs(margin) < MARGIN_TOLERANCE_MM, 0.0, margin) if snap else margin
    return result


class _Posed:
    """An assembly's parts posed once per call, with their bounding boxes."""

    def __init__(self, assembly: Assembly, poses: Poses) -> None:
        self.assembly, self.poses = assembly, poses
        self._polygons: dict[str, np.ndarray] = {}
        self._boxes: dict[str, tuple[np.ndarray, np.ndarray]] = {}

    def part(self, name: str) -> Part:
        return self.assembly.layout.part(name)

    def shape(self, name: str) -> Shape:
        return self.assembly.shapes[name]

    def polygon(self, name: str) -> np.ndarray:
        if name not in self._polygons:
            self._polygons[name] = self.poses.polygon(self.part(name).body, self.shape(name))
        return self._polygons[name]

    def box(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        if name not in self._boxes:
            shape, count = self.shape(name), self.poses.count
            if isinstance(shape, Polygon):
                polygon = self.polygon(name)
                box = (polygon.min(axis=1), polygon.max(axis=1))
            elif isinstance(shape, Disk):
                center = self.poses.point(shape.at)
                box = (center - shape.radius_mm, center + shape.radius_mm)
            elif isinstance(shape, Footprint):
                outline = shape.footprint.outline
                box = (np.broadcast_to(outline.min(axis=0), (count, 2)), np.broadcast_to(outline.max(axis=0), (count, 2)))
            else:
                start, end = self.poses.point(shape.start), self.poses.point(shape.end)
                box = (np.minimum(start, end) - shape.radius_mm, np.maximum(start, end) + shape.radius_mm)
            self._boxes[name] = box
        return self._boxes[name]


_RANK = {Polygon: 0, Disk: 1, Bar: 2, Footprint: 3}


def _gap(first: str, second: str, posed: _Posed, packaging: Packaging) -> np.ndarray:
    """Two parts' gap at every sample (signed by the overlap's depth as the
    packaging says). Parts of different bodies past the exact band may report
    their bounding boxes' gap; one body's (a keep-out) are always exact."""

    poses = posed.poses
    if posed.part(second).body == PALM:
        first, second = second, first
    if posed.part(first).body == PALM:
        return _palm_gap(second, first, posed, packaging)
    if _RANK[type(posed.shape(first))] > _RANK[type(posed.shape(second))]:
        first, second = second, first
    one, other = posed.shape(first), posed.shape(second)
    count = poses.count
    if isinstance(one, Polygon) and not isinstance(other, Bar):
        if posed.part(first).body != posed.part(second).body:
            # Broad phase: the bounding boxes' gap bounds the gap from below.
            (low, high), (other_low, other_high) = posed.box(first), posed.box(second)
            result = np.hypot(*np.maximum(np.maximum(other_low - high, low - other_high), 0.0).T)
            near = np.flatnonzero(result <= packaging.body_clearance_mm + _gaps.EXACT_BAND_MM)
            if not near.size:
                return result
        else:
            result, near = np.zeros(count), np.arange(count)
        polygon = posed.polygon(first)[near]
        if isinstance(other, Disk):  # signed already: the centre's distance less the radius
            result[near] = polygon_to_disk(polygon, poses.point(other.at)[near], other.radius_mm)
            return result
        if isinstance(other, Polygon):
            solid = posed.polygon(second)[near]
            result[near] = polygons_gap(polygon, solid)
        else:
            outline = other.footprint.outline
            solid = np.broadcast_to(outline, (near.size, *outline.shape))
            result[near] = polygon_to_footprint(polygon, other.footprint)
        if packaging.overlap_depth:
            result[near] = signed_gaps(result[near], polygon, solid)
        return result
    if isinstance(one, Polygon):  # and a bar
        return segment_polygon_gaps(posed.polygon(first), poses.point(other.start), poses.point(other.end)) \
            - other.radius_mm
    if isinstance(one, Disk) and isinstance(other, Disk):
        apart = poses.point(one.at) - poses.point(other.at)
        distance = np.fromiter((math.hypot(dx, dz) for dx, dz in apart), float, count)
        return distance - (one.radius_mm + other.radius_mm)
    if isinstance(one, Disk) and isinstance(other, Bar):
        center = poses.point(one.at)
        return (point_segment_distance(center, poses.point(other.start), poses.point(other.end))
                - (one.radius_mm + other.radius_mm))
    if isinstance(one, Disk):  # and a footprint
        center = poses.point(one.at)
        return other.footprint.distance_to_segments(center, center) - one.radius_mm
    if isinstance(one, Bar) and isinstance(other, Bar):
        ends = [poses.point(name) for name in (one.start, one.end, other.start, other.end)]
        distance = np.minimum.reduce([point_segment_distance(ends[0], ends[2], ends[3]),
                                      point_segment_distance(ends[1], ends[2], ends[3]),
                                      point_segment_distance(ends[2], ends[0], ends[1]),
                                      point_segment_distance(ends[3], ends[0], ends[1])])
        distance = np.where(segments_cross(*ends), 0.0, distance)
        return distance - (one.radius_mm + other.radius_mm)
    if isinstance(one, Bar):  # and a footprint
        return other.footprint.distance_to_segments(poses.point(one.start), poses.point(one.end)) - one.radius_mm
    return np.full(count, one.footprint.distance_to_footprint(other.footprint))


def _palm_gap(name: str, palm: str, posed: _Posed, packaging: Packaging) -> np.ndarray:
    """A part's gap to the palm's servo over the MCP samples (the palm turning about ``O``)."""

    if posed.shape(palm) != Footprint(packaging.mcp_footprint()):
        raise ValueError("the palm holds the packaging's MCP servo only")
    poses, shape = posed.poses, posed.shape(name)
    samples, count = poses.mcp_samples_rad, poses.count
    if isinstance(shape, Polygon):
        if not poses.moving(posed.part(name).body):  # fixed in the proximal frame: the same at every sample
            return np.full(count, proximal_pad_mcp_gap(shape.points, packaging, samples))
        return mcp_servo_gaps(posed.polygon(name), packaging, samples)
    if isinstance(shape, Footprint):
        # Footprints whose circumscribed circles are apart by more than the body clearance cannot touch.
        footprint = shape.footprint
        apart = math.hypot(*footprint.pivot) - footprint.reach_mm - packaging.mcp_footprint().reach_mm
        gap = apart if apart > packaging.body_clearance_mm else packaging.mcp_servo_gap_mm(footprint, samples)
        return np.full(count, gap)
    if isinstance(shape, Bar):
        segment = np.stack((poses.point(shape.start), poses.point(shape.end)), axis=1)
        return mcp_servo_gaps(segment, packaging, samples) - shape.radius_mm
    # A disk: turning the palm by +q is turning the disk by -q about O.
    q = -np.asarray(samples, dtype=float)[:, None]
    center = poses.point(shape.at)
    x, z = center[:, 0], center[:, 1]
    turned = np.stack((np.cos(q) * x - np.sin(q) * z, np.sin(q) * x + np.cos(q) * z), axis=-1).reshape(-1, 2)
    distance = packaging.mcp_footprint().distance_to_segments(turned, turned)
    return distance.reshape(len(q), -1).min(axis=0) - shape.radius_mm


__all__ = [
    "PALM",
    "Assembly",
    "Bar",
    "Disk",
    "Footprint",
    "Joint",
    "KeepOut",
    "Layout",
    "Pair",
    "Part",
    "Polygon",
    "Poses",
    "Shape",
    "margins",
]
