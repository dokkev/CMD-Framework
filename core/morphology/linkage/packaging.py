"""Planar packaging of the finger: servos, pin housings and their clearances.

The PIP servo rides on the proximal link with its shaft (along ``Y``) on
``A'`` and drives the coupler ``A'-B``; sitting dorsal of ``OA``, it leaves
the proximal contact interface free. Its rear body lies back along the link,
along ``-X`` tilted dorsally by the design's ``pip_servo_tilt_rad``. The PIP
pivot ``A`` hangs off the servo body on a bracket, in the same plane, so its
bushing housing must clear the body. The MCP servo's shaft is on ``O``.

Each servo's section in the linkage plane is a D shape: a disk of the
actuator radius around the shaft and a rectangle reaching ``rear_extent_mm``
back from it. :class:`Packaging` holds every hardware dimension these checks
use, so another servo or bushing is a different ``Packaging``. All checks are
planar, in the proximal-link (reference) frame. Margins are in mm, feasible
when >= 0.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from core.hardware import DShapedExtrusion, XM335Actuator
from core.morphology.linkage import FingerLinkage


BUSHING_INNER_DIAMETER_MM = 3.0
BUSHING_OUTER_DIAMETER_MM = 4.5
"""Pin bushing: McMaster 2706T72, high-load dry-running flanged sleeve bearing,
3 mm shaft, 4.5 mm OD, 5 mm long, 7.5 x 0.75 mm flange."""
HOUSING_WALL_MM = 1.0
"""Housing wall around a bushing, machined aluminium."""
BODY_CLEARANCE_MM = 1.0
"""Servo to servo, servo to bushing housing, and servo to moving parts."""
DEFAULT_PIP_SAMPLES_RAD = tuple(math.radians(degrees) for degrees in range(0, 91))


def _xm335() -> DShapedExtrusion:
    return XM335Actuator().geometry


@dataclass(frozen=True)
class Packaging:
    """Hardware dimensions of the finger's planar packaging.

    ``mcp_servo_rear`` is the unit direction of the MCP servo's rear body in
    the linkage frame at zero MCP flexion (``+Z``: standing dorsally).
    """

    pip_servo: DShapedExtrusion = field(default_factory=_xm335)
    mcp_servo: DShapedExtrusion = field(default_factory=_xm335)
    mcp_servo_rear: tuple[float, float] = (0.0, 1.0)
    bushing_outer_diameter_mm: float = BUSHING_OUTER_DIAMETER_MM
    housing_wall_mm: float = HOUSING_WALL_MM
    body_clearance_mm: float = BODY_CLEARANCE_MM

    def __post_init__(self) -> None:
        length = math.hypot(*self.mcp_servo_rear)
        if length <= 0.0:
            raise ValueError("mcp_servo_rear must be a non-zero direction")
        object.__setattr__(
            self, "mcp_servo_rear", (self.mcp_servo_rear[0] / length, self.mcp_servo_rear[1] / length)
        )

    @property
    def housing_radius_mm(self) -> float:
        """Outer radius of a bushing housing."""

        return self.bushing_outer_diameter_mm / 2.0 + self.housing_wall_mm

    @property
    def pin_spacing_mm(self) -> float:
        """Least centre distance of two pins on one rigid body: their housings touch."""

        return 2.0 * self.housing_radius_mm

    def pip_footprint(self, linkage: FingerLinkage) -> "ServoFootprint":
        """The PIP servo on ``A'``, rear body along ``-X`` tilted dorsally."""

        tilt = linkage.parameters.pip_servo_tilt_rad
        return ServoFootprint(
            (linkage.ap.x_mm, linkage.ap.z_mm),
            (-math.cos(tilt), math.sin(tilt)),
            self.pip_servo.radius_mm,
            self.pip_servo.rear_extent_mm,
        )

    def mcp_footprint(self, mcp_flexion_rad: float = 0.0) -> "ServoFootprint":
        """The MCP servo on ``O`` as seen from the proximal link at an MCP flexion.

        MCP flexion ``q`` turns the hand-fixed MCP servo by ``+q`` about ``O``
        in the proximal-link frame.
        """

        footprint = ServoFootprint(
            (0.0, 0.0), self.mcp_servo_rear, self.mcp_servo.radius_mm, self.mcp_servo.rear_extent_mm
        )
        return footprint.turned(mcp_flexion_rad)

    def mcp_servo_gap_mm(self, pip_servo: "ServoFootprint", mcp_samples_rad: Sequence[float]) -> float:
        """Least gap between the PIP and MCP servo bodies over the MCP flexion samples."""

        mcp = self.mcp_footprint()
        # Turning the MCP servo by +q is turning the PIP servo by -q about O.
        pip_outline, mcp_outline = pip_servo.outline, mcp.outline
        pip_edges = [turn_points(pip_outline, -q) for q in mcp_samples_rad]
        mcp_edges = [turn_points(mcp_outline, q) for q in mcp_samples_rad]
        start = np.concatenate(pip_edges)
        end = np.concatenate([np.roll(edge, -1, axis=0) for edge in pip_edges])
        to_mcp = float(mcp.distance_to_segments(start, end).min())
        start = np.concatenate(mcp_edges)
        end = np.concatenate([np.roll(edge, -1, axis=0) for edge in mcp_edges])
        return min(to_mcp, float(pip_servo.distance_to_segments(start, end).min()))


HOUSING_RADIUS_MM = Packaging().housing_radius_mm
"""Outer radius of a bushing housing with the default packaging (8 mm diameter)."""
PIN_SPACING_MM = Packaging().pin_spacing_mm


@dataclass(frozen=True)
class ServoClearance:
    """Signed clearance margins of one linkage design (mm, >= 0 is feasible)."""

    mcp_servo_mm: float
    """PIP servo beyond the MCP servo body over the MCP angles, less the body clearance."""
    pin_mm: float
    """The PIP bushing housing at ``A`` beyond the servo body, less the body clearance."""
    dip_pin_mm: float
    """The DIP bushing housing at ``B'`` beyond the servo body over the PIP samples,
    less the body clearance."""
    link_mm: float
    """The middle link ``A-B'`` beyond the servo over the PIP samples. Reported
    only: its plates straddle the servo in other layers."""

    @property
    def feasible(self) -> bool:
        return min(self.mcp_servo_mm, self.pin_mm, self.dip_pin_mm, self.link_mm) >= 0.0


def servo_clearance(
    linkage: FingerLinkage,
    packaging: Packaging | None = None,
    pip_samples_rad: Sequence[float] = DEFAULT_PIP_SAMPLES_RAD,
    mcp_samples_rad: Sequence[float] = (0.0,),
) -> ServoClearance:
    """Return the servo clearance margins over the increasing PIP samples.

    The pins and link are checked up to the last sample that closes. The PIP
    servo is checked against the MCP servo body at each MCP flexion sample;
    extension (negative flexion) brings it toward the MCP servo's rear body.
    """

    packaging = packaging or Packaging()
    footprint = packaging.pip_footprint(linkage)
    sweep = linkage.sweep_pip(pip_samples_rad)
    a = np.array((linkage.a.x_mm, linkage.a.z_mm))
    clearance = packaging.body_clearance_mm
    housing = packaging.housing_radius_mm
    link = footprint.distance_to_segments(np.broadcast_to(a, sweep.bp.shape), sweep.bp)
    dip = footprint.distance_to_segments(sweep.bp, sweep.bp)
    return ServoClearance(
        mcp_servo_mm=packaging.mcp_servo_gap_mm(footprint, mcp_samples_rad) - clearance,
        pin_mm=footprint.distance_to_point(a) - housing - clearance,
        dip_pin_mm=(float(dip.min()) if dip.size else math.inf) - housing - clearance,
        link_mm=(float(link.min()) if link.size else math.inf) - clearance,
    )


@dataclass(frozen=True)
class ServoFootprint:
    """A servo's D-shaped section in the linkage plane.

    A disk of ``radius_mm`` around the shaft ``pivot`` and the rectangle
    reaching ``rear_extent_mm`` from it along the unit ``rear`` direction,
    ``radius_mm`` to either side.
    """

    pivot: tuple[float, float]
    rear: tuple[float, float]
    radius_mm: float
    rear_extent_mm: float

    @property
    def reach_mm(self) -> float:
        """Radius of the circle around the shaft that contains the footprint."""

        return math.hypot(self.radius_mm, self.rear_extent_mm)

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

        start, end = self._to_local(start), self._to_local(end)
        r, rear = self.radius_mm, self.rear_extent_mm
        disk = np.maximum(point_segment_distance(np.zeros(2), start, end) - r, 0.0)
        corners = np.array(((-rear, -r), (0.0, -r), (0.0, r), (-rear, r)))
        to_box = np.minimum(self._box_distance(start), self._box_distance(end))
        for k in range(4):
            to_box = np.minimum(to_box, point_segment_distance(corners[k], start, end))
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


def turn_points(points: np.ndarray, angle_rad: float) -> np.ndarray:
    """Points ``(n, 2)`` turned by ``angle_rad`` about the origin ``O``."""

    cosine, sine = math.cos(angle_rad), math.sin(angle_rad)
    return np.stack((cosine * points[:, 0] - sine * points[:, 1],
                     sine * points[:, 0] + cosine * points[:, 1]), axis=1)



def point_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> np.ndarray:
    """Distance from ``point`` to each segment ``start[i]-end[i]``."""

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
    "BUSHING_INNER_DIAMETER_MM",
    "BUSHING_OUTER_DIAMETER_MM",
    "DEFAULT_PIP_SAMPLES_RAD",
    "HOUSING_RADIUS_MM",
    "HOUSING_WALL_MM",
    "PIN_SPACING_MM",
    "Packaging",
    "ServoClearance",
    "ServoFootprint",
    "point_segment_distance",
    "turn_points",
    "servo_clearance",
]
