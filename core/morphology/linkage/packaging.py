"""Planar packaging of the finger: servos, pin housings and their clearances.

The PIP servo rides on the proximal link with its shaft (along ``Y``) on
``A'`` and drives the coupler ``A'-B``; sitting dorsal of ``OA``, it leaves
the proximal contact interface free. Its rear body lies back along the link,
along ``-X`` tilted dorsally by the design's ``pip_servo_tilt_rad``. The PIP
pivot ``A`` hangs off the servo body on a bracket, in the same plane, so its
bearing housing must clear the body. The MCP servo's shaft is on ``O``.

Each servo's section in the linkage plane is a D shape: a disk of the
actuator radius around the shaft and a rectangle reaching ``rear_extent_mm``
back from it. :class:`Packaging` holds every hardware dimension these checks
use, so another servo or bearing is a different ``Packaging``. All checks are
planar, in the proximal-link (reference) frame. Margins are in mm, feasible
when >= 0.

**Contact pads.** Each pad is a block ``a-b-d-c`` fixed to a link and free
to stand out of it: ``a-b`` on the link's mounting line, ``c-d`` its
height palmar of it (:class:`ContactPad`). The proximal pad rides on
``O-A-A'``: it starts at :attr:`Packaging.proximal_pad_start_mm` (``O``, past
the MCP servo's shaft disk by the body clearance when ``pad_mcp_check``, plus
its offset) with a square edge ``a-c`` and runs right to ``A''``. The distal
pad rides on ``B'-B-C``: it starts its offset short of ``C`` and runs left to
``B''``. A pad's length is not set: it ends at ``A''`` or ``B''``. The ends
facing the PIP lose a 45 deg chamfer at their palmar corner. Without
``pad_mcp_check`` the MCP servo is not checked.

The pads are 2.5D solids: XZ polygons with a Y interval (:data:`COMPONENTS`).
They share the central layer with the servos, as wide as the servo; the
middle link and coupler plates are outside it and the pins run through every
layer. Only components in different collision groups (rigid bodies,
:data:`COLLISION_GROUPS`) whose Y intervals overlap are checked
(:func:`checked_pairs`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Sequence

import numpy as np

from core.hardware import DShapedExtrusion, XM335Actuator
from core.morphology.linkage import FingerLinkage, FingerLinkageParameters


PIN_ENVELOPE_DIAMETER_MM = 8.0
"""Metal around a pin in the side view, which the servo bodies keep clear of.
The pins run in McMaster 57155K424 flanged ball bearings (trade no. 63: 3 mm
shaft, 6 mm OD, 2 mm wide, 7.2 x 0.6 mm flange), one on each side of a joint,
pressed into 1 mm aluminium walls: the 8 mm housing is wider than the flange."""
BODY_CLEARANCE_MM = 1.0
"""Servo to servo, servo to pin envelope, servo to moving parts, and the
pads to the parts of other bodies."""
EXACT_BAND_MM = 5.0
"""Pad gaps are exact up to the body clearance plus this; a
larger gap may be reported by a cheaper lower bound (a bounding-box gap, or
``|p| - reach`` against the MCP servo), which is still beyond it."""
_MCP_COARSE_STEP_RAD = math.radians(5.0)
_MARGIN_TOLERANCE_MM = 1e-9
DEFAULT_PIP_SAMPLES_RAD = tuple(math.radians(degrees) for degrees in range(0, 91))


def _xm335() -> DShapedExtrusion:
    return XM335Actuator().geometry


@dataclass(frozen=True)
class ContactPad:
    """One contact pad block (provisional Stage-0 values).

    ``height_mm`` palmar of its mounting line, starting ``offset_mm`` in from
    its outer end (``O`` or the MCP servo, or ``C``), and a 45 deg chamfer of
    ``chamfer_mm`` at its palmar corner facing the PIP. ``length_mm`` fixes
    its length, which the mounting line must have room for; ``None`` runs it
    to ``A''`` or ``B''``, the whole room (see :func:`pad_spans`).
    """

    height_mm: float = 5.0
    offset_mm: float = 0.0
    chamfer_mm: float = 2.5
    length_mm: float | None = None

    def __post_init__(self) -> None:
        if not self.height_mm > 0.0:
            raise ValueError("height_mm must be positive")
        if self.length_mm is not None and not self.length_mm > 0.0:
            raise ValueError("length_mm must be positive, or None")
        if not self.offset_mm >= 0.0:
            raise ValueError("offset_mm must be non-negative")
        if not 0.0 <= self.chamfer_mm <= self.height_mm:
            raise ValueError("chamfer_mm must be within [0, height_mm]")


@dataclass(frozen=True)
class Packaging:
    """Hardware dimensions of the finger's planar packaging.

    ``mcp_servo_rear`` is the unit direction of the MCP servo's rear body in
    the linkage frame at zero MCP flexion (``+Z``: standing dorsally).
    """

    pip_servo: DShapedExtrusion = field(default_factory=_xm335)
    mcp_servo: DShapedExtrusion = field(default_factory=_xm335)
    mcp_servo_rear: tuple[float, float] = (0.0, 1.0)
    pin_envelope_diameter_mm: float = PIN_ENVELOPE_DIAMETER_MM
    body_clearance_mm: float = BODY_CLEARANCE_MM
    proximal_pad: ContactPad = field(default_factory=ContactPad)
    distal_pad: ContactPad = field(default_factory=ContactPad)
    pad_mcp_check: bool = True
    pad_link_ratio_max: float | None = None
    """Most pad length per length of its link (the proximal pad over ``|OA|``,
    the distal pad over ``BC``): a pad may stand out of its link by this much.
    A free-length pad is cut to it; a fixed one must not exceed it. ``None``:
    unbounded."""
    pip_servo_at: str = "ap"
    """Where the PIP servo's shaft is: ``"ap"`` (``A'``, driving ``A'-B``) or
    ``"a"`` (the PIP ``A``, driving the middle link ``A-B'``; body along ``-X``,
    no tilt, the proximal pad on its palmar face)."""

    def __post_init__(self) -> None:
        if self.pad_link_ratio_max is not None and not self.pad_link_ratio_max > 0.0:
            raise ValueError("pad_link_ratio_max must be positive, or None")
        if self.pip_servo_at not in ("ap", "a"):
            raise ValueError("pip_servo_at must be 'ap' or 'a'")
        for name in ("proximal_pad", "distal_pad"):
            if not isinstance(getattr(self, name), ContactPad):
                raise TypeError(f"{name} must be a ContactPad")
        length = math.hypot(*self.mcp_servo_rear)
        if length <= 0.0:
            raise ValueError("mcp_servo_rear must be a non-zero direction")
        object.__setattr__(
            self, "mcp_servo_rear", (self.mcp_servo_rear[0] / length, self.mcp_servo_rear[1] / length)
        )

    @property
    def pin_radius_mm(self) -> float:
        """Radius of a pin's envelope in the plane; clearances are measured from it."""

        return self.pin_envelope_diameter_mm / 2.0

    @property
    def pin_spacing_mm(self) -> float:
        """Least centre distance of two pins on one rigid body: their envelopes touch."""

        return 2.0 * self.pin_radius_mm

    @property
    def proximal_pad_start_mm(self) -> float:
        """``x`` of the proximal pad's first edge ``a-c``: ``O``, or the MCP servo's
        shaft disk plus the body clearance with ``pad_mcp_check``, plus the pad's
        offset."""

        clear = self.mcp_servo.radius_mm + self.body_clearance_mm if self.pad_mcp_check else 0.0
        return clear + self.proximal_pad.offset_mm

    @property
    def pip_servo_on_a(self) -> bool:
        return self.pip_servo_at == "a"

    def proximal_pad_line_z(self, parameters: FingerLinkageParameters) -> float:
        """``z`` of the proximal pad's mounting line: ``O-A''``, or the palmar face
        of the PIP servo on ``A``."""

        return parameters.a_z_mm - self.pip_servo.radius_mm if self.pip_servo_on_a else 0.0

    def layer_y_mm(self, layer: str) -> tuple[float, float]:
        """``|y|`` interval of a layer (``y`` across the finger, 0 at its middle)."""

        central = max(self.pip_servo.depth_mm, self.mcp_servo.depth_mm) / 2.0
        return {"central": (0.0, central), "outer": (central, math.inf),
                "through": (0.0, math.inf)}[layer]

    def pip_footprint(self, linkage: FingerLinkage) -> "ServoFootprint":
        """The PIP servo on ``A'``, rear body along ``-X`` tilted dorsally; or on
        ``A`` (``pip_servo_at``), rear body along ``-X`` (the tilt is ignored)."""

        if self.pip_servo_on_a:
            return ServoFootprint((linkage.a.x_mm, linkage.a.z_mm), (-1.0, 0.0),
                                  self.pip_servo.radius_mm, self.pip_servo.rear_extent_mm)
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


PIN_RADIUS_MM = Packaging().pin_radius_mm
PIN_SPACING_MM = Packaging().pin_spacing_mm


@dataclass(frozen=True)
class ServoClearance:
    """Signed clearance margins of one linkage design (mm, >= 0 is feasible)."""

    mcp_servo_mm: float
    """PIP servo beyond the MCP servo body over the MCP angles, less the body clearance."""
    pin_mm: float
    """The PIP pin envelope at ``A`` beyond the servo body, less the body clearance."""
    dip_pin_mm: float
    """The DIP pin envelope at ``B'`` beyond the servo body over the PIP samples,
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
    housing = packaging.pin_radius_mm
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


@dataclass(frozen=True)
class Component:
    """A solid of the finger: its collision group and its Y layer."""

    group: str
    layer: str


COLLISION_GROUPS = ("palm", "proximal", "distal", "middle", "coupler")
"""Rigid bodies: parts of one group never collide with each other. Parts of
different groups must not collide where their Y layers overlap."""
COMPONENTS = MappingProxyType({
    "mcp_servo": Component("palm", "central"),
    "proximal_pad": Component("proximal", "central"),
    "pip_servo": Component("proximal", "central"),
    "a_pin": Component("proximal", "through"),
    "ap_pin": Component("proximal", "through"),
    "distal_pad": Component("distal", "central"),
    "bp_pin": Component("distal", "through"),
    "b_pin": Component("distal", "through"),
    "middle_link": Component("middle", "outer"),
    "coupler": Component("coupler", "outer"),
})
"""Each solid's collision group and where it sits across the finger. The PIP
servo is bolted to the proximal body. A pin joins two bodies and sits in the
group of the one that carries it in the central layer, so it is the joint
interface of both: the middle link and coupler, its other bodies, are only in
the outer layer."""
PADS = ("proximal_pad", "distal_pad")
FIXED_MARGIN_PAIRS = (("mcp_servo", "proximal_pad"),)
"""Checked pairs whose margin is set by definition, not by the design: the
proximal pad starts the body clearance (plus its offset) past the MCP servo's
shaft disk. Still checked; left out of the reported least margin. (With the
PIP servo on ``A`` the pad starts at that servo instead: see
:func:`fixed_margin_pairs`.)"""


def fixed_margin_pairs(packaging: "Packaging") -> tuple[tuple[str, str], ...]:
    return () if packaging.pip_servo_on_a else FIXED_MARGIN_PAIRS
SERVO_PAIRS = (("mcp_servo", "pip_servo"), ("a_pin", "pip_servo"), ("bp_pin", "pip_servo"))
"""The servo checks besides the pads': the two servo bodies, and the A and B'
pin envelopes against the PIP servo (the middle link's is reported only)."""


def checked_pairs(packaging: Packaging) -> tuple[tuple[str, str], ...]:
    """The pad pairs that can collide: no shared body, overlapping Y; the MCP
    servo only with ``pad_mcp_check``."""

    pairs = []
    for pad in PADS:
        for other, component in COMPONENTS.items():
            pair = tuple(sorted((pad, other)))
            if other == pad or pair in pairs or (other == "mcp_servo" and not packaging.pad_mcp_check):
                continue
            if COMPONENTS[pad].group == component.group:
                continue  # one rigid body
            if _overlap(packaging.layer_y_mm(COMPONENTS[pad].layer),
                        packaging.layer_y_mm(component.layer)):
                pairs.append(pair)
    return tuple(pairs)


def layer_separated_pairs(packaging: Packaging) -> tuple[tuple[str, str], ...]:
    """Pad pairs in different bodies that may cross in the side view but not in Y."""

    return tuple(
        tuple(sorted((pad, other)))
        for pad in PADS for other, component in COMPONENTS.items()
        if COMPONENTS[pad].group != component.group
        and not _overlap(packaging.layer_y_mm(COMPONENTS[pad].layer),
                         packaging.layer_y_mm(component.layer))
    )


def _overlap(first: tuple[float, float], second: tuple[float, float]) -> bool:
    return max(first[0], second[0]) < min(first[1], second[1])


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


def pad_spans(parameters: FingerLinkageParameters, packaging: Packaging) -> dict[str, PadSpan]:
    """Where each pad sits: the one place pad lengths are defined.

    The proximal pad runs right from :attr:`Packaging.proximal_pad_start_mm`,
    the distal pad left from its offset short of ``C``; each for its
    ``length_mm``, or to ``A''`` / ``B''`` when that is ``None`` (cut to
    ``pad_link_ratio_max`` times its link). With the PIP
    servo on ``A`` the proximal pad lies on the servo body's palmar face, from
    its rear end (plus the pad's offset) to its front.
    """

    proximal, distal = packaging.proximal_pad, packaging.distal_pad
    ratio = packaging.pad_link_ratio_max
    proximal_limit = math.inf if ratio is None else ratio * math.hypot(parameters.a_x_mm, parameters.a_z_mm)
    distal_limit = math.inf if ratio is None else ratio * parameters.bc_length_mm
    if packaging.pip_servo_on_a:
        body = packaging.pip_servo
        outer = parameters.a_x_mm - body.rear_extent_mm + proximal.offset_mm
        room = parameters.a_x_mm + body.radius_mm - outer
    else:
        outer = packaging.proximal_pad_start_mm
        room = parameters.ap.x_mm - outer
    length = proximal.length_mm or min(room, proximal_limit)  # a free pad is cut to the limit
    proximal_span = PadSpan(outer, outer + length, room, proximal_limit)
    outer = parameters.bc_length_mm - distal.offset_mm
    room = outer - (parameters.bp.x_mm - parameters.b_x_mm)
    length = distal.length_mm or min(room, distal_limit)
    distal_span = PadSpan(outer - length, outer, room, distal_limit)
    return {"proximal_pad": proximal_span, "distal_pad": distal_span}


def pad_outline(span: PadSpan, pad: ContactPad, chamfer_at: str) -> np.ndarray:
    """A pad in its mounting-line frame (``u`` along the line, ``v`` dorsal), ``(m, 2)``.

    ``a-b`` is the span on the line and ``c-d`` the pad's height palmar of it;
    the palmar corner at the ``chamfer_at`` end (``"start"`` or ``"end"``)
    loses the pad's 45 deg chamfer. Counter-clockwise, convex.
    """

    start_mm, end_mm = span.start_mm, span.end_mm
    t, k = pad.height_mm, min(pad.chamfer_mm, end_mm - start_mm)
    a, b, d, c = (start_mm, 0.0), (end_mm, 0.0), (end_mm, -t), (start_mm, -t)
    if k <= 0.0:
        return np.array((c, d, b, a))
    if chamfer_at == "end":
        return np.array((c, (end_mm - k, -t), (end_mm, -t + k), b, a))
    return np.array(((start_mm + k, -t), d, b, a, (start_mm, -t + k)))


def proximal_pad_polygon(linkage: FingerLinkage, packaging: Packaging) -> np.ndarray:
    """The proximal pad in the proximal-link frame, ``(m, 2)``: its span on
    ``O-A''``, chamfered at the end facing the PIP."""

    span = pad_spans(linkage.parameters, packaging)["proximal_pad"]
    outline = pad_outline(span, packaging.proximal_pad, "end")
    return outline + (0.0, packaging.proximal_pad_line_z(linkage.parameters))


def distal_pad_polygon(linkage: FingerLinkage, packaging: Packaging,
                       b: np.ndarray, c: np.ndarray) -> np.ndarray:
    """The distal pad at posed ``B`` and ``C`` (``(n, 2)``), ``(n, m, 2)``: its
    span on ``B''-C``, chamfered at the end facing the PIP.

    Fixed in the distal link: a point ``u`` along and ``v`` dorsal of ``BC``
    from ``B`` in the reference pose is ``B + u e + v n`` at every pose, ``e``
    the unit ``B->C`` and ``n`` it turned toward ``+Z``.
    """

    p = linkage.parameters
    local = pad_outline(pad_spans(p, packaging)["distal_pad"], packaging.distal_pad, "start")
    b, c = np.atleast_2d(b), np.atleast_2d(c)
    e = (c - b) / p.bc_length_mm
    n = np.stack((-e[:, 1], e[:, 0]), axis=1)
    return b[:, None] + local[None, :, :1] * e[:, None] + local[None, :, 1:] * n[:, None]


def usable_pad_lengths(
    linkage: FingerLinkage, packaging: Packaging, b: np.ndarray, bp: np.ndarray, c: np.ndarray,
) -> dict[str, np.ndarray]:
    """Per-sample length of each pad's face the links leave free, from its outer end.

    The middle link ``A-B'`` and the coupler ``A'-B`` sit in other layers, but
    where one passes through a pad's block in the side view (over its span,
    between the mounting line and the pad's height), it runs across the
    contact interface. (Passing further palmar, in front of the face, is the
    curling finger itself.) The free length runs from
    the pad's outer end (``a`` of the proximal pad, the ``C`` end of the distal
    one) to the first such point. Each link loses its pin envelope at both
    ends: ``A`` and ``B`` sit on the mounting lines as the links' own pivots.
    Samples are four-bar poses in the proximal-link frame (``(n, 2)``).
    """

    p = linkage.parameters
    b, bp, c = np.atleast_2d(b), np.atleast_2d(bp), np.atleast_2d(c)
    count = len(b)
    a = np.broadcast_to((p.a_x_mm, p.a_z_mm), (count, 2))
    ap = np.broadcast_to((p.ap.x_mm, p.ap.z_mm), (count, 2))
    links = [_trimmed(a, bp, packaging.pin_radius_mm), _trimmed(ap, b, packaging.pin_radius_mm)]
    spans = pad_spans(p, packaging)

    proximal = spans["proximal_pad"]  # frame: x along the mounting line, z dorsal of it
    line = np.array((0.0, packaging.proximal_pad_line_z(p)))
    near = np.full(count, np.inf)
    for start, end in links:
        low, _ = _block_extent(start - line, end - line, proximal.start_mm, proximal.end_mm,
                               packaging.proximal_pad.height_mm)
        near = np.fmin(near, low)
    proximal_free = np.where(np.isfinite(near), near - proximal.start_mm, proximal.length_mm)

    distal = spans["distal_pad"]  # frame: u along B->C from B, v dorsal
    e = (c - b) / p.bc_length_mm
    n = np.stack((-e[:, 1], e[:, 0]), axis=1)

    def local(points: np.ndarray) -> np.ndarray:
        rel = points - b
        return np.stack((np.sum(rel * e, axis=1), np.sum(rel * n, axis=1)), axis=1)

    far = np.full(count, -np.inf)
    for start, end in links:
        _, high = _block_extent(local(start), local(end), distal.start_mm, distal.end_mm,
                                packaging.distal_pad.height_mm)
        far = np.fmax(far, high)
    distal_free = np.where(np.isfinite(far), distal.end_mm - far, distal.length_mm)
    return {"proximal_pad": np.maximum(proximal_free, 0.0), "distal_pad": np.maximum(distal_free, 0.0)}


def _trimmed(start: np.ndarray, end: np.ndarray, radius: float) -> tuple[np.ndarray, np.ndarray]:
    """Segments ``start-end`` shortened by ``radius`` at both ends (to a point at most)."""

    direction = end - start
    length = np.linalg.norm(direction, axis=1)
    trim = np.minimum(radius / np.where(length > 0.0, length, 1.0), 0.5)[:, None]
    return start + trim * direction, end - trim * direction


def _block_extent(start: np.ndarray, end: np.ndarray, low: float, high: float,
                  height: float) -> tuple[np.ndarray, np.ndarray]:
    """Least and most ``u`` of each segment's part inside a pad block,
    ``low < u < high`` and ``-height < v < 0``.

    Points are ``(u, v)`` in a pad's line frame, ``(n, 2)``; ``(inf, -inf)``
    where the segment misses the block.
    """

    t_low, t_high = np.zeros(len(start)), np.ones(len(start))
    for axis, (lo, hi) in ((0, (low, high)), (1, (-height, 0.0))):
        a, d = start[:, axis], end[:, axis] - start[:, axis]
        with np.errstate(divide="ignore", invalid="ignore"):
            enter, leave = (lo - a) / d, (hi - a) / d
        first, second = np.minimum(enter, leave), np.maximum(enter, leave)
        flat = d == 0.0  # parallel: inside the slab all along, or never
        inside = (a > lo) & (a < hi)
        t_low = np.where(flat, np.where(inside, t_low, np.inf), np.maximum(t_low, first))
        t_high = np.where(flat, np.where(inside, t_high, -np.inf), np.minimum(t_high, second))
    valid = t_low < t_high
    u0, du = start[:, 0], end[:, 0] - start[:, 0]
    ua, ub = u0 + np.where(valid, t_low, 0.0) * du, u0 + np.where(valid, t_high, 0.0) * du
    return np.where(valid, np.minimum(ua, ub), np.inf), np.where(valid, np.maximum(ua, ub), -np.inf)


def pad_margins(
    linkage: FingerLinkage,
    packaging: Packaging,
    b: np.ndarray,
    c: np.ndarray,
    bp: np.ndarray,
    mcp_samples_rad: Sequence[float] = (0.0,),
) -> dict[tuple[str, str], np.ndarray]:
    """Per-sample margin of every :func:`checked_pairs` pair beyond the body clearance.

    Samples are four-bar poses in the proximal-link frame, given by the posed
    ``B``, ``C`` and ``B'`` (``(n, 2)``). The MCP servo turns by each MCP
    flexion sample about ``O``; its margin is the worst over them. Margins
    are in mm, clear when >= 0.
    """

    b, c, bp = np.atleast_2d(b), np.atleast_2d(c), np.atleast_2d(bp)
    count = len(b)
    proximal = proximal_pad_polygon(linkage, packaging)
    pads = {"proximal_pad": np.broadcast_to(proximal, (count, *proximal.shape)),
            "distal_pad": distal_pad_polygon(linkage, packaging, b, c)}
    pins = {"a_pin": np.broadcast_to((linkage.a.x_mm, linkage.a.z_mm), (count, 2)),
            "ap_pin": np.broadcast_to((linkage.ap.x_mm, linkage.ap.z_mm), (count, 2)),
            "bp_pin": bp, "b_pin": b}
    footprint = packaging.pip_footprint(linkage)
    radius = packaging.pin_radius_mm

    boxes = {name: (polygon.min(axis=1), polygon.max(axis=1)) for name, polygon in pads.items()}
    boxes.update({name: (center - radius, center + radius) for name, center in pins.items()})
    outline = footprint.outline
    boxes["pip_servo"] = (np.broadcast_to(outline.min(axis=0), (count, 2)),
                          np.broadcast_to(outline.max(axis=0), (count, 2)))

    def gap(pad: str, other: str) -> np.ndarray:
        polygon = pads[pad]
        if other == "mcp_servo":
            if pad == "proximal_pad":  # fixed in the proximal frame: the same at every sample
                return np.full(count, _mcp_gap(polygon[:1], packaging, mcp_samples_rad)[0])
            return _mcp_gap(polygon, packaging, mcp_samples_rad)
        # Broad phase: the bounding boxes' gap bounds the gap from below.
        (low, high), (other_low, other_high) = boxes[pad], boxes[other]
        result = np.hypot(*np.maximum(np.maximum(other_low - high, low - other_high), 0.0).T)
        near = np.flatnonzero(result <= packaging.body_clearance_mm + EXACT_BAND_MM)
        if near.size:
            if other in pads:
                result[near] = _polygons_gap(polygon[near], pads[other][near])
            elif other in pins:
                result[near] = _polygon_to_disk(polygon[near], pins[other][near], radius)
            elif other == "pip_servo":
                result[near] = _polygon_to_footprint(polygon[near], footprint)
            else:
                raise ValueError(f"no geometry for the pair {pad!r}, {other!r}")
        return result

    margins = {}
    for pair in checked_pairs(packaging):
        pad, other = pair if pair[0] in PADS else pair[::-1]
        margin = gap(pad, other) - packaging.body_clearance_mm
        # A pad set exactly the clearance away (the MCP offset) must not fail by rounding.
        margins[pair] = np.where(np.abs(margin) < _MARGIN_TOLERANCE_MM, 0.0, margin)
    return margins


def _mcp_gap(polygons: np.ndarray, packaging: Packaging, mcp_samples_rad: Sequence[float]) -> np.ndarray:
    """Least gap of ``(n, m, 2)`` polygons to the MCP servo over the MCP flexion samples.

    MCP flexion turns the servo about ``O`` in the proximal frame, which keeps
    every point's distance to ``O``: an edge ``d`` from ``O`` is at least
    ``d - reach`` from the servo at every MCP angle. Edges within
    :data:`EXACT_BAND_MM` of clearing it by that bound are checked on a coarse
    MCP grid first, bounded by how far the servo moves between its angles, and
    at every sample only if that bound does not clear them either; the rest
    report their bound.
    """

    servo = packaging.mcp_footprint()
    count = len(polygons)
    start, end = (array.reshape(-1, 2) for array in _edges(polygons))
    from_o = point_segment_distance(np.zeros(2), start, end)
    gap = from_o - servo.reach_mm
    exact = packaging.body_clearance_mm + EXACT_BAND_MM

    def at(edges: np.ndarray, angles: Sequence[float]) -> np.ndarray:
        # Turning the MCP servo by +q is turning the pads by -q about O.
        cosine, sine = np.cos(angles)[:, None], -np.sin(angles)[:, None]

        def turned(points: np.ndarray) -> np.ndarray:
            x, z = points[:, 0], points[:, 1]
            return np.stack((cosine * x - sine * z, sine * x + cosine * z), axis=-1).reshape(-1, 2)

        starts, ends = turned(start[edges]), turned(end[edges])
        return servo.distance_to_segments(starts, ends).reshape(len(angles), edges.size).min(axis=0)

    near = np.flatnonzero(gap <= exact)
    samples = np.asarray(mcp_samples_rad, dtype=float)
    if near.size:
        # Coarse pass: every angle is within half a coarse step of one, which
        # moves a servo point by at most reach * step / 2.
        stride = max(1, int(round(_MCP_COARSE_STEP_RAD / max(np.diff(samples).min(initial=math.inf), 1e-12))))
        coarse = np.unique(np.append(samples[::stride], samples[-1]))
        slack = servo.reach_mm * stride * (np.diff(samples).max(initial=0.0)) / 2.0
        gap[near] = at(near, coarse) - slack
        fine = near[gap[near] <= exact]
        if fine.size:
            gap[fine] = at(fine, samples)
    return gap.reshape(count, -1).min(axis=1)


def _edges(polygons: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return polygons, np.roll(polygons, -1, axis=-2)


def _inside(points: np.ndarray, polygons: np.ndarray) -> np.ndarray:
    """Whether each of ``(n, p, 2)`` points lies inside (or on) the convex ``(n, m, 2)`` polygon."""

    start, end = _edges(polygons)
    edge = (end - start)[:, None]                  # (n, 1, m, 2)
    offset = points[:, :, None] - start[:, None]   # (n, p, m, 2)
    cross = edge[..., 0] * offset[..., 1] - edge[..., 1] * offset[..., 0]
    return np.all(cross >= 0.0, axis=2) | np.all(cross <= 0.0, axis=2)


def _polygons_gap(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """Gap between two ``(n, m, 2)`` convex polygons per sample (0 when they touch or overlap)."""

    count = len(first)
    (p0, p1), (q0, q1) = _edges(first), _edges(second)
    p0, p1 = (array[:, :, None] for array in (p0, p1))
    q0, q1 = (array[:, None] for array in (q0, q1))
    shape = np.broadcast_shapes(p0.shape, q0.shape)
    p0, p1, q0, q1 = (np.broadcast_to(array, shape).reshape(-1, 2) for array in (p0, p1, q0, q1))
    distance = np.minimum.reduce((
        point_segment_distance(p0, q0, q1), point_segment_distance(p1, q0, q1),
        point_segment_distance(q0, p0, p1), point_segment_distance(q1, p0, p1),
    ))
    distance = np.where(_segments_cross(p0, p1, q0, q1), 0.0, distance).reshape(count, -1).min(axis=1)
    overlap = _inside(first, second).any(axis=1) | _inside(second, first).any(axis=1)
    return np.where(overlap, 0.0, distance)


def _polygon_to_disk(polygons: np.ndarray, centers: np.ndarray, radius: float) -> np.ndarray:
    """Gap from ``(n, m, 2)`` polygons to disks at ``(n, 2)`` centres (negative: overlap)."""

    count, sides = polygons.shape[:2]
    start, end = _edges(polygons)
    distance = point_segment_distance(np.repeat(centers, sides, axis=0),
                                      start.reshape(-1, 2), end.reshape(-1, 2))
    distance = distance.reshape(count, sides).min(axis=1)
    return np.where(_inside(centers[:, None], polygons)[:, 0], 0.0, distance) - radius


def _polygon_to_footprint(polygons: np.ndarray, footprint: "ServoFootprint") -> np.ndarray:
    """Gap from ``(n, m, 2)`` polygons to a servo footprint (0 when they touch or overlap)."""

    count = len(polygons)
    start, end = _edges(polygons)
    distance = footprint.distance_to_segments(start.reshape(-1, 2), end.reshape(-1, 2))
    distance = distance.reshape(count, -1).min(axis=1)
    pivot = np.broadcast_to(footprint.pivot, (count, 1, 2))
    return np.where(_inside(pivot, polygons)[:, 0], 0.0, distance)


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
    # Collinear segments only touch if an endpoint lies on the other, which
    # the endpoint distances already find.
    collinear = (d1 == 0.0) & (d2 == 0.0)
    return (d1 * d2 <= 0.0) & (d3 * d4 <= 0.0) & ~collinear


__all__ = [
    "BODY_CLEARANCE_MM",
    "COLLISION_GROUPS",
    "COMPONENTS",
    "Component",
    "ContactPad",
    "DEFAULT_PIP_SAMPLES_RAD",
    "EXACT_BAND_MM",
    "FIXED_MARGIN_PAIRS",
    "fixed_margin_pairs",
    "checked_pairs",
    "distal_pad_polygon",
    "layer_separated_pairs",
    "pad_outline",
    "pad_spans",
    "usable_pad_lengths",
    "proximal_pad_polygon",
    "pad_margins",
    "PIN_ENVELOPE_DIAMETER_MM",
    "PIN_RADIUS_MM",
    "PIN_SPACING_MM",
    "PadSpan",
    "SERVO_PAIRS",
    "Packaging",
    "ServoClearance",
    "ServoFootprint",
    "point_segment_distance",
    "turn_points",
    "servo_clearance",
]
