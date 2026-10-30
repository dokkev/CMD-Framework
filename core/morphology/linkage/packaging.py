"""Planar packaging of the finger: servos, pin housings and their clearances.

The PIP servo rides on the proximal link with its shaft (along ``Y``) on
``A'`` and drives the coupler ``A'-B``; sitting dorsal of ``OA``, it leaves
the proximal contact interface free. Its rear body lies back along the link,
along ``-X`` tilted dorsally by the design's ``pip_servo_tilt_rad``. The PIP
pivot ``A`` hangs off the servo body on a bracket, in the same plane: its
bearing housing must clear a virtual clearance disk around the servo's shaft
(``ap_clearance_diameter_mm``), not the body. The MCP servo's shaft is on ``O``.

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
driver link and coupler plates are outside it and the pins run through every
layer. Only components in different collision groups (rigid bodies,
:data:`COLLISION_GROUPS`) whose Y intervals overlap are checked
(:func:`checked_pairs`).
"""

from __future__ import annotations

import functools
import math
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Sequence

import numpy as np

from core.hardware import DShapedExtrusion, XM335Actuator
from core.morphology.linkage import _geometry as _kernels
from core.morphology.linkage import FingerLinkage, FingerLinkageParameters


PIVOT_CLEARANCE_DIAMETER_MM = 8.0
"""Diameter of the metal around each pivot pin (``A``, ``A'``, ``B``, ``B'``) in
the side view, which other parts keep the body clearance from.
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
PIVOTS = ("a", "ap", "b", "bp")
"""The pivots by name: ``A``, ``A'``, ``B``, ``B'``."""


def _xm335() -> DShapedExtrusion:
    return XM335Actuator().geometry


@dataclass(frozen=True)
class ContactPad:
    """One contact pad block (provisional values).

    ``height_mm`` palmar of its mounting line, starting ``offset_mm`` in from
    its outer end (``O`` or the MCP servo, or ``C``), and a 45 deg chamfer of
    ``chamfer_mm`` at its palmar corner facing the PIP (with ``chamfer_free``
    the design's own, 0 to ``height_mm``: :func:`pad_chamfer_mm`). ``length_mm`` fixes
    its length, which the mounting line must have room for; ``None`` runs it
    to ``A''`` or ``B''``, the whole room (see :func:`pad_spans`), cut to
    ``max_length_mm`` when given (no module needs more).
    """

    height_mm: float = 5.0
    offset_mm: float = 0.0
    chamfer_mm: float = 2.5
    length_mm: float | None = None
    max_length_mm: float | None = None
    """Most mounting length: a free pad is cut to it, a fixed one may not exceed it."""
    chamfer_free: bool = False
    """The chamfer is a design variable (``FingerLinkageParameters.proximal_chamfer_mm``
    or ``distal_chamfer_mm``), at most ``height_mm``; ``chamfer_mm`` is then unused."""

    def __post_init__(self) -> None:
        if not self.height_mm > 0.0:
            raise ValueError("height_mm must be positive")
        if self.length_mm is not None and not self.length_mm > 0.0:
            raise ValueError("length_mm must be positive, or None")
        if not self.offset_mm >= 0.0:
            raise ValueError("offset_mm must be non-negative")
        if not 0.0 <= self.chamfer_mm <= self.height_mm:
            raise ValueError("chamfer_mm must be within [0, height_mm]")
        if self.max_length_mm is not None and not self.max_length_mm > 0.0:
            raise ValueError("max_length_mm must be positive, or None")


@dataclass(frozen=True)
class Packaging:
    """Hardware dimensions of the finger's planar packaging.

    ``mcp_servo_rear`` is the unit direction of the MCP servo's rear body in
    the linkage frame at zero MCP flexion (``+Z``: standing dorsally).
    """

    pip_servo: DShapedExtrusion = field(default_factory=_xm335)
    mcp_servo: DShapedExtrusion = field(default_factory=_xm335)
    mcp_servo_rear: tuple[float, float] = (0.0, 1.0)
    pivot_clearance_diameter_mm: float = PIVOT_CLEARANCE_DIAMETER_MM
    """Diameter of each pivot pin's housing (:data:`PIVOT_CLEARANCE_DIAMETER_MM`); the
    body clearance is kept beyond it."""
    body_clearance_mm: float = BODY_CLEARANCE_MM
    proximal_pad: ContactPad = field(default_factory=ContactPad)
    distal_pad: ContactPad = field(default_factory=ContactPad)
    allow_collision_pip_servo_to_joint_a: bool = False
    """``False``: the pin envelope of the joint off the PIP servo (``A``; ``A'``
    with the servo on ``A``) keeps the body clearance from the virtual
    clearance disk around the servo's shaft (``ap_clearance_diameter_mm``)."""
    allow_collision_pip_servo_to_joint_bp: bool = True
    """``True``: the ``B'`` pin envelope may pass through the PIP servo body
    (the driver link sits in another Y layer). ``False``: it keeps the body
    clearance from the body over the motion, which ends the safe range."""
    ap_clearance_diameter_mm: float = 7.0
    """Virtual clearance around the PIP servo's shaft (``A'``; ``A`` with the
    servo on ``A``), what the other pivot's pin envelope keeps clear of. The
    servo body itself is not checked against that pin: it rides on a bracket
    off the body."""
    pad_mcp_check: bool = True
    pad_mcp_clearance_mm: float | None = None
    """Least gap of the pads to the MCP servo, which also places the proximal
    pad's start; ``None``: ``body_clearance_mm``."""
    pad_link_ratio_max: float | None = None
    """Most pad length per length of its link (the proximal pad over ``|OA|``,
    the distal pad over ``BC``): a pad may stand out of its link by this much.
    A free-length pad is cut to it; a fixed one must not exceed it. ``None``:
    unbounded."""
    pip_servo_at: str = "ap"
    """Where the PIP servo's shaft is: ``"ap"`` (``A'``, driving ``A'-B``) or
    ``"a"`` (the PIP ``A``, driving the driver link ``A-B'``; body along ``-X``,
    no tilt, the proximal pad on its palmar face)."""
    a_pivot_diameter_mm: float | None = None
    ap_pivot_diameter_mm: float | None = None
    b_pivot_diameter_mm: float | None = None
    bp_pivot_diameter_mm: float | None = None
    """Each pivot's own size in the side view (:meth:`pivot_radius_mm`);
    ``None``: ``pivot_clearance_diameter_mm``. The ``A``-``A'`` check still
    uses ``ap_clearance_diameter_mm`` at the servo's shaft."""
    proximal_pad_below_a: bool = False
    distal_pad_below_b: bool = False
    """Hang the pad half its own link's pivot (``A``, ``B``) below the link
    line, so its top face clears that pivot (:attr:`proximal_pad_drop_mm`);
    ``False``: on the line, over the pivot."""
    pip_servo_above_oa: bool = True
    """The PIP servo stays dorsal of the ``OA`` line (the ``oa_interface_mm``
    constraint)."""
    pip_servo_pad_check: bool = False
    """The PIP servo body keeps clear of the proximal pad (one body, still
    kept apart): it may come down to the pad's top face."""
    a_pin_servo_check: bool = False
    """The ``A`` pin also keeps clear of the PIP servo body (it hangs off the
    servo on a bracket otherwise)."""
    pad_pad_check: bool = True
    """The two pads keep clear of each other."""
    links_clear_pads: bool = False
    """The links (``A-B'``, ``A'-B``) bend around the pads, out of their way
    (as the serial 2R's truss of links): a pad is usable up to where the two
    contact faces cross in the fold instead (``face_crossing_lengths``).
    Always so in the goal/solver study (2026-10-27); ``False`` (runs before
    it): up to the first link crossing it (``usable_pad_lengths``)."""
    pads_off_pins: bool = False
    """Each pad keeps clear of its own body's pin housings (the proximal pad
    of ``A`` and ``A'``, the distal pad of ``B`` and ``B'``) and may end
    anywhere: short of its pins or past them. Always so in the goal/solver
    study (2026-10-27); ``False`` (runs before it): the pads run from ``A``
    and ``B`` outward (``A''_x >= A_x``, ``B''_x <= B_x``), their own pins
    unchecked."""
    distal_pad_mcp_check: bool | None = None
    """The distal pad keeps clear of the MCP servo; ``None``: as ``pad_mcp_check``
    (which then governs the proximal pad alone)."""
    distal_carrier_height_mm: float = 0.0
    """The distal pad's carrier plate: over the pad's span, this thick dorsal
    of its top face, kept clear of the PIP servo body. The rest of the distal
    link is not checked against the servo (a bracket in other Y layers). 0: no
    carrier checked."""
    overlap_depth: bool = False
    """Report an overlap of two solids by its depth (negative), not as a zero
    gap, so a zero clearance still rejects it. Off: overlapping solids count as
    touching, which only a positive clearance rejects."""

    def __post_init__(self) -> None:
        for pivot in PIVOTS:
            diameter = getattr(self, f"{pivot}_pivot_diameter_mm")
            if diameter is not None and not diameter >= 0.0:
                raise ValueError(f"{pivot}_pivot_diameter_mm must be non-negative, or None")
        if self.pad_mcp_clearance_mm is not None and not self.pad_mcp_clearance_mm >= 0.0:
            raise ValueError("pad_mcp_clearance_mm must be non-negative, or None")
        if self.pad_link_ratio_max is not None and not self.pad_link_ratio_max > 0.0:
            raise ValueError("pad_link_ratio_max must be positive, or None")
        if not self.distal_carrier_height_mm >= 0.0:
            raise ValueError("distal_carrier_height_mm must be non-negative")
        if not self.ap_clearance_diameter_mm >= 0.0:
            raise ValueError("ap_clearance_diameter_mm must be non-negative")
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

        return self.pivot_clearance_diameter_mm / 2.0

    @property
    def pin_spacing_mm(self) -> float:
        """Least centre distance of two pins on one rigid body: their envelopes touch."""

        return 2.0 * self.pin_radius_mm

    def pivot_radius_mm(self, pivot: str) -> float:
        """Radius of one pivot (``"a"``, ``"ap"``, ``"b"``, ``"bp"``) in the side view."""

        diameter = getattr(self, f"{pivot}_pivot_diameter_mm")
        return (self.pivot_clearance_diameter_mm if diameter is None else diameter) / 2.0

    @property
    def proximal_pad_drop_mm(self) -> float:
        """How far the proximal pad's top face lies palmar of ``O-A``: ``A``'s
        radius with ``proximal_pad_below_a``, else 0."""

        return self.pivot_radius_mm("a") if self.proximal_pad_below_a and not self.pip_servo_on_a else 0.0

    @property
    def distal_pad_drop_mm(self) -> float:
        """How far the distal pad's top face lies palmar of ``BC``: ``B``'s radius
        with ``distal_pad_below_b``, else 0."""

        return self.pivot_radius_mm("b") if self.distal_pad_below_b else 0.0

    def proximal_drop(self, parameters: FingerLinkageParameters) -> float:
        """The proximal pad's drop below ``O-A`` in a design: its own, or the packaging's."""

        own = parameters.proximal_drop_mm
        return self.proximal_pad_drop_mm if own is None or self.pip_servo_on_a else own

    def distal_drop(self, parameters: FingerLinkageParameters) -> float:
        """The distal pad's drop below ``B-C`` in a design: its own, or the packaging's."""

        own = parameters.distal_drop_mm
        return self.distal_pad_drop_mm if own is None else own

    @property
    def proximal_pad_start_mm(self) -> float:
        """``x`` of the proximal pad's first edge ``a-c``: ``O``, or the MCP servo's
        shaft disk plus the body clearance with ``pad_mcp_check``, plus the pad's
        offset."""

        clear = self.mcp_servo.radius_mm + self.mcp_pad_clearance_mm if self.pad_mcp_check else 0.0
        return clear + self.proximal_pad.offset_mm

    def pivot_pin_gap_mm(self, linkage: FingerLinkage) -> float:
        """Gap between the pin envelope of the pivot off the PIP servo (``A``, or
        ``A'`` with the servo on ``A``) and the virtual clearance disk around the
        servo's shaft: ``|AA'|`` less both radii."""

        return (math.hypot(linkage.a.x_mm - linkage.ap.x_mm, linkage.a.z_mm - linkage.ap.z_mm)
                - self.pivot_radius_mm("ap" if self.pip_servo_on_a else "a")
                - self.ap_clearance_diameter_mm / 2.0)

    @property
    def mcp_pad_clearance_mm(self) -> float:
        """The pads' least gap to the MCP servo (``pad_mcp_clearance_mm``)."""

        return self.body_clearance_mm if self.pad_mcp_clearance_mm is None else self.pad_mcp_clearance_mm

    def pad_checks_mcp(self, pad: str) -> bool:
        """Whether a pad (``"proximal_pad"``, ``"distal_pad"``) is checked against the MCP servo."""

        if pad == "distal_pad" and self.distal_pad_mcp_check is not None:
            return self.distal_pad_mcp_check
        return self.pad_mcp_check

    @property
    def pip_servo_on_a(self) -> bool:
        return self.pip_servo_at == "a"

    def proximal_pad_line_z(self, parameters: FingerLinkageParameters) -> float:
        """``z`` of the proximal pad's mounting line: ``O-A''`` (less
        :attr:`proximal_pad_drop_mm`), or the palmar face of the PIP servo on ``A``."""

        return parameters.a_z_mm - self.pip_servo.radius_mm if self.pip_servo_on_a else -self.proximal_drop(parameters)

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
        gap = min(to_mcp, float(pip_servo.distance_to_segments(start, end).min()))
        if self.overlap_depth and gap <= _MARGIN_TOLERANCE_MM:
            depth = float(_separation(np.stack(pip_edges), np.broadcast_to(
                mcp_outline, (len(pip_edges), *mcp_outline.shape))).min())
            gap = min(gap, depth) if depth < 0.0 else gap
        return gap


PIN_RADIUS_MM = Packaging().pin_radius_mm
PIN_SPACING_MM = Packaging().pin_spacing_mm


@dataclass(frozen=True)
class ServoClearance:
    """Signed clearance margins of one linkage design (mm, >= 0 is feasible)."""

    mcp_servo_mm: float
    """PIP servo beyond the MCP servo body over the MCP angles, less the body clearance."""
    pin_mm: float
    """The pin envelope of the pivot off the PIP servo beyond the virtual
    clearance at its shaft (:meth:`Packaging.pivot_pin_gap_mm`), less the body clearance."""
    dip_pin_mm: float
    """The DIP pin envelope at ``B'`` beyond the servo body over the PIP samples,
    less the body clearance (inf with ``allow_collision_pip_servo_to_joint_bp``)."""
    link_mm: float
    """The driver link ``A-B'`` beyond the servo over the PIP samples. Reported
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
    housing = packaging.pivot_radius_mm("bp")
    link = footprint.distance_to_segments(np.broadcast_to(a, sweep.bp.shape), sweep.bp)
    dip = footprint.distance_to_segments(sweep.bp, sweep.bp)
    return ServoClearance(
        mcp_servo_mm=packaging.mcp_servo_gap_mm(footprint, mcp_samples_rad) - clearance,
        pin_mm=packaging.pivot_pin_gap_mm(linkage) - clearance,
        dip_pin_mm=(math.inf if packaging.allow_collision_pip_servo_to_joint_bp or not dip.size
                    else float(dip.min()) - housing - clearance),
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

        forward, side = self._axes()
        return _kernels.footprint_distance(_points(start), _points(end), float(self.pivot[0]), float(self.pivot[1]),
                                           float(forward[0]), float(forward[1]), float(side[0]), float(side[1]),
                                           float(self.radius_mm), float(self.rear_extent_mm))

    def _axes(self) -> tuple[np.ndarray, np.ndarray]:
        """Unit vectors of the servo frame: ``forward`` (away from the rear body) and ``side``."""

        forward = -np.array(self.rear)
        return forward, np.array((-forward[1], forward[0]))


@dataclass(frozen=True)
class Component:
    """A solid of the finger: its collision group and its Y layer."""

    group: str
    layer: str


COLLISION_GROUPS = ("palm", "proximal", "distal", "driver", "coupler")
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
    "driver_link": Component("driver", "outer"),
    "coupler": Component("coupler", "outer"),
})
"""Each solid's collision group and where it sits across the finger. The PIP
servo is bolted to the proximal body. A pin joins two bodies and sits in the
group of the one that carries it in the central layer, so it is the joint
interface of both: the driver link and coupler, its other bodies, are only in
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
"""The servo checks besides the pads': the two servo bodies, the A pin
envelope against the virtual clearance at the PIP servo's shaft, and (unless
``allow_collision_pip_servo_to_joint_bp``) the B' pin envelope against the PIP
servo body (the driver link's is reported only)."""


def checked_pairs(packaging: Packaging) -> tuple[tuple[str, str], ...]:
    """The pad pairs that can collide: no shared body, overlapping Y; the MCP
    servo only with ``pad_mcp_check``."""

    pairs = []
    for pad in PADS:
        for other, component in COMPONENTS.items():
            pair = tuple(sorted((pad, other)))
            if other == pad or pair in pairs or (other == "mcp_servo" and not packaging.pad_checks_mcp(pad)):
                continue
            if other in PADS and not packaging.pad_pad_check:
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
    if proximal.max_length_mm is not None:  # no module needs more (morphology goal)
        proximal_limit = min(proximal_limit, proximal.max_length_mm)
    if distal.max_length_mm is not None:
        distal_limit = min(distal_limit, distal.max_length_mm)
    if packaging.pip_servo_on_a:
        body = packaging.pip_servo
        outer = parameters.a_x_mm - body.rear_extent_mm + proximal.offset_mm
        room = parameters.a_x_mm + body.radius_mm - outer
    else:
        outer = packaging.proximal_pad_start_mm
        room = parameters.app_x_mm - outer
    length = proximal.length_mm or min(room, proximal_limit)  # a free pad is cut to the limit
    proximal_span = PadSpan(outer, outer + length, room, proximal_limit)
    outer = parameters.bc_length_mm - distal.offset_mm
    room = outer - (parameters.bpp_x_mm - parameters.b_x_mm)
    length = distal.length_mm or min(room, distal_limit)
    distal_span = PadSpan(outer - length, outer, room, distal_limit)
    return {"proximal_pad": proximal_span, "distal_pad": distal_span}


def pad_chamfer_mm(pad: ContactPad, parameters: FingerLinkageParameters, side: str) -> float:
    """The chamfer a pad has in this design: its own (``chamfer_free``) or the packaging's."""

    if pad.chamfer_free:
        return min(getattr(parameters, f"{side}_chamfer_mm"), pad.height_mm)
    return pad.chamfer_mm


def pad_outline(span: PadSpan, pad: ContactPad, chamfer_at: str, chamfer_mm: float | None = None) -> np.ndarray:
    """A pad in its mounting-line frame (``u`` along the line, ``v`` dorsal), ``(m, 2)``.

    ``a-b`` is the span on the line and ``c-d`` the pad's height palmar of it;
    the palmar corner at the ``chamfer_at`` end (``"start"`` or ``"end"``)
    loses the pad's 45 deg chamfer. Counter-clockwise, convex.
    """

    start_mm, end_mm = span.start_mm, span.end_mm
    chamfer = pad.chamfer_mm if chamfer_mm is None else chamfer_mm
    t, k = pad.height_mm, min(chamfer, end_mm - start_mm)
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
    outline = pad_outline(span, packaging.proximal_pad, "end",
                          pad_chamfer_mm(packaging.proximal_pad, linkage.parameters, "proximal"))
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
    local = pad_outline(pad_spans(p, packaging)["distal_pad"], packaging.distal_pad, "start",
                        pad_chamfer_mm(packaging.distal_pad, p, "distal"))
    local = local - (0.0, packaging.distal_drop(p))
    b, c = np.atleast_2d(b), np.atleast_2d(c)
    e = (c - b) / p.bc_length_mm
    n = np.stack((-e[:, 1], e[:, 0]), axis=1)
    return b[:, None] + local[None, :, :1] * e[:, None] + local[None, :, 1:] * n[:, None]


def carrier_outline(pad_outline_mm: np.ndarray, height_mm: float) -> np.ndarray:
    """The carrier plate of a pad given in its mounting-line frame (top face
    at ``v = 0``): the pad's span, ``height_mm`` dorsal of it, ``(4, 2)``."""

    start, end = float(pad_outline_mm[:, 0].min()), float(pad_outline_mm[:, 0].max())
    return np.array(((start, 0.0), (end, 0.0), (end, height_mm), (start, height_mm)))


def distal_carrier_polygon(linkage: FingerLinkage, packaging: Packaging,
                           b: np.ndarray, c: np.ndarray) -> np.ndarray:
    """The distal pad's carrier plate (``packaging.distal_carrier_height_mm``)
    at posed ``B`` and ``C``, ``(n, 4, 2)``, as :func:`distal_pad_polygon`: over
    the whole mounting line from ``B''`` to the fingertip end (the support,
    there whatever module it carries; a module fills it unless its length is set)."""

    p = linkage.parameters
    span = pad_spans(p, packaging)["distal_pad"]
    support = PadSpan(span.end_mm - span.room_mm, span.end_mm, span.room_mm)
    local = carrier_outline(pad_outline(support, packaging.distal_pad, "start", 0.0),
                            packaging.distal_carrier_height_mm) - (0.0, packaging.distal_drop(p))
    b, c = np.atleast_2d(b), np.atleast_2d(c)
    e = (c - b) / p.bc_length_mm
    n = np.stack((-e[:, 1], e[:, 0]), axis=1)
    return b[:, None] + local[None, :, :1] * e[:, None] + local[None, :, 1:] * n[:, None]


def usable_pad_lengths(
    linkage: FingerLinkage, packaging: Packaging, b: np.ndarray, bp: np.ndarray, c: np.ndarray,
) -> dict[str, np.ndarray]:
    """Per-sample length of each pad's face the links leave free, from its outer end.

    The driver link ``A-B'`` and the coupler ``A'-B`` sit in other layers, but
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
    radius = packaging.pivot_radius_mm
    links = [_trimmed(a, bp, radius("a"), radius("bp")), _trimmed(ap, b, radius("ap"), radius("b"))]
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

    def local(points: np.ndarray) -> np.ndarray:  # v from the pad's top face
        rel = points - b
        return np.stack((np.sum(rel * e, axis=1), np.sum(rel * n, axis=1) + packaging.distal_drop(p)), axis=1)

    far = np.full(count, -np.inf)
    for start, end in links:
        _, high = _block_extent(local(start), local(end), distal.start_mm, distal.end_mm,
                                packaging.distal_pad.height_mm)
        far = np.fmax(far, high)
    distal_free = np.where(np.isfinite(far), distal.end_mm - far, distal.length_mm)
    return {"proximal_pad": np.maximum(proximal_free, 0.0), "distal_pad": np.maximum(distal_free, 0.0)}


def fold_free_lengths(
    proximal_face: np.ndarray, proximal_normal: np.ndarray, distal_face: np.ndarray, distal_normal: np.ndarray,
) -> dict[str, np.ndarray]:
    """Per-sample length of each contact face not hidden in the fold.

    Faces are segments ``(n, 2, 2)`` (or ``(2, 2)``), their outer end first
    (the MCP end of the proximal face, the fingertip end of the distal one),
    with their outward (palmar) unit normals ``(n, 2)`` (or ``(2,)``), all in
    the proximal frame. A point of one face is hidden when it lies behind the
    other face (on its pad's side of its line) and opposite it (its
    projection on that line falls within the other face): there the other
    face, not an object, is in front of it. Each face loses the length so
    hidden. The other face's segment, not its endless line, decides: barely
    flexed, two faces at different depths cross far from either pad, and
    hide nothing (the lines alone would hide a whole face there). Where the
    other face covers the crossing, this is the crossing rule
    (:func:`face_crossing_lengths`); the length lost grows continuously with
    the flexion.
    """

    def faces(face, normal):
        face = np.asarray(face, dtype=float)
        face = face[None] if face.ndim == 2 else face
        normal = np.broadcast_to(np.asarray(normal, dtype=float), (len(face), 2))
        return face, normal

    proximal_face, proximal_normal = faces(proximal_face, proximal_normal)
    distal_face, distal_normal = faces(distal_face, distal_normal)
    count = max(len(proximal_face), len(distal_face))
    proximal_face = np.broadcast_to(proximal_face, (count, 2, 2))
    distal_face = np.broadcast_to(distal_face, (count, 2, 2))
    proximal_normal = np.broadcast_to(proximal_normal, (count, 2))
    distal_normal = np.broadcast_to(distal_normal, (count, 2))

    def hidden(face: np.ndarray, other: np.ndarray, other_normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """The face's length and how much of it ``other`` hides."""

        outer, inner = face[:, 0], face[:, 1]
        length = np.linalg.norm(inner - outer, axis=1)
        t = (inner - outer) / np.where(length > 0.0, length, 1.0)[:, None]
        g0, g1 = other[:, 0], other[:, 1]
        other_length = np.linalg.norm(g1 - g0, axis=1)
        tg = (g1 - g0) / np.where(other_length > 0.0, other_length, 1.0)[:, None]
        rel = outer - g0
        # Behind the other face: (p - g0) . n < 0, linear in s along the face.
        a, b = np.sum(rel * other_normal, axis=1), np.sum(t * other_normal, axis=1)
        flat = np.abs(b) < 1e-12
        root = -a / np.where(flat, 1.0, b)
        behind_low = np.where(flat, np.where(a < 0.0, -np.inf, np.inf), np.where(b < 0.0, root, -np.inf))
        behind_high = np.where(flat, np.where(a < 0.0, np.inf, -np.inf), np.where(b > 0.0, root, np.inf))
        # Opposite it: 0 <= (p - g0) . tg <= its length.
        c, d = np.sum(rel * tg, axis=1), np.sum(t * tg, axis=1)
        level = np.abs(d) < 1e-12
        first = -c / np.where(level, 1.0, d)
        second = (other_length - c) / np.where(level, 1.0, d)
        inside = (c >= 0.0) & (c <= other_length)
        facing_low = np.where(level, np.where(inside, -np.inf, np.inf), np.minimum(first, second))
        facing_high = np.where(level, np.where(inside, np.inf, -np.inf), np.maximum(first, second))
        low = np.maximum.reduce([np.zeros(count), behind_low, facing_low])
        high = np.minimum.reduce([length, behind_high, facing_high])
        return length, np.maximum(high - low, 0.0)

    proximal_length, proximal_hidden = hidden(proximal_face, distal_face, distal_normal)
    distal_length, distal_hidden = hidden(distal_face, proximal_face, proximal_normal)
    return {"proximal_pad": proximal_length - proximal_hidden, "distal_pad": distal_length - distal_hidden}


def fold_hidden_points(face: np.ndarray, other_face: np.ndarray, other_normal: np.ndarray,
                       points: int) -> np.ndarray:
    """Which of ``points`` evenly spread points of a face (at the middles of
    equal pieces, as :func:`probe_blocked` probes) the other face hides in
    the fold, as :func:`fold_free_lengths` measures it: behind the other face
    and opposite it. Faces ``(n, 2, 2)``, the other's normal ``(n, 2)``;
    returns ``(n, points)``."""

    face = np.asarray(face, dtype=float)
    count = len(face)
    other_face = np.broadcast_to(np.asarray(other_face, dtype=float), (count, 2, 2))
    other_normal = np.broadcast_to(np.asarray(other_normal, dtype=float), (count, 2))
    share = (np.arange(points) + 0.5) / points
    at = face[:, :1] + share[None, :, None] * (face[:, 1:] - face[:, :1])           # (n, k, 2)
    g0, g1 = other_face[:, 0], other_face[:, 1]
    other_length = np.linalg.norm(g1 - g0, axis=1)
    tg = (g1 - g0) / np.where(other_length > 0.0, other_length, 1.0)[:, None]
    rel = at - g0[:, None]
    behind = np.sum(rel * other_normal[:, None], axis=-1) < 0.0
    along = np.sum(rel * tg[:, None], axis=-1)
    return behind & (along >= 0.0) & (along <= other_length[:, None])


def _point_segment_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    d = b - a
    t = np.clip(np.sum((p - a) * d, axis=-1) / np.maximum(np.sum(d * d, axis=-1), 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[..., None] * d), axis=-1)


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
    inside = _inside(np.ascontiguousarray(start[:, None]), polygons)[:, 0]
    return np.where(crossing | inside, 0.0, gap)


PROBE_STEP_MM = 0.5
"""Spacing of the face points :func:`probe_free_lengths` probes."""


def probe_free_lengths(face: np.ndarray, normal: np.ndarray, depth_mm: float, polygons=(), segments=(),
                       disks=(), step_mm: float = PROBE_STEP_MM) -> np.ndarray:
    """The face's length times the share of its points :func:`probe_blocked` leaves free."""

    length, blocked = probe_blocked(face, normal, depth_mm, polygons, segments, disks, step_mm)
    return length * (1.0 - blocked.mean(axis=1))


def probe_blocked(face: np.ndarray, normal: np.ndarray, depth_mm: float, polygons=(), segments=(),
                  disks=(), step_mm: float = PROBE_STEP_MM) -> tuple[np.ndarray, np.ndarray]:
    """Per-sample length of a contact face an object can reach.

    A point of the face is reachable when nothing lies within ``depth_mm`` in
    front of it: the probe from it along the outward normal, ``depth_mm``
    long, meets no solid. Faces ``(n, 2, 2)`` (outer end first) with unit
    normals ``(n, 2)``; solids, every other part of the finger at each
    sample, as convex ``polygons`` ``(n, m, 2)``, ``segments`` ``(n, 2, 2)``
    (bars seen edge on) and ``disks`` ``((n, 2) centres, radius)``. The face
    is probed every ``step_mm`` (at the middles of equal pieces). Returns
    each face's length ``(n,)`` and which points are blocked ``(n, k)``.
    """

    face = np.asarray(face, dtype=float)
    count = len(face)
    normal = np.broadcast_to(np.asarray(normal, dtype=float), (count, 2))
    outer, inner = face[:, 0], face[:, 1]
    length = np.linalg.norm(inner - outer, axis=1)
    points = max(int(math.ceil(float(length.max()) / step_mm)), 1) if count else 1
    share = (np.arange(points) + 0.5) / points
    start = outer[:, None] + share[None, :, None] * (inner - outer)[:, None]          # (n, k, 2)
    end = start + depth_mm * normal[:, None]
    start = start + 1e-6 * normal[:, None]  # off the face itself
    blocked = np.zeros((count, points), dtype=bool)
    start, end = np.ascontiguousarray(start), np.ascontiguousarray(end)
    # A bar is a polygon of two corners (seen edge on): the probe crosses it or not.
    for polygon in (*polygons, *segments):
        polygon = np.broadcast_to(np.asarray(polygon, dtype=float), (count, *np.shape(polygon)[-2:]))
        blocked |= _kernels.probe_polygon(start, end, _polygons(polygon))
    for centers, radius in disks:
        centers = np.broadcast_to(np.asarray(centers, dtype=float), (count, 2))[:, None]
        d = end - start
        t = np.clip(np.sum((centers - start) * d, axis=-1) / np.maximum(np.sum(d * d, axis=-1), 1e-12), 0.0, 1.0)
        blocked |= np.linalg.norm(start + t[..., None] * d - centers, axis=-1) <= radius
    return length, blocked


def probe_contact_lengths(linkage: FingerLinkage, packaging: Packaging, b: np.ndarray, c: np.ndarray,
                          bp: np.ndarray, depth_mm: float, solids: str = "all") -> dict[str, np.ndarray]:
    """Each contact face's reachable length (:func:`probe_free_lengths`) at
    the posed ``B``, ``C`` and ``B'`` (``(n, 2)``), MCP at 0, against the
    ``solids`` in front of it, whatever their Y layer (seen from the side, an
    object meets the outer layers' links first): ``"all"``, every solid of
    the finger but its own pad; ``"pads"``, the other pad alone;
    ``"hardware"``, all but the pads (the distal carrier, the links ``A-B'``
    and ``A'-B``, the four pins' housings, the PIP and MCP servos)."""

    p = linkage.parameters
    b, c, bp = np.atleast_2d(b), np.atleast_2d(c), np.atleast_2d(bp)
    count = len(b)
    proximal_face, proximal_normal, distal_face, distal_normal = contact_faces(linkage, packaging, b, c)
    proximal = np.broadcast_to(proximal_pad_polygon(linkage, packaging), (count, *proximal_pad_polygon(linkage,
                                                                                                    packaging).shape))
    distal = distal_pad_polygon(linkage, packaging, b, c)
    shared_polygons, segments, disks = [], [], []
    if solids not in ("all", "pads", "hardware"):
        raise ValueError(f"solids must be all, pads or hardware, not {solids!r}")
    if solids != "pads":
        if packaging.distal_carrier_height_mm > 0.0:
            shared_polygons.append(distal_carrier_polygon(linkage, packaging, b, c))
        shared_polygons += [packaging.pip_footprint(linkage).outline, packaging.mcp_footprint().outline]
        a = np.broadcast_to((p.a_x_mm, p.a_z_mm), (count, 2))
        ap = np.broadcast_to((p.ap.x_mm, p.ap.z_mm), (count, 2))
        segments = [np.stack((a, bp), axis=1), np.stack((ap, b), axis=1)]
        disks = [(a, packaging.pivot_radius_mm("a")), (ap, packaging.pivot_radius_mm("ap")),
                 (b, packaging.pivot_radius_mm("b")), (bp, packaging.pivot_radius_mm("bp"))]
    other = {"proximal": [] if solids == "hardware" else [distal], "distal": [] if solids == "hardware" else [proximal]}
    proximal_free = probe_free_lengths(np.broadcast_to(proximal_face, (count, 2, 2)), proximal_normal, depth_mm,
                                       [*other["proximal"], *shared_polygons], segments, disks)
    distal_free = probe_free_lengths(distal_face, distal_normal, depth_mm, [*other["distal"], *shared_polygons],
                                     segments, disks)
    return {"proximal_pad": proximal_free, "distal_pad": distal_free}


def contact_faces(linkage: FingerLinkage, packaging: Packaging, b: np.ndarray, c: np.ndarray):
    """The linkage's two contact faces as :func:`fold_free_lengths` takes them:
    the proximal one fixed, the distal one at the posed ``B`` and ``C``."""

    p = linkage.parameters
    b, c = np.atleast_2d(b), np.atleast_2d(c)
    spans = pad_spans(p, packaging)
    proximal, distal = spans["proximal_pad"], spans["distal_pad"]
    face_z = packaging.proximal_pad_line_z(p) - packaging.proximal_pad.height_mm
    proximal_face = np.array(((proximal.start_mm, face_z), (proximal.end_mm, face_z)))
    e = (c - b) / p.bc_length_mm
    n = np.stack((-e[:, 1], e[:, 0]), axis=1)
    face_v = -packaging.distal_drop(p) - packaging.distal_pad.height_mm  # dorsal of BC
    outer = b + distal.end_mm * e + face_v * n
    inner = b + distal.start_mm * e + face_v * n
    return proximal_face, np.array((0.0, -1.0)), np.stack((outer, inner), axis=1), -n


def face_crossing_lengths(
    linkage: FingerLinkage, packaging: Packaging, b: np.ndarray, c: np.ndarray,
) -> dict[str, np.ndarray]:
    """Per-sample length of each pad's face outside the fold, from its outer end.

    The two contact faces, extended, cross where the finger folds: past that
    point each face lies inside the V the other one closes, facing it, where
    no object reaches. Each pad counts from its outer end (``a`` of the
    proximal pad, the ``C`` end of the distal one) to the crossing. Barely
    flexed, the faces cross below the centre of rotation; extended (the faces
    in line, or parallel) neither loses any. Samples are four-bar poses in the
    proximal-link frame, given by the posed ``B`` and ``C`` (``(n, 2)``).
    """

    p = linkage.parameters
    b, c = np.atleast_2d(b), np.atleast_2d(c)
    spans = pad_spans(p, packaging)
    proximal, distal = spans["proximal_pad"], spans["distal_pad"]
    face_z = packaging.proximal_pad_line_z(p) - packaging.proximal_pad.height_mm
    face_v = -packaging.distal_drop(p) - packaging.distal_pad.height_mm  # dorsal of BC
    e = (c - b) / p.bc_length_mm
    n = np.stack((-e[:, 1], e[:, 0]), axis=1)
    # The distal face's line, B + u e + face_v n, meets z = face_z at u.
    rise = face_z - b[:, 1] - face_v * n[:, 1]
    crossing = np.abs(e[:, 1]) > 1e-9
    u = np.where(crossing, rise / np.where(crossing, e[:, 1], 1.0), np.inf)
    x = b[:, 0] + u * e[:, 0] + face_v * n[:, 0]
    in_line = ~crossing | (np.abs(rise) < 1e-9)  # extended: one line, no fold
    proximal_free = np.where(in_line, proximal.length_mm, np.clip(x - proximal.start_mm, 0.0, proximal.length_mm))
    distal_free = np.where(in_line, distal.length_mm, np.clip(distal.end_mm - u, 0.0, distal.length_mm))
    return {"proximal_pad": proximal_free, "distal_pad": distal_free}


def _trimmed(start: np.ndarray, end: np.ndarray, radius: float,
             end_radius: float | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Segments ``start-end`` shortened by ``radius`` at the start and
    ``end_radius`` (default: the same) at the end, to a point at most."""

    direction = end - start
    length = np.where((norm := np.linalg.norm(direction, axis=1)) > 0.0, norm, 1.0)
    if end_radius is None or end_radius == radius:
        head = tail = np.minimum(radius / length, 0.5)[:, None]
    else:  # the two trims meet at most, in proportion
        total = np.maximum((radius + end_radius) / length, 1.0)
        head, tail = (radius / length / total)[:, None], (end_radius / length / total)[:, None]
    return start + head * direction, end - tail * direction


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
    radius = {name: packaging.pivot_radius_mm(name[:-len("_pin")]) for name in pins}
    signed = packaging.overlap_depth

    boxes = {name: (polygon.min(axis=1), polygon.max(axis=1)) for name, polygon in pads.items()}
    boxes.update({name: (center - radius[name], center + radius[name]) for name, center in pins.items()})
    outline = footprint.outline
    boxes["pip_servo"] = (np.broadcast_to(outline.min(axis=0), (count, 2)),
                          np.broadcast_to(outline.max(axis=0), (count, 2)))

    def gap(pad: str, other: str) -> np.ndarray:
        polygon = pads[pad]
        if other == "mcp_servo":
            if pad == "proximal_pad":  # fixed in the proximal frame: the same at every sample
                return np.full(count, proximal_pad_mcp_gap(polygon[0], packaging, mcp_samples_rad))
            return _mcp_gap(polygon, packaging, mcp_samples_rad)
        # Broad phase: the bounding boxes' gap bounds the gap from below.
        (low, high), (other_low, other_high) = boxes[pad], boxes[other]
        result = np.hypot(*np.maximum(np.maximum(other_low - high, low - other_high), 0.0).T)
        near = np.flatnonzero(result <= packaging.body_clearance_mm + EXACT_BAND_MM)
        if near.size:
            if other in pins:  # signed already: a disk's centre distance less its radius
                result[near] = _polygon_to_disk(polygon[near], pins[other][near], radius[other])
                return result
            if other in pads:
                solid = pads[other][near]
                result[near] = _polygons_gap(polygon[near], solid)
            elif other == "pip_servo":
                solid = np.broadcast_to(outline, (near.size, *outline.shape))
                result[near] = _polygon_to_footprint(polygon[near], footprint)
            else:
                raise ValueError(f"no geometry for the pair {pad!r}, {other!r}")
            if signed:
                result[near] = _signed(result[near], polygon[near], solid)
        return result

    margins = {}
    for pair in checked_pairs(packaging):
        pad, other = pair if pair[0] in PADS else pair[::-1]
        clearance = packaging.mcp_pad_clearance_mm if other == "mcp_servo" else packaging.body_clearance_mm
        margin = gap(pad, other) - clearance
        # A pad set exactly the clearance away (the MCP offset) must not fail by rounding.
        margins[pair] = np.where(np.abs(margin) < _MARGIN_TOLERANCE_MM, 0.0, margin)
    if packaging.distal_carrier_height_mm > 0.0:
        carrier = distal_carrier_polygon(linkage, packaging, b, c)
        margin = footprint_gaps(carrier, footprint, packaging) - packaging.body_clearance_mm
        margins[("distal_carrier", "pip_servo")] = np.where(np.abs(margin) < _MARGIN_TOLERANCE_MM, 0.0, margin)
    return margins


def proximal_pad_mcp_gap(polygon: np.ndarray, packaging: Packaging, mcp_samples_rad: Sequence[float]) -> float:
    """Gap of the proximal pad (``(m, 2)``, fixed in the proximal frame) to the
    MCP servo over the MCP flexion samples.

    The pad starts at the servo by construction, so near the servo it is the
    same block in every design: its start, line and height are the study's.
    When the pad (chamfer included) ends beyond the servo's reach and the exact
    band, only that block can come near the servo, and its gap is solved once
    per study (:func:`_proximal_block_gap`); otherwise the whole pad is checked.
    """

    limit = packaging.mcp_footprint().reach_mm + packaging.mcp_pad_clearance_mm + EXACT_BAND_MM
    height = packaging.proximal_pad.height_mm
    near = polygon[:, 0] < limit
    if polygon[:, 0].max() - height > limit and near.any():
        top = float(polygon[:, 1].max())
        start = float(polygon[:, 0].min())
        bottom = float(polygon[:, 1].min())
        if np.allclose(polygon[near][:, 0], start) and math.isclose(top - bottom, height):  # the square edge a-c
            return _proximal_block_gap(packaging, start, top, height, limit, tuple(mcp_samples_rad))
    return float(_mcp_gap(polygon[None], packaging, mcp_samples_rad)[0])


@functools.lru_cache(maxsize=64)
def _proximal_block_gap(packaging: Packaging, start: float, top: float, height: float, limit: float,
                        mcp_samples_rad: tuple[float, ...]) -> float:
    """The MCP servo gap of the proximal pad's block within ``limit`` of ``O``."""

    block = np.array(((start, top - height), (limit, top - height), (limit, top), (start, top)))
    return float(_mcp_gap(block[None], packaging, mcp_samples_rad)[0])


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
    exact = packaging.mcp_pad_clearance_mm + EXACT_BAND_MM

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
    result = gap.reshape(count, -1).min(axis=1)
    touching = np.flatnonzero(result <= _MARGIN_TOLERANCE_MM)
    if packaging.overlap_depth and touching.size:  # touching or overlapping at some MCP angle
        outline = servo.outline
        depth = np.full(touching.size, math.inf)
        for angle in samples:
            turned = turn_points(polygons[touching].reshape(-1, 2), -angle).reshape(touching.size, -1, 2)
            depth = np.minimum(depth, _separation(turned, np.broadcast_to(outline, (touching.size, *outline.shape))))
        result[touching] = np.where(depth < 0.0, depth, result[touching])
    return result


def _edges(polygons: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return polygons, np.roll(polygons, -1, axis=-2)


def _separation(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """Signed separation of convex ``(n, m, 2)`` and ``(n, k, 2)`` polygons per
    sample: the most any edge normal of either separates their projections.
    Positive: apart (by at least that); 0: touching; negative: overlapping,
    by that depth."""

    return _kernels.separation(_polygons(first), _polygons(second))

def _signed(gap: np.ndarray, first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """A gap that is 0 for touching and overlapping alike, made negative by the
    overlap's depth (:func:`_separation`) where the convex polygons overlap."""

    depth = _separation(first, second)
    return np.where(depth < 0.0, depth, gap)


def polygon_gaps(first: np.ndarray, second: np.ndarray, packaging: Packaging) -> np.ndarray:
    """Gap per sample between convex ``(n, m, 2)`` and ``(n, k, 2)`` polygons;
    with ``packaging.overlap_depth`` an overlap is negative by its depth."""

    gap = _polygons_gap(first, second)
    return _signed(gap, first, second) if packaging.overlap_depth else gap


def footprint_gaps(polygons: np.ndarray, footprint: "ServoFootprint", packaging: Packaging) -> np.ndarray:
    """Gap per sample between ``(n, m, 2)`` polygons and a servo footprint
    (signed as :func:`polygon_gaps`)."""

    gap = _polygon_to_footprint(polygons, footprint)
    if not packaging.overlap_depth:
        return gap
    return _signed(gap, polygons, np.broadcast_to(footprint.outline, (len(polygons), *footprint.outline.shape)))


def mcp_servo_gaps(polygons: np.ndarray, packaging: Packaging, mcp_samples_rad: Sequence[float]) -> np.ndarray:
    """Least gap per sample of ``(n, m, 2)`` polygons to the MCP servo over the
    MCP flexion samples (signed as :func:`polygon_gaps`)."""

    return _mcp_gap(polygons, packaging, mcp_samples_rad)


def servo_floor_x(packaging: Packaging, pivot_z: float, rear: tuple[float, float],
                  mcp_angles_rad: Sequence[float]) -> float:
    """The least shaft ``x`` (at ``pivot_z``) of a servo of the PIP servo's size,
    laid back along ``rear``, that keeps it clear of the MCP servo at the MCP
    angles: searched from far ahead of ``O`` back to the first overlap (far
    behind ``O`` it is clear again, which is no finger), in 5 mm steps, then
    bisected to 0.05 mm."""

    servo = packaging.pip_servo
    angles = tuple(mcp_angles_rad)

    def clear(x: float) -> bool:
        footprint = ServoFootprint((x, pivot_z), rear, servo.radius_mm, servo.rear_extent_mm)
        return packaging.mcp_servo_gap_mm(footprint, angles) - packaging.body_clearance_mm >= 0.0

    high = 200.0
    low = high - 5.0
    while low > -200.0 and clear(low):
        high, low = low, low - 5.0
    while high - low > 0.05:
        middle = 0.5 * (low + high)
        low, high = (low, middle) if clear(middle) else (middle, high)
    return high


def pip_servo_pad_gap_mm(linkage: FingerLinkage, packaging: Packaging) -> float:
    """Gap between the PIP servo body and the proximal pad (both fixed in the
    proximal frame), signed as :func:`footprint_gaps`."""

    return float(footprint_gaps(proximal_pad_polygon(linkage, packaging)[None],
                                packaging.pip_footprint(linkage), packaging)[0])


def _inside(points: np.ndarray, polygons: np.ndarray) -> np.ndarray:
    """Whether each of ``(n, p, 2)`` points lies inside (or on) the convex ``(n, m, 2)`` polygon."""

    return _kernels.inside(_polygons(points), _polygons(polygons))

def _polygons_gap(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """Gap between two ``(n, m, 2)`` convex polygons per sample (0 when they touch or overlap)."""

    return _kernels.polygons_gap(_polygons(first), _polygons(second))

def _polygon_to_disk(polygons: np.ndarray, centers: np.ndarray, radius: float) -> np.ndarray:
    """Gap from ``(n, m, 2)`` polygons to disks at ``(n, 2)`` centres (negative: overlap)."""

    return _kernels.polygon_to_disk(_polygons(polygons), _points(centers), float(radius))

def _polygon_to_footprint(polygons: np.ndarray, footprint: "ServoFootprint") -> np.ndarray:
    """Gap from ``(n, m, 2)`` polygons to a servo footprint (0 when they touch or overlap)."""

    forward, side = footprint._axes()
    return _kernels.polygon_to_footprint(_polygons(polygons), float(footprint.pivot[0]), float(footprint.pivot[1]),
                                         float(forward[0]), float(forward[1]), float(side[0]), float(side[1]),
                                         float(footprint.radius_mm), float(footprint.rear_extent_mm))

def turn_points(points: np.ndarray, angle_rad: float) -> np.ndarray:
    """Points ``(n, 2)`` turned by ``angle_rad`` about the origin ``O``."""

    cosine, sine = math.cos(angle_rad), math.sin(angle_rad)
    return np.stack((cosine * points[:, 0] - sine * points[:, 1],
                     sine * points[:, 0] + cosine * points[:, 1]), axis=1)



def point_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> np.ndarray:
    """Distance from ``point`` (one, or one per segment) to each segment ``start[i]-end[i]``."""

    point, start, end = np.broadcast_arrays(*(np.asarray(a, dtype=float) for a in (point, start, end)))
    return _kernels.point_segment_distance(_points(point), _points(start), _points(end))

def _segments_cross(start: np.ndarray, end: np.ndarray, q0: np.ndarray, q1: np.ndarray) -> np.ndarray:
    """Whether segment ``start[i]-end[i]`` crosses ``q0[i]-q1[i]``. Collinear
    segments only touch if an endpoint lies on the other, which the endpoint
    distances already find."""

    arrays = np.broadcast_arrays(*(np.asarray(a, dtype=float) for a in (start, end, q0, q1)))
    return _kernels.segments_cross(*(_points(array) for array in arrays))


def _points(array) -> np.ndarray:
    return np.ascontiguousarray(np.asarray(array, dtype=float).reshape(-1, 2))


def _polygons(array) -> np.ndarray:
    array = np.asarray(array, dtype=float)
    return np.ascontiguousarray(array.reshape(-1, *array.shape[-2:]))

__all__ = [
    "contact_faces",
    "fold_free_lengths",
    "fold_hidden_points",
    "segment_polygon_gaps",
    "probe_blocked",
    "probe_contact_lengths",
    "probe_free_lengths",
    "BODY_CLEARANCE_MM",
    "COLLISION_GROUPS",
    "COMPONENTS",
    "Component",
    "ContactPad",
    "DEFAULT_PIP_SAMPLES_RAD",
    "EXACT_BAND_MM",
    "FIXED_MARGIN_PAIRS",
    "fixed_margin_pairs",
    "carrier_outline",
    "checked_pairs",
    "distal_carrier_polygon",
    "distal_pad_polygon",
    "layer_separated_pairs",
    "pad_chamfer_mm",
    "pad_outline",
    "pad_spans",
    "usable_pad_lengths",
    "proximal_pad_polygon",
    "pad_margins",
    "pip_servo_pad_gap_mm",
    "servo_floor_x",
    "proximal_pad_mcp_gap",
    "footprint_gaps",
    "mcp_servo_gaps",
    "polygon_gaps",
    "PIVOTS",
    "PIVOT_CLEARANCE_DIAMETER_MM",
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
