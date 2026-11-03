"""The linkage finger's packaging: where its servos, pin housings and pads
sit on the four-bar, and every collision check between them (the hardware
itself and the gaps of any mechanism are ``core.morphology.packaging``'s).

The checks, each a gap beyond its solids or a margin beyond the clearance:

- joints and servos: two pins on one body (:func:`pin_overlap_mm`), each pad
  and its own pins (:func:`own_pin_gaps`), the pin off the PIP servo and the
  servo's shaft (:func:`pivot_pin_gap_mm`), ``A`` and ``B'`` and the servo's
  body (:func:`a_pin_servo_gap_mm`, :func:`bp_pin_servo_margins`), the driver
  link and the servo (:func:`driver_link_servo_gaps`), the two servos
  (:func:`servo_servo_gap_mm`); :func:`servo_clearance` sums them up over a sweep;
- the pads against the parts of other bodies (:func:`pad_margins`), the PIP
  servo against the proximal pad (:func:`pip_servo_pad_gap_mm`) and the links
  as capsules (:func:`link_capsule_margins`).

Each pad is its segment's contact interface space: a block ``height_mm``
deep (the module and its mounting together), parallel to the link line
(``OA``, ``BC``), starting on it when the pad, there, clears its own body's
pin housings and (the proximal pad) the PIP servo, within its link; else on
the line tangent to its link's pin clearance (``A``'s for the proximal pad,
``B``'s for the distal one), running under the pin (:func:`pad_offsets`;
:meth:`LinkagePackaging.interface_offset_mm` the tangent one).

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
height palmar of it (``packaging.hardware.ContactPad``). The proximal pad rides on
``O-A-A'``: it starts at :attr:`Packaging.proximal_pad_start_mm` (``O``, past
the MCP servo's shaft disk by the body clearance when ``pad_mcp_check``, plus
its offset) with a square edge ``a-c`` and runs right to ``A''``. The distal
pad rides on ``B'-B-C``: it starts its offset short of ``C`` and runs left to
``B''``. A pad's length is not set: it ends at ``A''`` or ``B''``. Without
``pad_mcp_check`` the MCP servo is not checked.

**Rigid bodies.** Each part (:data:`PARTS`) sits on a rigid body (the palm,
the proximal and distal bodies, the driver link, the coupler) and in a set
of Y layers (:data:`LAYERS`, names, no thickness): the pads share the central
layer with the servos; the driver link's and the coupler's plates are
outside it, one on each side; the pins' bearing housings in the central
layer, on the body that carries them. Parts of different bodies that share a
layer are checked, but a pin and a body it joins (:data:`JOINTS`) and a
pin's housing and anything but the PIP servo and the pads
(:data:`PIN_CHECKED`); the packaging's switches leave pairs out, its
keep-outs add pairs of one body (:func:`layout`). The
gaps are ``core.morphology.packaging.assembly``'s, at the bodies' poses
(:func:`poses`).
"""

from __future__ import annotations

import functools
import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Sequence

import numpy as np

from core.morphology.finger import Finger
from core.morphology.mechanism.linkage.model import four_bar
from core.morphology.packaging.assembly import (
    PALM,
    Assembly,
    Bar,
    Disk,
    Footprint,
    Joint,
    KeepOut,
    Layout,
    Pair,
    Part,
    Polygon,
    Poses,
    margins,
)
from core.morphology.packaging.faces import probe_free_lengths
from core.morphology.packaging.hardware import Packaging
from core.morphology.packaging.shapes import PadSpan, ServoFootprint, pad_outline, segment_polygon_gaps


DEFAULT_DRIVER_SAMPLES_RAD = tuple(math.radians(degrees) for degrees in range(0, 91))
PIVOTS = ("a", "ap", "b", "bp")
"""The pivots by name: ``A``, ``A'``, ``B``, ``B'``."""


@dataclass(frozen=True)
class LinkagePackaging(Packaging):
    """The linkage's packaging: the common hardware (:class:`Packaging`) and
    where it sits on the four-bar: each pivot's size (``A``, ``A'``, ``B``,
    ``B'``), the clearance around the PIP servo's shaft ``A'``, where the
    contact interface spaces start, and the linkage's own collision
    rules (see :func:`layout`)."""

    allow_collision_pip_servo_to_joint_a: bool = False
    """``False``: ``A``'s pin envelope keeps the body clearance from the virtual
    clearance disk around the servo's shaft ``A'`` (``ap_clearance_diameter_mm``)."""
    allow_collision_pip_servo_to_joint_bp: bool = True
    """``True``: the ``B'`` pin envelope may pass through the PIP servo body
    (the driver link sits in another Y layer). ``False``: it keeps the body
    clearance from the body over the motion, which ends the safe range."""
    ap_clearance_diameter_mm: float = 7.0
    """Virtual clearance around the PIP servo's shaft ``A'``, what ``A``'s pin
    envelope keeps clear of. The
    servo body itself is not checked against that pin: it rides on a bracket
    off the body."""
    pad_link_ratio_max: float | None = None
    """Most pad length per length of its link (the proximal pad over ``|OA|``,
    the distal pad over ``BC``): a pad may stand out of its link by this much.
    A free-length pad is cut to it; a fixed one must not exceed it. ``None``:
    unbounded."""
    a_pivot_diameter_mm: float | None = None
    ap_pivot_diameter_mm: float | None = None
    b_pivot_diameter_mm: float | None = None
    bp_pivot_diameter_mm: float | None = None
    """Each pivot's own size in the side view (:meth:`pivot_radius_mm`);
    ``None``: ``pivot_clearance_diameter_mm``. The ``A``-``A'`` check still
    uses ``ap_clearance_diameter_mm`` at the servo's shaft."""
    pip_servo_above_oa: bool = True
    """The PIP servo stays dorsal of the ``OA`` line (the ``oa_interface_mm``
    constraint)."""
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

    def __post_init__(self) -> None:
        super().__post_init__()
        for pivot in PIVOTS:
            diameter = getattr(self, f"{pivot}_pivot_diameter_mm")
            if diameter is not None and not diameter >= 0.0:
                raise ValueError(f"{pivot}_pivot_diameter_mm must be non-negative, or None")
        if self.pad_link_ratio_max is not None and not self.pad_link_ratio_max > 0.0:
            raise ValueError("pad_link_ratio_max must be positive, or None")
        if not self.ap_clearance_diameter_mm >= 0.0:
            raise ValueError("ap_clearance_diameter_mm must be non-negative")

    def pivot_radius_mm(self, pivot: str) -> float:
        """Radius of one pivot (``"a"``, ``"ap"``, ``"b"``, ``"bp"``) in the side
        view: its own size, else the packaging's."""

        diameter = getattr(self, f"{pivot}_pivot_diameter_mm")
        return (self.pivot_clearance_diameter_mm if diameter is None else diameter) / 2.0

    def interface_offset_mm(self, pad: str) -> float:
        """How far palmar of its link line (``OA``, ``BC``) a pad's contact
        interface space starts when it runs under its link's pin: on the line
        tangent to the pin's clearance (``A``'s, ``B``'s), parallel to the link
        line (:func:`pad_offsets`: on the link line where it clears them)."""

        return self.pivot_radius_mm({"proximal_pad": "a", "distal_pad": "b"}[pad])




# The finger as rigid bodies (core.morphology.packaging.assembly): its parts,
# each on a body and in Y layers, and how the bodies join.

LAYERS = ("central", "driver", "coupler")
"""Across the finger: the pads and servos in the central layer, as wide as a
servo; the driver link's and the coupler's plates outside it, one on each
side (they cross in the side view); the pins' bearing housings in the
central layer, on their bodies (a pin's shaft, through its link's plate, is
not modelled)."""
_CENTRAL = frozenset({"central"})
PARTS = (
    Part("proximal_pad", "proximal", _CENTRAL, pad=True),
    Part("mcp_servo", PALM, _CENTRAL),
    Part("distal_pad", "distal", _CENTRAL, pad=True),
    Part("bp_pin", "distal", _CENTRAL),
    Part("b_pin", "distal", _CENTRAL),
    Part("pip_servo", "proximal", _CENTRAL),
    Part("a_pin", "proximal", _CENTRAL),
    Part("ap_pin", "proximal", _CENTRAL),
    Part("driver_link", "driver", frozenset({"driver"})),
    Part("coupler", "coupler", frozenset({"coupler"})),
    Part("pip_shaft", "proximal", frozenset()),
)
"""Each part's body and layers, in the order their pairs are listed. The PIP
servo is bolted to the proximal body; each pin is its bearing housing, on
the body that carries it in the central layer (``A``, ``A'`` the proximal; ``B``,
``B'`` the distal). ``pip_shaft`` is the virtual clearance around the servo's
shaft (``LinkagePackaging.ap_clearance_diameter_mm``): in no layer, a keep-out only.
The links are their centre lines (their plates' width is not set yet)."""
COMPONENTS = MappingProxyType({part.name: part for part in PARTS})
JOINTS = (
    Joint("a_pin", ("proximal", "driver")),
    Joint("ap_pin", ("proximal", "coupler")),
    Joint("b_pin", ("distal", "coupler")),
    Joint("bp_pin", ("distal", "driver")),
)
PADS = ("proximal_pad", "distal_pad")
PINS = ("a_pin", "ap_pin", "b_pin", "bp_pin")
PIN_CHECKED = frozenset({"pip_servo", *PADS})
"""What a pin's housing is kept off (2026-11-03): the PIP servo's body and the
pads; the rest (the MCP servo, the other body's pins) it may touch. Two pins on one link and ``A`` and the servo's shaft are
:func:`layout`'s keep-outs."""


@functools.lru_cache(maxsize=64)
def layout(packaging: LinkagePackaging) -> Layout:
    """The linkage's rigid bodies as the packaging sets their checks.

    The rule checks parts of different bodies that share a layer (a pin and
    a body it joins are the joint; a pin's housing only against
    :data:`PIN_CHECKED`); the packaging's switches leave pairs out
    (``pad_pad_check``, ``pad_checks_mcp``, ``B'`` passing the servo with
    ``allow_collision_pip_servo_to_joint_bp``). Kept apart whatever the rule:
    two pins on one link (``A``-``B'``, ``B'``-``B``; their housings may
    touch), the pin off the PIP servo and the servo's shaft (unless
    ``allow_collision_pip_servo_to_joint_a``), ``A`` and the servo's body
    (``a_pin_servo_check``), the servo and the proximal pad
    (``pip_servo_pad_check``) and each pad and its own pins (``pads_off_pins``).
    """

    parts = PARTS
    unchecked = {tuple(sorted((pin, part.name))) for pin in PINS for part in parts
                 if part.name != pin and part.name not in PIN_CHECKED}
    if not packaging.pad_pad_check:
        unchecked.add(("distal_pad", "proximal_pad"))
    if not packaging.pad_checks_mcp("proximal_pad"):
        unchecked.add(("mcp_servo", "proximal_pad"))
    if not packaging.pad_checks_mcp("distal_pad"):
        unchecked.add(("distal_pad", "mcp_servo"))
    if packaging.allow_collision_pip_servo_to_joint_bp:
        unchecked.add(("bp_pin", "pip_servo"))
    keep_outs = [KeepOut("a_pin", "bp_pin", touch=True), KeepOut("bp_pin", "b_pin", touch=True)]
    if not packaging.allow_collision_pip_servo_to_joint_a:
        keep_outs.append(KeepOut("a_pin", "pip_shaft"))
    if packaging.a_pin_servo_check:
        keep_outs.append(KeepOut("a_pin", "pip_servo"))
    if packaging.pip_servo_pad_check:
        keep_outs.append(KeepOut("pip_servo", "proximal_pad"))
    if packaging.pads_off_pins:
        keep_outs += [KeepOut("proximal_pad", "a_pin"), KeepOut("proximal_pad", "ap_pin"),
                      KeepOut("distal_pad", "b_pin"), KeepOut("distal_pad", "bp_pin")]
    return Layout(parts, JOINTS, tuple(keep_outs), frozenset(unchecked))


def assembly(finger: Finger, packaging: LinkagePackaging) -> Assembly:
    """The design's parts, each in its body's frame (the distal body's: ``B``, along ``B->C``)."""

    four_bar(finger)
    radius = packaging.pivot_radius_mm
    shapes = {
        "proximal_pad": Polygon(proximal_pad_polygon(finger, packaging)),
        "mcp_servo": Footprint(packaging.mcp_footprint()),
        "distal_pad": Polygon(distal_pad_local(finger, packaging)),
        "bp_pin": Disk("bp", radius("bp")),
        "b_pin": Disk("b", radius("b")),
        "pip_servo": Footprint(pip_footprint(packaging, finger)),
        "a_pin": Disk("a", radius("a")),
        "ap_pin": Disk("ap", radius("ap")),
        "driver_link": Bar("a", "bp"),
        "coupler": Bar("ap", "b"),
        "pip_shaft": Disk("ap", packaging.ap_clearance_diameter_mm / 2.0),
    }
    return Assembly(layout(packaging), shapes)


def poses(finger: Finger, b: np.ndarray, c: np.ndarray, bp: np.ndarray,
          mcp_samples_rad: Sequence[float] = (0.0,)) -> Poses:
    """The bodies at the posed ``B``, ``C`` and ``B'`` (``(n, 2)``), the palm over ``mcp_samples_rad``."""

    b, c, bp = np.atleast_2d(b), np.atleast_2d(c), np.atleast_2d(bp)
    return Poses(len(b), origin={"distal": b}, axis={"distal": (c - b) / finger.distal.end_mm},
                 points={"o": np.zeros(2), "a": np.array((finger.joint.a.x_mm, finger.joint.a.z_mm)),
                         "ap": np.array((finger.joint.ap.x_mm, finger.joint.ap.z_mm)), "b": b, "bp": bp, "c": c},
                 mcp_samples_rad=mcp_samples_rad)


def reference_poses(finger: Finger, mcp_samples_rad: Sequence[float] = (0.0,)) -> Poses:
    """The extended pose (one sample)."""

    b, c, bp = (np.array([[point.x_mm, point.z_mm]]) for point in (finger.joint.b, finger.tip, finger.joint.bp))
    return poses(finger, b, c, bp, mcp_samples_rad)


def least_margin_mm(built: Assembly, rest: Poses, packaging: LinkagePackaging, *pairs: tuple[str, str]) -> float:
    """The least margin of these checked pairs in one pose (not snapped to 0:
    as the static checks have always taken them)."""

    found = margins(built, rest, packaging, [built.layout.pair(*pair) for pair in pairs], snap=False)
    return min(float(margin[0]) for margin in found.values())


def gap_mm(built: Assembly, rest: Poses, packaging: LinkagePackaging, first: str, second: str) -> float:
    """Two parts' gap in one pose, checked or not (no clearance taken off)."""

    key = tuple(sorted((first, second)))
    return float(margins(built, rest, packaging, [Pair(*key, "touch")], snap=False)[key][0])


# Joint and servo clearances, as single queries (servo_clearance, the figures).

def pin_overlap_mm(finger: Finger, packaging: LinkagePackaging) -> float:
    """How deep the housings of two pins on one link overlap (``A`` and
    ``B'``, the driver link; ``B'`` and ``B``), the deeper; touching is 0,
    apart negative."""

    return -least_margin_mm(assembly(finger, packaging), reference_poses(finger), packaging,
                            ("a_pin", "bp_pin"), ("bp_pin", "b_pin"))


def own_pin_gaps(finger: Finger, packaging: LinkagePackaging) -> tuple[float, float]:
    """Each pad's least gap to the pin housings of its own body, extended:
    the proximal pad to ``A`` and ``A'``, the distal pad to ``B`` and ``B'``."""

    built, rest = assembly(finger, packaging), reference_poses(finger)
    gap = lambda pad, pin: gap_mm(built, rest, packaging, pad, pin)  # noqa: E731
    return (min(gap("proximal_pad", "a_pin"), gap("proximal_pad", "ap_pin")),
            min(gap("distal_pad", "b_pin"), gap("distal_pad", "bp_pin")))


def a_pin_servo_gap_mm(finger: Finger, packaging: LinkagePackaging) -> float:
    """Gap between ``A``'s pin housing and the PIP servo's body."""

    return gap_mm(assembly(finger, packaging), reference_poses(finger), packaging, "a_pin", "pip_servo")


def bp_pin_servo_margins(finger: Finger, packaging: LinkagePackaging, b: np.ndarray, c: np.ndarray,
                         bp: np.ndarray) -> np.ndarray:
    """Per-sample margin of ``B'``'s pin housing beyond the PIP servo's body
    and the body clearance (negative: too near), checked or not."""

    found = margins(assembly(finger, packaging), poses(finger, b, c, bp), packaging, [Pair("bp_pin", "pip_servo")])
    return found[("bp_pin", "pip_servo")]


def driver_link_servo_gaps(finger: Finger, footprint: ServoFootprint, bp: np.ndarray) -> np.ndarray:
    """Per-sample gap of the driver link ``A-B'`` (at the posed ``B'``) to the
    PIP servo in the side view (reported only: they sit in different layers)."""

    a = np.array((finger.joint.a.x_mm, finger.joint.a.z_mm))
    return footprint.distance_to_segments(np.broadcast_to(a, np.shape(bp)), bp)


def link_capsule_margins(finger: Finger, packaging: LinkagePackaging, b: np.ndarray, c: np.ndarray,
                         bp: np.ndarray, radius_mm: float) -> dict[tuple[str, str], np.ndarray]:
    """Per-sample margin (mm) of each pad against the bars it is not mounted
    on, each a capsule of ``radius_mm`` about its centre line, in the side
    view: the proximal pad against the crossing bars ``A-B'`` and ``A'-B``
    and the distal body (``B'-B``, ``B-C``); the distal pad against the
    crossing bars and the proximal body (``O-A``, ``A-A'``, ``A'-O``). A pad
    against its own body's bars is its mounting, not a collision."""

    b, c, bp = np.atleast_2d(b), np.atleast_2d(c), np.atleast_2d(bp)
    count = len(b)
    o = np.zeros((count, 2))
    a = np.broadcast_to((finger.joint.a.x_mm, finger.joint.a.z_mm), (count, 2))
    ap = np.broadcast_to((finger.joint.ap.x_mm, finger.joint.ap.z_mm), (count, 2))
    polygon = proximal_pad_polygon(finger, packaging)
    pads = {"proximal_pad": np.broadcast_to(polygon, (count, *polygon.shape)),
            "distal_pad": distal_pad_polygon(finger, packaging, b, c)}
    crossing = {"a_bp": (a, bp), "ap_b": (ap, b)}
    bodies = {"proximal_pad": {**crossing, "bp_b": (bp, b), "b_c": (b, c)},
              "distal_pad": {**crossing, "o_a": (o, a), "a_ap": (a, ap), "ap_o": (ap, o)}}
    clearance = packaging.body_clearance_mm
    return {(pad, f"link_{name}"): segment_polygon_gaps(pads[pad], start, end) - radius_mm - clearance
            for pad, bars in bodies.items() for name, (start, end) in bars.items()}


@dataclass(frozen=True)
class ServoClearance:
    """Signed clearance margins of one linkage design (mm, >= 0 is feasible)."""

    mcp_servo_mm: float
    """PIP servo beyond the MCP servo body over the MCP angles, less the body clearance."""
    pin_mm: float
    """The pin envelope of the pivot off the PIP servo beyond the virtual
    clearance at its shaft (:func:`pivot_pin_gap_mm`), less the body clearance."""
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
    finger: Finger,
    packaging: LinkagePackaging | None = None,
    driver_samples_rad: Sequence[float] = DEFAULT_DRIVER_SAMPLES_RAD,
    mcp_samples_rad: Sequence[float] = (0.0,),
) -> ServoClearance:
    """Return the servo clearance margins over the increasing driver-link
    rotations ``driver_samples_rad`` (the link ``A-B'`` about ``A``).

    The pins and link are checked up to the last sample that closes. The PIP
    servo is checked against the MCP servo body at each MCP flexion sample;
    extension (negative flexion) brings it toward the MCP servo's rear body.
    """

    packaging = packaging or LinkagePackaging()
    footprint = pip_footprint(packaging, finger)
    sweep = finger.place(four_bar(finger).sweep_driver(driver_samples_rad))
    clearance = packaging.body_clearance_mm
    link = driver_link_servo_gaps(finger, footprint, sweep.bp)
    dip = bp_pin_servo_margins(finger, packaging, sweep.b, sweep.tip, sweep.bp)
    return ServoClearance(
        mcp_servo_mm=packaging.mcp_servo_gap_mm(footprint, mcp_samples_rad) - clearance,
        pin_mm=pivot_pin_gap_mm(packaging, finger) - clearance,
        dip_pin_mm=(math.inf if packaging.allow_collision_pip_servo_to_joint_bp or not dip.size
                    else float(dip.min())),
        link_mm=(float(link.min()) if link.size else math.inf) - clearance,
    )


FIXED_MARGIN_PAIRS = (("mcp_servo", "proximal_pad"),)
"""Checked pairs whose margin is set by definition, not by the design: the
proximal pad starts the body clearance (plus its offset) past the MCP servo's
shaft disk. Still checked; left out of the reported least margin."""



def checked_pairs(packaging: LinkagePackaging) -> tuple[tuple[str, str], ...]:
    """The pad pairs that can collide (:func:`layout`'s rule): no shared body, a shared layer."""

    return tuple(pair.key for pair in layout(packaging).pairs_with(PADS))


def layer_separated_pairs(packaging: LinkagePackaging) -> tuple[tuple[str, str], ...]:
    """Pad pairs in different bodies that may cross in the side view but not across the finger."""

    return layout(packaging).separated(PADS)


def pad_spans(finger: Finger, packaging: LinkagePackaging) -> dict[str, PadSpan]:
    """Where each pad sits: the one place pad lengths are defined.

    Each pad lies on its segment (``finger.proximal``, ``finger.distal``):
    the proximal pad runs right from :attr:`Packaging.proximal_pad_start_mm`
    to where its segment ends (``A''``), the distal pad left from its offset
    short of the fingertip to where its segment starts (``B''``); each for
    its ``length_mm``, or the whole room when that is ``None`` (cut to
    ``pad_link_ratio_max`` times its link, ``|OA|`` or ``BC``, and to its
    ``max_length_mm``). The proximal pad's ``x`` from ``O``, the distal
    pad's along ``BC`` from ``B``.
    """

    proximal, distal = packaging.proximal_pad, packaging.distal_pad
    ratio, a = packaging.pad_link_ratio_max, finger.joint.a
    proximal_limit = math.inf if ratio is None else ratio * math.hypot(a.x_mm, a.z_mm)
    distal_limit = math.inf if ratio is None else ratio * finger.distal.end_mm
    if proximal.max_length_mm is not None:  # no module needs more (morphology goal)
        proximal_limit = min(proximal_limit, proximal.max_length_mm)
    if distal.max_length_mm is not None:
        distal_limit = min(distal_limit, distal.max_length_mm)
    outer = packaging.proximal_pad_start_mm
    room = finger.proximal.end_mm - outer
    length = proximal.length_mm or min(room, proximal_limit)  # a free pad is cut to the limit
    proximal_span = PadSpan(outer, outer + length, room, proximal_limit)
    outer = finger.distal.end_mm - distal.offset_mm
    room = outer - finger.distal.start_mm
    length = distal.length_mm or min(room, distal_limit)
    distal_span = PadSpan(outer - length, outer, room, distal_limit)
    return {"proximal_pad": proximal_span, "distal_pad": distal_span}


def pad_offsets(finger: Finger, packaging: LinkagePackaging) -> dict[str, float]:
    """How far palmar of its link line each pad's contact interface space
    starts (2026-11-03): on the line (0) when the pad there clears its own
    body's pin housings by the body clearance (the proximal pad ``A`` and
    ``A'``, and the PIP servo; the distal pad ``B`` and ``B'``), as the
    2R-DD's pads on theirs: a pad within its link needs no room under a pin.
    Else on the line tangent to its link's pin clearance
    (:meth:`LinkagePackaging.interface_offset_mm`), running under the pin."""

    from core.morphology.packaging.gaps import footprint_gaps
    from core.morphology.packaging.shapes import polygon_to_disk

    spans, joint, clearance = pad_spans(finger, packaging), finger.joint, packaging.body_clearance_mm
    radius = packaging.pivot_radius_mm
    proximal = pad_outline(spans["proximal_pad"], packaging.proximal_pad)[None]
    clear = min(float(polygon_to_disk(proximal, joint.a.array[None], radius("a"))[0]),
                float(polygon_to_disk(proximal, joint.ap.array[None], radius("ap"))[0]))
    if packaging.pip_servo_pad_check:
        clear = min(clear, float(footprint_gaps(proximal, pip_footprint(packaging, finger), packaging)[0]))
    b, c, bp = joint.b.array, finger.tip.array, joint.bp.array
    e = (c - b) / np.linalg.norm(c - b)
    n = np.array((-e[1], e[0]))
    bp_local = np.array((np.dot(bp - b, e), np.dot(bp - b, n)))
    distal = pad_outline(spans["distal_pad"], packaging.distal_pad)[None]
    distal_clear = min(float(polygon_to_disk(distal, np.zeros((1, 2)), radius("b"))[0]),
                       float(polygon_to_disk(distal, bp_local[None], radius("bp"))[0]))
    return {"proximal_pad": 0.0 if clear >= clearance else packaging.interface_offset_mm("proximal_pad"),
            "distal_pad": 0.0 if distal_clear >= clearance else packaging.interface_offset_mm("distal_pad")}


def proximal_pad_polygon(finger: Finger, packaging: LinkagePackaging) -> np.ndarray:
    """The proximal pad in the proximal-link frame, ``(m, 2)``: its span on
    ``O-A''``, :func:`pad_offsets` below ``OA``."""

    span = pad_spans(finger, packaging)["proximal_pad"]
    outline = pad_outline(span, packaging.proximal_pad)
    return outline + (0.0, -pad_offsets(finger, packaging)["proximal_pad"])


def distal_pad_local(finger: Finger, packaging: LinkagePackaging) -> np.ndarray:
    """The distal pad in the distal frame (``u`` from ``B`` along ``B->C``, ``v`` dorsal), ``(m, 2)``."""

    local = pad_outline(pad_spans(finger, packaging)["distal_pad"], packaging.distal_pad)
    return local - (0.0, pad_offsets(finger, packaging)["distal_pad"])


def distal_pad_polygon(finger: Finger, packaging: LinkagePackaging,
                       b: np.ndarray, c: np.ndarray) -> np.ndarray:
    """The distal pad at posed ``B`` and ``C`` (``(n, 2)``), ``(n, m, 2)``: its
    span on ``B''-C``.

    Fixed in the distal link: a point ``u`` along and ``v`` dorsal of ``BC``
    from ``B`` in the reference pose is ``B + u e + v n`` at every pose, ``e``
    the unit ``B->C`` and ``n`` it turned toward ``+Z``.
    """

    return _posed(finger, distal_pad_local(finger, packaging), b, c)


def _posed(finger: Finger, local: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    b, c = np.atleast_2d(b), np.atleast_2d(c)
    e = (c - b) / finger.distal.end_mm
    n = np.stack((-e[:, 1], e[:, 0]), axis=1)
    return b[:, None] + local[None, :, :1] * e[:, None] + local[None, :, 1:] * n[:, None]


def usable_pad_lengths(
    finger: Finger, packaging: LinkagePackaging, b: np.ndarray, bp: np.ndarray, c: np.ndarray,
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

    b, bp, c = np.atleast_2d(b), np.atleast_2d(bp), np.atleast_2d(c)
    count = len(b)
    a = np.broadcast_to((finger.joint.a.x_mm, finger.joint.a.z_mm), (count, 2))
    ap = np.broadcast_to((finger.joint.ap.x_mm, finger.joint.ap.z_mm), (count, 2))
    radius = packaging.pivot_radius_mm
    links = [_trimmed(a, bp, radius("a"), radius("bp")), _trimmed(ap, b, radius("ap"), radius("b"))]
    spans = pad_spans(finger, packaging)

    proximal = spans["proximal_pad"]  # frame: x along the mounting line, z dorsal of it
    offsets = pad_offsets(finger, packaging)
    line = np.array((0.0, -offsets["proximal_pad"]))
    near = np.full(count, np.inf)
    for start, end in links:
        low, _ = _block_extent(start - line, end - line, proximal.start_mm, proximal.end_mm,
                               packaging.proximal_pad.height_mm)
        near = np.fmin(near, low)
    proximal_free = np.where(np.isfinite(near), near - proximal.start_mm, proximal.length_mm)

    distal = spans["distal_pad"]  # frame: u along B->C from B, v dorsal
    e = (c - b) / finger.distal.end_mm
    n = np.stack((-e[:, 1], e[:, 0]), axis=1)

    top = offsets["distal_pad"]

    def local(points: np.ndarray) -> np.ndarray:  # v from the pad's top face
        rel = points - b
        return np.stack((np.sum(rel * e, axis=1), np.sum(rel * n, axis=1) + top), axis=1)

    far = np.full(count, -np.inf)
    for start, end in links:
        _, high = _block_extent(local(start), local(end), distal.start_mm, distal.end_mm,
                                packaging.distal_pad.height_mm)
        far = np.fmax(far, high)
    distal_free = np.where(np.isfinite(far), distal.end_mm - far, distal.length_mm)
    return {"proximal_pad": np.maximum(proximal_free, 0.0), "distal_pad": np.maximum(distal_free, 0.0)}


def probe_contact_lengths(finger: Finger, packaging: LinkagePackaging, b: np.ndarray, c: np.ndarray,
                          bp: np.ndarray, depth_mm: float, solids: str = "all") -> dict[str, np.ndarray]:
    """Each contact face's reachable length (:func:`probe_free_lengths`) at
    the posed ``B``, ``C`` and ``B'`` (``(n, 2)``), MCP at 0, against the
    ``solids`` in front of it, whatever their Y layer (seen from the side, an
    object meets the outer layers' links first): ``"all"``, every solid of
    the finger but its own pad; ``"pads"``, the other pad alone;
    ``"hardware"``, all but the pads (the links ``A-B'``
    and ``A'-B``, the four pins' housings, the PIP and MCP servos)."""

    b, c, bp = np.atleast_2d(b), np.atleast_2d(c), np.atleast_2d(bp)
    count = len(b)
    proximal_face, proximal_normal, distal_face, distal_normal = contact_faces(finger, packaging, b, c)
    proximal = np.broadcast_to(proximal_pad_polygon(finger, packaging), (count, *proximal_pad_polygon(finger,
                                                                                                    packaging).shape))
    distal = distal_pad_polygon(finger, packaging, b, c)
    shared_polygons, segments, disks = [], [], []
    if solids not in ("all", "pads", "hardware"):
        raise ValueError(f"solids must be all, pads or hardware, not {solids!r}")
    if solids != "pads":
        shared_polygons += [pip_footprint(packaging, finger).outline, packaging.mcp_footprint().outline]
        a = np.broadcast_to((finger.joint.a.x_mm, finger.joint.a.z_mm), (count, 2))
        ap = np.broadcast_to((finger.joint.ap.x_mm, finger.joint.ap.z_mm), (count, 2))
        segments = [np.stack((a, bp), axis=1), np.stack((ap, b), axis=1)]
        disks = [(a, packaging.pivot_radius_mm("a")), (ap, packaging.pivot_radius_mm("ap")),
                 (b, packaging.pivot_radius_mm("b")), (bp, packaging.pivot_radius_mm("bp"))]
    other = {"proximal": [] if solids == "hardware" else [distal], "distal": [] if solids == "hardware" else [proximal]}
    proximal_free = probe_free_lengths(np.broadcast_to(proximal_face, (count, 2, 2)), proximal_normal, depth_mm,
                                       [*other["proximal"], *shared_polygons], segments, disks)
    distal_free = probe_free_lengths(distal_face, distal_normal, depth_mm, [*other["distal"], *shared_polygons],
                                     segments, disks)
    return {"proximal_pad": proximal_free, "distal_pad": distal_free}


def contact_faces(finger: Finger, packaging: LinkagePackaging, b: np.ndarray, c: np.ndarray):
    """The linkage's two contact faces as :func:`fold_free_lengths` takes them:
    the proximal one fixed, the distal one at the posed ``B`` and ``C``."""

    b, c = np.atleast_2d(b), np.atleast_2d(c)
    spans = pad_spans(finger, packaging)
    proximal, distal = spans["proximal_pad"], spans["distal_pad"]
    offsets = pad_offsets(finger, packaging)
    face_z = -offsets["proximal_pad"] - packaging.proximal_pad.height_mm
    proximal_face = np.array(((proximal.start_mm, face_z), (proximal.end_mm, face_z)))
    e = (c - b) / finger.distal.end_mm
    n = np.stack((-e[:, 1], e[:, 0]), axis=1)
    face_v = -offsets["distal_pad"] - packaging.distal_pad.height_mm  # dorsal of BC
    outer = b + distal.end_mm * e + face_v * n
    inner = b + distal.start_mm * e + face_v * n
    return proximal_face, np.array((0.0, -1.0)), np.stack((outer, inner), axis=1), -n


def face_crossing_lengths(
    finger: Finger, packaging: LinkagePackaging, b: np.ndarray, c: np.ndarray,
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

    b, c = np.atleast_2d(b), np.atleast_2d(c)
    spans = pad_spans(finger, packaging)
    proximal, distal = spans["proximal_pad"], spans["distal_pad"]
    offsets = pad_offsets(finger, packaging)
    face_z = -offsets["proximal_pad"] - packaging.proximal_pad.height_mm
    face_v = -offsets["distal_pad"] - packaging.distal_pad.height_mm  # dorsal of BC
    e = (c - b) / finger.distal.end_mm
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
    finger: Finger,
    packaging: LinkagePackaging,
    b: np.ndarray,
    c: np.ndarray,
    bp: np.ndarray,
    mcp_samples_rad: Sequence[float] = (0.0,),
) -> dict[tuple[str, str], np.ndarray]:
    """Per-sample margin of every pair the rule checks between a pad and a
    part of another body, beyond its clearance.

    Samples are four-bar poses in the proximal-link frame, given by the posed
    ``B``, ``C`` and ``B'`` (``(n, 2)``). The MCP servo turns by each MCP
    flexion sample about ``O``; its margin is the worst over them. Margins
    are in mm, clear when >= 0.
    """

    built = assembly(finger, packaging)
    return margins(built, poses(finger, b, c, bp, mcp_samples_rad), packaging, built.layout.pairs_with(PADS))


def pip_servo_pad_gap_mm(finger: Finger, packaging: LinkagePackaging) -> float:
    """Gap between the PIP servo body and the proximal pad (both fixed in the
    proximal frame), signed as ``packaging.gaps.footprint_gaps``."""

    found = margins(assembly(finger, packaging), reference_poses(finger), packaging,
                    [Pair("pip_servo", "proximal_pad", "touch")])
    return float(found[("pip_servo", "proximal_pad")][0])


def pivot_pin_gap_mm(packaging: LinkagePackaging, finger: Finger) -> float:
    """Gap between ``A``'s pin envelope and the virtual clearance disk around
    the PIP servo's shaft ``A'``: ``|AA'|`` less both radii."""

    return (math.hypot(finger.joint.a.x_mm - finger.joint.ap.x_mm, finger.joint.a.z_mm - finger.joint.ap.z_mm)
            - packaging.pivot_radius_mm("a")
            - packaging.ap_clearance_diameter_mm / 2.0)


def pip_footprint(packaging: LinkagePackaging, finger: Finger) -> ServoFootprint:
    """The PIP servo on ``A'``, its rear body along ``-X`` tilted dorsally
    (``finger.servo``: how it is mounted)."""

    tilt = finger.servo.tilt_rad
    return ServoFootprint(
        (finger.joint.ap.x_mm, finger.joint.ap.z_mm),
        (-math.cos(tilt), math.sin(tilt)),
        packaging.pip_servo.radius_mm,
        packaging.pip_servo.rear_extent_mm,
    )


__all__ = [
    "pad_offsets",
    "COMPONENTS",
    "LinkagePackaging",
    "PIVOTS",
    "DEFAULT_DRIVER_SAMPLES_RAD",
    "FIXED_MARGIN_PAIRS",
    "JOINTS",
    "LAYERS",
    "PINS",
    "PIN_CHECKED",
    "PADS",
    "PARTS",
    "ServoClearance",
    "a_pin_servo_gap_mm",
    "assembly",
    "bp_pin_servo_margins",
    "checked_pairs",
    "contact_faces",
    "distal_pad_local",
    "distal_pad_polygon",
    "driver_link_servo_gaps",
    "face_crossing_lengths",
    "gap_mm",
    "layer_separated_pairs",
    "layout",
    "least_margin_mm",
    "link_capsule_margins",
    "own_pin_gaps",
    "pad_margins",
    "pad_spans",
    "pin_overlap_mm",
    "pip_footprint",
    "pip_servo_pad_gap_mm",
    "pivot_pin_gap_mm",
    "poses",
    "probe_contact_lengths",
    "proximal_pad_polygon",
    "reference_poses",
    "servo_clearance",
    "usable_pad_lengths",
]
