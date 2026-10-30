"""The hardware a finger is packaged with, whatever its mechanism: the servos
(the MCP servo's shaft on ``O``, the finger's own servo), the pin housings,
the contact pads and the clearances between them (:class:`Packaging`).

Each servo's section in the finger's plane is a D shape
(:class:`~core.morphology.packaging.shapes.ServoFootprint`): a disk of the
actuator radius around the shaft and a rectangle reaching ``rear_extent_mm``
back from it. Another servo or bearing is a different ``Packaging``. All
checks are planar, in the proximal segment's frame (``+X`` along it, ``+Z``
dorsal); margins are in mm, feasible when >= 0. Where a mechanism puts its
servo, pins and pads is its own (``core.morphology.linkage.packaging``,
``core.morphology.revolute``, ...).

**Contact pads.** Each pad is a block ``a-b-d-c`` fixed to its segment and
free to stand out of it: ``a-b`` on the segment's mounting line, ``c-d`` its
height palmar of it (:class:`ContactPad`). The proximal pad starts at
:attr:`Packaging.proximal_pad_start_mm` (``O``, past the MCP servo's shaft
disk by the body clearance when ``pad_mcp_check``, plus its offset); the
distal pad starts its offset short of the fingertip. Without
``pad_mcp_check`` the MCP servo is not checked.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from core.hardware import DShapedExtrusion, XM335Actuator
from core.morphology.packaging.shapes import ServoFootprint, separation, turn_points

PIVOT_CLEARANCE_DIAMETER_MM = 8.0
"""Diameter of the metal around each pivot pin (``A``, ``A'``, ``B``, ``B'``) in
the side view, which other parts keep the body clearance from.
The pins run in McMaster 57155K424 flanged ball bearings (trade no. 63: 3 mm
shaft, 6 mm OD, 2 mm wide, 7.2 x 0.6 mm flange), one on each side of a joint,
pressed into 1 mm aluminium walls: the 8 mm housing is wider than the flange."""


BODY_CLEARANCE_MM = 1.0
"""Servo to servo, servo to pin envelope, servo to moving parts, and the
pads to the parts of other bodies."""


MARGIN_TOLERANCE_MM = 1e-9




def _xm335() -> DShapedExtrusion:
    return XM335Actuator().geometry


@dataclass(frozen=True)
class ContactPad:
    """One contact pad block (provisional values).

    ``height_mm`` palmar of its mounting line, starting ``offset_mm`` in from
    its outer end (``O`` or the MCP servo, or ``C``), and a 45 deg chamfer of
    ``chamfer_mm`` at its palmar corner facing the PIP (with ``chamfer_free``
    the design's own, 0 to ``height_mm``: ``core.morphology.linkage.packaging.pad_chamfer_mm``). ``length_mm`` fixes
    its length, which the mounting line must have room for; ``None`` runs it
    to ``A''`` or ``B''``, the whole room (``core.morphology.linkage.packaging.pad_spans``), cut to
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
    """Hardware dimensions of a finger's planar packaging, whatever its mechanism.

    The finger's own servo (``pip_servo``) and the MCP servo, the pin
    housings' default size, the body clearance, the two contact pads and the
    distal pad's carrier, and which pairs every mechanism checks
    (``pad_pad_check``, ``pip_servo_pad_check``, ``a_pin_servo_check``, the
    pads against the MCP servo). ``mcp_servo_rear`` is the unit direction of
    the MCP servo's rear body in the proximal frame at zero MCP flexion
    (``+Z``: standing dorsally). A mechanism's own placement (its pins' sizes,
    where its servo sits) extends it (``core.morphology.linkage.LinkagePackaging``).
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
    pad_mcp_check: bool = True
    pad_mcp_clearance_mm: float | None = None
    """Least gap of the pads to the MCP servo, which also places the proximal
    pad's start; ``None``: ``body_clearance_mm``."""
    pip_servo_pad_check: bool = False
    """The PIP servo body keeps clear of the proximal pad (one body, still
    kept apart): it may come down to the pad's top face."""
    a_pin_servo_check: bool = False
    """The ``A`` pin also keeps clear of the PIP servo body (it hangs off the
    servo on a bracket otherwise)."""
    pad_pad_check: bool = True
    """The two pads keep clear of each other."""
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
        if self.pad_mcp_clearance_mm is not None and not self.pad_mcp_clearance_mm >= 0.0:
            raise ValueError("pad_mcp_clearance_mm must be non-negative, or None")
        if not self.distal_carrier_height_mm >= 0.0:
            raise ValueError("distal_carrier_height_mm must be non-negative")
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
        """Radius of a pivot's pin housing in the side view (any pivot: the
        packaging's; a mechanism's own packaging may size each)."""

        return self.pivot_clearance_diameter_mm / 2.0

    @property
    def proximal_pad_start_mm(self) -> float:
        """``x`` of the proximal pad's first edge ``a-c``: ``O``, or the MCP servo's
        shaft disk plus the body clearance with ``pad_mcp_check``, plus the pad's
        offset."""

        clear = self.mcp_servo.radius_mm + self.mcp_pad_clearance_mm if self.pad_mcp_check else 0.0
        return clear + self.proximal_pad.offset_mm

    @property
    def mcp_pad_clearance_mm(self) -> float:
        """The pads' least gap to the MCP servo (``pad_mcp_clearance_mm``)."""

        return self.body_clearance_mm if self.pad_mcp_clearance_mm is None else self.pad_mcp_clearance_mm

    def pad_checks_mcp(self, pad: str) -> bool:
        """Whether a pad (``"proximal_pad"``, ``"distal_pad"``) is checked against the MCP servo."""

        if pad == "distal_pad" and self.distal_pad_mcp_check is not None:
            return self.distal_pad_mcp_check
        return self.pad_mcp_check

    def mcp_footprint(self, mcp_flexion_rad: float = 0.0) -> ServoFootprint:
        """The MCP servo on ``O`` as seen from the proximal link at an MCP flexion.

        MCP flexion ``q`` turns the hand-fixed MCP servo by ``+q`` about ``O``
        in the proximal-link frame.
        """

        footprint = ServoFootprint(
            (0.0, 0.0), self.mcp_servo_rear, self.mcp_servo.radius_mm, self.mcp_servo.rear_extent_mm
        )
        return footprint.turned(mcp_flexion_rad)

    def mcp_servo_gap_mm(self, pip_servo: ServoFootprint, mcp_samples_rad: Sequence[float]) -> float:
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
        if self.overlap_depth and gap <= MARGIN_TOLERANCE_MM:
            depth = float(separation(np.stack(pip_edges), np.broadcast_to(
                mcp_outline, (len(pip_edges), *mcp_outline.shape))).min())
            gap = min(gap, depth) if depth < 0.0 else gap
        return gap


PIN_RADIUS_MM = Packaging().pin_radius_mm


PIN_SPACING_MM = Packaging().pin_spacing_mm


__all__ = [
    "BODY_CLEARANCE_MM",
    "ContactPad",
    "MARGIN_TOLERANCE_MM",
    "PIN_RADIUS_MM",
    "PIN_SPACING_MM",
    "PIVOT_CLEARANCE_DIAMETER_MM",
    "Packaging",
]
