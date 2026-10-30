"""The rolling contact joint (RCJ) finger: its design vector, solids and their gaps.

The MCP is at ``O``. Two rollers of one radius ``r`` roll on each other
without slip: the proximal one centred on ``A = (|OA|, 0)``, fixed to the
proximal segment, the distal one on ``B``, fixed to the distal segment
``B-C``. The servo's shaft is on ``A`` and turns the carrier ``A-B``
(``|AB| = 2r``) by ``theta``; the distal segment then turns by
``phi = 2 theta`` relative to the proximal one (the same ``phi`` as the
linkage's ``q_pip + q_dip`` and the 2R's), so ``k = 2`` throughout
(``core.morphology.rcj``'s kinematics). Ideal no-slip rolling: the crossed
cables that hold it are not modelled (provisional, as the retention study's
RCJ).

Packaging, in one plane, by the linkage's rules: the servo lies back along
``-X`` from ``A`` (``SERVO_REAR``; its rounded front within the roller); the proximal pad hangs a
searched drop below ``O-A`` (at most ``r``), laid from the MCP servo to
``A''``, at most ``A_x + r`` (plus ``goal.pads.proximal.overhang_max_mm``);
the distal pad hangs a searched drop below ``B-C`` (at most ``r``), laid
from ``C`` to ``B''`` (behind ``B`` at most the distal overhang past the
roller). Each pad clears its own roller (statically: each is fixed to its
roller). As the linkage's and the 2R's: each face counts less what the
other hides in the fold, the distal pad's carrier is checked against the
servo, and the joint hyperextends with every pair clear. The carrier and the links holding
the distal pad are in other layers, so they are not checked. No clearance,
overlaps by depth; checked over ``[0, 180 deg]`` from extension: the distal
pad against the proximal pad, the proximal roller, the servo and the MCP
servo (MCP flexion up to its max), the distal roller against the proximal
pad; statically the proximal pad against the servo and the MCP servo, and
the two servos. A pad's effective length is its face.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from core.morphology.packaging.gaps import footprint_gaps, mcp_servo_gaps, polygon_gaps, proximal_pad_mcp_gap
from core.morphology.packaging.hardware import Packaging
from core.morphology.packaging.shapes import PadSpan, ServoFootprint, pad_outline, polygon_to_disk
from core.morphology.revolute.solids import carrier_local


VARIABLE_NAMES = ("a_x_mm", "roller_radius_mm", "bc_length_mm", "app_x_mm", "bpp_from_b_mm", "distal_drop_mm",
                  "proximal_drop_mm")
"""``|OA|``, the rollers' radius ``r`` (``|AB| = 2r``), ``BC``, the proximal
pad's end ``A''_x``, the distal pad's joint end ``B''`` from ``B`` along
``B->C`` (negative: behind ``B``), and how far below ``B-C`` and ``O-A``
the distal and the proximal pad's top faces lie (each at most ``r``, the
joint's radius, as the 2R's; each pad clears its own roller). The pads are
plain blocks."""
SERVO_REAR = (-1.0, 0.0)
"""The servo on ``A`` lies back along ``-X``, untilted, as the 2R's: within
the roller's depth (``r`` is at least its radius), so no thicker than the
rollers; standing dorsal it would only add height, and it never meets a pad
either way (the pads hang a roller radius below ``O-A``)."""
TORQUE_RATIO = 2.0
"""``d(phi)/d(theta)`` of equal rollers: two units of servo torque per unit of distal moment."""


def finger_length_mm(a_x_mm: float, roller_radius_mm: float, bc_length_mm: float) -> float:
    """``|OC|`` extended: ``O-A``, the two radii to ``B``, then ``BC``."""

    return a_x_mm + 2.0 * roller_radius_mm + bc_length_mm


@dataclass(frozen=True)
class RcjGeometry:
    """A design's solids in the reference pose: the proximal pad and the servo
    in the proximal frame, the distal pad in the distal frame (``u`` from
    ``B`` along ``B->C``, ``v`` dorsal)."""

    a_x_mm: float
    roller_radius_mm: float
    bc_length_mm: float
    proximal_pad: np.ndarray
    distal_pad_local: np.ndarray
    distal_drop_mm: float
    servo: ServoFootprint
    proximal_length_mm: float
    distal_length_mm: float
    proximal_face_mm: float
    distal_face_mm: float
    proximal_room_mm: float
    distal_room_mm: float
    proximal_drop_mm: float = 0.0
    distal_carrier_local: np.ndarray | None = None
    """The distal pad's carrier plate in the distal frame (``None``: none)."""

    @property
    def finger_length_mm(self) -> float:
        return finger_length_mm(self.a_x_mm, self.roller_radius_mm, self.bc_length_mm)

    @property
    def a(self) -> np.ndarray:
        return np.array((self.a_x_mm, 0.0))

    def b(self, phi_rad: np.ndarray) -> np.ndarray:
        """The distal roller's centre at distal flexions ``phi``, ``(n, 2)``: the carrier turned by ``phi / 2``."""

        theta = np.atleast_1d(np.asarray(phi_rad, dtype=float)) / 2.0
        return self.a + 2.0 * self.roller_radius_mm * np.stack((np.cos(theta), -np.sin(theta)), axis=1)

    def posed(self, local: np.ndarray, phi_rad: np.ndarray) -> np.ndarray:
        """Points ``(m, 2)`` of the distal frame at ``phi``, ``(n, m, 2)`` in the proximal frame."""

        phi = np.atleast_1d(np.asarray(phi_rad, dtype=float))
        e = np.stack((np.cos(phi), -np.sin(phi)), axis=1)
        n = np.stack((-e[:, 1], e[:, 0]), axis=1)
        local = np.asarray(local, dtype=float)
        return self.b(phi)[:, None] + local[None, :, :1] * e[:, None] + local[None, :, 1:] * n[:, None]

    def distal_pad(self, phi_rad: np.ndarray) -> np.ndarray:
        return self.posed(self.distal_pad_local, phi_rad)

    @property
    def distal_reach_mm(self) -> float:
        """How fast a distal point moves per radian of ``phi``, at most: ``B``
        at ``r`` (half the carrier's rate on ``2r``), plus the pad's farthest
        point from ``B`` turning at the full rate."""

        solids = self.distal_pad_local if self.distal_carrier_local is None else np.vstack(
            (self.distal_pad_local, self.distal_carrier_local))
        return self.roller_radius_mm + float(np.hypot(solids[:, 0], solids[:, 1]).max())

    @property
    def distal_mount(self) -> np.ndarray:
        """The distal pad's mounting line (its top face), ``B''`` then ``C``'s end, in the distal frame."""

        top = self.distal_pad_local[:, 1].max()
        return np.array(((self.distal_pad_local[:, 0].min(), top), (self.distal_pad_local[:, 0].max(), top)))

    def tip(self, phi_rad: np.ndarray) -> np.ndarray:
        """``C`` at distal flexions ``phi``, ``(n, 2)``."""

        return self.posed(np.array(((self.bc_length_mm, 0.0),)), phi_rad)[:, 0]


def geometry(x: Sequence[float], packaging: Packaging) -> RcjGeometry:
    """The solids of a design vector."""

    v = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    proximal, distal = packaging.proximal_pad, packaging.distal_pad
    r = v["roller_radius_mm"]
    start = packaging.proximal_pad_start_mm
    proximal_room = v["app_x_mm"] - start
    distal_room = v["bc_length_mm"] - distal.offset_mm - v["bpp_from_b_mm"]
    proximal_length = min(proximal_room, proximal.max_length_mm or math.inf)
    distal_length = min(distal_room, distal.max_length_mm or math.inf)
    chamfer = {side: pad.chamfer_mm for side, pad in (("proximal", proximal), ("distal", distal))}  # 0: blocks
    outer = v["bc_length_mm"] - distal.offset_mm
    proximal_outline = pad_outline(PadSpan(start, start + max(proximal_length, 1e-6), proximal_room),
                                   proximal, "end", chamfer["proximal"]) - (0.0, v["proximal_drop_mm"])
    distal_span = PadSpan(outer - max(distal_length, 1e-6), outer, distal_room)
    distal_outline = pad_outline(distal_span, distal, "start", chamfer["distal"]) - (0.0, v["distal_drop_mm"])
    servo = ServoFootprint((v["a_x_mm"], 0.0), SERVO_REAR,
                           packaging.pip_servo.radius_mm, packaging.pip_servo.rear_extent_mm)
    face = lambda length, k: max(length - min(k, max(length, 0.0)), 0.0)  # noqa: E731
    return RcjGeometry(v["a_x_mm"], r, v["bc_length_mm"], proximal_outline, distal_outline, v["distal_drop_mm"],
                       servo, proximal_length, distal_length, face(proximal_length, chamfer["proximal"]),
                       face(distal_length, chamfer["distal"]), proximal_room, distal_room, v["proximal_drop_mm"],
                       carrier_local(distal_outline, v["distal_drop_mm"], packaging))


def margins(geo: RcjGeometry, phi: np.ndarray, packaging: Packaging, mcp_samples_rad: Sequence[float],
            until_unsafe: bool = False) -> dict[tuple[str, str], np.ndarray]:
    """Per-sample margin of every checked pair beyond the clearance, the pads
    against the MCP servo at ``mcp_samples_rad`` (``until_unsafe`` as the
    2R's: the MCP servo pair only up to the first sample another pair fails at)."""

    clearance = packaging.body_clearance_mm
    count = len(phi)
    r = geo.roller_radius_mm
    distal = geo.distal_pad(phi)
    proximal = np.broadcast_to(geo.proximal_pad, (count, *geo.proximal_pad.shape))
    margins = {
        ("distal_pad", "pip_servo"): footprint_gaps(distal, geo.servo, packaging) - clearance,
        **({} if geo.distal_carrier_local is None else {
            ("distal_carrier", "pip_servo"): footprint_gaps(geo.posed(geo.distal_carrier_local, phi), geo.servo,
                                                            packaging) - clearance}),
        ("distal_pad", "proximal_roller"): polygon_to_disk(distal, np.broadcast_to(geo.a, (count, 2)), r) - clearance,
        ("distal_roller", "proximal_pad"): polygon_to_disk(proximal, geo.b(phi), r) - clearance,
    }
    if packaging.pad_pad_check:
        margins[("distal_pad", "proximal_pad")] = polygon_gaps(distal, proximal, packaging) - clearance
    if packaging.pad_checks_mcp("distal_pad"):
        end = count
        if until_unsafe:
            failing = np.flatnonzero(np.min(np.stack(list(margins.values())), axis=0) < -1e-9)
            end = int(failing[0]) + 1 if failing.size else count
        margin = np.full(count, np.inf)
        margin[:end] = mcp_servo_gaps(distal[:end], packaging, mcp_samples_rad) - packaging.mcp_pad_clearance_mm
        margins[("distal_pad", "mcp_servo")] = margin
    if packaging.pad_checks_mcp("proximal_pad"):  # fixed in the proximal frame
        gap = proximal_pad_mcp_gap(geo.proximal_pad, packaging, mcp_samples_rad)
        margins[("mcp_servo", "proximal_pad")] = np.full(count, gap - packaging.mcp_pad_clearance_mm)
    return {pair: np.where(np.abs(margin) < 1e-9, 0.0, margin) for pair, margin in margins.items()}


__all__ = ["SERVO_REAR", "TORQUE_RATIO", "VARIABLE_NAMES", "RcjGeometry", "finger_length_mm", "geometry", "margins"]
