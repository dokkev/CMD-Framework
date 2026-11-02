"""What a linkage finger measures in its extended (reference) pose: its
length, where it bends, how far its pads run past their bodies, their
lengths. (The gaps its pins, servos and pads keep are its packaging's:
``core.morphology.mechanism.linkage.packaging``.)

Lengths in mm, in the proximal frame (``+X`` along ``O-A``, ``+Z`` dorsal).
Each is a measurement; what it must be is the study's requirement
(``core.optimization.linkage``).
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from core.morphology.finger import Finger
from core.morphology.mechanism.linkage.model import bar_crossings, four_bar
from core.morphology.mechanism.linkage.packaging import LinkagePackaging, pad_spans, pip_footprint


def finger_length_mm(finger: Finger) -> float:
    """``|OC|`` in the extended pose, ``C = (B_x + BC, B_z)``."""

    return math.hypot(finger.tip.x_mm, finger.tip.z_mm)


def symmetric_bp(a: np.ndarray, ap: np.ndarray, b: np.ndarray) -> np.ndarray | None:
    """``B'`` making ``A'-B-B'-A`` an antiparallelogram (``|B'B| = |A'A|``,
    ``|AB'| = |A'B|``, the bars crossed), ``None`` when the circles miss: of
    the two crossings, the one away from the parallelogram's ``B + A - A'``."""

    g, r = float(np.linalg.norm(a - ap)), float(np.linalg.norm(b - ap))
    d = float(np.linalg.norm(b - a))
    if not abs(r - g) < d < r + g:
        return None
    along = (r * r - g * g + d * d) / (2.0 * d)  # from A toward B
    unit = (b - a) / d
    off = math.sqrt(max(r * r - along * along, 0.0)) * np.array((-unit[1], unit[0]))
    parallel = b + a - ap
    return max((a + along * unit + off, a + along * unit - off), key=lambda p: float(np.linalg.norm(p - parallel)))


def extended_icr_x(a: Sequence[float], ap: Sequence[float], b: Sequence[float], bp: Sequence[float]) -> float | None:
    """``x`` of the extended pose's centre of rotation: ``A-B'`` crossing
    ``A'-B`` (``None``: the bars parallel)."""

    x = bar_crossings(*(np.asarray(point, dtype=float) for point in (a, ap, b, bp)))[0]
    return None if math.isnan(x) else float(x)


def linkage_icr_x(finger: Finger) -> float | None:
    """:func:`extended_icr_x` of a linkage."""

    icr = four_bar(finger).icr
    return None if icr is None else icr.x_mm


def overhang_mm(finger: Finger, packaging: LinkagePackaging) -> tuple[float, float]:
    """How far each pad runs past its own body in the extended pose, along
    the finger: the proximal pad's ``A''`` past the proximal body's front
    (``A``'s housing, the PIP servo's outline), the distal pad's ``B''``
    behind the distal body's back (``B``'s and ``B'``'s housings); negative
    when it stops short."""

    radius = packaging.pivot_radius_mm
    front = max(finger.joint.a.x_mm + radius("a"), float(pip_footprint(packaging, finger).outline[:, 0].max()))
    back = min(finger.joint.b.x_mm - radius("b"), finger.joint.bp.x_mm - radius("bp"))
    return finger.proximal_end.x_mm - front, back - finger.distal_start.x_mm


def pad_lengths(finger: Finger, packaging: LinkagePackaging) -> dict[str, float]:
    """Each pad's mounting length (``*_support_length_mm``) and its support's
    room (``*_room_mm``, the longest module it can carry)."""

    spans = pad_spans(finger, packaging)
    values = {}
    for side in ("proximal", "distal"):
        length = spans[f"{side}_pad"].length_mm
        values[f"{side}_support_length_mm"] = length
        values[f"{side}_room_mm"] = spans[f"{side}_pad"].room_mm
    return values


__all__ = [
    "extended_icr_x",
    "finger_length_mm",
    "linkage_icr_x",
    "overhang_mm",
    "pad_lengths",
    "symmetric_bp",
]
