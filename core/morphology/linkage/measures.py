"""What a linkage finger measures in its extended (reference) pose: its
length, where it bends, how far its pads run past their bodies, their
lengths. (The gaps its pins, servos and pads keep are its packaging's:
``core.morphology.linkage.packaging``.)

Lengths in mm, in the proximal frame (``+X`` along ``O-A``, ``+Z`` dorsal).
Each is a measurement; what it must be is the study's requirement
(``core.optimization.linkage``).
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from core.morphology.linkage.model import FingerLinkage
from core.morphology.linkage.packaging import LinkagePackaging, pad_chamfer_mm, pad_spans, pip_footprint
from core.morphology.linkage.parameters import FingerLinkageParameters


def finger_length_mm(parameters: FingerLinkageParameters) -> float:
    """``|OC|`` in the extended pose, ``C = (B_x + BC, B_z)``."""

    return math.hypot(parameters.b_x_mm + parameters.bc_length_mm, parameters.b_z_mm)


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

    a, ap, b, bp = (np.asarray(point, dtype=float) for point in (a, ap, b, bp))
    first, second = bp - a, b - ap
    det = first[0] * -second[1] - first[1] * -second[0]
    if abs(det) < 1e-9:
        return None
    gap = ap - a
    t = (gap[0] * -second[1] - gap[1] * -second[0]) / det
    return float(a[0] + t * first[0])


def linkage_icr_x(linkage: FingerLinkage) -> float | None:
    """:func:`extended_icr_x` of a linkage."""

    return extended_icr_x(*((point.x_mm, point.z_mm) for point in (linkage.a, linkage.ap, linkage.b, linkage.bp)))


def overhang_mm(linkage: FingerLinkage, packaging: LinkagePackaging) -> tuple[float, float]:
    """How far each pad runs past its own body in the extended pose, along
    the finger: the proximal pad's ``A''`` past the proximal body's front
    (``A``'s housing, the PIP servo's outline), the distal pad's ``B''``
    behind the distal body's back (``B``'s and ``B'``'s housings); negative
    when it stops short."""

    p = linkage.parameters
    radius = packaging.pivot_radius_mm
    front = max(p.a_x_mm + radius("a"), float(pip_footprint(packaging, linkage).outline[:, 0].max()))
    back = min(p.b_x_mm - radius("b"), p.bp.x_mm - radius("bp"))
    return p.app_x_mm - front, back - p.bpp_x_mm


def pad_lengths(parameters: FingerLinkageParameters, packaging: LinkagePackaging) -> dict[str, float]:
    """Each pad's mounting length (``*_support_length_mm``), its support's
    room (``*_room_mm``, the longest module it can carry) and its face, the
    mounting length less the chamfer (``*_face_mm``)."""

    spans = pad_spans(parameters, packaging)
    values = {}
    for side in ("proximal", "distal"):
        length = spans[f"{side}_pad"].length_mm
        chamfer = pad_chamfer_mm(getattr(packaging, f"{side}_pad"), parameters, side)
        values[f"{side}_support_length_mm"] = length
        values[f"{side}_room_mm"] = spans[f"{side}_pad"].room_mm
        values[f"{side}_face_mm"] = max(length - min(chamfer, length), 0.0)
    return values


__all__ = [
    "extended_icr_x",
    "finger_length_mm",
    "linkage_icr_x",
    "overhang_mm",
    "pad_lengths",
    "symmetric_bp",
]
