"""Design vector of the 2-DoF finger study: A_x, B (x, z), A', B', the PIP
servo's dorsal tilt (degrees, on the same 0.1 grid), BC (the middle + distal
phalanx), A_z, ``A''_x - A'_x``, ``B'_x - B''_x`` and the two pads' chamfers
(the last five added later, so older vectors read with them 0:
``DEFAULT_VALUES``; a chamfer counts only where the pad's chamfer is free).

With a machining resolution every part keeps its own dimensions on the grid:
the proximal link ``A`` and ``A'`` from ``O``, the distal link ``B'`` from ``B``
(and ``BC``), and the separate bars ``A'-B`` and ``A-B'`` their lengths.
``decode`` rounds the two bar lengths measured on the design vector, then
re-solves the assembled ``B`` (in the extended pose) from them; ``B`` moves by
about the resolution and is no longer on the grid, as it is not machined.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

import numpy as np

from core.morphology.linkage import FingerLinkage, FingerLinkageParameters, PointXZ


VARIABLE_NAMES: tuple[str, ...] = (
    "a_x_mm",
    "b_x_mm",
    "b_z_mm",
    "ap_x_mm",
    "ap_z_mm",
    "bp_x_mm",
    "bp_z_mm",
    "servo_tilt_deg",
    "bc_length_mm",
    "a_z_mm",
    "app_offset_mm",
    "bpp_offset_mm",
    "proximal_chamfer_mm",
    "distal_chamfer_mm",
)
DEFAULT_VALUES = {"a_z_mm": 0.0, "app_offset_mm": 0.0, "bpp_offset_mm": 0.0,
                  "proximal_chamfer_mm": 0.0, "distal_chamfer_mm": 0.0}
"""Values of variables added later, for design vectors written before them."""


def decode(
    x: Sequence[float], link_length_resolution_mm: float | None = None
) -> FingerLinkageParameters:
    """Map an ordered design vector to linkage parameters.

    With ``link_length_resolution_mm``, ``|A'B|`` and ``|AB'|`` are rounded to
    multiples of it and ``B`` is re-solved (see the module docstring);
    ``ValueError`` when the rounded bars no longer close.
    """

    missing = VARIABLE_NAMES[len(x):]
    if len(x) > len(VARIABLE_NAMES) or any(name not in DEFAULT_VALUES for name in missing):
        raise ValueError(f"design vector has {len(x)} values; expected {len(VARIABLE_NAMES)}")
    x = (*x, *(DEFAULT_VALUES[name] for name in missing))  # vectors from before a variable
    (a_x, b_x, b_z, ap_x, ap_z, bp_x, bp_z, tilt_deg, bc_length_mm, a_z, app_offset, bpp_offset,
     proximal_chamfer, distal_chamfer) = (float(v) for v in x)
    ap, bp = PointXZ(ap_x, ap_z), PointXZ(bp_x, bp_z)
    if link_length_resolution_mm is not None:
        b_x, b_z, bp = _assemble((a_x, a_z), (b_x, b_z), ap, bp, link_length_resolution_mm)
    return FingerLinkageParameters(
        a_x_mm=a_x,
        a_z_mm=a_z,
        b_x_mm=b_x,
        b_z_mm=b_z,
        ap=ap,
        bp=bp,
        bc_length_mm=bc_length_mm,
        pip_servo_tilt_rad=math.radians(tilt_deg),
        app_offset_mm=app_offset,
        bpp_offset_mm=bpp_offset,
        proximal_chamfer_mm=proximal_chamfer,
        distal_chamfer_mm=distal_chamfer,
    )


def _assemble(
    a: tuple[float, float], b: tuple[float, float], ap: PointXZ, bp: PointXZ, resolution_mm: float
) -> tuple[float, float, PointXZ]:
    """Return ``B`` and ``B'`` assembled from grid bars, ``B`` nearest the design's."""

    def on_grid(value: float) -> float:
        return round(value / resolution_mm) * resolution_mm

    b = np.array(b)
    offset = np.array((on_grid(bp.x_mm - b[0]), on_grid(bp.z_mm - b[1])))  # B' from B
    driven = on_grid(math.dist((ap.x_mm, ap.z_mm), b))                    # |A'B|
    driver = on_grid(math.dist(a, b + offset))                             # |AB'|
    # B lies on the circle about A' (radius |A'B|) and on the one about A - (B' - B).
    c1, c2 = np.array((ap.x_mm, ap.z_mm)), np.array(a) - offset
    gap = float(np.linalg.norm(c2 - c1))
    along = (driven**2 - driver**2 + gap**2) / (2.0 * gap) if gap > 0.0 else math.nan
    height2 = driven**2 - along**2
    if not height2 >= 0.0:
        raise ValueError("the rounded bars A'-B and A-B' do not close")
    u = (c2 - c1) / gap
    n = np.array((-u[1], u[0]))
    candidates = [c1 + along * u + sign * math.sqrt(height2) * n for sign in (1.0, -1.0)]
    nearest = min(candidates, key=lambda point: float(np.linalg.norm(point - b)))
    built_bp = nearest + offset
    return float(nearest[0]), float(nearest[1]), PointXZ(float(built_bp[0]), float(built_bp[1]))


def encode(linkage: FingerLinkage) -> tuple[float, ...]:
    """Return the design vector of a linkage."""

    p = linkage.parameters
    return (p.a_x_mm, p.b_x_mm, p.b_z_mm, p.ap.x_mm, p.ap.z_mm, p.bp.x_mm, p.bp.z_mm,
            round(math.degrees(p.pip_servo_tilt_rad), 9), p.bc_length_mm, p.a_z_mm, p.app_offset_mm,
            p.bpp_offset_mm, p.proximal_chamfer_mm, p.distal_chamfer_mm)


def read_design(row: Mapping[str, str]) -> list[float]:
    """Design vector of a CSV row, filling variables the row predates."""

    return [float(row[name]) if name in row else DEFAULT_VALUES[name] for name in VARIABLE_NAMES]


__all__ = ["DEFAULT_VALUES", "VARIABLE_NAMES", "decode", "encode", "read_design"]
