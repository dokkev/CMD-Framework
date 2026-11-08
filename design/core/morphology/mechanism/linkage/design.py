"""Design vector of the 2-DoF finger linkage: A_x, B (x, z), A', B', the PIP
servo's dorsal tilt (degrees, on the same 0.1 grid), BC (the middle + distal
phalanx), A_z, ``A''_x - A'_x`` and ``B'_x - B''_x``, and each pad's
mounting offset below its link line (``Finger.pad_offsets_mm``, 2026-11-04)
(the last five added later, so older vectors read with them at
``DEFAULT_VALUES``: 0). Vectors written with the pads' drops in those two
last places (2026-10-28 to 2026-11-03) read them as the offsets; with the
pads' chamfers too (16 values) without either.

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

from core._validation import require_positive
from core.morphology.finger import Finger, PointXZ, Segment, ServoMount
from core.morphology.mechanism.linkage.model import FourBar


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
    "proximal_mount_offset_mm",
    "distal_mount_offset_mm",
)
DEFAULT_VALUES = {"a_z_mm": 0.0, "app_offset_mm": 0.0, "bpp_offset_mm": 0.0, "proximal_mount_offset_mm": 0.0,
                  "distal_mount_offset_mm": 0.0}
"""Values of variables added later, for design vectors written before them."""
LEGACY_DROPS_LENGTH = 14
"""A vector with the pads' drops after the offsets (2026-10-28 to 2026-11-03): read as the mounting
offsets (2026-11-04: the same place, each pad's mounting surface below its link line)."""
LEGACY_CHAMFERED_LENGTH = 16
"""A vector with the pads' chamfers after the offsets, then the drops, which ``decode`` leaves out."""


def decode(
    x: Sequence[float], link_length_resolution_mm: float | None = None
) -> Finger:
    """Map an ordered design vector to the finger: its four-bar, its segments
    (the proximal one to ``A'' = A'_x + app_offset``, the distal one from
    ``B'' = B'_x - bpp_offset`` to ``C``, ``BC`` past ``B``) and the servo's tilt.

    With ``link_length_resolution_mm``, ``|A'B|`` and ``|AB'|`` are rounded to
    multiples of it and ``B`` is re-solved (see the module docstring);
    ``ValueError`` when the rounded bars no longer close.
    """

    if len(x) == LEGACY_CHAMFERED_LENGTH:
        x = x[:12]  # the deprecated chamfers and drops, after the offsets
    if not 9 <= len(x) <= len(VARIABLE_NAMES):
        raise ValueError(f"design vector has {len(x)} values; expected {len(VARIABLE_NAMES)}")
    missing = VARIABLE_NAMES[len(x):]
    x = (*x, *(DEFAULT_VALUES[name] for name in missing))  # vectors from before a variable
    (a_x, b_x, b_z, ap_x, ap_z, bp_x, bp_z, tilt_deg, bc_length_mm, a_z, app_offset, bpp_offset,
     proximal_mount, distal_mount) = (float(v) for v in x)
    ap, bp = PointXZ(ap_x, ap_z), PointXZ(bp_x, bp_z)
    if link_length_resolution_mm is not None:
        b_x, b_z, bp = _assemble((a_x, a_z), (b_x, b_z), ap, bp, link_length_resolution_mm)
    return make_finger(PointXZ(a_x, a_z), ap, PointXZ(b_x, b_z), bp, bc_length_mm, app_offset, bpp_offset,
                       math.radians(tilt_deg), (proximal_mount, distal_mount))


def make_finger(a: PointXZ, ap: PointXZ, b: PointXZ, bp: PointXZ, bc_length_mm: float, app_offset_mm: float = 0.0,
                bpp_offset_mm: float = 0.0, tilt_rad: float = 0.0,
                pad_offsets_mm: tuple[float, float] = (0.0, 0.0)) -> Finger:
    """A CoDe finger from its extended pose: the four-bar's points (the
    proximal frame), ``|BC|``, ``A''`` ``app_offset_mm`` past ``A'_x`` and
    ``B''`` ``bpp_offset_mm`` short of ``B'_x`` (where the segments end and
    start) and the servo's tilt."""

    require_positive("bc_length_mm", bc_length_mm)
    return Finger(
        proximal=Segment(0.0, ap.x_mm + app_offset_mm),
        distal=Segment((bp.x_mm - bpp_offset_mm) - b.x_mm, bc_length_mm),
        joint=FourBar(a=a, ap=ap, b=b, bp=bp),
        servo=ServoMount(tilt_rad),
        pad_offsets_mm=(float(pad_offsets_mm[0]), float(pad_offsets_mm[1])),
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


def encode(finger: Finger) -> tuple[float, ...]:
    """Return the design vector of a linkage finger."""

    f, j = finger, finger.joint
    return (j.a.x_mm, j.b.x_mm, j.b.z_mm, j.ap.x_mm, j.ap.z_mm, j.bp.x_mm, j.bp.z_mm,
            round(f.servo.tilt_deg, 9), f.distal.end_mm, j.a.z_mm, round(f.proximal.end_mm - j.ap.x_mm, 9),
            round(j.bp.x_mm - f.distal_start.x_mm, 9), *f.pad_offsets_mm)


def read_design(row: Mapping[str, str]) -> list[float]:
    """Design vector of a CSV row (by name), filling variables the row predates."""

    return [float(row[name]) if row.get(name, "") != "" else DEFAULT_VALUES[name] for name in VARIABLE_NAMES]


__all__ = ["DEFAULT_VALUES", "VARIABLE_NAMES", "decode", "encode", "make_finger", "read_design"]
