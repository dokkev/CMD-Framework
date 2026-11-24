# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Contact-interface morphologies: the pad's palmar surface, nothing else (Study 3).

A pad in a ``code_hand_sim/1`` export is a box on its segment (``bodies[].boxes``, names ending in ``_pad``):
its local ``+y`` face is the contact face, ``-y`` the mounting face on the segment, ``x`` runs along the
segment and ``z`` across it. A :class:`PadProfile` replaces the box by a convex prism with the same mounting
face, length and width and the same thickness at its middle. The contact face can be curved along either axis:

- ``flat`` (M0): the box itself, authored as a convex mesh (8 vertices), so that both variants go through the
  same mesh-collision path of the solver (a box primitive would use another collision routine);
- ``crowned`` (M1): the contact face a circular arc along the segment whose middle is the flat face and whose
  ends lie ``crown_mm`` lower (the sagitta), the arc sampled at ``segments`` chords.

The crowned pad is therefore contained in the flat one (same envelope, same mounting datum). Masses are the
export's body masses either way (the pads carry no mass of their own in the asset). Nothing here imports
Isaac Lab or ``pxr``.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class PadProfile:
    """A contact face profile, optionally applied only to named pads."""

    name: str
    """Variant name (``flat``, ``crowned``)."""
    crown_mm: float = 0.0
    """Contact face sagitta [mm]: how far the ends of the curved axis lie below its middle (0: flat)."""
    segments: int = 16
    """Chords the arc is sampled at (a flat face needs none)."""
    across_width: bool = False
    """Curve along local z (pad width) instead of local x (length)."""
    pads: tuple[str, ...] = ()
    """Pad names to curve. An empty selection applies the profile to every pad."""

    def __post_init__(self) -> None:
        if self.crown_mm < 0.0:
            raise ValueError("crown_mm must be >= 0")
        if self.crown_mm > 0.0 and self.segments < 2:
            raise ValueError("a crowned face needs at least 2 segments")

    def key(self) -> str:
        """A short content hash of the profile's parameters."""
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True).encode()).hexdigest()[:10]

    def for_pad(self, name: str) -> PadProfile:
        return self if not self.pads or name in self.pads else FLAT


FLAT = PadProfile("flat", crown_mm=0.0)
CROWNED = PadProfile("crowned", crown_mm=2.0)
VARIANTS: dict[str, PadProfile] = {"flat": FLAT, "crowned": CROWNED}
"""The Study 3 pilot's two morphologies (M0, M1). The crowned pads of the 3 mm export keep 1 mm at their ends."""


def contact_face_height(x: np.ndarray, half_length: float, half_thickness: float, crown: float) -> np.ndarray:
    """Height of the contact face above the pad's mid-plane at ``x`` along its curved axis [m]."""
    x = np.asarray(x, dtype=float)
    if crown <= 0.0:
        return np.full_like(x, half_thickness)
    radius = half_length**2 / (2.0 * crown) + crown / 2.0
    return half_thickness - (radius - np.sqrt(radius**2 - x**2))


def pad_profile_polygon(half_extents, profile: PadProfile) -> np.ndarray:
    """Counter-clockwise outline in the curved-axis and normal plane, starting with the mounting face."""
    hx, hy, hz = (float(v) for v in half_extents)
    half_span = hz if profile.across_width else hx
    crown = profile.crown_mm * 1e-3
    if crown >= 2.0 * hy:
        raise ValueError(f"crown {profile.crown_mm} mm leaves no pad at its ends (thickness {2e3 * hy:.2f} mm)")
    if crown <= 0.0:
        top = np.array([[half_span, hy], [-half_span, hy]])
    else:
        xs = np.linspace(half_span, -half_span, profile.segments + 1)
        top = np.stack([xs, contact_face_height(xs, half_span, hy, crown)], axis=1)
    return np.vstack([[[-half_span, -hy], [half_span, -hy]], top])


def pad_mesh(half_extents, profile: PadProfile) -> tuple[np.ndarray, list[int], list[int]]:
    """The pad as a closed convex prism in its local frame: points ``(2n, 3)``, face vertex counts and
    indices (outward, counter-clockwise). The extrusion is orthogonal to the curved axis and contact normal."""
    half_extrusion = float(half_extents[0 if profile.across_width else 2])
    outline = pad_profile_polygon(half_extents, profile)
    n = len(outline)
    points = np.vstack([
        np.column_stack([outline, np.full(n, -half_extrusion)]),
        np.column_stack([outline, np.full(n, half_extrusion)]),
    ])
    if profile.across_width:
        points = points[:, [2, 1, 0]] * [-1.0, 1.0, 1.0]
    counts, indices = [n, n], list(range(n - 1, -1, -1)) + [n + i for i in range(n)]  # -z cap reversed, +z cap
    for i in range(n):
        j = (i + 1) % n
        counts.append(4)
        indices += [i, j, n + j, n + i]
    return points, counts, indices


def is_pad(box: dict) -> bool:
    return str(box.get("name", "")).endswith("_pad")


def surface_report(desc_bodies: list[dict], profile: PadProfile) -> list[dict]:
    """Per pad: its length, width, thickness at the middle and at the ends, crown, and arc radius [mm]."""
    rows = []
    for body in desc_bodies:
        for box in body.get("boxes", []):
            if not is_pad(box):
                continue
            hx, hy, hz = (float(v) for v in box["half_extents"])
            pad = profile.for_pad(box["name"])
            crown = pad.crown_mm * 1e-3
            half_span = hz if pad.across_width else hx
            ends = float(contact_face_height(np.array([half_span]), half_span, hy, crown)[0]) + hy
            rows.append(
                {
                    "pad": box["name"],
                    "length_mm": 2e3 * hx,
                    "width_mm": 2e3 * hz,
                    "thickness_middle_mm": 2e3 * hy,
                    "thickness_ends_mm": 1e3 * ends,
                    "crown_mm": pad.crown_mm,
                    "arc_radius_mm": math.inf if crown <= 0 else 1e3 * (half_span**2 / (2 * crown) + crown / 2),
                }
            )
    return rows


__all__ = [
    "CROWNED",
    "FLAT",
    "VARIANTS",
    "PadProfile",
    "contact_face_height",
    "is_pad",
    "pad_mesh",
    "pad_profile_polygon",
    "surface_report",
]
