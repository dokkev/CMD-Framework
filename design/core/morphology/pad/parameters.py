"""Shared fingertip-family geometry and concrete per-digit parameters."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from core._validation import require_finite, require_positive


@dataclass(frozen=True)
class PadParameters:
    """Shared dimensions of the compliant fingertip pad, in millimetres."""

    vertical_pad_width_mm: float = 16.0
    vertical_pad_height_mm: float = 3.0
    elliptical_pad_width_mm: float = 16.0
    elliptical_pad_height_mm: float = 7.0
    tip_length_mm: float = 8.0

    def __post_init__(self) -> None:
        for name in (
            "vertical_pad_width_mm",
            "vertical_pad_height_mm",
            "elliptical_pad_width_mm",
            "elliptical_pad_height_mm",
            "tip_length_mm",
        ):
            require_positive(f"pad.{name}", getattr(self, name))


@dataclass(frozen=True)
class PhalanxParameters:
    """Shared cross-section dimensions of the rigid carrier and its stem."""

    carrier_width_mm: float = 16.0
    carrier_height_mm: float = 4.0
    stem_width_mm: float = 6.0
    stem_height_mm: float = 6.0

    def __post_init__(self) -> None:
        for name in (
            "carrier_width_mm",
            "carrier_height_mm",
            "stem_width_mm",
            "stem_height_mm",
        ):
            require_positive(f"phalanx.{name}", getattr(self, name))


@dataclass(frozen=True)
class FingertipFamilyParameters:
    """Shared structural morphology for all five fingertips.

    The defaults approximate a human index distal segment: about 16 mm wide
    and 14 mm thick (4 mm carrier plate, 3 + 7 mm pad).

    The carrier ends a shared ``carrier_distal_inset_mm`` before each digit's
    total fingertip length. Its default baseline is 3 mm; callers may vary it
    for the fingertip family. This keeps the distal carrier/pad relationship
    invariant as the proximal straight section changes length.

    The distal-joint rotation axis (R.A.) runs laterally through the carrier.
    ``rotation_axis_offset_mm`` (``o_RA``) is its signed height above the
    carrier shoulder plane (dorsal positive), strictly between the stem bottom
    and the carrier top; the default is provisional. Its distance to the pad
    tip is the linkage ``bc_length_mm``, which owns that value.

    ``minimum_pad_thickness_mm`` (``t_min``) is the least compliant pad
    thickness allowed between a stem bottom corner and the pad outer boundary
    in the straight-body cross-section.
    """

    pad: PadParameters = field(default_factory=PadParameters)
    phalanx: PhalanxParameters = field(default_factory=PhalanxParameters)
    carrier_distal_inset_mm: float = 3.0
    rotation_axis_offset_mm: float = -1.0
    minimum_pad_thickness_mm: float = 3.0

    def __post_init__(self) -> None:
        if not isinstance(self.pad, PadParameters):
            raise TypeError("pad must be a PadParameters instance")
        if not isinstance(self.phalanx, PhalanxParameters):
            raise TypeError("phalanx must be a PhalanxParameters instance")
        require_positive(
            "carrier_distal_inset_mm", self.carrier_distal_inset_mm
        )
        require_finite("rotation_axis_offset_mm", self.rotation_axis_offset_mm)
        require_positive("minimum_pad_thickness_mm", self.minimum_pad_thickness_mm)

        pad = self.pad
        phalanx = self.phalanx
        if not (
            -phalanx.stem_height_mm
            < self.rotation_axis_offset_mm
            < phalanx.carrier_height_mm
        ):
            raise ValueError(
                "rotation_axis_offset_mm must place the rotation axis between "
                f"the stem bottom (-{phalanx.stem_height_mm} mm) and the carrier "
                f"top ({phalanx.carrier_height_mm} mm)"
            )
        if self.carrier_distal_inset_mm >= pad.tip_length_mm:
            raise ValueError(
                "carrier_distal_inset_mm must be smaller than pad.tip_length_mm"
            )

        if phalanx.stem_width_mm >= phalanx.carrier_width_mm:
            raise ValueError(
                "phalanx.stem_width_mm must be smaller than phalanx.carrier_width_mm"
            )
        if phalanx.stem_width_mm >= pad.vertical_pad_width_mm:
            raise ValueError(
                "phalanx.stem_width_mm must be smaller than "
                "pad.vertical_pad_width_mm"
            )
        if phalanx.stem_width_mm >= pad.elliptical_pad_width_mm:
            raise ValueError(
                "phalanx.stem_width_mm must be smaller than "
                "pad.elliptical_pad_width_mm"
            )

        tip_half_width_mm = phalanx.carrier_width_mm / 2.0
        stem_half_width_mm = phalanx.stem_width_mm / 2.0
        ellipsoid_coordinate = (
            (stem_half_width_mm / tip_half_width_mm) ** 2
            + (
                (pad.tip_length_mm - self.carrier_distal_inset_mm)
                / pad.tip_length_mm
            )
            ** 2
        )
        if ellipsoid_coordinate >= 1.0:
            raise ValueError(
                "the distal rigid-core corner lies outside the half-ellipsoid "
                f"envelope: q={ellipsoid_coordinate:.16g} must be < 1"
            )

        penetration_mm = max(
            0.0,
            phalanx.stem_height_mm - pad.vertical_pad_height_mm,
        )
        allowable_penetration_mm = pad.elliptical_pad_height_mm * math.sqrt(
            1.0 - ellipsoid_coordinate
        )
        if penetration_mm >= allowable_penetration_mm:
            raise ValueError(
                "the distal rigid-core stem breaks through the half-ellipsoid "
                "envelope: penetration must be less than "
                f"{allowable_penetration_mm:.16g} mm"
            )

        thickness_mm = stem_corner_pad_thickness_mm(pad, phalanx)
        if thickness_mm < self.minimum_pad_thickness_mm:
            raise ValueError(
                f"the pad is {thickness_mm:.3f} mm thick at the stem corner, "
                f"less than minimum_pad_thickness_mm={self.minimum_pad_thickness_mm}"
            )


def stem_corner_pad_thickness_mm(pad: PadParameters, phalanx: PhalanxParameters) -> float:
    """Return the pad thickness ``t`` at a stem bottom corner, in mm.

    This is the distance from the corner ``(w_s/2, -h_s)`` to the nearest point
    of the straight-body pad boundary: the lower half-ellipse, the flat step
    beside it when it is narrower than the pad, and the vertical side wall.
    The section is symmetric, so one corner suffices.
    """

    corner_y = phalanx.stem_width_mm / 2.0
    corner_z = -phalanx.stem_height_mm
    half_width = pad.vertical_pad_width_mm / 2.0
    a = pad.elliptical_pad_width_mm / 2.0
    c = pad.elliptical_pad_height_mm
    h = pad.vertical_pad_height_mm
    if corner_z <= -h - c * math.sqrt(1.0 - (corner_y / a) ** 2):
        return 0.0

    samples = 4000
    distances = [
        math.hypot(a * math.cos(angle) - corner_y, -h - c * math.sin(angle) - corner_z)
        for angle in (math.pi * index / samples for index in range(samples + 1))
        if abs(a * math.cos(angle)) <= half_width
    ]
    if a < half_width:
        step_y = min(max(corner_y, a), half_width)
        distances.append(math.hypot(step_y - corner_y, -h - corner_z))
        wall_bottom = -h
    else:
        wall_bottom = -h - c * math.sqrt(1.0 - (half_width / a) ** 2)
    wall_z = min(max(corner_z, wall_bottom), 0.0)
    distances.append(math.hypot(half_width - corner_y, wall_z - corner_z))
    return min(distances)


@dataclass(frozen=True)
class FingertipParameters:
    """Parameters for one concrete fingertip using a shared fingertip family.

    In a hand the total length is the linkage ``bc_length_mm``; the standalone
    default is a human index distal segment length.
    """

    family: FingertipFamilyParameters = field(default_factory=FingertipFamilyParameters)
    total_length_mm: float = 24.0

    def __post_init__(self) -> None:
        if not isinstance(self.family, FingertipFamilyParameters):
            raise TypeError("family must be a FingertipFamilyParameters instance")
        require_positive("total_length_mm", self.total_length_mm)
        if self.total_length_mm <= self.family.pad.tip_length_mm:
            raise ValueError(
                "total_length_mm must be greater than family.pad.tip_length_mm"
            )


__all__ = [
    "FingertipFamilyParameters",
    "FingertipParameters",
    "PadParameters",
    "PhalanxParameters",
    "stem_corner_pad_thickness_mm",
]
