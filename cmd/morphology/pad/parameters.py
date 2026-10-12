"""Shared fingertip-family geometry and concrete per-digit parameters."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..._validation import require_positive


@dataclass(frozen=True)
class PadParameters:
    """Shared dimensions of the compliant fingertip pad, in millimetres."""

    vertical_pad_width_mm: float = 25.0
    vertical_pad_height_mm: float = 6.0
    elliptical_pad_width_mm: float = 25.0
    elliptical_pad_height_mm: float = 10.0
    tip_length_mm: float = 15.0

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

    carrier_width_mm: float = 25.0
    carrier_height_mm: float = 6.0
    stem_width_mm: float = 8.0
    stem_height_mm: float = 5.0

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

    The carrier ends a shared ``carrier_distal_inset_mm`` before each digit's
    total fingertip length. Its default baseline is 5 mm; callers may vary it
    for the fingertip family. This keeps the distal carrier/pad relationship
    invariant as the proximal straight section changes length.
    """

    pad: PadParameters = field(default_factory=PadParameters)
    phalanx: PhalanxParameters = field(default_factory=PhalanxParameters)
    carrier_distal_inset_mm: float = 5.0

    def __post_init__(self) -> None:
        if not isinstance(self.pad, PadParameters):
            raise TypeError("pad must be a PadParameters instance")
        if not isinstance(self.phalanx, PhalanxParameters):
            raise TypeError("phalanx must be a PhalanxParameters instance")
        require_positive(
            "carrier_distal_inset_mm", self.carrier_distal_inset_mm
        )

        pad = self.pad
        phalanx = self.phalanx
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


@dataclass(frozen=True)
class FingertipParameters:
    """Parameters for one concrete digit using a shared fingertip family."""

    family: FingertipFamilyParameters = field(default_factory=FingertipFamilyParameters)
    total_length_mm: float = 40.0

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
]
