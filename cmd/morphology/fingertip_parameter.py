"""Immutable physical parameters for the analytic fingertip morphology.

The dimensions in this module are all in millimetres.  The parameter groups
keep the compliant pad morphology separate from the plain T-shaped carrier.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from numbers import Real


def _require_positive(name: str, value: object) -> None:
    """Require one finite, strictly positive physical length."""

    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
        or value <= 0
    ):
        raise ValueError(f"{name} must be a finite, positive length in mm; got {value!r}")


@dataclass(frozen=True)
class PadParameters:
    """Independent dimensions of the compliant fingertip pad."""

    vertical_pad_width_mm: float = 25.0
    vertical_pad_height_mm: float = 6.0
    elliptical_pad_width_mm: float = 25.0
    elliptical_pad_height_mm: float = 10.0
    pad_length_mm: float = 40.0
    tip_length_mm: float = 15.0

    def __post_init__(self) -> None:
        for name in (
            "vertical_pad_width_mm",
            "vertical_pad_height_mm",
            "elliptical_pad_width_mm",
            "elliptical_pad_height_mm",
            "pad_length_mm",
            "tip_length_mm",
        ):
            _require_positive(f"pad.{name}", getattr(self, name))


@dataclass(frozen=True)
class PhalanxParameters:
    """Independent dimensions of the rigid carrier and its stem."""

    carrier_width_mm: float = 25.0
    carrier_height_mm: float = 6.0
    stem_width_mm: float = 8.0
    stem_height_mm: float = 5.0
    carrier_length_mm: float = 35.0

    def __post_init__(self) -> None:
        for name in (
            "carrier_width_mm",
            "carrier_height_mm",
            "stem_width_mm",
            "stem_height_mm",
            "carrier_length_mm",
        ):
            _require_positive(f"phalanx.{name}", getattr(self, name))


@dataclass(frozen=True)
class FingertipParameters:
    """Complete immutable input set for one concrete fingertip.

    ``pad`` defines the contact envelope.  ``phalanx`` defines the internal
    rigid T-shaped carrier, without a stepped bonding interface.
    """

    pad: PadParameters = field(default_factory=PadParameters)
    phalanx: PhalanxParameters = field(default_factory=PhalanxParameters)

    def __post_init__(self) -> None:
        if not isinstance(self.pad, PadParameters):
            raise TypeError("pad must be a PadParameters instance")
        if not isinstance(self.phalanx, PhalanxParameters):
            raise TypeError("phalanx must be a PhalanxParameters instance")

        pad = self.pad
        phalanx = self.phalanx
        straight_length_mm = pad.pad_length_mm - pad.tip_length_mm
        if straight_length_mm <= 0.0:
            raise ValueError(
                "pad.tip_length_mm must be smaller than pad.pad_length_mm"
            )

        if phalanx.carrier_length_mm <= straight_length_mm:
            raise ValueError(
                "phalanx.carrier_length_mm must extend beyond the straight pad body"
            )
        if phalanx.carrier_length_mm >= pad.pad_length_mm:
            raise ValueError(
                "phalanx.carrier_length_mm must be smaller than pad.pad_length_mm"
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
            + ((phalanx.carrier_length_mm - straight_length_mm) / pad.tip_length_mm) ** 2
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


__all__ = [
    "FingertipParameters",
    "PadParameters",
    "PhalanxParameters",
]
