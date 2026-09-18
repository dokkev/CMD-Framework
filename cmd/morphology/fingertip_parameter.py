"""Immutable physical parameters for the analytic fingertip morphology.

The dimensions in this module are all in millimetres.  The parameter groups
keep the fundamental compliant pad morphology separate from the internal
T-shaped phalanx and from the fixed mounting feature used by the prototype.
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
    """Fundamental compliant contact-envelope dimensions."""

    width_mm: float = 25.0
    pad_straight_height_mm: float = 6.0
    ellipse_height_mm: float = 10.0
    body_length_mm: float = 25.0
    tip_length_mm: float = 15.0

    def __post_init__(self) -> None:
        for name in (
            "width_mm",
            "pad_straight_height_mm",
            "ellipse_height_mm",
            "body_length_mm",
            "tip_length_mm",
        ):
            _require_positive(f"pad.{name}", getattr(self, name))


@dataclass(frozen=True)
class PhalanxParameters:
    """Dimensions of the rigid T-shaped distal phalanx / carrier."""

    carrier_height_mm: float = 6.0
    stem_width_mm: float = 8.0
    stem_height_mm: float = 5.0
    carrier_extension_mm: float = 10.0

    def __post_init__(self) -> None:
        for name in (
            "carrier_height_mm",
            "stem_width_mm",
            "stem_height_mm",
            "carrier_extension_mm",
        ):
            _require_positive(f"phalanx.{name}", getattr(self, name))


@dataclass(frozen=True)
class MountingInterfaceParameters:
    """Fixed stepped bonding dimensions added for practical fabrication."""

    bond_width_mm: float = 5.0
    bond_height_mm: float = 4.0

    def __post_init__(self) -> None:
        for name in ("bond_width_mm", "bond_height_mm"):
            _require_positive(f"mounting_interface.{name}", getattr(self, name))


@dataclass(frozen=True)
class FingertipParameters:
    """Complete immutable input set for one concrete fingertip.

    ``pad`` defines the contact envelope.  ``phalanx`` defines the internal
    rigid core.  ``mounting_interface`` defines the fixed stepped bonding
    feature and is intentionally not part of the fundamental pad morphology.
    """

    pad: PadParameters = field(default_factory=PadParameters)
    phalanx: PhalanxParameters = field(default_factory=PhalanxParameters)
    mounting_interface: MountingInterfaceParameters = field(
        default_factory=MountingInterfaceParameters
    )

    def __post_init__(self) -> None:
        if not isinstance(self.pad, PadParameters):
            raise TypeError("pad must be a PadParameters instance")
        if not isinstance(self.phalanx, PhalanxParameters):
            raise TypeError("phalanx must be a PhalanxParameters instance")
        if not isinstance(self.mounting_interface, MountingInterfaceParameters):
            raise TypeError(
                "mounting_interface must be a MountingInterfaceParameters instance"
            )

        pad = self.pad
        phalanx = self.phalanx
        mounting = self.mounting_interface

        if mounting.bond_height_mm >= phalanx.carrier_height_mm:
            raise ValueError(
                "mounting_interface.bond_height_mm must be below "
                "phalanx.carrier_height_mm"
            )
        if phalanx.stem_width_mm >= pad.width_mm:
            raise ValueError("phalanx.stem_width_mm must be smaller than pad.width_mm")

        half_width_mm = pad.width_mm / 2.0
        stem_half_width_mm = phalanx.stem_width_mm / 2.0
        if mounting.bond_width_mm >= half_width_mm - stem_half_width_mm:
            raise ValueError(
                "mounting_interface.bond_width_mm leaves no valid material "
                "between the bond step and the stem"
            )

        if phalanx.carrier_extension_mm >= pad.tip_length_mm:
            raise ValueError(
                "phalanx.carrier_extension_mm must be smaller than pad.tip_length_mm"
            )

        corner_x_mm = stem_half_width_mm
        q = (corner_x_mm / half_width_mm) ** 2 + (
            phalanx.carrier_extension_mm / pad.tip_length_mm
        ) ** 2
        if q >= 1.0:
            raise ValueError(
                "the distal rigid-core corner lies outside the quarter-ellipsoid "
                f"envelope: q={q:.16g} must be < 1"
            )

        penetration_mm = max(
            0.0,
            phalanx.stem_height_mm - pad.pad_straight_height_mm,
        )
        allowable_penetration_mm = pad.ellipse_height_mm * math.sqrt(1.0 - q)
        if penetration_mm >= allowable_penetration_mm:
            raise ValueError(
                "the distal rigid-core stem breaks through the quarter-ellipsoid "
                "envelope: penetration must be less than "
                f"{allowable_penetration_mm:.16g} mm"
            )


__all__ = [
    "FingertipParameters",
    "MountingInterfaceParameters",
    "PadParameters",
    "PhalanxParameters",
]
