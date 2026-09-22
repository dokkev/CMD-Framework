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
    """Fundamental compliant contact-envelope dimensions.

    The lateral width is constant along the finger. ``tip_length_mm`` is the
    longitudinal length of the rounded distal cap.
    """

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
    """Dimensions of the rigid T-shaped carrier.

    ``carrier_height_mm`` is the dorsal plate thickness. The stem extends
    below its shoulder and ends ``carrier_extension_mm`` beyond the straight
    body. The dorsal plate continues over the full rounded fingertip nose.
    """

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
        if phalanx.stem_width_mm >= pad.width_mm:
            raise ValueError("phalanx.stem_width_mm must be smaller than pad.width_mm")

        if phalanx.carrier_extension_mm >= pad.tip_length_mm:
            raise ValueError(
                "phalanx.carrier_extension_mm must be smaller than pad.tip_length_mm"
            )

        end_half_width_mm = (pad.width_mm / 2.0) * math.sqrt(
            1.0 - (phalanx.carrier_extension_mm / pad.tip_length_mm) ** 2
        )
        if phalanx.stem_width_mm / 2.0 >= end_half_width_mm:
            raise ValueError("the stem must remain narrower than the dorsal plate at its distal end")

        maximum_pad_depth_mm = pad.pad_straight_height_mm + pad.ellipse_height_mm
        if phalanx.stem_height_mm >= maximum_pad_depth_mm:
            raise ValueError(
                "the rigid-core stem must fit within the maximum compliant pad "
                f"depth of {maximum_pad_depth_mm:.16g} mm"
            )


__all__ = [
    "FingertipParameters",
    "PadParameters",
    "PhalanxParameters",
]
