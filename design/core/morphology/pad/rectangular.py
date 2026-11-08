"""A rectangular block pad, the simplest stand-in for a contact pad."""

from __future__ import annotations

from dataclasses import dataclass

from core.morphology.pad.base import Pad, PointXZ
from core._validation import require_finite, require_positive


@dataclass(frozen=True)
class RectangularPad(Pad):
    """A block ``length x width x thickness`` hanging below the mounting plane.

    It occupies ``0 <= x <= length_mm``, ``|y| <= width_mm / 2`` and
    ``-thickness_mm <= z <= 0``; the contact surface is its flat bottom.
    """

    length_mm: float
    width_mm: float
    thickness_mm: float
    rotation_axis_z_mm: float = 0.0

    def __post_init__(self) -> None:
        for name in ("length_mm", "width_mm", "thickness_mm"):
            require_positive(name, getattr(self, name))
        require_finite("rotation_axis_z_mm", self.rotation_axis_z_mm)

    def side_silhouette_mm(self, samples: int = 24) -> tuple[PointXZ, ...]:
        return (
            (0.0, 0.0),
            (self.length_mm, 0.0),
            (self.length_mm, -self.thickness_mm),
            (0.0, -self.thickness_mm),
        )


__all__ = ["RectangularPad"]
