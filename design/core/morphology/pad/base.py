"""The interface every fingertip contact pad provides.

A pad lives in the fingertip frame: ``+X`` distal, ``+Y`` lateral (parallel to
the distal-joint axis) and ``+Z`` dorsal, so the contact surface faces ``-Z``.
The pad spans ``0 <= x <= length_mm``; its distal-joint rotation axis runs
along ``Y`` at height ``rotation_axis_z_mm``. All values are in millimetres.

Implementations: :class:`~core.morphology.pad.Fingertip` (the parametric pad
and carrier), :class:`~core.morphology.pad.RectangularPad` and
:class:`~core.morphology.pad.MeshPad`.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


PointXZ = tuple[float, float]


class Pad(ABC):
    """A fingertip contact pad, attached to its digit by the rotation axis.

    Subclasses provide ``length_mm`` (the pad's extent along ``X``) and
    ``rotation_axis_z_mm`` (the axis height), as attributes or properties.
    """

    length_mm: float
    rotation_axis_z_mm: float

    @abstractmethod
    def side_silhouette_mm(self, samples: int = 24) -> tuple[PointXZ, ...]:
        """Closed ``(x, z)`` outline of the pad seen from the side (along ``Y``)."""

    @property
    def lowest_z_mm(self) -> float:
        """Deepest point of the contact surface."""

        return min(z for _, z in self.side_silhouette_mm())

    def rotation_axis_mm(self, rotation_axis_to_tip_mm: float) -> tuple[float, float, float]:
        """Return the rotation-axis point that lies ``rotation_axis_to_tip_mm`` from the tip."""

        return (self.length_mm - rotation_axis_to_tip_mm, 0.0, self.rotation_axis_z_mm)


__all__ = ["Pad", "PointXZ"]
