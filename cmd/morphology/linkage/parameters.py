"""Reference-pose design variables for the 2-DoF polycentric finger linkage."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..._validation import require_finite, require_positive


@dataclass(frozen=True)
class PointXZ:
    """A point in the linkage plane (+X distal, +Z dorsal), in mm."""

    x_mm: float
    z_mm: float

    def __post_init__(self) -> None:
        require_finite("x_mm", self.x_mm)
        require_finite("z_mm", self.z_mm)

    def distance_to(self, other: "PointXZ") -> float:
        """Return the planar Euclidean distance to another point."""

        return math.hypot(self.x_mm - other.x_mm, self.z_mm - other.z_mm)


@dataclass(frozen=True)
class FingerLinkageParameters:
    """Fully extended reference pose of the 2-DoF polycentric finger.

    ``O`` (MCP, actuated) is the origin. ``O``, ``A`` (PIP, actuated), ``B``
    and ``C`` lie on the ``z = 0`` line; ``A'`` and ``B'`` (DIP) are free in
    the plane. The bodies are the proximal link ``O-A-A'``, the middle link
    ``A-B'``, the distal link ``B'-B-C`` and the coupler ``A'-B``. The design
    variables are ``a_x_mm``, ``b_x_mm``, ``ap`` and ``bp``; ``bc_length_mm``
    is the fingertip length.
    """

    a_x_mm: float
    b_x_mm: float
    ap: PointXZ
    bp: PointXZ
    bc_length_mm: float

    def __post_init__(self) -> None:
        require_positive("a_x_mm", self.a_x_mm)
        require_finite("b_x_mm", self.b_x_mm)
        require_positive("bc_length_mm", self.bc_length_mm)
        for name in ("ap", "bp"):
            if not isinstance(getattr(self, name), PointXZ):
                raise TypeError(f"{name} must be a PointXZ instance")
        if self.b_x_mm <= self.a_x_mm:
            raise ValueError("b_x_mm must be greater than a_x_mm")


__all__ = ["FingerLinkageParameters", "PointXZ"]
