"""Reference-pose design variables for the 2-DoF polycentric finger linkage."""

from __future__ import annotations

import math
from dataclasses import dataclass

from core._validation import require_finite, require_positive


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

    ``O`` (MCP, actuated) is the origin and ``A`` (PIP, actuated) lies on the
    ``z = 0`` line. ``BC`` is parallel to ``OA``: ``B`` and ``C`` share the
    height ``b_z_mm``. ``A'`` and ``B'`` (DIP) are free in the plane. The
    bodies are the proximal link ``O-A-A'``, the middle link ``A-B'``, the
    distal link ``B'-B-C`` and the coupler ``A'-B``. The design variables are
    ``a_x_mm``, ``b_x_mm``, ``b_z_mm``, ``ap`` and ``bp``; ``bc_length_mm``
    is the fingertip length. The PIP servo's shaft is on ``A'``; its rear body
    lies back along ``-X`` tilted dorsally by ``pip_servo_tilt_rad``.
    """

    a_x_mm: float
    b_x_mm: float
    ap: PointXZ
    bp: PointXZ
    bc_length_mm: float
    b_z_mm: float = 0.0
    pip_servo_tilt_rad: float = 0.0

    def __post_init__(self) -> None:
        require_positive("a_x_mm", self.a_x_mm)
        require_finite("b_x_mm", self.b_x_mm)
        require_finite("b_z_mm", self.b_z_mm)
        require_finite("pip_servo_tilt_rad", self.pip_servo_tilt_rad)
        require_positive("bc_length_mm", self.bc_length_mm)
        for name in ("ap", "bp"):
            if not isinstance(getattr(self, name), PointXZ):
                raise TypeError(f"{name} must be a PointXZ instance")
        if self.b_x_mm <= self.a_x_mm:
            raise ValueError("b_x_mm must be greater than a_x_mm")


__all__ = ["FingerLinkageParameters", "PointXZ"]
