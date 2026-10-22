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

    ``O`` (MCP, actuated) is the origin and ``+X`` is the proximal link's
    axis; ``A`` (PIP) sits at ``(a_x_mm, a_z_mm)``. In this pose ``BC`` is
    parallel to ``+X``: ``B`` and ``C`` share the height ``b_z_mm``. ``A'``
    and ``B'`` (DIP) are free in the plane. The bodies are the proximal link
    ``O-A-A'``, the middle link ``A-B'``, the distal link ``B'-B-C`` and the
    coupler ``A'-B``. The design variables are ``a_x_mm``, ``a_z_mm``,
    ``b_x_mm``, ``b_z_mm``, ``ap`` and ``bp``; ``bc_length_mm``
    is the fingertip length. The PIP servo's shaft is on ``A'``; its rear body
    lies back along ``-X`` tilted dorsally by ``pip_servo_tilt_rad``. The
    proximal pad's end ``A''`` lies on the mounting line ``app_offset_mm`` past
    ``A'_x``; the distal pad's end ``B''`` lies on its mounting line
    ``bpp_offset_mm`` short of ``B'_x`` (toward the joint).
    """

    a_x_mm: float
    b_x_mm: float
    ap: PointXZ
    bp: PointXZ
    bc_length_mm: float
    b_z_mm: float = 0.0
    pip_servo_tilt_rad: float = 0.0
    a_z_mm: float = 0.0
    app_offset_mm: float = 0.0
    """``A''_x - A'_x``: 0 puts ``A''`` right below ``A'``."""
    bpp_offset_mm: float = 0.0
    """``B'_x - B''_x`` in the extended pose: 0 puts ``B''`` right below ``B'``;
    more runs the distal pad proximally, toward the joint."""

    @property
    def app_x_mm(self) -> float:
        """``x`` of ``A''``, the proximal pad's end on its mounting line."""

        return self.ap.x_mm + self.app_offset_mm

    @property
    def bpp_x_mm(self) -> float:
        """``x`` of ``B''`` in the extended pose, the distal pad's end on its mounting line."""

        return self.bp.x_mm - self.bpp_offset_mm

    def __post_init__(self) -> None:
        require_positive("a_x_mm", self.a_x_mm)
        require_finite("b_x_mm", self.b_x_mm)
        require_finite("b_z_mm", self.b_z_mm)
        require_finite("a_z_mm", self.a_z_mm)
        require_finite("pip_servo_tilt_rad", self.pip_servo_tilt_rad)
        require_finite("app_offset_mm", self.app_offset_mm)
        require_finite("bpp_offset_mm", self.bpp_offset_mm)
        require_positive("bc_length_mm", self.bc_length_mm)
        for name in ("ap", "bp"):
            if not isinstance(getattr(self, name), PointXZ):
                raise TypeError(f"{name} must be a PointXZ instance")
        if self.b_x_mm <= self.a_x_mm:
            raise ValueError("b_x_mm must be greater than a_x_mm")


__all__ = ["FingerLinkageParameters", "PointXZ"]
