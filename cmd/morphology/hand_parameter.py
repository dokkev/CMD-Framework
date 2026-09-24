"""Independent design variables for the reference CLEAR Hand."""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be finite; got {value!r}")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite; got {value!r}")
    return value


def _positive(name: str, value: object) -> float:
    value = _finite(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive; got {value!r}")
    return value


@dataclass(frozen=True)
class HandPlanePoint:
    """Independent hand-plane XY position, in millimetres."""

    x_mm: float
    y_mm: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x_mm", _finite("x_mm", self.x_mm))
        object.__setattr__(self, "y_mm", _finite("y_mm", self.y_mm))


@dataclass(frozen=True)
class HandMorphologyParameters:
    """Independent hand-layout and low-dimensional thumb design variables.

    The index MCP-AA is the fixed hand-frame origin. Ordinary MCP-FE positions
    and all downstream thumb positions are derived by transform composition.
    ``theta_rad`` is the positive CMC-RO rotation about ``+Z_H`` from the
    aligned reference axes; ``phi_rad`` is the positive MCP-FE rotation about
    the thumb-frame ``+Z`` axis after the second scalar inter-axis translation.
    """

    middle_mcp_aa: HandPlanePoint
    ring_mcp_aa: HandPlanePoint
    little_mcp_aa: HandPlanePoint
    thumb_cmc_ro: HandPlanePoint
    theta_rad: float
    ro_raa_distance_mm: float
    raa_mcp_fe_distance_mm: float
    phi_rad: float

    def __post_init__(self) -> None:
        for name in (
            "middle_mcp_aa",
            "ring_mcp_aa",
            "little_mcp_aa",
            "thumb_cmc_ro",
        ):
            if not isinstance(getattr(self, name), HandPlanePoint):
                raise TypeError(f"{name} must be a HandPlanePoint")
        object.__setattr__(self, "theta_rad", _finite("theta_rad", self.theta_rad))
        object.__setattr__(
            self,
            "ro_raa_distance_mm",
            _positive("ro_raa_distance_mm", self.ro_raa_distance_mm),
        )
        object.__setattr__(
            self,
            "raa_mcp_fe_distance_mm",
            _positive("raa_mcp_fe_distance_mm", self.raa_mcp_fe_distance_mm),
        )
        object.__setattr__(self, "phi_rad", _finite("phi_rad", self.phi_rad))


__all__ = ["HandMorphologyParameters", "HandPlanePoint"]
