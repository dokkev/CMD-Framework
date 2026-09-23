"""Immutable hand-level CLEAR Hand placement parameters."""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real


def _finite_mm(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite value in mm; got {value!r}")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite value in mm; got {value!r}")
    return value


@dataclass(frozen=True)
class PlanarPlacement:
    """A hand-layout point in the palm design plane ``Z = 0``."""

    x_mm: float
    y_mm: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x_mm", _finite_mm("x_mm", self.x_mm))
        object.__setattr__(self, "y_mm", _finite_mm("y_mm", self.y_mm))


@dataclass(frozen=True)
class HandDesignParameters:
    """Right-hand MCP-AA shaft-frame and thumb-CMC placement values.

    Coordinates use the canonical right-hand palm frame: +X is distal, +Y
    points toward the thumb/radial side, and +Z is dorsal. Every placement is
    planar and therefore expressed as an XY point on ``Z = 0``. For MCP-AA,
    each placement is the XM335 output-shaft / joint-frame position; the
    actuator-local origin is coincident with that point.
    """

    index_mcp_aa: PlanarPlacement
    middle_mcp_aa: PlanarPlacement
    ring_mcp_aa: PlanarPlacement
    little_mcp_aa: PlanarPlacement
    thumb_cmc: PlanarPlacement

    def __post_init__(self) -> None:
        for name in (
            "index_mcp_aa",
            "middle_mcp_aa",
            "ring_mcp_aa",
            "little_mcp_aa",
            "thumb_cmc",
        ):
            if not isinstance(getattr(self, name), PlanarPlacement):
                raise TypeError(f"{name} must be a PlanarPlacement instance")


__all__ = ["HandDesignParameters", "PlanarPlacement"]
