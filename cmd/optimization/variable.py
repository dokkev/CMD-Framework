"""Solver-independent optimization variables."""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real


@dataclass(frozen=True)
class BoxVariable:
    """One finite scalar design variable with inclusive box bounds."""

    name: str
    lower: float
    upper: float

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("variable name must be a non-empty string")
        lower = _finite_bound("lower", self.lower)
        upper = _finite_bound("upper", self.upper)
        if lower >= upper:
            raise ValueError("variable lower bound must be less than upper bound")
        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)

    def violation(self, value: float) -> float:
        """Return zero in bounds, otherwise the distance outside the box."""

        if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
            raise ValueError(f"{self.name} value must be finite; got {value!r}")
        return max(self.lower - value, 0.0, value - self.upper)


def _finite_bound(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"variable {name} bound must be finite; got {value!r}")
    return float(value)


__all__ = ["BoxVariable"]
