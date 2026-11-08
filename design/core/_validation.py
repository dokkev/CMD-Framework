"""Shared scalar validation for frozen dataclass inputs."""

from __future__ import annotations

import math
from numbers import Real


def require_finite(name: str, value: object) -> float:
    """Return ``value`` as a float, rejecting bools, non-reals, NaN and infinity."""

    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite; got {value!r}")
    return float(value)


def require_positive(name: str, value: object) -> float:
    """Return a finite, strictly positive ``value`` as a float."""

    value = require_finite(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive; got {value!r}")
    return value


def require_nonnegative(name: str, value: object) -> float:
    """Return a finite, non-negative ``value`` as a float."""

    value = require_finite(name, value)
    if value < 0.0:
        raise ValueError(f"{name} must be non-negative; got {value!r}")
    return value
