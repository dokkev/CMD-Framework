"""Independent reference-coordinate inputs for the planar finger linkage."""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real


def _require_finite(name: str, value: object) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be a finite value in mm; got {value!r}")


def _require_positive(name: str, value: object) -> None:
    _require_finite(name, value)
    if value <= 0:
        raise ValueError(f"{name} must be positive; got {value!r}")


@dataclass(frozen=True)
class PointYZ:
    """A point or translation in canonical linkage-plane coordinates (mm)."""

    y_mm: float
    z_mm: float

    def __post_init__(self) -> None:
        _require_finite("y_mm", self.y_mm)
        _require_finite("z_mm", self.z_mm)

    def distance_to(self, other: "PointYZ") -> float:
        """Return the planar Euclidean distance to another YZ point."""

        if not isinstance(other, PointYZ):
            raise TypeError("other must be a PointYZ instance")
        return math.hypot(self.y_mm - other.y_mm, self.z_mm - other.z_mm)


@dataclass(frozen=True)
class FingerLinkageParameters:
    """Reference-pose point translations for one planar CLEAR linkage.

    Each vector is expressed directly in the canonical reference-pose YZ
    coordinates. Its name identifies the point from which it is translated;
    there are no implicit body frames or user-provided reference rotations.
    """

    o_prime_from_o: PointYZ
    a_from_o: PointYZ
    a_prime_from_o: PointYZ
    b_from_a: PointYZ
    b_prime_from_a: PointYZ
    bc_length_mm: float
    c_prime_from_b: PointYZ

    def __post_init__(self) -> None:
        for name in (
            "o_prime_from_o",
            "a_from_o",
            "a_prime_from_o",
            "b_from_a",
            "b_prime_from_a",
            "c_prime_from_b",
        ):
            if not isinstance(getattr(self, name), PointYZ):
                raise TypeError(f"{name} must be a PointYZ instance")
        _require_positive("bc_length_mm", self.bc_length_mm)


__all__ = ["FingerLinkageParameters", "PointYZ"]
