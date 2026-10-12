"""Independent reference-coordinate inputs for the planar finger linkage."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..._validation import require_finite, require_positive


@dataclass(frozen=True)
class PointXZ:
    """A point or translation in canonical linkage-plane coordinates (mm)."""

    x_mm: float
    z_mm: float

    def __post_init__(self) -> None:
        require_finite("x_mm", self.x_mm)
        require_finite("z_mm", self.z_mm)

    def distance_to(self, other: "PointXZ") -> float:
        """Return the planar Euclidean distance to another XZ point."""

        if not isinstance(other, PointXZ):
            raise TypeError("other must be a PointXZ instance")
        return math.hypot(self.x_mm - other.x_mm, self.z_mm - other.z_mm)


@dataclass(frozen=True)
class FingerLinkageParameters:
    """Reference-pose point translations for one planar CLEAR linkage.

    Each vector is expressed directly in the canonical reference-pose XZ
    coordinates. Its name identifies the point from which it is translated;
    there are no implicit body frames or user-provided reference rotations.
    """

    o_prime_from_o: PointXZ
    a_from_o: PointXZ
    a_prime_from_o: PointXZ
    b_from_a: PointXZ
    b_prime_from_a: PointXZ
    bc_length_mm: float
    c_prime_from_b: PointXZ

    def __post_init__(self) -> None:
        for name in (
            "o_prime_from_o",
            "a_from_o",
            "a_prime_from_o",
            "b_from_a",
            "b_prime_from_a",
            "c_prime_from_b",
        ):
            if not isinstance(getattr(self, name), PointXZ):
                raise TypeError(f"{name} must be a PointXZ instance")
        require_positive("bc_length_mm", self.bc_length_mm)


__all__ = ["FingerLinkageParameters", "PointXZ"]
