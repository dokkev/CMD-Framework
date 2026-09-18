"""Immutable parameters for the planar CLEAR Hand flexion linkage.

The linkage uses the same canonical frame as the fingertip morphology: ``X``
is lateral, ``Y`` points distally, and ``Z`` is dorsal.  This parameter module
stores only local YZ geometry.  The planar assumption therefore keeps every
linkage point at ``X = 0`` without exposing an unnecessary optimization
coordinate.

Angles are intentionally absent.  The reference configuration is the fully
extended configuration with all future flexion angles equal to zero; solving
those angles belongs to a later kinematics component.
"""

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
class AnchorYZ:
    """A planar point expressed in a local or fixed YZ frame, in mm."""

    y_mm: float
    z_mm: float

    def __post_init__(self) -> None:
        _require_finite("y_mm", self.y_mm)
        _require_finite("z_mm", self.z_mm)

    def distance_to(self, other: "AnchorYZ") -> float:
        """Return the planar Euclidean distance to another YZ point."""

        if not isinstance(other, AnchorYZ):
            raise TypeError("other must be an AnchorYZ instance")
        return math.hypot(self.y_mm - other.y_mm, self.z_mm - other.z_mm)


@dataclass(frozen=True)
class MainChainParameters:
    """Reference lengths of the serial main chain O-A-B-C."""

    proximal_length_mm: float
    middle_length_mm: float
    distal_length_mm: float

    def __post_init__(self) -> None:
        _require_positive("proximal_length_mm", self.proximal_length_mm)
        _require_positive("middle_length_mm", self.middle_length_mm)
        _require_positive("distal_length_mm", self.distal_length_mm)


@dataclass(frozen=True)
class CouplingAnchorParameters:
    """Local YZ locations of the two four-bar coupling anchors."""

    base_anchor: AnchorYZ
    proximal_anchor: AnchorYZ
    middle_anchor: AnchorYZ
    distal_anchor: AnchorYZ

    def __post_init__(self) -> None:
        for name in (
            "base_anchor",
            "proximal_anchor",
            "middle_anchor",
            "distal_anchor",
        ):
            if not isinstance(getattr(self, name), AnchorYZ):
                raise TypeError(f"{name} must be an AnchorYZ instance")


@dataclass(frozen=True)
class FingerLinkageParameters:
    """Complete immutable design input for one CLEAR finger linkage.

    ``base_anchor`` is expressed in the fixed base frame.  The other three
    anchors are local to the proximal, middle, and distal main-link frames,
    respectively.  Coupler lengths are deliberately absent: they are derived
    from these anchors at the fully extended reference configuration.
    """

    main_chain: MainChainParameters
    coupling: CouplingAnchorParameters

    def __post_init__(self) -> None:
        if not isinstance(self.main_chain, MainChainParameters):
            raise TypeError("main_chain must be a MainChainParameters instance")
        if not isinstance(self.coupling, CouplingAnchorParameters):
            raise TypeError("coupling must be a CouplingAnchorParameters instance")


__all__ = [
    "AnchorYZ",
    "CouplingAnchorParameters",
    "FingerLinkageParameters",
    "MainChainParameters",
]
