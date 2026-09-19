"""Independent design variables for the planar CLEAR finger linkage.

The linkage uses the canonical finger frame: ``X`` is lateral, ``Y`` points
distally, and ``Z`` is dorsal. Flexion is planar in YZ and every represented
point therefore has ``X = 0`` without exposing an unnecessary optimization
coordinate.

This module owns only independent, optimizer-controlled design variables:

Independent design variables
----------------------------

* ``main_chain.proximal_length_mm`` = ``|OA|``
* ``main_chain.middle_length_mm`` = ``|AB|``
* ``main_chain.distal_length_mm`` = ``|BC|``
* ``coupling.base_anchor`` = ``O'`` in the fixed base frame
* ``coupling.proximal_anchor`` = ``A'`` in the proximal-body frame
* ``coupling.middle_anchor`` = ``B'`` in the middle-body frame
* ``coupling.distal_anchor`` = ``C'`` in the distal-body frame

Derived geometry belongs to :mod:`cmd.morphology.linkage`: reference point
positions and coupling-rod lengths are computed from these variables at the
fully extended reference configuration. Configuration-dependent quantities,
including joint angles and posed point positions, are intentionally absent and
will be produced later by kinematics. Optimization bounds, manufacturability
rules, collision margins, and other design-space constraints are also a
separate concern; this module enforces only local finite/positive-value
admissibility required to construct a meaningful parameter object.
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
    """An independent planar anchor coordinate in a local or fixed YZ frame."""

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
    """Independent optimizer-controlled lengths of the serial chain O-A-B-C."""

    proximal_length_mm: float
    middle_length_mm: float
    distal_length_mm: float

    def __post_init__(self) -> None:
        _require_positive("proximal_length_mm", self.proximal_length_mm)
        _require_positive("middle_length_mm", self.middle_length_mm)
        _require_positive("distal_length_mm", self.distal_length_mm)


@dataclass(frozen=True)
class CouplingAnchorParameters:
    """Independent local YZ coordinates for O', A', B', and C'.

    The four fields are design inputs, not world positions and not coupling-rod
    lengths. Their owning-frame interpretation is:

    * ``base_anchor``: ``O'`` in the fixed base frame;
    * ``proximal_anchor``: ``A'`` in the proximal-body frame;
    * ``middle_anchor``: ``B'`` in the middle-body frame;
    * ``distal_anchor``: ``C'`` in the distal-body frame.
    """

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
    """Independent design variables for the CLEAR flexion linkage.

    Independent / optimizer-controlled quantities
    ----------------------------------------------

    Main chain
    ~~~~~~~~~~

    * ``main_chain.proximal_length_mm`` = ``|OA|``
    * ``main_chain.middle_length_mm`` = ``|AB|``
    * ``main_chain.distal_length_mm`` = ``|BC|``

    Coupling-anchor geometry
    ~~~~~~~~~~~~~~~~~~~~~~~~

    * ``coupling.base_anchor`` = ``O'`` in the fixed base frame
    * ``coupling.proximal_anchor`` = ``A'`` in the proximal-body frame
    * ``coupling.middle_anchor`` = ``B'`` in the middle-body frame
    * ``coupling.distal_anchor`` = ``C'`` in the distal-body frame

    These values define the mechanism design. They do not include quantities
    uniquely determined from the reference assembly or from a commanded
    configuration. In particular, coupler lengths, world reference
    positions, joint angles, and posed positions are not fields of this
    dataclass.

    Local validation enforces finite anchor coordinates and positive finite
    main-link lengths. It does not encode optimizer bounds, manufacturing
    limits, collision constraints, or other application-specific design-space
    constraints; those belong in a separate design/optimization layer.
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
