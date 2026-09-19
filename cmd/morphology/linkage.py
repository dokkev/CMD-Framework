"""Derived reference geometry for the CLEAR Hand flexion linkage.

This module consumes the independent design variables from
``linkage_parameter.py`` and derives the fully extended reference assembly.
It owns no optimizer variables and does not solve configuration-dependent
joint angles or posed positions. Future kinematics and simulation modules can
consume these derived positions and fixed coupler lengths without redefining
the mechanism geometry.

The mechanism is planar in YZ, so every represented point has X = 0 by
construction. All revolute flexion axes are +X, but no joint-angle solving is
implemented here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

from .linkage_parameter import (
    AnchorYZ,
    FingerLinkageParameters,
)


@dataclass(frozen=True)
class ReferencePositions:
    """Derived YZ positions of all eight points at the reference assembly.

    Lowercase names mirror the mechanism notation: ``o``, ``a``, ``b``, and
    ``c`` are the main-chain pivots, while ``op``, ``ap``, ``bp``, and ``cp``
    are the corresponding primed coupling anchors. These are not independent
    design variables.
    """

    o: AnchorYZ
    a: AnchorYZ
    b: AnchorYZ
    c: AnchorYZ
    op: AnchorYZ
    ap: AnchorYZ
    bp: AnchorYZ
    cp: AnchorYZ


@dataclass(frozen=True)
class FingerLinkage:
    """Constructed derived reference geometry for one CLEAR linkage.

    The ``parameters`` field is the independent design input. All geometric
    properties exposed by this class are derived from it at q1=q2=q3=0.
    Configuration-dependent values must be added by a later kinematics layer.
    """

    parameters: FingerLinkageParameters

    FLEXION_AXIS_XYZ: ClassVar[tuple[float, float, float]] = (1.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, FingerLinkageParameters):
            raise TypeError("parameters must be a FingerLinkageParameters instance")

        reference = self.reference_positions
        if self._coupler_1_length(reference) <= 0.0:
            raise ValueError("coupler 1 must have a positive reference length")
        if self._coupler_2_length(reference) <= 0.0:
            raise ValueError("coupler 2 must have a positive reference length")

    @property
    def reference_positions(self) -> ReferencePositions:
        """Derive all linkage-point positions at q1=q2=q3=0."""

        main = self.parameters.main_chain
        coupling = self.parameters.coupling

        o = AnchorYZ(0.0, 0.0)
        a = AnchorYZ(main.proximal_length_mm, 0.0)
        b = AnchorYZ(
            main.proximal_length_mm + main.middle_length_mm,
            0.0,
        )
        c = AnchorYZ(
            main.proximal_length_mm
            + main.middle_length_mm
            + main.distal_length_mm,
            0.0,
        )

        # At the reference configuration all main-link frames are aligned
        # with the canonical frame, so local anchor coordinates are translated
        # by their owning frame origins without a rotation.
        op = coupling.base_anchor
        ap = self._translate(a=o, local=coupling.proximal_anchor)
        bp = self._translate(a=a, local=coupling.middle_anchor)
        cp = self._translate(a=b, local=coupling.distal_anchor)

        return ReferencePositions(o=o, a=a, b=b, c=c, op=op, ap=ap, bp=bp, cp=cp)

    @property
    def coupler_1_length_mm(self) -> float:
        """Return the derived fixed length of the O'-B' coupling rod."""

        return self._coupler_1_length(self.reference_positions)

    @property
    def coupler_2_length_mm(self) -> float:
        """Return the derived fixed length of the A'-C' coupling rod."""

        return self._coupler_2_length(self.reference_positions)

    @property
    def total_main_chain_length_mm(self) -> float:
        """Return the derived sum ``|OA| + |AB| + |BC|``."""

        main = self.parameters.main_chain
        return (
            main.proximal_length_mm
            + main.middle_length_mm
            + main.distal_length_mm
        )

    @property
    def flexion_axis_xyz(self) -> tuple[float, float, float]:
        """Return the fixed +X flexion-axis convention, not a design variable."""

        return self.FLEXION_AXIS_XYZ

    @staticmethod
    def _translate(a: AnchorYZ, local: AnchorYZ) -> AnchorYZ:
        return AnchorYZ(a.y_mm + local.y_mm, a.z_mm + local.z_mm)

    @staticmethod
    def _coupler_1_length(reference: ReferencePositions) -> float:
        return reference.op.distance_to(reference.bp)

    @staticmethod
    def _coupler_2_length(reference: ReferencePositions) -> float:
        return reference.ap.distance_to(reference.cp)


__all__ = ["FingerLinkage", "ReferencePositions"]
