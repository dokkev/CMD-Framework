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

import math
from dataclasses import dataclass
from typing import ClassVar

from .linkage_parameter import (
    AnchorYZ,
    FingerLinkageParameters,
)


_EXTENDED_COLLINEARITY_TOLERANCE_MM = 0.01
_DIRECTION_TOLERANCE_MM = 1e-10


@dataclass(frozen=True)
class ReferencePositions:
    """Derived YZ positions of all eight points at neutral full extension.

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

    The ``parameters`` field is the independent design input. The reference
    pose places ``O-A'-B'-B-C`` on +Y; ``OA`` and ``AB`` need not be aligned.
    Configuration-dependent values belong in the kinematics layer.
    """

    parameters: FingerLinkageParameters

    FLEXION_AXIS_XYZ: ClassVar[tuple[float, float, float]] = (1.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, FingerLinkageParameters):
            raise TypeError("parameters must be a FingerLinkageParameters instance")

        reference = self.reference_positions
        virtual_a_prime_b_prime = AnchorYZ(
            reference.bp.y_mm - reference.ap.y_mm,
            reference.bp.z_mm - reference.ap.z_mm,
        )
        if (
            virtual_a_prime_b_prime.y_mm <= 0.0
            or abs(virtual_a_prime_b_prime.z_mm)
            > _EXTENDED_COLLINEARITY_TOLERANCE_MM
        ):
            raise ValueError(
                "the fully extended reference requires O-A'-B'-B-C "
                "to be collinear and ordered along +Y; "
                f"A'-B' residual is {virtual_a_prime_b_prime.z_mm:.6g} mm"
            )
        if self._coupler_1_length(reference) <= 0.0:
            raise ValueError("coupler 1 must have a positive reference length")
        if self._coupler_2_length(reference) <= 0.0:
            raise ValueError("coupler 2 must have a positive reference length")

    @property
    def reference_positions(self) -> ReferencePositions:
        """Derive positions with O-A'-B'-B-C aligned along canonical +Y."""

        main = self.parameters.main_chain
        coupling = self.parameters.coupling

        o = AnchorYZ(0.0, 0.0)
        proximal_anchor_length = math.hypot(
            coupling.proximal_anchor.y_mm,
            coupling.proximal_anchor.z_mm,
        )
        middle_to_joint = AnchorYZ(
            main.middle_length_mm - coupling.middle_anchor.y_mm,
            -coupling.middle_anchor.z_mm,
        )
        middle_to_joint_length = math.hypot(
            middle_to_joint.y_mm,
            middle_to_joint.z_mm,
        )
        if proximal_anchor_length <= _DIRECTION_TOLERANCE_MM:
            raise ValueError("the extended reference requires a nonzero OA' vector")
        if middle_to_joint_length <= _DIRECTION_TOLERANCE_MM:
            raise ValueError("the extended reference requires a nonzero B'B vector")

        # q1 is measured from OA' at full extension. Rotate the proximal body
        # so its local OA' vector points along +Y; orient the middle body so
        # its local B'B vector points along the same line. BC already uses the
        # canonical +Y direction in the distal-body frame.
        theta1 = -math.atan2(
            coupling.proximal_anchor.z_mm,
            coupling.proximal_anchor.y_mm,
        )
        theta2 = -math.atan2(middle_to_joint.z_mm, middle_to_joint.y_mm)
        theta3 = 0.0

        a = self._rotate(AnchorYZ(main.proximal_length_mm, 0.0), theta1)
        b = self._translate(
            a=a,
            local=self._rotate(AnchorYZ(main.middle_length_mm, 0.0), theta2),
        )
        c = self._translate(
            a=b,
            local=self._rotate(AnchorYZ(main.distal_length_mm, 0.0), theta3),
        )

        op = coupling.base_anchor
        ap = self._rotate(coupling.proximal_anchor, theta1)
        bp = self._translate(
            a=a,
            local=self._rotate(coupling.middle_anchor, theta2),
        )
        cp = self._translate(
            a=b,
            local=self._rotate(coupling.distal_anchor, theta3),
        )

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
    def _rotate(point: AnchorYZ, angle_rad: float) -> AnchorYZ:
        cosine = math.cos(angle_rad)
        sine = math.sin(angle_rad)
        return AnchorYZ(
            cosine * point.y_mm - sine * point.z_mm,
            sine * point.y_mm + cosine * point.z_mm,
        )

    @staticmethod
    def _coupler_1_length(reference: ReferencePositions) -> float:
        return reference.op.distance_to(reference.bp)

    @staticmethod
    def _coupler_2_length(reference: ReferencePositions) -> float:
        return reference.ap.distance_to(reference.cp)


__all__ = ["FingerLinkage", "ReferencePositions"]
