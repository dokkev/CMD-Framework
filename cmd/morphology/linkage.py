"""Derived reference geometry for the planar CLEAR Hand finger linkage."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import ClassVar

from .linkage_parameter import FingerLinkageParameters, PointYZ


_FULLY_EXTENDED_ANGULAR_TOLERANCE = 1.0e-4


@dataclass(frozen=True)
class ReferencePositions:
    """Derived YZ positions of all points in the reference assembly."""

    o: PointYZ
    op: PointYZ
    a: PointYZ
    ap: PointYZ
    b: PointYZ
    bp: PointYZ
    c: PointYZ
    cp: PointYZ


@dataclass(frozen=True)
class FingerLinkage:
    """Materialized planar linkage geometry for its fully extended pose.

    All points are composed from the reference translations in
    :class:`FingerLinkageParameters`. The reference pose is accepted when
    ``BC`` and ``OC`` are parallel and point in the same distal direction.
    """

    parameters: FingerLinkageParameters

    FLEXION_AXIS_XYZ: ClassVar[tuple[float, float, float]] = (1.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, FingerLinkageParameters):
            raise TypeError("parameters must be a FingerLinkageParameters instance")

        reference = self.reference_positions
        bc = PointYZ(self.parameters.bc_length_mm, 0.0)
        oc = PointYZ(
            reference.c.y_mm - reference.o.y_mm,
            reference.c.z_mm - reference.o.z_mm,
        )
        bc_length = math.hypot(bc.y_mm, bc.z_mm)
        oc_length = math.hypot(oc.y_mm, oc.z_mm)
        cross_2d = bc.y_mm * oc.z_mm - bc.z_mm * oc.y_mm
        dot_2d = bc.y_mm * oc.y_mm + bc.z_mm * oc.z_mm
        scale = bc_length * oc_length
        if not all(math.isfinite(value) for value in (cross_2d, dot_2d, scale)):
            raise ValueError("fully extended reference geometry must remain finite")
        if (
            scale == 0.0
            or abs(cross_2d) > _FULLY_EXTENDED_ANGULAR_TOLERANCE * scale
            or dot_2d <= 0.0
        ):
            raise ValueError(
                "fully extended reference requires BC parallel to OC and pointing "
                "in the same direction"
            )

    @property
    def reference_positions(self) -> ReferencePositions:
        """Compose every reference point by translation from its named origin."""

        p = self.parameters
        o = PointYZ(0.0, 0.0)
        op = _add(o, p.o_prime_from_o)
        a = _add(o, p.a_from_o)
        ap = _add(o, p.a_prime_from_o)
        b = _add(a, p.b_from_a)
        bp = _add(a, p.b_prime_from_a)
        c = _add(b, PointYZ(p.bc_length_mm, 0.0))
        cp = _add(b, p.c_prime_from_b)
        return ReferencePositions(o=o, op=op, a=a, ap=ap, b=b, bp=bp, c=c, cp=cp)

    @property
    def oa_length_mm(self) -> float:
        """Return the derived ``|OA|`` length."""

        return self.reference_positions.o.distance_to(self.reference_positions.a)

    @property
    def ab_length_mm(self) -> float:
        """Return the derived ``|AB|`` length."""

        return self.reference_positions.a.distance_to(self.reference_positions.b)

    @property
    def coupler_1_length_mm(self) -> float:
        """Return the derived fixed length of the O'-B' coupling rod."""

        reference = self.reference_positions
        return reference.op.distance_to(reference.bp)

    @property
    def coupler_2_length_mm(self) -> float:
        """Return the derived fixed length of the A'-C' coupling rod."""

        reference = self.reference_positions
        return reference.ap.distance_to(reference.cp)

    @property
    def flexion_axis_xyz(self) -> tuple[float, float, float]:
        """Return the fixed +X flexion-axis convention, not a design variable."""

        return self.FLEXION_AXIS_XYZ


def _add(first: PointYZ, second: PointYZ) -> PointYZ:
    return PointYZ(first.y_mm + second.y_mm, first.z_mm + second.z_mm)


__all__ = ["FingerLinkage", "ReferencePositions"]
