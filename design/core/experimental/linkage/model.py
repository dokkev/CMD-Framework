"""Derived reference geometry for the planar CLEAR Hand finger linkage."""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

from core.experimental.linkage import FingerLinkageParameters, PointXZ


@dataclass(frozen=True)
class ReferencePositions:
    """Derived XZ positions of all points in the reference assembly."""

    o: PointXZ
    op: PointXZ
    a: PointXZ
    ap: PointXZ
    b: PointXZ
    bp: PointXZ
    c: PointXZ
    cp: PointXZ


@dataclass(frozen=True)
class FingerLinkage:
    """Materialized planar linkage geometry for its fully extended pose.

    All points are composed from the reference translations in
    :class:`FingerLinkageParameters`. In the fully extended reference pose
    ``BC`` points along ``+X`` (``C = B + (bc_length_mm, 0)``); ``B`` and ``C``
    may sit at any ``z``, and ``OC`` need not be parallel to ``BC``.
    """

    parameters: FingerLinkageParameters

    FLEXION_AXIS_XYZ: ClassVar[tuple[float, float, float]] = (0.0, 1.0, 0.0)

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, FingerLinkageParameters):
            raise TypeError("parameters must be a FingerLinkageParameters instance")

    @property
    def reference_positions(self) -> ReferencePositions:
        """Compose every reference point by translation from its named origin."""

        p = self.parameters
        o = PointXZ(0.0, 0.0)
        op = _add(o, p.op_from_o)
        a = _add(o, p.a_from_o)
        ap = _add(o, p.ap_from_o)
        b = _add(a, p.b_from_a)
        bp = _add(a, p.bp_from_a)
        c = _add(b, PointXZ(p.bc_length_mm, 0.0))
        cp = _add(b, p.cp_from_b)
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
        """Return the fixed +Y flexion-axis convention, not a design variable."""

        return self.FLEXION_AXIS_XYZ


def _add(first: PointXZ, second: PointXZ) -> PointXZ:
    return PointXZ(first.x_mm + second.x_mm, first.z_mm + second.z_mm)


__all__ = ["FingerLinkage", "ReferencePositions"]
