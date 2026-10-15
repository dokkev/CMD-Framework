"""Stage-0 design vector: the FingerLinkageParameters fields, flattened.

Each variable is one component of a ``FingerLinkageParameters`` translation,
so search bounds read in the same frames as the morphology inputs (e.g. C'
relative to B). In the fully extended pose Stage-0 places A', B', B and C on
``z = 0`` (``FingerLinkage`` builds ``C = B + (bc_length_mm, 0)``), so OA',
B'B and BC all lie along +X. The ``z`` components of ``ap_from_o`` (0) and of
``b_from_a`` and ``bp_from_a`` (both ``-a_from_o.z``) are therefore derived
rather than searched; encoding a linkage that is not aligned this way projects
A', B' and B onto ``z = 0``.
"""

from __future__ import annotations

from collections.abc import Sequence

from core.experimental.linkage import FingerLinkage, FingerLinkageParameters, PointXZ


VARIABLE_NAMES: tuple[str, ...] = (
    "op_from_o_x_mm",
    "op_from_o_z_mm",
    "a_from_o_x_mm",
    "a_from_o_z_mm",
    "ap_from_o_x_mm",
    "b_from_a_x_mm",
    "bp_from_a_x_mm",
    "bc_length_mm",
    "cp_from_b_x_mm",
    "cp_from_b_z_mm",
)


def decode(x: Sequence[float]) -> FingerLinkageParameters:
    """Map an ordered design vector to linkage inputs with A', B', B, C on ``z = 0``."""

    if len(x) != len(VARIABLE_NAMES):
        raise ValueError(f"design vector has {len(x)} values; expected {len(VARIABLE_NAMES)}")
    op_x, op_z, a_x, a_z, ap_x, b_x, bp_x, bc, cp_x, cp_z = (float(v) for v in x)
    return FingerLinkageParameters(
        op_from_o=PointXZ(op_x, op_z),
        a_from_o=PointXZ(a_x, a_z),
        ap_from_o=PointXZ(ap_x, 0.0),
        b_from_a=PointXZ(b_x, -a_z),
        bp_from_a=PointXZ(bp_x, -a_z),
        bc_length_mm=bc,
        cp_from_b=PointXZ(cp_x, cp_z),
    )


def encode(linkage: FingerLinkage) -> tuple[float, ...]:
    """Return the design vector of a linkage, dropping the derived ``z`` components."""

    p = linkage.parameters
    return (
        p.op_from_o.x_mm, p.op_from_o.z_mm,
        p.a_from_o.x_mm, p.a_from_o.z_mm,
        p.ap_from_o.x_mm,
        p.b_from_a.x_mm,
        p.bp_from_a.x_mm,
        p.bc_length_mm,
        p.cp_from_b.x_mm, p.cp_from_b.z_mm,
    )


def bounds_around(
    linkage: FingerLinkage, half_width_mm: float
) -> dict[str, tuple[float, float]]:
    """Return box bounds of ``±half_width_mm`` centred on a linkage's design vector."""

    return {
        name: (value - half_width_mm, value + half_width_mm)
        for name, value in zip(VARIABLE_NAMES, encode(linkage))
    }


__all__ = ["VARIABLE_NAMES", "bounds_around", "decode", "encode"]
