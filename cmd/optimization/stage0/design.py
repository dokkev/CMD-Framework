"""Stage-0 design vector: twelve absolute reference coordinates in the XZ plane.

B and C are placed on the distal O-C ray (``z = 0``), which satisfies the
fully extended reference condition by construction. Encoding a linkage whose
B lies slightly off that ray projects B onto it.
"""

from __future__ import annotations

from collections.abc import Sequence

from cmd.morphology.linkage import FingerLinkage, FingerLinkageParameters, PointXZ


VARIABLE_NAMES: tuple[str, ...] = (
    "o_prime_from_o_x_mm",
    "o_prime_from_o_z_mm",
    "a_from_o_x_mm",
    "a_from_o_z_mm",
    "a_prime_from_o_x_mm",
    "a_prime_from_o_z_mm",
    "b_from_o_x_mm",
    "b_prime_from_o_x_mm",
    "b_prime_from_o_z_mm",
    "bc_length_mm",
    "c_prime_from_o_x_mm",
    "c_prime_from_o_z_mm",
)


def decode(x: Sequence[float]) -> FingerLinkageParameters:
    """Map an ordered design vector to canonical relative linkage inputs."""

    if len(x) != len(VARIABLE_NAMES):
        raise ValueError(f"design vector has {len(x)} values; expected {len(VARIABLE_NAMES)}")
    op_x, op_z, a_x, a_z, ap_x, ap_z, b_x, bp_x, bp_z, bc, cp_x, cp_z = (float(v) for v in x)
    return FingerLinkageParameters(
        o_prime_from_o=PointXZ(op_x, op_z),
        a_from_o=PointXZ(a_x, a_z),
        a_prime_from_o=PointXZ(ap_x, ap_z),
        b_from_a=PointXZ(b_x - a_x, -a_z),
        b_prime_from_a=PointXZ(bp_x - a_x, bp_z - a_z),
        bc_length_mm=bc,
        c_prime_from_b=PointXZ(cp_x - b_x, cp_z),
    )


def encode(linkage: FingerLinkage) -> tuple[float, ...]:
    """Return the design vector of a linkage, projecting B onto ``z = 0``."""

    r = linkage.reference_positions
    return (
        r.op.x_mm, r.op.z_mm,
        r.a.x_mm, r.a.z_mm,
        r.ap.x_mm, r.ap.z_mm,
        r.b.x_mm,
        r.bp.x_mm, r.bp.z_mm,
        linkage.parameters.bc_length_mm,
        r.cp.x_mm, r.cp.z_mm,
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
