"""Optimizer coordinates mapped to canonical linkage morphology parameters."""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar, Mapping

from cmd.morphology import FingerLinkage, FingerLinkageParameters, PointXZ


@dataclass(frozen=True)
class LinkageBaselineParameterization:
    """Use twelve explicit reference positions as optimization coordinates.

    The optimizer coordinates place B and C exactly on the distal O-C ray.
    Encoding a canonical linkage projects its tiny B-z reference residual to
    zero; :meth:`projection_error_mm` reports the largest point displacement
    this causes without mutating the canonical linkage.
    """

    VARIABLE_NAMES: ClassVar[tuple[str, ...]] = (
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

    @property
    def variable_names(self) -> tuple[str, ...]:
        return self.VARIABLE_NAMES

    def decode(self, named_values: Mapping[str, float]) -> FingerLinkageParameters:
        """Decode absolute reference point coordinates to relative morphology inputs."""

        if set(named_values) != set(self.variable_names):
            missing = set(self.variable_names) - set(named_values)
            unexpected = set(named_values) - set(self.variable_names)
            raise ValueError(
                f"design values mismatch; missing={sorted(missing)}, "
                f"unexpected={sorted(unexpected)}"
            )

        value = named_values.__getitem__
        o = PointXZ(0.0, 0.0)
        op = PointXZ(value("o_prime_from_o_x_mm"), value("o_prime_from_o_z_mm"))
        a = PointXZ(value("a_from_o_x_mm"), value("a_from_o_z_mm"))
        ap = PointXZ(
            value("a_prime_from_o_x_mm"),
            value("a_prime_from_o_z_mm"),
        )
        b = PointXZ(value("b_from_o_x_mm"), 0.0)
        bp = PointXZ(
            value("b_prime_from_o_x_mm"),
            value("b_prime_from_o_z_mm"),
        )
        cp = PointXZ(
            value("c_prime_from_o_x_mm"),
            value("c_prime_from_o_z_mm"),
        )

        return FingerLinkageParameters(
            o_prime_from_o=_difference(op, o),
            a_from_o=_difference(a, o),
            a_prime_from_o=_difference(ap, o),
            b_from_a=_difference(b, a),
            b_prime_from_a=_difference(bp, a),
            bc_length_mm=value("bc_length_mm"),
            c_prime_from_b=_difference(cp, b),
        )

    def encode(self, linkage: FingerLinkage) -> tuple[float, ...]:
        """Encode a linkage reference geometry, projecting B onto ``z = 0``."""

        if not isinstance(linkage, FingerLinkage):
            raise TypeError("linkage must be a FingerLinkage instance")
        reference = linkage.reference_positions
        return (
            reference.op.x_mm,
            reference.op.z_mm,
            reference.a.x_mm,
            reference.a.z_mm,
            reference.ap.x_mm,
            reference.ap.z_mm,
            reference.b.x_mm,
            reference.bp.x_mm,
            reference.bp.z_mm,
            linkage.parameters.bc_length_mm,
            reference.cp.x_mm,
            reference.cp.z_mm,
        )

    def projection_error_mm(self, linkage: FingerLinkage) -> float:
        """Return the maximum reference-point displacement after encode/decode."""

        encoded = self.encode(linkage)
        decoded = self.decode(dict(zip(self.variable_names, encoded)))
        projected = FingerLinkage(decoded).reference_positions
        original = linkage.reference_positions
        return max(
            getattr(original, name).distance_to(getattr(projected, name))
            for name in ("o", "op", "a", "ap", "b", "bp", "c", "cp")
        )


def _difference(first: PointXZ, second: PointXZ) -> PointXZ:
    return PointXZ(first.x_mm - second.x_mm, first.z_mm - second.z_mm)


__all__ = ["LinkageBaselineParameterization"]
