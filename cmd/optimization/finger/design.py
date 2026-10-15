"""Design vector of the 2-DoF finger study: A_x, B (x, z), A', B' and the PIP
servo's dorsal tilt (degrees, on the same 0.1 grid).

``BC`` (the fingertip length) is fixed by the study configuration.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from cmd.morphology.linkage import FingerLinkage, FingerLinkageParameters, PointXZ


VARIABLE_NAMES: tuple[str, ...] = (
    "a_x_mm",
    "b_x_mm",
    "b_z_mm",
    "ap_x_mm",
    "ap_z_mm",
    "bp_x_mm",
    "bp_z_mm",
    "servo_tilt_deg",
)


def decode(x: Sequence[float], bc_length_mm: float) -> FingerLinkageParameters:
    """Map an ordered design vector to linkage parameters."""

    if len(x) != len(VARIABLE_NAMES):
        raise ValueError(f"design vector has {len(x)} values; expected {len(VARIABLE_NAMES)}")
    a_x, b_x, b_z, ap_x, ap_z, bp_x, bp_z, tilt_deg = (float(value) for value in x)
    return FingerLinkageParameters(
        a_x_mm=a_x,
        b_x_mm=b_x,
        b_z_mm=b_z,
        ap=PointXZ(ap_x, ap_z),
        bp=PointXZ(bp_x, bp_z),
        bc_length_mm=bc_length_mm,
        pip_servo_tilt_rad=math.radians(tilt_deg),
    )


def encode(linkage: FingerLinkage) -> tuple[float, ...]:
    """Return the design vector of a linkage."""

    p = linkage.parameters
    return (p.a_x_mm, p.b_x_mm, p.b_z_mm, p.ap.x_mm, p.ap.z_mm, p.bp.x_mm, p.bp.z_mm,
            round(math.degrees(p.pip_servo_tilt_rad), 9))


__all__ = ["VARIABLE_NAMES", "decode", "encode"]
