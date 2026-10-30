"""The serial 2R driven from a servo on ``A'`` through a belt: the 2R's
solids (``core.morphology.revolute``, its joint ``A`` holding only its pin)
with the servo and the drive pulley on ``A' = A + (dx, dz)``.

The servo's body is laid back along ``-X`` tilted dorsally (as the
linkage's); a belt from its pulley turns the driven pulley on ``A``, fixed
to the distal segment, by the same angle: ``phi = theta``, so ``k = 1``. The
belt and its pulleys run in the outer layer (meeting neither pad).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from core.morphology.packaging.gaps import footprint_gaps
from core.morphology.packaging.hardware import Packaging
from core.morphology.packaging.shapes import ServoFootprint
from core.morphology.revolute.solids import (
    VARIABLE_NAMES as REVOLUTE_NAMES,
    JointChecks,
    RevoluteGeometry,
    geometry as revolute_geometry,
    margins as revolute_margins,
)


VARIABLE_NAMES = (*REVOLUTE_NAMES, "ap_from_a_x_mm", "ap_from_a_z_mm")
"""The 2R's (``|OA|``, ``|AC|``, the pads' ends and drops, the servo's tilt,
here the servo on ``A'``), then the servo shaft ``A'`` from ``A``."""


@dataclass(frozen=True)
class ParallelGeometry:
    """The 2R's solids (``joint``) with the servo and the drive pulley on ``A'``."""

    joint: RevoluteGeometry
    ap: np.ndarray
    servo: ServoFootprint
    pulley_radius_mm: float

    def __getattr__(self, name: str):  # the 2R's solids and poses
        return getattr(self.joint, name)

    @property
    def a(self) -> np.ndarray:
        return np.array((self.joint.a_x_mm, 0.0))


def geometry(x: Sequence[float], packaging: Packaging, pulley_radius_mm: float,
             proximal_carrier_height_mm: float | None = None) -> ParallelGeometry:
    """The solids of a design vector (``proximal_carrier_height_mm`` as the 2R's)."""

    v = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    joint = revolute_geometry(tuple(x[:len(REVOLUTE_NAMES)]), packaging, proximal_carrier_height_mm)
    ap = np.array((joint.a_x_mm + v["ap_from_a_x_mm"], v["ap_from_a_z_mm"]))
    servo = packaging.pip_servo
    tilt = math.radians(v["servo_tilt_deg"])
    footprint = ServoFootprint((float(ap[0]), float(ap[1])), (-math.cos(tilt), math.sin(tilt)),
                               servo.radius_mm, servo.rear_extent_mm)
    return ParallelGeometry(joint, ap, footprint, pulley_radius_mm)


def margins(geo: ParallelGeometry, phi: np.ndarray, checks: JointChecks,
            until_unsafe: bool = False) -> dict[tuple[str, str], np.ndarray]:
    """The idealized 2R's pairs (the pads, the joint, the MCP servo; ``checks``
    the joint's, no servo on it), then the servo against the distal pad and
    its carrier (the pulleys, in the outer layer, meet neither pad)."""

    packaging = checks.packaging
    clearance = packaging.body_clearance_mm
    joint = geo.joint
    distal = joint.distal_pad(phi)
    margins = {
        ("distal_pad", "pip_servo"): footprint_gaps(distal, geo.servo, packaging) - clearance,
    }
    if joint.distal_carrier_local is not None:
        margins[("distal_carrier", "pip_servo")] = (
            footprint_gaps(joint.posed(joint.distal_carrier_local, phi), geo.servo, packaging) - clearance)
    margins = {pair: np.where(np.abs(margin) < 1e-9, 0.0, margin) for pair, margin in margins.items()}
    return {**revolute_margins(joint, phi, checks, until_unsafe=until_unsafe), **margins}


__all__ = ["VARIABLE_NAMES", "ParallelGeometry", "geometry", "margins"]
