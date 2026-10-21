"""Forward kinematics of the equal-radius rolling contact joint.

The servo turns the carrier ``A-B`` by ``theta`` (positive flexion turns
``+X`` toward ``-Z``). With equal radii and ideal no-slip rolling the distal
roller turns twice as far: the distal phalanx flexes ``phi = 2 theta``, and
the contact point ``P`` stays midway between the roller centres.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from core._validation import require_finite
from core.morphology.linkage.parameters import PointXZ
from core.morphology.rolling_contact_joint.parameters import RollingContactJointParameters


@dataclass(frozen=True)
class RollingContactPose:
    """One solved pose of the RCJ."""

    q_carrier_rad: float
    """Carrier flexion from extension, ``theta``."""
    carrier_angle_rad: float
    """Carrier direction from ``+X`` (toward ``+Z``): ``alpha_0 - theta``."""
    distal_flexion_rad: float
    """Distal phalanx flexion from extension, ``phi = 2 theta``."""
    a: PointXZ
    b: PointXZ
    p: PointXZ
    c: PointXZ


@dataclass(frozen=True)
class RollingContactJoint:
    """Pose solver for one RCJ."""

    parameters: RollingContactJointParameters

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, RollingContactJointParameters):
            raise TypeError("parameters must be a RollingContactJointParameters instance")

    def solve(self, q_carrier_rad: float) -> RollingContactPose:
        """Return the pose for a carrier flexion ``theta``."""

        theta = require_finite("q_carrier_rad", q_carrier_rad)
        p = self.parameters
        alpha = p.carrier_angle_rad - theta
        phi = 2.0 * theta
        r = p.radius_mm
        a = p.a
        b = PointXZ(a.x_mm + 2.0 * r * math.cos(alpha), a.z_mm + 2.0 * r * math.sin(alpha))
        contact = PointXZ(a.x_mm + r * math.cos(alpha), a.z_mm + r * math.sin(alpha))
        # The distal phalanx points along +X at extension and flexes by phi.
        c = PointXZ(b.x_mm + p.bc_length_mm * math.cos(-phi), b.z_mm + p.bc_length_mm * math.sin(-phi))
        return RollingContactPose(theta, alpha, phi, a, b, contact, c)


__all__ = ["RollingContactJoint", "RollingContactPose"]
