"""The equal-radius rolling contact joint as a retention mechanism (packaged).

The ideal no-slip kinematics of ``core.morphology.rolling_contact_joint``:
the servo at the proximal roller's centre ``A`` turns the carrier ``A-B`` by
``theta`` and the distal phalanx ``B-C`` flexes ``phi = 2 theta``. The
proximal module hangs palmar of ``O-A''`` (``z = 0``, as the linkage's), the
distal one palmar of the line through ``B`` along ``BC``, anchored at ``C``.

Only the defined solids are checked, all in one plane: the two rollers
(radius ``r``; their rolling contact is the joint), the carrier and the
phalanx centrelines, the XM335 at ``A`` fixed to the proximal segment (rear
body along ``joint_servo_rear``), the MCP servo at ``O`` and the MCP pin. The
cable routing, bearing widths and Y layers are not defined yet, so they are
not modelled, and every candidate is provisional: its cables are not
shown feasible (``r >= servo radius + cable_allowance_mm`` is all that is
asked of them).

Servo torque per distal moment: ``d phi / d theta = 2``, over the whole range.
"""

from __future__ import annotations

from collections.abc import Mapping

import numpy as np

from core.morphology.contact.retention import Module, Solids
from core.morphology.rolling_contact_joint import RollingContactJointParameters
from core.morphology.rolling_contact_joint.solids import PAIRS, joint_parameters, solids as joint_solids

from core.optimization.retention.config import RetentionConfig
from core.optimization.retention.mechanism import RetentionMechanism


class RollingContactJointMechanism(RetentionMechanism):
    name = "rolling_contact_joint"
    geometry_names = ("a_x_mm", "a_z_mm", "radius_mm", "carrier_angle_deg", "bc_length_mm")
    pairs = PAIRS
    provisional = "cable routing unverified"

    def geometry_box(self, config: RetentionConfig) -> Mapping[str, tuple[float, float]]:
        box = dict(config.bounds)
        low, high = box["radius_mm"]
        least = config.packaging.pip_servo.radius_mm + config.cable_allowance_mm
        box["radius_mm"] = (min(max(low, least), high), high)
        return box

    def distal_length_mm(self, geometry: Mapping[str, float]) -> float:
        return geometry["bc_length_mm"]

    def torque_ratio(self, geometry, flexion_rad: np.ndarray) -> np.ndarray:
        return np.full_like(flexion_rad, 2.0)

    def servo_travel_rad(self, config: RetentionConfig) -> float:
        return config.required_rom_rad / 2.0

    def parameters(self, geometry: Mapping[str, float], config: RetentionConfig) -> RollingContactJointParameters:
        """``ValueError`` when the radius does not clear the servo and the cables."""

        return joint_parameters(geometry, config.cable_allowance_mm, config.packaging.pip_servo.radius_mm)

    def solids(self, geometry, modules: tuple[Module, Module], config: RetentionConfig) -> Solids:
        return joint_solids(geometry, modules, config.packaging, config.rom_samples_rad, config.joint_servo_rear,
                            config.cable_allowance_mm)


MECHANISM = RollingContactJointMechanism()


__all__ = ["MECHANISM", "PAIRS", "RollingContactJointMechanism"]
