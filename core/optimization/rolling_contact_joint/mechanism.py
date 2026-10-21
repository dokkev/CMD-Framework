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

import math
from collections.abc import Mapping

import numpy as np

from core.morphology.linkage.packaging import ServoFootprint
from core.morphology.linkage.parameters import PointXZ
from core.morphology.rolling_contact_joint import RollingContactJoint, RollingContactJointParameters
from core.optimization.retention.config import RetentionConfig
from core.optimization.retention.geometry import MCP_SERVO, Disk, Housing, Module, Polygon, Segment, place
from core.optimization.retention.mechanism import Pair, RetentionMechanism, Solids


PAIRS = (
    Pair("proximal_module", "distal_module"),
    Pair("proximal_module", "distal_roller"),
    Pair("proximal_module", "distal_link"),
    Pair("proximal_module", "carrier"),
    Pair("proximal_module", "proximal_roller", "none"),
    Pair("proximal_module", "a_servo", "none"),
    Pair("proximal_module", "o_pin"),
    Pair("proximal_module", MCP_SERVO, "mcp", fixed=True),
    Pair("distal_module", "proximal_roller"),
    Pair("distal_module", "proximal_link"),
    Pair("distal_module", "carrier"),
    Pair("distal_module", "a_servo"),
    Pair("distal_module", "o_pin"),
    Pair("distal_module", "distal_roller", "none"),
    Pair("distal_module", MCP_SERVO, "mcp"),
    Pair("distal_roller", "proximal_link"),
    Pair("distal_roller", "a_servo"),
    Pair("distal_roller", MCP_SERVO),
    Pair("distal_link", "proximal_roller"),
    Pair("distal_link", "o_pin"),
    Pair("distal_link", MCP_SERVO),
    Pair("carrier", MCP_SERVO),
    Pair("a_servo", MCP_SERVO),
)
"""Solids of different bodies, and each module against the hardware of its own
body (they may touch, not overlap). Left out as the joint itself: the rollers'
contact, the carrier inside both rollers and on the servo's horn, and each
centreline at its own roller."""


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

        return RollingContactJointParameters(
            radius_mm=geometry["radius_mm"], bc_length_mm=geometry["bc_length_mm"],
            a=PointXZ(geometry["a_x_mm"], geometry["a_z_mm"]),
            carrier_angle_rad=math.radians(geometry["carrier_angle_deg"]),
            cable_allowance_mm=config.cable_allowance_mm,
            servo_radius_mm=config.packaging.pip_servo.radius_mm,
        )

    def solids(self, geometry, modules: tuple[Module, Module], config: RetentionConfig) -> Solids:
        parameters = self.parameters(geometry, config)
        joint = RollingContactJoint(parameters)
        poses = [joint.solve(phi / 2.0) for phi in config.rom_samples_rad]
        count, r = len(poses), parameters.radius_mm
        a = np.array([(p.a.x_mm, p.a.z_mm) for p in poses])
        b = np.array([(p.b.x_mm, p.b.z_mm) for p in poses])
        c = np.array([(p.c.x_mm, p.c.z_mm) for p in poses])
        proximal, distal = modules
        o = np.zeros((count, 2))
        servo = config.packaging.pip_servo
        solids = {
            "proximal_module": Polygon(np.broadcast_to(proximal.outline(), (count, 4, 2))),
            "proximal_link": Segment(o, a),
            "proximal_roller": Disk(a, r),
            "a_servo": Housing(ServoFootprint((float(a[0, 0]), float(a[0, 1])), config.joint_servo_rear,
                                              servo.radius_mm, servo.rear_extent_mm)),
            "o_pin": Disk(o, config.packaging.pin_radius_mm),
            "carrier": Segment(a, b),
            "distal_roller": Disk(b, r),
            "distal_module": Polygon(place(distal.outline(), b, (c - b) / parameters.bc_length_mm)),
            "distal_link": Segment(b, c),
        }
        return Solids(solids, proximal, distal, float(np.linalg.norm(c[0])))


MECHANISM = RollingContactJointMechanism()


__all__ = ["MECHANISM", "PAIRS", "RollingContactJointMechanism"]
