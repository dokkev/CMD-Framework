"""The open-chain (serial 2R) finger as a retention mechanism, idealized or packaged.

Both use the same morphology and kinematics (``core.morphology.open_chain``):
the MCP at ``O`` and the distal joint at ``A``, each segment's module palmar
of its centreline (the mounting line), the distal one anchored at ``EE``. The
structure is the two centrelines and the joint pin envelopes at ``O`` and
``A``. *Packaged* adds the XM335 housings, their output axes on the joints:
the MCP servo at ``O`` fixed to the palm (checked over the MCP flexion) and
the distal servo at ``A`` fixed to the proximal segment, rear body along
``joint_servo_rear``. *Idealized* leaves the housings out; nothing else differs.

The distal servo drives the distal joint directly: one unit of servo torque
per unit of distal moment over the whole range.
"""

from __future__ import annotations

from collections.abc import Mapping

import numpy as np

from core.morphology.linkage.packaging import ServoFootprint
from core.morphology.open_chain import OpenChainFinger, OpenChainParameters
from core.optimization.retention.config import RetentionConfig
from core.optimization.retention.geometry import MCP_SERVO, Disk, Housing, Module, Polygon, Segment, place
from core.optimization.retention.mechanism import Pair, RetentionMechanism, Solids


STRUCTURE_PAIRS = (
    Pair("proximal_module", "distal_module"),
    Pair("proximal_module", "distal_link"),
    Pair("distal_module", "proximal_link"),
    Pair("distal_module", "a_pin"),
    Pair("distal_module", "o_pin"),
    Pair("proximal_module", "o_pin"),
    Pair("distal_link", "o_pin"),
)
"""Checked in both: the modules, the centrelines and the pin envelopes of
different bodies. A centreline and the pin of its own joint are the joint."""
HOUSING_PAIRS = (
    Pair("distal_module", "a_servo"),
    Pair("proximal_module", "a_servo", "none"),
    Pair("distal_module", MCP_SERVO, "mcp"),
    Pair("proximal_module", MCP_SERVO, "mcp", fixed=True),
    Pair("a_servo", MCP_SERVO),
    Pair("distal_link", MCP_SERVO),
)
"""Packaged only. The proximal module may not overlap the housing on its own
segment; the distal link starts at the distal servo's horn."""


class OpenChainMechanism(RetentionMechanism):
    geometry_names = ("proximal_length_mm", "distal_length_mm")

    def __init__(self, name: str, packaged: bool) -> None:
        self.name, self.packaged = name, packaged
        self.pairs = STRUCTURE_PAIRS + (HOUSING_PAIRS if packaged else ())

    def distal_length_mm(self, geometry: Mapping[str, float]) -> float:
        return geometry["distal_length_mm"]

    def torque_ratio(self, geometry, flexion_rad: np.ndarray) -> np.ndarray:
        return np.ones_like(flexion_rad)

    def solids(self, geometry, modules: tuple[Module, Module], config: RetentionConfig) -> Solids:
        finger = OpenChainFinger(OpenChainParameters(
            proximal_length_mm=geometry["proximal_length_mm"], distal_length_mm=geometry["distal_length_mm"]))
        samples = config.rom_samples_rad
        poses = [finger.solve(0.0, q) for q in samples]
        count = len(poses)
        a = np.array([(p.a.x_mm, p.a.z_mm) for p in poses])
        ee = np.array([(p.ee.x_mm, p.ee.z_mm) for p in poses])
        direction = (ee - a) / geometry["distal_length_mm"]
        proximal, distal = modules
        pin = config.packaging.pin_radius_mm
        o = np.zeros((count, 2))
        solids = {
            "proximal_module": Polygon(np.broadcast_to(proximal.outline(), (count, 4, 2))),
            "proximal_link": Segment(o, a),
            "a_pin": Disk(a, pin),
            "o_pin": Disk(o, pin),
            "distal_module": Polygon(place(distal.outline(), a, direction)),
            "distal_link": Segment(a, ee),
        }
        if self.packaged:
            servo = config.packaging.pip_servo
            solids["a_servo"] = Housing(ServoFootprint(
                (float(a[0, 0]), float(a[0, 1])), config.joint_servo_rear, servo.radius_mm, servo.rear_extent_mm))
        return Solids(solids, proximal, distal, float(np.linalg.norm(ee[0])))


IDEALIZED = OpenChainMechanism("open_chain_idealized", packaged=False)
PACKAGED = OpenChainMechanism("open_chain_packaged", packaged=True)


__all__ = ["HOUSING_PAIRS", "IDEALIZED", "PACKAGED", "STRUCTURE_PAIRS", "OpenChainMechanism"]
