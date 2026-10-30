"""The open-chain (serial 2R) finger as a retention mechanism, idealized or packaged.

Its solids are the morphology's (``core.morphology.open_chain.solids``), on
the same kinematics (``core.morphology.open_chain``):
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

from core.morphology.contact.retention import Module, Solids
from core.morphology.open_chain.solids import HOUSING_PAIRS, STRUCTURE_PAIRS, solids as open_chain_solids

from core.optimization.retention.config import RetentionConfig
from core.optimization.retention.mechanism import RetentionMechanism


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
        return open_chain_solids(geometry, modules, config.packaging, config.rom_samples_rad,
                                 config.joint_servo_rear, self.packaged)


IDEALIZED = OpenChainMechanism("open_chain_idealized", packaged=False)
PACKAGED = OpenChainMechanism("open_chain_packaged", packaged=True)


__all__ = ["HOUSING_PAIRS", "IDEALIZED", "PACKAGED", "STRUCTURE_PAIRS", "OpenChainMechanism"]
