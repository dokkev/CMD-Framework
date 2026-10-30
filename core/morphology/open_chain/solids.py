"""The open-chain (serial 2R) finger's solids in the retention comparison,
idealized or packaged (``core.optimization.open_chain``).

The kinematics are :class:`~core.morphology.open_chain.OpenChainFinger`'s:
the MCP at ``O`` and the distal joint at ``A``, each segment's module palmar
of its centreline (the mounting line), the distal one anchored at ``EE``. The
structure is the two centrelines and the joint pin envelopes at ``O`` and
``A``. *Packaged* adds the XM335 housings, their output axes on the joints:
the MCP servo at ``O`` fixed to the palm (checked over the MCP flexion) and
the distal servo at ``A`` fixed to the proximal segment, rear body along
``joint_servo_rear``. *Idealized* leaves the housings out; nothing else differs.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np

from core.morphology.contact.retention import MCP_SERVO, Disk, Housing, Module, Pair, Polygon, Segment, Solids, place
from core.morphology.packaging.hardware import Packaging
from core.morphology.packaging.shapes import ServoFootprint
from core.morphology.open_chain.model import OpenChainFinger
from core.morphology.open_chain.parameters import OpenChainParameters


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


def solids(geometry: Mapping[str, float], modules: tuple[Module, Module], packaging: Packaging,
           rom_samples_rad: Sequence[float], joint_servo_rear: tuple[float, float], packaged: bool) -> Solids:
    """The finger's solids over the distal flexions ``rom_samples_rad`` (``geometry``:
    ``proximal_length_mm`` and ``distal_length_mm``)."""

    finger = OpenChainFinger(OpenChainParameters(
        proximal_length_mm=geometry["proximal_length_mm"], distal_length_mm=geometry["distal_length_mm"]))
    samples = rom_samples_rad
    poses = [finger.solve(0.0, q) for q in samples]
    count = len(poses)
    a = np.array([(p.a.x_mm, p.a.z_mm) for p in poses])
    ee = np.array([(p.ee.x_mm, p.ee.z_mm) for p in poses])
    direction = (ee - a) / geometry["distal_length_mm"]
    proximal, distal = modules
    pin = packaging.pin_radius_mm
    o = np.zeros((count, 2))
    solids = {
        "proximal_module": Polygon(np.broadcast_to(proximal.outline(), (count, 4, 2))),
        "proximal_link": Segment(o, a),
        "a_pin": Disk(a, pin),
        "o_pin": Disk(o, pin),
        "distal_module": Polygon(place(distal.outline(), a, direction)),
        "distal_link": Segment(a, ee),
    }
    if packaged:
        servo = packaging.pip_servo
        solids["a_servo"] = Housing(ServoFootprint(
            (float(a[0, 0]), float(a[0, 1])), joint_servo_rear, servo.radius_mm, servo.rear_extent_mm))
    return Solids(solids, proximal, distal, float(np.linalg.norm(ee[0])))


__all__ = ["HOUSING_PAIRS", "STRUCTURE_PAIRS", "solids"]
