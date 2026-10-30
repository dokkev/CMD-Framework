"""The 2-DoF polycentric finger linkage (CoDe): actuated MCP and PIP, coupled DIP.

A library: build a linkage from its parameters (or a design vector), give it
a packaging, then pose it, check it and measure it; nothing here knows a
study or a requirement (``core.optimization.linkage`` holds those).

- ``parameters``, ``model``: the four-bar and its kinematics (:class:`FingerLinkage`)
- ``design``: the design vector and the machined linkage it decodes to
- ``packaging``: the hardware placed on the four-bar (:class:`LinkagePackaging`),
  its rigid bodies and Y layers, and every collision check between them
- ``motion``: the motion swept by the PIP servo, how far it stays safe, what
  it measures on ``[0, phi]``
- ``measures``: what the extended finger measures (length, ICR, overhang, pads)
- ``pad_extension``: growing the pads along their links, the links kept

    linkage = FingerLinkage(decode(x))
    packaging = LinkagePackaging(distal_carrier_height_mm=4.0, overlap_depth=True)
    margins = pad_margins(linkage, packaging, b, c, bp)            # one per checked pair
    motion = sweep_motion(linkage, packaging, settings, math.radians(150.0))
"""

from core.morphology.linkage.parameters import FingerLinkageParameters, PointXZ
from core.morphology.linkage.model import FingerLinkage, FingerPose, KinematicClosureError, PipSweep
from core.morphology.linkage.design import DEFAULT_VALUES, VARIABLE_NAMES, decode, encode, read_design
from core.morphology.linkage.packaging import (
    JOINTS,
    LAYERS,
    PARTS,
    LinkagePackaging,
    ServoClearance,
    assembly,
    checked_pairs,
    layout,
    pad_margins,
    pip_footprint,
    poses,
    reference_poses,
    servo_clearance,
)
from core.morphology.linkage.measures import extended_icr_x, finger_length_mm, linkage_icr_x, overhang_mm, pad_lengths
from core.morphology.linkage.motion import FingerMotion, MotionSettings, servo_lever, servo_sense, sweep_motion
from core.morphology.linkage.pad_extension import extended, grow_pads
from core.morphology.linkage.seed import SKETCH_LINKAGE

__all__ = [
    "DEFAULT_VALUES",
    "JOINTS",
    "LAYERS",
    "PARTS",
    "SKETCH_LINKAGE",
    "VARIABLE_NAMES",
    "FingerLinkage",
    "FingerLinkageParameters",
    "FingerMotion",
    "FingerPose",
    "KinematicClosureError",
    "LinkagePackaging",
    "MotionSettings",
    "PipSweep",
    "PointXZ",
    "ServoClearance",
    "assembly",
    "checked_pairs",
    "decode",
    "encode",
    "extended",
    "extended_icr_x",
    "finger_length_mm",
    "grow_pads",
    "layout",
    "linkage_icr_x",
    "overhang_mm",
    "pad_lengths",
    "pad_margins",
    "pip_footprint",
    "poses",
    "read_design",
    "reference_poses",
    "servo_clearance",
    "servo_lever",
    "servo_sense",
    "sweep_motion",
]
