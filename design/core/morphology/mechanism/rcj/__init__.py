"""The rolling contact joint (RCJ) finger: its design vector, solids and their gaps."""

from core.morphology.mechanism.rcj.model import RollingJoint, RollingPose, RollingSweep, decode, make_finger
from core.morphology.mechanism.rcj.solids import TORQUE_RATIO, VARIABLE_NAMES, RcjGeometry, geometry, margins

__all__ = [
    "TORQUE_RATIO",
    "VARIABLE_NAMES",
    "RcjGeometry",
    "RollingJoint",
    "RollingPose",
    "RollingSweep",
    "decode",
    "geometry",
    "make_finger",
    "margins",
]
