"""The rolling contact joint (RCJ) finger: its design vector, solids and their
gaps (the retention study's RCJ is ``core.morphology.mechanism.rolling_contact_joint``)."""

from core.morphology.mechanism.rcj.solids import TORQUE_RATIO, VARIABLE_NAMES, RcjGeometry, geometry, margins

__all__ = ["TORQUE_RATIO", "VARIABLE_NAMES", "RcjGeometry", "geometry", "margins"]
