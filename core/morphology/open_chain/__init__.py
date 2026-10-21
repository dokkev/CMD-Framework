"""The open-chain (serial 2R) finger: independently actuated MCP and distal FE joints."""

from core.morphology.open_chain.parameters import OpenChainParameters
from core.morphology.open_chain.model import FLEXION_AXIS, OpenChainFinger, OpenChainPose

__all__ = ["FLEXION_AXIS", "OpenChainFinger", "OpenChainParameters", "OpenChainPose"]
