"""The CoDe hand's parametric model: independent parameters (``parameters``) and derived frames (``model``).

Joint angles are CCW positive about local +Z. Zero is the extended geometric reference, independent of
the CAD pose; ``core.cad.code_hand`` maps saved CAD observations into these coordinates.
"""

from core.morphology.code_hand.model import ClosureError, CoDeHand, HandFrames, four_bar
from core.morphology.code_hand.parameters import (
    DIGITS,
    FINGERS,
    FingerConfiguration,
    FingerParameters,
    HandConfiguration,
    HandLayout,
    HandParameters,
    Pose,
    ThumbConfiguration,
    ThumbParameters,
    load,
)

__all__ = [
    "DIGITS", "FINGERS", "ClosureError", "CoDeHand", "FingerConfiguration", "FingerParameters",
    "HandConfiguration", "HandFrames", "HandLayout", "HandParameters", "Pose", "ThumbConfiguration",
    "ThumbParameters", "four_bar", "load",
]
