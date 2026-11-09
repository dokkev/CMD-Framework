"""The CoDe hand's parametric model: independent parameters (``parameters``) and derived frames (``model``).

The current CAD (Onshape ``hand-layout``) is the ground truth the defaults reproduce; see ``core.cad.code_hand``.
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
