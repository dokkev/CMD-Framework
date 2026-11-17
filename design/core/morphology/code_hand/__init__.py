"""The CoDe hand's parametric model: independent parameters (``parameters``) and derived frames (``model``).

Joint angles are CCW positive about local +Z, flexion positive on every digit. Zero is the extended geometric
reference, independent of the CAD pose; ``core.cad.code_hand`` maps saved CAD observations into these
coordinates. Actuator coordinates (``CoDeHand.actuators``) start from the CAD mates' datum instead.
"""

from core.morphology.code_hand.model import (
    ClosureError,
    CoDeHand,
    HandFrames,
    PadBox,
    four_bar,
    pip_from_servo,
    pip_servo_deg,
)
from core.morphology.code_hand.parameters import (
    DIGITS,
    FINGERS,
    FingerConfiguration,
    FingerParameters,
    HandConfiguration,
    HandLayout,
    HandParameters,
    PadParameters,
    Pose,
    ThumbConfiguration,
    ThumbParameters,
    load,
)

__all__ = [
    "DIGITS", "FINGERS", "ClosureError", "CoDeHand", "FingerConfiguration", "FingerParameters",
    "HandConfiguration", "HandFrames", "HandLayout", "HandParameters", "PadBox", "PadParameters", "Pose", "ThumbConfiguration",
    "ThumbParameters", "four_bar", "load", "pip_from_servo", "pip_servo_deg",
]
