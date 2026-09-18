"""Minimal actuator hardware representations."""

from .actuator import (
    Actuator,
    MechanicalParameters,
    RectangularBox,
    ShaftLocation,
    Vector3,
)
from .xm335 import XM335Actuator

__all__ = [
    "Actuator",
    "MechanicalParameters",
    "RectangularBox",
    "ShaftLocation",
    "Vector3",
    "XM335Actuator",
]
