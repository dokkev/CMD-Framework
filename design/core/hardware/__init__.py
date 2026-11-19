"""Minimal actuator hardware representations."""

from core.hardware.actuator import (
    ACTUATOR_SHAFT_AXIS_XYZ,
    Actuator,
    DShapedExtrusion,
    MechanicalParameters,
    Vector3,
)
from core.hardware.compact_servos import FeetechHD1910Actuator, UnitreeJ288Actuator, XC330Actuator
from core.hardware.xm335 import XM335Actuator

ACTUATORS = {
    "xm335": XM335Actuator,
    "xc330": XC330Actuator,
    "feetech_hd1910": FeetechHD1910Actuator,
    "unitree_j288": UnitreeJ288Actuator,
}
"""The actuators a config can name, by their config name."""


def actuator(name: str) -> Actuator:
    """The actuator a config names, e.g. ``"xm335"``."""

    try:
        return ACTUATORS[name]()
    except KeyError:
        raise ValueError(f"unknown actuator {name!r}; known: {sorted(ACTUATORS)}") from None


__all__ = [
    "ACTUATORS",
    "ACTUATOR_SHAFT_AXIS_XYZ",
    "Actuator",
    "actuator",
    "DShapedExtrusion",
    "MechanicalParameters",
    "Vector3",
    "XM335Actuator",
    "XC330Actuator",
    "FeetechHD1910Actuator",
    "UnitreeJ288Actuator",
]
