"""Minimal actuator hardware representations."""

from core.hardware.actuator import (
    ACTUATOR_SHAFT_AXIS_XYZ,
    Actuator,
    DShapedExtrusion,
    MechanicalParameters,
    Vector3,
)
from core.hardware.xm335 import XM335Actuator

ACTUATORS = {"xm335": XM335Actuator}
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
]
