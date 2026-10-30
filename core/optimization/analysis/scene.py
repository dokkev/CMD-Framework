"""A study's design as a contact scene (``core.morphology.contact.loss``):
its mechanism's solids at a set of distal flexions, from its config."""

from __future__ import annotations

import importlib
import math
from collections.abc import Sequence

import numpy as np

from core.morphology.contact.loss import Scene, joint_scene, linkage_scene
from core.morphology.linkage import FingerLinkage
from core.morphology.linkage.design import decode
from core.morphology.linkage.motion import servo_sense


def scene(mechanism: str, config, x: Sequence[float], phi_deg: Sequence[float]) -> Scene:
    """The scene of a design ``x`` of ``mechanism`` (the comparison's) at
    distal flexions ``phi_deg`` (the linkage: the first sample of its servo
    sweep reaching each)."""

    phi_deg = np.atleast_1d(np.asarray(phi_deg, dtype=float))
    if mechanism == "linkage":
        return _linkage_scene(config, x, phi_deg)
    return _joint_scene(mechanism, config, x, phi_deg)


def _linkage_scene(config, x, phi_deg) -> Scene:
    linkage = FingerLinkage(decode(x, config.link_length_resolution_mm))
    step = math.radians(0.05)
    sweep = linkage.sweep_servo(servo_sense(linkage, step) * np.arange(0.0, math.pi, step))
    reached = np.maximum.accumulate(sweep.total_flexion_rad)
    index = np.minimum(np.searchsorted(reached, np.radians(phi_deg) - 1e-9), len(reached) - 1)
    return linkage_scene(linkage, config.packaging, sweep.b[index], sweep.c[index], sweep.bp[index],
                         np.degrees(sweep.total_flexion_rad[index]), config.proximal_carrier_height_mm)


def _joint_scene(mechanism: str, config, x, phi_deg) -> Scene:
    module = importlib.import_module(
        f"core.optimization.{'revolute' if mechanism == 'revolute_idealized' else mechanism}.evaluate")
    return joint_scene(mechanism, module.geometry(x, config), config.goal.packaging, phi_deg)


__all__ = ["scene"]
