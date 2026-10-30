"""Contact module accommodation of one fixed finger (fixed architecture
versatility): how far it flexes with given proximal and distal modules.

A module is a pad block: its length along its segment and its thickness
below the segment's mounting surface, ``(l_p, h_p)`` proximal and
``(l_d, h_d)`` distal. The finger is held fixed while the modules change
(:func:`with_modules`): its pivots, links and servo, and its supports, the
mounting lines to ``A''`` and ``B''`` and the distal carrier over its whole
line. Each module keeps its outer end (the proximal one at the MCP servo's
clearance, ``Packaging.proximal_pad_start_mm``; the distal one at the
fingertip) and reaches toward the joint as far as its length; one longer
than its support does not mount. (The architecture capability, the shortest
finger re-optimized for each module pair, is the accommodation study's.)

:func:`phi_max_deg` is the finger's continuous reach: from extension, the
last pose before any check fails (pad against pad, servo or carrier; the
linkage's closure and transmission range; the fingertip behind ``O``), the
evaluator's ``phi_max``; the distal carrier's flexion relative to the
proximal one, not the servo's turn. A module the finger cannot mount (on its
support, off its own pins and housing, under the dorsal space) gives NaN.
The faces' loss in the fold is not counted here: a module either fits
through the motion or it does not.
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from core.optimization.mechanism import Mechanism, mechanism, read_config_file

MODULE_OVERRIDES = ("goal.pads.proximal.length_mm=[0, null]", "goal.pads.distal.length_mm=[0, null]",
                    "goal.loss.integrity_depth_mm=null", "goal.loss.fold=false")
"""What a module sweep sets on any goal: no least pad (the modules set it),
no loss rule (a module fits or not)."""


@dataclass(frozen=True)
class Modules:
    """A proximal and a distal module (mm)."""

    proximal_length_mm: float
    proximal_height_mm: float
    distal_length_mm: float
    distal_height_mm: float


def load(path: str | Path, *overrides: str):
    """A config file's mechanism and config, for module sweeps (``MODULE_OVERRIDES``)."""

    values = read_config_file(path, (*MODULE_OVERRIDES, *overrides))
    found = mechanism(values.pop("mechanism"))
    values.pop("output", None)
    values, _ = found.split_settings(values)
    return found, found.config_from_mapping(values)


def goal_of(config):
    return getattr(config, "goal", config)


def with_modules(config, modules: Modules):
    """``config`` with its pads ``modules``, each of its set length and thickness
    on the design's own support (its least length the module's too)."""

    goal = goal_of(config)
    packaging = goal.packaging
    pads = {side: dataclasses.replace(getattr(packaging, f"{side}_pad"), length_mm=length, max_length_mm=length,
                                      height_mm=height)
            for side, length, height in (("proximal", modules.proximal_length_mm, modules.proximal_height_mm),
                                         ("distal", modules.distal_length_mm, modules.distal_height_mm))}
    goal = dataclasses.replace(goal, packaging=dataclasses.replace(packaging, proximal_pad=pads["proximal"],
                                                                     distal_pad=pads["distal"]),
                               minimum_proximal_support_mm=modules.proximal_length_mm,
                               minimum_distal_support_mm=modules.distal_length_mm)
    return dataclasses.replace(config, goal=goal) if hasattr(config, "goal") else goal


MOUNTING = ("proximal_support_mm", "distal_support_mm", "dorsal_space_mm", "pip_servo_pad_mm", "mcp_servo_mm",
            "pad_collision_mm", "pivot_pin_mm", "pin_spacing_mm", "extended_length_mm", "valid_reference",
            "a_pin_servo_mm", "pad_drop_mm", "roller_pad_mm")
"""Checks of the extended finger with its modules: failing one, it cannot carry them."""


def phi_max_deg(found: Mechanism, config, x: Sequence[float], modules: Modules) -> float:
    """How far (deg) the fixed finger ``x`` flexes, continuously from
    extension, carrying ``modules`` (NaN: it cannot mount them)."""

    evaluation = found.evaluate(x, with_modules(config, modules))
    if evaluation is None:
        return math.nan
    if any(evaluation.constraints.get(name, 0.0) > 0.0 for name in MOUNTING):
        return math.nan
    return math.degrees(evaluation.phi_max_rad)


def proximal_curve(found: Mechanism, config, x: Sequence[float], modules: Modules,
                   lengths_mm: Sequence[float]) -> np.ndarray:
    """``phi_max`` (deg) for each proximal module length, the rest of ``modules`` kept."""

    return np.array([phi_max_deg(found, config, x, dataclasses.replace(modules, proximal_length_mm=length))
                     for length in lengths_mm])


def distal_curve(found: Mechanism, config, x: Sequence[float], modules: Modules,
                 lengths_mm: Sequence[float]) -> np.ndarray:
    """``phi_max`` (deg) for each distal module length, the rest of ``modules`` kept."""

    return np.array([phi_max_deg(found, config, x, dataclasses.replace(modules, distal_length_mm=length))
                     for length in lengths_mm])


__all__ = ["MODULE_OVERRIDES", "MOUNTING", "Modules", "distal_curve", "goal_of", "load", "phi_max_deg",
           "proximal_curve", "with_modules"]
