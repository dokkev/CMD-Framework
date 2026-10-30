"""Contact module accommodation of one fixed finger: how far it flexes with
given proximal and distal modules.

A module is a pad block: its length along its segment and its thickness
below the segment's mounting surface, ``(l_p, h_p)`` proximal and
``(l_d, h_d)`` distal. The finger (pivots, links, servo, carrier, mounting
lines) is held fixed while the modules change: each keeps its outer end
(the proximal one at the MCP servo's clearance, ``Packaging.proximal_pad_start_mm``;
the distal one at the fingertip) and grows toward the joint.

:func:`phi_max_deg` is the finger's continuous reach: from extension, the
last pose before any check fails (pad against pad, servo or carrier; the
linkage's closure and transmission range; the fingertip behind ``O``), the
evaluator's ``phi_max``; the distal carrier's flexion relative to the
proximal one, not the servo's turn. A module the finger cannot mount (off
its own pins and housing, within its overhang, under the dorsal space) gives
NaN. The faces' loss in the fold is not counted here: a module either fits
through the motion or it does not.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from core.optimization.mechanism import Mechanism, mechanism, read_config_file

MODULE_OVERRIDES = ("goal.pads.proximal.length_mm=[0, null]", "goal.pads.distal.length_mm=[0, null]",
                    "goal.loss.integrity_depth_mm=null")
"""What a module sweep sets on any goal: no least pad (the length is the
sweep's), no loss rule (a module fits or not)."""


@dataclass(frozen=True)
class Modules:
    """A proximal and a distal module (mm)."""

    proximal_length_mm: float
    proximal_height_mm: float
    distal_length_mm: float
    distal_height_mm: float


def load(path: str | Path, modules: Modules, *overrides: str):
    """A config file's mechanism and config, its pads' thicknesses ``modules``'
    (the lengths are set per design by :func:`with_modules`)."""

    values = read_config_file(path, (*MODULE_OVERRIDES,
                                     f"goal.pads.proximal.height_mm={modules.proximal_height_mm}",
                                     f"goal.pads.distal.height_mm={modules.distal_height_mm}", *overrides))
    found = mechanism(values.pop("mechanism"))
    values.pop("output", None)
    values, _ = found.split_settings(values)
    return found, found.config_from_mapping(values)


def goal_of(config):
    return getattr(config, "goal", config)


def with_modules(found: Mechanism, config, x: Sequence[float], modules: Modules) -> tuple[float, ...]:
    """The design ``x`` with its pads' inner ends set for ``modules``' lengths (outer ends kept)."""

    v = dict(zip(found.variable_names, (float(value) for value in x)))
    packaging = goal_of(config).packaging
    start, offset = packaging.proximal_pad_start_mm, packaging.distal_pad.offset_mm
    if found.name.startswith("revolute"):
        v["app_x_mm"] = start + modules.proximal_length_mm
        v["bpp_from_a_mm"] = v["ac_length_mm"] - offset - modules.distal_length_mm
    elif found.name == "linkage":
        # A'' and B'' from A' and B' (linkage.design); the distal pad along B-C.
        v["app_offset_mm"] = start + modules.proximal_length_mm - v["ap_x_mm"]
        v["bpp_offset_mm"] = v["bp_x_mm"] - (v["b_x_mm"] + v["bc_length_mm"] - offset - modules.distal_length_mm)
    else:
        raise ValueError(f"no module sweep for {found.name!r}")
    return tuple(v[name] for name in found.variable_names)


MOUNTING = ("proximal_support_mm", "distal_support_mm", "dorsal_space_mm", "pip_servo_pad_mm", "mcp_servo_mm",
            "pad_collision_mm", "pivot_pin_mm", "pin_spacing_mm", "extended_length_mm", "valid_reference",
            "a_pin_servo_mm", "pad_drop_mm", "roller_pad_mm")
"""Checks of the extended finger with its modules: failing one, it cannot carry them."""


def phi_max_deg(found: Mechanism, config, x: Sequence[float], modules: Modules) -> float:
    """How far (deg) the finger ``x`` flexes, continuously from extension,
    carrying ``modules`` (NaN: it cannot mount them)."""

    evaluation = found.evaluate(with_modules(found, config, x, modules), config)
    if evaluation is None:
        return math.nan
    if any(evaluation.constraints.get(name, 0.0) > 0.0 for name in MOUNTING):
        return math.nan
    return math.degrees(evaluation.phi_max_rad)


def proximal_curve(found: Mechanism, config, x: Sequence[float], modules: Modules,
                   lengths_mm: Sequence[float]) -> np.ndarray:
    """``phi_max`` (deg) for each proximal module length, the rest of ``modules`` kept."""

    return np.array([phi_max_deg(found, config, x, Modules(length, modules.proximal_height_mm,
                                                           modules.distal_length_mm, modules.distal_height_mm))
                     for length in lengths_mm])


def longest_proximal_mm(found: Mechanism, config, x: Sequence[float], modules: Modules, phi_deg: float,
                        lengths_mm: Sequence[float]) -> float:
    """The longest of ``lengths_mm`` whose proximal module the finger carries
    through ``phi_deg`` (NaN: none)."""

    reach = proximal_curve(found, config, x, modules, lengths_mm)
    fits = np.asarray(lengths_mm)[np.nan_to_num(reach, nan=-1.0) >= phi_deg - 1e-9]
    return float(fits.max()) if fits.size else math.nan


def design_vector(found: Mechanism, values: Mapping[str, float]) -> tuple[float, ...]:
    return tuple(float(values[name]) for name in found.variable_names)


__all__ = ["MODULE_OVERRIDES", "MOUNTING", "Modules", "design_vector", "goal_of", "load", "longest_proximal_mm",
           "phi_max_deg", "proximal_curve", "with_modules"]
