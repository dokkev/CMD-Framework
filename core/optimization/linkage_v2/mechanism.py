"""The linkage in formulation v2, as a pluggable mechanism (``linkage_v2``).

v1's design vector, machining grid and fine-grid verification; v2's
evaluation (``evaluate``), objectives and diagnostics.
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Mapping, Sequence
from types import MappingProxyType
from typing import Any

from core.optimization.linkage.config import config_from_json, config_from_mapping, config_to_json
from core.optimization.linkage.design import DEFAULT_VALUES, VARIABLE_NAMES, decode
from core.optimization.linkage.mechanism import LINK_COLUMNS, LinkageMechanism, link_values
from core.optimization.linkage_v2.config import ALIASES, LinkageV2Config
from core.optimization.linkage_v2.evaluate import CONSTRAINT_NAMES, LinkageV2Evaluation, evaluate
from core.optimization.mechanism import Objective, SummaryColumn


OBJECTIVES = MappingProxyType({
    "transmission": Objective(
        value=lambda e: e.torque_ratio_max, column="torque_ratio_max",
        label="worst torque ratio k_max on [0, phi_req]", role="best_transmission",
        larger_is_better=False),
    "finger_length": Objective(
        value=lambda e: e.finger_length_mm, column="finger_length_mm",
        label="finger length |OC| [mm]", role="shortest", larger_is_better=False),
})

BOUND_TOLERANCE = 0.005
"""A variable is at a bound within this share of its search range."""


def bound_hits(x: Sequence[float], config: LinkageV2Config, finger_length_mm: float) -> tuple[str, ...]:
    """The variables within :data:`BOUND_TOLERANCE` of a bound of the search box
    the user gave (``bounds``; fixed ones left out), and ``|OC|`` near its
    search limits, as ``name:low`` or ``name:high``."""

    values = dict(zip(VARIABLE_NAMES, x))
    values = {name: values.get(name, DEFAULT_VALUES.get(name)) for name in VARIABLE_NAMES}  # older vectors
    hits = []
    for name in VARIABLE_NAMES:
        low, high = config.bounds[name]
        width = high - low
        if width <= 0.0:
            continue
        if values[name] - low <= BOUND_TOLERANCE * width:
            hits.append(f"{name}:low")
        elif high - values[name] <= BOUND_TOLERANCE * width:
            hits.append(f"{name}:high")
    low, high = config.extended_length_bounds_mm
    if finger_length_mm - low <= BOUND_TOLERANCE * (high - low):
        hits.append("finger_length:low")
    elif high - finger_length_mm <= BOUND_TOLERANCE * (high - low):
        hits.append("finger_length:high")
    return tuple(hits)


class LinkageV2Mechanism(LinkageMechanism):
    name = "linkage_v2"
    formulation = "linkage-v2-rev2"
    """Pads as an input (least lengths), the required ROM a constraint on
    ``[0, phi_req + margin]``, ``k_max`` and ``|OC|`` minimized."""
    legacy_formulation = None
    legacy_objectives = None
    variable_names = VARIABLE_NAMES
    constraint_names = CONSTRAINT_NAMES
    objectives = OBJECTIVES
    summary = (
        SummaryColumn("k_max", 6, lambda r: f"{float(r['torque_ratio_max']):.3f}"),
        SummaryColumn("|OC| mm", 8, lambda r: f"{float(r['finger_length_mm']):.1f}"),
        SummaryColumn("pads P + D mm", 15, lambda r: f"{float(r['proximal_support_length_mm']):.1f} + "
                                                     f"{float(r['distal_support_length_mm']):.1f}"),
        SummaryColumn("phi_max", 8, lambda r: f"{float(r['phi_max_deg']):.1f}°"),
        SummaryColumn("pad clr", 8, lambda r: f"{float(r['pad_clearance_mm']):.2f}mm"),
        SummaryColumn(" bounds", 0, lambda r: f" {r['bound_hits'] or '-'}"),
    )

    def config_from_mapping(self, values: Mapping[str, Any]) -> LinkageV2Config:
        return config_from_mapping(values, LinkageV2Config, ALIASES)

    def config_to_json(self, config: LinkageV2Config) -> dict[str, Any]:
        values = config_to_json(config)
        values["hypervolume_reference"] = dict(config.hypervolume_reference)
        return values

    def config_from_json(self, values: dict[str, Any]) -> LinkageV2Config:
        return config_from_json(values, LinkageV2Config)

    def evaluate(self, x: Sequence[float], config: LinkageV2Config) -> LinkageV2Evaluation | None:
        try:
            parameters = decode(x, config.link_length_resolution_mm)
        except ValueError:
            return None
        evaluation = evaluate(parameters, config)
        return dataclasses.replace(evaluation, bound_hits=bound_hits(x, config, evaluation.finger_length_mm))

    def report(self, e: LinkageV2Evaluation) -> dict[str, Any]:
        return {
            "torque_ratio_max": e.torque_ratio_max,
            "torque_ratio_min": e.torque_ratio_min,
            "finger_length_mm": e.finger_length_mm,
            "required_rom_deg": math.degrees(e.required_rom_rad),
            "required_proximal_mm": e.required_proximal_mm,
            "required_distal_mm": e.required_distal_mm,
            "phi_max_deg": math.degrees(e.phi_max_rad),
            "safe_reach_deg": math.degrees(e.phi_safe_rad),
            "stop_reason": e.stop_reason,
            "collision_pair": e.collision_pair,
            "proximal_support_length_mm": e.proximal_support_length_mm,
            "distal_support_length_mm": e.distal_support_length_mm,
            "app_offset_used_mm": e.parameters.app_offset_mm,
            "bpp_offset_used_mm": e.parameters.bpp_offset_mm,
            "proximal_face_mm": e.proximal_face_mm,
            "distal_face_mm": e.distal_face_mm,
            "pad_clearance_mm": e.pad_clearance_mm,
            "min_margin_deg": math.degrees(e.min_margin_rad),
            "toggle_margin_deg": math.degrees(e.toggle_margin_rad),
            "hyperextension_phi_deg": math.degrees(e.hyperextension_phi_rad),
            "least_tip_x_mm": e.least_tip_x_mm,
            "servo_travel_deg": math.degrees(e.servo_travel_rad),
            "lever_max_mm": e.lever_max_mm,
            "middle_link_servo_clearance_mm": e.middle_link_servo_clearance_mm,
            "servo_lowest_z_mm": e.servo_lowest_z_mm,
            "bound_hits": ";".join(e.bound_hits),
            **dict(zip(LINK_COLUMNS, link_values(e.parameters))),
        }

    def hypervolume_reference(self, config: LinkageV2Config) -> Mapping[str, float]:
        return config.hypervolume_reference


MECHANISM = LinkageV2Mechanism()


__all__ = ["BOUND_TOLERANCE", "MECHANISM", "OBJECTIVES", "LinkageV2Mechanism", "bound_hits"]
