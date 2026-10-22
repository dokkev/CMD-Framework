"""The four-bar linkage (PIP servo at ``A'``) as a pluggable mechanism."""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Mapping, Sequence
from typing import Any

from core.morphology.linkage import FingerLinkageParameters

from core.optimization.linkage.config import (
    FingerStudyConfig,
    config_from_json,
    config_from_mapping,
    config_to_json,
)
from core.optimization.linkage.design import VARIABLE_NAMES, decode, read_design
from core.optimization.linkage.evaluate import CONSTRAINT_NAMES, FingerEvaluation, evaluate
from core.optimization.linkage.objectives import OBJECTIVES
from core.optimization.mechanism import Mechanism, SummaryColumn


LINK_COLUMNS = ("b_built_x_mm", "b_built_z_mm", "bp_from_b_x_mm", "bp_from_b_z_mm",
                "apb_mm", "abp_mm")
"""As built: ``B`` from ``O`` in the extended pose, ``B'`` from ``B``, and the bars."""


def link_values(p: FingerLinkageParameters) -> tuple[float, ...]:
    return (
        p.b_x_mm, p.b_z_mm, p.bp.x_mm - p.b_x_mm, p.bp.z_mm - p.b_z_mm,
        math.hypot(p.b_x_mm - p.ap.x_mm, p.b_z_mm - p.ap.z_mm),
        math.hypot(p.bp.x_mm - p.a_x_mm, p.bp.z_mm - p.a_z_mm),
    )


def _pads(row: Mapping[str, Any]) -> str:
    return (f"{float(row['support_length_mm']):.1f} ({float(row['proximal_support_length_mm']):.1f} + "
            f"{float(row['distal_support_length_mm']):.1f})")


def _stop(row: Mapping[str, Any]) -> str:
    return " " + row["stop_reason"] + (f" ({row['collision_pair']})" if row["collision_pair"] else "")


class LinkageMechanism(Mechanism):
    name = "linkage"
    formulation = "palmar-pad-v3"
    """Contact pads palmar of ``O-A''`` and ``B''-C`` that end the safe range
    where they collide; the ``A`` pin envelope clears a virtual clearance disk
    at ``A'`` and the ``B'`` pin may cross the PIP servo body
    (``palmar-pad-v1`` checked both pins against the servo body). Runs
    without it used ``OA + BC`` (``oa-bc``); other values are earlier pad
    models (``support-structure-v1``, ``palmar-pad-v2``)."""
    legacy_formulation = "oa-bc"
    legacy_objectives = ("transmission", "support_length", "curl")
    variable_names = VARIABLE_NAMES
    constraint_names = CONSTRAINT_NAMES
    objectives = OBJECTIVES
    summary = (
        SummaryColumn("k_max", 6, lambda r: f"{float(r['torque_ratio_max']):.2f}"),
        SummaryColumn("k ratio", 7, lambda r: f"{float(r['torque_ratio_spread']):.2f}"),
        SummaryColumn("pad length (P + D) mm", 22, _pads),
        SummaryColumn("phi_safe", 9, lambda r: f"{float(r['phi_safe_deg']):.1f}°"),
        SummaryColumn("pad clr", 8, lambda r: f"{float(r['pad_clearance_mm']):.2f}mm"),
        SummaryColumn(" stop", 0, _stop),
    )
    verification_dip_tolerance_rad = math.radians(0.1)
    """DIP and driven-link backoff allowed when verifying on a fine grid."""

    def config_from_mapping(self, values: Mapping[str, Any]) -> FingerStudyConfig:
        return config_from_mapping(values)

    def config_to_json(self, config: FingerStudyConfig) -> dict[str, Any]:
        return config_to_json(config)

    def config_from_json(self, values: dict[str, Any]) -> FingerStudyConfig:
        return config_from_json(values)

    def search_box(self, config: FingerStudyConfig) -> tuple[Sequence[float], Sequence[float]]:
        return config.lower, config.upper

    def refine(self, config: FingerStudyConfig, step_rad: float) -> FingerStudyConfig:
        return dataclasses.replace(config, servo_step_rad=step_rad,
                                   dip_backoff_tolerance_rad=self.verification_dip_tolerance_rad)

    def evaluate(self, x: Sequence[float], config: FingerStudyConfig) -> FingerEvaluation | None:
        try:
            parameters = decode(x, config.link_length_resolution_mm)
        except ValueError:  # B not beyond A, or the rounded bars do not close
            return None
        return evaluate(parameters, config)

    def read_design(self, row: Mapping[str, str]) -> list[float]:
        return read_design(row)

    def report(self, e: FingerEvaluation) -> dict[str, Any]:
        return {
            "torque_ratio_max": e.torque_ratio_max,
            "torque_ratio_min": e.torque_ratio_min,
            "torque_ratio_spread": e.torque_ratio_spread,
            "usable_support_length_mm": e.usable_support_length_mm,
            "proximal_usable_length_mm": e.proximal_usable_length_mm,
            "distal_usable_length_mm": e.distal_usable_length_mm,
            "support_length_mm": e.support_length_mm,
            "proximal_support_length_mm": e.proximal_support_length_mm,
            "distal_support_length_mm": e.distal_support_length_mm,
            "phi_safe_deg": math.degrees(e.phi_safe_rad),
            "usable_pip_deg": math.degrees(e.usable_pip_rad),
            "stop_reason": e.stop_reason,
            "collision_pair": e.collision_pair,
            "pad_clearance_mm": e.pad_clearance_mm,
            "worst_transmission_quality": e.worst_transmission_quality,
            "toggle_margin_deg": math.degrees(e.toggle_margin_rad),
            "hyperextension_phi_deg": math.degrees(e.hyperextension_phi_rad),
            "min_margin_deg": math.degrees(e.min_margin_rad),
            "lever_max_mm": e.lever_max_mm,
            "servo_travel_deg": math.degrees(e.servo_travel_rad),
            "ab_mm": e.ab_mm,
            "curl_pip_deg": math.degrees(e.curl_pip_rad),
            "curl_dip_deg": math.degrees(e.curl_dip_rad),
            "curl_tip_x_mm": e.curl_tip_x_mm,
            "closes_to_deg": math.degrees(e.closes_to_rad),
            "middle_link_servo_clearance_mm": e.middle_link_servo_clearance_mm,
            "servo_lowest_z_mm": e.servo_lowest_z_mm,
            **dict(zip(LINK_COLUMNS, link_values(e.parameters))),
        }


MECHANISM = LinkageMechanism()


__all__ = ["LINK_COLUMNS", "MECHANISM", "LinkageMechanism", "link_values"]
