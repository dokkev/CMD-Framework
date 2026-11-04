"""The serial 2R (revolute) finger on the linkage study's goal, as a pluggable
mechanism (``revolute``): the same config file, objectives and report columns
as ``linkage``, so the two fronts plot on the same axes."""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Mapping, Sequence
from typing import Any

from core.morphology.icr import icr0_share
from core.optimization.linkage.config import config_from_goal, config_from_json, config_to_json
from core.optimization.linkage.mechanism import MECHANISM as LINKAGE, OBJECTIVES, bound_hits_of
from core.optimization.mechanism import Mechanism
from core.optimization.optimizer import OptimizerSettings
from core.optimization.revolute import search
from core.optimization.revolute.config import VARIABLE_NAMES, RevoluteConfig
from core.optimization.revolute.evaluate import CONSTRAINT_NAMES, RevoluteEvaluation, evaluate


def bound_hits(x: Sequence[float], config: RevoluteConfig, finger_length_mm: float) -> tuple[str, ...]:
    """The variables near a bound of the search box, and ``|OC|`` near its bounds."""

    return bound_hits_of(dict(zip(VARIABLE_NAMES, x)), config.bounds, finger_length_mm,
                         config.goal.extended_length_bounds_mm)


class RevoluteMechanism(Mechanism):
    name = "revolute"
    formulation = "revolute-goal-v11"
    """The 2R on the linkage's goal (2026-10-26): direct drive at ``A``
    (``k = 1``), the distal servo on ``A`` laid back along ``-X``;
    v2: the distal pad's drop searched, the distal phalanx a truss of links;
    v3 (2026-10-27): the pads off the joint's housing, each on its own
    segment, the proximal pad's drop and the servo's tilt searched;
    v4 (2026-10-28): ``|OA|`` and ``BC`` held to the goal's shares of ``|OC|``
    (``goal.link.*_ratio``), as the linkage's; the faces less what the other
    hides in the fold, the distal pad's carrier against the servo and the
    hyperextension checked (the linkage's v4 rules); v5 (2026-10-29):
    the integrity rule (``goal.loss.integrity_depth_mm``), as the linkage's v5;
    v6 (2026-10-29): no setback: each pad reaches the joint's housing and past it by its overhang (``goal.pads.*.overhang_max_mm``), the housing a collision (``pad_reach_mm``). v7 (2026-11-03), as the linkage's v7 and v8: each pad its contact
    interface space, starting on the line tangent to the joint's housing
    (``evaluate.interfaced``; the drops no longer searched), no carrier, each
    pad counted at the required flexion to where the other's face line
    crosses it and none mounted past that (``pad_waste_mm``), the segment
    ratios without maxima. v8 (2026-11-03): the segment ratios on the
    segments ``|OA''|`` and ``|B''C|`` (``evaluate.segment_lengths``), as the
    linkage's, not on ``|OA|`` and ``|AC|``, which they no longer narrow.
    v9 (2026-11-04): a contact module's least length, not the segment's least share, bounds its segment from below; the share gives way and is reported (``*_ratio_shortfall_mm``).
    v10 (2026-11-04): each pad's mounting offset below its segment's line (the one datum) a design variable on the goal's range, the same for every mechanism (``goal.pads.*.mount_offset_mm``); the joints' hardware collisions at any offset.
    v11 (2026-11-04): the constraint ``mount_depth_mm`` (the pads' depths against ``goal.pads.mount_policy``: 0 when searched, the production's, whose designs are unchanged; ``shallowest_supported`` sets each depth, the study's, ``core.optimization.mounting``)."""
    variable_names = VARIABLE_NAMES
    constraint_names = CONSTRAINT_NAMES
    objectives = OBJECTIVES
    summary = LINKAGE.summary

    def split_settings(self, values: Mapping[str, Any]) -> tuple[dict[str, Any], OptimizerSettings]:
        return LINKAGE.split_settings(values)

    def config_from_mapping(self, values: Mapping[str, Any]) -> RevoluteConfig:
        return RevoluteConfig.from_goal(config_from_goal(values))

    def config_to_json(self, config: RevoluteConfig) -> dict[str, Any]:
        return {"goal": config_to_json(config.goal), "bounds": dict(config.bounds),
                "search_bounds": dict(config.search_bounds), "joint_servo": config.joint_servo,
                "joint_pulley": config.joint_pulley}

    def config_from_json(self, values: dict[str, Any]) -> RevoluteConfig:
        # Runs before 2026-10-26 also searched the chamfers, now gone (plain blocks).
        bounds = {name: tuple(pair) for name, pair in values["bounds"].items() if name in VARIABLE_NAMES}
        return RevoluteConfig(config_from_json(values["goal"]), bounds, values.get("joint_servo", True),
                              values.get("joint_pulley", False))

    # Searched in core.optimization.revolute.search's coordinates.

    def search_names(self, config: RevoluteConfig) -> tuple[str, ...]:
        return search.search_names(config)

    def search_box(self, config: RevoluteConfig) -> tuple[Sequence[float], Sequence[float]]:
        return search.search_box(config)

    def search_steps(self, config: RevoluteConfig) -> tuple[float, ...]:
        return search.search_steps(config)

    def to_design(self, coordinates: Sequence[float], config: RevoluteConfig) -> tuple[float, ...] | None:
        return search.to_design(coordinates, config)

    def prepare(self, config: RevoluteConfig) -> None:
        pass  # search.least_a_x is cached per tilt as it is met

    def from_design(self, x: Sequence[float], config: RevoluteConfig) -> tuple[float, ...]:
        return search.from_design(x, config)

    def refine(self, config: RevoluteConfig, step_rad: float) -> RevoluteConfig:
        return dataclasses.replace(config, goal=dataclasses.replace(config.goal, servo_step_rad=step_rad))

    def evaluate(self, x: Sequence[float], config: RevoluteConfig) -> RevoluteEvaluation | None:
        evaluation = evaluate(x, config)
        return dataclasses.replace(evaluation, bound_hits=bound_hits(x, config, evaluation.finger_length_mm))

    def report(self, e: RevoluteEvaluation) -> dict[str, Any]:
        return {
            "torque_ratio_max": e.torque_ratio_max,
            "pad_coverage": e.pad_coverage,
            "curl_deg": e.curl_deg,
            "proximal_ratio_shortfall_mm": e.proximal_ratio_shortfall_mm,
            "distal_ratio_shortfall_mm": e.distal_ratio_shortfall_mm,
            "proximal_effective_mm": e.proximal_effective_mm,
            "distal_effective_mm": e.distal_effective_mm,
            "mounted_coverage": e.mounted_coverage,
            "finger_length_mm": e.finger_length_mm,
            "icr0_share": icr0_share(self.name, e.x, self.variable_names, e.finger_length_mm),
            "proximal_room_mm": e.proximal_room_mm, "distal_room_mm": e.distal_room_mm,
            "support_balance_mm": e.support_balance_mm,
            "phi_max_deg": math.degrees(e.phi_max_rad),
            "safe_reach_deg": math.degrees(e.phi_safe_rad),
            "stop_reason": e.stop_reason,
            "collision_pair": e.collision_pair,
            "phi_max_stop": e.phi_max_stop,
            "proximal_support_length_mm": e.proximal_support_length_mm,
            "distal_support_length_mm": e.distal_support_length_mm,
            "proximal_usable_length_mm": e.proximal_usable_length_mm,
            "distal_usable_length_mm": e.distal_usable_length_mm,
            "proximal_depth_mm": e.proximal_depth_mm,
            "distal_depth_mm": e.distal_depth_mm,
            "pad_clearance_mm": e.pad_clearance_mm,
            "least_tip_x_mm": e.least_tip_x_mm,
            "proximal_length_mm": e.proximal_length_mm,
            "dip_flexion_deg": math.degrees(e.required_rom_rad),
            "bound_hits": ";".join(e.bound_hits),
        }

    def hypervolume_reference(self, config: RevoluteConfig) -> Mapping[str, float]:
        return LINKAGE.hypervolume_reference(config.goal)


class IdealizedRevoluteMechanism(RevoluteMechanism):
    name = "revolute_idealized"
    formulation = "revolute-idealized-goal-v10"
    """The 2R without its distal servo (2026-10-27): driven from elsewhere,
    only ``A``'s pin at the joint; the kinematic bound of a fixed pivot.
    v3 (2026-10-28): the goal's link shares, the fold, the carrier and the
    hyperextension, as the 2R's v4. v4 (2026-10-29): the integrity rule (``goal.loss.integrity_depth_mm``), as the linkage's v5.
    v5 (2026-10-29): no setback: each pad reaches the joint's housing and past it by its overhang (``goal.pads.*.overhang_max_mm``), the housing a collision (``pad_reach_mm``), the proximal pad checked against the pin.
    v6 (2026-11-03): the 2R's v7, the pads tangent to the pin's housing. v7: the 2R's v8. v8: its v9. v9: its v10. v10: its v11."""

    def config_from_mapping(self, values: Mapping[str, Any]) -> RevoluteConfig:
        return RevoluteConfig.from_goal(config_from_goal(values), joint_servo=False)


MECHANISM = RevoluteMechanism()
IDEALIZED = IdealizedRevoluteMechanism()


__all__ = ["IDEALIZED", "MECHANISM", "IdealizedRevoluteMechanism", "RevoluteMechanism", "bound_hits"]
