"""The serial 2R (revolute) finger on the linkage study's goal, as a pluggable
mechanism (``revolute``): the same config file, objectives and report columns
as ``linkage``, so the two fronts plot on the same axes."""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Mapping, Sequence
from typing import Any

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
    formulation = "revolute-goal-v4"
    """The 2R on the linkage's goal (2026-10-26): direct drive at ``A``
    (``k = 1``), the distal servo on ``A`` laid back along ``-X``;
    v2: the distal pad's drop searched, the distal phalanx a truss of links;
    v3 (2026-10-27): the pads off the joint's housing, each on its own
    segment, the proximal pad's drop and the servo's tilt searched;
    v4 (2026-10-28): ``|OA|`` and ``BC`` held to the goal's shares of ``|OC|``
    (``goal.link.*_ratio``), as the linkage's; the faces less what the other
    hides in the fold, the distal pad's carrier against the servo and the
    hyperextension checked (the linkage's v4 rules)."""
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
        return search.SEARCH_NAMES

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
            "proximal_effective_mm": e.proximal_effective_mm,
            "distal_effective_mm": e.distal_effective_mm,
            "mounted_coverage": e.mounted_coverage,
            "finger_length_mm": e.finger_length_mm,
            "phi_max_deg": math.degrees(e.phi_max_rad),
            "safe_reach_deg": math.degrees(e.phi_safe_rad),
            "stop_reason": e.stop_reason,
            "collision_pair": e.collision_pair,
            "phi_max_stop": e.phi_max_stop,
            "proximal_support_length_mm": e.proximal_support_length_mm,
            "distal_support_length_mm": e.distal_support_length_mm,
            "proximal_face_mm": e.proximal_face_mm,
            "distal_face_mm": e.distal_face_mm,
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
    formulation = "revolute-idealized-goal-v3"
    """The 2R without its distal servo (2026-10-27): driven from elsewhere,
    only ``A``'s pin at the joint; the kinematic bound of a fixed pivot.
    v3 (2026-10-28): the goal's link shares, the fold, the carrier and the
    hyperextension, as the 2R's v4."""

    def config_from_mapping(self, values: Mapping[str, Any]) -> RevoluteConfig:
        return RevoluteConfig.from_goal(config_from_goal(values), joint_servo=False)


MECHANISM = RevoluteMechanism()
IDEALIZED = IdealizedRevoluteMechanism()


__all__ = ["IDEALIZED", "MECHANISM", "IdealizedRevoluteMechanism", "RevoluteMechanism", "bound_hits"]
