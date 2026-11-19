"""The serial 2R (revolute) finger on the linkage study's goal, as a pluggable
mechanism (``revolute``): the same config file, objectives and report columns
as ``linkage``, so the two fronts plot on the same axes."""

from __future__ import annotations

import dataclasses
from collections.abc import Mapping, Sequence
from typing import Any

from core.morphology.icr import icr0_share
from core.optimization.linkage.config import config_from_goal, config_from_json, config_to_json
from core.optimization.linkage.mechanism import MECHANISM as LINKAGE, OBJECTIVES, bound_hits_of
from core.optimization.mechanism import Mechanism, evaluation_report
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
    formulation = "two-r-dd-pf1"
    """The 2R-DD (servo on the joint ``A``, ``MA = 1``) under the problem
    formulation (2026-11-18, ``design/SSoT/problem_formulation.md``), as the
    CFL's: ``F_static`` / ``F_motion`` on the uniform ``q_2`` grid, the mean
    cylinder-based coverage over ``Q_eval``. Earlier formulations
    (``revolute-goal-v1`` to ``v12``) are in ``docs/decisions.md``."""
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
        return dataclasses.replace(config, goal=dataclasses.replace(config.goal, flexion_step_rad=step_rad))

    def evaluate(self, x: Sequence[float], config: RevoluteConfig) -> RevoluteEvaluation | None:
        evaluation = evaluate(x, config)
        return dataclasses.replace(evaluation, bound_hits=bound_hits(x, config, evaluation.finger_length_mm))

    def report(self, e: RevoluteEvaluation) -> dict[str, Any]:
        return {**evaluation_report(e), "icr0_share": icr0_share(self.name, e.x, self.variable_names, e.finger_length_mm)}

    def hypervolume_reference(self, config: RevoluteConfig) -> Mapping[str, float]:
        return LINKAGE.hypervolume_reference(config.goal)


class IdealizedRevoluteMechanism(RevoluteMechanism):
    name = "revolute_idealized"
    formulation = "two-r-idealized-pf1"
    """The 2R without its distal servo (an upper bound, problem formulation A.6.1),
    under the problem formulation as the 2R-DD's."""

    def config_from_mapping(self, values: Mapping[str, Any]) -> RevoluteConfig:
        return RevoluteConfig.from_goal(config_from_goal(values), joint_servo=False)


MECHANISM = RevoluteMechanism()
IDEALIZED = IdealizedRevoluteMechanism()


__all__ = ["IDEALIZED", "MECHANISM", "IdealizedRevoluteMechanism", "RevoluteMechanism", "bound_hits"]
