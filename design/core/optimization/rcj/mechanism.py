"""The rolling contact joint (RCJ) finger on the linkage study's goal, as a
pluggable mechanism (``rcj``): the same config file, objectives and report
columns as ``linkage`` and ``revolute``, so the fronts plot on the same axes."""

from __future__ import annotations

import dataclasses
from collections.abc import Mapping, Sequence
from typing import Any

from core.optimization.linkage.goal import config_from_goal
from core.optimization.linkage.record import config_from_json, config_to_json
from core.optimization.linkage.mechanism import bound_hits_of
from core.optimization.rcj import search
from core.optimization.rcj.config import VARIABLE_NAMES, RcjConfig
from core.optimization.rcj.evaluate import CONSTRAINT_NAMES, RcjEvaluation, evaluate
from core.optimization.revolute.mechanism import RevoluteMechanism


def bound_hits(x: Sequence[float], config: RcjConfig, finger_length_mm: float) -> tuple[str, ...]:
    """The variables near a bound of the search box, and ``|OC|`` near its bounds."""

    return bound_hits_of(dict(zip(VARIABLE_NAMES, x)), config.bounds, finger_length_mm,
                         config.goal.extended_length_bounds_mm)


class RcjMechanism(RevoluteMechanism):
    name = "rcj"
    formulation = "rcj-pf1"
    """The symmetric RCJ (``MA = 1/2``) under the problem formulation (2026-11-18), as the CFL's."""
    variable_names = VARIABLE_NAMES
    constraint_names = CONSTRAINT_NAMES

    def config_from_mapping(self, values: Mapping[str, Any]) -> RcjConfig:
        return RcjConfig.from_goal(config_from_goal(values))

    def config_to_json(self, config: RcjConfig) -> dict[str, Any]:
        return {"goal": config_to_json(config.goal), "bounds": dict(config.bounds), "joint_servo": config.joint_servo}

    def config_from_json(self, values: dict[str, Any]) -> RcjConfig:
        return RcjConfig(config_from_json(values["goal"]), {name: tuple(pair) for name, pair in values["bounds"].items()},
                         values.get("joint_servo", True))

    def search_names(self, config: RcjConfig) -> tuple[str, ...]:
        return search.search_names(config)

    def search_box(self, config: RcjConfig) -> tuple[Sequence[float], Sequence[float]]:
        return search.search_box(config)

    def search_steps(self, config: RcjConfig) -> tuple[float, ...]:
        return search.search_steps(config)

    def to_design(self, coordinates: Sequence[float], config: RcjConfig) -> tuple[float, ...] | None:
        return search.to_design(coordinates, config)

    def prepare(self, config: RcjConfig) -> None:
        pass  # search.least_a_x is cached per tilt as it is met

    def read_design(self, row: Mapping[str, str]) -> list[float]:
        """A row's design vector; a row of a run before the tilt (rcj-goal-v6) was untilted."""

        return [float(row.get(name) or 0.0) if name == "servo_tilt_deg" else float(row[name])
                for name in self.variable_names]

    def from_design(self, x: Sequence[float], config: RcjConfig) -> tuple[float, ...]:
        return search.from_design(x, config)

    def evaluate(self, x: Sequence[float], config: RcjConfig) -> RcjEvaluation | None:
        evaluation = evaluate(x, config)
        return dataclasses.replace(evaluation, bound_hits=bound_hits(x, config, evaluation.finger_length_mm))

    def report(self, e: RcjEvaluation) -> dict[str, Any]:
        return {**super().report(e), "roller_radius_mm": e.roller_radius_mm}


class IdealizedRcjMechanism(RcjMechanism):
    name = "rcj_idealized"
    formulation = "rcj-idealized-pf1"
    """The RCJ without its joint servo (an upper bound), under the problem formulation."""

    def config_from_mapping(self, values: Mapping[str, Any]) -> RcjConfig:
        return RcjConfig.from_goal(config_from_goal(values), joint_servo=False)


MECHANISM = RcjMechanism()
IDEALIZED = IdealizedRcjMechanism()


__all__ = ["IDEALIZED", "MECHANISM", "IdealizedRcjMechanism", "RcjMechanism", "bound_hits"]
