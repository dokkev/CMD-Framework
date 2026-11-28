"""The serial 2R driven from a servo on ``A'`` through a belt, on the linkage
study's goal, as a pluggable mechanism (``revolute_parallel``): the same
config file, objectives and report columns as ``linkage`` and ``revolute``.

Searched in the 2R's coordinates (``core.optimization.revolute.search``) for
the joint, pads and the servo's tilt, then ``A'`` from ``A``, each on the
prototyping resolution.
"""

from __future__ import annotations

import dataclasses
from collections.abc import Mapping, Sequence
from typing import Any

from core.optimization.linkage.goal import config_from_goal
from core.optimization.linkage.record import config_from_json, config_to_json
from core.optimization.linkage.mechanism import bound_hits_of
from core.optimization.revolute import search as revolute_search
from core.optimization.revolute.config import RevoluteConfig
from core.optimization.revolute.mechanism import RevoluteMechanism
from core.optimization.revolute_parallel.config import VARIABLE_NAMES, ParallelConfig
from core.optimization.revolute_parallel.evaluate import CONSTRAINT_NAMES, ParallelEvaluation, evaluate

JOINT_VARIABLES = len(revolute_search.VARIABLE_NAMES)
"""The 2R's design variables come first (its drops among them), then ``A'`` from ``A``."""
OWN_NAMES = ("ap_from_a_x_mm", "ap_from_a_z_mm")
SEARCH_NAMES = (*revolute_search.SEARCH_NAMES, *OWN_NAMES)
"""The coordinates with the drops searched (no mounting policy)."""


def search_names(config: ParallelConfig) -> tuple[str, ...]:
    """The 2R's coordinates (without the drops under a mounting policy), then the belt's."""

    return (*revolute_search.search_names(config.base), *OWN_NAMES)


def bound_hits(x: Sequence[float], config: ParallelConfig, finger_length_mm: float) -> tuple[str, ...]:
    """The variables near a bound of the search box, and ``|OC|`` near its bounds."""

    return bound_hits_of(dict(zip(VARIABLE_NAMES, x)), config.bounds, finger_length_mm,
                         config.goal.extended_length_bounds_mm)


class ParallelMechanism(RevoluteMechanism):
    name = "revolute_parallel"
    formulation = "two-r-remote-pf1"
    """The 2R-remote (belt from a servo on ``A'``, ``MA = 1``) under the
    problem formulation (2026-11-18), as the CFL's; the belt and pulleys hide
    the faces they pass in front of with ``include_linkage_occlusion``."""
    variable_names = VARIABLE_NAMES
    constraint_names = CONSTRAINT_NAMES

    def config_from_mapping(self, values: Mapping[str, Any]) -> ParallelConfig:
        return ParallelConfig.from_goal(config_from_goal(values))

    def config_to_json(self, config: ParallelConfig) -> dict[str, Any]:
        return {"goal": config_to_json(config.goal), "bounds": dict(config.bounds),
                **({} if config.base.heights is None else {"heights": [list(pair) for pair in config.base.heights]})}

    def config_from_json(self, values: dict[str, Any]) -> ParallelConfig:
        goal = config_from_json(values["goal"])
        bounds = {name: tuple(pair) for name, pair in values["bounds"].items()}
        joint = RevoluteConfig.from_goal(goal, joint_servo=False)
        heights = values.get("heights")
        base = RevoluteConfig(goal, {**joint.bounds, "servo_tilt_deg": bounds["servo_tilt_deg"]}, False,
                              heights=None if heights is None else tuple(tuple(pair) for pair in heights))
        return ParallelConfig(base, bounds)

    def search_names(self, config: ParallelConfig) -> tuple[str, ...]:
        return search_names(config)

    def search_box(self, config: ParallelConfig) -> tuple[Sequence[float], Sequence[float]]:
        lower, upper = revolute_search.search_box(config.base)
        return ((*lower, *(config.bounds[name][0] for name in OWN_NAMES)),
                (*upper, *(config.bounds[name][1] for name in OWN_NAMES)))

    def search_steps(self, config: ParallelConfig) -> tuple[float, ...]:
        return (*revolute_search.search_steps(config.base), config.resolution_mm, config.resolution_mm)

    def to_design(self, coordinates: Sequence[float], config: ParallelConfig) -> tuple[float, ...] | None:
        joint_count = len(revolute_search.search_names(config.base))
        joint = revolute_search.to_design(coordinates[:joint_count], config.base)
        if joint is None:
            return None
        resolution = config.resolution_mm
        dx, dz = (round(round(float(value) / resolution) * resolution, 9)
                  for value in coordinates[joint_count:joint_count + 2])
        heights = tuple(joint[JOINT_VARIABLES:])  # the joint's heights, when searched, after A'
        x = (*joint[:JOINT_VARIABLES], dx, dz, *heights)
        if config.goal.mount_policy == "searched":
            return x
        # Each pad at the least depth A's pin and the servo on A' need, within what they carry.
        from core.optimization.mounting import parallel_mounting

        found = parallel_mounting(x, config)
        if not found.supported:
            return None
        design = dict(zip(VARIABLE_NAMES, x))
        design["proximal_drop_mm"], design["distal_drop_mm"] = found.depths_mm
        return tuple(design[name] for name in VARIABLE_NAMES) + heights

    def prepare(self, config: ParallelConfig) -> None:
        pass

    def from_design(self, x: Sequence[float], config: ParallelConfig) -> tuple[float, ...]:
        own = len(VARIABLE_NAMES)
        return (*revolute_search.from_design((*x[:JOINT_VARIABLES], *x[own:]), config.base),
                *(float(value) for value in x[JOINT_VARIABLES:own]))

    def refine(self, config: ParallelConfig, step_rad: float) -> ParallelConfig:
        goal = dataclasses.replace(config.goal, flexion_step_rad=step_rad)
        return dataclasses.replace(config, base=dataclasses.replace(config.base, goal=goal))

    def evaluate(self, x: Sequence[float], config: ParallelConfig) -> ParallelEvaluation | None:
        evaluation = evaluate(x, config)
        return dataclasses.replace(evaluation, bound_hits=bound_hits(x, config, evaluation.finger_length_mm))


MECHANISM = ParallelMechanism()


__all__ = ["MECHANISM", "ParallelMechanism", "SEARCH_NAMES", "bound_hits", "search_names"]
