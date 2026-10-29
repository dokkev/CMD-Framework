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

from core.optimization.linkage.config import config_from_goal, config_from_json, config_to_json
from core.optimization.linkage.mechanism import bound_hits_of
from core.optimization.revolute import search as revolute_search
from core.optimization.revolute.config import RevoluteConfig
from core.optimization.revolute.mechanism import RevoluteMechanism
from core.optimization.revolute_parallel.config import VARIABLE_NAMES, ParallelConfig
from core.optimization.revolute_parallel.evaluate import CONSTRAINT_NAMES, ParallelEvaluation, evaluate

JOINT = len(revolute_search.SEARCH_NAMES)
"""The 2R's coordinates (and design variables) come first."""
SEARCH_NAMES = (*revolute_search.SEARCH_NAMES, "ap_from_a_x_mm", "ap_from_a_z_mm")


def bound_hits(x: Sequence[float], config: ParallelConfig, finger_length_mm: float) -> tuple[str, ...]:
    """The variables near a bound of the search box, and ``|OC|`` near its bounds."""

    return bound_hits_of(dict(zip(VARIABLE_NAMES, x)), config.bounds, finger_length_mm,
                         config.goal.extended_length_bounds_mm)


class ParallelMechanism(RevoluteMechanism):
    name = "revolute_parallel"
    formulation = "revolute-parallel-goal-v4"
    """The 2R with its servo on ``A'``, as the linkage's (2026-10-27).
    v2 (2026-10-28): the idealized 2R's v3 joint and pads (v1 runs predate
    them), ``|OA|`` and ``BC`` held to the goal's shares of ``|OC|``. v3
    (2026-10-28): driven through a belt (``k = 1``, no dead centre; the
    parallelogram of v1-v2 cannot keep its transmission angle in [30, 150]
    deg past 120 deg of flexion), its pulleys on ``A`` and ``A'``
    (``goal.hardware.pulley_radius_mm``) in the outer layer, as the linkage's
    links (not checked against the pads); the servo's tilt searched, ``A``'s
    pin and the distal pad's carrier checked against the servo, the fold and
    the hyperextension, as the linkage's v4. v4 (2026-10-29): the integrity rule (``goal.loss.integrity_depth_mm``), as the linkage's v5."""
    variable_names = VARIABLE_NAMES
    constraint_names = CONSTRAINT_NAMES

    def config_from_mapping(self, values: Mapping[str, Any]) -> ParallelConfig:
        return ParallelConfig.from_goal(config_from_goal(values))

    def config_to_json(self, config: ParallelConfig) -> dict[str, Any]:
        return {"goal": config_to_json(config.goal), "bounds": dict(config.bounds)}

    def config_from_json(self, values: dict[str, Any]) -> ParallelConfig:
        goal = config_from_json(values["goal"])
        bounds = {name: tuple(pair) for name, pair in values["bounds"].items()}
        joint = RevoluteConfig.from_goal(goal, joint_servo=False)
        base = RevoluteConfig(goal, {**joint.bounds, "servo_tilt_deg": bounds["servo_tilt_deg"]}, False)
        return ParallelConfig(base, bounds)

    def search_names(self, config: ParallelConfig) -> tuple[str, ...]:
        return SEARCH_NAMES

    def search_box(self, config: ParallelConfig) -> tuple[Sequence[float], Sequence[float]]:
        lower, upper = revolute_search.search_box(config.base)
        own = ("ap_from_a_x_mm", "ap_from_a_z_mm")
        return ((*lower, *(config.bounds[name][0] for name in own)),
                (*upper, *(config.bounds[name][1] for name in own)))

    def search_steps(self, config: ParallelConfig) -> tuple[float, ...]:
        return (*revolute_search.search_steps(config.base), config.resolution_mm, config.resolution_mm)

    def to_design(self, coordinates: Sequence[float], config: ParallelConfig) -> tuple[float, ...] | None:
        joint = revolute_search.to_design(coordinates[:JOINT], config.base)
        if joint is None:
            return None
        resolution = config.resolution_mm
        dx, dz = (round(round(float(value) / resolution) * resolution, 9) for value in coordinates[JOINT:JOINT + 2])
        return (*joint, dx, dz)

    def prepare(self, config: ParallelConfig) -> None:
        pass

    def from_design(self, x: Sequence[float], config: ParallelConfig) -> tuple[float, ...]:
        return (*revolute_search.from_design(x[:JOINT], config.base), *(float(value) for value in x[JOINT:]))

    def refine(self, config: ParallelConfig, step_rad: float) -> ParallelConfig:
        goal = dataclasses.replace(config.goal, servo_step_rad=step_rad)
        return dataclasses.replace(config, base=dataclasses.replace(config.base, goal=goal))

    def evaluate(self, x: Sequence[float], config: ParallelConfig) -> ParallelEvaluation | None:
        evaluation = evaluate(x, config)
        return dataclasses.replace(evaluation, bound_hits=bound_hits(x, config, evaluation.finger_length_mm))


MECHANISM = ParallelMechanism()


__all__ = ["MECHANISM", "ParallelMechanism", "SEARCH_NAMES", "bound_hits"]
