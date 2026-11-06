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
    formulation = "revolute-parallel-goal-v11"
    """The 2R with its servo on ``A'``, as the linkage's (2026-10-27).
    v2 (2026-10-28): the idealized 2R's v3 joint and pads (v1 runs predate
    them), ``|OA|`` and ``BC`` held to the goal's shares of ``|OC|``. v3
    (2026-10-28): driven through a belt (``k = 1``, no dead centre; the
    parallelogram of v1-v2 cannot keep its transmission angle in [30, 150]
    deg past 120 deg of flexion), its pulleys on ``A`` and ``A'``
    (``goal.hardware.pulley_radius_mm``) in the outer layer, as the linkage's
    links (not checked against the pads); the servo's tilt searched, ``A``'s
    pin and the distal pad's carrier checked against the servo, the fold and
    the hyperextension, as the linkage's v4. v4 (2026-10-29): the integrity rule (``goal.loss.integrity_depth_mm``), as the linkage's v5. v5 (2026-10-29): the 2R's search ranges for the pads' ends (no setback: each pad reaches the joint's housing and past it by its overhang (``goal.pads.*.overhang_max_mm``), the housing a collision (``pad_reach_mm``)),
    its own pad rules unchanged (shelved). v6 (2026-11-03): the 2R's v7, the
    pads tangent to the pin's housing at ``A`` (the pulleys in the outer layer). v7: the 2R's v8. v8: its v9. v9: its v10. v10: no constraint ``pad_waste_mm``: a pad mounted past where it counts at the required flexion is allowed, that part only left out of the coverage (it may serve short of it).
    v11 (2026-11-05): the constraint ``mount_depth_mm`` (the pads' drops against ``goal.pads.mount_policy``: 0 when searched, whose designs are unchanged; ``shallowest_supported``, the study's, sets each drop by ``A``'s pin and, for the proximal pad, the servo on ``A'``, ``core.optimization.mounting.parallel_mounting``); with ``goal.loss.links_block_face`` each face also less where the belt or its pulleys lie in front of it at the required flexion."""
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
        x = (*joint, dx, dz)
        if config.goal.mount_policy == "searched":
            return x
        # Each pad at the least depth A's pin and the servo on A' need, within what they carry.
        from core.optimization.mounting import parallel_mounting

        found = parallel_mounting(x, config)
        if not found.supported:
            return None
        design = dict(zip(VARIABLE_NAMES, x))
        design["proximal_drop_mm"], design["distal_drop_mm"] = found.depths_mm
        return tuple(design[name] for name in VARIABLE_NAMES)

    def prepare(self, config: ParallelConfig) -> None:
        pass

    def from_design(self, x: Sequence[float], config: ParallelConfig) -> tuple[float, ...]:
        return (*revolute_search.from_design(x[:JOINT_VARIABLES], config.base),
                *(float(value) for value in x[JOINT_VARIABLES:]))

    def refine(self, config: ParallelConfig, step_rad: float) -> ParallelConfig:
        goal = dataclasses.replace(config.goal, servo_step_rad=step_rad)
        return dataclasses.replace(config, base=dataclasses.replace(config.base, goal=goal))

    def evaluate(self, x: Sequence[float], config: ParallelConfig) -> ParallelEvaluation | None:
        evaluation = evaluate(x, config)
        return dataclasses.replace(evaluation, bound_hits=bound_hits(x, config, evaluation.finger_length_mm))


MECHANISM = ParallelMechanism()


__all__ = ["MECHANISM", "ParallelMechanism", "SEARCH_NAMES", "bound_hits", "search_names"]
