"""The serial 2R driven through a parallel four-bar, on the linkage study's
goal, as a pluggable mechanism (``revolute_parallel``): the same config file,
objectives and report columns as ``linkage`` and ``revolute``.

Searched in the idealized 2R's coordinates (``core.optimization.revolute.search``)
for the joint and pads, then ``A'`` from ``A`` and the cranks' length, each
itself on the prototyping resolution, and the cranks' direction as
``crank_turn`` in ``[-1, 1]``: the transmission angle at extension within the
band that keeps it in the goal's range all through the required flexion (the
cranks turn by ``phi`` from the coupler ``A'->A``, toward it or away from it
by the sign), so the search spends no time on dead-centre designs. With a
flexion beyond the range's width no band is left and the full circle is
searched (nothing there is feasible). The angle is kept to 0.1 deg.
"""

from __future__ import annotations

import dataclasses
import math
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
PARALLEL_NAMES = VARIABLE_NAMES[JOINT:]
SEARCH_NAMES = (*revolute_search.SEARCH_NAMES, "ap_from_a_x_mm", "ap_from_a_z_mm", "crank_mm", "crank_turn")
ANGLE_STEP_DEG = 0.1


def _coupler_deg(dx: float, dz: float) -> float:
    """The coupler's direction ``A' -> A``, degrees from ``+X`` toward ``+Z``."""

    return math.degrees(math.atan2(-dz, -dx))


def _band(config: ParallelConfig) -> tuple[float, float] | None:
    """The transmission angles at extension that stay in range up to the
    required flexion, ``beta - phi`` falling from ``beta`` (``None``: none do)."""

    low, high = (math.degrees(v) for v in config.goal.transmission_angle_range_rad)
    need = math.degrees(config.goal.required_rom_rad)
    return (low + need, high) if low + need <= high else None


def _crank_angle(turn: float, dx: float, dz: float, config: ParallelConfig) -> float:
    band = _band(config)
    if band is None:  # nothing keeps the range: the whole circle
        beta = 180.0 * turn
    else:
        # turn > 0: the cranks turn toward the coupler (beta falls to beta - phi);
        # turn < 0: away from it, the mirror image (-beta rises).
        low, high = band
        beta = low + (high - low) * abs(turn)
        beta = beta if turn >= 0.0 else -(beta - math.degrees(config.goal.required_rom_rad))
    angle = (_coupler_deg(dx, dz) + beta + 180.0) % 360.0 - 180.0
    return round(round(angle / ANGLE_STEP_DEG) * ANGLE_STEP_DEG, 9)


def _crank_turn(angle: float, dx: float, dz: float, config: ParallelConfig) -> float:
    beta = (angle - _coupler_deg(dx, dz) + 180.0) % 360.0 - 180.0
    band = _band(config)
    if band is None:
        return beta / 180.0
    low, high = band
    width = max(high - low, 1e-9)
    if beta >= 0.0:
        return float(min(max((beta - low) / width, 0.0), 1.0))
    return -float(min(max((-beta + math.degrees(config.goal.required_rom_rad) - low) / width, 0.0), 1.0))


def bound_hits(x: Sequence[float], config: ParallelConfig, finger_length_mm: float) -> tuple[str, ...]:
    """The variables near a bound of the search box, and ``|OC|`` near its bounds."""

    bounds = {name: pair for name, pair in config.bounds.items() if name != "crank_angle_deg"}  # a circle
    values = {name: value for name, value in zip(VARIABLE_NAMES, x) if name in bounds}
    return bound_hits_of(values, bounds, finger_length_mm, config.goal.extended_length_bounds_mm)


class ParallelMechanism(RevoluteMechanism):
    name = "revolute_parallel"
    formulation = "revolute-parallel-goal-v1"
    """The 2R with its servo on ``A'``, as the linkage's, turning the joint
    through a parallelogram (2026-10-27): ``k = 1``, the transmission angle
    held within the goal's range."""
    variable_names = VARIABLE_NAMES
    constraint_names = CONSTRAINT_NAMES

    def config_from_mapping(self, values: Mapping[str, Any]) -> ParallelConfig:
        return ParallelConfig.from_goal(config_from_goal(values))

    def config_to_json(self, config: ParallelConfig) -> dict[str, Any]:
        return {"goal": config_to_json(config.goal), "bounds": dict(config.bounds)}

    def config_from_json(self, values: dict[str, Any]) -> ParallelConfig:
        goal = config_from_json(values["goal"])
        base = RevoluteConfig.from_goal(goal, joint_servo=False)
        return ParallelConfig(base, {name: tuple(pair) for name, pair in values["bounds"].items()})

    def search_names(self, config: ParallelConfig) -> tuple[str, ...]:
        return SEARCH_NAMES

    def search_box(self, config: ParallelConfig) -> tuple[Sequence[float], Sequence[float]]:
        lower, upper = revolute_search.search_box(config.base)
        own = ("ap_from_a_x_mm", "ap_from_a_z_mm", "crank_mm")
        return ((*lower, *(config.bounds[name][0] for name in own), -1.0),
                (*upper, *(config.bounds[name][1] for name in own), 1.0))

    def search_steps(self, config: ParallelConfig) -> tuple[float, ...]:
        return (*revolute_search.search_steps(config.base), *(config.resolution_mm,) * 3, revolute_search.SHARE_STEP)

    def to_design(self, coordinates: Sequence[float], config: ParallelConfig) -> tuple[float, ...] | None:
        joint = revolute_search.to_design(coordinates[:JOINT], config.base)
        if joint is None:
            return None
        resolution = config.resolution_mm
        dx, dz, crank = (round(round(float(value) / resolution) * resolution, 9) for value in coordinates[JOINT:JOINT + 3])
        if math.hypot(dx, dz) < 1e-9:
            return None
        return (*joint, dx, dz, crank, _crank_angle(float(coordinates[JOINT + 3]), dx, dz, config))

    def prepare(self, config: ParallelConfig) -> None:
        pass

    def from_design(self, x: Sequence[float], config: ParallelConfig) -> tuple[float, ...]:
        dx, dz, crank, angle = (float(value) for value in x[JOINT:])
        return (*revolute_search.from_design(x[:JOINT], config.base), dx, dz, crank, _crank_turn(angle, dx, dz, config))

    def refine(self, config: ParallelConfig, step_rad: float) -> ParallelConfig:
        goal = dataclasses.replace(config.goal, servo_step_rad=step_rad)
        return dataclasses.replace(config, base=dataclasses.replace(config.base, goal=goal))

    def evaluate(self, x: Sequence[float], config: ParallelConfig) -> ParallelEvaluation | None:
        evaluation = evaluate(x, config)
        return dataclasses.replace(evaluation, bound_hits=bound_hits(x, config, evaluation.finger_length_mm))

    def report(self, e: ParallelEvaluation) -> dict[str, Any]:
        return {**super().report(e), "proximal_usable_length_mm": e.proximal_usable_length_mm,
                "distal_usable_length_mm": e.distal_usable_length_mm,
                "min_transmission_deg": e.min_transmission_deg}


MECHANISM = ParallelMechanism()


__all__ = ["MECHANISM", "ParallelMechanism", "SEARCH_NAMES", "bound_hits"]
