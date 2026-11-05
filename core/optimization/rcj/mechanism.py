"""The rolling contact joint (RCJ) finger on the linkage study's goal, as a
pluggable mechanism (``rcj``): the same config file, objectives and report
columns as ``linkage`` and ``revolute``, so the fronts plot on the same axes."""

from __future__ import annotations

import dataclasses
from collections.abc import Mapping, Sequence
from typing import Any

from core.optimization.linkage.config import config_from_goal, config_from_json, config_to_json
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
    formulation = "rcj-goal-v7"
    """The RCJ on the linkage's goal (2026-10-27): equal rollers, the servo on
    the proximal roller's centre driving the carrier (``k = 2``), laid back
    along ``-X`` as the 2R's; ideal no-slip rolling, the cables not modelled
    (provisional). v2 (2026-10-28): ``|OA|``, ``BC`` and ``|AB| = 2r`` held to
    the goal's shares of ``|OC|`` (``goal.link.*_ratio``), as the linkage's;
    the proximal pad's drop searched (at most ``r``), the pads' overhang past
    the rollers (``goal.pads.*.overhang_max_mm``), the fold, the carrier and
    the hyperextension (the linkage's v4 rules). v3 (2026-10-29): the integrity rule (``goal.loss.integrity_depth_mm``), as the linkage's v5. v4 (2026-11-03): each pad counted at the required
    flexion to where the other's face line crosses it (the 2R's
    ``usable_lengths``, as the linkage's v8) and none mounted past that
    (``pad_waste_mm``), the segment ratios without maxima
    (its own pads, drops and carrier unchanged). v5: the segment ratios on
    ``|OA''|`` and ``|B''C|``, as the linkage's and the 2R's. v6 (2026-11-04): a contact module's least length, not the segment's least share, bounds its segment from below; the share gives way and is reported (``*_ratio_shortfall_mm``).
    v7 (2026-11-05): the servo's tilt a design variable (``servo_tilt_deg``, the goal's PIP servo's, as the 2R's; a run without it was untilted); the constraint ``mount_depth_mm`` (the pads' drops against ``goal.pads.mount_policy``: 0 when searched, whose designs are unchanged; ``shallowest_supported``, the study's, sets each drop by the roller and servo it runs under, ``core.optimization.mounting``)."""
    variable_names = VARIABLE_NAMES
    constraint_names = CONSTRAINT_NAMES

    def config_from_mapping(self, values: Mapping[str, Any]) -> RcjConfig:
        return RcjConfig.from_goal(config_from_goal(values))

    def config_to_json(self, config: RcjConfig) -> dict[str, Any]:
        return {"goal": config_to_json(config.goal), "bounds": dict(config.bounds)}

    def config_from_json(self, values: dict[str, Any]) -> RcjConfig:
        return RcjConfig(config_from_json(values["goal"]), {name: tuple(pair) for name, pair in values["bounds"].items()})

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


MECHANISM = RcjMechanism()


__all__ = ["MECHANISM", "RcjMechanism", "bound_hits"]
