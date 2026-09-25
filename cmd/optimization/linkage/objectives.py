"""Stage-0 linkage objectives over core quality and usable workspace."""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real

from .evaluator import LinkageEvaluation
from .workspace import (
    LinkageWorkspaceCriteria,
    core_motion_ratios,
    core_sample_indices,
    core_workspace_is_complete,
    evaluate_linkage_workspace,
)


@dataclass(frozen=True)
class DistalEndpointExcursion:
    """Maximize sampled point-C endpoint displacement over usable motion."""

    criteria: LinkageWorkspaceCriteria
    name: str = "distal_endpoint_excursion_mm"
    sense: str = "max"

    def evaluate(self, context: LinkageEvaluation) -> float:
        return evaluate_linkage_workspace(
            context,
            self.criteria,
        ).distal_endpoint_excursion_mm


@dataclass(frozen=True)
class WorstCaseTransmissionQuality:
    """Maximize worst transmission quality over a fixed core interval."""

    core_input_workspace_rad: float
    name: str = "worst_case_transmission_quality"
    sense: str = "max"

    def __post_init__(self) -> None:
        _require_positive_finite(
            "core_input_workspace_rad",
            self.core_input_workspace_rad,
        )

    def evaluate(self, context: LinkageEvaluation) -> float:
        if not core_workspace_is_complete(context, self.core_input_workspace_rad):
            return 0.0
        indices = core_sample_indices(context, self.core_input_workspace_rad)
        qualities = tuple(
            values[index]
            for values in (
                context.transmission_quality_loop_1,
                context.transmission_quality_loop_2,
            )
            for index in indices
        )
        return min(qualities, default=0.0)


@dataclass(frozen=True)
class CoordinationRatioVariation:
    """Minimize q2/q3 ratio CVs computed only inside the core interval.

    An incomplete core currently evaluates to ``+inf``. A downstream solver
    adapter must define how it handles non-finite objective values before this
    objective is used with that solver.
    """

    core_input_workspace_rad: float
    epsilon: float = 1e-9
    name: str = "coordination_ratio_variation"
    sense: str = "min"

    def __post_init__(self) -> None:
        _require_positive_finite(
            "core_input_workspace_rad",
            self.core_input_workspace_rad,
        )
        _require_positive_finite("epsilon", self.epsilon)

    def evaluate(self, context: LinkageEvaluation) -> float:
        if not core_workspace_is_complete(context, self.core_input_workspace_rad):
            return math.inf
        return _coefficient_of_variation(
            core_motion_ratios(context, self.core_input_workspace_rad, "q2"),
            self.epsilon,
        ) + _coefficient_of_variation(
            core_motion_ratios(context, self.core_input_workspace_rad, "q3"),
            self.epsilon,
        )


def _coefficient_of_variation(values: tuple[float, ...], epsilon: float) -> float:
    if not values:
        return math.inf
    mean = math.fsum(values) / len(values)
    variance = math.fsum((value - mean) ** 2 for value in values) / len(values)
    return math.sqrt(variance) / (abs(mean) + epsilon)


def _require_positive_finite(name: str, value: float) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
        or value <= 0.0
    ):
        raise ValueError(f"{name} must be finite and positive")


__all__ = [
    "CoordinationRatioVariation",
    "DistalEndpointExcursion",
    "WorstCaseTransmissionQuality",
]
