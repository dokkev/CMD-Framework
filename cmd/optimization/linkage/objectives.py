"""Objectives for linkage workspace transmission and motion coordination."""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real

from .evaluator import LinkageEvaluation


@dataclass(frozen=True)
class WorstCaseTransmissionQuality:
    """Maximize the worst transmission quality across both loops and the sweep."""

    name: str = "worst_case_transmission_quality"
    sense: str = "max"

    def evaluate(self, context: LinkageEvaluation) -> float:
        if not context.complete_workspace:
            return 0.0
        qualities = context.transmission_quality_loop_1 + context.transmission_quality_loop_2
        return min(qualities, default=0.0)


@dataclass(frozen=True)
class CoordinationRatioVariation:
    """Minimize the sum of q2/q3 motion-ratio coefficients of variation."""

    epsilon: float = 1e-9
    name: str = "coordination_ratio_variation"
    sense: str = "min"

    def __post_init__(self) -> None:
        if (
            isinstance(self.epsilon, bool)
            or not isinstance(self.epsilon, Real)
            or not math.isfinite(self.epsilon)
            or self.epsilon <= 0.0
        ):
            raise ValueError("epsilon must be finite and positive")

    def evaluate(self, context: LinkageEvaluation) -> float:
        if not context.complete_workspace:
            return math.inf
        return _coefficient_of_variation(
            context.dq2_flexion_dq1_flexion,
            self.epsilon,
        ) + _coefficient_of_variation(
            context.dq3_flexion_dq1_flexion,
            self.epsilon,
        )


def _coefficient_of_variation(values: tuple[float, ...], epsilon: float) -> float:
    if not values:
        return math.inf
    mean = math.fsum(values) / len(values)
    variance = math.fsum((value - mean) ** 2 for value in values) / len(values)
    return math.sqrt(variance) / (abs(mean) + epsilon)


__all__ = ["CoordinationRatioVariation", "WorstCaseTransmissionQuality"]
