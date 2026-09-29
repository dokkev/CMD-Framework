"""Stage-0 linkage objectives over core quality and usable workspace."""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real
from typing import TYPE_CHECKING

from cmd.morphology.linkage_motion import LinkageWorkspaceCriteria

if TYPE_CHECKING:
    from .stage0 import LinkageStage0Evaluation


@dataclass(frozen=True)
class DistalEndpointExcursion:
    """Maximize usable point-C distal-link proxy displacement."""

    name: str = "distal_endpoint_excursion_mm"
    sense: str = "max"

    def evaluate(self, context: LinkageStage0Evaluation) -> float:
        return context.workspace.distal_endpoint_excursion_mm


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

    def evaluate(self, context: LinkageStage0Evaluation) -> float:
        from .stage0 import core_sample_indices, core_workspace_is_complete

        motion = context.motion
        if not core_workspace_is_complete(motion, self.core_input_workspace_rad):
            return 0.0
        indices = core_sample_indices(motion, self.core_input_workspace_rad)
        qualities = tuple(
            values[index]
            for values in (
                motion.transmission_quality_loop_1,
                motion.transmission_quality_loop_2,
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

    def evaluate(self, context: LinkageStage0Evaluation) -> float:
        from .stage0 import core_motion_ratios, core_workspace_is_complete

        if not core_workspace_is_complete(context.motion, self.core_input_workspace_rad):
            return math.inf
        return _coefficient_of_variation(
            core_motion_ratios(context.motion, self.core_input_workspace_rad, "q2"),
            self.epsilon,
        ) + _coefficient_of_variation(
            core_motion_ratios(context.motion, self.core_input_workspace_rad, "q3"),
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
