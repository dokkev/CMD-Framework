"""Solver-independent Stage-0 evaluation: two objectives and three constraint groups.

Objectives (measured over the core input workspace):

- worst-case transmission quality of both coupler loops (maximize);
- summed coefficient of variation of the q2 and q3 motion ratios (minimize).

Constraint groups use ``g <= 0`` and keep a fixed residual count per study:

- ``workspace_health``: closure, transmission margin, and motion-ratio bounds
  at every sample through the required input workspace;
- ``minimum_flexion_at_core_q2`` / ``_q3``: joint flexion at the core endpoint;
- ``minimum_anchor_dorsal_offset``: A' and B' above B in the reference pose.

Point-C excursion is deliberately a diagnostic, not an objective: it rewards
link size and does not imply grasp quality.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

from cmd.morphology.linkage import (
    FingerLinkageParameters,
    LinkageMotionEvaluation,
    LinkageMotionEvaluator,
    LinkageWorkspaceCriteria,
    LinkageWorkspaceEvaluation,
    evaluate_linkage_workspace,
)

from .config import Stage0Config


_CV_EPSILON = 1e-9


@dataclass(frozen=True)
class Stage0Evaluation:
    """Objectives, constraint residuals, and diagnostics for one design."""

    motion: LinkageMotionEvaluation
    workspace: LinkageWorkspaceEvaluation
    worst_transmission_quality: float
    ratio_variation: float
    constraints: Mapping[str, tuple[float, ...]]

    @property
    def g(self) -> tuple[float, ...]:
        return tuple(value for group in self.constraints.values() for value in group)

    @property
    def feasible(self) -> bool:
        return all(value <= 0.0 for value in self.g)

    def max_violation(self, group: str) -> float:
        return max(0.0, *self.constraints[group])


class Stage0Evaluator:
    """Evaluate designs on the study's fixed input-flexion sample grid."""

    def __init__(self, config: Stage0Config) -> None:
        self.config = config
        self.samples = input_samples(config)
        self.core_count = self.samples.index(config.core_input_workspace_rad) + 1
        self.required_count = self.samples.index(config.required_input_workspace_rad) + 1
        self._motion = LinkageMotionEvaluator(self.samples)

    @property
    def constraint_count(self) -> int:
        # Closure fraction, 2 loops per sample, 2 joints per interval, 2 flexion, 2 anchors.
        return 1 + 2 * self.required_count + 2 * (self.required_count - 1) + 4

    def __call__(self, parameters: FingerLinkageParameters) -> Stage0Evaluation:
        config = self.config
        motion = self._motion.evaluate(parameters)
        core_complete = motion.solved_sample_count >= self.core_count
        return Stage0Evaluation(
            motion=motion,
            workspace=evaluate_linkage_workspace(motion, config.criteria),
            worst_transmission_quality=(
                worst_transmission_quality(motion, self.core_count) if core_complete else 0.0
            ),
            ratio_variation=(
                ratio_variation(motion, self.core_count) if core_complete else math.inf
            ),
            constraints=MappingProxyType({
                "workspace_health": workspace_health_residuals(
                    motion, config.criteria, self.required_count
                ),
                "minimum_flexion_at_core_q2": endpoint_flexion_residual(
                    motion.q2_flexion_rad, self.core_count,
                    config.q2_minimum_endpoint_flexion_rad,
                ),
                "minimum_flexion_at_core_q3": endpoint_flexion_residual(
                    motion.q3_flexion_rad, self.core_count,
                    config.q3_minimum_endpoint_flexion_rad,
                ),
                "minimum_anchor_dorsal_offset": anchor_dorsal_offset_residuals(
                    parameters, config.minimum_anchor_dorsal_offset_mm
                ),
            }),
        )


def input_samples(config: Stage0Config) -> tuple[float, ...]:
    """Create the uniform horizon grid with exact core and required endpoints.

    An endpoint within 1% of a grid step replaces that uniform sample instead
    of being inserted beside it, so finite-difference intervals cannot collapse.
    """

    count = config.sample_count
    step = config.search_horizon_rad / (count - 1)
    samples = [config.search_horizon_rad * index / (count - 1) for index in range(count)]
    endpoints: list[float] = []
    for endpoint in (config.core_input_workspace_rad, config.required_input_workspace_rad):
        if endpoint in endpoints:
            continue
        nearest = min(
            (index for index in range(1, len(samples)) if samples[index] not in endpoints),
            key=lambda index: abs(samples[index] - endpoint),
        )
        if abs(samples[nearest] - endpoint) <= 0.01 * step:
            samples[nearest] = endpoint
        else:
            samples.append(endpoint)
            samples.sort()
        endpoints.append(endpoint)
    return tuple(samples)


def worst_transmission_quality(motion: LinkageMotionEvaluation, core_count: int) -> float:
    return min(
        *motion.transmission_quality_loop_1[:core_count],
        *motion.transmission_quality_loop_2[:core_count],
    )


def ratio_variation(motion: LinkageMotionEvaluation, core_count: int) -> float:
    return _coefficient_of_variation(
        motion.dq2_flexion_dq1_flexion[: core_count - 1]
    ) + _coefficient_of_variation(motion.dq3_flexion_dq1_flexion[: core_count - 1])


def workspace_health_residuals(
    motion: LinkageMotionEvaluation,
    criteria: LinkageWorkspaceCriteria,
    required_count: int,
) -> tuple[float, ...]:
    """Fixed-shape residuals; unsolved samples count only through the closure fraction."""

    solved = min(motion.solved_sample_count, required_count)
    residuals = [1.0 - solved / required_count]
    for angles in (motion.transmission_angle_loop_1_rad, motion.transmission_angle_loop_2_rad):
        residuals.extend(
            criteria.transmission_shortfall_rad(angles[index]) if index < solved else 0.0
            for index in range(required_count)
        )
    for joint in ("q2", "q3"):
        ratios = getattr(motion, f"d{joint}_flexion_dq1_flexion")
        residuals.extend(
            criteria.motion_ratio_violation(joint, ratios[interval])
            if interval < solved - 1
            else 0.0
            for interval in range(required_count - 1)
        )
    return tuple(residuals)


def endpoint_flexion_residual(
    flexion_rad: tuple[float, ...], core_count: int, minimum_rad: float
) -> tuple[float]:
    if core_count > len(flexion_rad):
        return (0.0,)
    return (minimum_rad - flexion_rad[core_count - 1],)


def anchor_dorsal_offset_residuals(
    parameters: FingerLinkageParameters, minimum_offset_mm: float
) -> tuple[float, float]:
    a_z = parameters.a_from_o.z_mm
    b_z = a_z + parameters.b_from_a.z_mm
    a_prime_z = parameters.a_prime_from_o.z_mm
    b_prime_z = a_z + parameters.b_prime_from_a.z_mm
    return (
        minimum_offset_mm - (a_prime_z - b_z),
        minimum_offset_mm - (b_prime_z - b_z),
    )


def _coefficient_of_variation(values: tuple[float, ...]) -> float:
    if not values:
        return math.inf
    mean = math.fsum(values) / len(values)
    variance = math.fsum((value - mean) ** 2 for value in values) / len(values)
    return math.sqrt(variance) / (abs(mean) + _CV_EPSILON)


__all__ = [
    "Stage0Evaluation",
    "Stage0Evaluator",
    "anchor_dorsal_offset_residuals",
    "endpoint_flexion_residual",
    "input_samples",
    "ratio_variation",
    "workspace_health_residuals",
    "worst_transmission_quality",
]
