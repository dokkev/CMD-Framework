"""Convenience composition for the first CLEAR linkage baseline study."""

from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Sequence

from cmd.morphology import FingerLinkageParameters
from cmd.optimization import BoxVariable, OptimizationProblem

from .constraints import (
    FullWorkspaceKinematicValidity,
    MaximumMotionRatio,
    MinimumAnchorDorsalOffset,
    MinimumEndpointFlexion,
    MinimumTransmissionMargin,
    MonotonicFlexion,
)
from .evaluator import LinkageEvaluation, LinkageSweepConfig, LinkageSweepEvaluator
from .objectives import CoordinationRatioVariation, WorstCaseTransmissionQuality
from .parameterization import LinkageBaselineParameterization


@dataclass(frozen=True)
class LinkageBaselineConstraintConfig:
    """Caller-selected hard thresholds; none are canonical physical bounds."""

    minimum_anchor_dorsal_offset_mm: float
    minimum_transmission_margin_rad: float
    q2_minimum_motion_ratio: float
    q3_minimum_motion_ratio: float
    q2_maximum_motion_ratio: float
    q3_maximum_motion_ratio: float
    q2_minimum_endpoint_flexion_rad: float
    q3_minimum_endpoint_flexion_rad: float


def build_linkage_baseline_problem(
    variables: Sequence[BoxVariable],
    constraint_config: LinkageBaselineConstraintConfig,
    sweep_config: LinkageSweepConfig = LinkageSweepConfig(),
) -> OptimizationProblem[FingerLinkageParameters, LinkageEvaluation]:
    """Compose the standard two-objective linkage study with caller bounds."""

    inequalities = (
        FullWorkspaceKinematicValidity(),
        MinimumAnchorDorsalOffset(constraint_config.minimum_anchor_dorsal_offset_mm),
        MinimumTransmissionMargin(constraint_config.minimum_transmission_margin_rad),
        MonotonicFlexion("q2", constraint_config.q2_minimum_motion_ratio),
        MonotonicFlexion("q3", constraint_config.q3_minimum_motion_ratio),
        MaximumMotionRatio("q2", constraint_config.q2_maximum_motion_ratio),
        MaximumMotionRatio("q3", constraint_config.q3_maximum_motion_ratio),
        MinimumEndpointFlexion(
            "q2",
            constraint_config.q2_minimum_endpoint_flexion_rad,
        ),
        MinimumEndpointFlexion(
            "q3",
            constraint_config.q3_minimum_endpoint_flexion_rad,
        ),
    )
    return OptimizationProblem(
        variables=variables,
        parameterization=LinkageBaselineParameterization(),
        evaluator=LinkageSweepEvaluator(sweep_config),
        objectives=(WorstCaseTransmissionQuality(), CoordinationRatioVariation()),
        equalities=(),
        inequalities=inequalities,
    )


__all__ = ["LinkageBaselineConstraintConfig", "build_linkage_baseline_problem"]
