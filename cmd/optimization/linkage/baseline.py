"""Composition for the solver-independent CLEAR Stage-0 linkage problem."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Real

from cmd.morphology import FingerLinkageParameters
from cmd.optimization import BoxVariable, OptimizationProblem

from .constraints import (
    CoreWorkspaceKinematicValidity,
    MaximumMotionRatio,
    MinimumAnchorDorsalOffset,
    MinimumFlexionAtInput,
    MinimumTransmissionMargin,
    MinimumUsableInputWorkspace,
    MonotonicFlexion,
)
from .evaluator import LinkageEvaluation, LinkageSweepConfig, LinkageSweepEvaluator
from .objectives import (
    CoordinationRatioVariation,
    DistalEndpointExcursion,
    WorstCaseTransmissionQuality,
)
from .parameterization import LinkageBaselineParameterization
from .workspace import LinkageWorkspaceCriteria


@dataclass(frozen=True)
class LinkageBaselineConstraintConfig:
    """Caller-selected Stage-0 workspace, anchor, endpoint, and health settings."""

    core_input_workspace_rad: float
    minimum_required_input_workspace_rad: float
    minimum_anchor_dorsal_offset_mm: float
    q2_minimum_endpoint_flexion_rad: float
    q3_minimum_endpoint_flexion_rad: float
    workspace_criteria: LinkageWorkspaceCriteria

    def __post_init__(self) -> None:
        _require_positive_finite(
            "core_input_workspace_rad",
            self.core_input_workspace_rad,
        )
        _require_positive_finite(
            "minimum_required_input_workspace_rad",
            self.minimum_required_input_workspace_rad,
        )
        if self.minimum_required_input_workspace_rad < self.core_input_workspace_rad:
            raise ValueError(
                "minimum_required_input_workspace_rad must be at least the core input workspace"
            )
        _require_nonnegative_finite(
            "minimum_anchor_dorsal_offset_mm",
            self.minimum_anchor_dorsal_offset_mm,
        )
        _require_nonnegative_finite(
            "q2_minimum_endpoint_flexion_rad",
            self.q2_minimum_endpoint_flexion_rad,
        )
        _require_nonnegative_finite(
            "q3_minimum_endpoint_flexion_rad",
            self.q3_minimum_endpoint_flexion_rad,
        )
        if not isinstance(self.workspace_criteria, LinkageWorkspaceCriteria):
            raise TypeError("workspace_criteria must be LinkageWorkspaceCriteria")


def build_linkage_baseline_problem(
    variables: Sequence[BoxVariable],
    constraint_config: LinkageBaselineConstraintConfig,
    sweep_config: LinkageSweepConfig,
) -> OptimizationProblem[FingerLinkageParameters, LinkageEvaluation]:
    """Compose Stage-0 objectives and core-only hard linkage constraints."""

    bc_variable = next(
        (variable for variable in variables if variable.name == "bc_length_mm"),
        None,
    )
    if bc_variable is None:
        raise ValueError("Stage-0 variables must include bc_length_mm")
    if bc_variable.lower <= 0.0:
        raise ValueError("bc_length_mm lower bound must be greater than zero")

    core = constraint_config.core_input_workspace_rad
    if core >= sweep_config.search_horizon_rad:
        raise ValueError("core_input_workspace_rad must be below search_horizon_rad")
    if constraint_config.minimum_required_input_workspace_rad > sweep_config.search_horizon_rad:
        raise ValueError(
            "minimum_required_input_workspace_rad must not exceed search_horizon_rad"
        )
    criteria = constraint_config.workspace_criteria
    evaluator = LinkageSweepEvaluator(
        sweep_config,
        core_input_workspace_rad=core,
    )

    inequalities = (
        CoreWorkspaceKinematicValidity(core),
        MinimumUsableInputWorkspace(
            constraint_config.minimum_required_input_workspace_rad,
            criteria,
        ),
        MinimumAnchorDorsalOffset(
            constraint_config.minimum_anchor_dorsal_offset_mm
        ),
        MinimumTransmissionMargin(
            criteria,
            core_input_workspace_rad=core,
        ),
        MonotonicFlexion(
            "q2",
            criteria,
            core_input_workspace_rad=core,
        ),
        MonotonicFlexion(
            "q3",
            criteria,
            core_input_workspace_rad=core,
        ),
        MaximumMotionRatio(
            "q2",
            criteria,
            core_input_workspace_rad=core,
        ),
        MaximumMotionRatio(
            "q3",
            criteria,
            core_input_workspace_rad=core,
        ),
        MinimumFlexionAtInput(
            "q2",
            input_flexion_rad=core,
            minimum_flexion_rad=constraint_config.q2_minimum_endpoint_flexion_rad,
        ),
        MinimumFlexionAtInput(
            "q3",
            input_flexion_rad=core,
            minimum_flexion_rad=constraint_config.q3_minimum_endpoint_flexion_rad,
        ),
    )
    return OptimizationProblem(
        variables=variables,
        parameterization=LinkageBaselineParameterization(),
        evaluator=evaluator,
        objectives=(
            WorstCaseTransmissionQuality(core),
            CoordinationRatioVariation(core),
            DistalEndpointExcursion(criteria),
        ),
        equalities=(),
        inequalities=inequalities,
    )


def _require_positive_finite(name: str, value: float) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
        or value <= 0.0
    ):
        raise ValueError(f"{name} must be finite and positive")


def _require_nonnegative_finite(name: str, value: float) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
        or value < 0.0
    ):
        raise ValueError(f"{name} must be finite and non-negative")


__all__ = ["LinkageBaselineConstraintConfig", "build_linkage_baseline_problem"]
