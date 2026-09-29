"""Stage-0 linkage formulation composed over reusable morphology motion."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Real
from typing import TYPE_CHECKING, Literal

from cmd.morphology.linkage import (
    BASELINE_LINKAGE_V0,
    FingerLinkageParameters,
    LinkageMotionEvaluation,
    LinkageMotionEvaluator,
    LinkageWorkspaceCriteria,
    LinkageWorkspaceEvaluation,
    evaluate_linkage_workspace,
)
from cmd.optimization import BoxVariable, OptimizationProblem

from . import (
    CoreWorkspaceKinematicValidity,
    CoordinationRatioVariation,
    DistalEndpointExcursion,
    LinkageBaselineParameterization,
    MaximumMotionRatio,
    MinimumAnchorDorsalOffset,
    MinimumFlexionAtCore,
    MinimumTransmissionMargin,
    MinimumUsableInputWorkspace,
    MonotonicFlexion,
    WorstCaseTransmissionQuality,
)

if TYPE_CHECKING:
    from cmd.optimization.solvers.pymoo import NSGA2Config, NSGA2RunResult


@dataclass(frozen=True)
class LinkageStage0Config:
    """Stage-0-specific sampling, workspace, and feasibility settings."""

    core_input_workspace_rad: float
    minimum_required_input_workspace_rad: float
    search_horizon_rad: float
    sample_count: int
    minimum_anchor_dorsal_offset_mm: float
    q2_minimum_endpoint_flexion_rad: float
    q3_minimum_endpoint_flexion_rad: float
    workspace_criteria: LinkageWorkspaceCriteria

    def __post_init__(self) -> None:
        for name in (
            "core_input_workspace_rad",
            "minimum_required_input_workspace_rad",
            "search_horizon_rad",
        ):
            _require_positive_finite(name, getattr(self, name))
        if self.core_input_workspace_rad >= self.search_horizon_rad:
            raise ValueError("core_input_workspace_rad must be below search_horizon_rad")
        if self.minimum_required_input_workspace_rad < self.core_input_workspace_rad:
            raise ValueError(
                "minimum_required_input_workspace_rad must be at least the core input workspace"
            )
        if self.minimum_required_input_workspace_rad > self.search_horizon_rad:
            raise ValueError(
                "minimum_required_input_workspace_rad must not exceed search_horizon_rad"
            )
        if isinstance(self.sample_count, bool) or not isinstance(self.sample_count, int) or self.sample_count < 3:
            raise ValueError("sample_count must be an integer of at least 3")
        _require_nonnegative_finite(
            "minimum_anchor_dorsal_offset_mm", self.minimum_anchor_dorsal_offset_mm
        )
        _require_nonnegative_finite(
            "q2_minimum_endpoint_flexion_rad", self.q2_minimum_endpoint_flexion_rad
        )
        _require_nonnegative_finite(
            "q3_minimum_endpoint_flexion_rad", self.q3_minimum_endpoint_flexion_rad
        )
        if not isinstance(self.workspace_criteria, LinkageWorkspaceCriteria):
            raise TypeError("workspace_criteria must be LinkageWorkspaceCriteria")
        for name in (
            "core_input_workspace_rad",
            "minimum_required_input_workspace_rad",
            "search_horizon_rad",
            "minimum_anchor_dorsal_offset_mm",
            "q2_minimum_endpoint_flexion_rad",
            "q3_minimum_endpoint_flexion_rad",
        ):
            object.__setattr__(self, name, float(getattr(self, name)))


@dataclass(frozen=True)
class LinkageStage0Evaluation:
    """Motion, workspace, and core interval view for one Stage-0 candidate."""

    motion: LinkageMotionEvaluation
    workspace: LinkageWorkspaceEvaluation
    core_sample_count: int

    def __post_init__(self) -> None:
        if (
            isinstance(self.core_sample_count, bool)
            or not isinstance(self.core_sample_count, int)
            or not 1 <= self.core_sample_count <= self.motion.requested_sample_count
        ):
            raise ValueError("core_sample_count must index a non-empty prefix of the motion sweep")

    @property
    def core_complete(self) -> bool:
        return self.motion.solved_sample_count >= self.core_sample_count

    def core_motion_ratios(self, joint: Literal["q2", "q3"]) -> tuple[float, ...]:
        if joint == "q2":
            ratios = self.motion.dq2_flexion_dq1_flexion
        elif joint == "q3":
            ratios = self.motion.dq3_flexion_dq1_flexion
        else:
            raise ValueError("joint must be 'q2' or 'q3'")
        return ratios[: self.core_sample_count - 1]


@dataclass(frozen=True)
class Stage0Evaluator:
    """Apply Stage-0 sample semantics, then evaluate motion and workspace once."""

    config: LinkageStage0Config

    def evaluate(self, parameters: FingerLinkageParameters) -> LinkageStage0Evaluation:
        samples = stage0_input_samples(self.config)
        motion = LinkageMotionEvaluator(samples).evaluate(parameters)
        workspace = evaluate_linkage_workspace(motion, self.config.workspace_criteria)
        core_sample_count = sum(
            sample <= self.config.core_input_workspace_rad for sample in samples
        )
        return LinkageStage0Evaluation(motion, workspace, core_sample_count)


@dataclass(frozen=True)
class LinkageStage0Result:
    """Stage-0 configuration, composed problem, and NSGA-II result."""

    problem: OptimizationProblem[FingerLinkageParameters, LinkageStage0Evaluation]
    stage0_config: LinkageStage0Config
    solver_result: NSGA2RunResult
    study_name: str | None
    include_baseline_seed: bool


def stage0_input_samples(config: LinkageStage0Config) -> tuple[float, ...]:
    """Create the Stage-0 horizon grid with its exact core endpoint included."""
    uniform = [
        config.search_horizon_rad * index / (config.sample_count - 1)
        for index in range(config.sample_count)
    ]
    core = config.core_input_workspace_rad
    match = next(
        (
            index
            for index in range(1, len(uniform) - 1)
            if math.isclose(uniform[index], core, rel_tol=1e-12, abs_tol=0.0)
        ),
        None,
    )
    if match is None:
        uniform.append(core)
    else:
        uniform[match] = core
    return tuple(sorted(uniform))


def build_linkage_stage0_problem(
    variables: Sequence[BoxVariable], config: LinkageStage0Config
) -> OptimizationProblem[FingerLinkageParameters, LinkageStage0Evaluation]:
    """Compose the fixed Stage-0 objective/constraint formulation."""
    bc_variable = next((v for v in variables if v.name == "bc_length_mm"), None)
    if bc_variable is None:
        raise ValueError("Stage-0 variables must include bc_length_mm")
    if bc_variable.lower <= 0.0:
        raise ValueError("bc_length_mm lower bound must be greater than zero")
    criteria = config.workspace_criteria
    return OptimizationProblem(
        variables=variables,
        parameterization=LinkageBaselineParameterization(),
        evaluator=Stage0Evaluator(config),
        objectives=(
            WorstCaseTransmissionQuality(),
            CoordinationRatioVariation(),
            DistalEndpointExcursion(),
        ),
        equalities=(),
        inequalities=(
            CoreWorkspaceKinematicValidity(),
            MinimumUsableInputWorkspace(config.minimum_required_input_workspace_rad),
            MinimumAnchorDorsalOffset(config.minimum_anchor_dorsal_offset_mm),
            MinimumTransmissionMargin(criteria),
            MonotonicFlexion("q2", criteria),
            MonotonicFlexion("q3", criteria),
            MaximumMotionRatio("q2", criteria),
            MaximumMotionRatio("q3", criteria),
            MinimumFlexionAtCore("q2", config.q2_minimum_endpoint_flexion_rad),
            MinimumFlexionAtCore("q3", config.q3_minimum_endpoint_flexion_rad),
        ),
    )


def run_linkage_stage0(
    variables: Sequence[BoxVariable],
    stage0_config: LinkageStage0Config,
    solver_config: NSGA2Config,
    *,
    include_baseline_seed: bool = True,
    study_name: str | None = None,
) -> LinkageStage0Result:
    """Run the Stage-0 problem through the existing pymoo NSGA-II adapter."""
    if not isinstance(include_baseline_seed, bool):
        raise ValueError("include_baseline_seed must be a bool")
    if study_name is not None and (not isinstance(study_name, str) or not study_name.strip()):
        raise ValueError("study_name must be None or a non-empty string")
    problem = build_linkage_stage0_problem(variables, stage0_config)
    prototype_x = LinkageBaselineParameterization().encode(BASELINE_LINKAGE_V0)
    initial_designs: tuple[tuple[float, ...], ...] = ()
    if include_baseline_seed:
        for variable, value in zip(problem.variables, prototype_x):
            if variable.violation(value) > 0.0:
                raise ValueError(
                    "projected BASELINE_LINKAGE_V0 is outside caller-supplied "
                    f"bounds for {variable.name!r}: {value} not in "
                    f"[{variable.lower}, {variable.upper}]"
                )
        initial_designs = (prototype_x,)
    solver_result = run_nsga2(problem, prototype_x, solver_config, initial_designs=initial_designs)
    return LinkageStage0Result(
        problem, stage0_config, solver_result, study_name, include_baseline_seed
    )


def run_nsga2(*args: object, **kwargs: object) -> NSGA2RunResult:
    """Lazy bridge so importing Stage-0 composition does not import pymoo."""
    from cmd.optimization.solvers.pymoo import run_nsga2 as execute

    return execute(*args, **kwargs)


def _require_finite(name: str, value: float) -> None:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


def _positive_finite(name: str, value: float) -> float:
    _require_finite(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return float(value)


def _require_positive_finite(name: str, value: float) -> None:
    _positive_finite(name, value)


def _require_nonnegative_finite(name: str, value: float) -> None:
    _require_finite(name, value)
    if value < 0.0:
        raise ValueError(f"{name} must be non-negative")


__all__ = [
    "LinkageStage0Config", "LinkageStage0Evaluation", "LinkageStage0Result",
    "Stage0Evaluator", "build_linkage_stage0_problem", "run_linkage_stage0",
    "stage0_input_samples",
]
