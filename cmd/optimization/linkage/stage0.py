"""Stage-0 linkage formulation composed over reusable morphology motion."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Real
from typing import TYPE_CHECKING

from cmd.morphology.linkage_motion import (
    LinkageMotionEvaluation,
    LinkageMotionEvaluator,
    LinkageWorkspaceCriteria,
    LinkageWorkspaceEvaluation,
    evaluate_linkage_workspace,
)
from cmd.morphology.linkage_parameter import FingerLinkageParameters
from cmd.morphology.linkage_seed import BASELINE_LINKAGE_V0
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
from .objectives import (
    CoordinationRatioVariation,
    DistalEndpointExcursion,
    WorstCaseTransmissionQuality,
)
from .parameterization import LinkageBaselineParameterization

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
    """One Stage-0 evaluation with motion and its derived workspace cached."""

    motion: LinkageMotionEvaluation
    workspace: LinkageWorkspaceEvaluation


@dataclass(frozen=True)
class Stage0Evaluator:
    """Apply Stage-0 sample semantics, then evaluate motion and workspace once."""

    config: LinkageStage0Config

    def evaluate(self, parameters: FingerLinkageParameters) -> LinkageStage0Evaluation:
        samples = stage0_input_samples(self.config)
        motion = LinkageMotionEvaluator(samples).evaluate(parameters)
        workspace = evaluate_linkage_workspace(motion, self.config.workspace_criteria)
        return LinkageStage0Evaluation(motion, workspace)


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


def core_sample_indices(
    motion: LinkageMotionEvaluation, core_input_workspace_rad: float
) -> tuple[int, ...]:
    """Return motion samples from extension through the exact Stage-0 core."""
    core = _positive_finite("core_input_workspace_rad", core_input_workspace_rad)
    if not motion.input_flexion_rad or core >= motion.input_flexion_rad[-1]:
        raise ValueError("core_input_workspace_rad must be below the search horizon")
    core_index = next(
        (
            index
            for index, sample in enumerate(motion.input_flexion_rad)
            if math.isclose(sample, core, rel_tol=1e-12, abs_tol=0.0)
        ),
        None,
    )
    if core_index is None:
        raise ValueError("the exact core input endpoint is absent from the requested sweep")
    return tuple(range(core_index + 1))


def core_workspace_is_complete(
    motion: LinkageMotionEvaluation, core_input_workspace_rad: float
) -> bool:
    indices = core_sample_indices(motion, core_input_workspace_rad)
    return bool(indices) and indices[-1] < motion.solved_sample_count


def core_motion_ratios(
    motion: LinkageMotionEvaluation,
    core_input_workspace_rad: float,
    joint: str,
) -> tuple[float, ...]:
    if joint not in ("q2", "q3"):
        raise ValueError("joint must be 'q2' or 'q3'")
    indices = core_sample_indices(motion, core_input_workspace_rad)
    ratios = getattr(motion, f"d{joint}_flexion_dq1_flexion")
    return ratios[: len(indices) - 1]


def build_linkage_stage0_problem(
    variables: Sequence[BoxVariable], config: LinkageStage0Config
) -> OptimizationProblem[FingerLinkageParameters, LinkageStage0Evaluation]:
    """Compose the fixed Stage-0 objective/constraint formulation."""
    bc_variable = next((v for v in variables if v.name == "bc_length_mm"), None)
    if bc_variable is None:
        raise ValueError("Stage-0 variables must include bc_length_mm")
    if bc_variable.lower <= 0.0:
        raise ValueError("bc_length_mm lower bound must be greater than zero")
    core = config.core_input_workspace_rad
    criteria = config.workspace_criteria
    return OptimizationProblem(
        variables=variables,
        parameterization=LinkageBaselineParameterization(),
        evaluator=Stage0Evaluator(config),
        objectives=(
            WorstCaseTransmissionQuality(core),
            CoordinationRatioVariation(core),
            DistalEndpointExcursion(),
        ),
        equalities=(),
        inequalities=(
            CoreWorkspaceKinematicValidity(core),
            MinimumUsableInputWorkspace(config.minimum_required_input_workspace_rad),
            MinimumAnchorDorsalOffset(config.minimum_anchor_dorsal_offset_mm),
            MinimumTransmissionMargin(criteria, core),
            MonotonicFlexion("q2", criteria, core),
            MonotonicFlexion("q3", criteria, core),
            MaximumMotionRatio("q2", criteria, core),
            MaximumMotionRatio("q3", criteria, core),
            MinimumFlexionAtInput(
                "q2", core, config.q2_minimum_endpoint_flexion_rad
            ),
            MinimumFlexionAtInput(
                "q3", core, config.q3_minimum_endpoint_flexion_rad
            ),
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
    "Stage0Evaluator", "build_linkage_stage0_problem", "core_motion_ratios",
    "core_sample_indices", "core_workspace_is_complete", "run_linkage_stage0",
    "stage0_input_samples",
]
