"""Core/search workspace semantics for CLEAR linkage evaluations."""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real
from typing import Literal

from .evaluator import LinkageEvaluation, differentiate_flexion_samples


_CORE_SAMPLE_RELATIVE_TOLERANCE = 1e-12

WorkspaceReason = Literal[
    "kinematic_failure",
    "transmission_loop_1",
    "transmission_loop_2",
    "q2_minimum_motion_ratio",
    "q3_minimum_motion_ratio",
    "q2_maximum_motion_ratio",
    "q3_maximum_motion_ratio",
    "q2_motion_ratio_unavailable",
    "q3_motion_ratio_unavailable",
    "search_horizon",
]


@dataclass(frozen=True)
class LinkageWorkspaceCriteria:
    """Shared health thresholds for usable workspace and core constraints."""

    minimum_transmission_margin_rad: float
    q2_minimum_motion_ratio: float
    q3_minimum_motion_ratio: float
    q2_maximum_motion_ratio: float
    q3_maximum_motion_ratio: float

    def __post_init__(self) -> None:
        margin = _finite("minimum_transmission_margin_rad", self.minimum_transmission_margin_rad)
        if not 0.0 <= margin <= math.pi / 2.0:
            raise ValueError("minimum_transmission_margin_rad must be in [0, pi/2]")

        for joint in ("q2", "q3"):
            minimum = _finite(f"{joint}_minimum_motion_ratio", getattr(self, f"{joint}_minimum_motion_ratio"))
            maximum = _finite(f"{joint}_maximum_motion_ratio", getattr(self, f"{joint}_maximum_motion_ratio"))
            if minimum < 0.0:
                raise ValueError(f"{joint}_minimum_motion_ratio must be non-negative")
            if maximum < minimum:
                raise ValueError(
                    f"{joint}_maximum_motion_ratio must be greater than or equal to its minimum"
                )

        object.__setattr__(self, "minimum_transmission_margin_rad", margin)
        object.__setattr__(self, "q2_minimum_motion_ratio", float(self.q2_minimum_motion_ratio))
        object.__setattr__(self, "q3_minimum_motion_ratio", float(self.q3_minimum_motion_ratio))
        object.__setattr__(self, "q2_maximum_motion_ratio", float(self.q2_maximum_motion_ratio))
        object.__setattr__(self, "q3_maximum_motion_ratio", float(self.q3_maximum_motion_ratio))


@dataclass(frozen=True)
class LinkageWorkspaceEvaluation:
    """Input-angle workspace and distal-link C-motion diagnostics."""

    closure_input_workspace_rad: float
    usable_input_workspace_rad: float
    distal_endpoint_excursion_mm: float
    distal_path_length_mm: float
    distal_y_excursion_mm: float
    distal_z_excursion_mm: float
    closure_sample_count: int
    usable_sample_count: int
    first_unusable_index: int | None
    limiting_reasons: tuple[WorkspaceReason, ...]


def core_sample_indices(
    evaluation: LinkageEvaluation,
    core_input_workspace_rad: float,
) -> tuple[int, ...]:
    """Select sweep samples through an exactly represented core endpoint."""

    core = _positive_finite("core_input_workspace_rad", core_input_workspace_rad)
    if not evaluation.input_flexion_rad:
        raise ValueError("evaluation has no requested flexion samples")
    if core >= evaluation.input_flexion_rad[-1]:
        raise ValueError("core_input_workspace_rad must be below the search horizon")

    core_index = next(
        (
            index
            for index, sample in enumerate(evaluation.input_flexion_rad)
            if math.isclose(
                sample,
                core,
                rel_tol=_CORE_SAMPLE_RELATIVE_TOLERANCE,
                abs_tol=0.0,
            )
        ),
        None,
    )
    if core_index is None:
        raise ValueError("the exact core input endpoint is absent from the requested sweep")

    return tuple(
        index
        for index, sample in enumerate(evaluation.input_flexion_rad)
        if sample < core or index == core_index
    )


def core_workspace_is_complete(
    evaluation: LinkageEvaluation,
    core_input_workspace_rad: float,
) -> bool:
    """Whether every requested sample through the exact core endpoint solved."""

    indices = core_sample_indices(evaluation, core_input_workspace_rad)
    return bool(indices) and indices[-1] < evaluation.solved_sample_count


def core_motion_ratios(
    evaluation: LinkageEvaluation,
    core_input_workspace_rad: float,
    joint: Literal["q2", "q3"],
) -> tuple[float, ...]:
    """Recompute motion ratios using only flexion samples inside the core."""

    if joint not in ("q2", "q3"):
        raise ValueError("joint must be 'q2' or 'q3'")
    values = (
        evaluation.q2_flexion_rad
        if joint == "q2"
        else evaluation.q3_flexion_rad
    )
    indices = tuple(
        index
        for index in core_sample_indices(evaluation, core_input_workspace_rad)
        if index < len(values)
    )
    input_flexion = tuple(evaluation.input_flexion_rad[index] for index in indices)
    joint_flexion = tuple(values[index] for index in indices)
    return differentiate_flexion_samples(input_flexion, joint_flexion)


def evaluate_linkage_workspace(
    evaluation: LinkageEvaluation,
    criteria: LinkageWorkspaceCriteria,
) -> LinkageWorkspaceEvaluation:
    """Measure closure and the longest contiguous prefix meeting all criteria."""

    if not evaluation.input_flexion_rad:
        return LinkageWorkspaceEvaluation(
            closure_input_workspace_rad=0.0,
            usable_input_workspace_rad=0.0,
            distal_endpoint_excursion_mm=0.0,
            distal_path_length_mm=0.0,
            distal_y_excursion_mm=0.0,
            distal_z_excursion_mm=0.0,
            closure_sample_count=0,
            usable_sample_count=0,
            first_unusable_index=0,
            limiting_reasons=("kinematic_failure",),
        )

    closure_count = min(evaluation.solved_sample_count, len(evaluation.input_flexion_rad))
    closure_input_workspace = (
        evaluation.input_flexion_rad[closure_count - 1]
        if closure_count > 0
        else 0.0
    )

    usable_count = 0
    first_unusable_index: int | None = None
    limiting_reasons: tuple[WorkspaceReason, ...] = ()
    for index in range(len(evaluation.input_flexion_rad)):
        reasons = _sample_limiting_reasons(evaluation, criteria, index)
        if reasons:
            first_unusable_index = index
            limiting_reasons = reasons
            break
        usable_count += 1

    if first_unusable_index is None:
        usable_input_workspace = evaluation.input_flexion_rad[-1]
        limiting_reasons = ("search_horizon",)
    elif usable_count == 0:
        usable_input_workspace = 0.0
    else:
        usable_input_workspace = evaluation.input_flexion_rad[usable_count - 1]

    usable_points = tuple(pose.c for pose in evaluation.poses[:usable_count])
    if usable_points:
        start = usable_points[0]
        end = usable_points[-1]
        distal_endpoint_excursion = start.distance_to(end)
        distal_y_excursion = end.y_mm - start.y_mm
        distal_z_excursion = end.z_mm - start.z_mm
        distal_path_length = math.fsum(
            first.distance_to(second)
            for first, second in zip(usable_points, usable_points[1:])
        )
    else:
        distal_endpoint_excursion = 0.0
        distal_y_excursion = 0.0
        distal_z_excursion = 0.0
        distal_path_length = 0.0

    return LinkageWorkspaceEvaluation(
        closure_input_workspace_rad=closure_input_workspace,
        usable_input_workspace_rad=usable_input_workspace,
        distal_endpoint_excursion_mm=distal_endpoint_excursion,
        distal_path_length_mm=distal_path_length,
        distal_y_excursion_mm=distal_y_excursion,
        distal_z_excursion_mm=distal_z_excursion,
        closure_sample_count=closure_count,
        usable_sample_count=usable_count,
        first_unusable_index=first_unusable_index,
        limiting_reasons=limiting_reasons,
    )


def _sample_limiting_reasons(
    evaluation: LinkageEvaluation,
    criteria: LinkageWorkspaceCriteria,
    index: int,
) -> tuple[WorkspaceReason, ...]:
    if index >= evaluation.solved_sample_count:
        return ("kinematic_failure",)

    reasons: list[WorkspaceReason] = []
    for angles, reason in (
        (evaluation.transmission_angle_loop_1_rad, "transmission_loop_1"),
        (evaluation.transmission_angle_loop_2_rad, "transmission_loop_2"),
    ):
        if index >= len(angles):
            reasons.append(reason)
            continue
        margin = min(angles[index], math.pi - angles[index])
        if margin < criteria.minimum_transmission_margin_rad:
            reasons.append(reason)

    for joint in ("q2", "q3"):
        ratios = (
            evaluation.dq2_flexion_dq1_flexion
            if joint == "q2"
            else evaluation.dq3_flexion_dq1_flexion
        )
        if index >= len(ratios):
            reasons.append(f"{joint}_motion_ratio_unavailable")
            continue
        ratio = ratios[index]
        minimum = getattr(criteria, f"{joint}_minimum_motion_ratio")
        maximum = getattr(criteria, f"{joint}_maximum_motion_ratio")
        if ratio < minimum:
            reasons.append(f"{joint}_minimum_motion_ratio")
        if ratio > maximum:
            reasons.append(f"{joint}_maximum_motion_ratio")

    return tuple(reasons)


def _finite(name: str, value: float) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _positive_finite(name: str, value: float) -> float:
    value = _finite(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return value


__all__ = [
    "LinkageWorkspaceCriteria",
    "LinkageWorkspaceEvaluation",
    "WorkspaceReason",
    "core_motion_ratios",
    "core_sample_indices",
    "core_workspace_is_complete",
    "evaluate_linkage_workspace",
]
