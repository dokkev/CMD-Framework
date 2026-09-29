"""Reusable sampled linkage motion and workspace evaluation.

This module depends only on canonical linkage morphology and can be reused by
optimization stages or other geometry/mechanics evaluators.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real
from typing import Literal

from . import (
    FingerLinkage,
    FingerLinkageParameters,
    KinematicClosureError,
    LinkageKinematics,
    LinkagePose,
    PointXZ,
)


@dataclass(frozen=True)
class LinkageMotionEvaluation:
    """Solved poses and derived kinematic metrics at explicit input samples."""

    parameters: FingerLinkageParameters
    linkage: FingerLinkage | None
    input_flexion_rad: tuple[float, ...]
    q1_rad: tuple[float, ...]
    poses: tuple[LinkagePose, ...]
    unwrapped_q2_rad: tuple[float, ...]
    unwrapped_q3_rad: tuple[float, ...]
    q2_flexion_rad: tuple[float, ...]
    q3_flexion_rad: tuple[float, ...]
    dq2_flexion_dq1_flexion: tuple[float, ...]
    dq3_flexion_dq1_flexion: tuple[float, ...]
    transmission_angle_loop_1_rad: tuple[float, ...]
    transmission_angle_loop_2_rad: tuple[float, ...]
    transmission_quality_loop_1: tuple[float, ...]
    transmission_quality_loop_2: tuple[float, ...]
    complete_sweep: bool
    solved_sample_count: int
    failure_index: int | None
    failure_reason: str | None

    @property
    def requested_sample_count(self) -> int:
        return len(self.input_flexion_rad)


@dataclass(frozen=True)
class LinkageMotionEvaluator:
    """Evaluate a linkage at a caller-provided increasing flexion sequence."""

    input_flexion_rad: tuple[float, ...]

    def __post_init__(self) -> None:
        samples = tuple(_finite("input_flexion_rad sample", value) for value in self.input_flexion_rad)
        if not samples:
            raise ValueError("input_flexion_rad must contain at least one sample")
        if any(value < 0.0 for value in samples):
            raise ValueError("input_flexion_rad samples must be non-negative")
        if any(right <= left for left, right in zip(samples, samples[1:])):
            raise ValueError("input_flexion_rad samples must be strictly increasing")
        object.__setattr__(self, "input_flexion_rad", samples)

    def evaluate(self, parameters: FingerLinkageParameters) -> LinkageMotionEvaluation:
        if not isinstance(parameters, FingerLinkageParameters):
            raise TypeError("parameters must be FingerLinkageParameters")
        input_flexion = self.input_flexion_rad
        q1_values = tuple(-value for value in input_flexion)
        try:
            linkage = FingerLinkage(parameters)
            solver = LinkageKinematics(linkage)
        except (ValueError, ArithmeticError) as exc:
            return _finish_motion(
                parameters, None, input_flexion, q1_values, (), 0,
                f"{type(exc).__name__}: {exc}",
            )

        poses: list[LinkagePose] = []
        failure_index: int | None = None
        failure_reason: str | None = None
        for index, q1 in enumerate(q1_values):
            try:
                poses.append(solver.solve(q1))
            except (KinematicClosureError, ValueError, ArithmeticError) as exc:
                failure_index = index
                failure_reason = f"{type(exc).__name__}: {exc}"
                break
        return _finish_motion(
            parameters,
            linkage,
            input_flexion,
            q1_values,
            tuple(poses),
            failure_index,
            failure_reason,
        )


def _finish_motion(
    parameters: FingerLinkageParameters,
    linkage: FingerLinkage | None,
    input_flexion: tuple[float, ...],
    q1_values: tuple[float, ...],
    poses: tuple[LinkagePose, ...],
    failure_index: int | None,
    failure_reason: str | None,
) -> LinkageMotionEvaluation:
    q2_unwrapped = unwrap_principal_angles(tuple(pose.q2_rad for pose in poses))
    q3_unwrapped = unwrap_principal_angles(tuple(pose.q3_rad for pose in poses))
    q2_flexion = tuple(-angle for angle in q2_unwrapped)
    q3_flexion = tuple(-angle for angle in q3_unwrapped)
    angle_1: list[float] = []
    angle_2: list[float] = []
    quality_1: list[float] = []
    quality_2: list[float] = []
    for pose in poses:
        angle, quality = _transmission_angle_and_quality(pose.bp, pose.a, pose.op)
        angle_1.append(angle)
        quality_1.append(quality)
        angle, quality = _transmission_angle_and_quality(pose.cp, pose.b, pose.ap)
        angle_2.append(angle)
        quality_2.append(quality)
    solved_samples = input_flexion[: len(poses)]
    return LinkageMotionEvaluation(
        parameters=parameters,
        linkage=linkage,
        input_flexion_rad=input_flexion,
        q1_rad=q1_values,
        poses=poses,
        unwrapped_q2_rad=q2_unwrapped,
        unwrapped_q3_rad=q3_unwrapped,
        q2_flexion_rad=q2_flexion,
        q3_flexion_rad=q3_flexion,
        dq2_flexion_dq1_flexion=differentiate_flexion_samples(solved_samples, q2_flexion),
        dq3_flexion_dq1_flexion=differentiate_flexion_samples(solved_samples, q3_flexion),
        transmission_angle_loop_1_rad=tuple(angle_1),
        transmission_angle_loop_2_rad=tuple(angle_2),
        transmission_quality_loop_1=tuple(quality_1),
        transmission_quality_loop_2=tuple(quality_2),
        complete_sweep=len(poses) == len(input_flexion),
        solved_sample_count=len(poses),
        failure_index=failure_index,
        failure_reason=failure_reason,
    )


def unwrap_principal_angles(angles: tuple[float, ...]) -> tuple[float, ...]:
    if not angles:
        return ()
    unwrapped = [angles[0]]
    for angle in angles[1:]:
        unwrapped.append(unwrapped[-1] + math.remainder(angle - unwrapped[-1], 2.0 * math.pi))
    return tuple(unwrapped)


def differentiate_flexion_samples(
    input_flexion_rad: tuple[float, ...], joint_flexion_rad: tuple[float, ...]
) -> tuple[float, ...]:
    """Return causal interval ratios, indexed by their ending sample minus one."""
    if len(input_flexion_rad) != len(joint_flexion_rad):
        raise ValueError("finite-difference coordinates must have equal lengths")
    return tuple(
        (joint_flexion_rad[i] - joint_flexion_rad[i - 1])
        / (input_flexion_rad[i] - input_flexion_rad[i - 1])
        for i in range(1, len(input_flexion_rad))
    )


def _transmission_angle_and_quality(
    vertex: PointXZ, point_0: PointXZ, point_1: PointXZ
) -> tuple[float, float]:
    vector_0 = PointXZ(point_0.x_mm - vertex.x_mm, point_0.z_mm - vertex.z_mm)
    vector_1 = PointXZ(point_1.x_mm - vertex.x_mm, point_1.z_mm - vertex.z_mm)
    norm_0 = math.hypot(vector_0.x_mm, vector_0.z_mm)
    norm_1 = math.hypot(vector_1.x_mm, vector_1.z_mm)
    if norm_0 == 0.0 or norm_1 == 0.0:
        return 0.0, 0.0
    ux0, uz0 = vector_0.x_mm / norm_0, vector_0.z_mm / norm_0
    ux1, uz1 = vector_1.x_mm / norm_1, vector_1.z_mm / norm_1
    angle = math.atan2(abs(ux0 * uz1 - uz0 * ux1), ux0 * ux1 + uz0 * uz1)
    margin = min(angle, math.pi - angle)
    return angle, min(1.0, max(0.0, math.sin(margin)))


WorkspaceReason = Literal[
    "kinematic_failure", "transmission_loop_1", "transmission_loop_2",
    "q2_minimum_motion_ratio", "q3_minimum_motion_ratio",
    "q2_maximum_motion_ratio", "q3_maximum_motion_ratio",
    "q2_motion_ratio_unavailable", "q3_motion_ratio_unavailable", "search_horizon",
]


@dataclass(frozen=True)
class LinkageWorkspaceCriteria:
    """Motion-health criteria used to measure a contiguous usable workspace."""

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
            lower = _finite(f"{joint}_minimum_motion_ratio", getattr(self, f"{joint}_minimum_motion_ratio"))
            upper = _finite(f"{joint}_maximum_motion_ratio", getattr(self, f"{joint}_maximum_motion_ratio"))
            if lower < 0.0:
                raise ValueError(f"{joint}_minimum_motion_ratio must be non-negative")
            if upper < lower:
                raise ValueError(f"{joint}_maximum_motion_ratio must be greater than or equal to its minimum")
        for name in (
            "minimum_transmission_margin_rad", "q2_minimum_motion_ratio",
            "q3_minimum_motion_ratio", "q2_maximum_motion_ratio", "q3_maximum_motion_ratio",
        ):
            object.__setattr__(self, name, float(getattr(self, name)))


@dataclass(frozen=True)
class LinkageWorkspaceEvaluation:
    """Workspace extent and distal-link C-motion diagnostics."""

    closure_input_workspace_rad: float
    usable_input_workspace_rad: float
    distal_endpoint_excursion_mm: float
    distal_path_length_mm: float
    distal_x_excursion_mm: float
    dorsal_z_excursion_mm: float
    closure_sample_count: int
    usable_sample_count: int
    first_unusable_index: int | None
    limiting_reasons: tuple[WorkspaceReason, ...]


def evaluate_linkage_workspace(
    motion: LinkageMotionEvaluation, criteria: LinkageWorkspaceCriteria
) -> LinkageWorkspaceEvaluation:
    """Measure the contiguous prefix that satisfies the supplied criteria."""
    if not motion.input_flexion_rad:
        return LinkageWorkspaceEvaluation(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, ("kinematic_failure",))
    closure_count = min(motion.solved_sample_count, len(motion.input_flexion_rad))
    closure = motion.input_flexion_rad[closure_count - 1] if closure_count else 0.0
    usable_count = 0
    first_unusable: int | None = None
    reasons: tuple[WorkspaceReason, ...] = ()
    for index in range(len(motion.input_flexion_rad)):
        reasons = _sample_limiting_reasons(motion, criteria, index)
        if reasons:
            first_unusable = index
            break
        usable_count += 1
    if first_unusable is None:
        usable = motion.input_flexion_rad[-1]
        reasons = ("search_horizon",)
    else:
        usable = motion.input_flexion_rad[usable_count - 1] if usable_count else 0.0
    points = tuple(pose.c for pose in motion.poses[:usable_count])
    if points:
        start, end = points[0], points[-1]
        endpoint = start.distance_to(end)
        x_excursion = end.x_mm - start.x_mm
        z_excursion = end.z_mm - start.z_mm
        path = math.fsum(a.distance_to(b) for a, b in zip(points, points[1:]))
    else:
        endpoint = x_excursion = z_excursion = path = 0.0
    return LinkageWorkspaceEvaluation(
        closure, usable, endpoint, path, x_excursion, z_excursion,
        closure_count, usable_count, first_unusable, reasons,
    )


def _sample_limiting_reasons(
    motion: LinkageMotionEvaluation, criteria: LinkageWorkspaceCriteria, index: int
) -> tuple[WorkspaceReason, ...]:
    if index >= motion.solved_sample_count:
        return ("kinematic_failure",)
    reasons: list[WorkspaceReason] = []
    for angles, reason in (
        (motion.transmission_angle_loop_1_rad, "transmission_loop_1"),
        (motion.transmission_angle_loop_2_rad, "transmission_loop_2"),
    ):
        if index >= len(angles):
            reasons.append(reason)
        elif min(angles[index], math.pi - angles[index]) < criteria.minimum_transmission_margin_rad:
            reasons.append(reason)
    if index > 0:
        for joint in ("q2", "q3"):
            ratios = getattr(motion, f"d{joint}_flexion_dq1_flexion")
            interval = index - 1
            if interval >= len(ratios):
                reasons.append(f"{joint}_motion_ratio_unavailable")
                continue
            ratio = ratios[interval]
            if ratio < getattr(criteria, f"{joint}_minimum_motion_ratio"):
                reasons.append(f"{joint}_minimum_motion_ratio")
            if ratio > getattr(criteria, f"{joint}_maximum_motion_ratio"):
                reasons.append(f"{joint}_maximum_motion_ratio")
    return tuple(reasons)


def _finite(name: str, value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


__all__ = [
    "LinkageMotionEvaluation", "LinkageMotionEvaluator", "LinkageWorkspaceCriteria",
    "LinkageWorkspaceEvaluation", "WorkspaceReason", "differentiate_flexion_samples",
    "evaluate_linkage_workspace", "unwrap_principal_angles",
]
