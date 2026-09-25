"""Workspace sweep evaluation for CLEAR ordinary-finger linkage candidates."""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real

from cmd.morphology import (
    FingerLinkage,
    FingerLinkageParameters,
    KinematicClosureError,
    LinkageKinematics,
    LinkagePose,
    PointYZ,
)


@dataclass(frozen=True)
class LinkageSweepConfig:
    """Search-horizon extent and nominal uniform sample count.

    The exact core-workspace endpoint may add one point to the nominal grid.
    """

    search_horizon_rad: float
    sample_count: int = 101

    def __post_init__(self) -> None:
        if (
            isinstance(self.search_horizon_rad, bool)
            or not isinstance(self.search_horizon_rad, Real)
            or not math.isfinite(self.search_horizon_rad)
            or self.search_horizon_rad <= 0.0
        ):
            raise ValueError("search_horizon_rad must be finite and positive")
        if (
            isinstance(self.sample_count, bool)
            or not isinstance(self.sample_count, int)
            or self.sample_count < 3
        ):
            raise ValueError("sample_count must be an integer of at least 3")
        object.__setattr__(self, "search_horizon_rad", float(self.search_horizon_rad))


@dataclass(frozen=True)
class LinkageEvaluation:
    """Immutable sampled poses and geometric transmission metrics.

    Workspace-valued pose and metric tuples contain only successfully solved
    samples, in order from extension toward flexion. Input sweep coordinates
    always contain the full requested grid, including any failed samples.
    """

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
    complete_workspace: bool
    solved_sample_count: int
    failure_index: int | None
    failure_reason: str | None

    @property
    def requested_sample_count(self) -> int:
        return len(self.input_flexion_rad)


@dataclass(frozen=True)
class LinkageSweepEvaluator:
    """Evaluate a candidate using only canonical linkage kinematics."""

    config: LinkageSweepConfig
    core_input_workspace_rad: float

    def __post_init__(self) -> None:
        if (
            isinstance(self.core_input_workspace_rad, bool)
            or not isinstance(self.core_input_workspace_rad, Real)
            or not math.isfinite(self.core_input_workspace_rad)
            or not (
                0.0
                < self.core_input_workspace_rad
                < self.config.search_horizon_rad
            )
        ):
            raise ValueError(
                "core_input_workspace_rad must satisfy "
                "0 < core_input_workspace_rad < search_horizon_rad"
            )
        object.__setattr__(
            self,
            "core_input_workspace_rad",
            float(self.core_input_workspace_rad),
        )

    def evaluate(self, parameters: FingerLinkageParameters) -> LinkageEvaluation:
        if not isinstance(parameters, FingerLinkageParameters):
            raise TypeError("parameters must be FingerLinkageParameters")

        input_flexion = _requested_flexion_grid(
            self.config,
            self.core_input_workspace_rad,
        )
        q1_values = tuple(-value for value in input_flexion)
        try:
            linkage = FingerLinkage(parameters)
            solver = LinkageKinematics(linkage)
        except (ValueError, ArithmeticError) as exc:
            return self._finish(
                parameters,
                None,
                input_flexion,
                q1_values,
                (),
                failure_index=0,
                failure_reason=f"{type(exc).__name__}: {exc}",
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

        return self._finish(
            parameters,
            linkage,
            input_flexion,
            q1_values,
            tuple(poses),
            failure_index=failure_index,
            failure_reason=failure_reason,
        )

    def _finish(
        self,
        parameters: FingerLinkageParameters,
        linkage: FingerLinkage | None,
        input_flexion: tuple[float, ...],
        q1_values: tuple[float, ...],
        poses: tuple[LinkagePose, ...],
        *,
        failure_index: int | None,
        failure_reason: str | None,
    ) -> LinkageEvaluation:
        solved_s = input_flexion[: len(poses)]
        q2_unwrapped = unwrap_principal_angles(tuple(pose.q2_rad for pose in poses))
        q3_unwrapped = unwrap_principal_angles(tuple(pose.q3_rad for pose in poses))
        q2_flexion = tuple(-angle for angle in q2_unwrapped)
        q3_flexion = tuple(-angle for angle in q3_unwrapped)

        angles_1: list[float] = []
        angles_2: list[float] = []
        quality_1: list[float] = []
        quality_2: list[float] = []
        for pose in poses:
            angle, quality = _transmission_angle_and_quality(pose.bp, pose.a, pose.op)
            angles_1.append(angle)
            quality_1.append(quality)
            angle, quality = _transmission_angle_and_quality(pose.cp, pose.b, pose.ap)
            angles_2.append(angle)
            quality_2.append(quality)

        return LinkageEvaluation(
            parameters=parameters,
            linkage=linkage,
            input_flexion_rad=input_flexion,
            q1_rad=q1_values,
            poses=poses,
            unwrapped_q2_rad=q2_unwrapped,
            unwrapped_q3_rad=q3_unwrapped,
            q2_flexion_rad=q2_flexion,
            q3_flexion_rad=q3_flexion,
            dq2_flexion_dq1_flexion=_differentiate(solved_s, q2_flexion),
            dq3_flexion_dq1_flexion=_differentiate(solved_s, q3_flexion),
            transmission_angle_loop_1_rad=tuple(angles_1),
            transmission_angle_loop_2_rad=tuple(angles_2),
            transmission_quality_loop_1=tuple(quality_1),
            transmission_quality_loop_2=tuple(quality_2),
            complete_workspace=len(poses) == len(input_flexion),
            solved_sample_count=len(poses),
            failure_index=failure_index,
            failure_reason=failure_reason,
        )


def unwrap_principal_angles(angles: tuple[float, ...]) -> tuple[float, ...]:
    """Unwrap principal angles continuously in the supplied sample order."""

    if not angles:
        return ()
    unwrapped = [angles[0]]
    for angle in angles[1:]:
        increment = math.remainder(angle - unwrapped[-1], 2.0 * math.pi)
        unwrapped.append(unwrapped[-1] + increment)
    return tuple(unwrapped)


def _differentiate(x: tuple[float, ...], y: tuple[float, ...]) -> tuple[float, ...]:
    """Use one-sided endpoint and centered interior finite differences."""

    if len(x) != len(y):
        raise ValueError("finite-difference coordinates must have equal lengths")
    if len(x) < 2:
        return ()

    derivatives = [(y[1] - y[0]) / (x[1] - x[0])]
    derivatives.extend(
        (y[index + 1] - y[index - 1]) / (x[index + 1] - x[index - 1])
        for index in range(1, len(x) - 1)
    )
    derivatives.append((y[-1] - y[-2]) / (x[-1] - x[-2]))
    return tuple(derivatives)


def differentiate_flexion_samples(
    input_flexion_rad: tuple[float, ...],
    joint_flexion_rad: tuple[float, ...],
) -> tuple[float, ...]:
    """Return sampled ``d joint_flexion / d input_flexion`` ratios."""

    return _differentiate(input_flexion_rad, joint_flexion_rad)


def _requested_flexion_grid(
    config: LinkageSweepConfig,
    core_input_workspace_rad: float,
) -> tuple[float, ...]:
    uniform = [
        config.search_horizon_rad * index / (config.sample_count - 1)
        for index in range(config.sample_count)
    ]
    relative_tolerance = 1e-12
    matching_interior_index = next(
        (
            index
            for index in range(1, len(uniform) - 1)
            if math.isclose(
                uniform[index],
                core_input_workspace_rad,
                rel_tol=relative_tolerance,
                abs_tol=0.0,
            )
        ),
        None,
    )
    if matching_interior_index is None:
        uniform.append(core_input_workspace_rad)
    else:
        uniform[matching_interior_index] = core_input_workspace_rad
    return tuple(sorted(uniform))


def _transmission_angle_and_quality(
    vertex: PointYZ,
    point_0: PointYZ,
    point_1: PointYZ,
) -> tuple[float, float]:
    vector_0 = PointYZ(point_0.y_mm - vertex.y_mm, point_0.z_mm - vertex.z_mm)
    vector_1 = PointYZ(point_1.y_mm - vertex.y_mm, point_1.z_mm - vertex.z_mm)
    norm_0 = math.hypot(vector_0.y_mm, vector_0.z_mm)
    norm_1 = math.hypot(vector_1.y_mm, vector_1.z_mm)
    if norm_0 == 0.0 or norm_1 == 0.0:
        return 0.0, 0.0

    unit_0 = PointYZ(vector_0.y_mm / norm_0, vector_0.z_mm / norm_0)
    unit_1 = PointYZ(vector_1.y_mm / norm_1, vector_1.z_mm / norm_1)
    cross = unit_0.y_mm * unit_1.z_mm - unit_0.z_mm * unit_1.y_mm
    dot = unit_0.y_mm * unit_1.y_mm + unit_0.z_mm * unit_1.z_mm
    angle = math.atan2(abs(cross), dot)
    margin = min(angle, math.pi - angle)
    quality = min(1.0, max(0.0, math.sin(margin)))
    return angle, quality


__all__ = [
    "LinkageEvaluation",
    "LinkageSweepConfig",
    "LinkageSweepEvaluator",
    "differentiate_flexion_samples",
    "unwrap_principal_angles",
]
