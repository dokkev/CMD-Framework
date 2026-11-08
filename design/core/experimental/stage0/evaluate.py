"""Solver-independent Stage-0 evaluation: three objectives and fixed-shape constraints.

q1, q2 and q3 are the MCP, PIP and DIP flexion angles. Each design is swept to
its motion limit: the last sample of the contiguous healthy range (closure,
transmission margin and motion-ratio bounds), at most the search horizon.

Objectives, measured at the grasp pose (the grasp pose whose pad best faces
OA') or over extension up to it:

- worst-case transmission quality of both coupler loops from extension to the
  grasp pose (maximize);
- facing error: the angle between the pad direction B->C and A'->O at the
  grasp pose, i.e. how far the pad is from facing back along the proximal
  segment OA' (minimize). Lengths need not match and the pad need not be
  aligned end to end with OA';
- grasp MCP angle: the MCP flexion at the grasp pose (minimize: the earlier the
  pad faces OA', the better).

A grasp pose is a sample from extension to the motion limit where both pad
ends B and C lie on the palmar side of OA' at least ``minimum_grasp_gap_mm``
away and in front of it (each projects onto OA' between O and A').

Constraint groups use ``g <= 0`` and keep a fixed residual count per study:

- ``usable_motion``: the healthy range reaches ``minimum_motion_range_rad``
  and spans at least one sample step;
- ``extended_length``: ``|OC|`` at ``q = 0`` within the configured range;
- ``minimum_pin_spacing``: bearing pins sharing a rigid link (A-A' on the
  proximal, A-B' and B-B' on the middle, B-C' on the distal link) at least the
  configured centre distance apart;
- ``grasp_pose``: some pose within the motion range is a grasp pose
  (residual: the smallest, over those poses, of the summed gap shortfalls and
  projection overshoots of B and C, in mm).

The post-grasp travel is how much further the MCP can flex past the grasp
pose while the motion stays safe: both loops close, both transmission margins
stay at or above ``criteria.minimum_transmission_margin_rad`` and neither
q2 nor q3 reverses (motion ratio > 0), capped at the search horizon. The grasp
pose plus this travel is the safe operating limit; a mechanical stopper belongs
beyond the required safe range and before the first failure.

The summed coefficient of variation of the q2 and q3 motion ratios
(``ratio_variation``) is kept as a diagnostic.

Point-C excursion is deliberately a diagnostic, not an objective: it rewards
link size and does not imply grasp quality.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

from core.experimental.linkage import (
    FingerLinkageParameters,
    LinkageMotionEvaluation,
    LinkageMotionEvaluator,
    LinkagePose,
    LinkageWorkspaceCriteria,
    LinkageWorkspaceEvaluation,
    PointXZ,
    evaluate_linkage_workspace,
)

from core.experimental.stage0.config import Stage0Config


_CV_EPSILON = 1e-9


@dataclass(frozen=True)
class Stage0Evaluation:
    """Objectives, constraint residuals, and diagnostics for one design."""

    motion: LinkageMotionEvaluation
    workspace: LinkageWorkspaceEvaluation
    worst_transmission_quality: float
    ratio_variation: float
    grasp: Grasp | None
    """Grasp pose with the smallest facing error, if any sample is a grasp pose."""
    criteria: LinkageWorkspaceCriteria
    constraints: Mapping[str, tuple[float, ...]]

    @property
    def g(self) -> tuple[float, ...]:
        return tuple(value for group in self.constraints.values() for value in group)

    @property
    def feasible(self) -> bool:
        return all(value <= 0.0 for value in self.g)

    def max_violation(self, group: str) -> float:
        return max(0.0, *self.constraints[group])

    @property
    def limit_index(self) -> int:
        """Index of the last healthy sample (the motion limit); -1 if none."""

        return self.workspace.usable_sample_count - 1

    @property
    def motion_range_rad(self) -> float:
        """MCP flexion at the motion limit."""

        return self.workspace.usable_input_workspace_rad

    @property
    def facing_error_rad(self) -> float:
        return self.grasp.facing_error_rad if self.grasp else math.inf

    @property
    def grasp_mcp_rad(self) -> float:
        return self.motion.input_flexion_rad[self.grasp.index] if self.grasp else math.inf

    @property
    def post_grasp_travel_rad(self) -> float:
        if self.grasp is None:
            return 0.0
        return post_grasp_travel_rad(self.motion, self.grasp.index, self.criteria)


class Stage0Evaluator:
    """Evaluate designs on the study's fixed input-flexion sample grid."""

    def __init__(self, config: Stage0Config) -> None:
        self.config = config
        self.samples = input_samples(config)
        self._motion = LinkageMotionEvaluator(self.samples)

    @property
    def constraint_count(self) -> int:
        # Usable motion, 2 extended-length bounds, 4 pin spacings, grasp pose,
        # post-grasp safety.
        return 9

    def __call__(self, parameters: FingerLinkageParameters) -> Stage0Evaluation:
        config = self.config
        motion = self._motion.evaluate(parameters)
        workspace = evaluate_linkage_workspace(motion, config.criteria)
        last = workspace.usable_sample_count - 1
        usable = last >= 1
        grasp = find_grasp(motion, last, config.minimum_grasp_gap_mm) if usable else None
        violations = [
            grasp_violation_mm(pose, config.minimum_grasp_gap_mm)
            for pose in motion.poses[: last + 1]
        ]
        return Stage0Evaluation(
            motion=motion,
            workspace=workspace,
            worst_transmission_quality=(
                worst_transmission_quality(motion, grasp.index) if grasp else 0.0
            ),
            ratio_variation=ratio_variation(motion, last) if usable else math.inf,
            grasp=grasp,
            criteria=config.criteria,
            constraints=MappingProxyType({
                "usable_motion": (
                    max(self.samples[1], config.minimum_motion_range_rad)
                    - workspace.usable_input_workspace_rad,
                ),
                "extended_length": extended_length_residuals(
                    parameters, config.extended_length_bounds_mm
                ),
                "minimum_pin_spacing": pin_spacing_residuals(
                    parameters, config.minimum_pin_spacing_mm
                ),
                "grasp_pose": (min(violations, default=config.minimum_grasp_gap_mm),),
                "post_grasp_safety": (
                    config.minimum_post_grasp_travel_rad
                    - (
                        post_grasp_travel_rad(motion, grasp.index, config.criteria)
                        if grasp
                        else 0.0
                    ),
                ),
            }),
        )


def input_samples(config: Stage0Config) -> tuple[float, ...]:
    """Uniform MCP samples from extension to the search horizon."""

    count = config.sample_count
    return tuple(config.search_horizon_rad * index / (count - 1) for index in range(count))


def pad_rotation_rad(motion: LinkageMotionEvaluation) -> tuple[float, ...]:
    """Pad (BC) rotation from +X toward flexion, ``q1 + q2 + q3``, per solved sample."""

    return tuple(
        q1 + q2 + q3
        for q1, q2, q3 in zip(motion.input_flexion_rad, motion.q2_flexion_rad, motion.q3_flexion_rad)
    )


def worst_transmission_quality(motion: LinkageMotionEvaluation, last: int) -> float:
    return min(
        *motion.transmission_quality_loop_1[: last + 1],
        *motion.transmission_quality_loop_2[: last + 1],
    )


def ratio_variation(motion: LinkageMotionEvaluation, last: int) -> float:
    return _coefficient_of_variation(
        motion.dq2_flexion_dq1_flexion[:last]
    ) + _coefficient_of_variation(motion.dq3_flexion_dq1_flexion[:last])


@dataclass(frozen=True)
class Grasp:
    """The grasp pose within the motion range whose pad best faces OA'."""

    index: int
    facing_error_rad: float
    gap_mm: float
    """Smaller palmar distance of B and C from line OA'."""
    b_along_mm: float
    c_along_mm: float
    """B and C projected onto OA', measured from O (0 at O, |OA'| at A')."""


def facing_error_rad(pose: LinkagePose) -> float:
    """Angle between B->C and A'->O; zero when the pad faces back along OA'."""

    bc = (pose.c.x_mm - pose.b.x_mm, pose.c.z_mm - pose.b.z_mm)
    apo = (pose.o.x_mm - pose.ap.x_mm, pose.o.z_mm - pose.ap.z_mm)
    cross = bc[0] * apo[1] - bc[1] * apo[0]
    dot = bc[0] * apo[0] + bc[1] * apo[1]
    return abs(math.atan2(cross, dot))


def pad_offsets_from_oa_mm(
    pose: LinkagePose,
) -> tuple[tuple[float, float], tuple[float, float], float]:
    """B and C relative to OA' as (along from O, palmar gap) pairs, plus ``|OA'|``."""

    ux, uz = pose.ap.x_mm - pose.o.x_mm, pose.ap.z_mm - pose.o.z_mm
    length = math.hypot(ux, uz)
    ux, uz = ux / length, uz / length

    def offset(point: PointXZ) -> tuple[float, float]:
        px, pz = point.x_mm - pose.o.x_mm, point.z_mm - pose.o.z_mm
        # The palmar normal of OA' is OA' turned toward flexion: (u_z, -u_x).
        return px * ux + pz * uz, px * uz - pz * ux

    return offset(pose.b), offset(pose.c), length


def grasp_violation_mm(pose: LinkagePose, minimum_gap_mm: float) -> float:
    """Summed gap shortfall and projection overshoot of B and C; zero for a grasp pose."""

    b, c, length = pad_offsets_from_oa_mm(pose)
    return math.fsum(
        max(0.0, minimum_gap_mm - gap) + max(0.0, -along) + max(0.0, along - length)
        for along, gap in (b, c)
    )


def post_grasp_travel_rad(
    motion: LinkageMotionEvaluation, grasp_index: int, criteria: LinkageWorkspaceCriteria
) -> float:
    """MCP flexion from the grasp pose to the last sample of contiguous safe motion.

    A sample is safe when both loops close, both transmission margins meet the
    criteria, and q2 and q3 kept flexing (ratio > 0) over the interval into it.
    """

    safe = grasp_index
    for index in range(grasp_index + 1, motion.solved_sample_count):
        unsafe_transmission = any(
            criteria.transmission_shortfall_rad(angles[index]) > 0.0
            for angles in (motion.transmission_angle_loop_1_rad, motion.transmission_angle_loop_2_rad)
        )
        reversing = (
            motion.dq2_flexion_dq1_flexion[index - 1] <= 0.0
            or motion.dq3_flexion_dq1_flexion[index - 1] <= 0.0
        )
        if unsafe_transmission or reversing:
            break
        safe = index
    return motion.input_flexion_rad[safe] - motion.input_flexion_rad[grasp_index]


def find_grasp(
    motion: LinkageMotionEvaluation, last: int, minimum_gap_mm: float
) -> Grasp | None:
    """Grasp pose in ``0..last`` with the smallest facing error, if any (earliest on ties)."""

    best = None
    for index, pose in enumerate(motion.poses[: last + 1]):
        if grasp_violation_mm(pose, minimum_gap_mm) > 0.0:
            continue
        error = facing_error_rad(pose)
        if best is None or error < best.facing_error_rad:
            (b_along, b_gap), (c_along, c_gap), _ = pad_offsets_from_oa_mm(pose)
            best = Grasp(index, error, min(b_gap, c_gap), b_along, c_along)
    return best


def extended_length_mm(parameters: FingerLinkageParameters) -> float:
    """Return ``|OC|`` in the fully extended reference pose (``C = B + (bc, 0)``)."""

    a, b = parameters.a_from_o, parameters.b_from_a
    return math.hypot(a.x_mm + b.x_mm + parameters.bc_length_mm, a.z_mm + b.z_mm)


def extended_length_residuals(
    parameters: FingerLinkageParameters, bounds_mm: tuple[float, float]
) -> tuple[float, float]:
    length = extended_length_mm(parameters)
    shortest, longest = bounds_mm
    return (shortest - length, length - longest)


def pin_spacings_mm(parameters: FingerLinkageParameters) -> tuple[float, float, float, float]:
    """Centre distances |AA'|, |AB'|, |BB'|, |BC'| of pins sharing one rigid link."""

    a, ap = parameters.a_from_o, parameters.ap_from_o
    b, bp = parameters.b_from_a, parameters.bp_from_a
    return (
        math.hypot(ap.x_mm - a.x_mm, ap.z_mm - a.z_mm),
        math.hypot(bp.x_mm, bp.z_mm),
        math.hypot(bp.x_mm - b.x_mm, bp.z_mm - b.z_mm),
        math.hypot(parameters.cp_from_b.x_mm, parameters.cp_from_b.z_mm),
    )


def pin_spacing_residuals(
    parameters: FingerLinkageParameters, minimum_mm: float
) -> tuple[float, float, float, float]:
    return tuple(minimum_mm - spacing for spacing in pin_spacings_mm(parameters))


def _coefficient_of_variation(values: tuple[float, ...]) -> float:
    if not values:
        return math.inf
    mean = math.fsum(values) / len(values)
    variance = math.fsum((value - mean) ** 2 for value in values) / len(values)
    return math.sqrt(variance) / (abs(mean) + _CV_EPSILON)


__all__ = [
    "Stage0Evaluation",
    "Stage0Evaluator",
    "extended_length_mm",
    "extended_length_residuals",
    "input_samples",
    "Grasp",
    "facing_error_rad",
    "find_grasp",
    "grasp_violation_mm",
    "pad_offsets_from_oa_mm",
    "pad_rotation_rad",
    "pin_spacing_residuals",
    "pin_spacings_mm",
    "post_grasp_travel_rad",
    "ratio_variation",
    "worst_transmission_quality",
]
