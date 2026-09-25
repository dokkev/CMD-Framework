"""Composable hard feasibility constraints for linkage sweep evaluations."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from numbers import Real
from typing import Literal

from .evaluator import LinkageEvaluation


JointName = Literal["q2", "q3"]


@dataclass(frozen=True)
class FullWorkspaceKinematicValidity:
    """Require every requested flexion sample to close on the reference branch."""

    name: str = "full_workspace_kinematic_validity"

    def residual(self, context: LinkageEvaluation) -> tuple[float, ...]:
        requested = context.requested_sample_count
        if requested == 0:
            return (1.0,)
        return (1.0 - context.solved_sample_count / requested,)


@dataclass(frozen=True)
class MinimumAnchorDorsalOffset:
    """Require A' and B' to be at least ``minimum_offset_mm`` above B."""

    minimum_offset_mm: float
    name: str = "minimum_anchor_dorsal_offset"

    def __post_init__(self) -> None:
        _require_nonnegative_finite("minimum_offset_mm", self.minimum_offset_mm)

    def residual(self, context: LinkageEvaluation) -> tuple[float, ...]:
        parameters = context.parameters
        a_z = parameters.a_from_o.z_mm
        b_z = a_z + parameters.b_from_a.z_mm
        a_prime_z = parameters.a_prime_from_o.z_mm
        b_prime_z = a_z + parameters.b_prime_from_a.z_mm
        return (
            self.minimum_offset_mm - (a_prime_z - b_z),
            self.minimum_offset_mm - (b_prime_z - b_z),
        )


@dataclass(frozen=True)
class MinimumTransmissionMargin:
    """Require every loop/sample margin from a singular angle to exceed a limit."""

    minimum_margin_rad: float
    name: str = "minimum_transmission_margin"

    def __post_init__(self) -> None:
        if (
            isinstance(self.minimum_margin_rad, bool)
            or not isinstance(self.minimum_margin_rad, Real)
            or not math.isfinite(self.minimum_margin_rad)
            or not 0.0 <= self.minimum_margin_rad <= math.pi / 2.0
        ):
            raise ValueError("minimum_margin_rad must be finite and within [0, pi/2]")

    def residual(self, context: LinkageEvaluation) -> tuple[float, ...]:
        residuals: list[float] = []
        for index in range(context.requested_sample_count):
            for angles in (
                context.transmission_angle_loop_1_rad,
                context.transmission_angle_loop_2_rad,
            ):
                if index >= len(angles):
                    residuals.append(math.inf)
                    continue
                angle = angles[index]
                margin = min(angle, math.pi - angle)
                residuals.append(self.minimum_margin_rad - margin)
        return tuple(residuals)


@dataclass(frozen=True)
class MonotonicFlexion:
    """Require one positive output-flexion magnitude to increase with input."""

    joint: JointName
    minimum_motion_ratio: float = 0.0
    name: str = field(init=False)

    def __post_init__(self) -> None:
        if self.joint not in ("q2", "q3"):
            raise ValueError("joint must be 'q2' or 'q3'")
        _require_finite("minimum_motion_ratio", self.minimum_motion_ratio)
        object.__setattr__(self, "name", f"monotonic_flexion_{self.joint}")

    def residual(self, context: LinkageEvaluation) -> tuple[float, ...]:
        ratios = _available_motion_ratios(context, self.joint)
        residuals = tuple(self.minimum_motion_ratio - ratio for ratio in ratios)
        missing = context.requested_sample_count - len(residuals)
        return residuals + (math.inf,) * max(0, missing)


@dataclass(frozen=True)
class MaximumMotionRatio:
    """Bound one positive output/input flexion motion ratio from above."""

    joint: JointName
    maximum_motion_ratio: float
    name: str = field(init=False)

    def __post_init__(self) -> None:
        if self.joint not in ("q2", "q3"):
            raise ValueError("joint must be 'q2' or 'q3'")
        _require_nonnegative_finite(
            "maximum_motion_ratio",
            self.maximum_motion_ratio,
        )
        object.__setattr__(self, "name", f"maximum_motion_ratio_{self.joint}")

    def residual(self, context: LinkageEvaluation) -> tuple[float, ...]:
        ratios = _available_motion_ratios(context, self.joint)
        residuals = tuple(
            ratio - self.maximum_motion_ratio
            for ratio in ratios
        )
        missing = context.requested_sample_count - len(residuals)
        return residuals + (math.inf,) * max(0, missing)


@dataclass(frozen=True)
class MinimumEndpointFlexion:
    """Require one output joint to reach a minimum flexion at the sweep endpoint."""

    joint: JointName
    minimum_flexion_rad: float
    name: str = field(init=False)

    def __post_init__(self) -> None:
        if self.joint not in ("q2", "q3"):
            raise ValueError("joint must be 'q2' or 'q3'")
        _require_nonnegative_finite("minimum_flexion_rad", self.minimum_flexion_rad)
        object.__setattr__(self, "name", f"minimum_endpoint_flexion_{self.joint}")

    def residual(self, context: LinkageEvaluation) -> tuple[float, ...]:
        values = _joint_flexion(context, self.joint)
        if not context.complete_workspace or not values:
            return (math.inf,)
        return (self.minimum_flexion_rad - values[-1],)


def _available_motion_ratios(
    context: LinkageEvaluation,
    joint: JointName,
) -> tuple[float, ...]:
    return (
        context.dq2_flexion_dq1_flexion
        if joint == "q2"
        else context.dq3_flexion_dq1_flexion
    )


def _joint_flexion(context: LinkageEvaluation, joint: JointName) -> tuple[float, ...]:
    return context.q2_flexion_rad if joint == "q2" else context.q3_flexion_rad


def _require_finite(name: str, value: float) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be finite")


def _require_nonnegative_finite(name: str, value: float) -> None:
    _require_finite(name, value)
    if value < 0.0:
        raise ValueError(f"{name} must be non-negative")


__all__ = [
    "FullWorkspaceKinematicValidity",
    "MaximumMotionRatio",
    "MinimumAnchorDorsalOffset",
    "MinimumEndpointFlexion",
    "MinimumTransmissionMargin",
    "MonotonicFlexion",
]
