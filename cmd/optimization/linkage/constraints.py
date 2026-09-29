"""Composable hard feasibility constraints for linkage core-workspace studies."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from numbers import Real
from typing import TYPE_CHECKING, Literal

from cmd.morphology.linkage_motion import LinkageWorkspaceCriteria

if TYPE_CHECKING:
    from .stage0 import LinkageStage0Evaluation


JointName = Literal["q2", "q3"]


@dataclass(frozen=True)
class CoreWorkspaceKinematicValidity:
    """Require closure for every requested sample through the core endpoint."""

    core_input_workspace_rad: float
    name: str = "core_workspace_kinematic_validity"

    def __post_init__(self) -> None:
        _require_positive_finite(
            "core_input_workspace_rad",
            self.core_input_workspace_rad,
        )

    def residual(self, context: LinkageStage0Evaluation) -> tuple[float, ...]:
        from .stage0 import core_sample_indices

        indices = core_sample_indices(context.motion, self.core_input_workspace_rad)
        solved = sum(index < context.motion.solved_sample_count for index in indices)
        return (1.0 - solved / len(indices),)


@dataclass(frozen=True)
class MinimumUsableInputWorkspace:
    """Require the mechanically usable contiguous q1 interval to be long enough."""

    minimum_required_input_workspace_rad: float
    name: str = "minimum_usable_input_workspace"

    def __post_init__(self) -> None:
        _require_nonnegative_finite(
            "minimum_required_input_workspace_rad",
            self.minimum_required_input_workspace_rad,
        )
    def residual(self, context: LinkageStage0Evaluation) -> tuple[float, ...]:
        return (
            self.minimum_required_input_workspace_rad
            - context.workspace.usable_input_workspace_rad,
        )


@dataclass(frozen=True)
class MinimumAnchorDorsalOffset:
    """Require A' and B' to be at least ``minimum_offset_mm`` above B."""

    minimum_offset_mm: float
    name: str = "minimum_anchor_dorsal_offset"

    def __post_init__(self) -> None:
        _require_nonnegative_finite("minimum_offset_mm", self.minimum_offset_mm)

    def residual(self, context: LinkageStage0Evaluation) -> tuple[float, ...]:
        parameters = context.motion.parameters
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
    """Require every available core-sample transmission margin to meet a limit."""

    criteria: LinkageWorkspaceCriteria
    core_input_workspace_rad: float
    name: str = "minimum_transmission_margin"

    def __post_init__(self) -> None:
        if not isinstance(self.criteria, LinkageWorkspaceCriteria):
            raise TypeError("criteria must be LinkageWorkspaceCriteria")
        _require_positive_finite(
            "core_input_workspace_rad",
            self.core_input_workspace_rad,
        )

    def residual(self, context: LinkageStage0Evaluation) -> tuple[float, ...]:
        from .stage0 import core_sample_indices

        motion = context.motion
        indices = core_sample_indices(motion, self.core_input_workspace_rad)
        residuals: list[float] = []
        for index in indices:
            for angles in (
                motion.transmission_angle_loop_1_rad,
                motion.transmission_angle_loop_2_rad,
            ):
                if index >= len(angles):
                    residuals.append(0.0)
                    continue
                angle = angles[index]
                margin = min(angle, math.pi - angle)
                residuals.append(self.criteria.minimum_transmission_margin_rad - margin)
        return tuple(residuals)


@dataclass(frozen=True)
class MonotonicFlexion:
    """Require a joint's flexion to increase across the core workspace."""

    joint: JointName
    criteria: LinkageWorkspaceCriteria
    core_input_workspace_rad: float
    name: str = field(init=False)

    def __post_init__(self) -> None:
        _validate_joint(self.joint)
        if not isinstance(self.criteria, LinkageWorkspaceCriteria):
            raise TypeError("criteria must be LinkageWorkspaceCriteria")
        _require_positive_finite(
            "core_input_workspace_rad",
            self.core_input_workspace_rad,
        )
        object.__setattr__(self, "name", f"monotonic_flexion_{self.joint}")

    def residual(self, context: LinkageStage0Evaluation) -> tuple[float, ...]:
        from .stage0 import core_motion_ratios, core_sample_indices

        indices = core_sample_indices(context.motion, self.core_input_workspace_rad)
        ratios = core_motion_ratios(context.motion, self.core_input_workspace_rad, self.joint)
        minimum = getattr(self.criteria, f"{self.joint}_minimum_motion_ratio")
        residuals = tuple(minimum - ratio for ratio in ratios)
        return _pad_missing(residuals, len(indices) - 1)


@dataclass(frozen=True)
class MaximumMotionRatio:
    """Bound a joint's positive output/input flexion ratio in the core."""

    joint: JointName
    criteria: LinkageWorkspaceCriteria
    core_input_workspace_rad: float
    name: str = field(init=False)

    def __post_init__(self) -> None:
        _validate_joint(self.joint)
        if not isinstance(self.criteria, LinkageWorkspaceCriteria):
            raise TypeError("criteria must be LinkageWorkspaceCriteria")
        _require_positive_finite(
            "core_input_workspace_rad",
            self.core_input_workspace_rad,
        )
        object.__setattr__(self, "name", f"maximum_motion_ratio_{self.joint}")

    def residual(self, context: LinkageStage0Evaluation) -> tuple[float, ...]:
        from .stage0 import core_motion_ratios, core_sample_indices

        indices = core_sample_indices(context.motion, self.core_input_workspace_rad)
        ratios = core_motion_ratios(context.motion, self.core_input_workspace_rad, self.joint)
        maximum = getattr(self.criteria, f"{self.joint}_maximum_motion_ratio")
        residuals = tuple(ratio - maximum for ratio in ratios)
        return _pad_missing(residuals, len(indices) - 1)


@dataclass(frozen=True)
class MinimumFlexionAtInput:
    """Require one joint to reach its minimum at an exact input-flexion sample."""

    joint: JointName
    input_flexion_rad: float
    minimum_flexion_rad: float
    name: str = field(init=False)

    def __post_init__(self) -> None:
        _validate_joint(self.joint)
        _require_positive_finite("input_flexion_rad", self.input_flexion_rad)
        _require_nonnegative_finite("minimum_flexion_rad", self.minimum_flexion_rad)
        object.__setattr__(self, "name", f"minimum_flexion_at_input_{self.joint}")

    def residual(self, context: LinkageStage0Evaluation) -> tuple[float, ...]:
        from .stage0 import core_sample_indices

        indices = core_sample_indices(context.motion, self.input_flexion_rad)
        endpoint_index = indices[-1]
        values = (
            context.motion.q2_flexion_rad
            if self.joint == "q2"
            else context.motion.q3_flexion_rad
        )
        if endpoint_index >= len(values):
            return (0.0,)
        return (self.minimum_flexion_rad - values[endpoint_index],)


def _pad_missing(
    residuals: tuple[float, ...],
    requested_count: int,
) -> tuple[float, ...]:
    return residuals + (0.0,) * max(0, requested_count - len(residuals))


def _validate_joint(joint: JointName) -> None:
    if joint not in ("q2", "q3"):
        raise ValueError("joint must be 'q2' or 'q3'")


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


def _require_positive_finite(name: str, value: float) -> None:
    _require_finite(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")


__all__ = [
    "CoreWorkspaceKinematicValidity",
    "MaximumMotionRatio",
    "MinimumAnchorDorsalOffset",
    "MinimumFlexionAtInput",
    "MinimumTransmissionMargin",
    "MinimumUsableInputWorkspace",
    "MonotonicFlexion",
]
