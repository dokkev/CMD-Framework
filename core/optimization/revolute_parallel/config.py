"""The serial 2R driven through a parallel four-bar: the linkage study's
``goal`` / ``solver`` file, read as is (as ``core.optimization.revolute``'s).

The joint ``A`` holds only its pin, as the idealized 2R's (the pads hang its
housing's radius below their segment lines); the servo sits on ``A'``,
searched where the linkage's is (``solver.search_bounds.ap_from_a_*``),
tilted as the goal's PIP servo, and turns the joint through a parallelogram.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

from core._validation import require_finite

from core.optimization.linkage.config import LinkageConfig
from core.optimization.revolute.config import VARIABLE_NAMES as REVOLUTE_NAMES, RevoluteConfig


VARIABLE_NAMES = (*REVOLUTE_NAMES, "ap_from_a_x_mm", "ap_from_a_z_mm", "crank_mm", "crank_angle_deg")
"""The 2R's (``|OA|``, ``|AC|``, the pads' ends, the distal pad's drop), then
the servo shaft ``A'`` from ``A``, the cranks' length ``|A'Q| = |AR|`` and
their direction at extension (degrees from ``+X``, toward ``+Z``)."""
CRANK_RANGE_MM = (5.0, 30.0)
"""Cranks from about a pin's size to a third of the finger."""


@dataclass(frozen=True)
class ParallelConfig:
    base: RevoluteConfig
    """The idealized 2R's config (pin only at ``A``) the pads and joint follow."""
    bounds: Mapping[str, tuple[float, float]]

    def __post_init__(self) -> None:
        if set(self.bounds) != set(VARIABLE_NAMES):
            raise ValueError(f"bounds must cover exactly {VARIABLE_NAMES}")
        bounds = {}
        for name in VARIABLE_NAMES:
            low, high = (require_finite(name, value) for value in self.bounds[name])
            if low > high:
                raise ValueError(f"{name} lower bound exceeds its upper bound")
            bounds[name] = (low, high)
        object.__setattr__(self, "bounds", MappingProxyType(bounds))
        if self.goal.pip_servo_tilt_rad is None:
            raise ValueError("the parallel 2R takes the PIP servo's tilt as an input (one value)")

    @classmethod
    def from_goal(cls, goal: LinkageConfig) -> "ParallelConfig":
        base = RevoluteConfig.from_goal(goal, joint_servo=False)
        relative = goal.relative_bounds or {}
        bounds = {**base.bounds,
                  "ap_from_a_x_mm": tuple(relative.get("ap_from_a_x_mm", (-20.0, 30.0))),
                  "ap_from_a_z_mm": tuple(relative.get("ap_from_a_z_mm", (0.0, 25.0))),
                  "crank_mm": CRANK_RANGE_MM, "crank_angle_deg": (-180.0, 180.0)}
        return cls(base, bounds)

    @property
    def goal(self) -> LinkageConfig:
        return self.base.goal

    @property
    def tilt_rad(self) -> float:
        return float(self.goal.pip_servo_tilt_rad)

    @property
    def joint_radius_mm(self) -> float:
        return self.base.joint_radius_mm

    @property
    def search_bounds(self) -> Mapping[str, tuple[float, float]]:
        return MappingProxyType({**self.base.search_bounds,
                                 **{name: self.bounds[name] for name in VARIABLE_NAMES[len(REVOLUTE_NAMES):]}})

    @property
    def lower(self) -> tuple[float, ...]:
        return tuple(self.search_bounds[name][0] for name in VARIABLE_NAMES)

    @property
    def upper(self) -> tuple[float, ...]:
        return tuple(self.search_bounds[name][1] for name in VARIABLE_NAMES)

    # The goal's settings the study reads, as the linkage config has them.
    @property
    def resolution_mm(self) -> float:
        return self.goal.resolution_mm

    @property
    def objectives(self) -> tuple[str, ...] | None:
        return self.goal.objectives

    @property
    def seeds(self) -> tuple[int, ...]:
        return self.goal.seeds

    @property
    def warm_start(self) -> tuple[str, ...]:
        return self.goal.warm_start

    @property
    def verify_step_rad(self) -> float:
        return self.goal.verify_step_rad

    @property
    def hypervolume_reference(self) -> Mapping[str, float]:
        return self.goal.hypervolume_reference


__all__ = ["CRANK_RANGE_MM", "VARIABLE_NAMES", "ParallelConfig"]
