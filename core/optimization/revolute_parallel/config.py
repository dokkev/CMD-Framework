"""The serial 2R driven from a servo on ``A'`` through a belt: the linkage
study's ``goal`` / ``solver`` file, read as is (as ``core.optimization.revolute``'s).

The joint ``A`` holds its pin (the pads hang and keep off its housing, as
the idealized 2R's); the servo sits on ``A'``, searched where the linkage's
is (``solver.search_bounds.ap_from_a_*``), its tilt searched as the
linkage's (``goal.hardware.pip_servo.tilt_angle_deg``). The belt and its two
pulleys (``goal.hardware.pulley_radius_mm``, on ``A`` and on the servo's horn
at ``A'``) run in the outer layer, beside the servo, as the linkage's links:
no collision with the pads, which lie in the central layer; seen from the
side they still stand in front of a face (``core.morphology.contact.loss``).
The tendon 2R's pulleys are the same size; its routing band lies in the
pads' layer instead.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

from core._validation import require_finite

from core.morphology.revolute.solids import VARIABLE_NAMES as REVOLUTE_NAMES
from core.morphology.revolute_parallel.solids import VARIABLE_NAMES

from core.optimization.linkage.config import LinkageConfig
from core.optimization.revolute.config import RevoluteConfig


@dataclass(frozen=True)
class ParallelConfig:
    base: RevoluteConfig
    """The idealized 2R's config (the pads and joint follow it), its tilt
    bounds the servo's on ``A'``."""
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
        if self.goal.pulley_radius_mm is None:
            raise ValueError("the belt-driven 2R needs goal.hardware.pulley_radius_mm")

    @classmethod
    def from_goal(cls, goal: LinkageConfig) -> "ParallelConfig":
        joint = RevoluteConfig.from_goal(goal, joint_servo=False)
        if goal.pip_servo_tilt_rad is not None:
            tilt = (round(math.degrees(goal.pip_servo_tilt_rad), 9),) * 2
        else:
            tilt = tuple(goal.bounds["servo_tilt_deg"])
        base = RevoluteConfig(goal, {**joint.bounds, "servo_tilt_deg": tilt}, False)
        relative = goal.relative_bounds or {}
        bounds = {**base.bounds,
                  "ap_from_a_x_mm": tuple(relative.get("ap_from_a_x_mm", (-20.0, 30.0))),
                  "ap_from_a_z_mm": tuple(relative.get("ap_from_a_z_mm", (0.0, 25.0)))}
        return cls(base, bounds)

    @property
    def goal(self) -> LinkageConfig:
        return self.base.goal

    @property
    def joint_radius_mm(self) -> float:
        return self.base.joint_radius_mm

    @property
    def pulley_radius_mm(self) -> float:
        return float(self.goal.pulley_radius_mm)

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


__all__ = ["VARIABLE_NAMES", "ParallelConfig"]
