"""The serial 2R study: the linkage study's ``goal`` / ``solver`` file, read as is.

The goal (motion, pads, link lengths, hardware, collision) and the solver's
objectives, reference point, sampling, resolution and optimizer are shared
with the linkage, so the two fronts compare on the same requirements. The
2R's design variables are all bounded by the goal; ``solver.search_bounds``
(the linkage's free variables) is read and not used.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

from core._validation import require_finite

from core.morphology.mechanism.revolute.solids import VARIABLE_NAMES

from core.optimization.linkage.config import LinkageConfig


DISTAL_DROP_MIN_MM = 0.0
"""Before 2026-11-03 each pad's top face was searched below its segment's
line from on it down to the joint's palmar face; now each pad's contact
interface space starts on the line tangent to ``A``'s pin housing
(:func:`interface_radius`; ``evaluate.interfaced``), the drops' bounds that
radius alone."""


def pad_reach_mm(goal: LinkageConfig, radius_mm: float) -> tuple[float, float]:
    """How far each pad may reach toward and past the joint, from ``A``: the
    proximal pad's end at most ``A_x`` plus this, the distal pad's start at
    least minus this (``bpp_from_a_mm``): the joint's housing (its radius)
    plus the pad's overhang past it (``goal.pads.*.overhang_max_mm``), as the
    linkage's past its own body; no overhang limit (``None``): no reach limit. Nearer the joint, only the pads' actual
    clearance of the housing and of each other limits them."""

    return tuple(math.inf if overhang is None else radius_mm + overhang
                 for overhang in (goal.proximal_overhang_max_mm, goal.distal_overhang_max_mm))


def joint_radius(goal: LinkageConfig, joint_servo: bool, joint_pulley: bool = False) -> float:
    """The joint's envelope at ``A``: the servo's rounded front (servo on the
    joint), else the pin's housing, or a pulley on it when larger."""

    packaging = goal.packaging
    if joint_servo:
        return packaging.pip_servo.radius_mm
    pin = packaging.pivot_radius_mm("a")
    if not joint_pulley:
        return pin
    if goal.pulley_radius_mm is None:
        raise ValueError("a pulley on the joint needs goal.hardware.pulley_radius_mm")
    return max(pin, goal.pulley_radius_mm)


def interface_radius(goal: LinkageConfig, joint_servo: bool, joint_pulley: bool = False) -> float:
    """The pads' line below the segments (:attr:`RevoluteConfig.interface_radius_mm`):
    on them for the 2R-DD (the servo's shaft is the joint, no bearing
    housing to clear; the servo itself a collision), else tangent to ``A``'s
    pin housing (or the pulley on it)."""

    return 0.0 if joint_servo else joint_radius(goal, joint_servo, joint_pulley)


@dataclass(frozen=True)
class RevoluteConfig:
    goal: LinkageConfig
    """The goal and solver as the linkage reads them."""
    bounds: Mapping[str, tuple[float, float]]
    joint_servo: bool = True
    """The distal servo on ``A`` (packaged). ``False``: the idealized 2R, driven
    from elsewhere, only ``A``'s pin housing (``goal.collision.param``) at the
    joint (and no tilt)."""
    joint_pulley: bool = False
    """A pulley on ``A`` (``goal.hardware.pulley_radius_mm``), as a belt or a
    tendon drives the joint through (with ``joint_servo`` off): the joint's
    envelope is the larger of it and the pin's housing."""

    @property
    def joint_radius_mm(self) -> float:
        """What the joint holds at ``A``: the servo's rounded front, or the pin
        housing (or the pulley, when larger)."""

        return joint_radius(self.goal, self.joint_servo, self.joint_pulley)

    @property
    def interface_radius_mm(self) -> float:
        """Where the pads' contact interface spaces start, below the segments'
        lines (:func:`interface_radius`): on them for the 2R-DD, its servo a
        collision; else tangent to ``A``'s pin housing, as the linkage's pads
        to their pins (2026-11-03). With a pulley on the joint, its envelope."""

        return interface_radius(self.goal, self.joint_servo, self.joint_pulley)

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

    @classmethod
    def from_goal(cls, goal: LinkageConfig, joint_servo: bool = True, joint_pulley: bool = False) -> "RevoluteConfig":
        """The 2R's search box from the goal: ``|OA|`` and ``|AC|`` as the
        linkage's ``A_x`` and ``BC``, the pads within the finger and off the
        joint, the tilt the goal's PIP servo's."""

        finger_high = goal.extended_length_bounds_mm[1]
        start = goal.packaging.proximal_pad_start_mm
        radius = joint_radius(goal, joint_servo, joint_pulley)
        if not joint_servo:
            tilt = (0.0, 0.0)
        elif goal.pip_servo_tilt_rad is not None:
            tilt = (round(math.degrees(goal.pip_servo_tilt_rad), 9),) * 2
        else:
            tilt = tuple(goal.bounds["servo_tilt_deg"])
        bounds = {
            "a_x_mm": goal.within_share(goal.bounds["a_x_mm"], goal.proximal_length_ratio_bounds),
            "ac_length_mm": goal.within_share(goal.bounds["bc_length_mm"], goal.distal_length_ratio_bounds),
            "app_x_mm": (start + goal.minimum_proximal_support_mm, finger_high),
            "bpp_from_a_mm": (-min(pad_reach_mm(goal, radius)[1], finger_high),
                              finger_high - goal.minimum_distal_support_mm),
            # Tangent to the pin's housing, not searched.
            "distal_drop_mm": (interface_radius(goal, joint_servo, joint_pulley),) * 2,
            "proximal_drop_mm": (interface_radius(goal, joint_servo, joint_pulley),) * 2,
            "servo_tilt_deg": tilt,
        }
        return cls(goal, bounds, joint_servo, joint_pulley)

    @property
    def search_bounds(self) -> Mapping[str, tuple[float, float]]:
        """``bounds`` narrowed by ``|OA| + |AC|`` within the finger's length
        and each pad's room for its least length."""

        bounds = dict(self.bounds)

        def narrow(name: str, low: float = -math.inf, high: float = math.inf) -> None:
            old_low, old_high = bounds[name]
            new_low, new_high = max(old_low, low), min(old_high, high)
            bounds[name] = (min(new_low, old_high), max(new_high, old_low))

        length_low, length_high = self.goal.extended_length_bounds_mm
        for _ in range(3):
            (a_lo, a_hi), (c_lo, c_hi) = bounds["a_x_mm"], bounds["ac_length_mm"]
            narrow("a_x_mm", length_low - c_hi, length_high - c_lo)
            narrow("ac_length_mm", length_low - a_hi, length_high - a_lo)
            narrow("bpp_from_a_mm", high=bounds["ac_length_mm"][1] - self.goal.minimum_distal_support_mm)
            # The proximal pad ends past the joint's housing by its overhang at most.
            narrow("app_x_mm", high=bounds["a_x_mm"][1] + pad_reach_mm(self.goal, self.joint_radius_mm)[0])
        return MappingProxyType(bounds)

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


__all__ = ["DISTAL_DROP_MIN_MM", "VARIABLE_NAMES", "RevoluteConfig", "interface_radius", "joint_radius", "pad_reach_mm"]
