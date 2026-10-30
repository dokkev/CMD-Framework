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

from core.optimization.linkage.config import LinkageConfig


VARIABLE_NAMES = ("a_x_mm", "ac_length_mm", "app_x_mm", "bpp_from_a_mm", "distal_drop_mm", "proximal_drop_mm",
                  "servo_tilt_deg")
"""``|OA|``, ``|AC|``, the proximal pad's end ``A''_x``, the distal pad's
joint end ``B''`` from ``A`` along ``A->C``, how far below ``A-C`` the distal
pad's top face lies and below ``O-A`` the proximal pad's, and the distal
servo's dorsal tilt from ``-X`` (degrees; the goal's PIP servo tilt, fixed or
searched). The pads are plain blocks, each on its own segment and off the
joint's housing (:data:`DISTAL_DROP_MIN_MM`'s radius): the proximal pad ends
at ``A_x`` less it, the distal pad starts at it past ``A``."""
DISTAL_DROP_MIN_MM = 0.0
"""Each pad's top face may be searched below its segment's line from on the
line down to the joint's palmar face (the servo's radius, 9.5 mm for the
XM335; the idealized 2R's: its pin housing's): the fingertip no deeper than
the proximal segment can be (deeper, the optimum at little flexion is a hook)."""


def pad_reach_mm(goal: LinkageConfig, radius_mm: float) -> tuple[float, float]:
    """How far each pad may reach toward and past the joint, from ``A``: the
    proximal pad's end at most ``A_x`` plus this, the distal pad's start at
    least minus this (``bpp_from_a_mm``): the joint's housing (its radius)
    plus the pad's overhang past it (``goal.pads.*.overhang_max_mm``), as the
    linkage's past its own body. Nearer the joint, only the pads' actual
    clearance of the housing and of each other limits them."""

    return (radius_mm + (goal.proximal_overhang_max_mm or 0.0), radius_mm + (goal.distal_overhang_max_mm or 0.0))


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


def servo_rear(tilt_deg: float) -> tuple[float, float]:
    """The direction the distal servo's body lies back in from its shaft on
    ``A``: ``-X`` turned dorsally by ``tilt_deg``."""

    tilt = math.radians(tilt_deg)
    return (-math.cos(tilt), math.sin(tilt))


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
            "bpp_from_a_mm": (-pad_reach_mm(goal, radius)[1], finger_high - goal.minimum_distal_support_mm),
            "distal_drop_mm": (DISTAL_DROP_MIN_MM, radius),
            "proximal_drop_mm": (DISTAL_DROP_MIN_MM, radius),
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


__all__ = ["DISTAL_DROP_MIN_MM", "VARIABLE_NAMES", "RevoluteConfig", "joint_radius", "pad_reach_mm", "servo_rear"]
