"""The rolling contact joint (RCJ) study: the linkage study's ``goal`` /
``solver`` file, read as is (as ``core.optimization.revolute``'s).

The goal (motion, pads, link lengths, hardware, collision) and the solver's
objectives, reference point, sampling, resolution and optimizer are shared
with the linkage and the 2R, so the fronts compare on the same requirements.
``goal.link.proximal_length_mm`` bounds ``|OA|`` (``A`` the proximal roller's
centre, on the servo shaft) and ``goal.link.distal_length_mm`` bounds ``BC``;
``solver.search_bounds`` (the linkage's free variables) is read and not used.
This is not the retention study's RCJ (``core.optimization.rolling_contact_joint``).
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

from core._validation import require_finite

from core.morphology.mechanism.rcj.solids import VARIABLE_NAMES

from core.optimization.linkage.config import LinkageConfig
from core.optimization.revolute.config import joint_radius


ROLLER_RADIUS_MAX_MM = 20.0
"""The largest roller searched; the least is the servo's radius (the roller
holds the servo's shaft and rounded front), or the idealized RCJ's pin housing."""
DISTAL_PAD_REACH_MM = 20.0
"""How far behind ``B`` the distal pad's joint end may be searched, without
``goal.pads.distal.overhang_max_mm`` (with it: past the distal roller by
at most that)."""
@dataclass(frozen=True)
class RcjConfig:
    goal: LinkageConfig
    """The goal and solver as the linkage reads them."""
    bounds: Mapping[str, tuple[float, float]]
    joint_servo: bool = True
    """The servo on ``A`` (packaged). ``False``: the idealized RCJ, driven from
    elsewhere, its rollers at least ``A``'s pin housing (and no tilt), as the
    idealized 2R's."""

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
    def from_goal(cls, goal: LinkageConfig, joint_servo: bool = True) -> "RcjConfig":
        """The RCJ's search box from the goal: ``|OA|`` and ``BC`` as the
        linkage's ``A_x`` and ``BC``; the rollers from the largest of what
        they hold at ``A`` (the servo's radius, or with ``joint_servo`` off
        the pin's housing), ``B``'s pin housing and half the two pins'
        spacing (``|AB| = 2r``), to :data:`ROLLER_RADIUS_MAX_MM`; the pads
        within the finger."""

        finger_high = goal.extended_length_bounds_mm[1]
        # The rollers hold the pins at A and B: each at least its pin's housing,
        # and |AB| = 2r at least the two pins' spacing (their housings touching).
        radius = max(joint_radius(goal, joint_servo), goal.packaging.pivot_radius_mm("b"),
                     goal.packaging.pin_spacing_mm / 2.0)
        # The servo's tilt, as the 2R's: the goal's PIP servo's, fixed or searched (none without it).
        if not joint_servo:
            tilt = (0.0, 0.0)
        elif goal.pip_servo_tilt_rad is not None:
            tilt = (round(math.degrees(goal.pip_servo_tilt_rad), 9),) * 2
        else:
            tilt = tuple(goal.bounds["servo_tilt_deg"])
        start = goal.packaging.proximal_pad_start_mm
        bounds = {
            # (The segment ratios bound |OA''| and |B''C|, set by the pads' ends: checked, not narrowed by.)
            "a_x_mm": tuple(goal.bounds["a_x_mm"]),
            "roller_radius_mm": (radius, max(radius, ROLLER_RADIUS_MAX_MM)),
            "bc_length_mm": tuple(goal.bounds["bc_length_mm"]),
            "app_x_mm": (start + goal.minimum_proximal_support_mm, finger_high),
            "bpp_from_b_mm": (-distal_reach(goal, ROLLER_RADIUS_MAX_MM), finger_high - goal.minimum_distal_support_mm),
            "distal_drop_mm": (0.0, max(radius, ROLLER_RADIUS_MAX_MM)),
            "proximal_drop_mm": (0.0, max(radius, ROLLER_RADIUS_MAX_MM)),
            "servo_tilt_deg": tilt,
        }
        return cls(goal, bounds, joint_servo)

    @property
    def search_bounds(self) -> Mapping[str, tuple[float, float]]:
        return self.bounds

    @property
    def lower(self) -> tuple[float, ...]:
        return tuple(self.bounds[name][0] for name in VARIABLE_NAMES)

    @property
    def upper(self) -> tuple[float, ...]:
        return tuple(self.bounds[name][1] for name in VARIABLE_NAMES)

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


def distal_reach(goal: LinkageConfig, roller_radius_mm: float) -> float:
    """How far behind ``B`` the distal pad may reach: past the roller by
    ``goal.pads.distal.overhang_max_mm``, or :data:`DISTAL_PAD_REACH_MM`."""

    overhang = goal.distal_overhang_max_mm
    return DISTAL_PAD_REACH_MM if overhang is None else roller_radius_mm + overhang


def proximal_reach(goal: LinkageConfig, roller_radius_mm: float) -> float:
    """How far past ``A`` the proximal pad may reach: the roller's front, and
    ``goal.pads.proximal.overhang_max_mm`` past it."""

    return roller_radius_mm + (goal.proximal_overhang_max_mm or 0.0)


__all__ = ["DISTAL_PAD_REACH_MM", "ROLLER_RADIUS_MAX_MM", "VARIABLE_NAMES", "RcjConfig", "distal_reach",
           "proximal_reach"]
