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

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

from core._validation import require_finite

from core.optimization.linkage.config import LinkageConfig


VARIABLE_NAMES = ("a_x_mm", "roller_radius_mm", "bc_length_mm", "app_x_mm", "bpp_from_b_mm", "distal_drop_mm",
                  "proximal_drop_mm")
"""``|OA|``, the rollers' radius ``r`` (``|AB| = 2r``), ``BC``, the proximal
pad's end ``A''_x``, the distal pad's joint end ``B''`` from ``B`` along
``B->C`` (negative: behind ``B``), and how far below ``B-C`` and ``O-A``
the distal and the proximal pad's top faces lie (each at most ``r``, the
joint's radius, as the 2R's; each pad clears its own roller). The pads are
plain blocks."""
ROLLER_RADIUS_MAX_MM = 20.0
"""The largest roller searched; the least is the servo's radius (the roller
holds the servo's shaft and rounded front)."""
DISTAL_PAD_REACH_MM = 20.0
"""How far behind ``B`` the distal pad's joint end may be searched, without
``goal.pads.distal.overhang_max_mm`` (with it: past the distal roller by
at most that)."""
SERVO_REAR = (-1.0, 0.0)
"""The servo on ``A`` lies back along ``-X``, untilted, as the 2R's: within
the roller's depth (``r`` is at least its radius), so no thicker than the
rollers; standing dorsal it would only add height, and it never meets a pad
either way (the pads hang a roller radius below ``O-A``)."""


@dataclass(frozen=True)
class RcjConfig:
    goal: LinkageConfig
    """The goal and solver as the linkage reads them."""
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

    @classmethod
    def from_goal(cls, goal: LinkageConfig) -> "RcjConfig":
        """The RCJ's search box from the goal: ``|OA|`` and ``BC`` as the
        linkage's ``A_x`` and ``BC``, the rollers from the servo's radius,
        the pads within the finger."""

        finger_high = goal.extended_length_bounds_mm[1]
        radius = goal.packaging.pip_servo.radius_mm
        start = goal.packaging.proximal_pad_start_mm
        bounds = {
            "a_x_mm": goal.within_share(goal.bounds["a_x_mm"], goal.proximal_length_ratio_bounds),
            "roller_radius_mm": (radius, max(radius, ROLLER_RADIUS_MAX_MM)),
            "bc_length_mm": goal.within_share(goal.bounds["bc_length_mm"], goal.distal_length_ratio_bounds),
            "app_x_mm": (start + goal.minimum_proximal_support_mm, finger_high),
            "bpp_from_b_mm": (-distal_reach(goal, ROLLER_RADIUS_MAX_MM), finger_high - goal.minimum_distal_support_mm),
            "distal_drop_mm": (0.0, max(radius, ROLLER_RADIUS_MAX_MM)),
            "proximal_drop_mm": (0.0, max(radius, ROLLER_RADIUS_MAX_MM)),
        }
        return cls(goal, bounds)

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


def finger_length_mm(a_x_mm: float, roller_radius_mm: float, bc_length_mm: float) -> float:
    """``|OC|`` extended: ``O-A``, the two radii to ``B``, then ``BC``."""

    return a_x_mm + 2.0 * roller_radius_mm + bc_length_mm


__all__ = ["DISTAL_PAD_REACH_MM", "ROLLER_RADIUS_MAX_MM", "SERVO_REAR", "VARIABLE_NAMES", "RcjConfig",
           "distal_reach", "finger_length_mm", "proximal_reach"]
