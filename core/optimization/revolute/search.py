"""The coordinates the serial 2R is searched in, each in a range the earlier
ones set (as ``core.optimization.linkage.search``):

1. ``|OC|``;
2. ``|AC|`` a share of ``[its least, |OC| - the least |OA|]``, the least
   ``|OA|`` keeping the distal servo off the MCP servo and within the goal;
3. ``A''_x`` a share of ``[the pad's start + its least length, A_x + the
   servo's radius]``;
4. ``B''`` from ``A`` a share of ``[-DISTAL_PAD_REACH_MM, |AC| - its least length]``;

Lengths are rounded to the prototyping resolution; empty ranges describe no design.
"""

from __future__ import annotations

import functools
from collections.abc import Sequence

import numpy as np

from core.morphology.linkage.packaging import Packaging, servo_floor_x

from core.optimization.revolute.config import DISTAL_PAD_REACH_MM, VARIABLE_NAMES, RevoluteConfig


SEARCH_NAMES = ("finger_length_mm", "ac_share", "app_share", "bpp_share")
SHARE_STEP = 0.001


@functools.lru_cache(maxsize=64)
def least_a_x(packaging: Packaging, angles: tuple[float, ...]) -> float:
    """The least ``|OA|`` keeping the distal servo (on ``A``, laid back along
    ``-X``) clear of the MCP servo (``packaging.servo_floor_x``, as the linkage's)."""

    return servo_floor_x(packaging, 0.0, (-1.0, 0.0), angles)


def _limits(config: RevoluteConfig) -> dict[str, tuple[float, float]]:
    goal = config.goal
    a_low, a_high = goal.bounds["a_x_mm"]
    floor = least_a_x(goal.packaging, tuple(goal.mcp_servo_angles_rad)) - 0.5  # sampled: stay below it
    return {"a": (max(a_low, floor), a_high), "ac": config.bounds["ac_length_mm"]}


def search_box(config: RevoluteConfig) -> tuple[tuple[float, ...], tuple[float, ...]]:
    box = {"finger_length_mm": config.goal.extended_length_bounds_mm, "ac_share": (0.0, 1.0),
           "app_share": (0.0, 1.0), "bpp_share": (0.0, 1.0)}
    return tuple(box[name][0] for name in SEARCH_NAMES), tuple(box[name][1] for name in SEARCH_NAMES)


def search_steps(config: RevoluteConfig) -> tuple[float, ...]:
    return tuple(SHARE_STEP if name.endswith("_share") else config.resolution_mm for name in SEARCH_NAMES)


def _ranges(config: RevoluteConfig, length: float):
    goal = config.goal
    limits = _limits(config)
    ac_range = (max(limits["ac"][0], length - limits["a"][1]), min(limits["ac"][1], length - limits["a"][0]))
    start = goal.packaging.proximal_pad_start_mm
    radius = goal.packaging.pip_servo.radius_mm

    def pads(ac: float) -> tuple[tuple[float, float], tuple[float, float]]:
        a_x = length - ac
        return ((start + goal.minimum_proximal_support_mm, a_x + radius),
                (-DISTAL_PAD_REACH_MM, ac - goal.minimum_distal_support_mm))

    return ac_range, pads


def to_design(coordinates: Sequence[float], config: RevoluteConfig) -> tuple[float, ...] | None:
    v = dict(zip(SEARCH_NAMES, (float(c) for c in coordinates)))
    resolution = config.resolution_mm
    grid = lambda value: round(round(value / resolution) * resolution, 9)  # noqa: E731
    length = v["finger_length_mm"]
    ac_range, pads = _ranges(config, length)
    if ac_range[1] < ac_range[0]:
        return None
    ac = grid(ac_range[0] + v["ac_share"] * (ac_range[1] - ac_range[0]))
    (p_low, p_high), (d_low, d_high) = pads(ac)
    if p_high < p_low or d_high < d_low:
        return None
    design = {"a_x_mm": grid(length - ac), "ac_length_mm": ac,
              "app_x_mm": grid(p_low + v["app_share"] * (p_high - p_low)),
              "bpp_from_a_mm": grid(d_low + v["bpp_share"] * (d_high - d_low))}
    return tuple(design[name] for name in VARIABLE_NAMES)


def from_design(x: Sequence[float], config: RevoluteConfig) -> tuple[float, ...]:
    d = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    length = d["a_x_mm"] + d["ac_length_mm"]
    ac_range, pads = _ranges(config, length)
    (p_low, p_high), (d_low, d_high) = pads(d["ac_length_mm"])

    def share(value: float, low: float, high: float) -> float:
        return float(np.clip((value - low) / (high - low), 0.0, 1.0)) if high > low else 0.0

    coordinates = {"finger_length_mm": length, "ac_share": share(d["ac_length_mm"], *ac_range),
                   "app_share": share(d["app_x_mm"], p_low, p_high),
                   "bpp_share": share(d["bpp_from_a_mm"], d_low, d_high)}
    return tuple(coordinates[name] for name in SEARCH_NAMES)


__all__ = ["SEARCH_NAMES", "from_design", "least_a_x", "search_box", "search_steps", "to_design"]
