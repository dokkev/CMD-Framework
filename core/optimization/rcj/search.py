"""The coordinates the RCJ is searched in, each in a range the earlier ones
set (as ``core.optimization.revolute.search``):

1. ``|OC|``;
2. the rollers' radius ``r`` itself;
3. ``BC`` a share of the range keeping ``|OA| = |OC| - 2r - BC`` within its
   bounds, the least ``|OA|`` also keeping the servo off the MCP servo;
4. ``A''_x`` a share of ``[the pad's start + its least length, A_x + r]``;
5. ``B''`` from ``B`` a share of ``[-DISTAL_PAD_REACH_MM, BC - its least length]``;
6. the distal pad's drop a share of ``[0, r]``.

Lengths are rounded to the prototyping resolution; empty ranges describe no design.
"""

from __future__ import annotations

import functools
from collections.abc import Sequence

import numpy as np

from core.morphology.linkage.packaging import Packaging, servo_floor_x

from core.optimization.rcj import config as rcj_config
from core.optimization.rcj.config import DISTAL_PAD_REACH_MM, VARIABLE_NAMES, RcjConfig


SEARCH_NAMES = ("finger_length_mm", "roller_radius_mm", "bc_share", "app_share", "bpp_share", "drop_share")
SHARE_STEP = 0.001


@functools.lru_cache(maxsize=64)
def least_a_x(packaging: Packaging, angles: tuple[float, ...]) -> float:
    """The least ``|OA|`` keeping the servo on ``A`` clear of the MCP servo."""

    return servo_floor_x(packaging, 0.0, rcj_config.SERVO_REAR, angles)


def search_box(config: RcjConfig) -> tuple[tuple[float, ...], tuple[float, ...]]:
    box = {"finger_length_mm": config.goal.extended_length_bounds_mm,
           "roller_radius_mm": config.bounds["roller_radius_mm"]}
    return (tuple(box.get(name, (0.0, 1.0))[0] for name in SEARCH_NAMES),
            tuple(box.get(name, (0.0, 1.0))[1] for name in SEARCH_NAMES))


def search_steps(config: RcjConfig) -> tuple[float, ...]:
    return tuple(SHARE_STEP if name.endswith("_share") else config.resolution_mm for name in SEARCH_NAMES)


def _ranges(config: RcjConfig, length: float, r: float):
    goal = config.goal
    a_low, a_high = goal.bounds["a_x_mm"]
    a_low = max(a_low, least_a_x(goal.packaging, tuple(goal.mcp_servo_angles_rad)) - 0.5)  # sampled: stay below it
    bc_low, bc_high = config.bounds["bc_length_mm"]
    bc_range = (max(bc_low, length - 2.0 * r - a_high), min(bc_high, length - 2.0 * r - a_low))
    start = goal.packaging.proximal_pad_start_mm

    def pads(bc: float) -> tuple[tuple[float, float], tuple[float, float]]:
        a_x = length - 2.0 * r - bc
        return ((start + goal.minimum_proximal_support_mm, a_x + r),
                (-DISTAL_PAD_REACH_MM, bc - goal.minimum_distal_support_mm))

    return bc_range, pads


def to_design(coordinates: Sequence[float], config: RcjConfig) -> tuple[float, ...] | None:
    v = dict(zip(SEARCH_NAMES, (float(c) for c in coordinates)))
    resolution = config.resolution_mm
    grid = lambda value: round(round(value / resolution) * resolution, 9)  # noqa: E731
    length, r = v["finger_length_mm"], grid(v["roller_radius_mm"])
    bc_range, pads = _ranges(config, length, r)
    if bc_range[1] < bc_range[0]:
        return None
    bc = grid(bc_range[0] + v["bc_share"] * (bc_range[1] - bc_range[0]))
    (p_low, p_high), (d_low, d_high) = pads(bc)
    if p_high < p_low or d_high < d_low:
        return None
    design = {"a_x_mm": grid(length - 2.0 * r - bc), "roller_radius_mm": r, "bc_length_mm": bc,
              "app_x_mm": grid(p_low + v["app_share"] * (p_high - p_low)),
              "bpp_from_b_mm": grid(d_low + v["bpp_share"] * (d_high - d_low)),
              "distal_drop_mm": min(grid(v["drop_share"] * r), r)}
    return tuple(design[name] for name in VARIABLE_NAMES)


def from_design(x: Sequence[float], config: RcjConfig) -> tuple[float, ...]:
    d = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    r = d["roller_radius_mm"]
    length = rcj_config.finger_length_mm(d["a_x_mm"], r, d["bc_length_mm"])
    bc_range, pads = _ranges(config, length, r)
    (p_low, p_high), (d_low, d_high) = pads(d["bc_length_mm"])

    def share(value: float, low: float, high: float) -> float:
        return float(np.clip((value - low) / (high - low), 0.0, 1.0)) if high > low else 0.0

    coordinates = {"finger_length_mm": length, "roller_radius_mm": r,
                   "bc_share": share(d["bc_length_mm"], *bc_range),
                   "app_share": share(d["app_x_mm"], p_low, p_high),
                   "bpp_share": share(d["bpp_from_b_mm"], d_low, d_high),
                   "drop_share": share(d["distal_drop_mm"], 0.0, r)}
    return tuple(coordinates[name] for name in SEARCH_NAMES)


__all__ = ["SEARCH_NAMES", "from_design", "least_a_x", "search_box", "search_steps", "to_design"]
