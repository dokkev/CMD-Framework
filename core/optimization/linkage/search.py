"""The coordinates the linkage is searched in.

The design vector (``design.VARIABLE_NAMES``) holds absolute positions, and
most of its box breaks the simplest constraints: ``B`` behind ``A``, ``|OC|``
out of range, the PIP servo in the MCP servo. The optimizer searches these
coordinates instead, each in a range the earlier ones set, so that those
hold by construction:

1. ``|OC|`` and ``BC`` give ``B_x = sqrt(|OC|^2 - B_z^2) - BC``;
2. ``A'_z = A_z +`` its offset from ``A``;
3. ``A'_x`` a share of its range: from the least that keeps the PIP servo
   off the MCP servo at that ``A'_z`` and tilt (:func:`least_ap_x`) and the
   least ``A`` allows, to the most ``A`` (short of ``B``) allows;
4. ``A_x`` a share of what ``A'`` and ``B`` leave it: ``A'`` within its
   offset from ``A``, ``A`` between ``O`` and ``B`` and within the goal's ``|OA|``;
5. ``B'`` from ``B``; the tilt in whole degrees from the least of its
   range (the least itself when fixed); the pad offsets as they are. The pads
   are plain blocks: the design vector's chamfers are held at 0.

Every position is rounded to the prototyping resolution. Coordinates whose
ranges are empty describe no design (``None``).
"""

from __future__ import annotations

import functools
import math
from collections.abc import Sequence

import numpy as np

from core.morphology.linkage.packaging import Packaging, servo_floor_x

from core.optimization.linkage.config import LinkageConfig
from core.optimization.linkage.design import VARIABLE_NAMES


SEARCH_NAMES = ("finger_length_mm", "bc_length_mm", "b_z_mm", "a_z_mm", "ap_from_a_z_mm", "ap_x_share",
                "a_x_share", "bp_from_b_x_mm", "bp_from_b_z_mm", "servo_tilt_from_min_deg", "app_offset_mm",
                "bpp_offset_mm")
SHARE_STEP = 0.001
"""Grid step of a share of a range."""
TILT_STEP_DEG = 1.0
"""Grid step of the PIP servo's tilt, from the least of its range."""
FLOOR_STEP_MM = 0.5
"""``A'_z`` step of the PIP servo floor's table."""
FLOOR_MARGIN_MM = 0.5
"""How far below the solved floor the search starts (the table is sampled)."""


@functools.lru_cache(maxsize=256)
def _floor_table(packaging: Packaging, tilt_deg: float, angles: tuple[float, ...],
                 z_low: float, z_high: float) -> tuple[np.ndarray, np.ndarray]:
    """``A'_z`` samples over its search range and the least ``A'_x`` at each
    that keeps the PIP servo (``tilt_deg``) clear of the MCP servo."""

    tilt = math.radians(tilt_deg)
    rear = (-math.cos(tilt), math.sin(tilt))
    z = np.arange(z_low, z_high + FLOOR_STEP_MM, FLOOR_STEP_MM)
    return z, np.array([servo_floor_x(packaging, float(zz), rear, angles) for zz in z])


def least_ap_x(config: LinkageConfig, ap_z: float, tilt_deg: float) -> float:
    """The least ``A'_x`` searched at ``A'_z``: the PIP servo floor, less a
    margin for the table's sampling (the constraint itself stays checked).
    An ``A'_z`` off the search range takes the range's end."""

    z_low, z_high = config.bounds["ap_z_mm"]
    z, floor = _floor_table(config.packaging, round(tilt_deg, 6), tuple(config.mcp_servo_angles_rad),
                            z_low, z_high)
    index = int(np.clip(np.searchsorted(z, ap_z), 1, max(len(z) - 1, 1)))
    return float(min(floor[index - 1], floor[min(index, len(z) - 1)])) - FLOOR_MARGIN_MM


def tilts_deg(config: LinkageConfig) -> np.ndarray:
    """The tilts searched: from the least, every :data:`TILT_STEP_DEG`."""

    low, high = config.bounds["servo_tilt_deg"]
    return low + TILT_STEP_DEG * np.arange(int(math.floor((high - low) / TILT_STEP_DEG + 1e-9)) + 1)


def search_box(config: LinkageConfig) -> tuple[tuple[float, ...], tuple[float, ...]]:
    bounds, relative = config.search_bounds, config.relative_bounds
    box = {
        "finger_length_mm": config.extended_length_bounds_mm,
        "bc_length_mm": bounds["bc_length_mm"],
        "b_z_mm": bounds["b_z_mm"],
        "a_z_mm": bounds["a_z_mm"],
        "ap_from_a_z_mm": relative["ap_from_a_z_mm"],
        "ap_x_share": (0.0, 1.0),
        "a_x_share": (0.0, 1.0),
        "bp_from_b_x_mm": relative["bp_from_b_x_mm"],
        "bp_from_b_z_mm": relative["bp_from_b_z_mm"],
        "servo_tilt_from_min_deg": (0.0, float(tilts_deg(config)[-1] - bounds["servo_tilt_deg"][0])),
        **{name: bounds[name] for name in ("app_offset_mm", "bpp_offset_mm")},
    }
    return tuple(box[name][0] for name in SEARCH_NAMES), tuple(box[name][1] for name in SEARCH_NAMES)


def search_steps(config: LinkageConfig) -> tuple[float, ...]:
    return tuple(SHARE_STEP if name.endswith("_share") else TILT_STEP_DEG if name == "servo_tilt_from_min_deg"
                 else config.resolution_mm for name in SEARCH_NAMES)


def _ranges(config: LinkageConfig, b_x: float, ap_z: float, tilt_deg: float):
    """The ranges ``A'_x`` and then ``A_x`` are shares of."""

    resolution = config.resolution_mm
    dx_low, dx_high = config.relative_bounds["ap_from_a_x_mm"]
    a_low, a_high = config.bounds["a_x_mm"]
    a_low, a_high = max(a_low, resolution), min(a_high, b_x - resolution)
    ap_range = (max(least_ap_x(config, ap_z, tilt_deg), a_low + dx_low), a_high + dx_high)

    def a_range(ap_x: float) -> tuple[float, float]:
        return max(a_low, ap_x - dx_high), min(a_high, ap_x - dx_low)

    return ap_range, a_range


def _on_grid(value: float, resolution: float) -> float:
    return round(round(value / resolution) * resolution, 9)


def to_design(coordinates: Sequence[float], config: LinkageConfig) -> tuple[float, ...] | None:
    v = dict(zip(SEARCH_NAMES, (float(c) for c in coordinates)))
    resolution = config.resolution_mm
    grid = lambda value: _on_grid(value, resolution)  # noqa: E731
    b_x = grid(math.sqrt(max(v["finger_length_mm"] ** 2 - v["b_z_mm"] ** 2, 0.0)) - v["bc_length_mm"])
    if b_x <= 2.0 * resolution:
        return None
    ap_z = grid(v["a_z_mm"] + v["ap_from_a_z_mm"])
    tilt = round(config.bounds["servo_tilt_deg"][0] + v["servo_tilt_from_min_deg"], 9)
    ap_range, a_range = _ranges(config, b_x, ap_z, tilt)
    if ap_range[1] < ap_range[0]:
        return None
    ap_x = grid(ap_range[0] + v["ap_x_share"] * (ap_range[1] - ap_range[0]))
    low, high = a_range(ap_x)
    if high < low:
        return None
    a_x = grid(low + v["a_x_share"] * (high - low))
    if not 0.0 < a_x < b_x:
        return None
    design = {
        "a_x_mm": a_x, "b_x_mm": b_x, "b_z_mm": v["b_z_mm"], "ap_x_mm": ap_x, "ap_z_mm": ap_z,
        "bp_x_mm": grid(b_x + v["bp_from_b_x_mm"]), "bp_z_mm": grid(v["b_z_mm"] + v["bp_from_b_z_mm"]),
        "servo_tilt_deg": tilt,
        **{name: v[name] for name in ("bc_length_mm", "a_z_mm", "app_offset_mm", "bpp_offset_mm")},
        "proximal_chamfer_mm": 0.0, "distal_chamfer_mm": 0.0,
    }
    return tuple(design[name] for name in VARIABLE_NAMES)


def from_design(x: Sequence[float], config: LinkageConfig) -> tuple[float, ...]:
    """Search coordinates of a design (the shares clipped to their ranges)."""

    d = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    ap_range, a_range = _ranges(config, d["b_x_mm"], d["ap_z_mm"], d["servo_tilt_deg"])

    def share(value: float, low: float, high: float) -> float:
        return float(np.clip((value - low) / (high - low), 0.0, 1.0)) if high > low else 0.0

    coordinates = {
        "finger_length_mm": math.hypot(d["b_x_mm"] + d["bc_length_mm"], d["b_z_mm"]),
        "ap_from_a_z_mm": d["ap_z_mm"] - d["a_z_mm"],
        "ap_x_share": share(d["ap_x_mm"], *ap_range),
        "a_x_share": share(d["a_x_mm"], *a_range(d["ap_x_mm"])),
        "bp_from_b_x_mm": d["bp_x_mm"] - d["b_x_mm"],
        "bp_from_b_z_mm": d["bp_z_mm"] - d["b_z_mm"],
        "servo_tilt_from_min_deg": float(round(d["servo_tilt_deg"] - config.bounds["servo_tilt_deg"][0])),
        **{name: d[name] for name in ("bc_length_mm", "b_z_mm", "a_z_mm", "app_offset_mm", "bpp_offset_mm")},
    }
    return tuple(coordinates[name] for name in SEARCH_NAMES)


__all__ = ["SEARCH_NAMES", "TILT_STEP_DEG", "from_design", "least_ap_x", "search_box", "search_steps", "tilts_deg",
           "to_design"]
