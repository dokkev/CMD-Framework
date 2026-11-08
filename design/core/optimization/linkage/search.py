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
5. ``B'`` from ``B``; the tilt every ``tilt_step_deg`` from the least of its
   range (the least itself when fixed); the pad offsets as they are, or, with
   the pads only kept off their own pins (``packaging.pads_off_pins``), each
   pad's end a share of its range, along the extended finger: ``A''_x`` from
   the pad's start plus its least length to ``C`` less the distal pad's least
   length (and at most ``proximal_pad_past_icr_max_mm`` past the extended
   centre of rotation), ``B''_x`` from ``A''_x`` (the pads side by side) to the same,
   each pad, on its link line, off its own joint's pin (:func:`_clear_of_pins`)
   (:func:`search_names`); with ``pads_on_links``, not searched: the proximal
   pad ends at ``A`` and the distal one starts at ``B``.

Every position is rounded to the prototyping resolution. Coordinates whose
ranges are empty describe no design (``None``).
"""

from __future__ import annotations

import functools
import math
from collections.abc import Sequence

import numpy as np

from core.morphology.packaging.gaps import servo_floor_x
from core.morphology.packaging.hardware import Packaging

from core.morphology.mechanism.linkage.design import DEFAULT_VALUES, VARIABLE_NAMES, decode
from core.morphology.mechanism.linkage.measures import extended_icr_x, symmetric_bp

from core.optimization.linkage.config import LinkageConfig


SEARCH_NAMES = ("finger_length_mm", "bc_length_mm", "b_z_mm", "a_z_mm", "ap_from_a_z_mm", "ap_x_share",
                "a_x_share", "bp_from_b_x_mm", "bp_from_b_z_mm", "servo_tilt_from_min_deg",
                "proximal_mount_offset_mm", "distal_mount_offset_mm", "app_offset_mm", "bpp_offset_mm")
SHARE_STEP = 0.001
"""Grid step of a share of a range."""
FLOOR_STEP_MM = 0.5
"""``A'_z`` step of the PIP servo floor's table."""
FLOOR_MARGIN_MM = 0.5
"""How far below the solved floor the search starts (the table is sampled)."""
PAD_SHARE_NAMES = ("app_share", "bpp_share")
"""The pad offsets' coordinates with the pads only kept off their own pins."""
MOUNT_NAMES = ("proximal_mount_offset_mm", "distal_mount_offset_mm")
"""The mounting depths: searched, or set by ``config.mount_policy``."""


SYMMETRIC_DROPPED = ("bp_from_b_x_mm", "bp_from_b_z_mm")
"""Not searched in a symmetric four-bar: ``B'`` follows from ``A``, ``A'`` and ``B``."""


def search_names(config: LinkageConfig) -> tuple[str, ...]:
    """:data:`SEARCH_NAMES`, the pad offsets as shares with the pads off their
    pins, without ``B'`` in a symmetric four-bar, without the distal mounting
    offset when the two are aligned (the proximal one sets both)."""

    if config.pads_on_links:  # the pads set by the links
        names = SEARCH_NAMES[:-2]
    else:
        names = (*SEARCH_NAMES[:-2], *PAD_SHARE_NAMES) if config.packaging.pads_off_pins else SEARCH_NAMES
    return tuple(name for name in names if not (config.symmetric_four_bar and name in SYMMETRIC_DROPPED)
                 and not (config.align_mount_offsets and name == "distal_mount_offset_mm")
                 and not (config.mount_policy != "searched" and name in MOUNT_NAMES))


def _pad_ranges(config: LinkageConfig, b_x: float, bc: float,
                icr_x: float | None = None) -> tuple[tuple[float, float], float]:
    """``A''_x``'s range (at most ``proximal_pad_past_icr_max_mm`` past the
    extended centre of rotation ``icr_x``) and ``B''_x``'s most with the pads
    only kept off their own pins (``B''_x`` from ``A''_x``)."""

    packaging = config.packaging
    last = b_x + bc - packaging.distal_pad.offset_mm - config.minimum_distal_support_mm
    past = config.proximal_pad_past_icr_max_mm
    proximal_last = last if past is None or icr_x is None else min(last, icr_x + past)
    return (packaging.proximal_pad_start_mm + config.minimum_proximal_support_mm, proximal_last), last


def _reach_into_block(center_z: float, radius: float, top: float, bottom: float) -> float:
    """How far along the line a disk of ``radius`` at height ``center_z``
    reaches into a block between ``bottom`` and ``top`` (0: it misses it)."""

    if center_z - radius >= top or center_z + radius <= bottom:
        return 0.0
    nearest = min(max(center_z, bottom), top)  # the block's height nearest the centre
    return math.sqrt(max(radius ** 2 - (center_z - nearest) ** 2, 0.0))


def _clear_of_pins(config: LinkageConfig, a_x: float, a_z: float, b_x: float,
                   offsets: tuple[float, float] = (0.0, 0.0)) -> tuple[float, float]:
    """The most ``A''_x`` and the least ``B''_x`` that keep each pad, mounted
    ``offsets`` below its link line, off its own joint's pin housing (``A``
    on its axis ``a_z`` above the proximal line, ``B`` on ``B-C``): short of
    the pin where its housing reaches into the pad's block, anywhere when the
    pad hangs below it (a deep enough offset). The evaluation checks the
    housings (``pads_off_pins``); the search does not offer pads it would
    always reject, whose violation (the housing's radius) shows no way out."""

    packaging, radius = config.packaging, config.packaging.pivot_radius_mm
    proximal_reach = _reach_into_block(a_z, radius("a"), -offsets[0], -offsets[0] - packaging.proximal_pad.height_mm)
    distal_reach = _reach_into_block(0.0, radius("b"), -offsets[1], -offsets[1] - packaging.distal_pad.height_mm)
    return (a_x - proximal_reach if proximal_reach > 0.0 else math.inf,
            b_x + distal_reach if distal_reach > 0.0 else -math.inf)


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
    """The tilts searched: from the least, every ``config.tilt_step_deg``."""

    low, high = config.bounds["servo_tilt_deg"]
    step = config.tilt_step_deg
    return low + step * np.arange(int(math.floor((high - low) / step + 1e-9)) + 1)


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
        **{name: bounds[name] for name in ("proximal_mount_offset_mm", "distal_mount_offset_mm",
                                           "app_offset_mm", "bpp_offset_mm")},
        **{name: (0.0, 1.0) for name in PAD_SHARE_NAMES},
    }
    names = search_names(config)
    return tuple(box[name][0] for name in names), tuple(box[name][1] for name in names)


def search_steps(config: LinkageConfig) -> tuple[float, ...]:
    return tuple(SHARE_STEP if name.endswith("_share") else config.tilt_step_deg if name == "servo_tilt_from_min_deg"
                 else config.resolution_mm for name in search_names(config))


def _ranges(config: LinkageConfig, b_x: float, ap_z: float, tilt_deg: float, length: float | None = None):
    """The ranges ``A'_x`` and then ``A_x`` are shares of."""

    resolution = config.resolution_mm
    dx_low, dx_high = config.relative_bounds["ap_from_a_x_mm"]
    a_low, a_high = config.bounds["a_x_mm"]
    a_low, a_high = max(a_low, resolution), min(a_high, b_x - resolution)
    span_low, span_high = config.joint_span_bounds_mm  # |AB|, A and B on the axis extended
    a_low = a_low if span_high is None else max(a_low, b_x - span_high)
    a_high = a_high if span_low is None else min(a_high, b_x - span_low)
    # (The segment ratios bound |OA''| and |B''C|, which the pads' ends set
    # after A: the evaluation checks them, the search does not narrow A by them.)
    if length is not None:
        span_low, span_high = config.joint_span_ratio_bounds  # |AB| / |OC|
        a_low = a_low if span_high is None else max(a_low, b_x - span_high * length)
        a_high = a_high if span_low is None else min(a_high, b_x - span_low * length)
    ap_range = (max(least_ap_x(config, ap_z, tilt_deg), a_low + dx_low), a_high + dx_high)

    def a_range(ap_x: float) -> tuple[float, float]:
        return max(a_low, ap_x - dx_high), min(a_high, ap_x - dx_low)

    return ap_range, a_range


def _on_grid(value: float, resolution: float) -> float:
    return round(round(value / resolution) * resolution, 9)


def to_design(coordinates: Sequence[float], config: LinkageConfig) -> tuple[float, ...] | None:
    v = dict(zip(search_names(config), (float(c) for c in coordinates)))
    if config.align_mount_offsets:
        v["distal_mount_offset_mm"] = v["proximal_mount_offset_mm"]
    if config.mount_policy != "searched":  # set below, once the pads' ends are: here the deepest allowed
        for name in MOUNT_NAMES:
            v[name] = config.bounds[name][1]
    resolution = config.resolution_mm
    grid = lambda value: _on_grid(value, resolution)  # noqa: E731
    b_x = grid(math.sqrt(max(v["finger_length_mm"] ** 2 - v["b_z_mm"] ** 2, 0.0)) - v["bc_length_mm"])
    if b_x <= 2.0 * resolution:
        return None
    ap_z = grid(v["a_z_mm"] + v["ap_from_a_z_mm"])
    tilt = round(config.bounds["servo_tilt_deg"][0] + v["servo_tilt_from_min_deg"], 9)
    ap_range, a_range = _ranges(config, b_x, ap_z, tilt, v["finger_length_mm"])
    if ap_range[1] < ap_range[0]:
        return None
    ap_x = grid(ap_range[0] + v["ap_x_share"] * (ap_range[1] - ap_range[0]))
    low, high = a_range(ap_x)
    if high < low:
        return None
    a_x = grid(low + v["a_x_share"] * (high - low))
    if not 0.0 < a_x < b_x:
        return None
    if config.symmetric_four_bar:
        bp = symmetric_bp(np.array((a_x, v["a_z_mm"])), np.array((ap_x, ap_z)), np.array((b_x, v["b_z_mm"])))
        if bp is None:
            return None
        bp_x, bp_z = grid(bp[0]), grid(bp[1])
    else:
        bp_x, bp_z = grid(b_x + v["bp_from_b_x_mm"]), grid(v["b_z_mm"] + v["bp_from_b_z_mm"])
    if config.pads_on_links:  # A'' at A, B'' at B
        v["app_offset_mm"], v["bpp_offset_mm"] = grid(a_x - ap_x), grid(bp_x - b_x)
    elif config.packaging.pads_off_pins:
        icr_x = extended_icr_x((a_x, v["a_z_mm"]), (ap_x, ap_z), (b_x, v["b_z_mm"]), (bp_x, bp_z))
        (p_low, p_high), d_high = _pad_ranges(config, b_x, v["bc_length_mm"], icr_x)
        offsets = (grid(v["proximal_mount_offset_mm"]), grid(v["distal_mount_offset_mm"]))
        proximal_most, distal_least = _clear_of_pins(config, a_x, v["a_z_mm"], b_x, offsets)
        p_high = min(p_high, proximal_most)
        if p_high < p_low:
            return None
        app = grid(p_low + v["app_share"] * (p_high - p_low))
        d_low = max(app, distal_least)
        if d_high < d_low:
            return None
        v["app_offset_mm"] = grid(app - ap_x)
        v["bpp_offset_mm"] = grid(bp_x - grid(d_low + v["bpp_share"] * (d_high - d_low)))
    design = {
        "a_x_mm": a_x, "b_x_mm": b_x, "b_z_mm": v["b_z_mm"], "ap_x_mm": ap_x, "ap_z_mm": ap_z,
        "bp_x_mm": bp_x, "bp_z_mm": bp_z,
        "servo_tilt_deg": tilt,
        **{name: v[name] for name in ("bc_length_mm", "a_z_mm", "app_offset_mm", "bpp_offset_mm")},
        **{name: grid(v[name]) for name in ("proximal_mount_offset_mm", "distal_mount_offset_mm")},
    }
    x = tuple(design[name] for name in VARIABLE_NAMES)
    if config.mount_policy == "searched":
        return x
    # Each pad at the least depth its own hardware needs, within what it supports.
    from core.optimization.mounting import linkage_mounting

    try:
        found = linkage_mounting(decode(x, config.link_length_resolution_mm), config)
    except ValueError:  # the rounded bars do not close: the evaluation says so
        return x
    if not found.supported:
        return None
    design.update(zip(MOUNT_NAMES, found.depths_mm))
    return tuple(design[name] for name in VARIABLE_NAMES)


def from_design(x: Sequence[float], config: LinkageConfig) -> tuple[float, ...]:
    """Search coordinates of a design (the shares clipped to their ranges;
    variables the vector predates at their defaults)."""

    d = {**DEFAULT_VALUES, **dict(zip(VARIABLE_NAMES, (float(value) for value in x)))}
    if config.align_mount_offsets:  # a design with its own offsets: the proximal one for both
        d["distal_mount_offset_mm"] = d["proximal_mount_offset_mm"]
    ap_range, a_range = _ranges(config, d["b_x_mm"], d["ap_z_mm"], d["servo_tilt_deg"],
                                math.hypot(d["b_x_mm"] + d["bc_length_mm"], d["b_z_mm"]))

    def share(value: float, low: float, high: float) -> float:
        return float(np.clip((value - low) / (high - low), 0.0, 1.0)) if high > low else 0.0

    coordinates = {
        "finger_length_mm": math.hypot(d["b_x_mm"] + d["bc_length_mm"], d["b_z_mm"]),
        "ap_from_a_z_mm": d["ap_z_mm"] - d["a_z_mm"],
        "ap_x_share": share(d["ap_x_mm"], *ap_range),
        "a_x_share": share(d["a_x_mm"], *a_range(d["ap_x_mm"])),
        "bp_from_b_x_mm": d["bp_x_mm"] - d["b_x_mm"],
        "bp_from_b_z_mm": d["bp_z_mm"] - d["b_z_mm"],
        "servo_tilt_from_min_deg": float(round((d["servo_tilt_deg"] - config.bounds["servo_tilt_deg"][0])
                                               / config.tilt_step_deg) * config.tilt_step_deg),
        **{name: d[name] for name in ("bc_length_mm", "b_z_mm", "a_z_mm", "app_offset_mm", "bpp_offset_mm",
                                      "proximal_mount_offset_mm", "distal_mount_offset_mm")},
    }
    if config.packaging.pads_off_pins and not config.pads_on_links:
        icr_x = extended_icr_x((d["a_x_mm"], d["a_z_mm"]), (d["ap_x_mm"], d["ap_z_mm"]),
                               (d["b_x_mm"], d["b_z_mm"]), (d["bp_x_mm"], d["bp_z_mm"]))
        (p_low, p_high), d_high = _pad_ranges(config, d["b_x_mm"], d["bc_length_mm"], icr_x)
        offsets = (tuple(config.bounds[name][1] for name in MOUNT_NAMES) if config.mount_policy != "searched"
                   else (d["proximal_mount_offset_mm"], d["distal_mount_offset_mm"]))  # as to_design ranges the ends
        proximal_most, distal_least = _clear_of_pins(config, d["a_x_mm"], d["a_z_mm"], d["b_x_mm"], offsets)
        app = d["ap_x_mm"] + d["app_offset_mm"]
        coordinates["app_share"] = share(app, p_low, min(p_high, proximal_most))
        coordinates["bpp_share"] = share(d["bp_x_mm"] - d["bpp_offset_mm"], max(app, distal_least), d_high)
    return tuple(coordinates[name] for name in search_names(config))


__all__ = ["SEARCH_NAMES", "from_design", "least_ap_x", "search_box", "search_names", "search_steps",
           "tilts_deg", "to_design"]
