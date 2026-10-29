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
   centre of rotation), ``B''_x`` from ``A''_x`` (the pads side by side) to the same
   (:func:`search_names`); with ``pads_on_links``, not searched: the proximal
   pad ends at ``A`` and the distal one starts at ``B``. The pads are plain
   blocks: the design vector's chamfers are held at 0. 6. The pads' drops
   below their link lines themselves, when searched (``goal.pads.*.drop_mm``);
   else NaN in the design vector: the packaging's.

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
FLOOR_STEP_MM = 0.5
"""``A'_z`` step of the PIP servo floor's table."""
FLOOR_MARGIN_MM = 0.5
"""How far below the solved floor the search starts (the table is sampled)."""
PAD_SHARE_NAMES = ("app_share", "bpp_share")
"""The pad offsets' coordinates with the pads only kept off their own pins."""


SYMMETRIC_DROPPED = ("bp_from_b_x_mm", "bp_from_b_z_mm")
"""Not searched in a symmetric four-bar: ``B'`` follows from ``A``, ``A'`` and ``B``."""


def search_names(config: LinkageConfig) -> tuple[str, ...]:
    """:data:`SEARCH_NAMES`, the pad offsets as shares with the pads off their
    pins, without ``B'`` in a symmetric four-bar."""

    if config.pads_on_links:  # the pads set by the links
        names = SEARCH_NAMES[:-2]
    else:
        names = (*SEARCH_NAMES[:-2], *PAD_SHARE_NAMES) if config.packaging.pads_off_pins else SEARCH_NAMES
    names = (*names, *config.searched_drops)
    return tuple(name for name in names if not (config.symmetric_four_bar and name in SYMMETRIC_DROPPED))


def symmetric_bp(a: np.ndarray, ap: np.ndarray, b: np.ndarray) -> np.ndarray | None:
    """``B'`` making ``A'-B-B'-A`` an antiparallelogram (``|B'B| = |A'A|``,
    ``|AB'| = |A'B|``, the bars crossed), ``None`` when the circles miss: of
    the two crossings, the one away from the parallelogram's ``B + A - A'``."""

    g, r = float(np.linalg.norm(a - ap)), float(np.linalg.norm(b - ap))
    d = float(np.linalg.norm(b - a))
    if not abs(r - g) < d < r + g:
        return None
    along = (r * r - g * g + d * d) / (2.0 * d)  # from A toward B
    unit = (b - a) / d
    off = math.sqrt(max(r * r - along * along, 0.0)) * np.array((-unit[1], unit[0]))
    parallel = b + a - ap
    return max((a + along * unit + off, a + along * unit - off), key=lambda p: float(np.linalg.norm(p - parallel)))


def extended_icr_x(a: Sequence[float], ap: Sequence[float], b: Sequence[float], bp: Sequence[float]) -> float | None:
    """``x`` of the extended pose's centre of rotation: ``A-B'`` crossing
    ``A'-B`` (``None``: the bars parallel)."""

    a, ap, b, bp = (np.asarray(point, dtype=float) for point in (a, ap, b, bp))
    first, second = bp - a, b - ap
    det = first[0] * -second[1] - first[1] * -second[0]
    if abs(det) < 1e-9:
        return None
    gap = ap - a
    t = (gap[0] * -second[1] - gap[1] * -second[0]) / det
    return float(a[0] + t * first[0])


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
        **{name: bounds[name] for name in ("app_offset_mm", "bpp_offset_mm")},
        **{name: (0.0, 1.0) for name in PAD_SHARE_NAMES},
        **{name: bounds[name] for name in ("proximal_drop_mm", "distal_drop_mm")},
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
    ratio_low, ratio_high = config.proximal_length_ratio_bounds  # |OA| / |OC|, A on the axis
    if length is not None:
        a_low = a_low if ratio_low is None else max(a_low, ratio_low * length)
        a_high = a_high if ratio_high is None else min(a_high, ratio_high * length)
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
        if p_high < p_low:
            return None
        app = grid(p_low + v["app_share"] * (p_high - p_low))
        v["app_offset_mm"] = grid(app - ap_x)
        v["bpp_offset_mm"] = grid(bp_x - grid(app + v["bpp_share"] * (d_high - app)))
    design = {
        "a_x_mm": a_x, "b_x_mm": b_x, "b_z_mm": v["b_z_mm"], "ap_x_mm": ap_x, "ap_z_mm": ap_z,
        "bp_x_mm": bp_x, "bp_z_mm": bp_z,
        "servo_tilt_deg": tilt,
        **{name: v[name] for name in ("bc_length_mm", "a_z_mm", "app_offset_mm", "bpp_offset_mm")},
        "proximal_chamfer_mm": 0.0, "distal_chamfer_mm": 0.0,
        **{name: grid(v[name]) if name in v else math.nan for name in ("proximal_drop_mm", "distal_drop_mm")},
    }
    return tuple(design[name] for name in VARIABLE_NAMES)


def from_design(x: Sequence[float], config: LinkageConfig) -> tuple[float, ...]:
    """Search coordinates of a design (the shares clipped to their ranges)."""

    d = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
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
        **{name: d[name] for name in ("bc_length_mm", "b_z_mm", "a_z_mm", "app_offset_mm", "bpp_offset_mm")},
        **{name: d[name] for name in config.searched_drops},
    }
    if config.packaging.pads_off_pins and not config.pads_on_links:
        icr_x = extended_icr_x((d["a_x_mm"], d["a_z_mm"]), (d["ap_x_mm"], d["ap_z_mm"]),
                               (d["b_x_mm"], d["b_z_mm"]), (d["bp_x_mm"], d["bp_z_mm"]))
        (p_low, p_high), d_high = _pad_ranges(config, d["b_x_mm"], d["bc_length_mm"], icr_x)
        app = d["ap_x_mm"] + d["app_offset_mm"]
        coordinates["app_share"] = share(app, p_low, p_high)
        coordinates["bpp_share"] = share(d["bp_x_mm"] - d["bpp_offset_mm"], app, d_high)
    return tuple(coordinates[name] for name in search_names(config))


__all__ = ["SEARCH_NAMES", "from_design", "least_ap_x", "search_box", "search_names", "search_steps",
           "extended_icr_x", "symmetric_bp", "tilts_deg", "to_design"]
