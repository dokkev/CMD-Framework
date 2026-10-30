"""Gaps between a finger's solids as its :class:`~core.morphology.packaging.hardware.Packaging`
sets them: signed by the overlap's depth with ``overlap_depth``, and the MCP
servo turning about ``O`` over the MCP flexion samples."""

from __future__ import annotations

import functools
import math
from typing import Sequence

import numpy as np

from core.morphology.packaging.hardware import MARGIN_TOLERANCE_MM, Packaging
from core.morphology.packaging.shapes import (
    ServoFootprint,
    point_segment_distance,
    polygon_edges,
    polygon_to_footprint,
    polygons_gap,
    separation,
    signed_gaps,
    turn_points,
)

EXACT_BAND_MM = 5.0
"""Pad gaps are exact up to the body clearance plus this; a
larger gap may be reported by a cheaper lower bound (a bounding-box gap, or
``|p| - reach`` against the MCP servo), which is still beyond it."""


_MCP_COARSE_STEP_RAD = math.radians(5.0)


def polygon_gaps(first: np.ndarray, second: np.ndarray, packaging: Packaging) -> np.ndarray:
    """Gap per sample between convex ``(n, m, 2)`` and ``(n, k, 2)`` polygons;
    with ``packaging.overlap_depth`` an overlap is negative by its depth."""

    gap = polygons_gap(first, second)
    return signed_gaps(gap, first, second) if packaging.overlap_depth else gap


def footprint_gaps(polygons: np.ndarray, footprint: ServoFootprint, packaging: Packaging) -> np.ndarray:
    """Gap per sample between ``(n, m, 2)`` polygons and a servo footprint
    (signed as :func:`polygon_gaps`)."""

    gap = polygon_to_footprint(polygons, footprint)
    if not packaging.overlap_depth:
        return gap
    return signed_gaps(gap, polygons, np.broadcast_to(footprint.outline, (len(polygons), *footprint.outline.shape)))


def mcp_servo_gaps(polygons: np.ndarray, packaging: Packaging, mcp_samples_rad: Sequence[float]) -> np.ndarray:
    """Least gap of ``(n, m, 2)`` polygons to the MCP servo over the MCP flexion
    samples (with ``packaging.overlap_depth``, an overlap negative by its depth).

    MCP flexion turns the servo about ``O`` in the proximal frame, which keeps
    every point's distance to ``O``: an edge ``d`` from ``O`` is at least
    ``d - reach`` from the servo at every MCP angle. Edges within
    :data:`EXACT_BAND_MM` of clearing it by that bound are checked on a coarse
    MCP grid first, bounded by how far the servo moves between its angles, and
    at every sample only if that bound does not clear them either; the rest
    report their bound.
    """

    servo = packaging.mcp_footprint()
    count = len(polygons)
    start, end = (array.reshape(-1, 2) for array in polygon_edges(polygons))
    from_o = point_segment_distance(np.zeros(2), start, end)
    gap = from_o - servo.reach_mm
    exact = packaging.mcp_pad_clearance_mm + EXACT_BAND_MM

    def at(edges: np.ndarray, angles: Sequence[float]) -> np.ndarray:
        # Turning the MCP servo by +q is turning the pads by -q about O.
        cosine, sine = np.cos(angles)[:, None], -np.sin(angles)[:, None]

        def turned(points: np.ndarray) -> np.ndarray:
            x, z = points[:, 0], points[:, 1]
            return np.stack((cosine * x - sine * z, sine * x + cosine * z), axis=-1).reshape(-1, 2)

        starts, ends = turned(start[edges]), turned(end[edges])
        return servo.distance_to_segments(starts, ends).reshape(len(angles), edges.size).min(axis=0)

    near = np.flatnonzero(gap <= exact)
    samples = np.asarray(mcp_samples_rad, dtype=float)
    if near.size:
        # Coarse pass: every angle is within half a coarse step of one, which
        # moves a servo point by at most reach * step / 2.
        stride = max(1, int(round(_MCP_COARSE_STEP_RAD / max(np.diff(samples).min(initial=math.inf), 1e-12))))
        coarse = np.unique(np.append(samples[::stride], samples[-1]))
        slack = servo.reach_mm * stride * (np.diff(samples).max(initial=0.0)) / 2.0
        gap[near] = at(near, coarse) - slack
        fine = near[gap[near] <= exact]
        if fine.size:
            gap[fine] = at(fine, samples)
    result = gap.reshape(count, -1).min(axis=1)
    touching = np.flatnonzero(result <= MARGIN_TOLERANCE_MM)
    if packaging.overlap_depth and touching.size:  # touching or overlapping at some MCP angle
        outline = servo.outline
        depth = np.full(touching.size, math.inf)
        for angle in samples:
            turned = turn_points(polygons[touching].reshape(-1, 2), -angle).reshape(touching.size, -1, 2)
            depth = np.minimum(depth, separation(turned, np.broadcast_to(outline, (touching.size, *outline.shape))))
        result[touching] = np.where(depth < 0.0, depth, result[touching])
    return result


def proximal_pad_mcp_gap(polygon: np.ndarray, packaging: Packaging, mcp_samples_rad: Sequence[float]) -> float:
    """Gap of the proximal pad (``(m, 2)``, fixed in the proximal frame) to the
    MCP servo over the MCP flexion samples.

    The pad starts at the servo by construction, so near the servo it is the
    same block in every design: its start, line and height are the study's.
    When the pad (chamfer included) ends beyond the servo's reach and the exact
    band, only that block can come near the servo, and its gap is solved once
    per study (:func:`_proximal_block_gap`); otherwise the whole pad is checked.
    """

    limit = packaging.mcp_footprint().reach_mm + packaging.mcp_pad_clearance_mm + EXACT_BAND_MM
    height = packaging.proximal_pad.height_mm
    near = polygon[:, 0] < limit
    if polygon[:, 0].max() - height > limit and near.any():
        top = float(polygon[:, 1].max())
        start = float(polygon[:, 0].min())
        bottom = float(polygon[:, 1].min())
        if np.allclose(polygon[near][:, 0], start) and math.isclose(top - bottom, height):  # the square edge a-c
            return _proximal_block_gap(packaging, start, top, height, limit, tuple(mcp_samples_rad))
    return float(mcp_servo_gaps(polygon[None], packaging, mcp_samples_rad)[0])


@functools.lru_cache(maxsize=64)
def _proximal_block_gap(packaging: Packaging, start: float, top: float, height: float, limit: float,
                        mcp_samples_rad: tuple[float, ...]) -> float:
    """The MCP servo gap of the proximal pad's block within ``limit`` of ``O``."""

    block = np.array(((start, top - height), (limit, top - height), (limit, top), (start, top)))
    return float(mcp_servo_gaps(block[None], packaging, mcp_samples_rad)[0])


def servo_floor_x(packaging: Packaging, pivot_z: float, rear: tuple[float, float],
                  mcp_angles_rad: Sequence[float]) -> float:
    """The least shaft ``x`` (at ``pivot_z``) of a servo of the PIP servo's size,
    laid back along ``rear``, that keeps it clear of the MCP servo at the MCP
    angles: searched from far ahead of ``O`` back to the first overlap (far
    behind ``O`` it is clear again, which is no finger), in 5 mm steps, then
    bisected to 0.05 mm."""

    servo = packaging.pip_servo
    angles = tuple(mcp_angles_rad)

    def clear(x: float) -> bool:
        footprint = ServoFootprint((x, pivot_z), rear, servo.radius_mm, servo.rear_extent_mm)
        return packaging.mcp_servo_gap_mm(footprint, angles) - packaging.body_clearance_mm >= 0.0

    high = 200.0
    low = high - 5.0
    while low > -200.0 and clear(low):
        high, low = low, low - 5.0
    while high - low > 0.05:
        middle = 0.5 * (low + high)
        low, high = (low, middle) if clear(middle) else (middle, high)
    return high


__all__ = [
    "EXACT_BAND_MM",
    "footprint_gaps",
    "mcp_servo_gaps",
    "polygon_gaps",
    "proximal_pad_mcp_gap",
    "servo_floor_x",
]
