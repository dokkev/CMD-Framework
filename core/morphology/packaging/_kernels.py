"""Compiled kernels of the packaging's planar geometry (numba).

Each one is the loop form of an array expression in ``packaging``, with the
same arithmetic in the same order, so that the results match the array form.
The array form called numpy dozens of times per small polygon pair; here a
whole batch of samples runs in one compiled call.
"""

from __future__ import annotations

import math

import numpy as np
from numba import njit


@njit(cache=True, inline="always")
def _point_segment(px, py, sx, sy, ex, ey):
    dx, dy = ex - sx, ey - sy
    length_squared = dx * dx + dy * dy
    t = ((px - sx) * dx + (py - sy) * dy) / length_squared if length_squared > 0.0 else 0.0
    t = min(max(t, 0.0), 1.0)
    nx, ny = px - (sx + t * dx), py - (sy + t * dy)
    return math.sqrt(nx * nx + ny * ny)


@njit(cache=True, inline="always")
def _cross(ox, oy, px, py, qx, qy):
    return (px - ox) * (qy - oy) - (py - oy) * (qx - ox)


@njit(cache=True, inline="always")
def _segments_cross(ax, ay, bx, by, cx, cy, dx, dy):
    d1, d2 = _cross(ax, ay, bx, by, cx, cy), _cross(ax, ay, bx, by, dx, dy)
    d3, d4 = _cross(cx, cy, dx, dy, ax, ay), _cross(cx, cy, dx, dy, bx, by)
    collinear = d1 == 0.0 and d2 == 0.0
    return d1 * d2 <= 0.0 and d3 * d4 <= 0.0 and not collinear


@njit(cache=True)
def point_segment_distance(point, start, end):
    """Distance from ``point[i]`` to segment ``start[i]-end[i]`` (``(n, 2)`` each)."""

    out = np.empty(len(start))
    for i in range(len(start)):
        out[i] = _point_segment(point[i, 0], point[i, 1], start[i, 0], start[i, 1], end[i, 0], end[i, 1])
    return out


@njit(cache=True)
def segments_cross(start, end, q0, q1):
    out = np.empty(len(start), dtype=np.bool_)
    for i in range(len(start)):
        out[i] = _segments_cross(start[i, 0], start[i, 1], end[i, 0], end[i, 1],
                                 q0[i, 0], q0[i, 1], q1[i, 0], q1[i, 1])
    return out


@njit(cache=True, inline="always")
def _inside(px, py, polygon):
    """Whether the point lies inside (or on) the convex polygon ``(m, 2)``."""

    m = len(polygon)
    all_positive, all_negative = True, True
    for a in range(m):
        b = (a + 1) % m
        ex, ey = polygon[b, 0] - polygon[a, 0], polygon[b, 1] - polygon[a, 1]
        ox, oy = px - polygon[a, 0], py - polygon[a, 1]
        value = ex * oy - ey * ox
        if value < 0.0:
            all_positive = False
        if value > 0.0:
            all_negative = False
    return all_positive or all_negative


@njit(cache=True)
def inside(points, polygons):
    """``(n, p)``: whether ``points[i, j]`` lies inside the convex ``polygons[i]``."""

    out = np.empty((points.shape[0], points.shape[1]), dtype=np.bool_)
    for i in range(points.shape[0]):
        for j in range(points.shape[1]):
            out[i, j] = _inside(points[i, j, 0], points[i, j, 1], polygons[i])
    return out


@njit(cache=True)
def probe_polygon(start, end, polygons):
    """``(n, k)``: whether the probe ``start[i, j] -> end[i, j]`` starts inside
    the convex ``polygons[i]`` or crosses one of its edges (see ``packaging.probe_blocked``)."""

    out = np.empty((start.shape[0], start.shape[1]), dtype=np.bool_)
    m = polygons.shape[1]
    for i in range(start.shape[0]):
        polygon = polygons[i]
        low_x, high_x = polygon[:, 0].min(), polygon[:, 0].max()
        low_y, high_y = polygon[:, 1].min(), polygon[:, 1].max()
        for j in range(start.shape[1]):
            sx, sy, ex, ey = start[i, j, 0], start[i, j, 1], end[i, j, 0], end[i, j, 1]
            if (max(sx, ex) < low_x or min(sx, ex) > high_x or max(sy, ey) < low_y
                    or min(sy, ey) > high_y):  # the boxes apart: no hit
                out[i, j] = False
                continue
            hit = _inside(sx, sy, polygon)
            a = 0
            while not hit and a < m:
                b = (a + 1) % m
                hit = _segments_cross(sx, sy, ex, ey, polygon[a, 0], polygon[a, 1], polygon[b, 0], polygon[b, 1])
                a += 1
            out[i, j] = hit
    return out


@njit(cache=True)
def separation(first, second):
    """Signed separation of convex polygons per sample (see ``packaging._separation``)."""

    count = first.shape[0]
    out = np.empty(count)
    for i in range(count):
        best = -np.inf
        for which in range(2):
            polygon = first[i] if which == 0 else second[i]
            m = len(polygon)
            for a in range(m):
                b = (a + 1) % m
                ex, ey = polygon[b, 0] - polygon[a, 0], polygon[b, 1] - polygon[a, 1]
                nx, ny = ey, -ex
                length = math.sqrt(nx * nx + ny * ny)
                if not length > 0.0:  # a repeated vertex is no axis
                    continue
                nx, ny = nx / length, ny / length
                low1, high1 = np.inf, -np.inf
                for p in range(first.shape[1]):
                    value = nx * first[i, p, 0] + ny * first[i, p, 1]
                    low1, high1 = min(low1, value), max(high1, value)
                low2, high2 = np.inf, -np.inf
                for p in range(second.shape[1]):
                    value = nx * second[i, p, 0] + ny * second[i, p, 1]
                    low2, high2 = min(low2, value), max(high2, value)
                best = max(best, max(low2 - high1, low1 - high2))
        out[i] = best
    return out


@njit(cache=True)
def polygons_gap(first, second):
    """Gap between convex polygons per sample, 0 when they touch or overlap."""

    count, m, k = first.shape[0], first.shape[1], second.shape[1]
    out = np.empty(count)
    for i in range(count):
        overlap = False
        for p in range(m):
            if _inside(first[i, p, 0], first[i, p, 1], second[i]):
                overlap = True
                break
        if not overlap:
            for q in range(k):
                if _inside(second[i, q, 0], second[i, q, 1], first[i]):
                    overlap = True
                    break
        if overlap:
            out[i] = 0.0
            continue
        least = np.inf
        for a in range(m):
            b = (a + 1) % m
            p0x, p0y, p1x, p1y = first[i, a, 0], first[i, a, 1], first[i, b, 0], first[i, b, 1]
            for c in range(k):
                d = (c + 1) % k
                q0x, q0y, q1x, q1y = second[i, c, 0], second[i, c, 1], second[i, d, 0], second[i, d, 1]
                if _segments_cross(p0x, p0y, p1x, p1y, q0x, q0y, q1x, q1y):
                    least = 0.0
                    continue
                distance = min(min(_point_segment(p0x, p0y, q0x, q0y, q1x, q1y),
                                   _point_segment(p1x, p1y, q0x, q0y, q1x, q1y)),
                               min(_point_segment(q0x, q0y, p0x, p0y, p1x, p1y),
                                   _point_segment(q1x, q1y, p0x, p0y, p1x, p1y)))
                least = min(least, distance)
        out[i] = least
    return out


@njit(cache=True)
def polygon_to_disk(polygons, centers, radius):
    """Gap from polygons to disks at ``centers`` per sample (negative: overlap)."""

    count, m = polygons.shape[0], polygons.shape[1]
    out = np.empty(count)
    for i in range(count):
        cx, cy = centers[i, 0], centers[i, 1]
        if _inside(cx, cy, polygons[i]):
            out[i] = 0.0 - radius
            continue
        least = np.inf
        for a in range(m):
            b = (a + 1) % m
            least = min(least, _point_segment(cx, cy, polygons[i, a, 0], polygons[i, a, 1],
                                              polygons[i, b, 0], polygons[i, b, 1]))
        out[i] = least - radius
    return out


@njit(cache=True, inline="always")
def _box_distance(x, z, radius, rear):
    dx = max(max(-rear - x, 0.0), x)
    dz = max(abs(z) - radius, 0.0)
    return math.hypot(dx, dz)


@njit(cache=True, inline="always")
def _footprint_segment(sx, sy, ex, ey, radius, rear):
    """Distance from one segment (servo frame) to the D-shaped footprint."""

    disk = max(_point_segment(0.0, 0.0, sx, sy, ex, ey) - radius, 0.0)
    to_box = min(_box_distance(sx, sy, radius, rear), _box_distance(ex, ey, radius, rear))
    cx = (-rear, 0.0, 0.0, -rear)
    cz = (-radius, -radius, radius, radius)
    for k in range(4):
        to_box = min(to_box, _point_segment(cx[k], cz[k], sx, sy, ex, ey))
        n = (k + 1) % 4
        if _segments_cross(sx, sy, ex, ey, cx[k], cz[k], cx[n], cz[n]):
            to_box = 0.0
    return min(disk, to_box)


@njit(cache=True)
def footprint_distance(start, end, pivot_x, pivot_z, fx, fz, sx, sz, radius, rear):
    """Distance from each segment (linkage plane) to a servo footprint: pivot,
    the unit ``forward`` and ``side`` axes, radius and rear extent."""

    out = np.empty(len(start))
    for i in range(len(start)):
        ax, az = start[i, 0] - pivot_x, start[i, 1] - pivot_z
        bx, bz = end[i, 0] - pivot_x, end[i, 1] - pivot_z
        out[i] = _footprint_segment(ax * fx + az * fz, ax * sx + az * sz,
                                    bx * fx + bz * fz, bx * sx + bz * sz, radius, rear)
    return out


@njit(cache=True)
def polygon_to_footprint(polygons, pivot_x, pivot_z, fx, fz, sx, sz, radius, rear):
    """Gap from polygons to a servo footprint per sample (0 when they touch or overlap)."""

    count, m = polygons.shape[0], polygons.shape[1]
    out = np.empty(count)
    for i in range(count):
        if _inside(pivot_x, pivot_z, polygons[i]):
            out[i] = 0.0
            continue
        least = np.inf
        for a in range(m):
            b = (a + 1) % m
            ax, az = polygons[i, a, 0] - pivot_x, polygons[i, a, 1] - pivot_z
            bx, bz = polygons[i, b, 0] - pivot_x, polygons[i, b, 1] - pivot_z
            least = min(least, _footprint_segment(ax * fx + az * fz, ax * sx + az * sz,
                                                  bx * fx + bz * fz, bx * sx + bz * sz, radius, rear))
        out[i] = least
    return out
