"""The compiled geometry kernels match the array expressions they replaced."""

from __future__ import annotations

import numpy as np
import pytest

from core.morphology.packaging import shapes as P
from core.morphology.packaging.shapes import ServoFootprint


def _edges(polygons):
    return polygons, np.roll(polygons, -1, axis=-2)


def ref_point_segment(point, start, end):
    direction = end - start
    length_squared = np.sum(direction * direction, axis=1)
    safe = np.where(length_squared > 0.0, length_squared, 1.0)
    t = np.where(length_squared > 0.0, np.sum((point - start) * direction, axis=1) / safe, 0.0)
    nearest = start + np.clip(t, 0.0, 1.0)[:, None] * direction
    return np.linalg.norm(point - nearest, axis=1)


def ref_cross(start, end, q0, q1):
    def cross(o, p, q):
        return (p[..., 0] - o[..., 0]) * (q[..., 1] - o[..., 1]) - (p[..., 1] - o[..., 1]) * (q[..., 0] - o[..., 0])

    d1, d2 = cross(start, end, q0), cross(start, end, q1)
    d3, d4 = cross(q0, q1, start), cross(q0, q1, end)
    return (d1 * d2 <= 0.0) & (d3 * d4 <= 0.0) & ~((d1 == 0.0) & (d2 == 0.0))


def ref_inside(points, polygons):
    start, end = _edges(polygons)
    edge = (end - start)[:, None]
    offset = points[:, :, None] - start[:, None]
    cross = edge[..., 0] * offset[..., 1] - edge[..., 1] * offset[..., 0]
    return np.all(cross >= 0.0, axis=2) | np.all(cross <= 0.0, axis=2)


def ref_separation(first, second):
    best = np.full(len(first), -np.inf)
    for polygon in (first, second):
        start, end = _edges(polygon)
        edge = end - start
        normal = np.stack((edge[..., 1], -edge[..., 0]), axis=-1)
        length = np.linalg.norm(normal, axis=-1)
        normal = normal / np.where(length > 0.0, length, 1.0)[..., None]
        on_first = np.einsum("nad,npd->nap", normal, first)
        on_second = np.einsum("nad,npd->nap", normal, second)
        apart = np.maximum(on_second.min(axis=2) - on_first.max(axis=2), on_first.min(axis=2) - on_second.max(axis=2))
        best = np.maximum(best, np.where(length > 0.0, apart, -np.inf).max(axis=1))
    return best


def ref_polygons_gap(first, second):
    count = len(first)
    (p0, p1), (q0, q1) = _edges(first), _edges(second)
    p0, p1 = (a[:, :, None] for a in (p0, p1))
    q0, q1 = (a[:, None] for a in (q0, q1))
    shape = np.broadcast_shapes(p0.shape, q0.shape)
    p0, p1, q0, q1 = (np.broadcast_to(a, shape).reshape(-1, 2) for a in (p0, p1, q0, q1))
    distance = np.minimum.reduce((ref_point_segment(p0, q0, q1), ref_point_segment(p1, q0, q1),
                                  ref_point_segment(q0, p0, p1), ref_point_segment(q1, p0, p1)))
    distance = np.where(ref_cross(p0, p1, q0, q1), 0.0, distance).reshape(count, -1).min(axis=1)
    overlap = ref_inside(first, second).any(axis=1) | ref_inside(second, first).any(axis=1)
    return np.where(overlap, 0.0, distance)


def ref_footprint(footprint, start, end):
    forward = -np.array(footprint.rear)
    side = np.array((-forward[1], forward[0]))

    def local(points):
        delta = points - np.array(footprint.pivot)
        return np.stack((delta @ forward, delta @ side), axis=-1)

    def box(points):
        dx = np.maximum.reduce((-footprint.rear_extent_mm - points[:, 0], np.zeros(len(points)), points[:, 0]))
        return np.hypot(dx, np.maximum(np.abs(points[:, 1]) - footprint.radius_mm, 0.0))

    start, end = local(start), local(end)
    r, rear = footprint.radius_mm, footprint.rear_extent_mm
    disk = np.maximum(ref_point_segment(np.zeros(2), start, end) - r, 0.0)
    corners = np.array(((-rear, -r), (0.0, -r), (0.0, r), (-rear, r)))
    to_box = np.minimum(box(start), box(end))
    for k in range(4):
        to_box = np.minimum(to_box, ref_point_segment(corners[k], start, end))
        to_box = np.where(ref_cross(start, end, corners[k], corners[(k + 1) % 4]), 0.0, to_box)
    return np.minimum(disk, to_box)


def blocks(rng, count):
    """Random rotated rectangles, some chamfered (5 vertices), near each other."""

    width, height = rng.uniform(5, 40, count), rng.uniform(2, 10, count)
    centre = rng.uniform(-20, 20, (count, 2))
    angle = rng.uniform(-np.pi, np.pi, count)
    local = np.stack([np.stack(v, axis=-1) for v in (
        (-width / 2, -height / 2), (width / 2, -height / 2), (width / 2, height / 2), (-width / 2, height / 2))], axis=1)
    c, s = np.cos(angle)[:, None], np.sin(angle)[:, None]
    return np.stack((c * local[..., 0] - s * local[..., 1], s * local[..., 0] + c * local[..., 1]), axis=-1) + centre[:, None]


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_kernels_match_the_array_forms(seed):
    rng = np.random.default_rng(seed)
    first, second = blocks(rng, 2000), blocks(rng, 2000)
    start, end = rng.uniform(-40, 40, (2000, 2)), rng.uniform(-40, 40, (2000, 2))
    end[:50] = start[:50]  # degenerate segments
    point = rng.uniform(-40, 40, (2000, 2))
    tolerance = dict(rel=0.0, abs=1e-12)
    assert P.point_segment_distance(point, start, end) == pytest.approx(ref_point_segment(point, start, end), **tolerance)
    assert np.array_equal(P.segments_cross(start, end, first[:, 0], first[:, 1]),
                          ref_cross(start, end, first[:, 0], first[:, 1]))
    assert np.array_equal(P.points_inside(second, first), ref_inside(second, first))
    assert P.separation(first, second) == pytest.approx(ref_separation(first, second), **tolerance)
    assert P.polygons_gap(first, second) == pytest.approx(ref_polygons_gap(first, second), **tolerance)
    radius = 3.0
    expected = np.where(ref_inside(point[:, None], first)[:, 0], 0.0,
                        ref_point_segment(np.repeat(point, 4, axis=0), *(a.reshape(-1, 2) for a in _edges(first)))
                        .reshape(-1, 4).min(axis=1)) - radius
    assert P.polygon_to_disk(first, point, radius) == pytest.approx(expected, **tolerance)
    footprint = ServoFootprint((5.0, 3.0), (-np.cos(0.3), np.sin(0.3)), 9.5, 25.5)
    assert footprint.distance_to_segments(start, end) == pytest.approx(ref_footprint(footprint, start, end), **tolerance)
    edges = [a.reshape(-1, 2) for a in _edges(first)]
    expected = np.where(ref_inside(np.broadcast_to(footprint.pivot, (2000, 1, 2)), first)[:, 0], 0.0,
                        ref_footprint(footprint, *edges).reshape(2000, -1).min(axis=1))
    assert P.polygon_to_footprint(first, footprint) == pytest.approx(expected, **tolerance)
