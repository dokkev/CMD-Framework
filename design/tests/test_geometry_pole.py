"""The finite pole and the exact clearance of two rectangular pads (problem formulation A.6.0)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from core.geometry.pole import pole, rectangular_clearance, rotation, symmetric_gap_law, turned
from core.morphology.packaging import shapes


def corners(center, half) -> np.ndarray:
    (x, z), (a, b) = center, half
    return np.array([(x - a, z - b), (x + a, z - b), (x + a, z + b), (x - a, z + b)], dtype=float)


def polygons(center_a, half_a, center_b0, half_b, about, q):
    return corners(center_a, half_a), turned(corners(center_b0, half_b), about, q)


def test_the_pole_is_the_fixed_point_of_the_displacement():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q, about, start = rng.uniform(0.02, math.pi), rng.normal(size=2) * 30, rng.normal(size=2) * 40
        assert pole(q, turned(start, about, q), start) == pytest.approx(about, abs=1e-8)
    assert np.isnan(pole(0.0, (1.0, 2.0), (1.0, 2.0))).all()  # the extended pose has none


def test_the_pole_tends_to_the_instantaneous_centre_as_q_goes_to_0():
    # Equal rollers of radius r, the proximal one's centre at (a, 0): the pole is (a + r, -r tan(q/4)),
    # and its limit is the rollers' contact point, their instantaneous centre.
    a, r = 40.0, 9.5
    for q in (1e-2, 1e-3, 1e-4):
        centre = np.array((a, 0.0)) + 2 * r * np.array((math.cos(q / 2), -math.sin(q / 2)))
        tip = centre + rotation(q) @ np.array((90.0 - a - 2 * r, 0.0))
        found = pole(q, tip, (90.0, 0.0))
        assert found == pytest.approx((a + r, -r * math.tan(q / 4)), abs=1e-5)
        assert np.linalg.norm(found - np.array((a + r, 0.0))) < r * q  # shrinking onto the contact point


def test_the_exact_clearance_agrees_with_the_shared_polygon_evaluator():
    # Unequal depths, unequal lengths, a pole anywhere: the four-axis value is the shared kernel's.
    rng = np.random.default_rng(1)
    for _ in range(400):
        a = (rng.uniform(15, 35), rng.uniform(-14, -2)), (rng.uniform(3, 18), rng.uniform(1, 5))
        b = (rng.uniform(55, 80), rng.uniform(-14, -2)), (rng.uniform(3, 18), rng.uniform(1, 5))
        about, q = (rng.uniform(35, 65), rng.uniform(-12, 8)), rng.uniform(0.0, math.radians(160))
        value = rectangular_clearance(a[0], a[1], b[0], b[1], about, q)
        first, second = polygons(a[0], a[1], b[0], b[1], about, q)
        assert value == pytest.approx(float(shapes.separation(first[None], second[None])[0]), abs=1e-9)
        gap = float(shapes.polygons_gap(first[None], second[None])[0])
        assert (gap > 1e-9) == (value > 1e-9) or abs(value) < 1e-6  # apart for one, apart for the other


def test_the_pads_lengths_enter_the_exact_clearance():
    # The same joint-side corners (6 mm either side of the axis, faces 5 mm deep). At 150 deg two long pads
    # overlap; two short ones have gone past each other and are apart: at one pose, finite pads are not the
    # long pads of the symmetric law. (On the way there the short ones did overlap.)
    about, q = (48.0, 0.0), math.radians(150.0)
    long_pads = ((30.0, -4.0), (12.0, 1.0)), ((66.0, -4.0), (12.0, 1.0))
    short_pads = ((41.0, -4.0), (1.0, 1.0)), ((55.0, -4.0), (1.0, 1.0))
    assert rectangular_clearance(*long_pads[0], *long_pads[1], about, q) < 0.0
    assert rectangular_clearance(*short_pads[0], *short_pads[1], about, q) > 0.0
    on_the_way = min(rectangular_clearance(*short_pads[0], *short_pads[1], about, math.radians(v))
                     for v in np.arange(1.0, 150.0, 0.5))
    assert on_the_way < 0.0


def test_the_symmetric_law_is_the_least_gap_where_its_conditions_hold():
    # Equal depths, the pole midway and fixed in x, its depth changing with q: the gap the law gives
    # is clear over the whole way from the extended pose, and any less overlaps somewhere.
    t, h, centre, required = 8.0, 5.0, 50.0, math.radians(120.0)
    q = np.radians(np.arange(0.0, 120.0 + 1e-9, 0.25))
    depth = -5.0 + 12.0 * q / required  # dorsal at first, 7 mm palmar at the end
    gap, at = symmetric_gap_law(t, depth, q, required)
    assert 0.0 < at < required  # set inside the range, not at its end
    for scale, clear in ((1.0, True), (0.97, False)):
        s = scale * gap / 2
        a, b = ((centre - s - 15.0, -t + h / 2), (15.0, h / 2)), ((centre + s + 15.0, -t + h / 2), (15.0, h / 2))
        least = min(rectangular_clearance(*a, *b, (centre, -z), angle) for z, angle in zip(depth[1:], q[1:]))
        assert (least >= -1e-9) == clear


def test_the_symmetric_law_is_not_the_least_gap_of_pads_placed_unevenly_about_the_pole():
    # The law is the least gap of pads placed evenly about the pole. With the pole off the centre the same
    # gap is no longer on the boundary; and pads placed unevenly can do with less: at 90 deg about a fixed
    # axis, one pad against the axis and the other t away are clear with a gap of t, half the law's 2 t.
    t, h, centre, required = 6.0, 4.0, 50.0, math.radians(120.0)
    q = np.radians(np.arange(0.0, 120.0 + 1e-9, 0.25))
    gap, _ = symmetric_gap_law(t, np.zeros(len(q)), q, required)
    a = ((centre - gap / 2 - 15.0, -t + h / 2), (15.0, h / 2))
    b = ((centre + gap / 2 + 15.0, -t + h / 2), (15.0, h / 2))
    on_centre = min(rectangular_clearance(*a, *b, (centre, 0.0), angle) for angle in q[1:])
    shifted = min(rectangular_clearance(*a, *b, (centre + 3.0, 0.0), angle) for angle in q[1:])
    assert on_centre == pytest.approx(0.0, abs=1e-9) and abs(shifted) > 0.1
    quarter = np.radians(np.arange(0.25, 90.0 + 1e-9, 0.25))
    a = ((50.0 - t - 15.0, -t + h / 2), (15.0, h / 2))      # its corner t behind the axis
    b = ((50.0 + 15.0, -t + h / 2), (15.0, h / 2))          # its corner under the axis
    assert min(rectangular_clearance(*a, *b, (50.0, 0.0), angle) for angle in quarter) >= -1e-9
    assert symmetric_gap_law(t, np.zeros(len(quarter)), quarter, math.pi / 2)[0] == pytest.approx(2 * t)


def test_unequal_depths_are_judged_exactly_not_by_the_law():
    # Faces 4 and 10 mm deep about a fixed axis: with the corners equally far (both 7 tan 60 off the axis,
    # what the law gives for the mean depth) the deep pad's corner strikes the shallow pad's end face first.
    h, required = 3.0, math.radians(120.0)
    q = np.radians(np.arange(0.0, 120.0 + 1e-9, 0.25))
    s = 7.0 * math.tan(required / 2)
    a = ((50.0 - s - 15.0, -4.0 + h / 2), (15.0, h / 2))
    b = ((50.0 + s + 15.0, -10.0 + h / 2), (15.0, h / 2))
    least = min(rectangular_clearance(*a, *b, (50.0, 0.0), angle) for angle in q[1:])
    assert least != pytest.approx(0.0, abs=1e-3)  # the mean-depth law does not sit on the exact boundary here


def test_rollers_can_set_the_gap_before_the_required_flexion():
    # z = r tan(q/4): (t - z) tan(q/2) rises with q only while t >= r sin(q/2).
    r, required = 12.0, math.radians(150.0)
    q = np.radians(np.arange(0.0, 150.0 + 1e-9, 0.05))
    depth = r * np.tan(q / 4)
    deep = symmetric_gap_law(r * math.sin(required / 2) + 0.5, depth, q, required)
    shallow = symmetric_gap_law(6.0, depth, q, required)  # 6 < 12 sin 75 deg
    assert deep[1] == pytest.approx(required, abs=1e-3)
    assert shallow[1] == pytest.approx(2 * math.asin(6.0 / r), abs=2e-3) and shallow[1] < required
