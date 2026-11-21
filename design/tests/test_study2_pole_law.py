"""Study 0's checks (problem formulation A.6.0): the pole from a fingertip path, the symmetric law, the long-pad
stand-in and the exact clearance of a design's own pads."""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
STUDY2 = ROOT / "scripts" / "study" / "study2_morphology_adaptation"
for path in (ROOT, ROOT / "scripts" / "study", STUDY2):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

spec = importlib.util.spec_from_file_location("study2_pole_law", STUDY2 / "pole_law.py")
law = importlib.util.module_from_spec(spec)
sys.modules["study2_pole_law"] = law
spec.loader.exec_module(law)

Q = np.radians(np.arange(0.0, 150.0 + 1e-9, 0.05))


def _turn(points, q):
    c, s = math.cos(-q), math.sin(-q)
    return np.asarray(points) @ np.array([[c, -s], [s, c]]).T


def revolute_tips(axis_x: float, tip_x: float = 90.0) -> np.ndarray:
    return np.array([np.array((axis_x, 0.0)) + _turn((tip_x - axis_x, 0.0), q) for q in Q])


def rolling_tips(a: float, r: float, tip_x: float = 90.0) -> np.ndarray:
    """Equal rollers of radius ``r`` rolling without slip, the proximal one's centre at ``(a, 0)``."""

    found = []
    for q in Q:
        centre = np.array((a, 0.0)) + 2 * r * np.array((math.cos(q / 2), -math.sin(q / 2)))
        found.append(centre + _turn((tip_x - a - 2 * r, 0.0), q))
    return np.array(found)


def pad(end: float, start: float, t: float, h: float) -> dict:
    return {"proximal_end": end, "distal_start": start, "t1": t, "t2": t, "proximal_top": -t + h, "distal_top": -t + h}


def test_a_revolute_joint_turns_about_its_axis():
    path = law.poles(revolute_tips(48.0), Q)
    assert np.isnan(path[0]).all()  # the extended pose has no pole
    assert path[1:] == pytest.approx(np.tile((48.0, 0.0), (len(Q) - 1, 1)), abs=1e-6)


def test_rolling_rollers_turn_about_a_pole_that_goes_down_as_r_tan_q_over_4():
    a, r = 40.0, 9.5
    path = law.poles(rolling_tips(a, r), Q)
    assert path[1:, 0] == pytest.approx(a + r, abs=1e-6)  # on the rollers' first contact line
    assert -path[1:, 1] == pytest.approx(r * np.tan(Q[1:] / 4), abs=1e-6)


def test_the_gap_about_a_fixed_axis_is_2_t_tan_half_q():
    # Both pads' faces t below the axis: the corners meet on the bisector at s = t tan(q/2) each.
    path = law.poles(revolute_tips(50.0), Q)
    for t, h, required in ((5.0, 5.0, 90.0), (5.0, 5.0, 120.0), (8.0, 3.0, 120.0), (3.0, 3.0, 150.0)):
        expected = 2 * t * math.tan(math.radians(required) / 2)
        shape = pad(50.0 - expected / 2, 50.0 + expected / 2, t, h)
        closed, at = law.symmetric_gap(shape, path, Q, math.radians(required))
        assert closed == pytest.approx(expected, abs=1e-6) and at == pytest.approx(required, abs=0.05)
        assert law.least_gap_long_pads(shape, path, Q, math.radians(required)) == pytest.approx(expected, abs=0.02)


def test_the_gap_about_rollers_takes_the_poles_depth_off_the_faces():
    a, r, t, required = 40.0, 9.5, 13.5, math.radians(120.0)
    path = law.poles(rolling_tips(a, r), Q)
    expected = 2 * (t * math.tan(required / 2) - r * (1 / math.cos(required / 2) - 1))  # about the pole
    shape = pad(a + r - expected / 2, a + r + expected / 2, t, 5.0)
    assert law.symmetric_gap(shape, path, Q, required)[0] == pytest.approx(expected, abs=1e-6)
    assert law.least_gap_long_pads(shape, path, Q, required) == pytest.approx(expected, abs=0.02)


def test_the_pads_first_meet_where_the_law_says():
    path = law.poles(revolute_tips(50.0), Q)
    s, t, h = 8.0, 5.0, 5.0
    proximal = law.rectangle(9.5, 50.0 - s, -t + h, -t)
    distal = law.rectangle(50.0 + s, 90.0, -t + h, -t)
    expected = 180.0 - 2 * math.degrees(math.atan2(t, s))  # atan(t/s) twice is what is left of the half turn
    assert law.first_contact_deg(proximal, distal, path, Q) == pytest.approx(expected, abs=0.06)
    # A pole below the faces: what is above the pole parts, the pads never meet.
    low = np.tile((50.0, -9.0), (len(Q), 1))
    assert math.isnan(law.first_contact_deg(proximal, distal, low, Q))
    assert law.symmetric_gap(pad(42.0, 58.0, t, h), low, Q, math.radians(120.0))[0] == 0.0


def test_a_designs_own_pads_are_judged_by_their_exact_clearance():
    # Pads set at the law's gap about a fixed axis: no room and no overlap, at the required flexion.
    path = law.poles(revolute_tips(50.0), Q)
    t, h, required = 5.0, 5.0, math.radians(120.0)
    s = t * math.tan(required / 2)
    proximal, distal = law.rectangle(9.5, 50.0 - s, -t + h, -t), law.rectangle(50.0 + s, 90.0, -t + h, -t)
    room, at = law.least_clearance(proximal, distal, path, Q, required)
    assert room == pytest.approx(0.0, abs=1e-6) and at == pytest.approx(120.0, abs=0.05)
    # 1 mm more on each side leaves room; 1 mm less overlaps before the required flexion.
    wide = law.least_clearance(law.rectangle(9.5, 49.0 - s, -t + h, -t), law.rectangle(51.0 + s, 90.0, -t + h, -t),
                               path, Q, required)[0]
    tight = law.least_clearance(law.rectangle(9.5, 51.0 - s, -t + h, -t), law.rectangle(49.0 + s, 90.0, -t + h, -t),
                                path, Q, required)[0]
    assert wide > 0.0 > tight


def test_a_rectangle_is_its_centre_and_half_lengths():
    assert law.rectangle(10.0, 30.0, -3.0, -8.0) == ((20.0, -5.5), (10.0, 2.5))


def test_first_contact_on_a_moving_pole_agrees_with_the_shared_polygon_evaluator():
    # Pads of unequal thickness and depth, placed unevenly, about a pole that moves along and across the
    # link as the finger bends: the first overlap by the four-axis clearance is the shared kernel's.
    from core.geometry.pole import turned
    from core.morphology.packaging import shapes

    q = np.radians(np.arange(0.0, 150.0 + 1e-9, 0.25))
    path = np.stack((52.0 - 6.0 * q / q[-1], 5.0 - 11.0 * q / q[-1]), axis=1)  # x 52 -> 46, z +5 -> -6
    proximal = law.rectangle(9.5, 47.0, -4.0, -9.0)   # 5 mm thick, face 9 mm deep
    distal = law.rectangle(50.0, 90.0, -3.0, -6.0)    # 3 mm thick, face 6 mm deep
    found = law.first_contact_deg(proximal, distal, path, q)

    def corners(pad):
        (x, z), (a, b) = pad
        return np.array([(x - a, z - b), (x + a, z - b), (x + a, z + b), (x - a, z + b)])

    expected = math.nan
    for angle, about in zip(q[1:], path[1:]):
        moved = turned(corners(distal), about, angle)
        if float(shapes.separation(corners(proximal)[None], moved[None])[0]) < -1e-9:
            expected = math.degrees(angle)
            break
    assert math.isfinite(expected) and found == pytest.approx(expected)
    room, at = law.least_clearance(proximal, distal, path, q, math.radians(expected - 1.0))
    assert room >= -1e-9 and at <= expected - 1.0  # clear up to just before it


def test_the_coverage_from_the_pole_matches_the_closed_form_and_uses_the_structures_ends():
    # Symmetric pads about a pole of fixed depth: proposition 3's closed form. Structures that end
    # nearer the joint than the pads (a pivot housing) change the posed length, and so the coverage.
    length, start, t, z, required = 90.0, 9.5, 9.0, 3.0, math.radians(120.0)
    gap = 2 * (t - z) * math.tan(required / 2)
    centre = start + (length - start) / 2
    path = np.tile((centre, -z), (len(Q), 1)).astype(float)
    shape = pad(centre - gap / 2, centre + gap / 2, t, 5.0)
    closed = np.mean([(length - start - gap) / (length - gap * (1 - math.cos(a / 2)) + 2 * z * math.sin(a / 2))
                      for a in (math.pi / 2, (math.pi / 2 + required) / 2, required)])
    assert law.coverage_law(shape, start, length, path, Q, required) == pytest.approx(closed, abs=1e-9)
    nearer = law.coverage_law(shape, start, length, path, Q, required, (centre - gap / 2 + 2.0, centre + gap / 2 - 2.0))
    assert abs(nearer - closed) > 1e-3
    # The distal line 0.2 mm lower in the extended pose (a four-bar closed on rounded link lengths) changes it.
    lower = law.coverage_law(shape, start, length, path, Q, required, None, (length, -0.2), (1.0, 0.0))
    assert lower != pytest.approx(closed, abs=1e-6)
