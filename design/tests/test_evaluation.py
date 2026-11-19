"""The shared evaluation quantities (core.evaluation): cylinder coverage, motion reach, MA."""

import math

import numpy as np
import pytest

from core.evaluation import (
    MotionSamples,
    common_length_mm,
    evaluation_poses_rad,
    gamma_violation_deg,
    pad_coverage,
    posed_finger_length_mm,
    reach_rad,
    uniform_samples_rad,
    worst_mechanical_advantage,
)
from core.geometry.intervals import IntervalSet

U2 = np.array((math.cos(math.radians(-60.0)), math.sin(math.radians(-60.0))))
PROXIMAL = np.array(((-40.0, 0.0), (-10.0, 0.0)))  # outer end first; covers t in [10, 40] from X = 0
DISTAL = np.array((30.0 * U2, 5.0 * U2))  # covers t in [5, 30]


def test_common_length_is_the_overlap_of_both_rays():
    assert common_length_mm(PROXIMAL, DISTAL) == pytest.approx(20.0)


def test_a_longer_pad_without_a_partner_adds_nothing():
    longer = np.array(((-90.0, 0.0), (-10.0, 0.0)))
    assert common_length_mm(longer, DISTAL) == pytest.approx(20.0)


def test_pad_past_the_apex_counts_only_its_open_side():
    past = np.array(((-40.0, 0.0), (8.0, 0.0)))  # runs past X to t = -8
    assert common_length_mm(past, DISTAL) == pytest.approx(25.0)


def test_hidden_part_is_removed_before_the_intersection():
    near_inner = (IntervalSet.closed(25.0, 30.0), IntervalSet())  # t in [10, 15] on the proximal face
    assert common_length_mm(PROXIMAL, DISTAL, near_inner) == pytest.approx(15.0)
    near_outer = (IntervalSet.closed(0.0, 5.0), IntervalSet())  # t in [35, 40]: outside T anyway
    assert common_length_mm(PROXIMAL, DISTAL, near_outer) == pytest.approx(20.0)


def test_parallel_faces_share_nothing():
    other = np.array(((40.0, -5.0), (10.0, -5.0)))
    assert common_length_mm(PROXIMAL, other) == 0.0


def test_coverage_uses_the_posed_finger_length():
    lf = posed_finger_length_mm(40.0, 30.0, (40.0, 0.0), (43.0, -4.0))
    assert lf == pytest.approx(75.0)
    assert pad_coverage(20.0, lf) == pytest.approx(40.0 / 75.0)


def test_evaluation_poses():
    assert np.degrees(evaluation_poses_rad(math.radians(150))) == pytest.approx((90.0, 120.0, 150.0))
    assert np.degrees(evaluation_poses_rad(math.radians(90))) == pytest.approx((90.0,))
    with pytest.raises(ValueError):
        evaluation_poses_rad(math.radians(60))


def test_reach_stops_before_the_first_bad_sample():
    q = uniform_samples_rad(math.radians(10), math.radians(1))
    assert len(q) == 11
    ok = np.ones(11, dtype=bool)
    assert reach_rad(q, ok) == pytest.approx(math.radians(10))
    ok[4] = False
    assert reach_rad(q, ok) == pytest.approx(math.radians(3))
    ok[0] = False
    assert reach_rad(q, ok) == 0.0


def test_motion_samples_constraints_and_mechanical_advantage():
    q = uniform_samples_rad(math.radians(120), math.radians(1))
    gamma = np.full(len(q), math.radians(90))
    gamma[100:] = math.radians(20)  # out of range from 100 deg
    motion = MotionSamples(q, np.ones(len(q), dtype=bool), 2.0 * q, gamma, (math.radians(30), math.radians(150)))
    assert motion.f_motion_violation_deg(math.radians(120)) == 0.0
    assert math.degrees(motion.q_max_rad()) == pytest.approx(99.0)
    assert motion.gamma_violation_deg(math.radians(90)) == 0.0
    assert motion.gamma_violation_deg(math.radians(120)) == pytest.approx(10.0)
    assert gamma_violation_deg(np.array([]), (0.0, 1.0)) == 0.0
    assert worst_mechanical_advantage(motion, math.radians(120)) == pytest.approx(2.0)
    short = MotionSamples(q[:50], np.ones(50, dtype=bool), 2.0 * q[:50])
    assert short.f_motion_violation_deg(math.radians(120)) == pytest.approx(71.0)
    assert worst_mechanical_advantage(short, math.radians(120)) == 0.0


def test_a_collision_between_two_clear_samples_is_found_on_the_finer_grid():
    from core.evaluation import resample_between

    q = np.radians([0.0, 1.0, 2.0])
    coarse = {("a", "b"): np.array([0.4, 0.3, 0.4])}
    # The true gap dips below 0 between 1 and 2 deg; the samples miss it.
    true_gap = lambda inner, pairs: {("a", "b"): 0.3 - 2.0 * np.sin(np.pi * (np.degrees(inner) - 1.0))}  # noqa: E731
    found = resample_between(q, coarse, np.array([2.0, 2.0]), np.radians(0.1), true_gap)
    assert found[("a", "b")][2] < 0.0 and found[("a", "b")][1] == pytest.approx(0.3)
    # A step that moves too little to close the gaps is not resampled.
    assert resample_between(q, coarse, np.array([0.1, 0.1]), np.radians(0.1), true_gap)[("a", "b")][2] == 0.4


def test_only_the_pairs_that_could_collide_are_resampled():
    from core.evaluation import resample_between

    q = np.radians([0.0, 1.0, 2.0])
    coarse = {("a", "b"): np.array([0.4, 0.3, 0.4]), ("c", "d"): np.array([5.0, 5.0, 5.0])}
    asked = []

    def gaps(inner, pairs):
        asked.append(set(pairs))
        return {("a", "b"): 0.3 - 2.0 * np.sin(np.pi * (np.degrees(inner) - 1.0))}

    found = resample_between(q, coarse, np.array([2.0, 2.0]), np.radians(0.1), gaps)
    # (c, d)'s gaps sum to 10 mm against a 2 mm travel: it cannot collide in between.
    assert asked == [{("a", "b")}] and found[("a", "b")][2] < 0.0 and found[("c", "d")][2] == 5.0
