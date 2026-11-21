"""The four-bar's closed forms against the linkage model's own kinematics."""

import math

import numpy as np
import pytest

from core.geometry import fourbar
from core.morphology.mechanism.linkage.design import PointXZ, make_finger

REQUIRED = math.radians(120.0)
STEP = math.radians(0.1)


def _random_coordinates(rng, near_limit: bool):
    e, c = rng.uniform(8.0, 30.0, 2)
    delta = rng.uniform(0.0, 2.0 * math.pi)
    low, high = fourbar.centre_distance_range(e, c, delta, REQUIRED)
    if near_limit:
        total, difference = high + rng.uniform(-0.5, 0.5), rng.uniform(-0.9, 0.9) * low
    else:
        total, difference = high + rng.uniform(1.0, 40.0), rng.uniform(-0.95, 0.95) * low
    return 20.0, e, c, delta, total, difference


def _pivots(rng, near_limit: bool, count: int):
    found = []
    while len(found) < count:
        coordinates = _random_coordinates(rng, near_limit)
        branch = int(rng.choice((1, -1)))
        pivots = fourbar.from_closure_coordinates(*coordinates, branch=branch)
        if pivots is not None and pivots[2][0] > pivots[0][0] + 1.0:
            found.append((coordinates, branch, pivots))
    return found


def test_coordinates_and_pivots_are_inverse():
    for coordinates, branch, pivots in _pivots(np.random.default_rng(0), False, 200):
        assert pivots[0][1] == 0.0 and abs(pivots[2][1]) < 1e-12  # A and B on the link line
        assert fourbar.branch_of(*pivots) == branch
        back = fourbar.closure_coordinates(*pivots)
        wrapped = (back[3] - coordinates[3] + math.pi) % (2.0 * math.pi) - math.pi
        assert np.allclose((*back[:3], *back[4:]), (*coordinates[:3], *coordinates[4:]), atol=1e-9)
        assert abs(wrapped) < 1e-9


def test_box_of_the_coordinates_closes_to_the_required_flexion():
    """Inside ``total > D_max`` and ``|difference| < D_min`` the model's own
    sweep closes at every sample up to the required flexion."""

    q = np.arange(0.0, REQUIRED + 1e-12, STEP)
    for _, _, (a, ap, b, bp) in _pivots(np.random.default_rng(1), False, 200):
        finger = make_finger(PointXZ(*a), PointXZ(*ap), PointXZ(*b), PointXZ(*bp), 10.0)
        assert len(finger.sweep_pip(q).q_pip_rad) == len(q)


def test_closure_limit_agrees_with_the_sweep_near_tangency():
    q = np.arange(0.0, math.pi + 1e-12, STEP)
    for _, _, (a, ap, b, bp) in _pivots(np.random.default_rng(2), True, 300):
        finger = make_finger(PointXZ(*a), PointXZ(*ap), PointXZ(*b), PointXZ(*bp), 10.0)
        closing = len(finger.sweep_pip(q).q_pip_rad)
        swept = closing * STEP if closing < len(q) else math.pi  # the first sample that does not close
        shape = fourbar.shape(a, ap, b, bp)
        assert 0.0 <= swept - fourbar.reach(*shape) <= STEP + 1e-9
        assert fourbar.closes(*shape, REQUIRED) == (closing > round(REQUIRED / STEP))


def test_transmission_angle_is_the_models():
    from core.morphology.mechanism.linkage.motion import transmission_angle

    q = np.arange(0.0, REQUIRED + 1e-12, STEP)
    for _, branch, (a, ap, b, bp) in _pivots(np.random.default_rng(3), False, 100):
        finger = make_finger(PointXZ(*a), PointXZ(*ap), PointXZ(*b), PointXZ(*bp), 10.0)
        sweep = finger.sweep_pip(q)
        gamma = fourbar.transmission_angle(*fourbar.shape(a, ap, b, bp), q, branch)
        assert np.allclose(gamma, transmission_angle(np.asarray(a), sweep.bp, sweep.b), atol=1e-7)


def test_centre_distance_range_holds_the_interior_extremes():
    low, high = fourbar.centre_distance_range(10.0, 6.0, -0.5, 2.0)   # delta + q passes 0
    assert low == pytest.approx(4.0)
    low, high = fourbar.centre_distance_range(10.0, 6.0, 2.5, 2.0)    # delta + q passes pi
    assert high == pytest.approx(16.0)
