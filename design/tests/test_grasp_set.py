"""The grasp-set benchmark: object geometry, the force-closure tests and a known-easy search."""

import math
from pathlib import Path

import numpy as np
import pytest

from core.optimization.thumb_layout.grasp_set import Shape, antipodal, epsilon, rotvec_matrix

HAND = Path(__file__).resolve().parents[1] / "output/sim/study3/hands/cfl_q120_h3to7_h0.yaml"


def test_shapes_closest_points():
    s = Shape("sphere", 20.0)
    d, p, n = s.closest(np.array([[0.0, 0, 15.0]]))
    assert d[0] == pytest.approx(5.0) and p[0] == pytest.approx([0, 0, 10]) and n[0] == pytest.approx([0, 0, 1])
    b = Shape("box", 20.0)
    d, p, n = b.closest(np.array([[13.0, 2.0, 0.0], [8.0, 1.0, 0.0], [12.0, 14.0, 0.0]]))
    assert d == pytest.approx([3.0, -2.0, math.hypot(2, 4)])
    assert n[0] == pytest.approx([1, 0, 0]) and n[1] == pytest.approx([1, 0, 0])
    assert p[1] == pytest.approx([10, 1, 0]) and p[2] == pytest.approx([10, 10, 0])
    c = Shape("cylinder", 20.0, 100.0)
    d, p, n = c.closest(np.array([[0.0, 12.0, 10.0], [0.0, 3.0, 49.0], [0.0, 0.0, 0.0], [13.0, 0.0, 54.0]]))
    assert d == pytest.approx([2.0, -1.0, -10.0, 5.0])
    assert n[0] == pytest.approx([0, 1, 0]) and n[1] == pytest.approx([0, 0, 1])
    assert p[3] == pytest.approx([10, 0, 50]) and n[3] == pytest.approx([0.6, 0, 0.8])
    assert c.rho == pytest.approx(math.hypot(10, 50))


def test_rotvec():
    r = rotvec_matrix(np.array([[0.0, 0.0, math.pi / 2], [0.0, 0.0, 0.0]]))
    assert r[0] @ np.array([1.0, 0, 0]) == pytest.approx([0, 1, 0], abs=1e-12)
    assert r[1] == pytest.approx(np.eye(3))


def _on_sphere(angles_deg, r=10.0):
    a = np.radians(angles_deg)
    p = np.stack((r * np.cos(a), r * np.sin(a), np.zeros_like(a)), axis=1)
    return p, -p / r


def test_two_contacts_antipodal_and_epsilon_agree():
    p, n = _on_sphere([0.0, 180.0])
    assert antipodal(p, n, 0.5) and epsilon(p, n, np.zeros(3), 10.0, 0.5) > 0
    # 2 contacts never close the full 6-D space (no torsional friction): only the 5-D subspace
    p, n = _on_sphere([0.0, 120.0])  # chord 30 deg off each normal: outside the 26.6 deg cone of mu 0.5
    assert not antipodal(p, n, 0.5) and epsilon(p, n, np.zeros(3), 10.0, 0.5) == 0.0
    assert antipodal(p, n, 1.0) and epsilon(p, n, np.zeros(3), 10.0, 1.0) > 0
    rng = np.random.default_rng(0)
    for _ in range(40):  # away from the cone boundary the two tests agree
        a = rng.uniform(90.0, 180.0)
        half = (180.0 - a) / 2.0
        if abs(half - math.degrees(math.atan(0.5))) < 3.0:
            continue
        p, n = _on_sphere([0.0, a])
        assert antipodal(p, n, 0.5) == (epsilon(p, n, np.zeros(3), 10.0, 0.5) > 0)


def test_three_contacts():
    p, n = _on_sphere([0.0, 120.0, 240.0])
    assert epsilon(p, n, np.zeros(3), 10.0, 0.5) > 0
    p, n = _on_sphere([0.0, 30.0, 60.0])  # all on one side
    assert epsilon(p, n, np.zeros(3), 10.0, 0.5) == 0.0
    # friction matters: a box face pair plus one side face, frictionless, cannot resist the third direction's opposite
    p = np.array([[10.0, 0, 0], [-10.0, 0, 0], [0, 10.0, 0]])
    n = np.array([[-1.0, 0, 0], [1.0, 0, 0], [0, -1.0, 0]])
    assert epsilon(p, n, np.zeros(3), 10.0 * math.sqrt(3), 0.5) > 0
    assert epsilon(p, n, np.zeros(3), 10.0 * math.sqrt(3), 1e-3) < 1e-3


@pytest.mark.skipif(not HAND.exists(), reason="O's parameters are not exported here")
def test_small_sphere_pinch_is_found():
    from core.morphology.code_hand import load
    from core.optimization.thumb_layout import ThumbDesign
    from core.optimization.thumb_layout.grasp_set import GraspHand, run_pair

    p, _ = load(HAND)
    hand = GraspHand(p, ThumbDesign.prototype(p))
    r = run_pair(hand, Shape("sphere", 20.0), "pinch_index", n_seeds=12)
    assert r.found["0.5"] and r.eps["0.5"] > 0
    best = r.best["0.5"]
    assert best["max_sdf_mm"] <= 1.0 and best["clearance_mm"] >= -1.0 and best["max_angle_deg"] <= 26.57
    lo, hi = -85.0, 85.0  # thumb RO within its 5 deg margin
    assert lo - 1e-6 <= best["thumb_deg"][0] <= hi + 1e-6
