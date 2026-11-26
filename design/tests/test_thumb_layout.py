"""The thumb-layout evaluator: design derivations, frames against the Drake URDF, and the IK on a known pose."""

import numpy as np
import pytest

from core.morphology.code_hand import load
from core.optimization.thumb_layout import ThumbDesign, hand_parameters, lever_arms
from core.optimization.thumb_layout.evaluator import FingerKinematics, ThumbKinematics, capsule_box_distance, segment_distance

HAND = "output/sim/study3/hands/cfl_q120_h3to7_h0.yaml"


@pytest.fixture(scope="module")
def parameters():
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / HAND
    if not path.exists():
        pytest.skip("O's parameters are not exported here")
    return load(path)[0]


def test_prototype_design_reproduces_the_prototype_thumb(parameters):
    d = ThumbDesign.prototype(parameters)
    assert d.mcp_to_ip_mm == pytest.approx(parameters.thumb.mcp_to_ip_mm)
    assert d.ee_mm == pytest.approx(-parameters.thumb.ee_y_mm)
    assert lever_arms(d) == pytest.approx({"ip": 41.0, "mcp": 76.5})
    flipped = ThumbDesign(d.cmc_x_mm, d.cmc_z_mm, 30.0, "proximal")
    assert flipped.mcp_to_ip_mm == pytest.approx(51.5) and flipped.ee_mm == pytest.approx(40.0)


def test_frames_match_the_drake_urdf(parameters):
    """Values from sim/drake (load_hand.py, O URDF): thumb pad (0.013, 0.041, 0) in thumb_distal, index distal pad
    (-0.0065, 0, -0.006) in index_distal, in the palm frame [mm]."""
    d = ThumbDesign.prototype(parameters)
    h = hand_parameters(parameters, d)
    tk, fk = ThumbKinematics(h, d), FingerKinematics(h, "index")
    pt, nt = tk.pad(tk.frames(np.radians([[-60, 20, 40, 30]])))
    pf, nf = fk.target(fk.frames(np.radians([[10, 50, 60]])), "distal")
    np.testing.assert_allclose(pt[0], [36.603, 9.036, -65.317], atol=2e-3)
    np.testing.assert_allclose(nt[0], [0.3214, -0.5712, 0.7553], atol=2e-4)
    np.testing.assert_allclose(pf[0], [55.007, 7.209, -61.25], atol=2e-3)
    np.testing.assert_allclose(nf[0], [-0.9254, -0.1632, 0.342], atol=2e-4)


def test_proximity():
    a = np.array([[0.0, 0, 0]]), np.array([[10.0, 0, 0]])
    b = np.array([[5.0, 3, -5]]), np.array([[5.0, 3, 5]])
    assert segment_distance(a[0], a[1], b[0], b[1])[0] == pytest.approx(3.0)
    sd = capsule_box_distance(np.array([0.0, 0, 5]), np.array([10.0, 0, 5]), 1.0, np.zeros(3), np.array([20.0, 20, 2]))
    assert sd == pytest.approx(2.0)
