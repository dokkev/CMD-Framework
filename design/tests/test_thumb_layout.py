"""The thumb-layout evaluator: design derivations, frames against the Drake URDF, and the IK on a known pose."""

import numpy as np
import pytest

from core.morphology.code_hand import load
from core.optimization.thumb_layout import ThumbDesign, hand_parameters, lever_arms, palm_arch
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


def test_palm_arch(parameters):
    """The arch turns the middle and ring bases about H's x axis through the index base; the index stays."""
    assert palm_arch(parameters, 0.0) is parameters
    arched = palm_arch(parameters, 10.0)
    lay, new = parameters.layout, arched.layout
    assert new.index == lay.index
    pivot = np.array([lay.index.y_mm, lay.index.z_mm])
    for k, d in ((1, "middle"), (2, "ring")):
        a, b = lay.base(d), new.base(d)
        assert b.x_mm == pytest.approx(a.x_mm)
        r0, r1 = np.array([a.y_mm, a.z_mm]) - pivot, np.array([b.y_mm, b.z_mm]) - pivot
        assert np.linalg.norm(r1) == pytest.approx(np.linalg.norm(r0))
        assert np.degrees(np.arctan2(r1[1], r1[0]) - np.arctan2(r0[1], r0[0])) == pytest.approx(10.0 * k)
        assert b.z_mm < a.z_mm  # palmar


def test_double_support_plates_tighten_the_raa_sweep(parameters):
    from core.optimization.thumb_layout import Evaluator, ThumbDesign
    d = ThumbDesign(-50.5, -10.5, 34.0, "proximal", 0, -16, 0)
    h = hand_parameters(parameters, d)
    plain = Evaluator(parameters).envelope(h)["raa_sweep_clearance_mm"]
    plates = Evaluator(parameters, double_support=True).envelope(h)["raa_sweep_clearance_mm"]
    assert plain == pytest.approx(1.46, abs=0.05)  # the earlier, servo-only value
    assert plates < plain - 3.0  # the far plate reaches 3.5 mm closer to the abduction servos


def test_proximal_bend(parameters):
    """A zero bend is the straight phalanx; a bend keeps |MCP-IP| at the chord of the two halves and splits the
    proximal capsule in two."""
    import math
    from core.optimization.thumb_layout import ThumbDesign
    from core.optimization.thumb_layout.design import BEND_FROM_MCP_MM
    straight = ThumbDesign(-50.5, -10.5, 34.0, "proximal", 0, -16, 0)
    bent = ThumbDesign(-50.5, -10.5, 34.0, "proximal", 0, -16, 0, proximal_bend_deg=-20.0)
    q = np.radians([[-40.0, 20.0, 30.0, 10.0]])
    ks, kb = ThumbKinematics(hand_parameters(parameters, straight), straight), ThumbKinematics(hand_parameters(parameters, bent), bent)
    fs, fb = ks.frames(q), kb.frames(q)
    np.testing.assert_allclose(fs["mcp"], fb["mcp"])
    l1, l = BEND_FROM_MCP_MM, straight.mcp_to_ip_mm
    chord = math.sqrt(l1**2 + (l - l1)**2 + 2 * l1 * (l - l1) * math.cos(math.radians(20.0)))
    assert np.linalg.norm(fb["ip"][0, :3, 3] - fb["mcp"][0, :3, 3]) == pytest.approx(chord)
    assert len(kb.radii) == 6 and kb.capsules(fb).shape[1] == 6 and ks.capsules(fs).shape[1] == 5
    # the IP frame turns by the bend about the flexion axis
    rel = np.linalg.inv(fs["ip"][0]) @ fb["ip"][0]
    assert math.degrees(math.atan2(rel[1, 0], rel[0, 0])) == pytest.approx(-20.0)


def test_base_pitch_is_ro_clocking(parameters):
    """The base pitch offset turns the CMC frame about the RO axis itself (the servo's z): the axis stays put, so a
    pitch offset is the same chain as the RO joint shifted by it. Roll and yaw are what tilt the RO axis."""
    q = np.radians([[-40.0, 20.0, 30.0, 10.0]])
    pitched = ThumbDesign(-55.0, -10.5, 33.0, "proximal", 0.0, -16.0, 0.0)
    flat = ThumbDesign(-55.0, -10.5, 33.0, "proximal")
    kp = ThumbKinematics(hand_parameters(parameters, pitched), pitched)
    kf = ThumbKinematics(hand_parameters(parameters, flat), flat)
    np.testing.assert_allclose(kp.base[:3, 2], kf.base[:3, 2], atol=1e-12)
    shifted = q + np.radians([[-16.0, 0.0, 0.0, 0.0]])
    for name in ("raa", "mcp", "ip"):
        np.testing.assert_allclose(kp.frames(q)[name], kf.frames(shifted)[name], atol=1e-9)
    rolled = ThumbDesign(-55.0, -10.5, 33.0, "proximal", 0.0, 0.0, 10.0)
    kr = ThumbKinematics(hand_parameters(parameters, rolled), rolled)
    assert np.degrees(np.arcsin(kr.base[2, 2])) == pytest.approx(10.0, abs=1e-6)
