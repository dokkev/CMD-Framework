"""The RCJ on the linkage study's goal (core.optimization.rcj)."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from core.optimization.mechanism import mechanism
from core.optimization.study import load_yaml

CONFIG = Path(__file__).resolve().parents[1] / "scripts" / "optimize" / "linkage" / "config.yaml"
MECHANISM = mechanism("rcj")
STUDY = ("goal.pads.proximal.length_mm=[0, null]", "goal.pads.distal.length_mm=[0, null]",
         "goal.link.finger_length_mm=[99.5, 100]", "goal.link.proximal_length_mm=[40, 60]")
X = (52.0, 9.5, 29.0, 39.5, 1.0, 9.5, 9.5)
"""|OA| 52, r 9.5, BC 29: the proximal pad 30 mm, the distal 28 mm, both at the rollers' bottom."""


def load(*overrides: str):
    return load_yaml(MECHANISM, CONFIG, (*STUDY, *overrides))[0]


def test_equal_rollers_roll_the_distal_segment_twice_as_far_as_the_carrier():
    from core.optimization.rcj import geometry

    geo = geometry(X, load())
    assert geo.finger_length_mm == pytest.approx(52.0 + 19.0 + 29.0)
    phi = np.radians([0.0, 90.0, 180.0])
    # B on the carrier, turned by phi / 2 about A.
    assert geo.b(phi) == pytest.approx(np.array(((71.0, 0.0), (52.0 + 19.0 / math.sqrt(2), -19.0 / math.sqrt(2)),
                                                 (52.0, -19.0))))
    # C along the distal segment turned by phi; folded, it points back along -X.
    assert geo.tip(phi)[-1] == pytest.approx((52.0 - 29.0, -19.0))
    # The rollers stay in contact: |AB| = 2r throughout.
    assert np.linalg.norm(geo.b(np.linspace(0.0, math.pi, 7)) - geo.a, axis=1) == pytest.approx(19.0)


def test_a_design_is_swept_and_reported():
    e = MECHANISM.evaluate(X, load())
    assert e.torque_ratio_max == 2.0
    # The faces (30 and 28 mm) less what each hides of the other folded to 150 deg.
    assert (e.proximal_face_mm, e.distal_face_mm) == pytest.approx((30.0, 28.0))
    assert e.proximal_effective_mm == pytest.approx(min(30.0, e.proximal_usable_length_mm))
    assert e.distal_effective_mm == pytest.approx(min(28.0, e.distal_usable_length_mm))
    assert 20.0 < e.proximal_effective_mm < 30.0 and 20.0 < e.distal_effective_mm < 28.0
    assert 0.0 < math.degrees(e.phi_safe_rad) < 180.0 and e.collision_pair
    report = MECHANISM.report(e)
    assert report["roller_radius_mm"] == 9.5 and report["proximal_depth_mm"] == pytest.approx(14.5)


def test_each_pad_clears_its_own_roller():
    # On B-C (drop 0) from 1 mm past B the distal pad is inside the distal roller.
    inside = MECHANISM.evaluate((*X[:5], 0.0, 9.5), load())
    assert inside.constraints["roller_pad_mm"] == pytest.approx(8.5)
    clear = MECHANISM.evaluate((*X[:4], 9.5, 0.0, 9.5), load())
    assert clear.constraints["roller_pad_mm"] <= 0.0
    # The proximal pad, ending under the proximal roller, clears it only a radius below O-A.
    assert MECHANISM.evaluate((*X[:3], 50.0, *X[4:6], 5.0), load()).constraints["roller_pad_mm"] > 0.0
    # Each drop is at most r.
    assert MECHANISM.evaluate((*X[:5], 10.0, 9.5), load()).constraints["pad_drop_mm"] == pytest.approx(0.5)
    assert MECHANISM.evaluate((*X[:6], 10.0), load()).constraints["pad_drop_mm"] == pytest.approx(0.5)


def test_k_is_held_against_the_goals_limit():
    assert MECHANISM.evaluate(X, load("goal.hardware.pip_servo.torque_ratio_max=2.0")).constraints["torque_ratio"] == 0.0
    assert MECHANISM.evaluate(X, load("goal.hardware.pip_servo.torque_ratio_max=1.5")).constraints[
        "torque_ratio"] == pytest.approx(0.5)


def test_the_search_coordinates_round_trip():
    config = load()
    assert MECHANISM.to_design(MECHANISM.from_design(X, config), config) == pytest.approx(X)
    lower, upper = MECHANISM.search_box(config)
    assert len(lower) == len(MECHANISM.search_names(config)) == len(upper)
