"""The serial 2R driven through a parallel four-bar (core.optimization.revolute_parallel)."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from core.optimization.mechanism import mechanism
from core.optimization.study import load_yaml

CONFIG = Path(__file__).resolve().parents[1] / "scripts" / "optimize" / "linkage" / "config.yaml"
MECHANISM = mechanism("revolute_parallel")
STUDY = ("goal.pads.proximal.length_mm=[0, null]", "goal.pads.distal.length_mm=[20, null]",
         "goal.link.finger_length_mm=[99.5, 100]", "goal.link.proximal_length_mm=[40, 60]",
         "goal.motion.dip_flexion_deg=110", "goal.hardware.pip_servo.tilt_angle_deg=20")
X = (50.0, 50.0, 39.5, 25.0, 3.0, 3.0, 0.0, 5.0, 15.0, 12.0, 40.0)
"""|OA| = |AC| = 50, pads 30 + 25 mm, A' 15 mm above and 5 past A, cranks 12 mm at 40 deg."""


def load(*overrides: str):
    return load_yaml(MECHANISM, CONFIG, (*STUDY, *overrides))[0]


def test_the_parallelogram_turns_the_joint_by_the_servo_angle():
    from core.optimization.revolute_parallel import geometry

    geo = geometry(X, load())
    phi = np.radians(np.linspace(0.0, 120.0, 7))
    # The coupler Q-R stays parallel and equal to A'-A: a parallelogram, k = 1.
    assert geo.r(phi) - geo.q(phi) == pytest.approx(np.broadcast_to(geo.a - geo.ap, (7, 2)))
    # The distal crank turns with the distal segment: fixed in its frame.
    e = np.stack((np.cos(phi), -np.sin(phi)), axis=1)
    assert np.sum((geo.r(phi) - geo.a) * e, axis=1) == pytest.approx(12.0 * math.cos(math.radians(40.0)))
    assert geo.servo.pivot == (55.0, 15.0)
    assert MECHANISM.evaluate(X, load()).torque_ratio_max == 1.0


def test_the_transmission_angle_ends_the_motion():
    config = load()
    e = MECHANISM.evaluate(X, config)
    assert e.feasible and e.min_transmission_deg >= 30.0 - 1e-9
    # The cranks turn 120 deg between the 30 and 150 deg transmission angles at most.
    far = MECHANISM.evaluate(X, load("goal.motion.dip_flexion_deg=130"))
    assert far.stop_reason == "transmission" and math.degrees(far.phi_safe_rad) < 120.0
    wide = MECHANISM.evaluate(X, load("goal.motion.dip_flexion_deg=130", "goal.link.transmission_angle_deg=[10, 170]"))
    assert wide.phi_safe_rad > far.phi_safe_rad


def test_a_link_through_a_pad_shortens_its_usable_face():
    from core.optimization.revolute_parallel import geometry
    from core.optimization.revolute_parallel.evaluate import usable_pad_lengths

    config = load()
    clear = usable_pad_lengths(geometry(X, config), np.zeros(1), config)
    assert (clear["proximal_pad"][0], clear["distal_pad"][0]) == pytest.approx((30.0, 25.0))
    # A long distal crank back and palmar from A: A-R runs through the proximal
    # pad's block (x up to 39.5, 3 to 8 mm below O-A), whose face stops there.
    down = (*X[:9], 25.0, -150.0)
    crossed = usable_pad_lengths(geometry(down, config), np.zeros(1), config)
    assert 0.0 < crossed["proximal_pad"][0] < 30.0
    assert crossed["distal_pad"][0] == pytest.approx(25.0)


def test_the_search_coordinates_round_trip():
    config = load()
    assert MECHANISM.to_design(MECHANISM.from_design(X, config), config) == pytest.approx(X)
    lower, upper = MECHANISM.search_box(config)
    assert len(lower) == len(upper) == len(MECHANISM.search_names(config)) == 11
    assert MECHANISM.config_from_json(MECHANISM.config_to_json(config)) == config
