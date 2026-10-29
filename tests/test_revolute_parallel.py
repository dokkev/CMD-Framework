"""The serial 2R driven through a belt from a servo on A' (core.optimization.revolute_parallel)."""

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
         "goal.motion.dip_flexion_deg=110", "goal.hardware.pulley_radius_mm=6")
X = (50.0, 50.0, 39.5, 25.0, 3.0, 3.0, 20.0, 5.0, 15.0)
"""|OA| = |AC| = 50, pads 30 + 25 mm 3 mm below their lines, the servo tilted
20 deg on A', 15 mm above and 5 past A."""


def load(*overrides: str):
    return load_yaml(MECHANISM, CONFIG, (*STUDY, *overrides))[0]


def test_the_belt_turns_the_joint_by_the_servo_angle():
    from core.optimization.revolute_parallel import geometry

    config = load()
    geo = geometry(X, config)
    assert geo.servo.pivot == (55.0, 15.0)
    rear = np.asarray(geo.servo.rear)
    assert rear == pytest.approx((-math.cos(math.radians(20.0)), math.sin(math.radians(20.0))))
    # The pulleys (6 mm) run in the outer layer: the pads keep off the pin's housing (3 mm) only.
    assert config.joint_radius_mm == 3.0 and config.pulley_radius_mm == 6.0
    e = MECHANISM.evaluate(X, config)
    assert e.feasible and e.torque_ratio_max == 1.0


def test_no_dead_centre_limits_the_flexion():
    # The parallelogram stopped at 120 deg (transmission 30-150 deg); the belt does not.
    e = MECHANISM.evaluate(X, load("goal.motion.dip_flexion_deg=130"))
    assert e.stop_reason != "transmission" and math.degrees(e.phi_safe_rad) > 120.0


def test_the_tilt_is_searched_within_the_goals_range():
    config = load("goal.hardware.pip_servo.tilt_angle_deg=[0, 90]")
    assert config.bounds["servo_tilt_deg"] == (0.0, 90.0)
    assert config.base.bounds["servo_tilt_deg"] == (0.0, 90.0)
    lower, upper = MECHANISM.search_box(config)
    index = MECHANISM.search_names(config).index("servo_tilt_deg")
    assert (lower[index], upper[index]) == (0.0, 90.0)


def test_the_joint_pin_pulleys_and_carrier_are_checked():
    config = load()
    # A' 10 mm behind A, 3 above: the servo's body over A's pin.
    over = MECHANISM.evaluate((*X[:7], -10.0, 3.0), config)
    assert over.constraints["a_pin_servo_mm"] > 0.0
    # The distal pad 2 mm past A: inside the pin's housing.
    near = MECHANISM.evaluate((*X[:3], 2.0, *X[4:]), config)
    assert near.constraints["distal_support_mm"] > 0.0
    # The distal pad's carrier is checked against the servo, as the linkage's.
    from core.optimization.revolute_parallel import geometry
    from core.optimization.revolute_parallel.evaluate import _margins

    margins = _margins(geometry(X, config), np.radians([0.0, 60.0]), config)
    assert ("distal_carrier", "pip_servo") in margins
    assert not any("pulley" in name for pair in margins for name in pair)  # outer layer


def test_the_search_coordinates_round_trip():
    config = load("goal.hardware.pip_servo.tilt_angle_deg=[0, 90]")
    assert MECHANISM.to_design(MECHANISM.from_design(X, config), config) == pytest.approx(X)
    lower, upper = MECHANISM.search_box(config)
    assert len(lower) == len(upper) == len(MECHANISM.search_names(config)) == 9
    assert MECHANISM.config_from_json(MECHANISM.config_to_json(config)) == config


def test_a_belt_needs_its_pulleys_size():
    with pytest.raises(ValueError, match="pulley_radius_mm"):
        load_yaml(MECHANISM, CONFIG, STUDY[:-1])
