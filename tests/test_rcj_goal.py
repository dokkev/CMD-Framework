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
         "goal.link.finger_length_mm=[99.5, 100]", "goal.link.proximal_length_mm=[40, 60]",
         "goal.link.proximal_length_ratio=[null, null]", "goal.link.distal_length_ratio=[null, null]")
X = (52.0, 9.5, 29.0, 39.5, 1.0, 9.5, 9.5, 0.0)
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
    # The faces (30 and 28 mm), each to where the other's line crosses it folded to 150 deg.
    assert (e.proximal_support_length_mm, e.distal_support_length_mm) == pytest.approx((30.0, 28.0))
    assert e.proximal_effective_mm == pytest.approx(min(30.0, e.proximal_usable_length_mm))
    assert e.distal_effective_mm == pytest.approx(min(28.0, e.distal_usable_length_mm))
    from core.morphology.mechanism.revolute.solids import crossing_free
    from core.optimization.rcj import geometry

    config = load()
    phi = np.asarray(config.goal.servo_samples_rad)
    last = phi[np.flatnonzero(phi >= config.goal.required_rom_rad - 1e-12)[0]]
    free = crossing_free(geometry(X, config), [last])
    assert e.proximal_usable_length_mm == pytest.approx(free["proximal_pad"][0])
    assert e.distal_usable_length_mm == pytest.approx(free["distal_pad"][0])
    assert 0.0 < e.distal_effective_mm < e.proximal_effective_mm < 30.0
    assert 0.0 < math.degrees(e.phi_safe_rad) < 180.0 and e.collision_pair
    report = MECHANISM.report(e)
    assert report["roller_radius_mm"] == 9.5 and report["proximal_depth_mm"] == pytest.approx(14.5)


def test_each_pad_clears_its_own_roller():
    # On B-C (drop 0) from 1 mm past B the distal pad is inside the distal roller.
    inside = MECHANISM.evaluate((*X[:5], 0.0, 9.5, 0.0), load())
    assert inside.constraints["roller_pad_mm"] == pytest.approx(8.5)
    clear = MECHANISM.evaluate((*X[:4], 9.5, 0.0, 9.5, 0.0), load())
    assert clear.constraints["roller_pad_mm"] <= 0.0
    # The proximal pad, ending under the proximal roller, clears it only a radius below O-A.
    assert MECHANISM.evaluate((*X[:3], 50.0, *X[4:6], 5.0, 0.0), load()).constraints["roller_pad_mm"] > 0.0
    # Each drop is at most r.
    assert MECHANISM.evaluate((*X[:5], 10.0, 9.5, 0.0), load()).constraints["pad_drop_mm"] == pytest.approx(0.5)
    assert MECHANISM.evaluate((*X[:6], 10.0, 0.0), load()).constraints["pad_drop_mm"] == pytest.approx(0.5)


def test_k_is_held_against_the_goals_limit():
    assert MECHANISM.evaluate(X, load("goal.hardware.pip_servo.torque_ratio_max=2.0")).constraints["torque_ratio"] == 0.0
    assert MECHANISM.evaluate(X, load("goal.hardware.pip_servo.torque_ratio_max=1.5")).constraints[
        "torque_ratio"] == pytest.approx(0.5)


def test_the_search_coordinates_round_trip():
    config = load()
    assert MECHANISM.to_design(MECHANISM.from_design(X, config), config) == pytest.approx(X)
    lower, upper = MECHANISM.search_box(config)
    assert len(lower) == len(MECHANISM.search_names(config)) == len(upper)


def test_the_servo_tilts_as_the_2rs():
    from core.optimization.rcj import geometry

    config = load("goal.hardware.pip_servo.tilt_angle_deg=45")
    assert config.bounds["servo_tilt_deg"] == (45.0, 45.0)
    x = (*X[:7], 45.0)
    rear = geometry(x, config).servo.rear
    assert rear == pytest.approx((-np.cos(np.radians(45.0)), np.sin(np.radians(45.0))))
    # Searched in whole degrees, the tilt kept through the search's coordinates.
    assert MECHANISM.search_names(config)[-1] == "servo_tilt_deg"
    assert MECHANISM.to_design(MECHANISM.from_design(x, config), config)[-1] == 45.0
    # A row of a run before the tilt reads as untilted.
    row = {name: str(value) for name, value in zip(MECHANISM.variable_names, (*X[:7], 0.0)) if name != "servo_tilt_deg"}
    assert MECHANISM.read_design(row)[-1] == 0.0


def test_the_idealized_rcj_has_no_servo_and_rollers_down_to_the_pin():
    from core.optimization.rcj import geometry

    ideal = mechanism("rcj_idealized")
    config = load_yaml(ideal, CONFIG, STUDY)[0]
    assert not config.joint_servo and config.bounds["servo_tilt_deg"] == (0.0, 0.0)
    assert config.bounds["roller_radius_mm"][0] == config.goal.packaging.pivot_radius_mm("a") == 3.0
    # The pins at A and B fit on the carrier: |AB| = 2r at least their spacing, 6 mm.
    assert 2.0 * config.bounds["roller_radius_mm"][0] >= config.goal.packaging.pin_spacing_mm == 6.0
    # A larger pin at B widens the least roller with it.
    widened = load_yaml(ideal, CONFIG, (*STUDY, "goal.collision.param.pivot_diameter_mm.b=8"))[0]
    assert widened.bounds["roller_radius_mm"][0] == 4.0
    x = (52.0, 4.0, 40.0, 39.5, 1.0, 4.0, 4.0, 0.0)  # |OC| 100, rollers of 4 mm
    assert geometry(x, config).servo is None
    e = ideal.evaluate(x, config)
    assert e.constraints["pip_servo_pad_mm"] == 0.0
    # No servo to keep off the MCP servo: its proximal roller is, 52 mm out, far clear of it.
    from core.optimization.rcj.evaluate import roller_mcp_gap_mm

    assert e.constraints["mcp_servo_mm"] < -30.0
    # Pulled back to |OA| 10 mm, the roller runs into the MCP servo (its 9.5 mm front about O).
    near = geometry((10.0, 4.0, 82.0, 39.5, 1.0, 4.0, 4.0, 0.0), config)
    assert roller_mcp_gap_mm(near, config.goal.packaging, config.goal.mcp_servo_angles_rad) < 0.0
    assert e.torque_ratio_max == 2.0 and e.roller_radius_mm == 4.0
    # The same design with the servo is checked against it; the idealized one has no such pair.
    packaged = MECHANISM.evaluate(x, load())
    assert packaged.constraints["mcp_servo_mm"] != 0.0 or packaged.constraints["pip_servo_pad_mm"] != 0.0
    assert ideal.config_from_json(ideal.config_to_json(config)).joint_servo is False


def test_the_idealized_rcj_is_seen_as_rollers_by_every_analysis():
    from core.morphology.mechanism import base_mechanism
    from core.optimization.analysis.scene import scene
    from core.optimization.mounting import mounting

    ideal = mechanism("rcj_idealized")
    config = load_yaml(ideal, CONFIG, STUDY)[0]
    x = (52.0, 4.0, 40.0, 39.5, 1.0, 4.0, 4.0, 0.0)
    assert base_mechanism("rcj_idealized") == "rcj" and base_mechanism("linkage") == "linkage"
    built = scene("rcj_idealized", config, x, [0.0, 90.0])
    solids = built.solids["proximal_pad"]
    assert len(solids["roller"].disks) == 2 and not solids["servo"].polygons and not solids["pin"].disks
    assert mounting("rcj_idealized", x, config).depths_mm is not None
