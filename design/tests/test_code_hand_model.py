"""The CoDe parametric model on its own: geometric zero, sign conventions, actuator coordinates (no CAD)."""

from dataclasses import replace

import numpy as np
import pytest

from core.morphology.code_hand import (
    FINGERS,
    CoDeHand,
    FingerConfiguration,
    FingerParameters,
    HandConfiguration,
    HandParameters,
    ThumbConfiguration,
    four_bar,
    load,
    pip_from_servo,
    pip_servo_deg,
)

PARAMETERS, _ = load()
MODEL = CoDeHand(PARAMETERS)
POSED = HandConfiguration(
    index=FingerConfiguration(5.0, 30.0, 40.0),
    middle=FingerConfiguration(0.0, 60.0, 80.0),
    ring=FingerConfiguration(-5.0, -10.0, 0.0),
    thumb=ThumbConfiguration(-20.0, 15.0, 45.0, 30.0),
)


def _posed(digit: str, joint: str, deg: float, base: HandConfiguration | None = None) -> HandConfiguration:
    base = base or HandConfiguration()
    return replace(base, **{digit: replace(getattr(base, digit), **{f"{joint}_deg": deg})})


def test_geometric_zero_has_a_straight_thumb():
    frames = MODEL.frames()
    points = [frames.servos[f"thumb/{joint}"][:3, 3] for joint in ("raa", "mcp", "ip")]
    points.append(frames.points["thumb/tip"])
    direction = frames.servos["thumb/raa"][:3, 2]
    for first, second in zip(points, points[1:]):
        assert np.cross(second - first, direction) == pytest.approx(np.zeros(3), abs=1e-10)
        assert (second - first) @ direction > 0.0


JOINTS = [(digit, joint, child) for digit in FINGERS
          for joint, child in (("aa", "fe"), ("fe", "pip"), ("pip", None))]
JOINTS += [("thumb", joint, child) for joint, child in
           (("ro", "raa"), ("raa", "mcp"), ("mcp", "ip"), ("ip", None))]


@pytest.mark.parametrize("digit,joint,child", JOINTS)
def test_positive_joint_increment_is_ccw_about_local_positive_z(digit, joint, child):
    before, after = MODEL.frames(), MODEL.frames(_posed(digit, joint, 0.001))
    if joint == "pip":
        # Distal orientation, rather than actuator rotation or a fixed-pivot assumption.
        vector = before.points[f"{digit}/C"] - before.axes[f"{digit}/B"][0]
        moved = after.points[f"{digit}/C"] - after.axes[f"{digit}/B"][0]
        axis = before.servos[f"{digit}/fe"][:3, 2]
    elif joint == "ip":
        vector = before.points["thumb/tip"] - before.servos["thumb/ip"][:3, 3]
        moved = after.points["thumb/tip"] - after.servos["thumb/ip"][:3, 3]
        axis = before.axes["thumb/ip"][1]
    else:
        vector = before.servos[f"{digit}/{child}"][:3, 0]
        moved = after.servos[f"{digit}/{child}"][:3, 0]
        axis = before.axes[f"{digit}/{joint}"][1]
    assert (moved - vector) @ np.cross(axis, vector) > 0.0


@pytest.mark.parametrize("digit,joint", [(d, j) for d in FINGERS for j in ("fe", "pip")])
def test_finger_flexion_is_positive(digit, joint):
    # Palmar is the finger frame's -Z, which is the hand frame's -Z for the prototype's level fingers.
    palmar = -(PARAMETERS.layout.base(digit).matrix() @ np.array([0.0, 0.0, 1.0, 0.0]))[:3]
    tip = lambda q: MODEL.frames(q).points[f"{digit}/C"]
    assert (tip(_posed(digit, joint, 1.0)) - tip(HandConfiguration())) @ palmar > 0.0


@pytest.mark.parametrize("joint", ["mcp", "ip"])
def test_thumb_flexion_is_positive(joint):
    # The thumb's pad faces the IP servo frame's +X (the CAD Thumb EE connector sits on that side).
    zero = MODEL.frames()
    pad = zero.servos["thumb/ip"][:3, 0]
    moved = MODEL.frames(_posed("thumb", joint, 1.0)).points["thumb/tip"] - zero.points["thumb/tip"]
    assert moved @ pad > 0.0


@pytest.mark.parametrize("pip_deg", [-10.0, 0.0, 20.0, 45.0, 70.0])
def test_four_bar_keeps_its_link_lengths(pip_deg):
    p = FingerParameters()
    pins = four_bar(p, pip_deg)
    assert np.linalg.norm(pins.b - pins.ap) == pytest.approx(p.apb_mm)
    assert np.linalg.norm(pins.bp - pins.a) == pytest.approx(p.abp_mm)
    assert np.linalg.norm(pins.bp - pins.b) == pytest.approx(np.hypot(p.bp_x_mm, p.bp_z_mm))
    assert np.linalg.norm(pins.c - pins.b) == pytest.approx(p.c_x_mm)


def test_layout_change_moves_only_that_digit():
    data = PARAMETERS.to_dict()
    moved = HandParameters.from_dict({**data, "layout": {**data["layout"], "ring": {**data["layout"]["ring"],
                                                                                     "y_mm": -60.0}}})
    before = CoDeHand(PARAMETERS).frames(POSED).servos
    after = CoDeHand(moved).frames(POSED).servos
    for key in before:
        shift = after[key][:3, 3] - before[key][:3, 3]
        expected = [0.0, -60.0 - PARAMETERS.layout.ring.y_mm, 0.0] if key.startswith("ring/") else [0.0] * 3
        assert shift == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize("pip_deg", [-10.0, 0.0, 30.0, 60.0, 90.0])
def test_pip_servo_angle_inverts(pip_deg):
    f = PARAMETERS.finger
    assert pip_from_servo(f, pip_servo_deg(f, pip_deg)) == pytest.approx(pip_deg, abs=1e-9)


def test_pip_servo_turns_with_flexion():
    f = PARAMETERS.finger
    angles = [pip_servo_deg(f, q) for q in np.linspace(-10.0, 90.0, 21)]
    assert np.all(np.diff(angles) > 0.0)


def test_pip_servo_angle_is_the_link_direction_in_the_servo_frame():
    frames = MODEL.frames(POSED)
    for digit in FINGERS:
        servo = frames.servos[f"{digit}/pip"]
        link = frames.axes[f"{digit}/B"][0] - frames.axes[f"{digit}/Ap"][0]
        local = servo[:3, :3].T @ link
        assert local[2] == pytest.approx(0.0, abs=1e-9)
        expected = pip_servo_deg(PARAMETERS.finger, getattr(POSED, digit).pip_deg)
        assert np.degrees(np.arctan2(local[1], local[0])) == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize("configuration", [HandConfiguration(), POSED])
def test_actuators_round_trip(configuration):
    back = MODEL.configuration_from_actuators(MODEL.actuators(configuration))
    for digit, joints in configuration.to_dict().items():
        assert back.to_dict()[digit] == pytest.approx(joints, abs=1e-9)


def test_actuators_are_the_joint_angles_from_their_zero():
    a = MODEL.actuators(POSED)
    f, t = PARAMETERS.finger, PARAMETERS.thumb
    assert a["index/fe"] == pytest.approx(POSED.index.fe_deg - f.fe_actuator_zero_deg)
    assert a["thumb/mcp"] == pytest.approx(POSED.thumb.mcp_deg - t.mcp_actuator_zero_deg)
    # A turn of one servo moves only its own actuator coordinate.
    turned = MODEL.actuators(_posed("thumb", "raa", POSED.thumb.raa_deg + 7.0, POSED))
    assert {k: v - a[k] for k, v in turned.items() if abs(v - a[k]) > 1e-12} == {"thumb/raa": pytest.approx(7.0)}
