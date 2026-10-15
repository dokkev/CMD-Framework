import math

import pytest

from cmd.morphology.linkage import FingerLinkage
from cmd.optimization.finger import (
    CONSTRAINT_NAMES,
    VARIABLE_NAMES,
    FingerStudyConfig,
    decode,
    encode,
    evaluate,
)

BOUNDS = {name: (0.0, 200.0) for name in VARIABLE_NAMES}
CONFIG = FingerStudyConfig(bounds=BOUNDS)
CROSSED = (40.0, 70.0, 0.0, 52.0, 15.0, 62.0, 15.0, 0.0)


def test_design_vector_round_trips() -> None:
    assert encode(FingerLinkage(decode((*CROSSED, 24.0)))) == (*CROSSED, 24.0)


def test_curl_pose_turns_the_distal_link_half_a_turn() -> None:
    evaluation = evaluate(decode((*CROSSED, 24.0)), CONFIG)

    assert math.isfinite(evaluation.curl_pip_rad)
    assert evaluation.curl_pip_rad + evaluation.curl_dip_rad == pytest.approx(math.pi, abs=1e-3)
    assert evaluation.constraints["curl_reached_deg"] == 0.0
    assert evaluation.ab_mm == pytest.approx(30.0)
    assert 0.0 < evaluation.worst_transmission_quality <= 1.0


def test_open_four_bar_is_evaluated_and_its_dip_direction_decides() -> None:
    # This open four-bar extends the DIP as the PIP flexes.
    open_bar = (40.0, 70.0, 0.0, 62.0, 15.0, 52.0, 15.0, 0.0)
    evaluation = evaluate(decode((*open_bar, 24.0)), CONFIG)

    assert evaluation.constraints["valid_reference"] == 0.0
    assert set(evaluation.constraints) == set(CONSTRAINT_NAMES)
    assert not evaluation.feasible


def test_curl_gap_is_a_minimum() -> None:
    gap = evaluate(decode((*CROSSED, 24.0)), CONFIG).curl_gap_mm
    for least, violation in ((gap - 2.0, -2.0), (gap + 2.0, 2.0)):
        config = FingerStudyConfig(bounds=BOUNDS, curl_gap_mm=least)
        assert evaluate(decode((*CROSSED, 24.0)), config).constraints["curl_gap_mm"] == pytest.approx(violation)


def test_link_clearance_is_reported_but_optional() -> None:
    off = evaluate(decode((*CROSSED, 24.0)), CONFIG)
    on = evaluate(decode((*CROSSED, 24.0)), FingerStudyConfig(
        bounds=BOUNDS, link_clearance=True))

    assert off.constraints["link_mm"] == 0.0
    assert on.constraints["link_mm"] == pytest.approx(-on.link_clearance_mm)
    assert off.link_clearance_mm == pytest.approx(on.link_clearance_mm)


def test_transmission_is_the_angle_at_b_prime_over_the_widened_range() -> None:
    parameters = decode((*CROSSED, 24.0))
    evaluation = evaluate(parameters, CONFIG)
    # The evaluated range is extension to curl, widened by 10 deg at both ends.
    end = math.degrees(evaluation.curl_pip_rad) + 10.0
    sweep = FingerLinkage(parameters).sweep_pip(
        [math.radians(d) for d in range(-10, 181) if d <= end + 1e-9])

    assert evaluation.worst_transmission_quality == pytest.approx(
        math.sin(sweep.margin_at_bp_rad.min()))


def test_oa_interface_keeps_the_servo_dorsal_of_oa() -> None:
    # The servo on A' = (52, 15) reaches down to z = 15 - 9.5.
    assert evaluate(decode((*CROSSED, 24.0)), CONFIG).constraints["oa_interface_mm"] == pytest.approx(
        -(15.0 - 9.5))
    padded = FingerStudyConfig(bounds=BOUNDS, oa_interface_clearance_mm=8.0)
    assert evaluate(decode((*CROSSED, 24.0)), padded).constraints["oa_interface_mm"] == pytest.approx(2.5)


def test_curl_gap_is_the_distance_between_the_oa_and_bc_lines() -> None:
    evaluation = evaluate(decode((*CROSSED, 24.0)), CONFIG)
    pose = FingerLinkage(decode((*CROSSED, 24.0))).solve(0.0, evaluation.curl_pip_rad)

    assert pose.c.z_mm == pytest.approx(pose.b.z_mm, abs=1e-3)
    assert evaluation.curl_gap_mm == pytest.approx(-pose.b.z_mm, abs=1e-3)


def test_bc_may_sit_off_the_oa_line_and_ab_is_a_distance() -> None:
    lowered = (40.0, 70.0, -4.0, 52.0, 15.0, 62.0, 15.0, 0.0)
    linkage = FingerLinkage(decode((*lowered, 24.0)))
    evaluation = evaluate(decode((*lowered, 24.0)), CONFIG)

    assert (linkage.b.z_mm, linkage.c.z_mm) == (-4.0, -4.0)
    assert evaluation.ab_mm == pytest.approx(math.hypot(30.0, 4.0))


def test_extended_length_is_the_distance_oc() -> None:
    lowered = decode((40.0, 70.0, -10.0, 52.0, 15.0, 62.0, 15.0, 0.0, 24.0))
    oc = math.hypot(70.0 + 24.0, -10.0)
    config = FingerStudyConfig(bounds=BOUNDS, extended_length_bounds_mm=(90.0, 94.0))

    assert evaluate(lowered, config).constraints["extended_length_mm"] == pytest.approx(oc - 94.0)


def test_fingertip_must_clear_the_mcp_servo() -> None:
    # A short proximal link curls the 24 mm fingertip into the MCP servo.
    short = (32.8, 75.0, 9.5, 39.3, 13.5, 57.2, 18.8, 11.9)
    long = (50.0, 72.0, 0.0, 60.0, 15.0, 66.0, 15.0, 0.0)

    assert evaluate(decode((*short, 24.0)), CONFIG).constraints["fingertip_mcp_mm"] > 0.0
    assert evaluate(decode((*long, 24.0)), CONFIG).constraints["fingertip_mcp_mm"] < 0.0


def test_static_failures_skip_the_sweep() -> None:
    from cmd.optimization.finger.evaluate import MOTION_CONSTRAINTS, UNSWEPT

    too_long = (60.0, 100.0, 0.0, 72.0, 15.0, 82.0, 15.0, 0.0)
    evaluation = evaluate(decode((*too_long, 24.0)), CONFIG)

    assert evaluation.constraints["extended_length_mm"] > 0.0
    assert all(evaluation.constraints[name] == UNSWEPT for name in MOTION_CONSTRAINTS)


def test_search_bounds_close_the_oa_interface_constraint() -> None:
    # Valid while the servo only tilts dorsally, by at most 90 deg.
    bounds = {**BOUNDS, "servo_tilt_deg": (0.0, 45.0)}
    config = FingerStudyConfig(bounds=bounds, oa_interface_clearance_mm=2.0)
    radius = config.packaging.pip_servo.radius_mm

    assert config.search_bounds["ap_z_mm"] == (radius + 2.0, BOUNDS["ap_z_mm"][1])
    assert config.bounds["ap_z_mm"] == BOUNDS["ap_z_mm"]
    assert FingerStudyConfig(bounds=BOUNDS).search_bounds["ap_z_mm"] == BOUNDS["ap_z_mm"]


def test_servo_lever_matches_a_finite_difference_of_virtual_work() -> None:
    from cmd.optimization.finger.evaluate import servo_lever

    parameters = decode((*CROSSED, 24.0))
    linkage = FingerLinkage(parameters)
    q = [math.radians(d) for d in range(0, 61)]
    sweep = linkage.sweep_pip(q)
    lever, ratio, travel = servo_lever(parameters, sweep, slice(None))

    # Recompute r at the tip C at one pose from two nearby poses.
    def state(q_pip):
        pose = linkage.solve(0.0, q_pip)
        theta = math.atan2(pose.b.z_mm - parameters.ap.z_mm, pose.b.x_mm - parameters.ap.x_mm)
        return pose, theta
    (p0, t0), (p1, t1) = state(math.radians(30.0)), state(math.radians(30.01))
    u = ((p0.c.x_mm - p0.b.x_mm) / 24.0, (p0.c.z_mm - p0.b.z_mm) / 24.0)
    n = (u[1], -u[0])
    r_c = abs(n[0] * (p1.c.x_mm - p0.c.x_mm) + n[1] * (p1.c.z_mm - p0.c.z_mm)) / abs(t1 - t0)

    assert lever >= r_c * 0.999
    assert 0.0 < travel < 2.0 * math.pi
    # On average the distal link turns (its whole rotation) / (servo travel) per servo radian.
    phi = sweep.total_flexion_rad
    assert ratio >= abs(phi[-1] - phi[0]) / travel * 0.999
    evaluation = evaluate(parameters, CONFIG)
    assert math.isfinite(evaluation.lever_max_mm)


def test_curl_tip_stays_ahead_of_o() -> None:
    evaluation = evaluate(decode((*CROSSED, 24.0)), CONFIG)
    tip = evaluation.curl_tip_x_mm
    config = FingerStudyConfig(bounds=BOUNDS, curl_tip_min_x_mm=tip + 3.0)

    assert evaluation.constraints["curl_tip_x_mm"] == pytest.approx(-tip)
    assert evaluate(decode((*CROSSED, 24.0)), config).constraints["curl_tip_x_mm"] == pytest.approx(3.0)


def test_reference_point_b_makes_length_and_curl_tip_independent_of_bc() -> None:
    lengths = []
    tips = []
    for bc in (24.0, 34.0):
        config = FingerStudyConfig(bounds=BOUNDS, reference_point="B",
                                   extended_length_bounds_mm=(0.0, 100.0))
        evaluation = evaluate(decode((*CROSSED, bc)), config)
        lengths.append(evaluation.constraints["extended_length_mm"])
        tips.append(evaluation.curl_tip_x_mm)
    assert lengths[0] == pytest.approx(lengths[1])
    assert tips[0] == pytest.approx(tips[1])
