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
CONFIG = FingerStudyConfig(bounds=BOUNDS, bc_length_mm=24.0)
CROSSED = (40.0, 70.0, 52.0, 15.0, 62.0, 15.0)


def test_design_vector_round_trips() -> None:
    assert encode(FingerLinkage(decode(CROSSED, 24.0))) == CROSSED


def test_curl_pose_turns_the_distal_link_half_a_turn() -> None:
    evaluation = evaluate(decode(CROSSED, 24.0), CONFIG)

    assert math.isfinite(evaluation.curl_pip_rad)
    assert evaluation.curl_pip_rad + evaluation.curl_dip_rad == pytest.approx(math.pi, abs=1e-3)
    assert evaluation.constraints["curl_reached_deg"] == 0.0
    assert evaluation.ab_mm == pytest.approx(30.0)
    assert 0.0 < evaluation.worst_transmission_quality <= 1.0


def test_open_four_bar_is_invalid_with_a_graded_violation() -> None:
    open_bar = (40.0, 70.0, 62.0, 15.0, 52.0, 15.0)
    evaluation = evaluate(decode(open_bar, 24.0), CONFIG)

    assert evaluation.constraints["valid_reference"] > 0.0
    assert not evaluation.feasible
    assert set(evaluation.constraints) == set(CONSTRAINT_NAMES)


def test_curl_gap_is_held_to_its_target() -> None:
    gap = evaluate(decode(CROSSED, 24.0), CONFIG).curl_gap_mm
    for target in (gap - 2.0, gap + 2.0):
        config = FingerStudyConfig(
            bounds=BOUNDS, bc_length_mm=24.0, curl_gap_mm=target, curl_gap_tolerance_mm=0.5
        )
        assert evaluate(decode(CROSSED, 24.0), config).constraints["curl_gap_mm"] == pytest.approx(1.5)


def test_link_clearance_is_reported_but_optional() -> None:
    off = evaluate(decode(CROSSED, 24.0), CONFIG)
    on = evaluate(decode(CROSSED, 24.0), FingerStudyConfig(
        bounds=BOUNDS, bc_length_mm=24.0, link_clearance=True))

    assert off.constraints["link_mm"] == 0.0
    assert on.constraints["link_mm"] == pytest.approx(-on.link_clearance_mm)
    assert off.link_clearance_mm == pytest.approx(on.link_clearance_mm)


def test_transmission_is_the_angle_at_b_prime_over_the_widened_range() -> None:
    parameters = decode(CROSSED, 24.0)
    evaluation = evaluate(parameters, CONFIG)
    # The evaluated range is extension to curl, widened by 10 deg at both ends.
    end = math.degrees(evaluation.curl_pip_rad) + 10.0
    sweep = FingerLinkage(parameters).sweep_pip(
        [math.radians(d) for d in range(-10, 181) if d <= end + 1e-9])

    assert evaluation.worst_transmission_quality == pytest.approx(
        math.sin(sweep.margin_at_bp_rad.min()))


def test_oa_interface_keeps_the_servo_dorsal_of_oa() -> None:
    # The servo on A' = (52, 15) reaches down to z = 15 - 9.5.
    assert evaluate(decode(CROSSED, 24.0), CONFIG).constraints["oa_interface_mm"] == pytest.approx(
        -(15.0 - 9.5))
    padded = FingerStudyConfig(bounds=BOUNDS, bc_length_mm=24.0, oa_interface_clearance_mm=8.0)
    assert evaluate(decode(CROSSED, 24.0), padded).constraints["oa_interface_mm"] == pytest.approx(2.5)


def test_curl_gap_is_the_distance_between_the_oa_and_bc_lines() -> None:
    evaluation = evaluate(decode(CROSSED, 24.0), CONFIG)
    pose = FingerLinkage(decode(CROSSED, 24.0)).solve(0.0, evaluation.curl_pip_rad)

    assert pose.c.z_mm == pytest.approx(pose.b.z_mm, abs=1e-3)
    assert evaluation.curl_gap_mm == pytest.approx(-pose.b.z_mm, abs=1e-3)
