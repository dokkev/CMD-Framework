import dataclasses
import math

import numpy as np
import pytest

from core.morphology.linkage import FingerLinkage
from core.morphology.linkage.packaging import ContactPad, Packaging
from core.optimization.linkage import (
    CONSTRAINT_NAMES,
    VARIABLE_NAMES,
    FingerStudyConfig,
    decode,
    encode,
    evaluate,
)

BOUNDS = {name: (0.0, 200.0) for name in VARIABLE_NAMES}
# The designs below predate the bearing and the pads: test the evaluation with
# the earlier 6.5 mm pin envelope and thin (1 mm) pads, which leave their
# motion to the linkage (tests/test_support_structure.py covers the pads).
HOUSING_ONLY = Packaging(pivot_clearance_diameter_mm=6.5,
                         proximal_pad=ContactPad(height_mm=1.0, chamfer_mm=0.0),
                         distal_pad=ContactPad(height_mm=1.0, chamfer_mm=0.0))
CONFIG = FingerStudyConfig(bounds=BOUNDS, packaging=HOUSING_ONLY)
CROSSED = (40.0, 70.0, 0.0, 52.0, 15.0, 62.0, 15.0, 0.0)
# A feasible linkage design (BC 30 mm) that curls to 180 deg safely.
GOOD = (36.0, 79.3, 10.0, 41.6, 12.6, 63.3, 17.4, 41.9, 30.0, 0.0)
GOOD_CONFIG = FingerStudyConfig(bounds=BOUNDS, packaging=HOUSING_ONLY, extended_length_bounds_mm=(90.0, 110.0))


def test_design_vector_round_trips() -> None:
    assert encode(FingerLinkage(decode((*CROSSED, 24.0, 0.0, 3.5, 2.0)))) == (*CROSSED, 24.0, 0.0, 3.5, 2.0)
    # A vector from before A''_x - A'_x and B'_x - B''_x reads with both right below A', B'.
    assert encode(FingerLinkage(decode((*CROSSED, 24.0, 0.0)))) == (*CROSSED, 24.0, 0.0, 0.0, 0.0)


def test_safe_range_ends_at_the_curl_pose_when_it_comes_first() -> None:
    from core.optimization.linkage import MECHANISM

    evaluation = evaluate(decode(GOOD), GOOD_CONFIG)

    assert evaluation.feasible
    assert evaluation.stop_reason == "curl"
    assert evaluation.phi_safe_rad == math.pi
    assert evaluation.usable_pip_rad == pytest.approx(evaluation.curl_pip_rad)
    assert evaluation.curl_pip_rad + evaluation.curl_dip_rad == pytest.approx(math.pi, abs=1e-3)
    # The pads' a-b: A'_x less the proximal pad's start, and B_x + BC - B'_x.
    start = GOOD_CONFIG.packaging.proximal_pad_start_mm
    assert MECHANISM.objective_values(evaluation, ("transmission", "support_length", "curl")) == pytest.approx(
        (evaluation.torque_ratio_max, -(41.6 - start + (79.3 + 30.0 - 63.3)), 0.0), abs=0.01)


def test_curl_needs_its_stopper_margin_inside_the_sweep() -> None:
    full = evaluate(decode(GOOD), GOOD_CONFIG)
    curl = full.curl_pip_rad  # about 104 deg of PIP
    # Sweep the servo only a little past the curl: less than 10 deg of PIP past it.
    short = FingerStudyConfig(bounds=BOUNDS, packaging=HOUSING_ONLY, extended_length_bounds_mm=(90.0, 110.0),
                              servo_horizon_rad=full.usable_servo_rad + math.radians(3.0))
    evaluation = evaluate(decode(GOOD), short)

    assert evaluation.stop_reason == "horizon"
    assert evaluation.usable_pip_rad < curl
    assert evaluation.phi_safe_rad < math.pi


def test_safe_range_stops_a_stopper_margin_before_the_first_unsafe_pose() -> None:
    strict = FingerStudyConfig(bounds=BOUNDS, packaging=HOUSING_ONLY, extended_length_bounds_mm=(90.0, 110.0),
                               minimum_transmission_margin_rad=math.radians(50.0))
    evaluation = evaluate(decode(GOOD), strict)
    # The servo flexes the finger turning positive; sweep it on the same 1 deg grid.
    sweep = FingerLinkage(decode(GOOD)).sweep_servo([math.radians(d) for d in range(0, 181)])
    first = next(i for i, m in enumerate(sweep.margin_at_bp_rad) if m < math.radians(50.0))
    usable = sweep.q_pip_rad[first] - math.radians(10.0)

    assert evaluation.stop_reason == "transmission"
    assert evaluation.usable_pip_rad == pytest.approx(max(usable, 0.0))
    assert evaluation.constraints["minimum_flexion_deg"] == pytest.approx(
        120.0 - math.degrees(evaluation.phi_safe_rad))


def test_open_four_bar_is_evaluated_and_its_dip_direction_decides() -> None:
    # This open four-bar extends the DIP as the PIP flexes.
    open_bar = (40.0, 70.0, 0.0, 62.0, 15.0, 52.0, 15.0, 0.0)
    evaluation = evaluate(decode((*open_bar, 24.0, 0.0)), CONFIG)

    assert evaluation.constraints["valid_reference"] == 0.0
    assert set(evaluation.constraints) == set(CONSTRAINT_NAMES)
    assert not evaluation.feasible


def test_middle_link_servo_clearance_is_reported_but_optional() -> None:
    off = evaluate(decode(GOOD), GOOD_CONFIG)
    on = evaluate(decode(GOOD), FingerStudyConfig(
        bounds=BOUNDS, packaging=HOUSING_ONLY, extended_length_bounds_mm=(90.0, 110.0),
        require_middle_link_servo_clearance=True))

    assert off.constraints["middle_link_servo_mm"] == 0.0
    assert on.constraints["middle_link_servo_mm"] == pytest.approx(-on.middle_link_servo_clearance_mm)
    assert off.middle_link_servo_clearance_mm == pytest.approx(on.middle_link_servo_clearance_mm)


def test_transmission_is_the_angle_at_b_prime_from_hyperextension_to_the_usable_end() -> None:
    parameters = decode(GOOD)
    evaluation = evaluate(parameters, GOOD_CONFIG)
    end = math.degrees(evaluation.usable_pip_rad)
    sweep = FingerLinkage(parameters).sweep_pip(np.radians(np.arange(-10.0, end, 0.01)))

    # The evaluation samples the servo every 1 deg: equal up to that sampling.
    assert evaluation.worst_transmission_quality == pytest.approx(
        math.sin(sweep.margin_at_bp_rad.min()), abs=2e-3)


def test_oa_interface_keeps_the_servo_dorsal_of_oa() -> None:
    # The servo on A' = (52, 15) reaches down to z = 15 - 9.5.
    assert evaluate(decode((*CROSSED, 24.0, 0.0)), CONFIG).constraints["oa_interface_mm"] == pytest.approx(
        -(15.0 - 9.5))


def test_bc_may_sit_off_the_oa_line_and_ab_is_a_distance() -> None:
    lowered = (40.0, 70.0, -4.0, 52.0, 15.0, 62.0, 15.0, 0.0)
    linkage = FingerLinkage(decode((*lowered, 24.0, 0.0)))
    evaluation = evaluate(decode((*lowered, 24.0, 0.0)), CONFIG)

    assert (linkage.b.z_mm, linkage.c.z_mm) == (-4.0, -4.0)
    assert evaluation.ab_mm == pytest.approx(math.hypot(30.0, 4.0))


def test_extended_length_is_the_distance_oc() -> None:
    lowered = decode((40.0, 70.0, -10.0, 52.0, 15.0, 62.0, 15.0, 0.0, 24.0, 0.0))
    oc = math.hypot(70.0 + 24.0, -10.0)
    config = FingerStudyConfig(bounds=BOUNDS, packaging=HOUSING_ONLY, extended_length_bounds_mm=(90.0, 94.0))

    assert evaluate(lowered, config).constraints["extended_length_mm"] == pytest.approx(oc - 94.0)


def test_fingertip_mcp_clearance_is_held_unless_enabled() -> None:
    # Over its usable range this short finger keeps the fingertip 3.5 mm
    # (beyond the 1 mm body clearance) off the MCP servo.
    short = (32.8, 75.0, 9.5, 39.3, 13.5, 57.2, 18.8, 11.9)
    enforced = FingerStudyConfig(bounds=BOUNDS, packaging=HOUSING_ONLY, enforce_fingertip_mcp=True)

    assert evaluate(decode((*short, 24.0, 0.0)), CONFIG).constraints["fingertip_mcp_mm"] == 0.0
    assert evaluate(decode((*short, 24.0, 0.0)), enforced).constraints["fingertip_mcp_mm"] == pytest.approx(
        -3.47, abs=0.05)


def test_static_failures_skip_the_sweep() -> None:
    from core.optimization.linkage.evaluate import MOTION_CONSTRAINTS, UNSWEPT

    too_long = (60.0, 100.0, 0.0, 72.0, 15.0, 82.0, 15.0, 0.0)
    evaluation = evaluate(decode((*too_long, 24.0, 0.0)), CONFIG)

    assert evaluation.constraints["extended_length_mm"] > 0.0
    assert all(evaluation.constraints[name] == UNSWEPT for name in MOTION_CONSTRAINTS)


def test_search_bounds_close_the_oa_interface_constraint() -> None:
    # Valid while the servo only tilts dorsally, by at most 90 deg.
    bounds = {**BOUNDS, "servo_tilt_deg": (0.0, 45.0)}
    config = FingerStudyConfig(bounds=bounds)
    radius = config.packaging.pip_servo.radius_mm

    assert config.search_bounds["ap_z_mm"] == (radius, BOUNDS["ap_z_mm"][1])
    assert config.bounds["ap_z_mm"] == BOUNDS["ap_z_mm"]
    assert FingerStudyConfig(bounds=BOUNDS, packaging=HOUSING_ONLY).search_bounds["ap_z_mm"] == BOUNDS["ap_z_mm"]


def test_servo_lever_matches_a_finite_difference_of_virtual_work() -> None:
    from core.optimization.linkage.evaluate import servo_lever

    parameters = decode((*CROSSED, 24.0, 0.0))
    linkage = FingerLinkage(parameters)
    sweep = linkage.sweep_servo([math.radians(d) for d in range(0, 61)])
    lever, ratio, travel = servo_lever(sweep, slice(None))

    # Recompute r at the tip C at one pose from two nearby servo angles.
    p0, p1 = (linkage.sweep_servo([math.radians(t)]) for t in (30.0, 30.01))
    c0, c1, b0 = p0.c[0], p1.c[0], p0.b[0]
    u = (c0 - b0) / 24.0
    n = (u[1], -u[0])
    r_c = abs(n[0] * (c1[0] - c0[0]) + n[1] * (c1[1] - c0[1])) / math.radians(0.01)

    assert lever >= r_c * 0.999
    assert 0.0 < travel < 2.0 * math.pi
    # On average the distal link turns (its whole rotation) / (servo travel) per servo radian.
    phi = sweep.total_flexion_rad
    assert ratio >= abs(phi[-1] - phi[0]) / travel * 0.999
    assert math.isfinite(evaluate(decode(GOOD), GOOD_CONFIG).lever_max_mm)


def test_curl_tip_ahead_of_o_is_held_unless_enabled() -> None:
    evaluation = evaluate(decode(GOOD), GOOD_CONFIG)
    tip = evaluation.curl_tip_x_mm
    config = FingerStudyConfig(bounds=BOUNDS, packaging=HOUSING_ONLY, extended_length_bounds_mm=(90.0, 110.0),
                               curl_tip_min_x_mm=tip + 3.0, enforce_curl_tip=True)

    assert evaluation.constraints["curl_tip_x_mm"] == 0.0
    assert evaluate(decode(GOOD), config).constraints["curl_tip_x_mm"] == pytest.approx(3.0)


def test_reference_point_b_makes_length_and_curl_tip_independent_of_bc() -> None:
    lengths = []
    tips = []
    for bc in (24.0, 34.0):
        config = FingerStudyConfig(bounds=BOUNDS, packaging=HOUSING_ONLY, reference_point="B",
                                   extended_length_bounds_mm=(0.0, 100.0))
        evaluation = evaluate(decode((*GOOD[:-2], bc, 0.0)), config)
        lengths.append(evaluation.constraints["extended_length_mm"])
        tips.append(evaluation.curl_tip_x_mm)
    assert lengths[0] == pytest.approx(lengths[1])
    assert tips[0] == pytest.approx(tips[1])


def test_machined_dimensions_sit_on_the_resolution_grid() -> None:
    raw = decode(GOOD)
    built = decode(GOOD, link_length_resolution_mm=0.1)

    def on_grid(value):
        return abs(value * 10.0 - round(value * 10.0)) < 1e-6

    # Proximal: A and A' from O, unchanged. Distal: B' from B. Bars: their lengths.
    assert (built.a_x_mm, built.ap) == (raw.a_x_mm, raw.ap)
    assert on_grid(built.bp.x_mm - built.b_x_mm) and on_grid(built.bp.z_mm - built.b_z_mm)
    assert on_grid(math.hypot(built.b_x_mm - built.ap.x_mm, built.b_z_mm - built.ap.z_mm))
    assert on_grid(math.hypot(built.bp.x_mm - built.a_x_mm, built.bp.z_mm))
    # The assembled B moves by about the resolution.
    assert math.hypot(built.b_x_mm - raw.b_x_mm, built.b_z_mm - raw.b_z_mm) < 0.2


def test_a_may_sit_off_the_proximal_axis() -> None:
    raised = (*GOOD[:-1], 3.0)
    linkage = FingerLinkage(decode(raised))
    evaluation = evaluate(decode(raised), GOOD_CONFIG)

    assert (linkage.a.x_mm, linkage.a.z_mm) == (36.0, 3.0)
    # The support lengths do not depend on A.
    assert evaluation.support_length_mm == pytest.approx(evaluate(decode(GOOD), GOOD_CONFIG).support_length_mm)
    assert evaluation.ab_mm == pytest.approx(math.hypot(79.3 - 36.0, 10.0 - 3.0))
    # The middle link A-B' turns about the raised A.
    pose = linkage.solve(0.0, math.radians(30.0))
    assert math.dist((pose.a.x_mm, pose.a.z_mm), (pose.bp.x_mm, pose.bp.z_mm)) == pytest.approx(
        math.dist((36.0, 3.0), (63.3, 17.4)))


def test_older_design_rows_read_with_a_on_the_axis_and_a_double_prime_below_a_prime() -> None:
    from core.optimization.linkage import read_design

    row = {name: "1.0" for name in VARIABLE_NAMES if name not in ("a_z_mm", "app_offset_mm", "bpp_offset_mm")}
    assert read_design(row)[-3:] == [0.0, 0.0, 0.0]


def test_the_proximal_pad_may_run_past_a_prime() -> None:
    from core.morphology.linkage.packaging import pad_spans

    base = decode(GOOD)
    longer = decode((*GOOD, 5.0))
    assert FingerLinkage(longer).app.x_mm == pytest.approx(base.ap.x_mm + 5.0)
    assert pad_spans(longer, HOUSING_ONLY)["proximal_pad"].length_mm == pytest.approx(
        pad_spans(base, HOUSING_ONLY)["proximal_pad"].length_mm + 5.0)
    e = evaluate(longer, GOOD_CONFIG)
    assert e.proximal_support_length_mm == pytest.approx(evaluate(base, GOOD_CONFIG).proximal_support_length_mm + 5.0)
    # After the fact, the pad runs as far past A' as keeps the design as it is.
    from core.optimization.linkage.extend import longest_proximal_pad

    result = longest_proximal_pad(encode(FingerLinkage(base)), GOOD_CONFIG, extra_clearance_mm=0.5)
    assert result.added_mm > 0.0 and result.evaluation.feasible
    assert result.evaluation.phi_safe_rad == evaluate(base, GOOD_CONFIG).phi_safe_rad
    assert result.evaluation.pad_clearance_mm >= 0.5
    offset = VARIABLE_NAMES.index("app_offset_mm")
    one_more = decode((*result.x[:offset], result.x[offset] + 0.1, *result.x[offset + 1:]))
    beyond = evaluate(one_more, GOOD_CONFIG)
    assert not beyond.feasible or beyond.pad_clearance_mm < 0.5 or beyond.phi_safe_rad != result.evaluation.phi_safe_rad
    # The distal pad runs past B' toward the joint the same way.
    toward = decode((*GOOD, 0.0, 4.0))
    assert FingerLinkage(toward).bpp.x_mm == pytest.approx(base.bp.x_mm - 4.0)
    assert pad_spans(toward, HOUSING_ONLY)["distal_pad"].length_mm == pytest.approx(
        pad_spans(base, HOUSING_ONLY)["distal_pad"].length_mm + 4.0)
    # Left out of a config's bounds, the offset is fixed at 0.
    bounds = {name: pair for name, pair in BOUNDS.items() if name != "app_offset_mm"}
    assert FingerStudyConfig(bounds=bounds).bounds["app_offset_mm"] == (0.0, 0.0)


def test_pins_clear_the_servo_by_their_envelope() -> None:
    bearing = FingerStudyConfig(bounds=BOUNDS, extended_length_bounds_mm=(90.0, 110.0))
    earlier = evaluate(decode(GOOD), GOOD_CONFIG).constraints["pivot_pin_mm"]

    # The default 8 mm bearing housing.
    assert bearing.packaging.pin_radius_mm == 4.0
    assert evaluate(decode(GOOD), bearing).constraints["pivot_pin_mm"] == pytest.approx(earlier + 0.75)


def test_b_prime_pin_ends_the_safe_range_at_the_servo_only_when_checked() -> None:
    # B' = (62, 15) starts inside the PIP servo on A' = (52, 15).
    design = decode((*CROSSED, 24.0, 0.0))
    assert evaluate(design, CONFIG).stop_reason != "dip_pin"  # another Y layer: allowed
    checked = dataclasses.replace(CONFIG, packaging=dataclasses.replace(
        HOUSING_ONLY, allow_collision_pip_servo_to_joint_bp=False))
    evaluation = evaluate(design, checked)

    assert evaluation.stop_reason == "dip_pin"
    assert evaluation.phi_safe_rad == pytest.approx(0.0)
    assert evaluation.constraints["minimum_flexion_deg"] == pytest.approx(120.0)


def test_config_files_give_angles_in_degrees() -> None:
    from core.optimization.linkage.config import config_from_mapping

    config = config_from_mapping({
        "bounds": {name: [0.0, 200.0] for name in VARIABLE_NAMES},
        "motion": {"minimum_flexion_deg": 150.0},
        "servo_travel_max_deg": None,
        "extended_length_bounds_mm": [80.0, 100.0],
        "packaging": {"pivot_clearance_diameter_mm": 8.0, "pip_servo": {
            "radius_mm": 10.0, "rear_extent_mm": 20.0, "depth_mm": 22.0}},
    })

    assert config.minimum_flexion_rad == pytest.approx(math.radians(150.0))
    assert config.servo_travel_max_rad is None
    assert config.extended_length_bounds_mm == (80.0, 100.0)
    assert config.packaging.pin_radius_mm == 4.0
    assert config.packaging.pip_servo.radius_mm == 10.0
    # Runs recorded the pivot clearance by its earlier name.
    earlier = config_from_mapping({"bounds": BOUNDS, "packaging": {"pin_envelope_diameter_mm": 9.0}})
    assert earlier.packaging.pivot_clearance_diameter_mm == 9.0
    with pytest.raises(ValueError, match="unknown config entry"):
        config_from_mapping({"bounds": BOUNDS, "minimum_flexion_mm": 1.0})


def test_the_linkage_script_config_loads() -> None:
    from pathlib import Path

    from core.optimization.linkage.config import load_yaml

    path = Path(__file__).resolve().parents[1] / "scripts/optimize/linkage/config.yaml"
    config = load_yaml(path)

    assert config.minimum_flexion_rad == pytest.approx(math.radians(150.0))
    assert config.dip_backoff_tolerance_rad == pytest.approx(math.radians(0.05))
    assert config.bounds["servo_tilt_deg"] == (20.0, 20.0)  # an input, not optimized
    assert config.bounds["a_z_mm"] == (0.0, 0.0)
    assert config.packaging.proximal_pad == ContactPad(height_mm=10.0, offset_mm=0.0, chamfer_mm=2.5)
    assert config.link_length_resolution_mm is None
    assert config.packaging.pin_radius_mm == 4.0


def test_hyperextension_and_flexion_margin_are_separate() -> None:
    strict = dict(bounds=BOUNDS, packaging=HOUSING_ONLY, extended_length_bounds_mm=(90.0, 110.0),
                  minimum_transmission_margin_rad=math.radians(50.0))
    base = evaluate(decode(GOOD), FingerStudyConfig(**strict))
    wider = evaluate(decode(GOOD), FingerStudyConfig(
        **strict, flexion_stopper_margin_rad=math.radians(20.0)))
    shallow = evaluate(decode(GOOD), FingerStudyConfig(**strict, hyperextension_rad=math.radians(5.0)))

    assert wider.usable_pip_rad == pytest.approx(base.usable_pip_rad - math.radians(10.0))
    assert shallow.usable_pip_rad == pytest.approx(base.usable_pip_rad)


def test_linear_constraints_narrow_the_search_box_without_losing_designs() -> None:
    from pathlib import Path

    from core.optimization.linkage.config import load_yaml

    import dataclasses

    # The script config's box, with the lengths fixed here: |OC| 80-100, BC 30-50,
    # A and B on the axis.
    config = load_yaml(Path(__file__).resolve().parents[1] / "scripts/optimize/linkage/config.yaml")
    config = dataclasses.replace(config, extended_length_bounds_mm=(80.0, 100.0), minimum_proximal_support_mm=0.0,
                                 minimum_distal_support_mm=0.0, packaging=dataclasses.replace(
        config.packaging, pad_link_ratio_max=None), bounds={
        **config.bounds, "bc_length_mm": (30.0, 50.0), "a_x_mm": (5.0, 120.0), "b_x_mm": (40.0, 100.0),
        "a_z_mm": (0.0, 0.0), "b_z_mm": (0.0, 0.0),
        "ap_x_mm": (20.0, 100.0), "bp_x_mm": (-50.0, 200.0)})
    box = config.search_bounds

    # |OC| = B_x + BC <= 100 with BC >= 30; A_x < B_x; B'_x <= B_x; and, all
    # on the x axis, the extended pads clear each other, the A and B pins (4 + 1
    # mm) and the MCP servo's shaft disk (9.5 + 1 mm).
    assert box["b_x_mm"] == (40.0, 70.0)
    assert box["a_x_mm"] == (5.0, 65.0)
    assert box["ap_x_mm"] == (20.0, 65.0)
    assert box["bp_x_mm"] == (21.0, 70.0)
    assert config.bounds["b_x_mm"] == (40.0, 100.0)  # the given bounds are untouched
    # Every design in the file's box that meets the linear constraints is in the search box.
    rng = np.random.default_rng(0)
    names = ("a_x_mm", "b_x_mm", "ap_x_mm", "bp_x_mm", "bc_length_mm")
    low = np.array([config.bounds[name][0] for name in names])
    high = np.array([config.bounds[name][1] for name in names])
    for a_x, b_x, ap_x, bp_x, bc in rng.uniform(low, high, (20000, len(names))):
        linear = (80.0 <= b_x + bc <= 100.0 and a_x < b_x and ap_x >= a_x and bp_x <= b_x
                  and ap_x - config.packaging.proximal_pad_start_mm >= 0.0
                  and bp_x >= max(ap_x + 1.0, a_x + 5.0, 10.5) and ap_x <= b_x - 5.0)
        if linear:
            for name, value in zip(names, (a_x, b_x, ap_x, bp_x, bc)):
                assert box[name][0] <= value <= box[name][1]


def test_the_toggle_margin_and_hyperextension_phi_are_reported() -> None:
    evaluation = evaluate(decode(GOOD), GOOD_CONFIG)
    end = math.degrees(evaluation.usable_pip_rad)
    sweep = FingerLinkage(decode(GOOD)).sweep_pip(np.radians(np.arange(-10.0, end, 0.01)))

    # The angle at B (the driven link and B-B' in line) over the same range as
    # the transmission margin, up to the servo sampling.
    assert evaluation.toggle_margin_rad == pytest.approx(sweep.margin_at_b_rad.min(), abs=2e-2)
    # At the -10 deg PIP stopper the passive DIP turns back too.
    assert evaluation.hyperextension_phi_rad == pytest.approx(sweep.total_flexion_rad[0], abs=1e-6)
    assert evaluation.hyperextension_phi_rad < -math.radians(10.0)


def test_the_curl_tip_is_taken_at_the_most_usable_curl() -> None:
    strict = FingerStudyConfig(bounds=BOUNDS, packaging=HOUSING_ONLY, extended_length_bounds_mm=(90.0, 110.0),
                               minimum_transmission_margin_rad=math.radians(50.0), enforce_curl_tip=True)
    evaluation = evaluate(decode(GOOD), strict)
    pose = FingerLinkage(decode(GOOD)).solve(0.0, evaluation.usable_pip_rad)

    # The range ends short of the curl, and the tip is still checked there.
    assert evaluation.stop_reason == "transmission"
    assert evaluation.curl_tip_x_mm == pytest.approx(pose.c.x_mm)
    assert evaluation.constraints["curl_tip_x_mm"] == pytest.approx(-pose.c.x_mm)


def test_the_pads_are_their_own_config_group() -> None:
    from core.optimization.linkage.config import config_from_mapping

    bounds = {name: [0.0, 200.0] for name in VARIABLE_NAMES}
    config = config_from_mapping({"bounds": bounds, "packaging": {"body_clearance_mm": 1.0}, "pad": {
        "allow_collision_with_mcp_servo": True, "allow_longer_pad_than_link": False,
        "proximal": {"height_mm": 4.0}, "distal": {"height_mm": 6.0, "length_mm": 35.0}}})

    # Allowing the MCP servo collision is not checking it; a pad no longer than
    # its link is a ratio of 1.
    assert config.packaging.pad_mcp_check is False
    assert config.packaging.pad_link_ratio_max == 1.0
    default = config_from_mapping({"bounds": bounds, "pad": {
        "allow_collision_with_mcp_servo": False, "allow_longer_pad_than_link": True}})
    assert default.packaging.pad_mcp_check is True
    assert default.packaging.pad_link_ratio_max is None
    # The pads' gap to the MCP servo, apart from the other clearances.
    gap = config_from_mapping({"bounds": bounds, "pad": {"mcp_servo_clearance_mm": 2.0}})
    assert gap.packaging.mcp_pad_clearance_mm == 2.0
    assert gap.packaging.proximal_pad_start_mm == pytest.approx(9.5 + 2.0)
    assert default.packaging.mcp_pad_clearance_mm == default.packaging.body_clearance_mm
    assert config.packaging.proximal_pad.height_mm == 4.0
    assert config.packaging.distal_pad.length_mm == 35.0
    with pytest.raises(ValueError, match="unknown pad entries"):
        config_from_mapping({"bounds": bounds, "pad": {"thickness_mm": 5.0}})
    with pytest.raises(ValueError, match="also given in packaging"):
        config_from_mapping({"bounds": bounds, "packaging": {"pad_mcp_check": True},
                             "pad": {"allow_collision_with_mcp_servo": True}})


def test_the_search_box_may_be_named_optimization_parameter_bounds() -> None:
    from core.optimization.linkage.config import config_from_mapping

    box = {name: [0.0, 200.0] for name in VARIABLE_NAMES}
    assert config_from_mapping({"optimization_parameter_bounds": box}).bounds == config_from_mapping(
        {"bounds": box}).bounds
    with pytest.raises(ValueError, match="not both"):
        config_from_mapping({"optimization_parameter_bounds": box, "bounds": box})


def test_the_pip_servo_to_joint_a_check_can_be_turned_off() -> None:
    from core.optimization.linkage.config import config_from_mapping

    allowed = config_from_mapping({"bounds": {name: [0.0, 200.0] for name in VARIABLE_NAMES},
                                   "packaging": {"allow_collision_pip_servo_to_joint_A": True,
                                                 "allow_collision_pip_servo_to_joint_Bp": False}})
    assert allowed.packaging.allow_collision_pip_servo_to_joint_a is True
    assert allowed.packaging.allow_collision_pip_servo_to_joint_bp is False
    # GOOD's A pin envelope sits near the PIP servo: checked by default, not when allowed.
    checked = evaluate(decode(GOOD), GOOD_CONFIG).constraints["pivot_pin_mm"]
    free = evaluate(decode(GOOD), dataclasses.replace(GOOD_CONFIG, packaging=dataclasses.replace(
        HOUSING_ONLY, allow_collision_pip_servo_to_joint_a=True))).constraints["pivot_pin_mm"]
    assert checked != 0.0 and free == 0.0


def test_the_servos_are_chosen_by_actuator_name() -> None:
    from core.hardware import XM335Actuator, actuator
    from core.optimization.linkage.config import config_from_mapping

    bounds = {name: [0.0, 200.0] for name in VARIABLE_NAMES}
    config = config_from_mapping({"bounds": bounds, "packaging": {"pip_servo": "xm335", "mcp_servo": "xm335"}})
    assert config.packaging.pip_servo == XM335Actuator().geometry
    assert config.packaging.mcp_servo == actuator("xm335").geometry
    # A custom shape still works.
    custom = config_from_mapping({"bounds": bounds, "packaging": {"pip_servo": {
        "radius_mm": 8.0, "rear_extent_mm": 20.0, "depth_mm": 18.0}}})
    assert custom.packaging.pip_servo.radius_mm == 8.0
    with pytest.raises(ValueError, match="unknown actuator"):
        config_from_mapping({"bounds": bounds, "packaging": {"pip_servo": "xm999"}})


def test_the_minimum_flexion_may_be_named_the_minimum_desired_flexion_angle() -> None:
    from core.optimization.linkage.config import config_from_mapping

    bounds = {name: [0.0, 200.0] for name in VARIABLE_NAMES}
    config = config_from_mapping({"bounds": bounds, "motion": {"minimum_desired_flexion_angle_deg": 150.0}})
    assert config.minimum_flexion_rad == pytest.approx(math.radians(150.0))
    with pytest.raises(ValueError, match="given twice"):
        config_from_mapping({"bounds": bounds, "minimum_desired_flexion_angle_deg": 150.0,
                             "minimum_flexion_deg": 150.0})


def test_the_pip_servo_tilt_can_be_an_input() -> None:
    from core.optimization.linkage.config import config_from_mapping

    bounds = {name: [0.0, 200.0] for name in VARIABLE_NAMES if name != "servo_tilt_deg"}
    config = config_from_mapping({"bounds": bounds, "inputs": {"pip_servo_tilt_deg": 35.0}})
    assert config.search_bounds["servo_tilt_deg"] == (35.0, 35.0)
    with pytest.raises(ValueError, match="not both"):
        config_from_mapping({"bounds": {**bounds, "servo_tilt_deg": [0.0, 30.0]},
                             "inputs": {"pip_servo_tilt_deg": 35.0}})
    # The script config holds it at 35 deg.
    from pathlib import Path
    from core.optimization.linkage.config import load_yaml
    script = load_yaml(Path(__file__).resolve().parents[1] / "scripts/optimize/linkage/config.yaml")
    assert script.search_bounds["servo_tilt_deg"] == (20.0, 20.0)
