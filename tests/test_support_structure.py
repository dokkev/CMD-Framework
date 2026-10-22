"""The contact pads under O-A'' and B''-C: geometry, objective and collisions."""

import dataclasses
import json
import math

import numpy as np
import pytest

from core.morphology.linkage import FingerLinkage
from core.morphology.linkage.packaging import (
    COMPONENTS,
    Packaging,
    checked_pairs,
    ContactPad,
    PadSpan,
    distal_pad_polygon,
    layer_separated_pairs,
    point_segment_distance,
    pad_outline,
    pad_spans,
    proximal_pad_polygon,
    pad_margins,
)
from core.optimization.linkage import VARIABLE_NAMES, FingerStudyConfig, decode, encode, evaluate

BOUNDS = {name: (0.0, 200.0) for name in VARIABLE_NAMES}
# The earlier 6.5 mm pin envelope and the default 10 mm pads.
HOUSING_ONLY = Packaging(pivot_clearance_diameter_mm=6.5)
CONFIG = FingerStudyConfig(bounds=BOUNDS, packaging=HOUSING_ONLY, extended_length_bounds_mm=(90.0, 110.0))
GOOD = (36.0, 79.3, 10.0, 41.6, 12.6, 63.3, 17.4, 41.9, 30.0, 0.0)
# With 10 mm pads its distal pad meets the MCP servo a little before the curl,
# past the minimum flexion.
THICK_CONFIG = dataclasses.replace(CONFIG, packaging=dataclasses.replace(
    HOUSING_ONLY, proximal_pad=ContactPad(height_mm=10.0), distal_pad=ContactPad(height_mm=10.0)))


def xz(point) -> np.ndarray:
    return np.array((point.x_mm, point.z_mm))


def test_reference_support_endpoints_project_a_prime_and_b_prime() -> None:
    linkage = FingerLinkage(decode(GOOD))
    pose = linkage.solve(0.0, 0.0)

    assert (linkage.app.x_mm, linkage.app.z_mm) == (41.6, 0.0)
    assert (linkage.bpp.x_mm, linkage.bpp.z_mm) == (63.3, 10.0)
    assert xz(pose.app) == pytest.approx(xz(linkage.app))
    assert xz(pose.bpp) == pytest.approx(xz(linkage.bpp))
    evaluation = evaluate(decode(GOOD), CONFIG)
    # The proximal pad runs on O-A'' from past the MCP servo (pad_mcp_check).
    start = HOUSING_ONLY.proximal_pad_start_mm
    assert evaluation.proximal_support_length_mm == pytest.approx(np.linalg.norm(xz(linkage.app)) - start)
    assert evaluation.distal_support_length_mm == pytest.approx(
        np.linalg.norm(xz(linkage.c) - xz(linkage.bpp)))
    assert evaluation.support_length_mm == pytest.approx(41.6 - start + 46.0)


def test_support_endpoints_ride_rigidly_on_their_bodies() -> None:
    linkage = FingerLinkage(decode(GOOD))
    reference = linkage.solve(0.0, 0.0)

    def distances(pose):
        return ([np.linalg.norm(xz(pose.app) - xz(getattr(pose, name))) for name in ("o", "a", "ap")]
                + [np.linalg.norm(xz(pose.bpp) - xz(getattr(pose, name))) for name in ("b", "c", "bp")])

    for q_mcp, q_pip in ((0.0, 0.6), (0.7, 1.2), (-0.17, 1.6)):
        pose = linkage.solve(q_mcp, q_pip)
        assert distances(pose) == pytest.approx(distances(reference))
        # B'' stays on B-C, not on a fixed horizontal line.
        (ux, uz), (vx, vz) = xz(pose.c) - xz(pose.b), xz(pose.bpp) - xz(pose.b)
        cross = ux * vz - uz * vx
        assert cross == pytest.approx(0.0, abs=1e-9)
    assert linkage.solve(0.0, 1.2).bpp.z_mm != pytest.approx(linkage.bpp.z_mm, abs=1.0)


def test_pads_hang_palmar_of_their_mounting_lines_and_ride_on_their_links() -> None:
    linkage = FingerLinkage(decode(GOOD))
    t, k = HOUSING_ONLY.proximal_pad.height_mm, HOUSING_ONLY.proximal_pad.chamfer_mm
    start = HOUSING_ONLY.proximal_pad_start_mm
    # The proximal pad: a-b on O-A'', a-c square to it, the corner d under A'' chamfered.
    assert proximal_pad_polygon(linkage, HOUSING_ONLY) == pytest.approx(np.array(
        ((start, -t), (41.6 - k, -t), (41.6, -t + k), (41.6, 0.0), (start, 0.0))))

    sweep = linkage.sweep_pip(np.radians([0.0, 30.0, 70.0]))
    distal = distal_pad_polygon(linkage, HOUSING_ONLY, sweep.b, sweep.c)
    for i in range(3):
        pose = linkage.solve(0.0, float(sweep.q_pip_rad[i]))
        pad = distal[i]
        e = (xz(pose.c) - xz(pose.b)) / linkage.parameters.bc_length_mm
        palmar = np.array((e[1], -e[0]))
        # a-b is B''-C; the block is t palmar of it, chamfered at the corner c under B''.
        assert pad[3] == pytest.approx(xz(pose.bpp))
        assert pad[2] == pytest.approx(xz(pose.c))
        assert pad[1] == pytest.approx(xz(pose.c) + t * palmar)
        assert pad[0] == pytest.approx(xz(pose.bpp) + k * e + t * palmar)
        assert pad[4] == pytest.approx(xz(pose.bpp) + (t - k) * palmar)


def test_the_mcp_check_moves_the_proximal_pad_off_o() -> None:
    off = dataclasses.replace(HOUSING_ONLY, pad_mcp_check=False)
    linkage = FingerLinkage(decode(GOOD))

    # Off: the pad starts at O and the MCP servo is not checked.
    assert proximal_pad_polygon(linkage, off)[-1] == pytest.approx((0.0, 0.0))
    assert not any("mcp_servo" in pair for pair in checked_pairs(off))
    # On: it starts past the servo's shaft disk, and both pads are checked against the servo.
    on_start = HOUSING_ONLY.mcp_servo.radius_mm + HOUSING_ONLY.body_clearance_mm
    assert proximal_pad_polygon(linkage, HOUSING_ONLY)[-1] == pytest.approx((on_start, 0.0))
    assert {("mcp_servo", "proximal_pad"), ("distal_pad", "mcp_servo")} <= set(checked_pairs(HOUSING_ONLY))
    on_eval = evaluate(decode(GOOD), CONFIG)
    off_eval = evaluate(decode(GOOD), dataclasses.replace(CONFIG, packaging=off))
    assert on_eval.proximal_support_length_mm == pytest.approx(41.6 - on_start)
    assert off_eval.proximal_support_length_mm == pytest.approx(41.6)
    # The default offset clears the servo over MCP hyperextension and flexion.
    reference = [np.array([[p.x_mm, p.z_mm]]) for p in (linkage.b, linkage.c, linkage.bp)]
    margins = pad_margins(linkage, HOUSING_ONLY, *reference, CONFIG.mcp_samples_rad)
    assert margins[("mcp_servo", "proximal_pad")][0] >= 0.0


def test_pad_offsets_move_the_outer_ends_inward() -> None:
    offset = dataclasses.replace(HOUSING_ONLY, proximal_pad=ContactPad(offset_mm=3.0),
                                 distal_pad=ContactPad(offset_mm=4.0))
    linkage = FingerLinkage(decode(GOOD))
    base, moved = (evaluate(decode(GOOD), dataclasses.replace(CONFIG, packaging=packaging))
                   for packaging in (HOUSING_ONLY, offset))

    # The proximal pad starts 3 mm further right; the distal pad starts 4 mm short of C.
    assert proximal_pad_polygon(linkage, offset)[-1][0] == pytest.approx(HOUSING_ONLY.proximal_pad_start_mm + 3.0)
    tip = distal_pad_polygon(linkage, offset, np.array([xz(linkage.b)]), np.array([xz(linkage.c)]))[0, 2]
    assert tip == pytest.approx(xz(linkage.c) - (4.0, 0.0))
    assert moved.proximal_support_length_mm == pytest.approx(base.proximal_support_length_mm - 3.0)
    assert moved.distal_support_length_mm == pytest.approx(base.distal_support_length_mm - 4.0)


def test_a_fixed_pad_length_sets_the_pad_and_needs_room_on_its_line() -> None:
    from core.morphology.linkage.packaging import pad_spans

    fixed = dataclasses.replace(HOUSING_ONLY, proximal_pad=ContactPad(length_mm=20.0),
                                distal_pad=ContactPad(length_mm=30.0))
    parameters = decode(GOOD)
    spans = pad_spans(parameters, fixed)
    evaluation = evaluate(parameters, dataclasses.replace(CONFIG, packaging=fixed))

    # Proximal: right from its start; distal: left from C. Both fit on GOOD.
    assert spans["proximal_pad"].start_mm == pytest.approx(fixed.proximal_pad_start_mm)
    assert spans["proximal_pad"].length_mm == pytest.approx(20.0)
    assert spans["distal_pad"].end_mm == pytest.approx(parameters.bc_length_mm)
    assert spans["distal_pad"].length_mm == pytest.approx(30.0)
    assert (evaluation.proximal_support_length_mm, evaluation.distal_support_length_mm) == (20.0, 30.0)
    assert evaluation.constraints["proximal_support_mm"] <= 0.0
    # A pad longer than its mounting line's room does not fit.
    too_long = dataclasses.replace(HOUSING_ONLY, distal_pad=ContactPad(length_mm=50.0))
    room = pad_spans(parameters, too_long)["distal_pad"].room_mm
    assert evaluate(parameters, dataclasses.replace(CONFIG, packaging=too_long)).constraints[
        "distal_support_mm"] == pytest.approx(50.0 - room)


def test_the_objectives_follow_the_config() -> None:
    from core.optimization.linkage import MECHANISM
    from core.optimization.study import MechanismProblem

    active_objectives = MECHANISM.active_objectives

    both_fixed = dataclasses.replace(CONFIG, packaging=dataclasses.replace(
        HOUSING_ONLY, proximal_pad=ContactPad(length_mm=20.0), distal_pad=ContactPad(length_mm=30.0)))
    one_fixed = dataclasses.replace(CONFIG, packaging=dataclasses.replace(
        HOUSING_ONLY, proximal_pad=ContactPad(length_mm=20.0)))

    # By default: transmission and curl; the pads are requirements, not an objective.
    assert active_objectives(CONFIG) == ("transmission", "curl")
    assert active_objectives(one_fixed) == ("transmission", "curl")
    assert active_objectives(both_fixed) == ("transmission", "curl")
    assert MechanismProblem(MECHANISM, both_fixed).n_obj == 2
    with_pads = dataclasses.replace(CONFIG, objectives=("transmission", "support_length", "curl"))
    assert active_objectives(with_pads) == ("transmission", "support_length", "curl")
    assert MechanismProblem(MECHANISM, with_pads).n_obj == 3
    chosen = dataclasses.replace(CONFIG, objectives=("curl", "transmission"))
    assert active_objectives(chosen) == ("transmission", "curl")
    with pytest.raises(ValueError, match="unknown objectives"):
        active_objectives(dataclasses.replace(CONFIG, objectives=("speed",)))
    evaluation = evaluate(decode(GOOD), both_fixed)
    assert MECHANISM.objective_values(evaluation, ("transmission", "curl")) == (
        evaluation.torque_ratio_max, evaluation.curl_shortfall_deg)


def test_the_reported_pad_clearance_leaves_out_the_mcp_offset() -> None:
    from core.morphology.linkage.packaging import FIXED_MARGIN_PAIRS

    evaluation = evaluate(decode(GOOD), CONFIG)
    linkage = FingerLinkage(decode(GOOD))
    reference = [np.array([xz(p)]) for p in (linkage.b, linkage.c, linkage.bp)]
    margins = pad_margins(linkage, HOUSING_ONLY, *reference, CONFIG.mcp_samples_rad)

    # The proximal pad sits exactly the clearance off the MCP servo, in every design.
    assert ("mcp_servo", "proximal_pad") in FIXED_MARGIN_PAIRS
    assert margins[("mcp_servo", "proximal_pad")][0] == 0.0
    # Still checked, but the reported clearance comes from the other pairs.
    assert evaluation.pad_clearance_mm > 0.0


def test_flatness_is_an_opt_in_objective_and_k_max_can_be_capped() -> None:
    from core.optimization.linkage import MECHANISM

    active_objectives = MECHANISM.active_objectives
    evaluation = evaluate(decode(GOOD), CONFIG)
    assert 0.0 < evaluation.torque_ratio_min <= evaluation.torque_ratio_max
    assert evaluation.torque_ratio_spread == pytest.approx(
        evaluation.torque_ratio_max / evaluation.torque_ratio_min)
    assert "flatness" not in active_objectives(CONFIG)
    flat = dataclasses.replace(CONFIG, objectives=("flatness", "curl"))
    assert active_objectives(flat) == ("flatness", "curl")
    # A cap on k_max is a constraint, off by default.
    assert evaluation.constraints["torque_ratio_max"] == 0.0
    capped = evaluate(decode(GOOD), dataclasses.replace(CONFIG, torque_ratio_max=1.5))
    assert capped.constraints["torque_ratio_max"] == pytest.approx(evaluation.torque_ratio_max - 1.5)
    assert not capped.feasible


def test_the_pip_servo_can_sit_on_a_and_carry_the_proximal_pad() -> None:
    import importlib

    on_a = dataclasses.replace(HOUSING_ONLY, pip_servo_at="a")
    linkage = FingerLinkage(decode(GOOD))
    body = on_a.pip_servo

    # The servo's shaft is on A, its body back along -X.
    footprint = on_a.pip_footprint(linkage)
    assert footprint.pivot == (36.0, 0.0) and footprint.rear == (-1.0, 0.0)
    # The proximal pad lies on the body's palmar face, as long as the body.
    pad = proximal_pad_polygon(linkage, on_a)
    assert pad[:, 1].max() == pytest.approx(-body.radius_mm)
    assert pad[:, 0].min() == pytest.approx(36.0 - body.rear_extent_mm)
    assert pad[:, 0].max() == pytest.approx(36.0 + body.radius_mm)
    # The servo drives the middle link A-B': its angle is the PIP's, and the
    # transmission angle is at B.
    module = importlib.import_module("core.optimization.linkage.evaluate")
    sweep = module._drive(linkage, on_a, np.radians([0.0, 10.0, 20.0]))
    assert sweep.servo_rad == pytest.approx(sweep.q_pip_rad)
    evaluation = evaluate(decode(GOOD), dataclasses.replace(CONFIG, packaging=on_a))
    if evaluation.constraints["minimum_flexion_deg"] < 50.0:  # swept
        reference = FingerLinkage(decode(GOOD)).sweep_pip([0.0])
        assert evaluation.min_margin_rad <= reference.margin_at_b_rad[0] + 1e-9
    assert evaluation.proximal_support_length_mm == pytest.approx(body.rear_extent_mm + body.radius_mm)
    assert evaluation.constraints["oa_interface_mm"] == 0.0


def test_a_pad_may_stand_out_of_its_link_by_the_ratio_at_most() -> None:
    capped = dataclasses.replace(HOUSING_ONLY, pad_link_ratio_max=1.3)
    free = evaluate(decode(GOOD), dataclasses.replace(CONFIG, packaging=capped))

    # GOOD: proximal pad 31.1 under 1.3 |OA| = 46.8, distal pad 46 over 1.3 BC = 39:
    # a free pad is cut to the limit, and the linkage stays feasible.
    assert free.proximal_support_length_mm == pytest.approx(31.1)
    assert free.distal_support_length_mm == pytest.approx(1.3 * 30.0)
    assert free.constraints["distal_support_mm"] <= 0.0
    # A fixed pad longer than the limit does not fit.
    fixed = dataclasses.replace(capped, distal_pad=ContactPad(length_mm=45.0))
    evaluation = evaluate(decode(GOOD), dataclasses.replace(CONFIG, packaging=fixed))
    assert evaluation.constraints["distal_support_mm"] == pytest.approx(45.0 - 1.3 * 30.0)


def test_a_chamfer_trims_only_the_palmar_corner_facing_the_pip() -> None:
    span = PadSpan(0.0, 30.0, 30.0)
    square = pad_outline(span, ContactPad(chamfer_mm=0.0), "end")
    chamfered = pad_outline(span, ContactPad(), "end")

    assert len(square) == 4 and len(chamfered) == 5
    # a-b, the mounting length, is untouched.
    assert {(0.0, 0.0), (30.0, 0.0)} <= {tuple(point) for point in chamfered}


def test_the_pad_objective_is_the_pad_length_and_the_free_length_is_reported() -> None:
    from core.optimization.linkage import MECHANISM

    base = evaluate(decode(GOOD), CONFIG)

    values = MECHANISM.objective_values(base, ("transmission", "support_length", "curl"))
    assert values[1] == pytest.approx(-base.support_length_mm)
    assert values[0] == base.torque_ratio_max
    assert values[2] == pytest.approx(180.0 - math.degrees(base.phi_safe_rad))
    # Free length is part of the pad; the pad itself is still reported.
    assert 0.0 <= base.proximal_usable_length_mm <= base.proximal_support_length_mm
    assert 0.0 <= base.distal_usable_length_mm <= base.distal_support_length_mm
    assert base.usable_support_length_mm < base.support_length_mm
    assert encode(FingerLinkage(decode(GOOD))) == (*GOOD, 0.0, 0.0)


def test_a_link_across_a_pad_ends_its_free_length() -> None:
    from core.morphology.linkage.packaging import usable_pad_lengths

    linkage = FingerLinkage(decode(GOOD))
    spans = pad_spans(linkage.parameters, HOUSING_ONLY)
    b, c = xz(linkage.b)[None], xz(linkage.c)[None]
    # The middle link A-B' straight down through the proximal pad at x = A_x = 36;
    # the coupler A'-B stays dorsal of both pads.
    free = usable_pad_lengths(linkage, HOUSING_ONLY, b, np.array([[36.0, -20.0]]), c)
    assert free["proximal_pad"][0] == pytest.approx(36.0 - spans["proximal_pad"].start_mm)
    assert free["distal_pad"][0] == pytest.approx(spans["distal_pad"].length_mm)


def test_only_a_crossing_of_the_pad_block_counts() -> None:
    from core.morphology.linkage.packaging import _block_extent

    # A 5 mm block over 10 < u < 40: a segment across it, one palmar of it.
    across = _block_extent(np.array([[20.0, 5.0]]), np.array([[30.0, -15.0]]), 10.0, 40.0, 5.0)
    below = _block_extent(np.array([[15.0, -8.0]]), np.array([[35.0, -8.0]]), 10.0, 40.0, 5.0)
    assert (across[0][0], across[1][0]) == pytest.approx((22.5, 25.0))
    assert (below[0][0], below[1][0]) == (np.inf, -np.inf)


def test_inverted_pads_are_rejected() -> None:
    a_past_ap = list(GOOD)
    a_past_ap[VARIABLE_NAMES.index("ap_x_mm")] = 35.0  # A'_x < A_x = 36
    bp_past_b = list(GOOD)
    bp_past_b[VARIABLE_NAMES.index("bp_x_mm")] = 80.0  # B'_x > B_x = 79.3

    assert evaluate(decode(a_past_ap), CONFIG).constraints["proximal_support_mm"] == pytest.approx(1.0)
    assert evaluate(decode(bp_past_b), CONFIG).constraints["distal_support_mm"] == pytest.approx(0.7)
    assert evaluate(decode(GOOD), CONFIG).constraints["proximal_support_mm"] <= 0.0
    minimum = dataclasses.replace(CONFIG, minimum_distal_support_mm=50.0)
    assert evaluate(decode(GOOD), minimum).constraints["distal_support_mm"] == pytest.approx(4.0)


def test_y_layers_decide_which_pairs_are_checked() -> None:
    pairs = checked_pairs(HOUSING_ONLY)

    # The link plates sit outside the central layer of the pads and servos.
    assert set(layer_separated_pairs(HOUSING_ONLY)) == {
        ("middle_link", "proximal_pad"), ("coupler", "proximal_pad"),
        ("distal_pad", "middle_link"), ("coupler", "distal_pad")}
    assert not any("middle_link" in pair or "coupler" in pair for pair in pairs)
    # A body's own parts and its joints' pins are its interfaces, not collisions.
    for pad, own in (("proximal_pad", ("pip_servo", "a_pin", "ap_pin")),
                         ("distal_pad", ("b_pin", "bp_pin"))):
        assert all(tuple(sorted((pad, part))) not in pairs for part in own)
    assert ("distal_pad", "proximal_pad") in pairs
    assert ("distal_pad", "pip_servo") in pairs
    assert set(COMPONENTS) >= {name for pair in pairs for name in pair}


def test_a_link_crossing_a_pad_in_the_side_view_is_not_a_collision() -> None:
    from core.morphology.linkage.packaging import _inside

    linkage = FingerLinkage(decode(GOOD))
    pad = proximal_pad_polygon(linkage, HOUSING_ONLY)
    # Flexed, the middle link A-B' passes through the proximal pad in the side view...
    pose = linkage.solve(0.0, math.radians(60.0))
    along = np.linspace(0.0, 1.0, 50)[:, None]
    link = xz(pose.a) + along * (xz(pose.bp) - xz(pose.a))
    assert _inside(link[None], pad[None]).any()
    # ...yet that pose is inside the usable range: the link plates are in other layers.
    evaluation = evaluate(decode(GOOD), CONFIG)
    assert evaluation.feasible and evaluation.usable_pip_rad > math.radians(60.0)


def test_a_pad_collision_past_the_minimum_flexion_ends_the_safe_range() -> None:
    evaluation = evaluate(decode(GOOD), THICK_CONFIG)

    assert evaluation.stop_reason == "pad_collision"
    assert evaluation.collision_pair == "distal_pad/mcp_servo"
    assert math.radians(150.0) < evaluation.phi_safe_rad < math.pi
    assert evaluation.feasible
    assert evaluation.pad_clearance_mm >= 0.0

    # The usable range ends a stopper margin before the first colliding sample.
    linkage = FingerLinkage(decode(GOOD))
    sweep = linkage.sweep_servo(THICK_CONFIG.servo_samples_rad)
    margins = pad_margins(linkage, THICK_CONFIG.packaging, sweep.b, sweep.c, sweep.bp, CONFIG.mcp_samples_rad)
    first = int(np.flatnonzero(np.min(np.stack(list(margins.values())), axis=0) < 0.0)[0])
    assert evaluation.usable_pip_rad == pytest.approx(sweep.q_pip_rad[first] - CONFIG.flexion_stopper_margin_rad)
    wider = evaluate(decode(GOOD), dataclasses.replace(
        THICK_CONFIG, flexion_stopper_margin_rad=math.radians(15.0)))
    assert wider.usable_pip_rad == pytest.approx(evaluation.usable_pip_rad - math.radians(5.0))


def test_a_pad_collision_before_the_minimum_flexion_is_infeasible() -> None:
    evaluation = evaluate(decode(GOOD), dataclasses.replace(
        THICK_CONFIG, minimum_flexion_rad=math.radians(179.9)))

    assert evaluation.stop_reason == "pad_collision"
    assert evaluation.constraints["minimum_flexion_deg"] > 0.0
    assert not evaluation.feasible


def test_pads_are_not_checked_in_hyperextension(monkeypatch) -> None:
    import importlib

    module = importlib.import_module("core.optimization.linkage.evaluate")
    real = module.pad_margins
    calls = []

    def recording(linkage, packaging, b, c, bp, mcp):
        calls.append((np.array(b), tuple(mcp)))
        return real(linkage, packaging, b, c, bp, mcp)

    monkeypatch.setattr(module, "pad_margins", recording)
    evaluation = evaluate(decode(GOOD), CONFIG)
    linkage = FingerLinkage(decode(GOOD))

    # Hyperextension is linkage range only: the pad sweep starts at extension,
    # and the MCP servo turns from MCP extension.
    sweep_b, mcp = calls[-1]
    assert len(sweep_b) > 1 and sweep_b[0] == pytest.approx(xz(linkage.b))
    assert min(mcp) == 0.0
    assert evaluation.constraints["hyperextension_deg"] == 0.0


def test_runs_record_the_mechanism_and_formulation_and_reload(tmp_path) -> None:
    from core.optimization.linkage import MECHANISM
    from core.optimization.optimizer import OptimizerSettings
    from core.optimization.study import Candidate, StudyResult, load_run, save_result

    candidate = Candidate(GOOD, evaluate(decode(GOOD), CONFIG))
    run = save_result(StudyResult(MECHANISM, CONFIG, OptimizerSettings(), (candidate,), 1), tmp_path / "run")

    loaded = load_run(run)
    assert loaded.mechanism is MECHANISM and loaded.current
    assert loaded.objectives == ("transmission", "curl")
    assert loaded.config.packaging == HOUSING_ONLY
    header = (run / "pareto.csv").read_text().splitlines()[0].split(",")
    assert {"support_length_mm", "proximal_support_length_mm", "distal_support_length_mm",
            "pad_clearance_mm", "collision_pair", "ab_mm"} <= set(header)
    assert "structural_length_mm" not in header

    # A run from before the mechanism and the formulation were recorded is an
    # OA + BC linkage run.
    metadata = json.loads((run / "run.json").read_text())
    del metadata["mechanism"], metadata["formulation"]
    (run / "run.json").write_text(json.dumps(metadata))
    legacy = load_run(run)
    assert legacy.mechanism is MECHANISM and not legacy.current
