"""The linkage study: the goal / solver config and what it changes."""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest

from core.morphology.mechanism.linkage import Finger
from core.morphology.mechanism.linkage.packaging import (
    LinkagePackaging,
    checked_pairs,
    distal_pad_polygon,
    pad_margins,
    pad_spans,
    proximal_pad_polygon,
)
from core.morphology.packaging.shapes import separation
from core.morphology.mechanism.linkage.design import VARIABLE_NAMES, decode
from core.optimization.mechanism import mechanism, read_config_file
from core.optimization.study import load_yaml

CONFIG = Path(__file__).resolve().parents[1] / "scripts" / "optimize" / "linkage" / "config.yaml"
MECHANISM = mechanism("linkage")
X = (40.6, 66.0, 0.0, 44.4, 12.1, 56.9, 6.0, 10.0, 33.9, 0.0, -7.0, -12.1)
"""A feasible design of a short run (linkage-v10, 2026-11-04): both pads on
their link lines, the proximal one short of A, the distal one past B."""
TRIMMED = X
""":data:`X`, none of its pads past where its face meets the other's at the
required flexion (``pad_waste_mm``)."""


PINNED = ("goal.link.finger_length_mm=[90, 110]", "goal.pads.proximal.length_mm=[20, null]",
          "goal.pads.distal.length_mm=[25, null]", "goal.collision.check.a_pin_and_pip_servo=false",
          "goal.collision.check.a_pin_and_a_prime_pin=true",
          "goal.hardware.pip_servo.tilt_angle_deg=10",
          "goal.hardware.pip_servo.tilt_step_deg=1", "goal.pads.proximal.past_icr_max_mm=null",
          "goal.link.distal_length_mm=[25, null]", "goal.link.joint_span_ratio=[null, null]",
          "goal.link.proximal_length_ratio=[null, null]", "goal.link.distal_length_ratio=[null, null]",
          "solver.optimizer.preset=search", "goal.pads.on_links=false")
"""The study these tests were written for, whatever the config file now asks."""


def load(*overrides: str):
    return load_yaml(MECHANISM, CONFIG, (*PINNED, *overrides))[0]


def test_the_goal_sets_the_packaging_and_requirements():
    config, settings = load_yaml(MECHANISM, CONFIG, PINNED)
    packaging = config.packaging
    assert (settings.algorithm, settings.population_size, settings.generations, settings.seed) == ("nsga2", 200, 1000, 0)
    assert config.seeds == (0, 1, 2)
    assert config.objectives == ("transmission", "pad_coverage", "curl")
    assert math.degrees(config.required_rom_rad) == pytest.approx(150.0)
    assert config.extended_length_bounds_mm == (90.0, 110.0)
    assert [math.degrees(v) for v in config.transmission_angle_range_rad] == pytest.approx([30.0, 150.0])
    assert config.pip_servo_tilt_rad == pytest.approx(math.radians(10.0))
    assert config.bounds["bc_length_mm"] == (25.0, 110.0)
    # Pivots at their own sizes, the pads mounted on the link lines, no clearance anywhere.
    assert [packaging.pivot_radius_mm(p) for p in ("a", "ap", "b", "bp")] == [3.0, 5.0, 3.0, 3.0]
    assert packaging.ap_clearance_diameter_mm == 10.0
    assert packaging.interface_offset_mm("proximal_pad") == 0.0 and packaging.interface_offset_mm("distal_pad") == 0.0
    assert packaging.body_clearance_mm == 0.0 and packaging.mcp_pad_clearance_mm == 0.0
    assert packaging.proximal_pad_start_mm == packaging.mcp_servo.radius_mm
    assert not packaging.pip_servo_above_oa and packaging.pip_servo_pad_check and packaging.overlap_depth
    # With pad_coverage the pads are capped at their max (null: the finger's max).
    assert packaging.proximal_pad.max_length_mm == 110.0
    assert config.minimum_proximal_support_mm == 20.0 and config.minimum_distal_support_mm == 25.0
    # The reference point in minimized terms: coverage negated, curl 180 - phi_req.
    assert MECHANISM.hypervolume_reference(config) == {
        "transmission": 2.3, "pad_coverage": 0.0, "curl": 30.0, "finger_length": 110.0, "support_balance": 0.0}


def test_a_misspelt_or_missing_entry_is_an_error():
    values = read_config_file(CONFIG)
    values["goal"]["motion"]["dip_flexion"] = 150
    with pytest.raises(ValueError, match="goal.motion.dip_flexion"):
        MECHANISM.config_from_mapping(MECHANISM.split_settings(values)[0])
    values = read_config_file(CONFIG)
    del values["goal"]["pads"]["distal"]["height_mm"]
    with pytest.raises(ValueError, match="goal.pads.distal.height_mm"):
        MECHANISM.config_from_mapping(MECHANISM.split_settings(values)[0])


def test_the_tilt_is_fixed_or_searched_in_whole_degrees():
    from core.optimization.linkage import search
    from core.optimization.study import MechanismProblem

    fixed = load("goal.hardware.pip_servo.tilt_angle_deg=20.25")  # off the grid: kept as it is
    assert fixed.bounds["servo_tilt_deg"] == (20.25, 20.25)
    problem = MechanismProblem(MECHANISM, fixed)
    x = problem.to_mm(np.round((problem.xl + problem.xu) / 2))
    assert x is None or dict(zip(VARIABLE_NAMES, x))["servo_tilt_deg"] == 20.25
    searched = load("goal.hardware.pip_servo.tilt_angle_deg=[10, 30.5]")
    assert searched.pip_servo_tilt_rad is None and list(search.tilts_deg(searched)) == list(range(10, 31))
    assert MechanismProblem(MECHANISM, searched).xu[search.SEARCH_NAMES.index("servo_tilt_from_min_deg")] == 20


def test_the_pads_mount_on_their_link_lines():
    # The mounting surface on O-A and B-C (offset 0), the 5 mm thickness palmar of it.
    config = load()
    linkage = decode(X, config.link_length_resolution_mm)
    proximal = proximal_pad_polygon(linkage, config.packaging)
    assert proximal[:, 1].max() == pytest.approx(0.0)
    assert proximal[:, 1].min() == pytest.approx(-5.0)
    b, c = linkage.joint.b, linkage.tip
    distal = distal_pad_polygon(linkage, config.packaging, np.array([[b.x_mm, b.z_mm]]), np.array([[c.x_mm, c.z_mm]]))
    assert distal[0, :, 1].max() == pytest.approx(b.z_mm, abs=0.05)  # B-C barely tilted
    assert MECHANISM.evaluate(X, config).distal_depth_mm == pytest.approx(5.0)


def test_the_removed_carrier_and_pad_hangs_are_ignored_in_a_goal():
    # The pads are the contact interface spaces: no carrier, no hang to choose.
    config = load("goal.collision.check.distal_carrier_and_pip_servo=true",
                  "goal.collision.check.a_pin_and_proximal_pad=false",
                  "goal.collision.param.distal_carrier_height_mm=4", "goal.pads.distal.drop_mm=[0, 1]")
    assert config.packaging == load().packaging and config.bounds == load().bounds
    linkage = decode(X, config.link_length_resolution_mm)
    b, c = (np.array([[point.x_mm, point.z_mm]]) for point in (linkage.joint.b, linkage.tip))
    assert not any("carrier" in name for pair in pad_margins(linkage, config.packaging, b, c, b) for name in pair)


def test_overlap_is_negative_touching_zero():
    square = np.array([[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]])
    assert separation(square, square + (2.0, 0.0))[0] == pytest.approx(1.0)
    assert separation(square, square + (1.0, 0.0))[0] == pytest.approx(0.0)
    assert separation(square, square + (0.75, 0.0))[0] == pytest.approx(-0.25)


def test_pad_pairs_follow_the_checks():
    config = load()
    assert ("distal_pad", "proximal_pad") in checked_pairs(config.packaging)
    assert ("distal_pad", "mcp_servo") in checked_pairs(config.packaging)
    off = load("goal.collision.check.proximal_pad_and_distal_pad=false",
               "goal.collision.check.mcp_servo_and_proximal_pad=false")
    pairs = checked_pairs(off.packaging)
    assert ("distal_pad", "proximal_pad") not in pairs and ("mcp_servo", "proximal_pad") not in pairs
    assert ("distal_pad", "mcp_servo") in pairs  # always checked
    assert off.packaging.proximal_pad_start_mm == 0.0


def test_a_design_is_evaluated_with_the_objectives():
    config = load("goal.pads.proximal.length_mm=[15, null]", "goal.pads.distal.length_mm=[20, null]")
    e = MECHANISM.evaluate(TRIMMED, config)
    assert e.feasible
    spans = pad_spans(e.finger, config.packaging)
    covered = spans["proximal_pad"].length_mm + spans["distal_pad"].length_mm
    assert e.mounted_coverage == pytest.approx(covered / e.finger_length_mm)
    # The links are bent around the pads; each face counts less what the other
    # face hides in the fold, the least over [0, dip_flexion] (v4: here the
    # distal pad, from B, is never opposite the proximal one's end).
    assert config.packaging.links_clear_pads
    assert e.proximal_effective_mm == min(e.proximal_usable_length_mm, e.proximal_support_length_mm)
    assert e.proximal_effective_mm <= e.proximal_support_length_mm
    assert e.distal_effective_mm <= e.distal_support_length_mm
    assert e.pad_coverage <= e.mounted_coverage
    # The least lengths hold on the faces.
    short = MECHANISM.evaluate(TRIMMED, load("goal.pads.proximal.length_mm=[60, null]"))
    assert short.constraints["proximal_support_mm"] > 0.0 and not short.feasible
    assert e.curl_deg == pytest.approx(180.0 - math.degrees(e.phi_max_rad))
    assert MECHANISM.objective_values(e, config.objectives) == pytest.approx(
        (e.torque_ratio_max, -e.pad_coverage, e.curl_deg))
    report = MECHANISM.report(e)
    assert report["pad_coverage"] == e.pad_coverage and report["dip_flexion_deg"] == pytest.approx(150.0)


def test_the_transmission_range_bounds_the_motion():
    wide = MECHANISM.evaluate(X, load())
    narrow = MECHANISM.evaluate(X, load("goal.link.transmission_angle_deg=[70, 110]"))
    assert narrow.phi_safe_rad < wide.phi_safe_rad
    assert not narrow.feasible


def test_k_max_may_be_limited():
    pads = ("goal.pads.proximal.length_mm=[15, null]", "goal.pads.distal.length_mm=[20, null]")
    free = MECHANISM.evaluate(TRIMMED, load(*pads))
    assert free.feasible and free.constraints["torque_ratio"] == 0.0
    k = free.torque_ratio_max
    for limit, feasible in ((k + 0.01, True), (k - 0.01, False)):
        e = MECHANISM.evaluate(TRIMMED, load(*pads, f"goal.hardware.pip_servo.torque_ratio_max={limit}"))
        assert e.constraints["torque_ratio"] == pytest.approx(k - limit)
        assert e.feasible is feasible


def test_the_pip_servo_may_come_down_to_the_pad_but_not_into_it():
    config = load()
    assert MECHANISM.evaluate(X, config).constraints["pip_servo_pad_mm"] <= 0.0
    low = list(X)
    low[VARIABLE_NAMES.index("ap_z_mm")] = 0.0  # the servo's shaft disk reaches past the pad's top face
    assert MECHANISM.evaluate(low, config).constraints["pip_servo_pad_mm"] > 0.0


def test_packaging_defaults_keep_the_older_checks():
    packaging = LinkagePackaging()
    assert packaging.interface_offset_mm("proximal_pad") == packaging.interface_offset_mm("distal_pad") == 0.0
    assert packaging.pip_servo_above_oa and not packaging.pip_servo_pad_check and not packaging.overlap_depth
    assert all(packaging.pivot_radius_mm(p) == packaging.pin_radius_mm for p in ("a", "ap", "b", "bp"))


def test_the_pads_are_checked_through_the_hyperextension(monkeypatch):
    import importlib

    module = importlib.import_module("core.morphology.mechanism.linkage.motion")
    real, calls = module.pad_margins, []

    def recording(linkage, packaging, b, c, bp, mcp):
        calls.append((np.array(b), tuple(mcp)))
        return real(linkage, packaging, b, c, bp, mcp)

    monkeypatch.setattr(module, "pad_margins", recording)
    config = load()
    evaluation = MECHANISM.evaluate(X, config)
    linkage = decode(X, config.link_length_resolution_mm)
    extended = (linkage.joint.b.x_mm, linkage.joint.b.z_mm)
    # The flexion from extension, then the hyperextension back to the stopper
    # (the MCP servo turning from MCP extension in both).
    forward, back = calls[-2:]
    assert len(forward[0]) > 1 and forward[0][0] == pytest.approx(extended)
    assert len(back[0]) > 1 and min(forward[1]) == min(back[1]) == 0.0
    assert not np.any(np.all(np.isclose(back[0], extended), axis=1))  # back samples only
    # Measured as the distal segment's rotation back (the 2R's phi): to the stopper.
    assert evaluation.constraints["hyperextension_deg"] == 0.0
    assert evaluation.hyperextension_phi_rad == pytest.approx(-config.hyperextension_rad)


def test_runs_record_the_mechanism_and_reload(tmp_path):
    import json

    from core.optimization.optimizer import OptimizerSettings
    from core.optimization.study import Candidate, StudyResult, load_run, save_result

    config = load()
    candidate = Candidate(X, MECHANISM.evaluate(X, config))
    run = save_result(StudyResult(MECHANISM, config, OptimizerSettings(), (candidate,), 1), tmp_path / "run")
    loaded = load_run(run)
    assert loaded.mechanism is MECHANISM and loaded.current
    assert loaded.objectives == config.objectives
    assert loaded.config == config
    reloaded = MECHANISM.evaluate(X, loaded.config)
    assert reloaded.pad_coverage == candidate.evaluation.pad_coverage
    # Runs recorded as linkage_v3 (before the rename) load as this mechanism.
    metadata = json.loads((run / "run.json").read_text())
    metadata["mechanism"] = "linkage_v3"
    (run / "run.json").write_text(json.dumps(metadata))
    assert load_run(run).mechanism is MECHANISM


def test_the_search_coordinates_keep_the_simple_constraints():
    from core.optimization.linkage import search
    from core.optimization.study import MechanismProblem

    config = load()
    problem = MechanismProblem(MECHANISM, config)
    assert problem.n_var == len(search.SEARCH_NAMES)
    rng = np.random.default_rng(0)
    designs = 0
    for _ in range(300):
        x = problem.to_mm(np.round(problem.xl + rng.random(problem.n_var) * (problem.xu - problem.xl)))
        if x is None:
            continue
        designs += 1
        d = dict(zip(VARIABLE_NAMES, x))
        assert 0.0 < d["a_x_mm"] < d["b_x_mm"]
        assert 90.0 - 0.1 <= d["b_x_mm"] + d["bc_length_mm"] <= 110.0 + 0.1
        assert d["ap_x_mm"] >= search.least_ap_x(config, d["ap_z_mm"], d["servo_tilt_deg"]) - 0.1
        assert all(abs(v * 10 - round(v * 10)) < 1e-6 for v in d.values() if v == v)  # on the 0.1 mm grid (NaN: the packaging's drop)
    assert designs > 200
    # A design maps to coordinates and back.
    assert problem.to_mm(problem.to_steps(X))[:len(X)] == pytest.approx(X, abs=0.11)  # (then the drops: NaN)


def test_older_studies_are_searched_in_the_design_vector():
    config = dataclasses.replace(load(), relative_bounds=None)
    assert MECHANISM.search_names(config) == VARIABLE_NAMES
    assert MECHANISM.to_design(X, config) == X


def test_a_may_be_kept_out_of_the_pip_servo_body():
    # A' 8 mm over A: A's housing (3 mm) lies inside the servo's rounded
    # front (9.5 mm), allowed only with the check off.
    x = list(X)
    x[VARIABLE_NAMES.index("ap_z_mm")] = 8.0
    assert MECHANISM.evaluate(x, load()).constraints["pivot_pin_mm"] <= 0.0
    checked = MECHANISM.evaluate(x, load("goal.collision.check.a_pin_and_pip_servo=true"))
    assert not checked.feasible
    # A's centre is inside the body (its distance 0): the violation is its whole radius.
    assert checked.constraints["pivot_pin_mm"] == pytest.approx(3.0)


def test_the_pads_may_be_kept_off_their_own_pins():
    off = load("goal.pads.proximal.length_mm=[20, null]", "goal.pads.distal.length_mm=[20, null]")
    names = list(VARIABLE_NAMES)
    # X's pads, on their link lines, end short of A and start past B: clear of them.
    e = MECHANISM.evaluate(X, off)
    assert e.constraints["proximal_support_mm"] <= 0.0 and e.constraints["distal_support_mm"] <= 0.0
    # Run under its pin on the line, a pad meets the housing: it does not fit
    # (the pad is not moved down to clear it).
    # The housing's whole radius (3 mm) inside the pad's block: the violation.
    x = list(X)
    x[names.index("app_offset_mm")] += 5.0  # A'' 2 mm past A
    assert MECHANISM.evaluate(x, off).constraints["proximal_support_mm"] == pytest.approx(3.0)
    x = list(X)
    x[names.index("bpp_offset_mm")] += 5.0  # B'' 2 mm behind B
    assert MECHANISM.evaluate(x, off).constraints["distal_support_mm"] == pytest.approx(3.0)


def test_a_symmetric_four_bar_sets_b_prime():
    from core.optimization.linkage import search
    from core.optimization.study import MechanismProblem

    config = load("goal.link.symmetric_four_bar=true")
    assert not set(search.SYMMETRIC_DROPPED) & set(MECHANISM.search_names(config))
    problem = MechanismProblem(MECHANISM, config)
    rng = np.random.default_rng(1)
    found = 0
    for _ in range(300):
        x = problem.to_mm(np.round(problem.xl + rng.random(problem.n_var) * (problem.xu - problem.xl)))
        if x is None:
            continue
        d = dict(zip(VARIABLE_NAMES, x))
        a, ap = np.array((d["a_x_mm"], d["a_z_mm"])), np.array((d["ap_x_mm"], d["ap_z_mm"]))
        b, bp = np.array((d["b_x_mm"], d["b_z_mm"])), np.array((d["bp_x_mm"], d["bp_z_mm"]))
        # Opposite sides equal (to the grid), and crossed: not the parallelogram
        # (the two coincide only with A, A' and B in line).
        assert np.linalg.norm(bp - b) == pytest.approx(np.linalg.norm(ap - a), abs=0.15)
        assert np.linalg.norm(bp - a) == pytest.approx(np.linalg.norm(b - ap), abs=0.15)
        area = abs((ap - a)[0] * (b - a)[1] - (ap - a)[1] * (b - a)[0]) / 2.0
        assert area < 20.0 or np.linalg.norm(bp - (b + a - ap)) > 0.5
        found += 1
    assert found > 50


def test_the_proximal_pad_may_stop_at_the_fold():
    from core.morphology.mechanism.linkage.measures import extended_icr_x

    d = dict(zip(VARIABLE_NAMES, X))
    icr_x = extended_icr_x((d["a_x_mm"], 0.0), (d["ap_x_mm"], d["ap_z_mm"]), (d["b_x_mm"], 0.0),
                           (d["bp_x_mm"], d["bp_z_mm"]))
    assert d["a_x_mm"] < icr_x < d["b_x_mm"]  # between the joints, where the bars cross
    # A limit short of A (a pad on O-A cannot pass under A anyway): A'' 1 mm past it.
    limit = d["a_x_mm"] - 5.0
    config = load(f"goal.pads.proximal.past_icr_max_mm={limit - icr_x}", "goal.pads.proximal.length_mm=[15, null]",
                  "goal.pads.distal.length_mm=[20, null]")
    names = list(VARIABLE_NAMES)
    x = list(X)
    x[names.index("app_offset_mm")] = limit + 1.0 - d["ap_x_mm"]
    assert MECHANISM.evaluate(x, config).constraints["proximal_support_mm"] == pytest.approx(1.0, abs=0.15)
    x[names.index("app_offset_mm")] -= 2.0
    assert MECHANISM.evaluate(x, config).constraints["proximal_support_mm"] <= 0.0


def test_the_four_bar_span_may_be_bounded():
    d = dict(zip(VARIABLE_NAMES, X))
    span = d["b_x_mm"] - d["a_x_mm"]  # A and B on the axis, extended (B within the grid)
    e = MECHANISM.evaluate(X, load(f"goal.link.joint_span_mm=[null, {span - 2.0}]"))
    assert e.constraints["proximal_length_mm"] == pytest.approx(2.0, abs=0.1)
    assert MECHANISM.evaluate(X, load(f"goal.link.joint_span_mm=[null, {span + 1.0}]")).constraints[
        "proximal_length_mm"] <= 0.0


def test_a_pad_counts_up_to_where_the_faces_cross():
    from core.morphology.mechanism.linkage.packaging import face_crossing_lengths

    config = load()
    linkage = decode(X, config.link_length_resolution_mm)
    spans = pad_spans(linkage, config.packaging)
    sweep = linkage.sweep_servo(np.radians([0.0, 1.0, 40.0]) * (1.0 if linkage.sweep_servo(
        [0.0, 0.01]).q_driver_rad[-1] >= 0.0 else -1.0))
    free = face_crossing_lengths(linkage, config.packaging, sweep.b, sweep.tip)
    # Extended, the faces lie in one line: nothing is lost.
    assert free["proximal_pad"][0] == pytest.approx(spans["proximal_pad"].length_mm)
    assert free["distal_pad"][0] == pytest.approx(spans["distal_pad"].length_mm)
    # Barely flexed, they cross below the centre of rotation, between A and B:
    # past the proximal pad, which ends short of A; folded further, nearer.
    assert free["proximal_pad"][1] == pytest.approx(spans["proximal_pad"].length_mm)
    assert free["proximal_pad"][2] <= free["proximal_pad"][1]


def test_the_links_may_be_bounded_as_shares_of_the_finger():
    # The finger's segments: |OA''| and |B''C|.
    e = MECHANISM.evaluate(X, load())
    oc, proximal, distal = e.finger_length_mm, e.finger.proximal.length_mm, e.finger.distal.length_mm
    # With no contact module on the segment, its least share is a constraint.
    free = "goal.pads.proximal.length_mm=[0, null]"
    low = MECHANISM.evaluate(X, load(free, f"goal.link.proximal_length_ratio=[{(proximal + 2.0) / oc}, null]"))
    assert low.constraints["proximal_length_mm"] == pytest.approx(2.0, abs=1e-6)
    assert low.proximal_ratio_shortfall_mm == 0.0
    high = MECHANISM.evaluate(X, load(f"goal.link.distal_length_ratio=[null, {(distal - 3.0) / oc}]"))
    assert high.constraints["proximal_length_mm"] == pytest.approx(3.0, abs=1e-6)
    assert MECHANISM.evaluate(X, load("goal.link.proximal_length_ratio=[0.2, 0.5]",
                                      "goal.link.distal_length_ratio=[0.2, 0.45]")).constraints[
        "proximal_length_mm"] <= 0.0


def test_a_contact_module_sets_its_segments_least_length():
    # The requirement hierarchy: the module's least length (here 20 mm on the
    # proximal segment) bounds the segment from below, not its proportion,
    # which gives way and is reported; physical feasibility holds either way.
    e = MECHANISM.evaluate(X, load())
    oc, proximal = e.finger_length_mm, e.finger.proximal.length_mm
    ratio = f"goal.link.proximal_length_ratio=[{(proximal + 2.0) / oc}, null]"
    relaxed = MECHANISM.evaluate(X, load(ratio))
    assert relaxed.feasible and relaxed.constraints["proximal_length_mm"] <= 0.0
    assert relaxed.proximal_ratio_shortfall_mm == pytest.approx(2.0, abs=1e-6)
    assert MECHANISM.report(relaxed)["proximal_ratio_shortfall_mm"] == pytest.approx(2.0, abs=1e-6)
    # A module the segment cannot carry is still infeasible, whatever the proportion.
    big = MECHANISM.evaluate(X, load(ratio, "goal.pads.proximal.length_mm=[40, null]"))
    assert not big.feasible and big.constraints["proximal_support_mm"] > 0.0
    # The distal segment keeps its own: no module relaxes what is not its own.
    assert relaxed.distal_ratio_shortfall_mm == 0.0


def test_the_four_bar_span_may_be_bounded_as_a_share():
    e = MECHANISM.evaluate(X, load())
    d = dict(zip(VARIABLE_NAMES, X))
    span = d["b_x_mm"] - d["a_x_mm"]
    over = MECHANISM.evaluate(X, load(f"goal.link.joint_span_ratio=[null, {(span - 2.0) / e.finger_length_mm}]"))
    assert over.constraints["proximal_length_mm"] == pytest.approx(2.0, abs=0.1)


def test_the_pads_may_be_searched_on_their_links():
    from core.optimization.study import MechanismProblem

    config = load("goal.pads.on_links=true")
    assert not {"app_share", "bpp_share", "app_offset_mm", "bpp_offset_mm"} & set(MECHANISM.search_names(config))
    problem = MechanismProblem(MECHANISM, config)
    rng = np.random.default_rng(2)
    for _ in range(100):
        x = problem.to_mm(np.round(problem.xl + rng.random(problem.n_var) * (problem.xu - problem.xl)))
        if x is None:
            continue
        try:
            linkage = decode(x, config.link_length_resolution_mm)
        except ValueError:  # the rounded bars do not close: no design
            continue
        # The proximal segment to A, the distal from B (to the grid).
        assert linkage.proximal_end.x_mm == pytest.approx(linkage.joint.a.x_mm, abs=0.1)
        assert linkage.distal_start.x_mm == pytest.approx(linkage.joint.b.x_mm, abs=0.1)  # B re-assembled, B' with it


def test_the_objective_counts_each_pad_outside_the_fold_at_the_required_flexion():
    from core.morphology.mechanism.linkage.packaging import face_crossing_lengths

    config = load()
    e = MECHANISM.evaluate(X, config)
    assert e.phi_safe_rad >= config.required_rom_rad  # swept past the required flexion
    linkage = decode(X, config.link_length_resolution_mm)
    spans = pad_spans(linkage, config.packaging)
    # The required flexion on the servo grid: the first sample at or past it.
    sense = 1.0 if linkage.sweep_servo([0.0, 0.01]).q_driver_rad[-1] >= 0.0 else -1.0
    sweep = linkage.sweep_servo(sense * np.arange(0.0, math.pi, config.servo_step_rad))
    last = int(np.flatnonzero(sweep.total_flexion_rad >= config.required_rom_rad - 1e-12)[0])
    free = face_crossing_lengths(linkage, config.packaging, sweep.b[last:last + 1], sweep.tip[last:last + 1])
    assert e.proximal_effective_mm == pytest.approx(min(free["proximal_pad"][0], spans["proximal_pad"].length_mm))
    assert e.distal_effective_mm == pytest.approx(min(free["distal_pad"][0], spans["distal_pad"].length_mm))
    assert e.pad_coverage == pytest.approx((e.proximal_effective_mm + e.distal_effective_mm) / e.finger_length_mm)


def test_every_pad_mounts_on_its_link_line_whatever_it_meets():
    from core.morphology.mechanism.linkage.packaging import pad_offsets

    config = load()
    names = list(VARIABLE_NAMES)
    within = decode(X, config.link_length_resolution_mm)
    under = list(X)
    under[names.index("app_offset_mm")] += 5.0  # the proximal pad 2 mm past A, under its pin
    under_a = decode(under, config.link_length_resolution_mm)
    # One datum: no offset, short of A or under it (the pin a collision, not a reason to move the pad).
    for finger in (within, under_a):
        assert pad_offsets(finger, config.packaging) == {"proximal_pad": 0.0, "distal_pad": 0.0}
    assert MECHANISM.evaluate(X, config).feasible
    assert not MECHANISM.evaluate(under, config).feasible

def test_a_crossing_at_the_distal_faces_end_is_still_a_crossing():
    from core.morphology.packaging.faces import crossing_free_lengths

    # Inclined faces whose lines cross exactly at the distal face's inner end
    # (u = 0 there): the proximal face counts to the crossing, not whole.
    proximal = np.array(((0.0, -5.0), (30.0, -5.0)))
    distal = np.array(((35.0, -25.0), (15.0, -5.0)))
    free = crossing_free_lengths(proximal, distal)
    assert free["proximal_pad"][0] == pytest.approx(15.0)
    nudged = crossing_free_lengths(proximal, distal + (0.0, 1e-6))
    assert nudged["proximal_pad"][0] == pytest.approx(15.0, abs=1e-4)  # continuous


def test_the_search_does_not_bound_a_by_the_segment_ratio():
    from core.optimization.study import MechanismProblem

    # The ratio bounds |OA''| (A'' set by the pad's end, after A): A itself may
    # lie short of the share, as the evaluation allows, with the pad past it.
    config = load("goal.link.proximal_length_ratio=[0.35, null]")
    problem = MechanismProblem(MECHANISM, config)
    rng = np.random.default_rng(0)
    short = 0
    for _ in range(300):
        x = problem.to_mm(np.round(problem.xl + rng.random(problem.n_var) * (problem.xu - problem.xl)))
        if x is None:
            continue
        d = dict(zip(VARIABLE_NAMES, x))
        length = math.hypot(d["b_x_mm"] + d["bc_length_mm"], d["b_z_mm"])
        short += d["a_x_mm"] < 0.35 * length - 1.0
    assert short > 0


def test_a_fixed_layout_keeps_its_mounting_when_its_modules_change():
    from core.morphology.mechanism.linkage.packaging import proximal_pad_polygon
    from core.optimization.analysis.tradeoff import Modules, with_modules

    config = load()
    linkage = decode(X, config.link_length_resolution_mm)
    # Shorter or thicker modules: the same mounting surface on O-A, the thickness palmar of it.
    for modules in (Modules(10.0, 5.0, 20.0, 5.0), Modules(25.0, 8.0, 25.0, 8.0)):
        pad = proximal_pad_polygon(linkage, with_modules(config, modules).packaging)
        assert pad[:, 1].max() == pytest.approx(0.0)
        assert pad[:, 1].min() == pytest.approx(-modules.proximal_height_mm)