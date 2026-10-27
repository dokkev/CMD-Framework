"""The linkage study: the goal / solver config and what it changes."""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest

from core.morphology.linkage import FingerLinkage
from core.morphology.linkage.packaging import (
    Packaging,
    _separation,
    checked_pairs,
    distal_pad_polygon,
    pad_spans,
    proximal_pad_polygon,
)
from core.optimization.linkage.design import VARIABLE_NAMES, decode
from core.optimization.mechanism import mechanism, read_config_file
from core.optimization.study import load_yaml

CONFIG = Path(__file__).resolve().parents[1] / "scripts" / "optimize" / "linkage" / "config.yaml"
MECHANISM = mechanism("linkage")
X = (34.3, 76.0, 0.0, 39.7, 7.7, 61.8, 14.5, 20.0, 25.0, 0.0, 9.7, 2.9, 0.0, 0.0)
"""A feasible design of a short run."""


PINNED = ("goal.link.finger_length_mm=[90, 110]", "goal.pads.proximal.length_mm=[30, null]",
          "goal.pads.distal.length_mm=[35, null]")
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
    assert config.pip_servo_tilt_rad == pytest.approx(math.radians(20.0))
    assert config.bounds["bc_length_mm"] == (25.0, 110.0)
    # Pivots at their own sizes, pads below A and B, no clearance anywhere.
    assert [packaging.pivot_radius_mm(p) for p in ("a", "ap", "b", "bp")] == [3.0, 5.0, 3.0, 3.0]
    assert packaging.ap_clearance_diameter_mm == 10.0
    assert packaging.proximal_pad_drop_mm == 3.0 and packaging.distal_pad_drop_mm == 3.0
    assert packaging.body_clearance_mm == 0.0 and packaging.mcp_pad_clearance_mm == 0.0
    assert packaging.proximal_pad_start_mm == packaging.mcp_servo.radius_mm
    assert not packaging.pip_servo_above_oa and packaging.pip_servo_pad_check and packaging.overlap_depth
    # With pad_coverage the pads are capped at their max (null: the finger's max).
    assert packaging.proximal_pad.max_length_mm == 110.0
    assert not packaging.proximal_pad.chamfer_free and packaging.proximal_pad.chamfer_mm == 0.0  # plain blocks
    assert config.minimum_proximal_support_mm == 30.0 and config.minimum_distal_support_mm == 35.0
    # The reference point in minimized terms: coverage negated, curl 180 - phi_req.
    assert MECHANISM.hypervolume_reference(config) == {
        "transmission": 2.3, "pad_coverage": 0.0, "curl": 30.0, "finger_length": 110.0}


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


def test_the_pads_hang_below_their_pivots():
    config = load()
    linkage = FingerLinkage(decode(X, config.link_length_resolution_mm))
    proximal = proximal_pad_polygon(linkage, config.packaging)
    assert proximal[:, 1].max() == pytest.approx(-3.0)
    assert proximal[:, 1].min() == pytest.approx(-8.0)
    b, c = linkage.b, linkage.c
    distal = distal_pad_polygon(linkage, config.packaging, np.array([[b.x_mm, b.z_mm]]), np.array([[c.x_mm, c.z_mm]]))
    assert distal[0, :, 1].max() == pytest.approx(b.z_mm - 3.0)
    # Off, the pads lie on their link lines again.
    flat = dataclasses.replace(config.packaging, proximal_pad_below_a=False)
    assert proximal_pad_polygon(linkage, flat)[:, 1].max() == pytest.approx(0.0)


def test_overlap_is_negative_touching_zero():
    square = np.array([[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]])
    assert _separation(square, square + (2.0, 0.0))[0] == pytest.approx(1.0)
    assert _separation(square, square + (1.0, 0.0))[0] == pytest.approx(0.0)
    assert _separation(square, square + (0.75, 0.0))[0] == pytest.approx(-0.25)


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
    config = load()
    e = MECHANISM.evaluate(X, config)
    assert e.feasible
    spans = pad_spans(e.parameters, config.packaging)
    covered = spans["proximal_pad"].length_mm + spans["distal_pad"].length_mm
    assert e.mounted_coverage == pytest.approx(covered / e.finger_length_mm)
    # The objective counts each pad's face the links leave free on [0, dip_flexion].
    assert e.proximal_effective_mm == min(e.proximal_usable_length_mm, e.proximal_face_mm)
    assert e.distal_effective_mm <= e.distal_face_mm
    assert e.pad_coverage == pytest.approx(
        (e.proximal_effective_mm + e.distal_effective_mm) / e.finger_length_mm)
    assert e.pad_coverage < e.mounted_coverage
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


def test_the_pip_servo_may_come_down_to_the_pad_but_not_into_it():
    config = load()
    assert MECHANISM.evaluate(X, config).constraints["pip_servo_pad_mm"] <= 0.0
    low = list(X)
    low[VARIABLE_NAMES.index("ap_z_mm")] = 0.0  # the servo's shaft disk reaches past the pad's top face
    assert MECHANISM.evaluate(low, config).constraints["pip_servo_pad_mm"] > 0.0


def test_packaging_defaults_keep_the_older_checks():
    packaging = Packaging()
    assert packaging.proximal_pad_drop_mm == 0.0 and packaging.distal_pad_drop_mm == 0.0
    assert packaging.pip_servo_above_oa and not packaging.pip_servo_pad_check and not packaging.overlap_depth
    assert all(packaging.pivot_radius_mm(p) == packaging.pin_radius_mm for p in ("a", "ap", "b", "bp"))


def test_pads_are_not_checked_in_hyperextension(monkeypatch):
    import importlib

    module = importlib.import_module("core.optimization.linkage.evaluate")
    real, calls = module.pad_margins, []

    def recording(linkage, packaging, b, c, bp, mcp):
        calls.append((np.array(b), tuple(mcp)))
        return real(linkage, packaging, b, c, bp, mcp)

    monkeypatch.setattr(module, "pad_margins", recording)
    config = load()
    evaluation = MECHANISM.evaluate(X, config)
    linkage = FingerLinkage(decode(X, config.link_length_resolution_mm))
    # Hyperextension is linkage range only: the pad sweep starts at extension,
    # and the MCP servo turns from MCP extension.
    sweep_b, mcp = calls[-1]
    assert len(sweep_b) > 1 and sweep_b[0] == pytest.approx((linkage.b.x_mm, linkage.b.z_mm))
    assert min(mcp) == 0.0
    assert evaluation.constraints["hyperextension_deg"] == 0.0


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
        assert all(abs(v * 10 - round(v * 10)) < 1e-6 for v in d.values())  # on the 0.1 mm grid
    assert designs > 200
    # A design maps to coordinates and back.
    assert problem.to_mm(problem.to_steps(X)) == pytest.approx(X, abs=0.11)


def test_older_studies_are_searched_in_the_design_vector():
    config = dataclasses.replace(load(), relative_bounds=None)
    assert MECHANISM.search_names(config) == VARIABLE_NAMES
    assert MECHANISM.to_design(X, config) == X
