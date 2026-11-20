"""Study 2 as re-optimization (problem formulation A.6.3, 2026-11-20)."""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest

from core.optimization.analysis.reoptimize import ClosestMechanism, interface_config, measure, path_change, tip_path
from core.optimization.analysis.tradeoff import Modules, goal_of
from core.optimization.mechanism import mechanism, read_config_file

ROOT = Path(__file__).resolve().parents[1]
STUDY2 = ROOT / "scripts" / "study" / "study2_morphology_adaptation"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


reopt = _load("study2_reoptimize", STUDY2 / "reoptimize.py")
shared = reopt.shared


@pytest.fixture(scope="module")
def references():
    """The matched references: built from the tracked design vector alone (native portfolios are not tracked)."""

    return {r.reference_id: r for r in shared.load_references(
        ("matched",), ROOT, "scripts/study/study2_morphology_adaptation/references_nominal15.yaml")}


def test_the_interface_fixes_both_pads_and_frees_the_finger_length_only_when_asked(references):
    reference = references["code_matched"]
    held = goal_of(interface_config(reference.config, Modules(32.5, 7.0, 27.5, 4.0)))
    assert held.extended_length_bounds_mm == goal_of(reference.config).extended_length_bounds_mm
    free = goal_of(interface_config(reference.config, Modules(32.5, 7.0, 27.5, 4.0), (60.0, 120.0)))
    assert free.extended_length_bounds_mm == (60.0, 120.0)
    for goal in (held, free):
        pads = goal.packaging.proximal_pad, goal.packaging.distal_pad
        assert (pads[0].length_mm, pads[0].height_mm, pads[1].length_mm, pads[1].height_mm) == (32.5, 7.0, 27.5, 4.0)
        # Each pad's least support is its own length: the study goal's 20/25 mm floor does not apply.
        assert (goal.minimum_proximal_support_mm, goal.minimum_distal_support_mm) == (32.5, 27.5)


def test_the_closest_design_is_held_to_the_coverage_floor(references):
    reference = references["code_matched"]
    base = mechanism(reference.mechanism)
    part = shared.partitions.partition(reference.mechanism)
    config = interface_config(reference.config, Modules(20.0, 3.0, 20.0, 3.0))
    d0, k0 = part.external(reference.x, config), part.mechanism(reference.x, config)
    path = tip_path(reference.mechanism, reference.x, config, math.radians(120.0))
    found = measure(base, part, reference.x, config, d0, k0, 90.0, path)
    assert found.evaluation.external_cost == pytest.approx(0.0) and found.evaluation.mechanism_cost == pytest.approx(0.0)
    if found.evaluation.base.feasible:
        assert found.evaluation.kinematic_cost == pytest.approx(0.0)
    coverage = found.coverage
    below = ClosestMechanism(base, part, d0, k0, 90.0, coverage + 0.01, path).evaluate(reference.x, config)
    within = ClosestMechanism(base, part, d0, k0, 90.0, coverage - 0.01, path).evaluate(reference.x, config)
    assert not below.feasible and below.floor == pytest.approx(0.01)
    assert within.feasible == bool(found.evaluation.base.feasible)


def test_the_sections_sweep_one_dimension_in_order():
    config = read_config_file(STUDY2 / "reoptimize.yaml")
    length = reopt.points("length", config["sections"]["length"])
    thickness = reopt.points("thickness", config["sections"]["thickness"])
    assert [p[0] for p in length] == sorted(p[0] for p in length) and {p[1] for p in length} == {5.0}
    assert length[1][0] - length[0][0] == 2.5
    assert [p[1] for p in thickness] == sorted(p[1] for p in thickness) and {p[0] for p in thickness} == {25.0}
    with pytest.raises(ValueError):
        reopt.points("proximal", {})


def test_a_small_reoptimization_runs_and_resumes(tmp_path, references, monkeypatch):
    config = {"references": "unused", "required_flexion_deg": [120], "finger_length_mm": [60, 120],
              "coverage_delta": 0.005, "sections": {"length": {"thickness_mm": 3, "length_mm": [20, 22.5]}},
              "search": {"algorithm": "ga", "population": 8, "generations": 2, "seeds": [0]},
              "closest": {"algorithm": "ga", "population": 8, "generations": 1, "seeds": [0]}, "workers": 1}
    study = reopt.Reoptimization(config, tmp_path, resume=False, finger_length=(60.0, 120.0))
    reference = references["code_matched"]
    grid = reopt.points("length", config["sections"]["length"])
    results = study.solve(reference, 120.0, "length", grid)
    assert [r["length_mm"] for r in results] == [20.0, 22.5]
    # Two sweeps per point, then the closest search where a design was found.
    labels = {r["pass"] for r in study.rows if r["length_mm"] == 20.0}
    assert {"up", "down"} <= labels
    assert all(label in ("up", "down") or label.startswith(("closest", "cross")) for label in labels)
    for r in results:
        if r.get("found") is not False:
            assert r["c_pad_mean"] >= r["best_c_pad_mean"] - config["coverage_delta"] - 1e-9
            assert math.isclose(r["delta_finger_length_mm"],
                                (r["d_finger_length_mm"] - study.ref_morphology[0]["finger_length_mm"]) / 90.0,
                                rel_tol=1e-6, abs_tol=1e-9)
    # The cross-evaluation: every retention at most 1 once the cross-check has corrected the best.
    matrix = reopt.retention(study, reference, 120.0, "length", grid, results)
    assert len(matrix) == sum(r.get("found") is not False for r in results) * len(grid)
    assert all(not math.isfinite(r["rho"]) or r["rho"] <= 1.0 + 1e-6 for r in matrix)
    count = len(study.rows)
    again = reopt.Reoptimization(config, tmp_path, resume=True, finger_length=(60.0, 120.0))
    again.solve(reference, 120.0, "length", grid)
    assert len(again.rows) == count  # nothing searched again


def test_the_family_bounds_the_swept_dimension_only():
    assert reopt.in_family("length", (20.0, 5.0), {"length_mm": [20, 35]})
    assert not reopt.in_family("length", (17.5, 5.0), {"length_mm": [20, 35]})
    assert reopt.in_family("thickness", (99.0, 10.0), {"thickness_mm": [3, 10]})
    assert reopt.in_family("thickness", (30.0, 11.0), None)


def test_a_frozen_finger_carries_an_interface_as_study_1_mounts_it(references):
    reference = references["code_matched"]
    carried = reopt.carry(reference, reference.x, reference.config, (20.0, 3.0), 120.0)
    pair = reopt.shared.ModulePair("x", "symmetric", 20.0, 3.0, 20.0, 3.0)
    study1 = reopt.study1.assembly(reference, pair, [120.0])[0]
    assert carried["feasible"] == study1["feasible"]
    if study1["feasible"]:  # the same coverage, unrounded (Study 1 rounds its rows)
        assert carried["c_pad_mean"] == pytest.approx(study1["c_pad_mean"], abs=1e-6)


def test_the_summary_takes_the_best_finger_s_worst_retention_over_the_family(references):
    reference = references["code_matched"]
    rows = [{"a_length_mm": a, "a_thickness_mm": 5.0, "b_length_mm": b, "b_thickness_mm": 5.0, "rho": rho}
            for a, b, rho in ((20.0, 20.0, 1.0), (20.0, 25.0, 0.5), (20.0, 40.0, 0.0),
                              (25.0, 20.0, 0.9), (25.0, 25.0, 0.95), (25.0, 40.0, 0.0))]
    found = reopt.summarize(rows, reference, 120.0, "length", {"length_mm": [20, 25]}, 2)
    assert found["rho_one"] == 0.9 and found["one_finger_length_mm"] == 25.0  # 40 mm is outside the family
    assert found["corrections"] == 2 and found["rho_above_one_left"] == 0


def test_the_presets_set_the_sweeps_and_the_budgets():
    config = read_config_file(STUDY2 / "reoptimize.yaml")
    quick, full = reopt.resolve(config, "quick"), reopt.resolve(config)
    assert quick["directions"] == ["up"] and full["directions"] == ["up", "down"]
    assert quick["search"]["generations"] < full["search"]["generations"]
    assert reopt.resolve({"search": {}})["search"] == {}  # no presets: as is
    with pytest.raises(ValueError):
        reopt.resolve(config, "convergence")


def test_the_two_studies_differ_in_the_finger_length_and_the_cross_evaluation():
    config = read_config_file(STUDY2 / "reoptimize.yaml")
    a, b = reopt.resolve(config, study="2a"), reopt.resolve(config, study="2b")
    assert a["finger_length_mm"] == [60, 120] and not a["cross_evaluation"]
    assert b["finger_length_mm"] is None and b["cross_evaluation"]
    assert a["output"] != b["output"]
    with pytest.raises(ValueError):
        reopt.resolve(config, study="3")


def test_the_tip_path_is_the_motion_whatever_the_mechanism(references):
    for name in ("code_matched", "2r_dd_tilt0_matched", "belt_2r_matched", "rcj_tilt0_matched"):
        reference = references[name]
        path = tip_path(reference.mechanism, reference.x, reference.config, math.radians(90.0))
        assert path.shape == (61, 2)
        assert path[0] == pytest.approx((goal_of(reference.config).extended_length_bounds_mm[0], 0.0), abs=0.2)
        assert path_change(path, path, 90.0) == 0.0
    # A 2R's joint moved: the same finger length bends elsewhere, its tip goes elsewhere.
    reference = references["2r_dd_tilt0_matched"]
    names = mechanism(reference.mechanism).variable_names
    x = dict(zip(names, reference.x))
    moved = dict(x, a_x_mm=x["a_x_mm"] - 9.0, ac_length_mm=x["ac_length_mm"] + 9.0)
    a = tip_path(reference.mechanism, reference.x, reference.config, math.radians(90.0))
    b = tip_path(reference.mechanism, tuple(moved[n] for n in names), reference.config, math.radians(90.0))
    assert path_change(b, a, 90.0) == pytest.approx(9.0 * math.sqrt(2.0) / 90.0, rel=1e-3)  # at 90 deg: (9, 9) apart
    with pytest.raises(ValueError):
        ClosestMechanism(mechanism(reference.mechanism), reopt.shared.partitions.partition(reference.mechanism),
                         {}, {}, 90.0)  # closest in motion needs the reference's path


def test_the_grid_is_swept_in_a_snake_each_point_after_a_neighbour():
    grid = reopt.points("grid", {"length_mm": [15, 20, 25], "thickness_mm": [3, 5]})
    assert grid == [(15.0, 3.0), (20.0, 3.0), (25.0, 3.0), (25.0, 5.0), (20.0, 5.0), (15.0, 5.0)]
    steps = [(abs(a[0] - b[0]), abs(a[1] - b[1])) for a, b in zip(grid, grid[1:])]
    assert all((dl == 0) != (dh == 0) for dl, dh in steps)  # one dimension at a time
    family = {"length_mm": [15, 20], "thickness_mm": [3, 3]}
    assert reopt.in_family("grid", (20.0, 3.0), family) and not reopt.in_family("grid", (20.0, 5.0), family)
