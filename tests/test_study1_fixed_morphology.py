"""The contact accommodation study (scripts/study/study1_fixed_morphology/run.py):
frozen cores carrying module pairs, on two small fixture cores."""

from __future__ import annotations

import csv
import dataclasses
import importlib.util
import json
import sys
from pathlib import Path

import pytest
import yaml

from core.optimization.analysis.contact_envelope import evaluate_fit
from core.optimization.analysis.tradeoff import Modules, goal_of, with_modules

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("study1_fixed_morphology_run",
                                              ROOT / "scripts" / "study" / "study1_fixed_morphology" / "run.py")
study = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = study  # dataclasses look their module up
spec.loader.exec_module(study)

RULES = ["goal.link.finger_length_mm=[90, 110]", "goal.pads.proximal.length_mm=[20, null]",
         "goal.pads.distal.length_mm=[25, null]", "goal.collision.check.a_pin_and_pip_servo=false",
         "goal.collision.check.a_pin_and_a_prime_pin=true", "goal.hardware.pip_servo.tilt_angle_deg=10",
         "goal.hardware.pip_servo.tilt_step_deg=1", "goal.pads.proximal.past_icr_max_mm=null",
         "goal.link.distal_length_mm=[25, null]", "goal.link.joint_span_ratio=[null, null]",
         "goal.link.proximal_length_ratio=[null, null]", "goal.link.distal_length_ratio=[null, null]",
         "goal.pads.on_links=false", "goal.motion.dip_flexion_deg=120"]
"""One set of physical rules for both fixture cores."""
CODE_X = [40.6, 66.0, 0.0, 44.4, 12.1, 56.9, 6.0, 10.0, 33.9, 0.0, -7.0, -12.1, 0.0, 3.0]
"""The linkage study's fixture design, its distal pad mounted 3 mm below BC."""
DD_X = [45.0, 55.0, 30.0, 29.0, 4.0, 4.0, 90.0]
"""The 2R-DD fixture design, both pads mounted 4 mm below their segments' lines."""
CORES = [{"id": "code", "mechanism": "linkage", "config": "scripts/optimize/linkage/config.yaml",
          "overrides": RULES, "x": CODE_X},
         {"id": "dd", "mechanism": "revolute", "config": "scripts/optimize/linkage/config.yaml",
          "overrides": RULES, "x": DD_X}]
MODULES = {"baseline": {"proximal": {"length_mm": 20, "thickness_mm": 3},
                        "distal": {"length_mm": 20, "thickness_mm": 3}},
           "modules": {"baseline": {}, "too_long": {"proximal": {"length_mm": 80}},
                       "thick": {"proximal": {"thickness_mm": 16}, "distal": {"thickness_mm": 16}}},
           "sweeps": {"distal": {"sides": ["distal"], "length_mm": [15, 25], "thickness_mm": [3, 8]}}}


def write_study(folder: Path, cores=CORES, modules=MODULES) -> Path:
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "cores.yaml").write_text(yaml.safe_dump({"cores": cores}))
    (folder / "modules.yaml").write_text(yaml.safe_dump(modules))
    config = folder / "config.yaml"
    config.write_text(yaml.safe_dump({"cores": str(folder / "cores.yaml"), "modules": str(folder / "modules.yaml"),
                                      "required_flexion_deg": [90, 120], "max_flexion_limit_deg": 180,
                                      "output": str(folder / "out")}))
    return config


@pytest.fixture(scope="module")
def results(tmp_path_factory):
    folder = tmp_path_factory.mktemp("study")
    output = study.run(write_study(folder), ROOT)
    with (output / "assembly_results.csv").open(encoding="utf-8", newline="") as stream:
        return output, list(csv.DictReader(stream))


def test_every_module_pair_is_evaluated_on_both_mechanisms(results):
    _, rows = results
    by_mechanism = {}
    for row in rows:
        by_mechanism.setdefault(row["mechanism"], set()).add((row["module_id"], row["required_flexion_deg"]))
    assert set(by_mechanism) == {"linkage", "revolute"}
    assert by_mechanism["linkage"] == by_mechanism["revolute"]
    assert len(by_mechanism["linkage"]) == (3 + 4) * 2


def test_a_module_changes_neither_the_core_nor_its_mounting_depths():
    cores = [study.load_core(spec, ROOT) for spec in CORES]
    frozen = [dataclasses.replace(core) for core in cores]
    catalog = study.module_catalog(MODULES)
    study.assemblies(cores, catalog, [120.0], 180.0)
    assert cores == frozen  # the design vectors and their configs, as loaded
    reference = catalog[0].modules
    assert study.mounting_depths(cores[0], reference) == (0.0, 3.0)
    assert study.mounting_depths(cores[1], reference) == (4.0, 4.0)
    for core in cores:  # whatever the module carried
        depths = study.mounting_depths(core, reference)
        assert all(study.mounting_depths(core, pair.modules) == depths for pair in catalog)


def test_the_rows_carry_the_cores_depths_below_the_modules(results):
    _, rows = results
    expected = {"code": (0.0, 3.0), "dd": (4.0, 4.0)}
    for row in rows:
        proximal, distal = expected[row["core_id"]]
        assert (float(row["proximal_mount_depth_mm"]), float(row["distal_mount_depth_mm"])) == (proximal, distal)
        assert float(row["proximal_contact_depth_mm"]) == pytest.approx(proximal + float(row["proximal_thickness_mm"]))
        assert float(row["distal_contact_depth_mm"]) == pytest.approx(distal + float(row["distal_thickness_mm"]))


def test_the_collision_checks_use_the_modules_dimensions_on_the_mounting_depth():
    from core.morphology.mechanism.linkage.design import decode
    from core.morphology.mechanism.linkage.packaging import distal_pad_local

    code = study.load_core(CORES[0], ROOT)
    finger = decode(code.x, code.config.link_length_resolution_mm)
    for thickness in (3.0, 16.0):
        config = with_modules(code.config, Modules(20.0, thickness, 20.0, thickness))
        pad = distal_pad_local(finger, goal_of(config).packaging)
        assert pad[:, 1].max() == pytest.approx(-3.0) and pad[:, 1].min() == pytest.approx(-3.0 - thickness)
    # The thin pair clears the fold at 120 deg; the thick one, on the same core, does not.
    thin = evaluate_fit("linkage", code.x, code.config, Modules(20.0, 3.0, 20.0, 3.0), 120.0)
    thick = evaluate_fit("linkage", code.x, code.config, Modules(20.0, 16.0, 20.0, 16.0), 120.0)
    assert thin.feasible and not thick.feasible
    assert thick.collision_pair == "distal_pad/proximal_pad" and thick.first_failure_deg < 120.0


def test_the_maximum_flexion_is_reported_for_feasible_and_failing_assemblies(results):
    _, rows = results
    rows = {row["assembly_id"]: row for row in rows}
    fits = rows["code__baseline__phi120"]
    assert fits["feasible"] == "True" and float(fits["max_safe_flexion_deg"]) >= 120.0
    assert fits["first_failure_deg"] == "" and float(fits["contact_coverage"]) > 0.0
    fails = rows["code__thick__phi120"]
    assert fails["feasible"] == "False" and fails["reason"] == "pad_pad_collision"
    assert float(fails["max_safe_flexion_deg"]) == pytest.approx(float(fails["first_failure_deg"]), abs=0.1)
    assert float(fails["max_safe_flexion_deg"]) < 120.0 and fails["contact_coverage"] == ""
    # The same thick pair still fits through 90 deg.
    assert rows["code__thick__phi90"]["feasible"] == "True"


def test_a_module_that_does_not_mount_is_reported_and_the_study_goes_on(results):
    _, rows = results
    too_long = [row for row in rows if row["module_id"] == "too_long"]
    assert too_long and all(row["reason"] == "support_exceeded" and row["mounted"] == "False"
                            and row["max_safe_flexion_deg"] == "" for row in too_long)
    assert any(row["feasible"] == "True" for row in rows)


def test_cores_under_different_physical_rules_are_not_compared(tmp_path):
    cores = [CORES[0], {**CORES[1], "overrides": [*RULES, "goal.collision.check.proximal_pad_and_distal_pad=false"]}]
    with pytest.raises(ValueError, match="different physical rules"):
        study.run(write_study(tmp_path, cores), ROOT)


def test_equal_inputs_give_equal_results(results, tmp_path):
    first, _ = results
    second = study.run(write_study(tmp_path), ROOT)
    for name in ("assembly_results.csv", "core_designs.csv", "module_catalog.csv"):
        assert (first / name).read_bytes() == (second / name).read_bytes()
    manifest = json.loads((second / "manifest.json").read_text())
    assert manifest["source"]["commit"] and manifest["required_flexion_deg"] == [90.0, 120.0]
    assert manifest["cores"]["cores"] == CORES and "pip_servo" in manifest["physical_rules"]


def test_a_module_the_cores_surfaces_cannot_carry_is_not_remounted(tmp_path):
    # A core built under the study's policy: its distal pad under B, 3 mm deep.
    policy = [*RULES, "goal.pads.mount_policy=shallowest_supported"]
    core = {"id": "code_b", "mechanism": "linkage", "config": "scripts/optimize/linkage/config.yaml",
            "overrides": policy, "x": [*CODE_X[:11], -5.0, 0.0, 3.0]}
    modules = {"baseline": {"proximal": {"length_mm": 20, "thickness_mm": 3},
                            "distal": {"length_mm": 35, "thickness_mm": 3}},
               "modules": {"under_b": {}, "thick_under_b": {"distal": {"thickness_mm": 6}},
                           "short_of_b": {"distal": {"length_mm": 20}}}}
    output = study.run(write_study(tmp_path, [core], modules), ROOT)
    with (output / "assembly_results.csv").open(encoding="utf-8", newline="") as stream:
        rows = {row["module_id"]: row for row in csv.DictReader(stream) if row["required_flexion_deg"] == "90.0"}
    for row in rows.values():  # the core's depths, whatever the module
        assert (row["proximal_mount_depth_mm"], row["distal_mount_depth_mm"]) == ("0.0", "3.0")
        assert row["proximal_native_support_mm"] == "0.0"
    for name in ("under_b", "thick_under_b"):  # the same least depth: carried
        assert (rows[name]["proximal_mount_case"], rows[name]["distal_mount_case"]) == ("A", "B")
        assert rows[name]["distal_required_depth_mm"] == "3.0" and rows[name]["reason"] != "mount_incompatible"
    short = rows["short_of_b"]  # it would mount on the line: not remounted, no design on this core
    assert short["distal_required_depth_mm"] == "0.0" and short["distal_mount_case"] == "A"
    assert short["reason"] == "mount_incompatible" and short["feasible"] == "False"
    assert short["mounted"] == "False" and short["max_safe_flexion_deg"] == ""
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["mounting_policy"]["policy"] == ["shallowest_supported"]


def test_a_module_fits_a_frozen_surface_at_least_as_deep_as_it_needs_and_still_carried():
    from core.optimization.mounting import Mounting

    core_depths = (0.0, 3.0)
    # Needs 2.5 mm under a 3 mm housing: the core's 3 mm surface carries it.
    assert study.compatible(Mounting((0.0, 2.5), (0.0, 3.0)), core_depths)
    # Needs 4 mm: the core's surface is too shallow.
    assert not study.compatible(Mounting((0.0, 4.0), (0.0, 4.0)), core_depths)
    # Too short to reach the housing: the core's 3 mm surface hangs over nothing.
    assert not study.compatible(Mounting((0.0, 0.0), (0.0, 0.0)), core_depths)
    # A surface on the line needs no support.
    assert study.compatible(Mounting((0.0, 0.0), (0.0, 0.0)), (0.0, 0.0))
    assert not study.compatible(Mounting((None, 0.0), (0.0, 0.0)), (0.0, 0.0))


def test_repeated_cores_are_evaluated_once_and_recorded(tmp_path):
    cores = [CORES[0], {**CORES[0], "id": "code_again"}, CORES[1]]
    output = study.run(write_study(tmp_path, cores, {**MODULES, "sweeps": {}}), ROOT)
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["duplicate_cores"] == {"code_again": "code"} and manifest["counts"]["cores"] == 2


def test_the_fingertip_passing_behind_the_mcp_ends_the_safe_flexion():
    import math as _math

    from core.optimization.analysis.contact_envelope import max_flexion_deg

    dd = study.load_core(CORES[1], ROOT)
    # |OA| 45 < |AC| 55: C_x = 45 + 55 cos(phi) turns negative past 180 - acos(45 / 55), short of any collision.
    reach, fit = max_flexion_deg("revolute", dd.x, dd.config, Modules(15.0, 1.0, 15.0, 1.0))
    assert fit.reason == "tip_behind_mcp" and fit.collision_pair == "tip/mcp"
    assert reach == pytest.approx(180.0 - _math.degrees(_math.acos(45.0 / 55.0)), abs=0.3)
    assert evaluate_fit("revolute", dd.x, dd.config, Modules(15.0, 1.0, 15.0, 1.0), 120.0).feasible


CODE_TURNS_BACK = (52.5, 87.4, 0.0, 49.6, 12.3, 83.0, 4.8, 40.0, 12.5, 0.0, -0.1, 24.7, 0.0, 3.0)
"""A CoDe design whose flexion peaks at 167.6 deg on the servo's sweep and then falls back."""


def test_codes_motion_ends_where_its_four_bar_turns_back():
    import dataclasses
    import math as _math

    from core.optimization.analysis.contact_envelope import motion

    code = study.load_core(CORES[0], ROOT)
    # The optimizer's limits out of the way: the four-bar's own turn alone ends the motion.
    loose = dataclasses.replace(code.config, dip_backoff_tolerance_rad=_math.pi,
                                transmission_angle_range_rad=(0.0, _math.pi))
    found = motion("linkage", CODE_TURNS_BACK, loose, 180.0)
    assert not found.reached and found.stop == "turns_back"
    assert found.phi_deg.max() == pytest.approx(167.55, abs=0.01) and found.phi_deg[-1] == found.phi_deg.max()
    fit = evaluate_fit("linkage", CODE_TURNS_BACK, loose, Modules(20.0, 3.0, 20.0, 3.0), 180.0, found)
    assert fit.reason in ("kinematic_unreachable", "pad_pad_collision") and fit.first_failure_deg <= 167.56


def test_codes_motion_stops_at_the_optimizers_mechanism_limits():
    import math as _math

    from core.optimization.analysis.contact_envelope import max_flexion_deg
    from core.optimization.analysis.tradeoff import with_modules
    from core.optimization.mechanism import mechanism

    code = study.load_core(CORES[0], ROOT)
    thin = Modules(15.0, 1.0, 15.0, 1.0)
    reach, fit = max_flexion_deg("linkage", code.x, code.config, thin)
    assert fit.reason == "mechanism_limit" and fit.collision_pair == "dip_backoff"
    # The evaluator stops there too, on its coarser servo grid (a step short at most).
    found = mechanism("linkage").evaluate(code.x, with_modules(code.config, thin))
    assert found.stop_reason == "dip_backoff"
    assert 0.0 <= reach - _math.degrees(found.phi_safe_rad) <= 2.0
    # The design that turns back: its limit, not its return branch's collisions.
    reach, fit = max_flexion_deg("linkage", CODE_TURNS_BACK, code.config, Modules(20.0, 3.0, 20.0, 3.0))
    found = mechanism("linkage").evaluate(CODE_TURNS_BACK, with_modules(code.config, Modules(20.0, 3.0, 20.0, 3.0)))
    assert fit.reason == "mechanism_limit" and reach == pytest.approx(_math.degrees(found.phi_safe_rad), abs=0.5)


def test_a_role_is_found_among_the_roles_one_seed_holds(tmp_path):
    seeds = tmp_path / "seeds.csv"
    seeds.write_text("seed_role,run,candidate\nmost_pad;balanced,run_001,1\nmost_curl,run_001,2\n")
    assert study._row(seeds, "seed_role", "balanced", joined=True)["candidate"] == "1"
    assert study._row(seeds, "seed_role", "most_curl", joined=True)["candidate"] == "2"
    with pytest.raises(ValueError, match="no row"):
        study._row(seeds, "seed_role", "best_transmission", joined=True)


def test_the_manifest_lists_the_link_checks_only_when_they_run(results):
    output, _ = results
    manifest = json.loads((output / "manifest.json").read_text())
    rules, policy = manifest["physical_rules"], manifest["collision_policy"]
    assert {"link_radius_mm", "mcp_samples_rad", "align_mount_offsets", "resolution_mm"} <= rules.keys()
    listed = any("link_radius_mm" in item for item in policy["checked_in_xz_projection"])
    assert listed == (rules["link_radius_mm"] is not None)
    assert study.collision_policy(2.0)["checked_in_xz_projection"] != study.collision_policy(None)[
        "checked_in_xz_projection"]
