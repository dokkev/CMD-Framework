"""The paper's comparison study (scripts/study/): every file loads under its
mechanism, on the one goal its base sets."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from core.optimization.linkage.mechanism import resolve_preset
from core.optimization.mechanism import mechanism, read_config_file

STUDY = Path(__file__).resolve().parents[1] / "scripts" / "study"
FILES = sorted(path for path in STUDY.glob("*.yaml") if path.name != "base.yaml")


def load(path: Path):
    values = read_config_file(path)
    found = mechanism(values.pop("mechanism"))
    assert values.pop("output") == "output/study"
    assert resolve_preset(values) == "search"
    values, _ = found.split_settings(values)
    return found, found.config_from_mapping(values)


def test_the_study_compares_the_code_the_2r_dd_and_the_rcj():
    assert ({mechanism(read_config_file(path)["mechanism"]).name for path in FILES}
            == {"linkage", "linkage_idealized", "revolute", "revolute_parallel", "rcj", "rcj_idealized"})


@pytest.mark.parametrize("path", FILES, ids=[path.stem for path in FILES])
def test_each_file_loads_on_the_one_goal(path):
    found, config = load(path)
    goal = config if found.name.startswith("linkage") else config.goal
    assert goal.extended_length_bounds_mm == (89.95, 90.05)  # 90 for every architecture
    assert goal.required_rom_rad == pytest.approx(math.radians(120.0))
    assert goal.warm_start == ()
    # Each pad's depth set by the study's policy, the two independent; Study profile defaults occlusion to False.
    assert goal.mount_policy == "shallowest_supported" and not goal.align_contact_faces
    assert goal.profile == "study" and goal.include_linkage_occlusion is False


def test_the_study_does_not_follow_the_production_config():
    for path in [STUDY / "base.yaml", *FILES]:
        chain = path
        while True:  # each file's base, up to the study's own baseline
            base = yaml.safe_load(chain.read_text()).get("base")
            if base is None:
                break
            chain = (chain.parent / base).resolve()
            assert chain.parent == STUDY.resolve(), f"{path.name} extends {chain} outside scripts/study"


def test_the_studies_run_native_references_of_every_architecture():
    import yaml

    root = STUDY.parent
    for study in ("study1_fixed_morphology", "study2_morphology_adaptation"):
        values = read_config_file(STUDY / study / "config.yaml")
        assert values["kinds"] == ["native"] and values["references"].endswith("references_native.yaml")
    native = yaml.safe_load((STUDY / "study2_morphology_adaptation" / "references_native.yaml").read_text())["native"]
    # Each a study config of its own (the CFL's is not the prototype's) and a portfolio under output/study.
    # Each architecture's own design for each required flexion, evaluated at it alone.
    assert {spec["id"] for spec in native} == {f"{name}_native_q{q}" for q in (90, 120, 150) for name in
                                               ("cfl", "2r_dd", "2r_remote", "rcj", "cfl_ideal", "rcj_ideal")}
    assert all(spec["id"].endswith(f"_q{spec['required_flexion_deg']}") for spec in native)
    assert all(spec["portfolio"].split("/")[-2].endswith(f"_q{spec['required_flexion_deg']}") for spec in native)
    for spec in native:
        assert spec["config"].startswith("scripts/study/") and spec["portfolio"].startswith("output/study/")
        assert (root.parent / spec["config"]).exists()


def test_the_2r_remote_servo_is_fixed_proximal_of_its_joint_and_still_comparable():
    import importlib.util
    import sys

    spec = importlib.util.spec_from_file_location("study_shared", STUDY / "shared.py")
    shared = importlib.util.module_from_spec(spec)
    sys.modules["study_shared"] = shared  # its dataclasses look their module up
    spec.loader.exec_module(shared)
    _, remote = load(STUDY / "2r_remote.yaml")
    _, cfl = load(STUDY / "cfl.yaml")
    assert remote.bounds["ap_from_a_x_mm"] == (-10.0, -10.0)  # A' fixed, proximal of the joint
    assert remote.bounds["ap_from_a_z_mm"] == (15.0, 15.0)
    # Where the servo sits is actuation (A.6.1), not a shared rule.
    rules = shared.comparison_rules(remote), shared.comparison_rules(cfl)
    assert rules[0]["bounds"] == rules[1]["bounds"] and rules[0]["relative_bounds"] == rules[1]["relative_bounds"]


def test_the_study_keeps_the_cfls_a_pin_off_the_mcp_servo():
    from core.morphology.mechanism.linkage.packaging import layout

    for name in ("cfl.yaml", "cfl_ideal.yaml"):
        _, config = load(STUDY / name)
        assert config.packaging.a_pin_mcp_check  # A.4: only a pivot and its own actuator may overlap
        assert layout(config.packaging).pair("a_pin", "mcp_servo") is not None


def test_a_native_reference_may_fix_some_coordinates_of_its_found_design():
    import importlib.util
    import sys

    spec = importlib.util.spec_from_file_location("study_shared_design", STUDY / "shared.py")
    shared = importlib.util.module_from_spec(spec)
    sys.modules["study_shared_design"] = shared
    spec.loader.exec_module(shared)
    found = read_config_file(STUDY / "study2_morphology_adaptation" / "references_native.yaml")["native"]
    fixed = {s["id"]: s["design"] for s in found if s.get("design")}
    # The 2R-remote's servo pose, fixed after the search, matches its study file.
    assert set(fixed) == {f"2r_remote_native_q{q}" for q in (90, 120, 150)}
    _, remote = load(STUDY / "2r_remote.yaml")
    for design in fixed.values():
        assert remote.bounds["ap_from_a_x_mm"] == (design["ap_from_a_x_mm"],) * 2
        assert remote.bounds["ap_from_a_z_mm"] == (design["ap_from_a_z_mm"],) * 2
        assert remote.bounds["servo_tilt_deg"] == (design["servo_tilt_deg"],) * 2
