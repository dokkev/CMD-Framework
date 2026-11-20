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
    assert goal.extended_length_bounds_mm == (80.0, 90.0)
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
    assert {spec["id"] for spec in native} == {"cfl_native", "2r_dd_native", "2r_remote_native", "rcj_native",
                                               "cfl_ideal_native", "rcj_ideal_native"}
    for spec in native:
        assert spec["config"].startswith("scripts/study/") and spec["portfolio"].startswith("output/study/")
        assert (root.parent / spec["config"]).exists()
