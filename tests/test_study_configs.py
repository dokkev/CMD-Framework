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
FILES = sorted(path for path in STUDY.glob("*.yaml") if path.name not in ("base.yaml", "baseline.yaml"))


def load(path: Path):
    values = read_config_file(path)
    found = mechanism(values.pop("mechanism"))
    assert values.pop("output") == "output/study"
    assert resolve_preset(values) == "search"
    values, _ = found.split_settings(values)
    return found, found.config_from_mapping(values)


def test_the_study_compares_the_code_and_the_2r_dd():
    assert {mechanism(read_config_file(path)["mechanism"]).name for path in FILES} == {"linkage", "revolute"}


@pytest.mark.parametrize("path", FILES, ids=[path.stem for path in FILES])
def test_each_file_loads_on_the_one_goal(path):
    found, config = load(path)
    goal = config if found.name == "linkage" else config.goal
    assert goal.extended_length_bounds_mm == (80.0, 90.0)
    assert goal.required_rom_rad == pytest.approx(math.radians(120.0))
    assert goal.warm_start == ()
    # Each pad's depth set by the study's policy, the two independent.
    assert goal.mount_policy == "shallowest_supported" and not goal.align_mount_offsets


def test_the_study_does_not_follow_the_production_config():
    for path in [STUDY / "base.yaml", *FILES]:
        chain = path
        while True:  # each file's base, up to the study's own baseline
            base = yaml.safe_load(chain.read_text()).get("base")
            if base is None:
                break
            chain = (chain.parent / base).resolve()
            assert chain.parent == STUDY.resolve(), f"{path.name} extends {chain} outside scripts/study"
