"""A fixed finger with pads of other thicknesses (problem formulation A.6.0, 2026-11-20)."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

from core.optimization.analysis.one_finger import coverage, fitted, mount
from core.optimization.analysis.tradeoff import goal_of
from core.optimization.mechanism import mechanism, read_config_file

ROOT = Path(__file__).resolve().parents[1]
STUDY2 = ROOT / "scripts" / "study" / "study2_morphology_adaptation"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


driver = _load("study2_one_finger", STUDY2 / "one_finger.py")
shared = driver.shared


@pytest.fixture(scope="module")
def references():
    """The matched references: built from the tracked design vector alone (native portfolios are not tracked)."""

    return {r.reference_id: r for r in shared.load_references(
        ("matched",), ROOT, "scripts/study/study2_morphology_adaptation/references_nominal15.yaml")}


def _parts(reference):
    return mechanism(reference.mechanism), shared.partitions.partition(reference.mechanism)


def test_the_pads_fill_the_fingers_own_supports_whatever_their_thickness(references):
    for name in ("code_matched", "2r_dd_tilt0_matched", "rcj_tilt0_matched"):
        reference = references[name]
        _, part = _parts(reference)
        thin, thick = (fitted(part, reference.x, reference.config, h) for h in (3.0, 9.0))
        assert (thin.proximal_length_mm, thin.distal_length_mm) == (thick.proximal_length_mm, thick.distal_length_mm)
        assert (thin.proximal_height_mm, thick.distal_height_mm) == (3.0, 9.0)
        external = part.external(reference.x, reference.config)
        assert 0.0 <= external["distal_span_mm"] - thin.distal_length_mm < 1.1e-3  # never past the support


def test_a_finger_shorter_of_support_than_the_goals_least_pads_carries_nothing(references):
    import dataclasses

    from core.optimization.analysis.tradeoff import with_goal

    reference = references["code_matched"]
    base, part = _parts(reference)
    asked = with_goal(reference.config, dataclasses.replace(goal_of(reference.config), minimum_distal_support_mm=80.0))
    assert fitted(part, reference.x, asked, 5.0) is None
    assert mount(base, part, reference.x, asked, 5.0) == (None, None)


def test_mounting_a_mounted_finger_again_changes_nothing(references):
    # The one finger's candidates include Study 1's designs: a design with its own pads keeps its own
    # coverage (checked on the native designs when the study runs; here on the tracked references).
    checked = 0
    for name in ("code_matched", "2r_dd_tilt0_matched", "rcj_tilt0_matched", "belt_2r_matched"):
        reference = references[name]
        base, part = _parts(reference)
        for thickness in (3.0, 4.0, 5.0):
            composed, evaluation = mount(base, part, reference.x, reference.config, thickness)
            if composed is None or evaluation is None:
                continue  # these pads do not mount on this reference
            again, repeated = mount(base, part, composed, reference.config, thickness)
            assert again == pytest.approx(composed, abs=2e-3)  # the pads' length down to a micrometre, once more
            assert bool(repeated.feasible) == bool(evaluation.feasible)
            assert coverage(base, repeated) == pytest.approx(coverage(base, evaluation), abs=1e-3)
            checked += 1
    assert checked >= 2
    assert coverage(base, None) == 0.0


def test_the_config_declares_the_conditions_and_the_architectures():
    config = read_config_file(STUDY2 / "one_finger.yaml")
    assert config["native_thickness_mm"] in config["thickness_mm"]
    assert set(config["architectures"]) == {"cfl", "2r_dd", "2r_remote", "rcj", "cfl_ideal", "rcj_ideal"}
    for spec in config["architectures"].values():
        assert (ROOT / spec["config"]).exists()
    folder = driver.condition_folder("cfl", config["architectures"]["cfl"], 120.0, 5.0, 5.0)
    assert folder.name == "cfl_q120" and folder.parent.name == "linkage"
    assert driver.condition_folder("cfl", config["architectures"]["cfl"], 90.0, 7.0, 5.0).name == "cfl_q90_h7"


def test_every_design_is_mounted_with_every_thickness(references):
    reference = references["code_matched"]
    designs = {3.0: reference, 5.0: reference}  # the same design standing for both thicknesses' own
    cross = driver.Study("cfl", 120.0, designs).cross()
    assert len(cross) == 4 and {r["designed_for_mm"] for r in cross} == {3.0, 5.0}
    assert {r["thickness_mm"] for r in cross} == {3.0, 5.0}
    for row in cross:
        assert row["carried"] == (row["c_pad_mean"] > 0.0) and 0.0 <= row["c_pad_mean"] <= 1.0
