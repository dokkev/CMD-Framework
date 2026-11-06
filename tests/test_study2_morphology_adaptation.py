"""Study 2, morphology adaptation (core.optimization.analysis.adaptation and
scripts/study/study2_morphology_adaptation): each level changes only what it
may, through the mechanisms' own evaluation and mounting policy."""

from __future__ import annotations

import csv
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from core.optimization.analysis import adaptation
from core.optimization.analysis.adaptation import DESCRIPTORS, AdaptationMechanism, Step, adapt
from core.optimization.analysis.tradeoff import Modules, goal_of, with_modules
from core.optimization.mechanism import mechanism
from core.optimization.mounting import mounting
from core.optimization.optimizer import OptimizerSettings

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "scripts" / "study" / "study2_morphology_adaptation"


def _module(name: str, path: Path):
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


run = _module("study2_run", STUDY / "run.py")
partitions = run.partitions
BASELINE = Modules(20.0, 3.0, 20.0, 3.0)
LONG_THICK = Modules(35.0, 8.0, 35.0, 8.0)
PROTOTYPE = (35.7, 67.2, 0.0, 37.5, 12.4, 60.5, 6.5, 10.0, 22.8, 0.0, 5.8, 0.7, 3.0, 3.0)
"""The current prototype (report/prototype.md)."""
E0 = {"finger_length_mm": 90.0, "proximal_span_mm": 43.3, "distal_span_mm": 30.2}


def matched(config_file: str, modules: Modules = BASELINE):
    """A mechanism's matched reference (the prototype's morphology) and its config carrying ``modules``."""

    found, config = run.load_config(f"scripts/study/{config_file}.yaml", ROOT)
    part = partitions.partition(found.name)
    carried = with_modules(config, modules)
    external = partitions.partition("linkage").external(PROTOTYPE, carried)
    coordinates, template = part.template(external, "linkage", PROTOTYPE, carried)
    return found, part, part.compose(external, coordinates, template, carried), carried


CONFIGS = ("code_tilt10", "2r_dd_tilt0", "2r_dd_tilt90", "belt_2r_tilt10", "rcj_tilt0", "rcj_tilt90", "rcj_ideal")


@pytest.mark.parametrize("name", CONFIGS)
def test_every_matched_reference_has_the_prototypes_morphology_and_carries_the_baseline(name):
    found, part, x, config = matched(name)
    external = part.external(x, config)
    for key, value in E0.items():
        assert external[key] == pytest.approx(value, abs=1e-9)
    assert external["articulation_mm"] == pytest.approx(55.0519, abs=1e-3)
    assert found.evaluate(x, config).feasible


def test_codes_matched_reference_is_the_prototype_its_depths_the_policys():
    found, part, x, config = matched("code_tilt10")
    assert x[:-2] == pytest.approx(PROTOTYPE[:-2])
    assert x[-2:] == mounting("linkage", x, config).depths_mm


@pytest.mark.parametrize("name", ("code_tilt10", "rcj_tilt90"))
def test_level_1_moves_the_mechanism_and_keeps_the_morphology_and_actuation_exactly(name):
    found, part, x, config = matched(name)
    level1 = AdaptationMechanism(found, part, x, config, "mechanism", "mechanism_cost", 90.0)
    lower, upper = level1.search_box(config)
    rng = np.random.default_rng(0)
    built = 0
    for _ in range(300):
        design = level1.to_design(rng.uniform(lower, upper), config)
        if design is None:
            continue
        built += 1
        external = part.external(design, config)
        assert all(external[key] == pytest.approx(level1.reference_external[key], abs=1e-6) for key in DESCRIPTORS)
        values, reference = part.values(design), part.values(x)
        assert all(values[key] == reference[key] for key in part.actuation)
        # Its mounting depths are the policy's, for this interface (never searched).
        assert tuple(values[slot] for slot in part.depths) == mounting(found.name, design, config).depths_mm
    assert built > 20


def test_the_2r_has_no_mechanism_freedom_once_its_morphology_is_fixed():
    found, part, x, config = matched("2r_dd_tilt90")
    level1 = AdaptationMechanism(found, part, x, config, "mechanism", "mechanism_cost", 90.0)
    assert level1.search_names(config) == ()
    step = adaptation._search(found, part, x, config, OptimizerSettings("nsga2", 8, 1), "level1", "mechanism",
                              "mechanism_cost", 90.0)
    assert step.status == "no_freedom"


def test_level_2_moves_the_morphology_with_the_actuation_held():
    found, part, x, config = matched("belt_2r_tilt10")
    level2 = AdaptationMechanism(found, part, x, config, "external", "external_cost", 90.0, 15.0)
    lower, upper = level2.search_box(config)
    rng = np.random.default_rng(1)
    reference = part.values(x)
    servo = reference["a_x_mm"] + reference["ap_from_a_x_mm"]
    moved = 0
    for _ in range(300):
        design = level2.to_design(rng.uniform(lower, upper), config)
        if design is None:
            continue
        values = part.values(design)
        assert values["servo_tilt_deg"] == reference["servo_tilt_deg"]
        assert values["ap_from_a_z_mm"] == reference["ap_from_a_z_mm"]
        assert values["a_x_mm"] + values["ap_from_a_x_mm"] == pytest.approx(servo)  # the servo stays put
        moved += part.external(design, config)["articulation_mm"] != pytest.approx(55.0519, abs=1e-3)
    assert moved > 20


def test_the_interface_is_the_configs_and_the_evaluation_the_mechanisms_own():
    found, part, x, config = matched("code_tilt10", LONG_THICK)
    goal = goal_of(config)
    assert (goal.packaging.proximal_pad.length_mm, goal.packaging.proximal_pad.height_mm) == (35.0, 8.0)
    adapted = AdaptationMechanism(found, part, x, config, "external", "external_cost", 90.0, 15.0)
    assert not set(adapted.search_names(config)) & {"proximal_length_mm", "thickness_mm", "length_mm"}
    own, wrapped = found.evaluate(x, config), adapted.evaluate(x, config)
    assert wrapped.base.feasible == own.feasible and tuple(wrapped.g[:-1]) == tuple(own.g)
    assert wrapped.external_cost == 0.0 and wrapped.mechanism_cost == 0.0


def test_changes_are_the_largest_normalized_change_of_their_group_never_summed():
    assert adaptation.change({"a": 92.0, "b": 40.0}, {"a": 90.0, "b": 43.0}, 90.0) == pytest.approx(3.0 / 90.0)
    assert adaptation.change({}, {}, 90.0) == 0.0


class _Recorder:
    """A stand-in for the searches: records each call, answers as told."""

    def __init__(self, answers):
        self.answers, self.calls = dict(answers), []

    def __call__(self, base, partition, reference, config, settings, level, free, objective, scale_mm,
                 external_range_mm=0.0, external_cap=None, warm=(), workers=1):
        self.calls.append({"level": level, "free": free, "objective": objective, "cap": external_cap,
                           "config": config})
        return self.answers[level]


def _found(level, external_cost, mechanism_cost=0.0):
    evaluation = adaptation.AdaptationEvaluation(None, {}, {}, external_cost, mechanism_cost)
    return Step(level, "found", (0.0,), evaluation)


def test_level_2_runs_only_when_level_1_finds_nothing_and_minimizes_the_morphology_first(monkeypatch):
    found, part, x, config = matched("code_tilt10", LONG_THICK)
    settings = OptimizerSettings("nsga2", 8, 1)
    # Level 1 finds a design: Level 2 never runs.
    recorder = _Recorder({"level1": _found("level1", 0.0, 0.05)})
    monkeypatch.setattr(adaptation, "_search", recorder)
    result = adapt(found, part, x, config, settings, 90.0, 15.0)
    assert result.status == "level1" and [c["level"] for c in recorder.calls] == ["level1"]
    # It does not: the morphology's least change, then the mechanism's within it.
    recorder = _Recorder({"level1": Step("level1", "not_found"), "level2_external": _found("level2_external", 0.1),
                          "level2_mechanism": _found("level2_mechanism", 0.1, 0.02)})
    monkeypatch.setattr(adaptation, "_search", recorder)
    result = adapt(found, part, x, config, settings, 90.0, 15.0)
    assert [c["level"] for c in recorder.calls] == ["level1", "level2_external", "level2_mechanism"]
    first, second = recorder.calls[1], recorder.calls[2]
    assert (first["objective"], second["objective"]) == ("external_cost", "mechanism_cost")
    assert first["cap"] is None and second["cap"] == pytest.approx(0.1 + 0.1 / 90.0)
    assert goal_of(first["config"]).extended_length_bounds_mm == pytest.approx((75.0, 105.0))
    assert result.status == "level2" and result.final.level == "level2_mechanism"


def test_a_reference_that_carries_the_interface_stops_at_once(monkeypatch):
    found, part, x, config = matched("code_tilt10")
    recorder = _Recorder({})
    monkeypatch.setattr(adaptation, "_search", recorder)
    result = adapt(found, part, x, config, OptimizerSettings("nsga2", 8, 1), 90.0, 15.0)
    assert result.status == "frozen_feasible" and not recorder.calls
    assert result.mechanism_cost == result.external_cost == 0.0


def test_the_same_reference_and_target_give_the_same_result(tmp_path):
    config = tmp_path / "config.yaml"
    config.write_text((STUDY / "config.yaml").read_text().replace(
        "output: output/study/study2_morphology_adaptation", f"output: {tmp_path / 'out'}").replace(
        "population: 100", "population: 12").replace("generations: 100", "generations: 3").replace(
        "workers: 8", "workers: 1"))
    rows = []
    for _ in range(2):
        out = run.run(config, ROOT, only_targets=["long"], only_references=["rcj_tilt90_matched"])
        with (out / "summary.csv").open() as stream:
            rows.append(list(csv.DictReader(stream)))
    assert rows[0] == rows[1] and rows[0][0]["status"] in ("level1", "level2", "not_found")
    assert math.isfinite(float(rows[0][0]["final_external_cost"] or "nan")) or rows[0][0]["status"] == "not_found"


@pytest.mark.parametrize("name", ("belt_2r_tilt10", "rcj_tilt90", "2r_dd_tilt90", "code_tilt10"))
def test_a_module_longer_than_its_segment_does_not_mount(name):
    # The matched references' distal span is 30.2 mm: a 45 mm distal module overhangs it.
    found, part, x, config = matched(name, Modules(15.0, 3.0, 45.0, 8.0))
    if x is None:  # its supports cannot carry it at any depth: no design at all
        return
    evaluation = found.evaluate(x, config)
    assert not evaluation.feasible and evaluation.constraints["distal_support_mm"] > 0.0
    # Evaluated as the matched designs are built, with no support check: a room check of its own.
    reference = matched(name)[2]
    assert part.external(reference, config)["distal_span_mm"] < 45.0
