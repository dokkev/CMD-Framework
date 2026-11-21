"""Contact placement on a fixed CFL linkage from its pole path (experimental; A.6.0)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

from core.optimization.analysis.placement import ARRANGEMENT, Linkage, PlacedMechanism, place
from core.optimization.mechanism import mechanism

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "scripts" / "study"
if str(STUDY) not in sys.path:
    sys.path.insert(0, str(STUDY))

import shared  # noqa: E402


@pytest.fixture(scope="module")
def cfl():
    """A CFL that is a design of its config: the tracked reference with its pads placed."""

    reference = {r.reference_id: r for r in shared.load_references(
        ("matched",), ROOT, "scripts/study/study2_morphology_adaptation/references_nominal15.yaml")}["code_matched"]
    base = mechanism(reference.mechanism)
    design, evaluation, _ = place(base, reference.x, reference.config)
    if design is None:
        pytest.skip("the tracked reference has no feasible pad arrangement under its config")
    return base, design, evaluation, reference.config


def _depths(base, evaluation):
    report = base.report(evaluation)
    return float(report["proximal_depth_mm"]), float(report["distal_depth_mm"])


def test_the_model_is_the_evaluators_coverage_and_pad_clearance(cfl):
    base, design, evaluation, config = cfl
    link = Linkage(base, design, config)
    depths = _depths(base, evaluation)
    own = dict(zip(base.variable_names, design))
    rng, checked = np.random.default_rng(0), 0
    for _ in range(30):
        app = round(own[ARRANGEMENT[0]] + rng.uniform(-2, 2), 1)
        bpp = round(own[ARRANGEMENT[1]] + rng.uniform(-2, 2), 1)
        trial = link.design(app, bpp)
        if trial is None:
            continue
        found = base.evaluate(trial, config)
        if _depths(base, found) != depths:
            continue  # the mounting rule gave this arrangement other depths: another model
        clearance, cover = link.grid(np.array([app]), np.array([bpp]), *depths)
        report = base.report(found)
        if found.feasible:
            assert cover[0, 0] == pytest.approx(float(report["c_pad_mean"]), abs=1e-9)
            assert clearance[0, 0] >= -1e-9
            checked += 1
        elif clearance[0, 0] < -1e-9:  # the model's overlap is the evaluator's pad collision
            assert float(report["pad_clearance_mm"]) < 0.0
    assert checked >= 3


def test_coarse_to_fine_finds_what_the_exhaustive_search_finds(cfl):
    base, design, evaluation, config = cfl
    link = Linkage(base, design, config)
    depths = _depths(base, evaluation)
    reference, fast = link.reference(*depths, width=3.0), link.fast(*depths, width=3.0)
    assert reference and fast and fast[0][0] == pytest.approx(reference[0][0], abs=1e-12)


def test_placing_is_never_worse_than_the_arrangement_it_starts_from(cfl):
    base, design, evaluation, config = cfl
    best = float(base.report(evaluation)["c_pad_mean"])
    own = dict(zip(base.variable_names, design))
    for shift in ((1.5, -1.0), (-2.0, 0.5), (0.0, 0.0)):
        moved = dict(own)
        moved[ARRANGEMENT[0]] += shift[0]
        moved[ARRANGEMENT[1]] += shift[1]
        start = tuple(moved[n] for n in base.variable_names)
        placed, found, spent = place(base, start, config)
        assert placed is not None and found.feasible and spent >= 1
        assert float(base.report(found)["c_pad_mean"]) == pytest.approx(best, abs=1e-9)  # the same linkage, the same best
        # and what it returns is a design the evaluator scores the same
        assert float(base.report(base.evaluate(placed, config))["c_pad_mean"]) == pytest.approx(best, abs=1e-9)


def test_the_placed_mechanism_scores_a_linkage_whatever_arrangement_it_comes_with(cfl):
    base, design, evaluation, config = cfl
    placed = PlacedMechanism(base)
    own = dict(zip(base.variable_names, design))
    moved = dict(own)
    moved[ARRANGEMENT[0]] += 2.0
    moved[ARRANGEMENT[1]] -= 1.5
    first = placed.evaluate(design, config)
    second = placed.evaluate(tuple(moved[n] for n in base.variable_names), config)
    assert first.feasible and second.feasible and placed.evaluations >= 2
    assert float(base.report(second)["c_pad_mean"]) == pytest.approx(float(base.report(first)["c_pad_mean"]), abs=1e-9)
    assert placed.constraint_names == tuple(base.constraint_names) and placed.variable_names == base.variable_names
