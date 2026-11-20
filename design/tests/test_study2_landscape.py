"""Study 2's landscape (scripts/study/study2_morphology_adaptation/landscape.py,
problem formulation A.6.3): the grid and its sections, both sweeps, the
recheck of the boundary, the markers, and a small run end to end."""

from __future__ import annotations

import csv
import importlib.util
import math
import sys
from pathlib import Path

import pytest
import yaml

from core.optimization.analysis.adaptation import adapt
from core.optimization.analysis.tradeoff import with_modules
from core.optimization.mechanism import mechanism
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


study2 = _module("study2_run", STUDY / "run.py")
landscape = _module("study2_landscape", STUDY / "landscape.py")
figure = _module("study2_figure_landscape", STUDY / "figure_landscape.py")
shared = study2.shared
CATALOG = shared.module_catalog(study2.read_config_file(ROOT / "scripts/study/study1_fixed_morphology/modules.yaml"))
BASELINE = next(pair for pair in CATALOG if pair.module_id == "baseline")


def test_each_section_varies_only_its_own_sides():
    points = {s: landscape.grid(s, [15, 45], [3, 9], BASELINE) for s in landscape.SECTIONS}
    sym, prox, dist = (points[s][(1, 1)].pair for s in landscape.SECTIONS)
    assert (sym.proximal_length_mm, sym.proximal_thickness_mm, sym.distal_length_mm, sym.distal_thickness_mm) \
        == (45, 9, 45, 9)
    assert (prox.proximal_length_mm, prox.proximal_thickness_mm) == (45, 9)
    assert (prox.distal_length_mm, prox.distal_thickness_mm) == (20, 3)  # the baseline's
    assert (dist.proximal_length_mm, dist.proximal_thickness_mm) == (20, 3)
    assert (dist.distal_length_mm, dist.distal_thickness_mm) == (45, 9)
    assert sym.module_id == "symmetric_l45_h9" and len({p.pair.module_id for p in points["proximal"].values()}) == 4
    with pytest.raises(ValueError):
        landscape.grid("diagonal", [15], [3], BASELINE)


@pytest.mark.parametrize("direction", landscape.DIRECTIONS)
def test_a_sweep_visits_every_point_once_after_its_predecessors(direction):
    shape = (4, 3)
    seen = []
    for key in landscape.order(shape, direction):
        assert all(other in seen for other in landscape.predecessors(key, shape, direction))
        seen.append(key)
    assert sorted(seen) == [(i, j) for i in range(4) for j in range(3)]
    first = (0, 0) if direction == "up" else (3, 2)
    assert seen[0] == first and landscape.predecessors(first, shape, direction) == []


def test_the_two_sweeps_start_from_opposite_neighbours():
    shape = (3, 3)
    assert sorted(landscape.predecessors((1, 1), shape, "up")) == [(0, 1), (1, 0)]
    assert sorted(landscape.predecessors((1, 1), shape, "down")) == [(1, 2), (2, 1)]


def test_the_boundary_is_where_the_outcome_or_the_change_jumps():
    def row(status, cost):
        return {"status": status, "final_external_cost": cost}

    best = {(0, 0): row("frozen_feasible", 0.0), (1, 0): row("frozen_feasible", 0.0),
            (0, 1): row("level2", 0.02), (1, 1): row("level2", 0.03),
            (0, 2): row("level2", 0.10), (1, 2): row("not_found", math.nan)}
    found = landscape.boundary(best, (2, 3), jump=0.05)
    # (0,0)-(0,1) and (1,0)-(1,1): the outcome changes; (0,1)-(0,2): eps_d jumps by 0.08;
    # (1,2): not found, and every neighbour of it.
    assert found == [(0, 0), (0, 1), (0, 2), (1, 0), (1, 1), (1, 2)]
    calm = {(0, 0): row("level2", 0.02), (1, 0): row("level2", 0.05)}
    assert landscape.boundary(calm, (2, 1), jump=0.05) == []


def test_the_catalog_pairs_mark_the_sections_they_lie_on():
    named = [pair for pair in CATALOG if pair.group == "named"]
    rows = landscape.markers(named, BASELINE)
    where = {}
    for row in rows:
        where.setdefault(row["module_id"], []).append((row["section"], row["length_mm"], row["thickness_mm"]))
    assert sorted(where["baseline"]) == [("distal", 20, 3), ("proximal", 20, 3), ("symmetric", 20, 3)]
    assert where["long_thick"] == [("symmetric", 35, 8)]
    assert [w[0] for w in where["asym_long_proximal"]] == ["table"]


MATCHED = "scripts/study/study2_morphology_adaptation/references_nominal15.yaml"


def test_a_neighbours_design_warm_starts_the_search():
    """A search started from a design it found before does no worse; a design
    the search cannot read is left out, not an error."""

    reference = [r for r in shared.references(study2.read_config_file(ROOT / MATCHED), ("matched",),
                                               BASELINE.modules, ROOT) if r.reference_id == "2r_dd_tilt0_matched"][0]
    part = study2.partitions.partition(reference.mechanism)
    carried = shared.with_required_flexion(with_modules(reference.config, landscape.section_pair(
        "symmetric", 30, 3, BASELINE).modules), 120)
    scale = part.external(reference.x, carried)["finger_length_mm"]
    settings = OptimizerSettings("nsga2", 20, 5, 0)
    cold = adapt(mechanism(reference.mechanism), part, reference.x, carried, settings, scale, 20.0)
    assert cold.final is not None, "the small search finds a design to warm-start from"
    warm = adapt(mechanism(reference.mechanism), part, reference.x, carried, settings, scale, 20.0,
                 warm=[cold.final.x, (math.nan,) * len(reference.x)])
    assert warm.final is not None and warm.external_cost <= cold.external_cost + 1e-12


def test_a_small_landscape_runs_and_resumes(tmp_path):
    values = study2.read_config_file(STUDY / "landscape.yaml")
    values.update(references=MATCHED, kinds=["matched"], required_flexion_deg=[120], sections=["symmetric"],
                  grid={"length_mm": [20, 40], "thickness_mm": [3, 8]}, workers=1,
                  output=str(tmp_path / "out"))
    values["optimizer"].update(population=12, generations=2, seeds=[0])
    values["recheck"]["optimizer"].update(population=12, generations=2, seeds=[1])
    config = tmp_path / "landscape.yaml"
    config.write_text(yaml.safe_dump(values))
    out = landscape.run(config, ROOT, only_references=["2r_dd_tilt0_matched"])
    with (out / "landscape.csv").open() as stream:
        table = list(csv.DictReader(stream))
    with (out / "rows.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(table) == 4 and {(r["length_mm"], r["thickness_mm"]) for r in table} == {
        ("20.0", "3.0"), ("20.0", "8.0"), ("40.0", "3.0"), ("40.0", "8.0")}
    small = next(r for r in table if r["length_mm"] == "20.0" and r["thickness_mm"] == "3.0")
    assert small["status"] == "frozen_feasible"  # the reference's own module: no change
    assert float(small["final_external_cost"]) == 0.0
    # Each point searched in both sweeps (the frozen one needs no search), each row its pass.
    searched = {r["target"] for r in rows if r["pass"] == "down"}
    assert searched == {r["target"] for r in rows if r["pass"] == "up"} - {"symmetric_l20_h3"}
    assert all(r["up_status"] and r["down_status"] for r in table if r["target"] != "symmetric_l20_h3")
    assert any(r["rechecked"] == "True" for r in table)  # the frozen point borders a changed one
    excess = [float(r["external_excess"]) for r in table if r["status"] == "level2"]
    assert all(value >= -1e-9 for value in excess)
    assert (out / "markers.csv").is_file() and (out / "manifest.json").is_file()
    # Resumed, nothing is searched again: the same table.
    again = landscape.run(config, ROOT, resume=True, only_references=["2r_dd_tilt0_matched"])
    with (again / "rows.csv").open() as stream:
        assert len(list(csv.DictReader(stream))) == len(rows)
    with (again / "landscape.csv").open() as stream:
        assert list(csv.DictReader(stream)) == table
    made = figure.figures(out)
    assert sorted(p.name for p in made) == ["eps_d_symmetric_q120.png", "excess_symmetric_q120.png",
                                             "regions_symmetric_q120.png"]
    assert all(p.suffix == ".png" and p.stat().st_size > 0 for p in made)


def test_cells_span_halfway_to_their_neighbours():
    assert figure.edges([15, 20, 30]).tolist() == [12.5, 17.5, 25.0, 35.0]
    assert figure.edges([3]).tolist() == [2.5, 3.5]


def test_a_region_is_outlined_where_it_meets_the_rest_inside_the_grid():
    import numpy as np

    mask = np.array([[True, True], [True, False]])  # [length, thickness]
    x, y = np.array([0.0, 1.0, 2.0]), np.array([0.0, 1.0, 2.0])
    segments = {tuple(map(tuple, s)) for s in figure.region_segments(mask, x, y)}
    # Only the two sides the region shares with the excluded cell; none on the domain's edge.
    assert segments == {((1.0, 1.0), (1.0, 2.0)), ((1.0, 1.0), (2.0, 1.0))}
    assert figure.region_segments(np.ones((2, 2), bool), x, y) == []


study1 = _module("study1_run", ROOT / "scripts/study/study1_fixed_morphology/run.py")
MATCHED_REFERENCES = shared.references(study2.read_config_file(ROOT / MATCHED), ("matched",), BASELINE.modules, ROOT)
AGREEMENT_PAIRS = [pair for pair in CATALOG if pair.group == "named"] + [
    landscape.section_pair(section, length, thickness, BASELINE)
    for section in landscape.SECTIONS for length, thickness in ((15, 3), (30, 6), (45, 9))]
"""The catalog's named pairs (the landscape's markers) and a few grid points of each section."""


@pytest.mark.parametrize("reference", MATCHED_REFERENCES, ids=lambda r: r.reference_id)
def test_level_0_accepts_exactly_what_study_1_accepts(reference):
    """Study 2 starts where Study 1 ends (A.6.3, Level 0): its frozen step and
    Study 1's acceptance agree on every module and required flexion."""

    from core.optimization.analysis.adaptation import frozen_step

    found = mechanism(reference.mechanism)
    part = study2.partitions.partition(reference.mechanism)
    flexions = (90.0, 120.0, 150.0)
    accepted, rejected, disagree = 0, 0, []
    for pair in AGREEMENT_PAIRS:
        study1_rows = study1.assembly(reference, pair, flexions)
        for flexion, row in zip(flexions, study1_rows):
            carried = shared.with_required_flexion(with_modules(reference.config, pair.modules), flexion)
            scale = part.external(reference.x, carried)["finger_length_mm"]
            level0 = frozen_step(found, part, reference.x, carried, scale).status == "feasible"
            accepted += level0
            rejected += not level0
            if level0 != bool(row["feasible"]):
                disagree.append((pair.module_id, flexion, row["feasible"], row.get("failure")))
    assert not disagree, disagree
    assert accepted and rejected, "some modules accepted, some not: the comparison is not vacuous"


def _matched(reference_id):
    return next(r for r in MATCHED_REFERENCES if r.reference_id == reference_id)


def test_a_native_reference_is_evaluated_at_its_own_required_flexion_alone():
    import dataclasses

    reference = _matched("code_matched")
    assert shared.flexions_for(reference, [90, 120, 150]) == [90.0, 120.0, 150.0]
    native = dataclasses.replace(reference, design_flexion_deg=150.0)
    assert shared.flexions_for(native, [90, 120, 150]) == [150.0]
    assert shared.flexions_for(native, [90, 120]) == []


def test_references_designed_for_other_flexions_share_every_other_rule():
    import dataclasses

    a, b = _matched("code_matched"), _matched("2r_dd_tilt0_matched")
    a = dataclasses.replace(a, config=shared.with_required_flexion(a.config, 90), design_flexion_deg=90.0)
    b = dataclasses.replace(b, config=shared.with_required_flexion(b.config, 150), design_flexion_deg=150.0)
    rules = shared.check_comparable([a, b])
    assert "required_rom_rad" not in rules
    with pytest.raises(ValueError):  # without design flexions, a different q_req is a different rule
        shared.check_comparable([dataclasses.replace(a, design_flexion_deg=None),
                                 dataclasses.replace(b, design_flexion_deg=None)])


def test_a_tilt_sweep_may_keep_the_design_at_its_own_tilt():
    reference = _matched("rcj_tilt0_matched")
    sweep = {"angles_deg": [15, 30], "references": ["rcj_tilt0_matched"]}
    assert [r.reference_id for r in shared.sweep_references([reference], sweep)] == [
        "rcj_tilt0_matched__tilt15", "rcj_tilt0_matched__tilt30"]
    kept = shared.sweep_references([reference], {**sweep, "include_reference": True})
    assert [r.reference_id for r in kept] == ["rcj_tilt0_matched", "rcj_tilt0_matched__tilt15",
                                               "rcj_tilt0_matched__tilt30"]
    assert kept[0] is reference
