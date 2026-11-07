"""Study 1, contact accommodation at a fixed morphology
(scripts/study/study1_fixed_morphology): every reference, frozen, carries
every module through the mechanisms' own evaluation, the same references and
modules as Study 2's."""

from __future__ import annotations

import csv
import dataclasses
import importlib.util
import sys
from pathlib import Path

import pytest

from core.optimization.analysis.tradeoff import goal_of, with_goal
from core.optimization.mechanism import LENGTH_TOLERANCE_MM, mechanism, shortfall_mm
from core.optimization.mounting import mounting

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "scripts" / "study"


def _module(name: str, path: Path):
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


shared = _module("study_shared", STUDY / "shared.py")
study1 = _module("study1_run", STUDY / "study1_fixed_morphology" / "run.py")
study2 = _module("study2_run", STUDY / "study2_morphology_adaptation" / "run.py")
REFERENCES = shared.load_references()
CATALOG = shared.catalog()
SMALL = """baseline:
  proximal: {length_mm: 20, thickness_mm: 3}
  distal: {length_mm: 20, thickness_mm: 3}
modules:
  baseline: {}
  thick: {proximal: {thickness_mm: 8}, distal: {thickness_mm: 8}}
  long_distal: {distal: {length_mm: 35}}
"""


@pytest.fixture(scope="module")
def results(tmp_path_factory):
    folder = tmp_path_factory.mktemp("study1")
    (folder / "modules.yaml").write_text(SMALL)
    (folder / "config.yaml").write_text(f"modules: {folder / 'modules.yaml'}\nkinds: [matched]\n"
                                        f"required_flexion_deg: [90, 150]\nworkers: 1\noutput: {folder / 'out'}\n")
    out = study1.run(folder / "config.yaml", ROOT)
    with (out / "assembly_results.csv").open() as stream:
        return out, list(csv.DictReader(stream))


def test_every_reference_carries_every_module_at_every_flexion(results):
    _, rows = results
    assert len(rows) == len(REFERENCES) * 3 * 2
    assert len({row["assembly_id"] for row in rows}) == len(rows)
    assert {row["mechanism"] for row in rows} == {"linkage", "revolute", "revolute_parallel", "rcj", "rcj_idealized"}


def test_a_module_changes_only_the_mounting_depths_and_those_by_the_policy():
    for reference in REFERENCES:
        part = shared.partitions.partition(reference.mechanism)
        x, carried = study1.frozen_design(reference, CATALOG["thick"].modules)
        if x is None:
            continue
        slots = [part.names.index(slot) for slot in part.depths]  # (proximal, distal), as the policy gives them
        assert all(a == b for i, (a, b) in enumerate(zip(x, reference.x)) if i not in slots)
        assert tuple(x[i] for i in slots) == mounting(reference.mechanism, x, carried).depths_mm


def test_a_module_longer_than_its_segment_does_not_mount_on_any_architecture(results):
    # The matched references' distal span is 30.2 mm: a 35 mm distal module overhangs every one.
    _, rows = results
    long = [row for row in rows if row["module_id"] == "long_distal"]
    assert long and all(row["mounted"] == "False" and row["failure"].startswith("not_mounted") for row in long)
    # ... and the study goes on: the other modules are evaluated.
    assert any(row["feasible"] == "True" for row in rows if row["module_id"] == "baseline")


def test_a_failed_assembly_is_reported_with_its_reasons(results):
    _, rows = results
    failed = [row for row in rows if row["mounted"] == "True" and row["feasible"] == "False"]
    assert failed and all(row["failure"] for row in failed)


def test_study_1_at_the_required_flexion_is_study_2s_frozen_check():
    for name in ("baseline", "thick", "long", "asym_long_distal"):
        modules = CATALOG[name].modules
        for reference in REFERENCES:
            row = next(r for r in study1.assembly(reference, CATALOG[name], [120.0]))
            found, part = mechanism(reference.mechanism), shared.partitions.partition(reference.mechanism)
            from core.optimization.analysis.adaptation import AdaptationMechanism
            from core.optimization.analysis.tradeoff import with_modules

            carried = with_modules(reference.config, modules)
            frozen = AdaptationMechanism(found, part, reference.x, carried, "mechanism", "mechanism_cost", 90.0)
            x = part.compose(frozen.reference_external, frozen.reference_coordinates, reference.x, carried)
            evaluation = None if x is None else frozen.evaluate(x, carried)
            assert (row["feasible"] is True) == bool(evaluation is not None and evaluation.feasible), \
                (name, reference.reference_id)


def test_equal_inputs_give_equal_results(results, tmp_path):
    out, rows = results
    config = (out.parent / "config.yaml").read_text().replace(str(out), str(tmp_path / "again"))
    (tmp_path / "config.yaml").write_text(config)
    again = study1.run(tmp_path / "config.yaml", ROOT)
    for name in ("assembly_results.csv", "references.csv", "module_catalog.csv"):
        assert (again / name).read_text() == (out / name).read_text()


def test_references_under_different_rules_are_not_compared():
    changed = dataclasses.replace(REFERENCES[1], config=with_goal(REFERENCES[1].config, dataclasses.replace(
        goal_of(REFERENCES[1].config), required_rom_rad=1.0)))
    with pytest.raises(ValueError, match="different comparison rules"):
        shared.check_comparable([REFERENCES[0], changed])


def test_the_manifest_records_the_rules_formulations_and_source(results):
    import json

    out, _ = results
    manifest = json.loads((out / "manifest.json").read_text())
    assert manifest["comparison_rules"]["required_rom_rad"] == pytest.approx(2.0943951023931953)
    assert set(manifest["formulations"]) == {r.mechanism for r in REFERENCES}
    assert "commit" in manifest["source"]


def test_a_module_meeting_its_usable_length_to_float_noise_is_usable():
    assert shortfall_mm(20.0, 20.0 - LENGTH_TOLERANCE_MM / 2) == 0.0
    assert shortfall_mm(20.0, 19.9) == pytest.approx(0.1)
    assert shortfall_mm(20.0, 21.0) == -1.0
    # The prototype carries its baseline modules at 90 deg (before: 3.6e-15 mm short, infeasible).
    code = next(r for r in REFERENCES if r.reference_id == "code_matched")
    row = study1.assembly(code, CATALOG["baseline"], [90.0])[0]
    assert row["feasible"] is True
