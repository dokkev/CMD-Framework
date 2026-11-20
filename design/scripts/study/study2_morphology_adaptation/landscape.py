#!/usr/bin/env python3
"""Study 2: the morphology-change landscape (problem formulation A.6.3).

    python scripts/study/study2_morphology_adaptation/landscape.py [landscape.yaml] [--resume]
        [--references a,b] [--flexions 120] [--sections symmetric]

For each reference hand, required flexion and 2D section of the module space
(``landscape.yaml``), the least external change ``eps_d*`` on a grid of module
lengths and thicknesses. Each grid point is ``run.py``'s hierarchy (frozen,
Level 1, Level 2), searched in two sweeps warm-started from the already-solved
neighbours (``up``: from the smallest module outward; ``down``: from the
largest inward), the better kept. Points on a boundary are then searched again
at a larger budget (``recheck``).

Writes to the config's ``output``:

- ``rows.csv``: every search (pass, seed), as ``run.py``'s summary rows plus the point;
- ``landscape.csv``: each point's best outcome over its passes, with each sweep's
  own best (``up_*``, ``down_*``), their gap and whether it was rechecked;
- ``references.csv``, ``markers.csv`` (the catalog's named pairs on each section)
  and ``manifest.json``.

``--resume`` keeps ``rows.csv``'s searches and runs only the missing ones.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.analysis.adaptation import DESCRIPTORS, adapt, config_resolution  # noqa: E402
from core.optimization.analysis.tradeoff import with_modules  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402
from core.optimization.optimizer import OptimizerSettings  # noqa: E402

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "landscape.yaml"

study2 = None  # run.py, loaded on first use (it loads shared.py)


def _study2():
    global study2
    if study2 is None:
        import importlib.util

        name = "study2_run"
        if name in sys.modules:
            study2 = sys.modules[name]
        else:
            spec = importlib.util.spec_from_file_location(name, HERE / "run.py")
            study2 = importlib.util.module_from_spec(spec)
            sys.modules[name] = study2
            spec.loader.exec_module(study2)
    return study2


SECTIONS = ("symmetric", "proximal", "distal")
DIRECTIONS = ("up", "down")
RECHECK = "recheck"


# The grid -------------------------------------------------------------------

@dataclass(frozen=True)
class Point:
    """One grid point of a section: its indices, its module length and thickness
    (mm), and the module pair it stands for."""

    section: str
    i: int
    """Index along the lengths."""
    j: int
    """Index along the thicknesses."""
    length_mm: float
    thickness_mm: float
    pair: Any
    """A ``shared.ModulePair`` (``module_id``: ``<section>_l<length>_h<thickness>``)."""

    @property
    def key(self) -> tuple[int, int]:
        return self.i, self.j


def section_pair(section: str, length_mm: float, thickness_mm: float, baseline):
    """The module pair at ``(length, thickness)`` of ``section``; ``baseline`` (a
    ``ModulePair``) holds the side a one-sided section does not vary."""

    shared = _study2().shared
    proximal = (length_mm, thickness_mm) if section in ("symmetric", "proximal") else \
        (baseline.proximal_length_mm, baseline.proximal_thickness_mm)
    distal = (length_mm, thickness_mm) if section in ("symmetric", "distal") else \
        (baseline.distal_length_mm, baseline.distal_thickness_mm)
    return shared.ModulePair(f"{section}_l{length_mm:g}_h{thickness_mm:g}", section, *proximal, *distal)


def grid(section: str, lengths, thicknesses, baseline) -> dict[tuple[int, int], Point]:
    if section not in SECTIONS:
        raise ValueError(f"section must be one of {SECTIONS}, not {section!r}")
    return {(i, j): Point(section, i, j, float(length), float(thickness),
                          section_pair(section, float(length), float(thickness), baseline))
            for i, length in enumerate(lengths) for j, thickness in enumerate(thicknesses)}


def order(shape: tuple[int, int], direction: str) -> list[tuple[int, int]]:
    """The points in sweep order: ``up`` by growing ``i + j`` (the smallest
    module first), ``down`` the reverse; each point after the neighbours it
    takes its warm start from (:func:`predecessors`)."""

    found = sorted(((i, j) for i in range(shape[0]) for j in range(shape[1])), key=lambda p: (p[0] + p[1], p[0]))
    if direction == "up":
        return found
    if direction == "down":
        return found[::-1]
    raise ValueError(f"direction must be one of {DIRECTIONS}, not {direction!r}")


def predecessors(key: tuple[int, int], shape: tuple[int, int], direction: str) -> list[tuple[int, int]]:
    """The neighbours a point is warm-started from in a sweep: those one step
    smaller (``up``) or larger (``down``) in length or thickness."""

    i, j = key
    step = -1 if direction == "up" else 1
    return [(a, b) for a, b in ((i + step, j), (i, j + step)) if 0 <= a < shape[0] and 0 <= b < shape[1]]


def neighbours(key: tuple[int, int], shape: tuple[int, int]) -> list[tuple[int, int]]:
    i, j = key
    return [(a, b) for a, b in ((i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1))
            if 0 <= a < shape[0] and 0 <= b < shape[1]]


def _cost(row: dict[str, Any]) -> float:
    value = row.get("final_external_cost")
    return math.nan if value in (None, "") else float(value)


def boundary(best: dict[tuple[int, int], dict[str, Any]], shape: tuple[int, int], jump: float) -> list:
    """The points to search again: not found, an outcome other than a
    neighbour's, or an ``eps_d`` more than ``jump`` from a neighbour's."""

    found = []
    for key, row in best.items():
        if row["status"] == "not_found":
            found.append(key)
            continue
        for other in neighbours(key, shape):
            if other not in best:
                continue
            neighbour = best[other]
            if neighbour["status"] != row["status"]:
                found.append(key)
                break
            a, b = _cost(row), _cost(neighbour)
            if math.isfinite(a) and math.isfinite(b) and abs(a - b) > jump:
                found.append(key)
                break
    return sorted(found)


def markers(pairs, baseline) -> list[dict[str, Any]]:
    """Where each named pair of the catalog lies: on which sections, at which
    ``(length, thickness)``; a pair on none goes in the table."""

    def same(a, b):
        return math.isclose(a[0], b[0]) and math.isclose(a[1], b[1])

    rows = []
    for pair in pairs:
        proximal = (pair.proximal_length_mm, pair.proximal_thickness_mm)
        distal = (pair.distal_length_mm, pair.distal_thickness_mm)
        base_proximal = (baseline.proximal_length_mm, baseline.proximal_thickness_mm)
        base_distal = (baseline.distal_length_mm, baseline.distal_thickness_mm)
        on = []
        if same(proximal, distal):
            on.append(("symmetric", proximal))
        if same(distal, base_distal):
            on.append(("proximal", proximal))
        if same(proximal, base_proximal):
            on.append(("distal", distal))
        common = {"module_id": pair.module_id, "proximal_length_mm": proximal[0], "proximal_thickness_mm": proximal[1],
                  "distal_length_mm": distal[0], "distal_thickness_mm": distal[1]}
        if not on:
            rows.append({**common, "section": "table", "length_mm": math.nan, "thickness_mm": math.nan})
        rows.extend({**common, "section": section, "length_mm": at[0], "thickness_mm": at[1]} for section, at in on)
    return rows


# The searches ---------------------------------------------------------------

def _parse(value: str):
    if value in ("True", "False"):
        return value == "True"
    for kind in (int, float):  # as written: "3" an int, "3.0" a float
        try:
            return kind(value)
        except ValueError:
            pass
    return value


def read_rows(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    with path.open(encoding="utf-8", newline="") as stream:
        return [{k: _parse(v) for k, v in row.items()} for row in csv.DictReader(stream)]


def _x(row: dict[str, Any] | None):
    text = None if row is None else row.get("final_x")
    if not text or not isinstance(text, str):
        return None
    return tuple(float(v) for v in text.split(";"))


def _row_key(row) -> tuple:
    return (row["reference_id"], float(row["required_flexion_deg"]), row["target"], row["pass"], int(row["seed"]))


class Landscape:
    """One run: the rows so far (``rows.csv``), each search appended as it ends."""

    def __init__(self, output: Path, resume: bool, workers: int, external_range_mm: float):
        self.output, self.workers, self.external_range_mm = output, workers, external_range_mm
        self.path = output / "rows.csv"
        self.rows = read_rows(self.path) if resume else []
        self.done = {_row_key(row): row for row in self.rows}
        self.frozen = {key[:3] for key, row in self.done.items() if row["status"] == "frozen_feasible"}

    def solve(self, reference, flexion: float, point: Point, label: str, optimizer: dict,
              warm) -> list[dict[str, Any]]:
        """``point``'s searches in pass ``label``, one per seed (one only if the
        reference carries the module as it is); kept rows reused."""

        run = _study2()
        part = run.partitions.partition(reference.mechanism)
        carried = run.shared.with_required_flexion(with_modules(reference.config, point.pair.modules), flexion)
        scale = part.external(reference.x, carried)["finger_length_mm"]
        found = []
        for seed in (int(seed) for seed in optimizer["seeds"]):
            key = (reference.reference_id, float(flexion), point.pair.module_id, label, seed)
            if key[:3] in self.frozen:  # no search: every pass and seed the same
                return [row for row in self.rows if _row_key(row)[:3] == key[:3]][:1]
            if key in self.done:
                found.append(self.done[key])
                continue
            settings = OptimizerSettings(optimizer["algorithm"], int(optimizer["population"]),
                                         int(optimizer["generations"]), seed)
            print(f"{reference.reference_id} / q_req {flexion:g} / {point.pair.module_id} / {label} / seed {seed} "
                  f"(warm {len(warm)}) ...", flush=True)
            result = adapt(mechanism(reference.mechanism), part, reference.x, carried, settings, scale,
                           self.external_range_mm, workers=self.workers, warm=warm)
            row = {"pass": label, "seed": seed, "required_flexion_deg": float(flexion), "section": point.section,
                   "i": point.i, "j": point.j, "length_mm": point.length_mm, "thickness_mm": point.thickness_mm,
                   **run.summary_row(reference, point.pair, result, scale)}
            print(f"  {row['status']}: eps_d {row['final_external_cost']}, eps_kappa {row['final_mechanism_cost']}",
                  flush=True)
            self.rows.append(row)
            self.done[key] = row
            run.shared.write_csv(self.path, self.rows)
            found.append(row)
            if result.status == "frozen_feasible":
                self.frozen.add(key[:3])
                break
        return found


def best(rows: list[dict[str, Any]], delta: float) -> dict[str, Any] | None:
    """The best of a point's rows by ``run.py``'s rule (``best_rows``)."""

    if not rows:
        return None
    chosen = _study2().best_rows(rows, delta)
    assert len(chosen) == 1, "rows of one point"
    return chosen[0]


def sweep(landscape: Landscape, reference, flexion: float, points: dict, shape, config: dict,
          delta: float) -> dict[tuple[int, int], dict[str, Any]]:
    """Both sweeps, then the recheck; each point's best over every pass."""

    by_point: dict[tuple[int, int], list[dict[str, Any]]] = {key: [] for key in points}
    for direction in config.get("directions") or DIRECTIONS:
        solved: dict[tuple[int, int], dict[str, Any]] = {}
        for key in order(shape, direction):
            warm = [x for x in (_x(solved.get(other)) for other in predecessors(key, shape, direction)) if x]
            rows = landscape.solve(reference, flexion, points[key], direction, config["optimizer"], warm)
            by_point[key].extend(rows)
            solved[key] = best(rows, delta)
    merged = {key: best(rows, delta) for key, rows in by_point.items()}
    recheck = config.get("recheck")
    rechecked = set()
    if recheck:
        for key in boundary(merged, shape, float(recheck["jump"])):
            warm = {x for x in (_x(row) for row in by_point[key]) if x}
            warm |= {x for x in (_x(merged.get(other)) for other in neighbours(key, shape)) if x}
            rows = landscape.solve(reference, flexion, points[key], RECHECK, recheck["optimizer"], sorted(warm))
            by_point[key].extend(rows)
            rechecked.add(key)
    table = {}
    for key, rows in by_point.items():
        row = dict(best(rows, delta))
        point = points[key]
        row.update(section=point.section, i=point.i, j=point.j, length_mm=point.length_mm,
                   thickness_mm=point.thickness_mm, rechecked=key in rechecked)
        sweeps = {}
        for direction in DIRECTIONS:
            own = best([r for r in rows if r["pass"] == direction], delta)
            sweeps[direction] = own
            row[f"{direction}_status"] = "" if own is None else own["status"]
            row[f"{direction}_external_cost"] = math.nan if own is None else _cost(own)
        up, down = (_cost(sweeps[d]) if sweeps[d] else math.nan for d in DIRECTIONS)
        row["direction_gap"] = abs(up - down) if math.isfinite(up) and math.isfinite(down) else math.nan
        row["best_pass"] = row.pop("pass", "")
        table[key] = row
    return table


def run(config_path: Path, root: Path = REPOSITORY_ROOT, resume: bool = False, only_references=None,
        only_flexions=None, only_sections=None) -> Path:
    study = _study2()
    shared = study.shared
    config = read_config_file(config_path)
    output = root / config["output"]
    output.mkdir(parents=True, exist_ok=True)
    catalog = shared.module_catalog(read_config_file(root / config["catalog"]))
    named = [pair for pair in catalog if pair.group == "named"]
    baseline = next(pair for pair in catalog if pair.module_id == "baseline")
    reference_values = read_config_file(root / config["references"])
    built = shared.references(reference_values, tuple(config["kinds"]), baseline.modules, root)
    rules = shared.check_comparable(built)
    if only_references:
        built = [r for r in built if r.reference_id in only_references]
    flexions = [float(v) for v in config["required_flexion_deg"]]
    if only_flexions:
        flexions = [v for v in flexions if v in only_flexions]
    sections = [s for s in config["sections"] if not only_sections or s in only_sections]
    lengths, thicknesses = config["grid"]["length_mm"], config["grid"]["thickness_mm"]
    shape = (len(lengths), len(thicknesses))
    landscape = Landscape(output, resume, int(config.get("workers", 1)), float(config["external_range_mm"]))
    commit = shared.git_commit(output)
    shared.write_csv(output / "markers.csv", markers(named, baseline))
    shared.write_csv(output / "references.csv", [
        {"reference_id": r.reference_id, "kind": r.kind, "mechanism": r.mechanism, "upper_bound": r.upper_bound,
         "servo_tilt_deg": r.servo_tilt_deg, "config": r.config_path, "source": r.source,
         **{f"e_{k}": v for k, v in study.partitions.partition(r.mechanism).external(r.x, r.config).items()},
         "x": ";".join(repr(float(v)) for v in r.x)} for r in built])
    table_path = output / "landscape.csv"
    table = [row for row in read_rows(table_path)
             if (row["reference_id"], float(row["required_flexion_deg"]), row["section"]) not in
             {(r.reference_id, f, s) for r in built for f in flexions for s in sections}] if resume else []
    for reference in built:
        part = study.partitions.partition(reference.mechanism)
        delta = config_resolution(reference.config) / part.external(reference.x, reference.config)["finger_length_mm"]
        for flexion in flexions:
            for section in sections:
                points = grid(section, lengths, thicknesses, baseline)
                found = sweep(landscape, reference, flexion, points, shape, config, delta)
                table.extend(found[key] for key in sorted(found))
                shared.write_csv(table_path, table)
    manifest = {
        "study": "study2_landscape", "config": config, "references": reference_values,
        "run_filters": {"references": [r.reference_id for r in built], "required_flexion_deg": flexions,
                        "sections": sections},
        "baseline_module": baseline.module_id, "descriptors": list(DESCRIPTORS),
        "normalization": "every change divided by the reference finger's length",
        "comparison_rules": rules, "formulations": shared.formulations(built), "source": commit,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str) + "\n", encoding="utf-8")
    return output


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("config", nargs="?", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--resume", action="store_true", help="keep rows.csv's searches")
    parser.add_argument("--references", help="comma-separated reference ids (default: all)")
    parser.add_argument("--flexions", help="comma-separated required flexions (deg; default: all)")
    parser.add_argument("--sections", help="comma-separated sections (default: the config's)")
    args = parser.parse_args(argv)
    output = run(args.config, resume=args.resume,
                 only_references=args.references and args.references.split(","),
                 only_flexions=args.flexions and [float(v) for v in args.flexions.split(",")],
                 only_sections=args.sections and args.sections.split(","))
    print(f"wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
