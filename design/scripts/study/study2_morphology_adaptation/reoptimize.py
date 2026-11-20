#!/usr/bin/env python3
"""Study 2: the morphology re-optimized for each contact interface (problem formulation A.6.3).

    python scripts/study/study2_morphology_adaptation/reoptimize.py [reoptimize.yaml] [--study 2a|2b]
        [--preset quick|full] [--resume] [--references a,b] [--flexions 120] [--sections length]
        [--finger-length fixed|a,b] [--output dir]

Study 2a (A.6.3) frees the finger's length; Study 2b (A.6.4) holds it and adds
the cross-evaluation (``retention.csv``, ``summary.csv``).

For each reference hand, required flexion and 1D section of the symmetric
interface (``reoptimize.yaml``), every point's interface is given to the
reference's mechanism and its design searched again for the pad coverage
(``core.optimization.analysis.reoptimize``): the pads fixed to the interface,
the finger's length in the config's range. The best coverage comes from two
sweeps (``up`` from the smallest interface, ``down`` from the largest), each
point warm-started from the reference and the sweep's previous point; then the
design within ``coverage_delta`` of it closest to the reference's external
morphology (``closest``) is the point's optimal morphology.

Writes to the output directory:

- ``rows.csv``: every search (pass, seed): its coverage and morphology;
- ``points.csv``: each point's best coverage and its closest near-optimal
  design (the morphology ``d*``, ``kappa*``, their change from the reference);
- ``retention.csv``: the cross-evaluation (A.6.4): the finger designed for
  each point ``a`` carrying each point ``b``'s interface by Study 1's mounting
  rule, accepted or not (and why), its coverage and the retention ``rho``;
- ``summary.csv``: per reference, flexion and section, the best single
  finger's worst retention over the declared family (``rho_one``), and the
  cross-check's corrections;
- ``references.csv`` and ``manifest.json``.

The cross-check (A.6.4): a frozen finger carrying ``b`` is one of ``b``'s
designs, so where it covers more than ``b``'s best it is added to ``b``'s
designs (pass ``cross``) before the closest design is chosen, and again after
(``cross_closest``, then ``b``'s closest search is run again).

``--resume`` keeps ``rows.csv``'s searches and runs only the missing ones.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import sys
from pathlib import Path
from typing import Any

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))
STUDY = REPOSITORY_ROOT / "scripts" / "study"
if str(STUDY) not in sys.path:
    sys.path.insert(0, str(STUDY))

import shared  # noqa: E402
from core.optimization.analysis.adaptation import DESCRIPTORS  # noqa: E402
from core.optimization.analysis.adaptation import with_finger_lengths  # noqa: E402
from core.optimization.analysis.reoptimize import best_coverage, closest, interface_config, measure, tip_path  # noqa: E402
from core.optimization.analysis.tradeoff import Modules  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402
from core.optimization.optimizer import OptimizerSettings  # noqa: E402

HERE = Path(__file__).resolve().parent
study1 = shared.load_module("study1_run", STUDY / "study1_fixed_morphology" / "run.py")
TOLERANCE = 1e-9
"""Coverage above a point's best by more than this is a correction (A.6.4 cross-check)."""
CROSS_ROUNDS = 3
DEFAULT_CONFIG = HERE / "reoptimize.yaml"
PASSES = ("up", "down")


def points(section: str, values: dict[str, Any]) -> list[tuple[float, float]]:
    """The section's interfaces, ``(length, thickness)``, in sweep order: along the
    swept dimension (``length``, ``thickness``), or over the ``grid`` of both in a
    snake (each row of lengths, a thickness, in turn the other way), so every point
    follows a neighbour."""

    if section == "length":
        return [(float(l), float(values["thickness_mm"])) for l in sorted(values["length_mm"])]
    if section == "thickness":
        return [(float(values["length_mm"]), float(h)) for h in sorted(values["thickness_mm"])]
    if section == "grid":
        lengths = sorted(float(l) for l in values["length_mm"])
        found = []
        for row, h in enumerate(sorted(float(h) for h in values["thickness_mm"])):
            found += [(l, h) for l in (lengths if row % 2 == 0 else reversed(lengths))]
        return found
    raise ValueError(f"unknown section {section!r}: length, thickness or grid")


def settings_of(values: dict[str, Any], seed: int) -> OptimizerSettings:
    return OptimizerSettings(values["algorithm"], int(values["population"]), int(values["generations"]), int(seed))


def _key(row: dict[str, Any]) -> tuple:
    return (row["reference_id"], float(row["required_flexion_deg"]), row["section"], float(row["length_mm"]),
            float(row["thickness_mm"]), row["pass"], int(row["seed"]))


def _parse(value: str):
    for kind in (int, float):
        try:
            return kind(value)
        except ValueError:
            pass
    return value


def read_rows(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8", newline="") as stream:
        return [{k: _parse(v) for k, v in row.items()} for row in csv.DictReader(stream)]


def design_row(design, reference_external, reference_mechanism, scale_mm: float) -> dict[str, Any]:
    """A found design's coverage, morphology and change from the reference (empty: none found)."""

    if design is None:
        return {"found": False}
    e = design.evaluation
    row = {"found": True, "c_pad_mean": e.c_pad_mean, "eps_kin": e.kinematic_cost, "eps_d": e.external_cost,
           "eps_kappa": e.mechanism_cost,
           "q_max_deg": getattr(e.base, "q_max_deg", math.nan), "ma_wc": getattr(e.base, "ma_wc", math.nan),
           "x": ";".join(f"{v:.6g}" for v in design.x)}
    for name in DESCRIPTORS:
        row[f"d_{name}"] = e.external[name]
        row[f"delta_{name}"] = (e.external[name] - reference_external[name]) / scale_mm
    for name, value in e.mechanism.items():
        row[f"k_{name}"] = value
        row[f"delta_k_{name}"] = (value - reference_mechanism[name]) / scale_mm
    return row


def in_family(section: str, point: tuple[float, float], family: dict[str, Any] | None) -> bool:
    """Whether ``point`` lies in the section's declared family (``None``: every point)."""

    if not family:
        return True
    for value, key in ((point[0], "length_mm"), (point[1], "thickness_mm")):
        if key in family:
            low, high = family[key]
            if not float(low) - 1e-9 <= value <= float(high) + 1e-9:
                return False
    return True


def carry(reference, x, config, point: tuple[float, float], flexion: float) -> dict[str, Any]:
    """The finger ``x`` (under ``config``) frozen, carrying ``point``'s interface by
    Study 1's mounting rule (A.6.4): Study 1's assembly row, and the composed design."""

    frozen = dataclasses.replace(reference, x=tuple(x), config=config)
    pair = shared.ModulePair(f"L{point[0]:g}_T{point[1]:g}", "symmetric", point[0], point[1], point[0], point[1])
    row = study1.assembly(frozen, pair, [flexion])[0]
    composed, carried = study1.frozen_design(frozen, pair.modules)
    if row.get("feasible") and composed is not None:  # its coverage unrounded (Study 1's rows are rounded)
        evaluation = mechanism(reference.mechanism).evaluate(composed, shared.with_required_flexion(carried, flexion))
        row["c_pad_mean"] = float(evaluation.c_pad_mean)
    return {**row, "x_composed": composed}


class Reoptimization:
    def __init__(self, config: dict[str, Any], output: Path, resume: bool, finger_length) -> None:
        self.config, self.output = config, output
        self.finger_length = finger_length
        self.rows_path = output / "rows.csv"
        self.rows = read_rows(self.rows_path) if resume else []
        self.done = {_key(row): row for row in self.rows}
        self.workers = int(config.get("workers", 1))
        self.corrections = 0
        self.ref_path = None

    def _config(self, reference, flexion: float, length: float, thickness: float):
        base = shared.with_required_flexion(reference.config, flexion)
        return interface_config(base, Modules(length, thickness, length, thickness), self.finger_length)

    def _frame(self, reference, flexion: float):
        """The reference's config at ``flexion``, its finger length the study's range (a frozen finger's frame)."""

        base = shared.with_required_flexion(reference.config, flexion)
        return base if self.finger_length is None else with_finger_lengths(base, *self.finger_length)

    def _candidate(self, reference, flexion, section, point, label, round_, x, base, part) -> dict | None:
        """A design found outside a search (the cross-check), added to ``point``'s designs."""

        key = (reference.reference_id, float(flexion), section, point[0], point[1], label, int(round_))
        if key in self.done:
            return self.done[key]
        design = measure(base, part, x, self._config(reference, flexion, *point), *self.ref_morphology[:2],
                         self.ref_morphology[2], self.ref_path)
        if design is None or not design.evaluation.feasible:
            return None
        row = {"reference_id": reference.reference_id, "mechanism": reference.mechanism,
               "required_flexion_deg": float(flexion), "section": section, "length_mm": point[0],
               "thickness_mm": point[1], "pass": label, "seed": int(round_), "evaluations": 0,
               **design_row(design, *self.ref_morphology)}
        self._save(row)
        return row

    def cross_check(self, reference, flexion, section, grid, designs, label, base, part) -> int:
        """Each point's finger carrying every other point's interface; where it covers
        more than that point's best, it joins that point's designs. The corrections made."""

        corrections = 0
        frame = self._frame(reference, flexion)
        for round_ in range(CROSS_ROUNDS):
            tops = {p: self._best(designs[p]) for p in grid}
            found = 0
            for a in grid:
                if tops[a] is None:
                    continue
                for b in grid:
                    if a == b and label == "cross":
                        continue
                    carried = carry(reference, self._x(tops[a]), frame, b, flexion)
                    target = tops[b]
                    if not carried.get("feasible") or carried.get("x_composed") is None:
                        continue
                    if target is not None and float(carried["c_pad_mean"]) <= float(target["c_pad_mean"]) + TOLERANCE:
                        continue
                    row = self._candidate(reference, flexion, section, b, label, 100 * round_ + grid.index(a),
                                          carried["x_composed"], base, part)
                    if row is not None and (target is None or float(row["c_pad_mean"]) > float(target["c_pad_mean"])):
                        designs[b].append(row)
                        tops[b] = row
                        found += 1
            corrections += found
            if not found:
                break
        return corrections

    def _save(self, row: dict[str, Any]) -> None:
        self.rows.append(row)
        self.done[_key(row)] = row
        shared.write_csv(self.rows_path, self.rows)

    def search(self, reference, flexion, section, point, label, values, warm, run) -> list[dict[str, Any]]:
        """``point``'s searches of pass ``label`` (one per seed), kept rows reused."""

        found = []
        for seed in values["seeds"]:
            key = (reference.reference_id, float(flexion), section, point[0], point[1], label, int(seed))
            if key in self.done:
                found.append(self.done[key])
                continue
            print(f"{reference.reference_id} / q_req {flexion:g} / {section} {point[0]:g}x{point[1]:g} / {label} / "
                  f"seed {seed} (warm {len([w for w in warm if w is not None])}) ...", flush=True)
            design, evaluations = run(settings_of(values, seed), warm)
            row = {"reference_id": reference.reference_id, "mechanism": reference.mechanism,
                   "required_flexion_deg": float(flexion), "section": section, "length_mm": point[0],
                   "thickness_mm": point[1], "pass": label, "seed": int(seed), "evaluations": evaluations,
                   **design_row(design, *self.ref_morphology)}
            print(f"  {'C %.4f, eps_kin %.4f, eps_d %.4f' % (row['c_pad_mean'], row['eps_kin'], row['eps_d']) if row['found'] else 'not found'}",
                  flush=True)
            self._save(row)
            found.append(row)
        return found

    @staticmethod
    def _x(row):
        return None if not row or not row.get("found") or row.get("found") == "False" else \
            tuple(float(v) for v in str(row["x"]).split(";"))

    @staticmethod
    def _best(rows):
        found = [r for r in rows if r.get("found") not in (False, "False")]
        return max(found, key=lambda r: float(r["c_pad_mean"])) if found else None

    def solve(self, reference, flexion: float, section: str, grid: list[tuple[float, float]]) -> list[dict]:
        base = mechanism(reference.mechanism)
        part = shared.partitions.partition(reference.mechanism)
        reference_config = shared.with_required_flexion(reference.config, flexion)
        external = part.external(reference.x, reference_config)
        self.ref_morphology = (external, part.mechanism(reference.x, reference_config), external["finger_length_mm"])
        # The reference's motion: its fingertip over [0, q_req] (A.6.3: the change of morphology is read from it).
        self.ref_path = tip_path(reference.mechanism, reference.x, reference_config, math.radians(flexion))
        scale = self.ref_morphology[2]
        best = {point: [] for point in grid}
        orders = {"up": grid, "down": list(reversed(grid))}
        for label, order in ((d, orders[d]) for d in self.config.get("directions", PASSES)):
            previous = None
            for point in order:
                config = self._config(reference, flexion, *point)

                def run(settings, warm, config=config):
                    return best_coverage(base, part, config, settings, *self.ref_morphology, warm=warm,
                                         workers=self.workers, reference_path=self.ref_path)

                rows = self.search(reference, flexion, section, point, label, self.config["search"],
                                   [reference.x, self._x(previous)], run)
                best[point].extend(rows)
                top = self._best(rows)
                previous = top or previous
        corrections = self.cross_check(reference, flexion, section, grid, best, "cross", base, part)
        results = self._closest_all(reference, flexion, section, grid, best, base, part, scale)
        # The retention (A.6.4) of each point's closest finger; a finger covering more than
        # a point's best corrects it, and that point's closest design is chosen again.
        for round_ in range(CROSS_ROUNDS):
            closest_designs = {p: [r for r in results if (r["length_mm"], r["thickness_mm"]) == p
                                   and r.get("found") not in (False, "False")] for p in grid}
            found = self.cross_check(reference, flexion, section, grid,
                                     {p: best[p] + closest_designs[p] for p in grid}, f"cross_closest{round_}",
                                     base, part)
            if not found:
                break
            corrections += found
            for p in grid:
                best[p] = [r for r in self.rows if (r["reference_id"], float(r["required_flexion_deg"]), r["section"],
                                                    float(r["length_mm"]), float(r["thickness_mm"]))
                           == (reference.reference_id, float(flexion), section, p[0], p[1])
                           and r.get("found") not in (False, "False")]
            results = self._closest_all(reference, flexion, section, grid, best, base, part, scale,
                                        label=f"closest{round_ + 1}")
        self.corrections = corrections
        return results

    def _closest_all(self, reference, flexion, section, grid, best, base, part, scale, label="closest"):
        results = []
        for point in grid:
            config = self._config(reference, flexion, *point)
            top = self._best(best[point])
            result = {"reference_id": reference.reference_id, "mechanism": reference.mechanism,
                      "upper_bound": reference.upper_bound, "required_flexion_deg": float(flexion),
                      "section": section, "length_mm": point[0], "thickness_mm": point[1],
                      "pad_coverage_reference_lf": 2 * point[0] / scale}
            if top is None:
                results.append({**result, "found": False})
                continue
            floor = float(top["c_pad_mean"]) - float(self.config["coverage_delta"])
            warm = [self._x(r) for r in best[point] if r.get("found") not in (False, "False")]
            # The reference itself carrying this interface (Study 1's rule): its motion is the reference's
            # (eps_kin 0), so where it is near the best coverage it is the answer, and otherwise a start.
            native = carry(reference, reference.x, self._frame(reference, flexion), point, flexion)
            if native.get("feasible") and native.get("x_composed") is not None:
                warm.insert(0, tuple(native["x_composed"]))

            def run(settings, warm, config=config, floor=floor):
                return closest(base, part, config, settings, *self.ref_morphology, floor, warm=warm,
                               workers=self.workers, reference_path=self.ref_path,
                               objective=self.config.get("closest_by", "kinematic_cost"))

            near = self._best(self.search(reference, flexion, section, point, label, self.config["closest"],
                                          warm, run)) or top
            if float(near["c_pad_mean"]) > float(top["c_pad_mean"]):  # the closest search found more: the best now
                top = near
            pass_best = {name: self._best([r for r in best[point] if r["pass"] == name]) for name in PASSES}
            results.append({**result, "best_c_pad_mean": float(top["c_pad_mean"]),
                            "best_pass": top["pass"], "best_seed": top["seed"],
                            **{f"{name}_c_pad_mean": (float(r["c_pad_mean"]) if r else math.nan)
                               for name, r in pass_best.items()},
                            **{k: v for k, v in near.items()
                               if k not in ("reference_id", "mechanism", "required_flexion_deg", "section",
                                            "length_mm", "thickness_mm", "pass", "seed", "evaluations")}})
        return results


def retention(study: Reoptimization, reference, flexion: float, section: str, grid, results) -> list[dict]:
    """The cross-evaluation (A.6.4): each point's closest finger carrying every point's interface."""

    by_point = {(r["length_mm"], r["thickness_mm"]): r for r in results}
    frame = study._frame(reference, flexion)
    rows = []
    for a in grid:
        design = by_point[a]
        if design.get("found") in (False, "False"):
            continue
        x = tuple(float(v) for v in str(design["x"]).split(";"))
        for b in grid:
            target = by_point[b]
            best = float(target["best_c_pad_mean"]) if target.get("found") not in (False, "False") else math.nan
            carried = carry(reference, x, frame, b, flexion)
            accepted = bool(carried.get("feasible"))
            coverage = float(carried["c_pad_mean"]) if carried.get("c_pad_mean") is not None else math.nan
            rho = (coverage / best if accepted and best > 0 else 0.0) if math.isfinite(best) else math.nan
            rows.append({"reference_id": reference.reference_id, "mechanism": reference.mechanism,
                         "upper_bound": reference.upper_bound, "required_flexion_deg": float(flexion),
                         "section": section, "a_length_mm": a[0], "a_thickness_mm": a[1],
                         "b_length_mm": b[0], "b_thickness_mm": b[1], "accepted": accepted,
                         "failure": carried.get("failure", ""), "q_max_deg": carried.get("q_max_deg"),
                         "c_pad_mean": coverage if accepted else math.nan, "c_star_b": best, "rho": rho})
    return rows


def summarize(rows: list[dict], reference, flexion: float, section: str, family, corrections: int) -> dict:
    """The best single finger's worst retention over the family (``rho_one``, A.6.4)."""

    worst = {}
    for r in rows:
        if not in_family(section, (r["b_length_mm"], r["b_thickness_mm"]), family):
            continue
        a = (r["a_length_mm"], r["a_thickness_mm"])
        rho = r["rho"] if math.isfinite(r["rho"]) else 0.0
        worst[a] = min(worst.get(a, math.inf), rho)
    best = max(worst.items(), key=lambda item: item[1]) if worst else (None, math.nan)
    above = [r["rho"] for r in rows if math.isfinite(r["rho"]) and r["rho"] > 1.0 + 1e-6]
    return {"reference_id": reference.reference_id, "mechanism": reference.mechanism,
            "upper_bound": reference.upper_bound, "required_flexion_deg": float(flexion), "section": section,
            "family": json.dumps(family or {}), "rho_one": best[1],
            "one_finger_length_mm": best[0][0] if best[0] else math.nan,
            "one_finger_thickness_mm": best[0][1] if best[0] else math.nan,
            "corrections": corrections, "rho_above_one_left": len(above)}


def reference_row(reference, flexion: float) -> dict[str, Any]:
    part = shared.partitions.partition(reference.mechanism)
    config = shared.with_required_flexion(reference.config, flexion)
    evaluation = mechanism(reference.mechanism).evaluate(reference.x, config)
    return {"reference_id": reference.reference_id, "mechanism": reference.mechanism,
            "required_flexion_deg": flexion, "upper_bound": reference.upper_bound,
            "c_pad_mean": evaluation.c_pad_mean, "q_max_deg": getattr(evaluation, "q_max_deg", math.nan),
            "proximal_support_length_mm": getattr(evaluation, "proximal_support_length_mm", math.nan),
            "distal_support_length_mm": getattr(evaluation, "distal_support_length_mm", math.nan),
            **{f"d_{k}": v for k, v in part.external(reference.x, config).items()},
            **{f"k_{k}": v for k, v in part.mechanism(reference.x, config).items()},
            "x": ";".join(f"{v:.6g}" for v in reference.x), "source": reference.source}


def resolve(config: dict[str, Any], preset: str | None = None, study: str | None = None) -> dict[str, Any]:
    """The config with its study's settings (``finger_length_mm``, ``cross_evaluation``,
    ``output``) and its preset's ``directions``, ``search`` and ``closest``; a config
    without studies or presets as is."""

    studies = config.get("studies")
    if studies:
        name = str(study or config.get("study", "2b"))
        if name not in studies:
            raise ValueError(f"unknown study {name!r}; known: {sorted(studies)}")
        config = {**config, **studies[name], "study": name}
    presets = config.get("presets")
    if not presets:
        return config
    name = preset or config.get("preset", "full")
    if name not in presets:
        raise ValueError(f"unknown preset {name!r}; known: {sorted(presets)}")
    return {**config, **presets[name], "preset": name}


def run(path: Path, resume: bool = False, only_references=None, only_flexions=None, only_sections=None,
        finger_length="config", output: Path | None = None, preset: str | None = None,
        study_name: str | None = None) -> Path:
    config = resolve(read_config_file(path), preset, study_name)
    references = shared.references(read_config_file(REPOSITORY_ROOT / config["references"]), ("native",),
                                   shared.catalog(REPOSITORY_ROOT)["baseline"].modules)
    if only_references:
        references = [r for r in references if r.reference_id in only_references]
    finger = config.get("finger_length_mm") if finger_length == "config" else finger_length
    finger = None if finger is None else tuple(float(v) for v in finger)
    output = REPOSITORY_ROOT / (output or config["output"])
    output.mkdir(parents=True, exist_ok=True)
    study = Reoptimization(config, output, resume, finger)
    sections = {k: v for k, v in config["sections"].items() if not only_sections or k in only_sections}
    flexions = [float(q) for q in config["required_flexion_deg"]]
    if only_flexions:
        flexions = [q for q in flexions if q in only_flexions]
    results, refs, kept, summary = [], [], [], []
    families = config.get("family") or {}
    for reference in references:
        for flexion in shared.flexions_for(reference, flexions):
            refs.append(reference_row(reference, flexion))
            for section, values in sections.items():
                grid = points(section, values)
                found = study.solve(reference, flexion, section, grid)
                results += found
                shared.write_csv(output / "points.csv", results)
                if not config.get("cross_evaluation", True):
                    continue
                matrix = retention(study, reference, flexion, section, grid, found)
                kept += matrix
                shared.write_csv(output / "retention.csv", kept)
                summary.append(summarize(matrix, reference, flexion, section, families.get(section),
                                         study.corrections))
                shared.write_csv(output / "summary.csv", summary)
    shared.write_csv(output / "references.csv", refs)
    (output / "manifest.json").write_text(json.dumps({
        "config": str(path), "formulation": "problem_formulation.md A.6.3 (re-optimization, 2026-11-20)",
        "finger_length_mm": finger, "coverage_delta": config["coverage_delta"], "sections": sections,
        "study": config.get("study"), "preset": config.get("preset"), "directions": config.get("directions", list(PASSES)),
        "closest_by": config.get("closest_by", "kinematic_cost"),
        "family": families, "search": config["search"], "closest": config["closest"],
        **shared.git_commit(output)}, indent=2))
    return output


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("config", nargs="?", default=str(DEFAULT_CONFIG))
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--references")
    parser.add_argument("--flexions")
    parser.add_argument("--sections")
    parser.add_argument("--finger-length", help="'fixed' (the reference's) or 'low,high' mm (default: the config's)")
    parser.add_argument("--output")
    parser.add_argument("--preset", help="quick or full (default: the config's)")
    parser.add_argument("--study", help="2a (finger length free) or 2b (held, cross-evaluation; default: the config's)")
    args = parser.parse_args(argv)
    finger = "config"
    if args.finger_length == "fixed":
        finger = None
    elif args.finger_length:
        finger = [float(v) for v in args.finger_length.split(",")]
    output = run(Path(args.config), args.resume,
                 args.references.split(",") if args.references else None,
                 [float(q) for q in args.flexions.split(",")] if args.flexions else None,
                 args.sections.split(",") if args.sections else None, finger, args.output, args.preset, args.study)
    print(f"wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
