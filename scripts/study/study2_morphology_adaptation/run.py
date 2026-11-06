#!/usr/bin/env python3
"""Study 2: morphology adaptation for contact accommodation.

    python scripts/study/study2_morphology_adaptation/run.py [config.yaml] [--targets a,b] [--references a,b]

For each reference hand (``references.yaml``) and contact interface
(``targets.yaml``, Study 1's catalog), the hierarchy of
``core.optimization.analysis.adaptation.adapt``: the reference as it is,
else Level 1 (mechanism geometry only), else Level 2 (external morphology
first, then the mechanism). The mechanism's own evaluation decides
feasibility; the existing optimizer searches. The two changes are reported
apart, never summed.

Writes to the config's ``output``: ``manifest.json``, ``references.csv``,
``summary.csv``, and each search's run under ``targets/<target>/<reference>/<level>``.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.analysis.adaptation import (  # noqa: E402
    DESCRIPTORS,
    AdaptationMechanism,
    AdaptationResult,
    adapt,
)
from core.optimization.analysis.tradeoff import with_modules  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402
from core.optimization.optimizer import OptimizerSettings  # noqa: E402
from core.optimization.study import save_result  # noqa: E402

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "config.yaml"
DIGITS = 6


def _module(name: str, path: Path):
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module  # before running it: its dataclasses look themselves up there
    spec.loader.exec_module(module)
    return module


partitions = _module("study2_partitions", HERE / "partitions.py")
study1 = _module("study1_run", HERE.parent / "study1_fixed_morphology" / "run.py")


@dataclass(frozen=True)
class Reference:
    """A reference hand: its design, mechanism and study config."""

    reference_id: str
    kind: str
    """``matched`` or ``native``."""
    mechanism: str
    x: tuple[float, ...]
    config: Any
    config_path: str
    source: str
    upper_bound: bool = False


def load_config(path: str, root: Path):
    """A study config file's mechanism and config (its optimizer settings aside)."""

    from core.optimization.linkage.mechanism import resolve_preset

    values = read_config_file(root / path)
    found = mechanism(values.pop("mechanism"))
    values.pop("output", None)
    resolve_preset(values)
    values, _ = found.split_settings(values)
    return found, found.config_from_mapping(values)


def references(values: dict[str, Any], kinds, baseline, root: Path) -> list[Reference]:
    """The references of ``kinds``: matched ones built to the shared morphology, native ones as found."""

    built = []
    if "matched" in kinds:
        source = study1.load_core({"id": values["morphology"]["id"], **values["morphology"]["source"]}, root)
        for spec in values["matched"]:
            found, config = load_config(spec["config"], root)
            part = partitions.partition(found.name)
            carried = with_modules(config, baseline)
            external = partitions.partition(source.mechanism).external(source.x, carried)
            coordinates, template = part.template(external, source.mechanism, source.x, carried)
            x = part.compose(external, coordinates, template, carried)
            if x is None:
                raise ValueError(f"reference {spec['id']}: no {found.name} design matches {source.source}")
            built.append(Reference(spec["id"], "matched", found.name, x, config, spec["config"],
                                   f"matched to {source.source}", bool(spec.get("upper_bound", False))))
    if "native" in kinds:
        for spec in values.get("native") or ():
            core = study1.load_core({k: v for k, v in spec.items() if k != "config"}, root)
            found, config = load_config(spec["config"], root)
            if core.mechanism != found.name:
                raise ValueError(f"reference {spec['id']}: a {core.mechanism} design under a {found.name} config")
            built.append(Reference(spec["id"], "native", found.name, core.x, config, spec["config"], core.source,
                                   bool(spec.get("upper_bound", False))))
    return built


def _round(value):
    if isinstance(value, float):
        return round(value, DIGITS) if math.isfinite(value) else value
    return value


def summary_row(reference: Reference, target, result: AdaptationResult, scale_mm: float) -> dict[str, Any]:
    """One reference and interface: each level's outcome and the final change, group by group."""

    level1 = result.step("level1")
    level2 = result.step("level2_external")
    final = result.final
    row = {"reference_id": reference.reference_id, "kind": reference.kind, "mechanism": reference.mechanism,
           "upper_bound": reference.upper_bound, "target": target.module_id,
           "frozen_feasible": result.status == "frozen_feasible",
           "level1_status": "skipped" if level1 is None else level1.status,
           "level1_mechanism_cost": math.nan if level1 is None or level1.evaluation is None
           else level1.evaluation.mechanism_cost,
           "level2_status": "skipped" if level2 is None else level2.status,
           "level2_external_cost": math.nan if level2 is None or level2.evaluation is None
           else level2.evaluation.external_cost,
           "status": result.status, "final_level": "" if final is None else final.level,
           "final_mechanism_cost": result.mechanism_cost, "final_external_cost": result.external_cost}
    if final is not None:
        evaluation = final.evaluation
        reference_external = AdaptationMechanism(
            mechanism(reference.mechanism), partitions.partition(reference.mechanism), reference.x,
            reference.config, "mechanism", "mechanism_cost", scale_mm).reference_external
        reference_mechanism = partitions.partition(reference.mechanism).mechanism(reference.x, reference.config)
        for name in DESCRIPTORS:
            row[f"{name.removesuffix('_mm')}_change_mm"] = evaluation.external[name] - reference_external[name]
        row["mechanism_change_mm"] = max((abs(evaluation.mechanism[name] - reference_mechanism[name])
                                          for name in reference_mechanism), default=0.0)
        base = evaluation.base
        row["final_phi_max_deg"] = math.degrees(base.phi_max_rad)
        row["final_clearance_mm"] = base.pad_clearance_mm
        row["final_x"] = ";".join(repr(float(v)) for v in final.x)  # exact: re-evaluates the same
    return {name: _round(value) for name, value in row.items()}


def _write_csv(path: Path, rows: list[dict]) -> None:
    columns = list(dict.fromkeys(name for row in rows for name in row))
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def run(config_path: Path, root: Path = REPOSITORY_ROOT, only_targets=None, only_references=None) -> Path:
    config = read_config_file(config_path)
    output = root / config["output"]
    output.mkdir(parents=True, exist_ok=True)
    reference_values = read_config_file(root / config["references"])
    target_values = read_config_file(root / config["targets"])
    catalog = {pair.module_id: pair for pair in study1.module_catalog(read_config_file(root / target_values["catalog"]))}
    targets = [catalog[name] for name in target_values["targets"]]
    baseline = catalog["baseline"].modules
    built = references(reference_values, tuple(config["kinds"]), baseline, root)
    if only_targets:
        targets = [t for t in targets if t.module_id in only_targets]
    if only_references:
        built = [r for r in built if r.reference_id in only_references]
    optimizer = config["optimizer"]
    settings = OptimizerSettings(optimizer["algorithm"], int(optimizer["population"]), int(optimizer["generations"]),
                                 int(optimizer.get("seed", 0)))
    commit = study1._git_commit()
    rows, reference_rows = [], []
    for reference in built:
        part = partitions.partition(reference.mechanism)
        external = part.external(reference.x, reference.config)
        reference_rows.append({"reference_id": reference.reference_id, "kind": reference.kind,
                               "mechanism": reference.mechanism, "upper_bound": reference.upper_bound,
                               "config": reference.config_path, "source": reference.source,
                               **{f"e_{k}": _round(v) for k, v in external.items()},
                               **{f"q_{k}": _round(v) for k, v in part.mechanism(reference.x, reference.config).items()},
                               "x": ";".join(repr(float(v)) for v in reference.x)})
    _write_csv(output / "references.csv", reference_rows)
    for target in targets:
        for reference in built:
            found = mechanism(reference.mechanism)
            part = partitions.partition(reference.mechanism)
            carried = with_modules(reference.config, target.modules)
            scale = part.external(reference.x, carried)["finger_length_mm"]
            print(f"{target.module_id} / {reference.reference_id} ...", flush=True)
            result = adapt(found, part, reference.x, carried, settings, scale, float(config["external_range_mm"]),
                           workers=int(config.get("workers", 1)))
            pair = output / "targets" / target.module_id / reference.reference_id
            if pair.is_dir():  # this study's own earlier result for the pair: replaced
                shutil.rmtree(pair)
            for step in result.steps:
                if step.result is not None:
                    save_result(step.result, pair / step.level, source_git_commit=commit.get("commit"))
            row = summary_row(reference, target, result, scale)
            rows.append(row)
            print(f"  {row['status']}: eps_Q {row['final_mechanism_cost']}, eps_E {row['final_external_cost']}",
                  flush=True)
            _write_csv(output / "summary.csv", rows)
    manifest = {
        "study": "study2_morphology_adaptation", "config": config,
        "references": reference_values, "targets": target_values,
        "descriptors": list(DESCRIPTORS),
        "partitions": {name: {"coordinates": list(cls.coordinates), "actuation": list(cls.actuation)}
                       for name, cls in partitions.PARTITIONS.items()},
        "normalization": "every change divided by the reference finger's length",
        "optimizer": {"algorithm": settings.algorithm, "population": settings.population_size,
                      "generations": settings.generations, "seed": settings.seed},
        "source": commit,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str) + "\n", encoding="utf-8")
    return output


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("config", nargs="?", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--targets", help="comma-separated target ids (default: all)")
    parser.add_argument("--references", help="comma-separated reference ids (default: all)")
    args = parser.parse_args(argv)
    output = run(args.config, only_targets=args.targets and args.targets.split(","),
                 only_references=args.references and args.references.split(","))
    print(f"wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
