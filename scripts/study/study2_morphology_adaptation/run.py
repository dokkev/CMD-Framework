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
import importlib.util
import json
import math
import shutil
import sys
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
    save_step,
)
from core.optimization.analysis.tradeoff import with_modules  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402
from core.optimization.optimizer import OptimizerSettings  # noqa: E402

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


shared = _module("study_shared", HERE.parent / "shared.py")
partitions = shared.partitions
Reference = shared.Reference
load_config = shared.load_config
references = shared.references


def _round(value):
    if isinstance(value, float):
        return round(value, DIGITS) if math.isfinite(value) else value
    return value


def summary_row(reference: Reference, target, result: AdaptationResult, scale_mm: float) -> dict[str, Any]:
    """One reference and interface: each level's outcome and the final change, group by group."""

    level1 = result.step("level1")
    final = result.final

    def status(step):
        return "skipped" if step is None else step.status

    def cost(step, name):
        return math.nan if step is None or step.evaluation is None else getattr(step.evaluation, name)

    external, refined = result.step("level2_external"), result.step("level2_mechanism")
    row = {"reference_id": reference.reference_id, "kind": reference.kind, "mechanism": reference.mechanism,
           "upper_bound": reference.upper_bound, "target": target.module_id,
           "frozen_feasible": result.status == "frozen_feasible",
           "level1_status": status(level1), "level1_mechanism_cost": cost(level1, "mechanism_cost"),
           # Level 2's two searches: the least external change, then the mechanism's within it.
           "level2_external_status": status(external),
           "level2_external_external_cost": cost(external, "external_cost"),
           "level2_external_mechanism_cost": cost(external, "mechanism_cost"),
           "level2_mechanism_status": status(refined),
           "level2_mechanism_external_cost": cost(refined, "external_cost"),
           "level2_mechanism_mechanism_cost": cost(refined, "mechanism_cost"),
           "status": result.status, "final_level": "" if final is None else final.level,
           "final_mechanism_cost": result.mechanism_cost, "final_external_cost": result.external_cost}
    reference_external = AdaptationMechanism(
        mechanism(reference.mechanism), partitions.partition(reference.mechanism), reference.x,
        reference.config, "mechanism", "mechanism_cost", scale_mm).reference_external
    # The least eps_E the module itself forces; the mechanisms differ only above it.
    row["external_floor"] = shared.span_floor(reference_external, target.modules, reference.config)
    row["external_excess"] = result.external_cost - row["external_floor"]
    if final is not None:
        evaluation = final.evaluation
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


def best_rows(rows: list[dict[str, Any]], delta: float) -> list[dict[str, Any]]:
    """Each reference and target over its seeds: the best outcome by the
    hierarchy's own rule (frozen; then Level 1, least ``eps_Q``; then Level 2,
    least ``eps_E`` and, among the seeds within ``delta`` of it, as ``adapt``'s
    cap allows, least ``eps_Q``), how many seeds found one, and the spread of
    their changes."""

    rank = {"frozen_feasible": 0, "level1": 1, "level2": 2, "not_found": 3}
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault((row["reference_id"], row["target"]), []).append(row)
    best = []
    for group in grouped.values():
        level = min(rank[row["status"]] for row in group)
        candidates = [row for row in group if rank[row["status"]] == level]
        if level == rank["level2"]:
            least = min(row["final_external_cost"] for row in candidates)
            candidates = [row for row in candidates if row["final_external_cost"] <= least + delta + 1e-12]
        chosen = min(candidates, key=lambda row: (row["final_mechanism_cost"], row["final_external_cost"])
                     if level != rank["not_found"] else (row["seed"],))
        found = [row for row in group if row["status"] != "not_found"]
        external = [row["final_external_cost"] for row in found]
        mech = [row["final_mechanism_cost"] for row in found]
        statuses = sorted({row["status"] for row in group})
        best.append({**{k: v for k, v in chosen.items() if k != "seed"}, "best_seed": chosen["seed"],
                     "seeds_run": len(group), "seeds_found": len(found), "seed_statuses": ";".join(statuses),
                     "external_cost_min": min(external, default=math.nan),
                     "external_cost_max": max(external, default=math.nan),
                     "mechanism_cost_min": min(mech, default=math.nan),
                     "mechanism_cost_max": max(mech, default=math.nan)})
    return best


def run(config_path: Path, root: Path = REPOSITORY_ROOT, only_targets=None, only_references=None) -> Path:
    config = read_config_file(config_path)
    output = root / config["output"]
    output.mkdir(parents=True, exist_ok=True)
    reference_values = read_config_file(root / config["references"])
    target_values = read_config_file(root / config["targets"])
    catalog = {pair.module_id: pair for pair in shared.module_catalog(read_config_file(root / target_values["catalog"]))}
    targets = [catalog[name] for name in target_values["targets"]]
    baseline = catalog["baseline"].modules
    built = references(reference_values, tuple(config["kinds"]), baseline, root)
    rules = shared.check_comparable(built)
    only_targets = only_targets or config.get("only_targets")
    only_references = only_references or config.get("only_references")
    if only_targets:
        targets = [t for t in targets if t.module_id in only_targets]
    if only_references:
        built = [r for r in built if r.reference_id in only_references]
    # A sensitivity run: goal fields set on every reference alike (recorded in the manifest).
    overrides = config.get("goal_overrides") or {}
    if overrides:
        import dataclasses

        from core.optimization.analysis.tradeoff import goal_of, with_goal

        built = [dataclasses.replace(r, config=with_goal(r.config, dataclasses.replace(goal_of(r.config), **overrides)))
                 for r in built]
        rules = shared.check_comparable(built)
    optimizer = config["optimizer"]
    seeds = [int(seed) for seed in optimizer["seeds"]]
    commit = shared.git_commit(output)
    from core.optimization.analysis.adaptation import config_resolution

    delta = config_resolution(built[0].config) / shared.partitions.partition(built[0].mechanism).external(
        built[0].x, built[0].config)["finger_length_mm"] if built else 0.0
    rows, reference_rows = [], []
    for reference in built:
        part = partitions.partition(reference.mechanism)
        external = part.external(reference.x, reference.config)
        reference_rows.append({"reference_id": reference.reference_id, "kind": reference.kind,
                               "mechanism": reference.mechanism, "upper_bound": reference.upper_bound,
                               "config": reference.config_path, "source": reference.source,
                               "formulation": mechanism(reference.mechanism).formulation,
                               **{f"e_{k}": _round(v) for k, v in external.items()},
                               **{f"q_{k}": _round(v) for k, v in part.mechanism(reference.x, reference.config).items()},
                               "x": ";".join(repr(float(v)) for v in reference.x)})
    shared.write_csv(output / "references.csv", reference_rows)
    for target in targets:
        for reference in built:
            found = mechanism(reference.mechanism)
            part = partitions.partition(reference.mechanism)
            carried = with_modules(reference.config, target.modules)
            scale = part.external(reference.x, carried)["finger_length_mm"]
            pair = output / "targets" / target.module_id / reference.reference_id
            if pair.is_dir():  # this study's own earlier result for the pair: replaced
                shutil.rmtree(pair)
            for seed in seeds:
                settings = OptimizerSettings(optimizer["algorithm"], int(optimizer["population"]),
                                             int(optimizer["generations"]), seed)
                print(f"{target.module_id} / {reference.reference_id} / seed {seed} ...", flush=True)
                result = adapt(found, part, reference.x, carried, settings, scale,
                               float(config["external_range_mm"]), workers=int(config.get("workers", 1)))
                for step in result.steps:
                    if step.result is not None:
                        save_step(step, pair / f"seed_{seed}" / step.level, source_git_commit=commit.get("commit"))
                row = {"seed": seed, **summary_row(reference, target, result, scale)}
                rows.append(row)
                print(f"  {row['status']}: eps_Q {row['final_mechanism_cost']}, eps_E {row['final_external_cost']}",
                      flush=True)
                shared.write_csv(output / "summary.csv", rows)
                shared.write_csv(output / "summary_best.csv", best_rows(rows, delta))
                if result.status == "frozen_feasible":  # no search ran: every seed's the same
                    break
    manifest = {
        "study": "study2_morphology_adaptation", "config": config,
        "run_filters": {"targets": [t.module_id for t in targets], "references": [r.reference_id for r in built]},
        "lexicographic_delta": delta,
        "references": reference_values, "targets": target_values,
        "descriptors": list(DESCRIPTORS),
        "partitions": {name: {"coordinates": list(cls.coordinates), "actuation": list(cls.actuation)}
                       for name, cls in partitions.PARTITIONS.items()},
        "normalization": "every change divided by the reference finger's length",
        "optimizer": {"algorithm": optimizer["algorithm"], "population": int(optimizer["population"]),
                      "generations": int(optimizer["generations"]), "seeds": seeds},
        "comparison_rules": rules, "formulations": shared.formulations(built),
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
