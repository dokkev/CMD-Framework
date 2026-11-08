#!/usr/bin/env python3
"""Study 1: contact accommodation at a fixed morphology.

    python scripts/study/study1_fixed_morphology/run.py [config.yaml]

Every reference hand (Study 2's ``references.yaml``: one tracked morphology,
each architecture built to it) is frozen: its skeleton, joints, actuator and
supports. Only the contact modules change (``modules.yaml``: each module's
length and thickness), and each pad's mounting depth is the mounting policy's
for that module on the hand's own hardware (no spacer: a module needing one
does not mount). Each assembly is evaluated by its mechanism's own
``evaluate()`` at each required flexion: whether it mounts, whether it is
feasible (collisions, the flexion reached, each module usable there), its
maximum safe flexion and its usable contact coverage. A failed assembly is
reported, never redesigned.

Writes ``manifest.json``, ``references.csv``, ``module_catalog.csv`` and
``assembly_results.csv`` to the config's ``output``.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
REPOSITORY_ROOT = HERE.parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

import importlib.util  # noqa: E402


def _shared():
    if "study_shared" in sys.modules:
        return sys.modules["study_shared"]
    spec = importlib.util.spec_from_file_location("study_shared", HERE.parent / "shared.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["study_shared"] = module
    spec.loader.exec_module(module)
    return module


shared = _shared()

from core.optimization.analysis.tradeoff import with_modules  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

DEFAULT_CONFIG = HERE / "config.yaml"
MOUNTING = ("proximal_support_mm", "distal_support_mm", "mount_depth_mm")
"""The evaluators' constraints on a module mounting at all: on its segment's
room, at the policy's depth. Past them the assembly does not mount, whatever
else fails (an unmountable linkage design reads INVALID everywhere)."""
DIGITS = 6
"""Results are rounded to this many decimals (equal inputs, equal files)."""


def _round(value):
    if isinstance(value, float):
        return round(value, DIGITS) if math.isfinite(value) else value
    return value


def frozen_design(reference, modules):
    """The reference carrying ``modules``: its design as it is, each depth the
    policy's for the module (``None``: it does not mount)."""

    part = shared.partitions.partition(reference.mechanism)
    carried = with_modules(reference.config, modules)
    external = part.external(reference.x, carried)
    return part.compose(external, part.coordinate_values(reference.x, carried), reference.x, carried), carried


def assembly(reference, pair, flexions_deg) -> list[dict[str, Any]]:
    """One reference carrying one module pair, at each required flexion."""

    found = mechanism(reference.mechanism)
    part = shared.partitions.partition(reference.mechanism)
    x, carried = frozen_design(reference, pair.modules)
    rows = []
    for flexion in flexions_deg:
        row = {"assembly_id": f"{reference.reference_id}__{pair.module_id}__phi{flexion:g}",
               "reference_id": reference.reference_id, "mechanism": reference.mechanism,
               "upper_bound": reference.upper_bound, "module_id": pair.module_id, "group": pair.group,
               "proximal_length_mm": pair.proximal_length_mm, "proximal_thickness_mm": pair.proximal_thickness_mm,
               "distal_length_mm": pair.distal_length_mm, "distal_thickness_mm": pair.distal_thickness_mm,
               "required_flexion_deg": float(flexion), "mounted": x is not None}
        if x is None:
            rows.append({**row, "feasible": False, "failure": "not_mounted:no_depth"})  # the policy gives no depth (a spacer needed)
            continue
        evaluation = found.evaluate(x, shared.with_required_flexion(carried, flexion))
        report = found.report(evaluation)
        constraints = evaluation.constraints
        unmounted = [name for name in MOUNTING if constraints.get(name, 0.0) > 0.0]
        if unmounted:
            rows.append({**row, "mounted": False, "feasible": False, "failure": "not_mounted:" + ";".join(unmounted)})
            continue
        failed = sorted((name for name, value in constraints.items() if value > 0.0), key=lambda n: -constraints[n])
        values = part.values(x)
        row.update({
            "feasible": bool(evaluation.feasible), "failure": ";".join(failed),
            "proximal_depth_mm": values[part.depths[0]], "distal_depth_mm": values[part.depths[1]],
            "phi_max_deg": float(report["phi_max_deg"]), "stop_reason": report.get("stop_reason"),
            "collision_pair": report.get("collision_pair"),
            "coverage": float(report["pad_coverage"]),
            "proximal_effective_mm": float(report["proximal_effective_mm"]),
            "distal_effective_mm": float(report["distal_effective_mm"]),
            "finger_length_mm": float(report["finger_length_mm"]),
        })
        rows.append(row)
    return [{name: _round(value) for name, value in row.items()} for row in rows]


_REFERENCES: dict[str, Any] = {}
"""Each worker's references, built once (the configs do not pickle)."""


def _load(kinds, root) -> None:
    _REFERENCES.update({r.reference_id: r for r in shared.load_references(kinds, root)})


def _assembly(job):
    reference_id, pair, flexions = job
    return assembly(_REFERENCES[reference_id], pair, flexions)


def run(config_path: Path = DEFAULT_CONFIG, root: Path = REPOSITORY_ROOT, output: Path | None = None,
        workers: int | None = None) -> Path:
    """The whole study as ``config_path`` says; returns the output folder."""

    config = read_config_file(config_path)
    output = output or root / config["output"]
    output.mkdir(parents=True, exist_ok=True)
    kinds = tuple(config.get("kinds", ("matched",)))
    found = shared.load_references(kinds, root)
    rules = shared.check_comparable(found)
    pairs = shared.module_catalog(read_config_file(root / config["modules"]))
    flexions = [float(v) for v in config["required_flexion_deg"]]
    jobs = [(reference.reference_id, pair, flexions) for reference in found for pair in pairs]
    workers = int(config.get("workers", 1)) if workers is None else workers
    if workers > 1:
        with ProcessPoolExecutor(workers, initializer=_load, initargs=(kinds, root)) as pool:
            rows = [row for rows in pool.map(_assembly, jobs, chunksize=4) for row in rows]
    else:
        _REFERENCES.update({r.reference_id: r for r in found})
        rows = [row for job in jobs for row in _assembly(job)]
    shared.write_csv(output / "assembly_results.csv", rows)
    shared.write_csv(output / "module_catalog.csv", [{"module_id": p.module_id, "group": p.group,
                                                       "proximal_length_mm": p.proximal_length_mm,
                                                       "proximal_thickness_mm": p.proximal_thickness_mm,
                                                       "distal_length_mm": p.distal_length_mm,
                                                       "distal_thickness_mm": p.distal_thickness_mm}
                                                      for p in pairs])
    reference_rows = []
    for reference in found:
        part = shared.partitions.partition(reference.mechanism)
        external = part.external(reference.x, reference.config)
        reference_rows.append({"reference_id": reference.reference_id, "kind": reference.kind,
                               "mechanism": reference.mechanism, "upper_bound": reference.upper_bound,
                               "config": reference.config_path, "source": reference.source,
                               "formulation": mechanism(reference.mechanism).formulation,
                               **{f"e_{k}": _round(v) for k, v in external.items()},
                               "x": ";".join(repr(float(v)) for v in reference.x)})
    shared.write_csv(output / "references.csv", reference_rows)
    manifest = {
        "study": "study1_fixed_morphology", "config": config,
        "references": read_config_file(root / shared.REFERENCES),
        "modules": read_config_file(root / config["modules"]),
        "comparison_rules": rules, "formulations": shared.formulations(found),
        "evaluation": "each mechanism's own evaluate(); depths by the goal's mounting policy for each module",
        "source": shared.git_commit(output),
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str) + "\n", encoding="utf-8")
    return output


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("config", nargs="?", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args(argv)
    print(f"wrote {run(args.config)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
