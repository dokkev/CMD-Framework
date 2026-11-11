#!/usr/bin/env python3
"""Fixed-H proximal/distal interface-family experiment; existing evaluator, new protocol.

Run from design/: python scripts/study/study1_fixed_morphology/interface_family.py
"""
from __future__ import annotations

import hashlib
import itertools
import json
import shutil
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import run as study1

shared = study1.shared
HERE = Path(__file__).resolve().parent
ROOT = study1.REPOSITORY_ROOT
CONFIG = HERE / "interface_family.yaml"
REFERENCES = {}
FLEXIONS = ()


def initialize():
    global FLEXIONS
    config = study1.read_config_file(CONFIG)
    FLEXIONS = tuple(config["sampled_flexion_deg"])
    nominal = shared.load_references(("matched",), ROOT, config["references"], config["servo_tilt_sweep"])
    shared.check_comparable(nominal)
    for ref in nominal:
        REFERENCES[(ref.reference_id, "nominal")] = ref
        for rom in config["required_flexion_deg"]:
            path = ROOT / config["conditioned_baselines"] / f"rom_{rom}" / ref.reference_id / "baseline.json"
            saved = json.loads(path.read_text())
            if saved["baseline_x"] is None:
                raise ValueError(f"Missing conditioned baseline: {path}")
            if tuple(saved["baseline_x"]) != tuple(saved["nominal_x"]):
                found = study1.mechanism(ref.mechanism)
                cfg = found.config_from_json(saved["config"])
                REFERENCES[(ref.reference_id, f"conditioned_{rom}")] = replace(
                    ref, x=tuple(saved["baseline_x"]), config=cfg)


def evaluate(job):
    ref_id, variant, lp, ld, tp, td = job
    pair = shared.ModulePair(f"L{lp}_{ld}_T{tp}_{td}", "cartesian", lp, tp, ld, td)
    rows = study1.assembly(REFERENCES[(ref_id, variant)], pair, FLEXIONS)
    return [{**r, "variant": variant} for r in rows]


def main():
    config = study1.read_config_file(CONFIG)
    output = ROOT / config["output"]
    output.mkdir(parents=True, exist_ok=False)
    initialize()
    inputs = output / "inputs"
    inputs.mkdir()
    shutil.copy2(CONFIG, inputs / CONFIG.name)
    baseline_inputs = []
    nominal = {ref.reference_id: ref for (rid, variant), ref in REFERENCES.items() if variant == "nominal"}
    for rid in nominal:
        for rom in config["required_flexion_deg"]:
            source = ROOT / config["conditioned_baselines"] / f"rom_{rom}" / rid / "baseline.json"
            target = inputs / f"rom_{rom}" / rid / "baseline.json"
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            baseline_inputs.append({"path": str(target.relative_to(output)),
                                    "sha256": hashlib.sha256(target.read_bytes()).hexdigest()})
    shared.write_csv(output / "references.csv", [
        {"reference_id": rid, "variant": variant, "mechanism": ref.mechanism,
         "upper_bound": ref.upper_bound, "x": ";".join(map(repr, ref.x)),
         **shared.partitions.partition(ref.mechanism).external(ref.x, ref.config)}
        for (rid, variant), ref in REFERENCES.items()])
    manifest = {"study": "fixed_H_interface_family", "status": "running", "config": config,
                "source": shared.git_commit(output), "baseline_inputs": baseline_inputs,
                "formulations": shared.formulations(list(nominal.values())),
                "protocol": "Fixed skeleton/actuation per variant; interface-dependent shallowest-supported seating. "
                            "Primary: previously selected ROM baseline. Nominal is separate control. "
                            "Endpoint and conjunction of sampled postures reported separately; no optimization."}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str) + "\n")
    jobs = [(rid, variant, lp, ld, tp, td)
            for rid, variant in REFERENCES
            for lp, ld in config["length_pairs_mm"]
            for tp, td in itertools.product(config["thickness_mm"], repeat=2)]
    rows = []
    with ProcessPoolExecutor(config["workers"], initializer=initialize) as pool:
        for i, result in enumerate(pool.map(evaluate, jobs, chunksize=12), 1):
            rows.extend(result)
            if i % 864 == 0:
                print(f"{i}/{len(jobs)} interface pairs evaluated", flush=True)
    shared.write_csv(output / "assembly_results.csv", rows)
    manifest.update(status="complete", evaluations=len(rows), interface_pairs=len(jobs))
    manifest["results_sha256"] = hashlib.sha256((output / "assembly_results.csv").read_bytes()).hexdigest()
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str) + "\n")
    print(f"wrote {output}", flush=True)


if __name__ == "__main__":
    main()
