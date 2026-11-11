#!/usr/bin/env python3
"""Post-hoc denser posture validation of the full 20 mm thickness stratum.

All architectures and all 144 thickness pairs, not only favorable candidates.
Uses each 150-degree baseline frozen by the preceding study; no optimization.
"""
import hashlib
import itertools
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import interface_family as family


def initialize():
    family.initialize()
    family.FLEXIONS = tuple(range(5, 151, 5))
    if "--fold-off" in sys.argv:
        for key, ref in list(family.REFERENCES.items()):
            goal = family.shared.goal_of(ref.config)
            config = family.shared.with_goal(ref.config, replace(goal, count_fold=False))
            family.REFERENCES[key] = replace(ref, config=config)


def main():
    initialize()
    config = family.study1.read_config_file(family.CONFIG)
    fold_off = "--fold-off" in sys.argv
    output = family.ROOT / config["output"] / ("dense_fold_off" if fold_off else "dense_postures")
    output.mkdir(exist_ok=False)
    refs = [rid for rid, variant in family.REFERENCES if variant == "nominal"]
    jobs = []
    for rid in refs:
        variant = "conditioned_150" if (rid,"conditioned_150") in family.REFERENCES else "nominal"
        jobs.extend((rid,variant,20,20,tp,td) for tp,td in itertools.product(config["thickness_mm"], repeat=2))
    manifest = {"status": "running", "fold_rule_enabled": not fold_off,
                "source": family.shared.git_commit(output),
                "parent_manifest_sha256": hashlib.sha256((output.parent / "manifest.json").read_bytes()).hexdigest(),
                "protocol": "Post-hoc full-stratum check, all seven architectures, 20/20 mm lengths, "
                            "all 144 original thickness pairs, sampled flexion 5..150 in 5-degree steps; "
                            "150-degree conditioned baselines, no per-interface optimization."}
    (output / "manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    rows = []
    with ProcessPoolExecutor(config["workers"], initializer=initialize) as pool:
        for result in pool.map(family.evaluate,jobs,chunksize=12):
            rows.extend(result)
    family.shared.write_csv(output / "assembly_results.csv",rows)
    manifest.update(status="complete", evaluations=len(rows),
                    results_sha256=hashlib.sha256((output / "assembly_results.csv").read_bytes()).hexdigest())
    (output / "manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    print(f"wrote {len(rows)} dense-posture evaluations to {output}")


if __name__ == "__main__":
    main()
