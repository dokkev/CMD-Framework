#!/usr/bin/env python3
"""E12: where do random CFL designs fail, and what does sampling in the
closure coordinates buy? (report/study0/study0.md)

    python scripts/study/exploratory/e12_closure_sampling.py [--flexion 120] [--thickness 7]
        [--samples 200000] [--workers 8]

One CFL condition of Study 1, the same goal and evaluator, random samples of
three kinds:

- baseline: uniform in the box of the study's search coordinates;
- closure: uniform in the box of the closure coordinates
  (``core.optimization.analysis.closure_search``), where the four-bar closes
  to the required flexion by construction;
- closure + filter: the same, kept only where the transmission angle stays
  in its range over the required flexion (closed form), before the evaluator.

Uniform means uniform in each kind's own coordinates: the two boxes weigh the
physical designs differently. For each kind: the stage of the evaluation a
sample first fails at, the CPU time spent mapping, filtering and evaluating,
the time per feasible design, and the feasible designs' coverage. Writes
``output/study/exploratory/e12_closure_sampling/`` (``summary.csv``, and each
kind's feasible designs in ``feasible_<kind>.csv``).
"""

from __future__ import annotations

import argparse
import collections
import csv
import importlib
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT, ROOT / "scripts" / "study"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import shared  # noqa: E402
from core.optimization.analysis.closure_search import ClosureMechanism  # noqa: E402

OUTPUT = ROOT / "output" / "study" / "exploratory" / "e12_closure_sampling"
KINDS = ("baseline", "closure", "closure_filter")
STAGES = ("no design", "does not assemble", "bounds and pins", "servos", "pads extended", "motion", "feasible")
SERVOS = ("static_mcp_servo_mm", "static_pip_servo_pad_mm")
PADS = ("static_pad_collision_mm",)
CHUNK = 2000


def condition(flexion: float, thickness: float):
    found, config = shared.load_config("scripts/study/cfl.yaml", ROOT, None, (
        f"goal.pads.proximal.height_mm={thickness:g}", f"goal.pads.distal.height_mm={thickness:g}"))
    return found, shared.with_required_flexion(config, flexion)


def stage_of(evaluation) -> tuple[str, tuple[str, ...]]:
    """The first stage of the evaluation the design fails at, and the
    constraints it breaks there (later stages are not checked)."""

    motion = importlib.import_module("core.optimization.linkage.evaluate").MOTION_CONSTRAINTS
    c = evaluation.constraints
    first = tuple(k for k, v in c.items() if v > 0.0 and k not in SERVOS + PADS + motion)
    if first:
        return "bounds and pins", first
    for stage, names in (("servos", SERVOS), ("pads extended", PADS), ("motion", motion)):
        broken = tuple(k for k in names if c[k] > 0.0)
        if broken:
            return stage, broken
    return "feasible", ()


def chunk(job):
    kind, flexion, thickness, seed, count = job
    found, config = condition(flexion, thickness)
    base = found
    mechanism = base if kind == "baseline" else ClosureMechanism(base)
    lower, upper = (np.array(v) for v in mechanism.search_box(config))
    steps = np.array(mechanism.search_steps(config))
    rng = np.random.default_rng(seed)
    spent = collections.Counter()
    stages, broken, feasible = collections.Counter(), collections.Counter(), []
    for _ in range(count):
        coordinates = lower + np.round(rng.random(len(lower)) * (upper - lower) / steps) * steps
        if kind == "closure_filter":
            start = time.process_time()
            kept = mechanism.accepts(coordinates, config)
            spent["filter"] += time.process_time() - start
            if not kept:
                stages["filtered"] += 1
                continue
        start = time.process_time()
        x = mechanism.to_design(coordinates, config)
        spent["map"] += time.process_time() - start
        if x is None:
            stages["no design"] += 1
            continue
        start = time.process_time()
        try:
            evaluation = mechanism.evaluate(x, config)
        except ValueError:
            evaluation = None
        spent["evaluate"] += time.process_time() - start
        if evaluation is None:
            stages["does not assemble"] += 1
            continue
        stage, names = stage_of(evaluation)
        stages[stage] += 1
        broken.update(f"{stage}: {name}" for name in names)
        if stage == "feasible":
            feasible.append((evaluation.c_pad_mean, *x))
    return kind, stages, broken, spent, feasible


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--flexion", type=float, default=120.0)
    parser.add_argument("--thickness", type=float, default=7.0)
    parser.add_argument("--samples", type=int, default=200000)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args(argv)
    found, _ = condition(args.flexion, args.thickness)
    jobs = [(kind, args.flexion, args.thickness, 1000 * k + i, CHUNK)
            for k, kind in enumerate(KINDS) for i in range(args.samples // CHUNK)]
    totals = {kind: (collections.Counter(), collections.Counter(), collections.Counter(), []) for kind in KINDS}
    with Pool(args.workers) as pool:
        for kind, stages, broken, spent, feasible in pool.imap_unordered(chunk, jobs):
            for total, part in zip(totals[kind][:3], (stages, broken, spent)):
                total.update(part)
            totals[kind][3].extend(feasible)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for kind in KINDS:
        stages, broken, spent, feasible = totals[kind]
        count, good = sum(stages.values()), len(feasible)
        seconds = sum(spent.values())
        coverage = np.array([row[0] for row in feasible])
        evaluated = count - stages["filtered"] - stages["no design"]
        rows.append({
            "kind": kind, "samples": count, **{stage: stages[stage] for stage in ("filtered", *STAGES)},
            "evaluated": evaluated, "feasible_share": good / count,
            "feasible_of_evaluated": good / max(evaluated, 1),
            "cpu_map_s": round(spent["map"], 1), "cpu_filter_s": round(spent["filter"], 1),
            "cpu_evaluate_s": round(spent["evaluate"], 1),
            "cpu_s_per_feasible": round(seconds / good, 3) if good else "",
            "coverage_best": round(float(coverage.max()), 4) if good else "",
            "coverage_median": round(float(np.median(coverage)), 4) if good else "",
            "coverage_p90": round(float(np.percentile(coverage, 90)), 4) if good else "",
        })
        print(f"\n{kind}: {count} samples")
        for stage in ("filtered", *STAGES):
            if stages[stage]:
                print(f"  {stage:18s} {stages[stage]:7d}  {100.0 * stages[stage] / count:7.3f}%")
        print("  broken where it fails:", dict(broken.most_common(8)))
        print(f"  cpu s: map {spent['map']:.0f}, filter {spent['filter']:.0f}, evaluate {spent['evaluate']:.0f}"
              + (f"; per feasible design {seconds / good:.2f}" if good else ""))
        if good:
            print(f"  coverage of the feasible: best {coverage.max():.3f}, 90th percentile "
                  f"{np.percentile(coverage, 90):.3f}, median {np.median(coverage):.3f}")
        with (OUTPUT / f"feasible_{kind}.csv").open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(("c_pad_mean", *found.variable_names))
            writer.writerows(feasible)
    with (OUTPUT / "summary.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nOutput: {OUTPUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
