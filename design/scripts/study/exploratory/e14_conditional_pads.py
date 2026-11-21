#!/usr/bin/env python3
"""E14: linkage first, pads sampled given the linkage (conditional sampling).

    python scripts/study/exploratory/e14_conditional_pads.py [--flexion 120] [--thickness 7]
        [--samples 200000] [--workers 6] [--tries 3]

The same linkages (closure coordinates, transmission-angle filter; E12's
seeds) with their pads drawn two ways:

- joint: the pads' ends drawn with the linkage, as shares of their ranges (E12);
- conditional: the linkage's motion is computed once, the pads' exact
  clearance over the required flexion (Study 0, proposition 1; every degree)
  is evaluated on a grid of arrangements, and the pads' ends are drawn at
  random among the arrangements that never overlap and keep the least pad
  lengths. Nothing is optimized: any such arrangement may be drawn. The
  evaluator then checks the design, up to ``--tries`` draws per linkage.

Reported: feasible linkages, where the rest fail, CPU time by step and per
feasible linkage, their coverage, and how spread they are: the cells they
occupy on a grid of (A_x: 5 mm, delta: 15 deg, assembly) - a stand-in for the
number of linkage families, which is not measured here. Writes
``output/study/exploratory/e14_conditional_pads/``.
"""

from __future__ import annotations

import argparse
import collections
import csv
import math
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT, ROOT / "scripts" / "study", ROOT / "scripts" / "study" / "exploratory"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import e12_closure_sampling as e12  # noqa: E402
from core.geometry import fourbar  # noqa: E402
from core.optimization.analysis.closure_search import ClosureMechanism  # noqa: E402
from core.optimization.analysis.placement import ARRANGEMENT, DEPTHS, Linkage  # noqa: E402
from core.optimization.analysis.tradeoff import goal_of  # noqa: E402

OUTPUT = ROOT / "output" / "study" / "exploratory" / "e14_conditional_pads"
KINDS = ("joint", "conditional")
CHUNK = 2000
GRID_MM = 1.0


def arrangements(link: Linkage, config, depths) -> np.ndarray:
    """The ``(app, bpp)`` on the grid whose pads never overlap over the
    required flexion and are at least their least lengths."""

    bounds = goal_of(config).bounds
    app = np.arange(bounds[ARRANGEMENT[0]][0], bounds[ARRANGEMENT[0]][1] + 1e-9, GRID_MM)
    bpp = np.arange(bounds[ARRANGEMENT[1]][0], bounds[ARRANGEMENT[1]][1] + 1e-9, GRID_MM)
    clearance, _ = link.grid(app, bpp, *depths)
    end1, start2 = link.ends(app, bpp)
    long_enough = ((end1 - link.start >= config.minimum_proximal_support_mm)[:, None]
                   & (link.tip[0] - start2 >= config.minimum_distal_support_mm)[None, :]
                   & (end1[:, None] <= start2[None, :]))
    i, j = np.nonzero((clearance >= 0.0) & long_enough)
    return np.stack((app[i], bpp[j]), axis=1)


def chunk(job):
    kind, flexion, thickness, seed, count, tries = job
    base, config = e12.condition(flexion, thickness)
    mechanism = ClosureMechanism(base)
    names = base.variable_names
    lower, upper = (np.array(v) for v in mechanism.search_box(config))
    steps = np.array(mechanism.search_steps(config))
    rng = np.random.default_rng(seed)        # the linkages: the same for both kinds
    pads = np.random.default_rng(seed + 7919)
    spent, stages, found = collections.Counter(), collections.Counter(), []
    for _ in range(count):
        coordinates = lower + np.round(rng.random(len(lower)) * (upper - lower) / steps) * steps
        start = time.process_time()
        kept = mechanism.accepts(coordinates, config)
        x = mechanism.to_design(coordinates, config) if kept else None
        spent["linkage"] += time.process_time() - start
        if x is None:
            stages["no linkage"] += 1
            continue
        candidates = [x]
        if kind == "conditional":
            start = time.process_time()
            try:
                link = Linkage(base, x, config)
            except ValueError:
                link = None
            candidates = []
            if link is not None and link.complete and len(link.q) > 1:
                d = dict(zip(names, x))
                depths = tuple(link.h + d[name] for name in DEPTHS)
                allowed = arrangements(link, config, depths)
                for app, bpp in allowed[pads.permutation(len(allowed))[:tries]]:
                    design = link.design(float(app), float(bpp))
                    if design is not None:
                        candidates.append(design)
            spent["pads"] += time.process_time() - start
            if not candidates:
                stages["no arrangement"] += 1
                continue
        last = None
        for design in candidates:
            start = time.process_time()
            try:
                evaluation = base.evaluate(design, config)
            except ValueError:
                evaluation = None
            spent["evaluate"] += time.process_time() - start
            stages["evaluations"] += 1
            if evaluation is None:
                last = ("does not assemble", ())
                continue
            last = e12.stage_of(evaluation)
            if last[0] == "feasible":
                found.append((evaluation.c_pad_mean, *design))
                break
        stages[last[0]] += 1
        if last[0] == "motion":
            stages["motion: " + (evaluation.collision_pair or evaluation.stop_reason)] += 1
    return kind, stages, spent, found


def cells(found, names) -> int:
    seen = set()
    for row in found:
        d = dict(zip(names, row[1:]))
        pivots = [(d["a_x_mm"], 0.0), (d["ap_x_mm"], d["ap_z_mm"]), (d["b_x_mm"], 0.0), (d["bp_x_mm"], d["bp_z_mm"])]
        delta = math.degrees(fourbar.shape(*pivots)[2]) % 360.0
        seen.add((int(d["a_x_mm"] // 5.0), int(delta // 15.0), fourbar.branch_of(*pivots)))
    return len(seen)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--flexion", type=float, default=120.0)
    parser.add_argument("--thickness", type=float, default=7.0)
    parser.add_argument("--samples", type=int, default=200000)
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--tries", type=int, default=3)
    args = parser.parse_args(argv)
    base, _ = e12.condition(args.flexion, args.thickness)
    names = base.variable_names
    jobs = [(kind, args.flexion, args.thickness, 2000 + i, CHUNK, args.tries)
            for kind in KINDS for i in range(args.samples // CHUNK)]
    totals = {kind: (collections.Counter(), collections.Counter(), []) for kind in KINDS}
    with Pool(args.workers) as pool:
        for kind, stages, spent, found in pool.imap_unordered(chunk, jobs):
            totals[kind][0].update(stages)
            totals[kind][1].update(spent)
            totals[kind][2].extend(found)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for kind in KINDS:
        stages, spent, found = totals[kind]
        seconds, good = sum(spent.values()), len(found)
        linkages = args.samples - stages["no linkage"]
        coverage = np.array([row[0] for row in found])
        occupied = cells(found, names)
        print(f"\n{kind}: {args.samples} samples, {linkages} linkages past the filter and the ranges")
        for stage in ("no arrangement", "does not assemble", "bounds and pins", "servos", "pads extended", "motion",
                      "feasible"):
            if stages[stage]:
                print(f"  {stage:18s} {stages[stage]:6d}  {100.0 * stages[stage] / max(linkages, 1):6.1f}% of the linkages")
        print("  motion failures:", {k[8:]: v for k, v in stages.most_common() if k.startswith("motion: ")})
        print(f"  evaluator calls {stages['evaluations']}; cpu s: linkage {spent['linkage']:.0f}, pads {spent['pads']:.0f}, "
              f"evaluate {spent['evaluate']:.0f}" + (f"; per feasible linkage {seconds / good:.2f}" if good else ""))
        if good:
            print(f"  coverage: best {coverage.max():.3f}, median {np.median(coverage):.3f}; cells occupied {occupied} "
                  f"({occupied / seconds * 60.0:.1f} per CPU minute)")
        rows.append({"kind": kind, "samples": args.samples, "linkages": linkages, "feasible": good,
                     "feasible_of_linkages": good / max(linkages, 1), "evaluations": stages["evaluations"],
                     "cpu_linkage_s": round(spent["linkage"], 1), "cpu_pads_s": round(spent["pads"], 1),
                     "cpu_evaluate_s": round(spent["evaluate"], 1),
                     "cpu_s_per_feasible": round(seconds / good, 3) if good else "",
                     "coverage_best": round(float(coverage.max()), 4) if good else "",
                     "coverage_median": round(float(np.median(coverage)), 4) if good else "",
                     "cells": occupied, "cells_per_cpu_minute": round(occupied / seconds * 60.0, 2)})
        with (OUTPUT / f"feasible_{kind}.csv").open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(("c_pad_mean", *names))
            writer.writerows(found)
    with (OUTPUT / "summary.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nOutput: {OUTPUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
