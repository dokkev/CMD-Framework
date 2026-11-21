#!/usr/bin/env python3
"""E13: how many linkage families does the CFL's feasible region hold?

    python scripts/study/exploratory/e13_linkage_families.py [--flexion 120] [--thickness 7] [--points 7]

A linkage (its four pivots) is called feasible when the evaluator accepts it
with some pad arrangement and one of the offered servo tilts: its motion is
computed once, the arrangements whose pads never overlap are found on a grid
(Study 0, proposition 1; E14), and up to ``--tries`` of them, drawn at random,
are evaluated. A violation of at most ``--tolerance`` counts as feasible: the
designs sit on their constraints' boundaries and the pivots are on a 0.1 mm
grid.

Two feasible linkages are joined when every linkage on the straight line
between their pivots is feasible (the tilts offered: the two ends'). A family
is a set joined through others. Straight lines miss curved paths and the
designs are a sample, so the count is an upper bound on the number of
connected pieces among them, and says nothing of pieces no design fell in.

The designs: the feasible random samples of E12 and E14 and the final designs
of the condition's Study 1 runs. Writes
``output/study/exploratory/e13_linkage_families/`` (``designs.csv``).
"""

from __future__ import annotations

import argparse
import csv
import glob
import itertools
import math
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT, ROOT / "scripts" / "study", ROOT / "scripts" / "study" / "exploratory"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import e12_closure_sampling as e12  # noqa: E402
import e14_conditional_pads as e14  # noqa: E402
from core.geometry import fourbar  # noqa: E402
from core.morphology.mechanism.linkage.design import DEFAULT_VALUES  # noqa: E402
from core.optimization.analysis.placement import ARRANGEMENT, DEPTHS, Linkage  # noqa: E402

OUTPUT = ROOT / "output" / "study" / "exploratory" / "e13_linkage_families"
PIVOTS = ("a_x_mm", "ap_x_mm", "ap_z_mm", "b_x_mm", "bp_x_mm", "bp_z_mm")
_STATE = {}


def _setup(flexion, thickness, tries, tolerance):
    base, config = e12.condition(flexion, thickness)
    lower, upper = base.search_box(config)
    length = dict(zip(base.search_names(config), zip(lower, upper)))["finger_length_mm"]
    _STATE.update(base=base, config=config, tries=tries, tolerance=tolerance, length=0.5 * sum(length))
    e14.GRID_MM = 2.0


def _accepted(design) -> bool:
    try:
        evaluation = _STATE["base"].evaluate(design, _STATE["config"])
    except ValueError:
        return False
    return evaluation is not None and evaluation.max_violation <= _STATE["tolerance"]


def feasible(pivots, tilts, own=None) -> bool:
    """The linkage of ``pivots`` (``PIVOTS`` order) is feasible with one of
    ``tilts`` and some pad arrangement (``own``: one to try first)."""

    base, config = _STATE["base"], _STATE["config"]
    step = config.resolution_mm
    p = {name: round(round(value / step) * step, 9) for name, value in zip(PIVOTS, pivots)}
    x = {**DEFAULT_VALUES, **p, "a_z_mm": 0.0, "b_z_mm": 0.0, "bc_length_mm": round(_STATE["length"] - p["b_x_mm"], 9)}
    rng = np.random.default_rng(abs(hash(tuple(p.values()))) % (2 ** 32))
    allowed = None
    for tilt in tilts:
        x["servo_tilt_deg"] = tilt
        vector = tuple(x[name] for name in base.variable_names)
        try:
            link = Linkage(base, vector, config)
        except ValueError:
            return False
        if not link.complete or len(link.q) < 2:
            return False
        if own is not None:
            design = link.design(*own)
            if design is not None and _accepted(design):
                return True
        if allowed is None:
            depths = (link.h, link.h)
            first = link.design(x[ARRANGEMENT[0]], x[ARRANGEMENT[1]])
            if first is not None:
                d = dict(zip(base.variable_names, first))
                depths = tuple(link.h + d[name] for name in DEPTHS)
            allowed = e14.arrangements(link, config, depths)
            if not len(allowed):
                return False
            allowed = allowed[rng.permutation(len(allowed))[:_STATE["tries"]]]
        for app, bpp in allowed:
            design = link.design(float(app), float(bpp))
            if design is not None and _accepted(design):
                return True
    return False


def _joined(job) -> bool:
    first, second, tilts, points = job
    order = sorted(np.linspace(0.0, 1.0, points + 2)[1:-1], key=lambda t: abs(t - 0.5))  # the middle first
    return all(feasible(first + t * (second - first), tilts) for t in order)


def _own(job) -> bool:
    pivots, tilt, arrangement = job
    return feasible(pivots, (tilt,), arrangement)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--flexion", type=float, default=120.0)
    parser.add_argument("--thickness", type=float, default=7.0)
    parser.add_argument("--points", type=int, default=7)
    parser.add_argument("--tries", type=int, default=3)
    parser.add_argument("--tolerance", type=float, default=0.05)
    parser.add_argument("--workers", type=int, default=6)
    args = parser.parse_args(argv)
    setup = (args.flexion, args.thickness, args.tries, args.tolerance)
    _setup(*setup)
    base, config = _STATE["base"], _STATE["config"]
    names = base.variable_names
    designs = {}  # x -> (source, coverage)
    for source, folder, kinds in (("random", e12.OUTPUT, e12.KINDS), ("random", e14.OUTPUT, e14.KINDS)):
        for kind in kinds:
            with (folder / f"feasible_{kind}.csv").open() as handle:
                for r in csv.DictReader(handle):
                    designs.setdefault(tuple(float(r[n]) for n in names), (source, float(r["c_pad_mean"])))
    tag = f"cfl_q{args.flexion:g}" + ("" if args.thickness == 5.0 else f"_h{args.thickness:g}")
    for path in sorted(glob.glob(str(ROOT / "output" / "study" / "linkage" / tag / "run_*" / "pareto.csv"))):
        with open(path) as handle:
            for r in csv.DictReader(handle):
                x = tuple(float(r[n]) for n in names)
                evaluation = base.evaluate(x, config)
                if evaluation is not None and evaluation.feasible:
                    designs[x] = (f"study1:{Path(path).parent.name[-3:]}", evaluation.c_pad_mean)
    # One entry per linkage (the same pivots with other tilts or pads are one linkage): its best coverage.
    linkages = {}
    for x, (source, coverage) in designs.items():
        d = dict(zip(names, x))
        key = tuple(d[name] for name in PIVOTS)
        entry = linkages.setdefault(key, {"sources": set(), "coverage": -1.0, "tilts": set()})
        entry["sources"].add(source)
        entry["tilts"].add(d["servo_tilt_deg"])
        if coverage > entry["coverage"]:
            entry.update(coverage=coverage, tilt=d["servo_tilt_deg"], pads=(d[ARRANGEMENT[0]], d[ARRANGEMENT[1]]))
    keys = list(linkages)
    points = [np.array(key) for key in keys]
    with Pool(args.workers, initializer=_setup, initargs=setup) as pool:
        alone = pool.map(_own, [(points[i], linkages[k]["tilt"], linkages[k]["pads"]) for i, k in enumerate(keys)])
        kept = [i for i, ok in enumerate(alone) if ok]
        print(f"{len(designs)} feasible designs, {len(keys)} linkages; feasible by this test: {len(kept)}", flush=True)
        pairs = list(itertools.combinations(kept, 2))
        jobs = [(points[i], points[j], tuple(sorted({linkages[keys[i]]["tilt"], linkages[keys[j]]["tilt"]})),
                 args.points) for i, j in pairs]
        joined = pool.map(_joined, jobs, chunksize=40)
    parent = {i: i for i in kept}

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for (i, j), ok in zip(pairs, joined):
        if ok:
            parent[root(i)] = root(j)
    families = {}
    for i in kept:
        families.setdefault(root(i), []).append(i)
    ordered = sorted(families.values(), key=lambda members: -max(linkages[keys[i]]["coverage"] for i in members))
    print(f"pairs joined by a straight line: {sum(joined)} of {len(pairs)}; families: {len(ordered)} "
          f"(of one linkage: {sum(len(m) == 1 for m in ordered)})")
    rows = []
    print(f"{'family':>6s} {'size':>4s} {'best C':>7s} {'A_x mm':>11s} {'e mm':>11s} {'c mm':>11s} {'delta deg':>11s} "
          f"{'branch':>6s}  sources")
    for number, members in enumerate(ordered, 1):
        shapes = []
        for i in members:
            d = dict(zip(PIVOTS, keys[i]))
            pivots = [(d["a_x_mm"], 0.0), (d["ap_x_mm"], d["ap_z_mm"]), (d["b_x_mm"], 0.0), (d["bp_x_mm"], d["bp_z_mm"])]
            e, c, delta, r1, r2 = fourbar.shape(*pivots)
            branch = fourbar.branch_of(*pivots)
            shapes.append((d["a_x_mm"], e, c, math.degrees(delta) % 360.0, branch))
            rows.append({"family": number, "sources": ";".join(sorted(linkages[keys[i]]["sources"])),
                         "c_pad_mean": linkages[keys[i]]["coverage"], **d, "e_mm": e, "c_mm": c,
                         "delta_deg": math.degrees(delta) % 360.0, "r1_mm": r1, "r2_mm": r2, "branch": branch})
        s = np.array(shapes)
        span = lambda k: f"{s[:, k].min():5.1f}-{s[:, k].max():5.1f}"  # noqa: E731
        sources = sorted(set().union(*(linkages[keys[i]]["sources"] for i in members)))
        if number <= 25 or len(members) > 1:
            print(f"{number:6d} {len(members):4d} {max(linkages[keys[i]]['coverage'] for i in members):7.3f} "
                  f"{span(0):>11s} {span(1):>11s} {span(2):>11s} {span(3):>11s} "
                  f"{'/'.join(str(int(b)) for b in sorted(set(s[:, 4]))):>6s}  {', '.join(sources)}")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    with (OUTPUT / "designs.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Output: {OUTPUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
