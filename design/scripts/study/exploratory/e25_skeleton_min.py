#!/usr/bin/env python3
"""Experimental: the skeleton-keeping redesign ("Conventional") in Study 2's own design space, the counterpart of the
Study 2 redesign (least Delta_contact) and the coverage-only design (docs/experimental/pr_mr.md).

    python scripts/study/exploratory/e25_skeleton_min.py [--to 5 7 9] [--seeds 0 1] [--coverage_floor 0.70]

Same mechanism, goal and bounds as Study 2 (A and B on the link line, the mounting rule); NSGA-II on Delta_skel (the
fingertip path from the proximal link line, against the 3 mm design) and -C_pad, warm-started from Study 1's design
for that thickness and the Study 2 redesign. Picks the least Delta_skel at C_pad no more than 0.05 below the Study 2 redesign's at that thickness ("Conv"; 2026-11-27:
a fixed floor of 0.70 is unfair at 9 mm, where the Study 2 redesign itself has 0.695).

Writes ``output/study/exploratory/e25_skeleton_min_h<t>/``: ``front.csv`` and ``picks.csv`` (label ``Conv``, e20's
columns, so e20_make_hands.py turns it into a hand file).
"""
import argparse
import csv
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
import e20_mcp_referenced as E  # noqa: E402
from core.optimization.analysis.contact_kinematics import INVALID, paths  # noqa: E402
from core.optimization.mechanism import Objective, mechanism, read_config_file  # noqa: E402
from core.optimization.optimizer import OptimizerSettings  # noqa: E402
from core.optimization.study import run_study  # noqa: E402


class SkeletonMin(E.MCPReferenced):
    def __init__(self, *a):
        super().__init__(*a)
        s = self.scale_mm
        self.name = f"skeleton_min:{self.base.name}"
        self.objectives = {
            "skeleton_change": Objective(lambda e: e.skeleton_change * s if e.skeleton_change < INVALID else INVALID,
                                         "skeleton_change", "D_s", "least_skeleton_change", False),
            "c_pad_mean": Objective(lambda e: -e.c_pad_mean, "c_pad_mean", "C_pad", "most_coverage", True)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--to", type=float, nargs="+", default=(5.0, 7.0, 9.0))
    ap.add_argument("--seeds", type=int, nargs="+", default=(0, 1))
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--coverage_floor", type=float, default=0.70)
    args = ap.parse_args(argv)
    config = read_config_file(ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    designs = E.e15.study1.study1_designs("cfl", config["architectures"]["cfl"], 120.0, config)
    h0 = designs[3.0]
    base = mechanism(h0.mechanism)
    reference = paths(base, h0.x, h0.config)
    rep = base.report(base.evaluate(h0.x, h0.config))
    scale = float(rep["finger_length_mm"])
    mech = SkeletonMin(base, reference, float(rep["proximal_depth_mm"]), scale)
    with (E.e15.STUDY2 / "front.csv").open(encoding="utf-8", newline="") as stream:
        study2 = [r for r in csv.DictReader(stream) if r["architecture"] == "cfl" and float(r["required_flexion_deg"]) == 120]
    for t in args.to:
        goal = designs[t]
        warm = [tuple(goal.x)]
        own = [r for r in study2 if float(r["to_mm"]) == t]
        if own:
            warm.append(tuple(float(v) for v in max(own, key=lambda r: float(r["c_pad_mean"]))["x"].split(";")))
        rows = []
        for seed in args.seeds:
            t0 = time.time()
            res = run_study(mech, goal.config, OptimizerSettings("nsga2", 200, 200, seed), initial_designs=warm,
                            workers=args.workers)
            for c in res.pareto:
                e = c.evaluation
                if e.feasible and e.skeleton_change < INVALID:
                    r = base.report(e.base)
                    rows.append({"seed": seed, "mcp_change_mm": e.mcp_change * scale,
                                 "contact_change_mm": e.contact_change * scale,
                                 "skeleton_change_mm": e.skeleton_change * scale, "c_pad_mean": e.c_pad_mean,
                                 "ma_wc": r.get("ma_wc"), "face_depth_proximal_mm": r.get("proximal_depth_mm"),
                                 "x": ";".join(f"{v:.6g}" for v in c.x)})
            print(f"{t:g} mm seed {seed}: {len(res.pareto)} front, {time.time() - t0:.0f} s", flush=True)
        out = ROOT / "output" / "study" / "exploratory" / f"e25_skeleton_min_h{t:g}"
        out.mkdir(parents=True, exist_ok=True)
        E.shared.write_csv(out / "front.csv", rows)
        floor = (max(float(r["c_pad_mean"]) for r in own) - 0.05) if own else args.coverage_floor
        ok = [r for r in rows if r["c_pad_mean"] >= floor]
        if not ok:
            print(f"{t:g} mm: no design with C_pad >= {args.coverage_floor}")
            continue
        best = {**min(ok, key=lambda r: r["skeleton_change_mm"]), "label": "Conv"}
        E.shared.write_csv(out / "picks.csv", [best])
        print(f"{t:g} mm Conv: D_c {best['contact_change_mm']:.2f} D_MCP {best['mcp_change_mm']:.2f} "
              f"D_s {best['skeleton_change_mm']:.2f} C {best['c_pad_mean']:.3f} face {best['face_depth_proximal_mm']}",
              flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
