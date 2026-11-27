#!/usr/bin/env python3
"""Experimental (playground): redesigns that keep Delta_contact, Delta_MCP and Delta_skel small together
(docs/experimental/pr_mr.md). NSGA-II, A and B heights free, objectives max(Delta_contact, Delta_MCP), Delta_skel, -C_pad.
Picks at C_pad >= floor: the least max(Delta_contact, Delta_MCP, Delta_skel) ("MRS").

    python scripts/study/exploratory/e22_all_three.py [--to 7] [--seeds 0 1]
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


class AllThree(E.MCPReferenced):
    def __init__(self, *a):
        super().__init__(*a)
        s = self.scale_mm
        self.name = f"all_three:{self.base.name}"
        self.objectives = {
            "both": Objective(lambda e: max(e.mcp_change, e.contact_change) * s if e.mcp_change < INVALID else INVALID,
                              "both", "max(D_c,D_MCP)", "both", False),
            "skeleton_change": Objective(lambda e: e.skeleton_change * s if e.skeleton_change < INVALID else INVALID,
                                         "skeleton_change", "D_s", "skel", False),
            "c_pad_mean": Objective(lambda e: -e.c_pad_mean, "c_pad_mean", "C_pad", "most_coverage", True)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--to", type=float, default=7.0)
    ap.add_argument("--seeds", type=int, nargs="+", default=(0, 1))
    ap.add_argument("--workers", type=int, default=7)
    ap.add_argument("--coverage_floor", type=float, default=0.70)
    args = ap.parse_args(argv)
    config = read_config_file(ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    designs = E.e15.study1.study1_designs("cfl", config["architectures"]["cfl"], 120.0, config)
    h0, goal = designs[3.0], designs[args.to]
    base = mechanism(h0.mechanism)
    reference = paths(base, h0.x, h0.config)
    rep = base.report(base.evaluate(h0.x, h0.config))
    scale = float(rep["finger_length_mm"])
    cfg = E.with_b_z(goal.config, -3, 10, a_z=(-3, 8))
    warm = []
    for folder in ("e20_mcp_referenced_free", "e21_mcp_vs_skeleton"):
        f = ROOT / "output" / "study" / "exploratory" / folder / "picks.csv"
        if f.exists() and args.to == 7.0:
            warm += [tuple(float(v) for v in r["x"].split(";")) for r in csv.DictReader(f.open()) if r["label"] != "O"]
    mech = AllThree(base, reference, float(rep["proximal_depth_mm"]), scale)
    rows = []
    for seed in args.seeds:
        t0 = time.time()
        res = run_study(mech, cfg, OptimizerSettings("nsga2", 200, 200, seed), initial_designs=warm, workers=args.workers)
        for c in res.pareto:
            e = c.evaluation
            if e.feasible and e.mcp_change < INVALID and e.skeleton_change < INVALID:
                rows.append({"seed": seed, "mcp_change_mm": e.mcp_change * scale, "contact_change_mm": e.contact_change * scale,
                             "skeleton_change_mm": e.skeleton_change * scale, "c_pad_mean": e.c_pad_mean,
                             "ma_wc": base.report(e.base).get("ma_wc"), "face_depth_proximal_mm": base.report(e.base).get("proximal_depth_mm"),
                             "x": ";".join(f"{v:.6g}" for v in c.x)})
        print(f"seed {seed}: {len(res.pareto)} front, {time.time() - t0:.0f} s", flush=True)
    out = ROOT / "output" / "study" / "exploratory" / f"e22_all_three_h{args.to:g}"
    out.mkdir(parents=True, exist_ok=True)
    E.shared.write_csv(out / "front.csv", rows)
    ok = [r for r in rows if r["c_pad_mean"] >= args.coverage_floor]
    best = min(ok, key=lambda r: max(r["mcp_change_mm"], r["contact_change_mm"], r["skeleton_change_mm"]))
    best = {**best, "label": "MRS"}
    print(f"MRS: D_c {best['contact_change_mm']:.2f} D_MCP {best['mcp_change_mm']:.2f} D_s {best['skeleton_change_mm']:.2f} "
          f"C {best['c_pad_mean']:.3f} MA {best['ma_wc']:.2f} face {best['face_depth_proximal_mm']}")
    E.shared.write_csv(out / "picks.csv", [best])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
