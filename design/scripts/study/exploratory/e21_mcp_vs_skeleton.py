#!/usr/bin/env python3
"""Experimental (playground): 7 mm redesigns that pull Delta_MCP and Delta_skel apart (docs/experimental/pr_mr.md).

    python scripts/study/exploratory/e21_mcp_vs_skeleton.py [--seeds 0 1] [--generations 200]

On the paper's hands Delta_MCP (the distal pad face point's path deviation from O's, in F_1) and Delta_skel (the
fingertip's, in F_1) correlate at rho 0.93, so an F_1-anchored error that follows one also follows the other. Two
NSGA-II searches (A and B heights free, as e20's free run), three objectives each, C_pad >= floor for the picks:
- lowM: least Delta_MCP, most Delta_skel, most C_pad;
- highM: most Delta_MCP, least Delta_skel, most C_pad.
Picks: lowM designs with Delta_MCP <= 1.5 mm at the largest Delta_skel in each of the bands {4-6, 6-8, 8+} mm; highM
designs with Delta_skel <= 3 mm at the largest Delta_MCP in each of {3-4.5, 4.5-6, 6+} mm. Writes
``output/study/exploratory/e21_mcp_vs_skeleton/{front,picks}.csv``.
"""
import argparse
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

OUT = ROOT / "output" / "study" / "exploratory" / "e21_mcp_vs_skeleton"


class Split(E.MCPReferenced):
    def __init__(self, base, reference, depth, scale, mode):
        super().__init__(base, reference, depth, scale)
        s, sign = self.scale_mm, (1.0 if mode == "lowM" else -1.0)
        self.name = f"split_{mode}:{base.name}"
        self.objectives = {
            "mcp_change": Objective(lambda e: sign * e.mcp_change * s if e.mcp_change < INVALID else INVALID,
                                    "mcp_change", "D_MCP", "mcp", False),
            "skeleton_change": Objective(lambda e: -sign * e.skeleton_change * s if e.skeleton_change < INVALID else INVALID,
                                         "skeleton_change", "D_s", "skel", False),
            "c_pad_mean": Objective(lambda e: -e.c_pad_mean, "c_pad_mean", "C_pad", "most_coverage", True)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=(0, 1))
    ap.add_argument("--population", type=int, default=200)
    ap.add_argument("--generations", type=int, default=200)
    ap.add_argument("--workers", type=int, default=7)
    ap.add_argument("--coverage_floor", type=float, default=0.70)
    args = ap.parse_args(argv)
    config = read_config_file(ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    designs = E.e15.study1.study1_designs("cfl", config["architectures"]["cfl"], 120.0, config)
    h0, goal = designs[3.0], designs[7.0]
    base = mechanism(h0.mechanism)
    reference = paths(base, h0.x, h0.config)
    rep = base.report(base.evaluate(h0.x, h0.config))
    scale = float(rep["finger_length_mm"])
    cfg = E.with_b_z(goal.config, -3, 10, a_z=(-3, 8))
    import csv
    warm = []
    for folder in ("e20_mcp_referenced_free", "e20_mcp_referenced_bz"):
        for r in csv.DictReader((ROOT / "output" / "study" / "exploratory" / folder / "picks.csv").open()):
            if r["label"] != "O":
                warm.append(tuple(float(v) for v in r["x"].split(";")))
    rows = []
    for mode in ("lowM", "highM"):
        mech = Split(base, reference, float(rep["proximal_depth_mm"]), scale, mode)
        for seed in args.seeds:
            t0 = time.time()
            res = run_study(mech, cfg, OptimizerSettings("nsga2", args.population, args.generations, seed),
                            initial_designs=warm, workers=args.workers)
            for c in res.pareto:
                e = c.evaluation
                if e.feasible and e.mcp_change < INVALID and e.skeleton_change < INVALID:
                    rows.append({"mode": mode, "seed": seed, "mcp_change_mm": e.mcp_change * scale,
                                 "contact_change_mm": e.contact_change * scale, "skeleton_change_mm": e.skeleton_change * scale,
                                 "c_pad_mean": e.c_pad_mean, "ma_wc": base.report(e.base).get("ma_wc"),
                                 "face_depth_proximal_mm": base.report(e.base).get("proximal_depth_mm"),
                                 "x": ";".join(f"{v:.6g}" for v in c.x)})
            print(f"{mode} seed {seed}: {len(res.pareto)} front, {time.time() - t0:.0f} s", flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    E.shared.write_csv(OUT / "front.csv", rows)
    ok = [r for r in rows if r["c_pad_mean"] >= args.coverage_floor]
    picks = []
    for lo, hi in ((4, 6), (6, 8), (8, 99)):
        c = [r for r in ok if r["mode"] == "lowM" and r["mcp_change_mm"] <= 1.5 and lo <= r["skeleton_change_mm"] < hi]
        if c:
            picks.append({**max(c, key=lambda r: r["skeleton_change_mm"]), "label": f"lowM_s{lo}"})
    for lo, hi in ((3, 4.5), (4.5, 6), (6, 99)):
        c = [r for r in ok if r["mode"] == "highM" and r["skeleton_change_mm"] <= 3.0 and lo <= r["mcp_change_mm"] < hi]
        if c:
            picks.append({**max(c, key=lambda r: r["mcp_change_mm"]), "label": f"highM_m{lo:g}".replace(".", "p")})
    for p in picks:
        print(f"{p['label']}: D_MCP {p['mcp_change_mm']:.2f} D_s {p['skeleton_change_mm']:.2f} D_c {p['contact_change_mm']:.2f} "
              f"C {p['c_pad_mean']:.3f} face {p['face_depth_proximal_mm']}")
    E.shared.write_csv(OUT / "picks.csv", picks)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
