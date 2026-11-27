#!/usr/bin/env python3
"""Experimental: the coverage ends of Study 2 run with A's and B's heights free ("Pad-Pad, pins free"), as picks for
e20_make_hands.py (docs/experimental/pr_mr.md, "Result: Study 2 with A and B heights free").

    python scripts/study/exploratory/e24_study2_free_picks.py [--flexion 120]

Reads ``output/study/study2_redesign_free_heights/front.csv`` and writes, per thickness, the design of most C_pad
(label ``S2F``) to ``output/study/exploratory/e24_study2_free_h<t>/picks.csv`` with e20's columns.
"""
import argparse
import csv
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
FRONT = ROOT / "output" / "study" / "study2_redesign_free_heights" / "front.csv"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--flexion", type=float, default=120.0)
    args = ap.parse_args(argv)
    rows = [r for r in csv.DictReader(FRONT.open()) if float(r["required_flexion_deg"]) == args.flexion
            and r["architecture"] == "cfl"]
    for t in sorted({float(r["to_mm"]) for r in rows}):
        best = max((r for r in rows if float(r["to_mm"]) == t), key=lambda r: float(r["c_pad_mean"]))
        out = ROOT / "output" / "study" / "exploratory" / f"e24_study2_free_h{t:g}"
        out.mkdir(parents=True, exist_ok=True)
        pick = {"label": "S2F", "seed": best["seed"], "to_mm": t, "required_flexion_deg": args.flexion,
                "mcp_change_mm": best["mcp_change_mm"], "contact_change_mm": best["contact_change_mm"],
                "skeleton_change_mm": float(best["skeleton_change"]) * 89.99478846398088 if best.get("skeleton_change") else "",
                "c_pad_mean": best["c_pad_mean"], "face_depth_proximal_mm": best["face_depth_proximal_mm"],
                "x": best["x"]}
        with (out / "picks.csv").open("w", newline="") as stream:
            w = csv.DictWriter(stream, fieldnames=list(pick))
            w.writeheader()
            w.writerow(pick)
        print(f"{t:g} mm: Dc {float(pick['contact_change_mm']):.2f} DMCP {float(pick['mcp_change_mm']):.2f} "
              f"C {float(pick['c_pad_mean']):.3f} face {pick['face_depth_proximal_mm']} -> {out.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
