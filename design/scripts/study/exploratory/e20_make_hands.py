#!/usr/bin/env python3
"""Experimental: the e20 MR picks as CoDe hand parameter files (``output/sim/study3/hands/cfl_q120_h3to7_<tag>.yaml``),
by Study 3's mapping (study3_trajectory/make_hands.py), checked against the design's own tip and contact paths.

    python scripts/study/exploratory/e20_make_hands.py
"""
import csv
import math
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
for p in (ROOT, ROOT / "scripts" / "study", ROOT / "scripts" / "study" / "study3_trajectory", HERE):
    sys.path.insert(0, str(p))
import e20_mcp_referenced as E  # noqa: E402
import make_hands as M  # noqa: E402
from core.morphology.code_hand.parameters import PROTOTYPE  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

RUNS = [("bz", "e20_mcp_referenced_bz", 7.0), ("free", "e20_mcp_referenced_free", 7.0),
        ("free", "e20_mcp_referenced_free_h5", 5.0), ("free", "e20_mcp_referenced_free_h9", 9.0),
        ("e21", "e21_mcp_vs_skeleton", 7.0), ("e22", "e22_all_three_h7", 7.0)]
LABELS = {"MR": "mr", "MR-mid": "mrmid", "lowM_s4": "lowm_s4", "highM_m3": "highm_m3", "highM_m4p5": "highm_m4p5", "MRS": "mrs"}


def main() -> int:
    config = read_config_file(ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    designs = E.e15.study1.study1_designs("cfl", config["architectures"]["cfl"], 120.0, config)
    base = mechanism(designs[3.0].mechanism)
    ok = True
    for run, folder, t in RUNS:
        cfg = E.with_b_z(designs[t].config, -3, 10, a_z=(-3, 8))
        for r in csv.DictReader((ROOT / "output" / "study" / "exploratory" / folder / "picks.csv").open()):
            if r["label"] not in LABELS:
                continue
            x = tuple(float(v) for v in r["x"].split(";"))
            name = f"{LABELS[r['label']]}_{run}"
            finger, pads = M.hand_parameters(x, cfg, t)
            tip_err, contact_err = M.check(base, x, cfg, finger, pads, math.radians(120.0))
            print(f"{name}: tip {tip_err:.2e} mm, contact {contact_err:.2e} mm; a_z {finger['a_z_mm']:.2f} "
                  f"mounts {pads['mount_offset_mm']}/{pads.get('distal_mount_offset_mm', pads['mount_offset_mm'])}")
            if max(tip_err, contact_err) > 1e-6:
                ok = False
                continue
            doc = {"base": str(Path("../../../../core/morphology/code_hand") / PROTOTYPE.name), "finger": finger, "pads": pads,
                   "source": {"study": "PR/MR (docs/experimental/pr_mr.md), e20 " + run, "label": r["label"],
                              "design": [round(v, 6) for v in x], "required_flexion_deg": 120.0, "pad_thickness_mm": t,
                              "mcp_change_mm": float(r["mcp_change_mm"]), "contact_change_mm": float(r["contact_change_mm"]),
                              "skeleton_change_mm": float(r.get("skeleton_change_mm") or "nan")}}
            path = M.OUT / f"cfl_q120_h3to{t:g}_{name}.yaml"
            path.write_text(yaml.safe_dump(doc, sort_keys=False))
            print(f"  -> {path.relative_to(ROOT)}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
