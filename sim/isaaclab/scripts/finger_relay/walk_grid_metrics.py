# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Attempt 9 (``docs/experimental/rl_pip_only.md``): the design quantities of every finger-walk hand.

    uv run python scripts/finger_relay/walk_grid_metrics.py [--out logs/finger_walk/grid/hand_metrics.json]

Per hand (R, U, R5, U5, R9, U9 and attempt 9's grid ``g<t>_*``, ``FA*``):
- ``d_contact_mm``, ``d_skel_mm``, ``c_pad``: Study 3's values, from the e18 design tables
  (``design/output/study/exploratory/e18_rct_dose/designs*.csv``: ``contact_change_mm``, ``skeleton_change`` x L_f 90 mm,
  ``c_pad_mean``), the same numbers ``scripts/study3_measurement/dose_analysis.py`` lists. The fixed-axis hands have no
  design row: their Delta values are the recomputed ones below (against O, not against FA3 as in Study 3) and their
  coverage is left out.
- ``d_contact_recomputed_mm``, ``d_skel_recomputed_mm``: recomputed from the Isaac exports. Middle finger, MCP 0,
  PIP 0-120 deg in 0.25 deg steps. Delta_contact = max_q |c_H(q) - c_O(q)| with c the distal pad face's tip-end
  centre (x, z) in the proximal pad-face frame (origin at the foot of the MCP axis on the face). Delta_skel is the same
  for the link-line fingertip (``tips``) in the MCP frame. This reproduces Study 3's values to 0.01 mm on R, U, R5 and
  the grid (``max_abs_diff`` in the output).
- ``face_depth_mm``: the proximal pad face's depth below the link line (mount offset + thickness, from the hand's yaml;
  the fixed-axis family mounts at 3 mm).
- ``thickness_mm``, ``proximal_pad_mm``: pad thickness and the proximal pad's length (exploratory).
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml

from code_isaaclab.assets.description import Description
from code_isaaclab.tasks.finger_walk.grid import GRID
from code_isaaclab.tasks.rotate_cube.grasp import HANDS, _joint_tf

ROOT = Path(__file__).resolve().parents[4]
E18 = ROOT / "design" / "output" / "study" / "exploratory" / "e18_rct_dose"
YAML_DIR = ROOT / "design" / "output" / "sim" / "study3" / "hands"
L_F_MM = 90.0
F = "middle"
PAIRS = {"R": "h3to7_hr", "U": "h3to7_hu", "R5": "h3to5_hr", "U5": "h3to5_hu", "R9": "h3to9_hr", "U9": "h3to9_hu"}


def design_rows() -> dict[str, dict]:
    """Stem (``h3to7_d1p5_minDs_s0``, ``h3to7_hr``) -> e18 row."""
    out = {}
    for name, t in (("designs.csv", 7), ("designs_skeleton.csv", 7), ("designs_expand.csv", 7), ("designs_h5.csv", 5),
                    ("designs_h9.csv", 9)):
        for r in csv.DictReader((E18 / name).open()):
            if r["feasible"] != "True":
                continue
            lab = r["label"]
            if lab in ("H_R", "H_U"):
                stem = f"h3to{t}_{'hr' if lab == 'H_R' else 'hu'}"
            else:
                parts = lab.split()  # "target 1.5", "target 1.5 min-Ds"
                d = "d" + parts[1].replace(".", "p")
                var = {"min-Ds": "_minDs", "max-Ds": "_maxDs"}.get(parts[2], "") if len(parts) > 2 else ""
                stem = f"h3to{t}_{d}{var}_s{r['seed']}"
            out.setdefault(stem, r)
    return out


def paths(path: Path) -> tuple[np.ndarray, np.ndarray]:
    d = Description.load(path)
    tips = json.loads(path.read_text())["tips"]
    fe = np.asarray(d.joint(f"{F}_fe")["anchor"], float)
    pp = next(b for body in d.bodies for b in (body.get("boxes") or []) if b["name"] == f"{F}_proximal_pad")
    face = np.array([fe[0], pp["center"][2] - pp["half_extents"][1]])
    dp = next(b for body in d.bodies for b in (body.get("boxes") or []) if b["name"] == f"{F}_distal_pad")
    c, h = np.asarray(dp["center"], float), dp["half_extents"]
    tip = np.asarray(tips[F], float)
    cs, ts = [], []
    for q in np.arange(0.0, 120.0 + 1e-9, 0.25):
        pose = d.closed_pose(F, float(q))
        m = _joint_tf(d, f"{F}_pip_servo", pose[f"{F}_pip_servo"]) @ _joint_tf(d, f"{F}_b", pose[f"{F}_b"])
        p = (m @ np.array([c[0] + h[0], c[1], c[2] - h[1], 1.0]))[:3]
        cs.append(np.array([p[0], p[2]]) - face)
        p = (m @ np.array([*tip, 1.0]))[:3]
        ts.append(np.array([p[0], p[2]]) - fe[[0, 2]])
    return np.array(cs), np.array(ts)


def pad_info(path: Path) -> tuple[float, float]:
    d = json.loads(path.read_text())
    pp = next(b for body in d["bodies"] for b in (body.get("boxes") or []) if b["name"] == f"{F}_proximal_pad")
    return 2e3 * pp["half_extents"][0], 2e3 * pp["half_extents"][1]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("logs/finger_walk/grid/hand_metrics.json"))
    args = ap.parse_args()
    rows = design_rows()
    c_o, t_o = paths(HANDS["O"])
    hands = {**{k: PAIRS[k] for k in PAIRS}, **{k: (k[1:2] and f"h3to{k[1]}_{k[3:]}") for k in GRID if k.startswith("g")},
             **{k: None for k in GRID if k.startswith("FA")}}
    out, diffs = {}, []
    for key, stem in hands.items():
        c, t = paths(HANDS[key])
        dc, ds = 1e3 * float(np.linalg.norm(c - c_o, axis=1).max()), 1e3 * float(np.linalg.norm(t - t_o, axis=1).max())
        length, thick = pad_info(HANDS[key])
        rec = {"stem": stem, "thickness_mm": round(thick, 3), "proximal_pad_mm": round(length, 2),
               "d_contact_recomputed_mm": round(dc, 3), "d_skel_recomputed_mm": round(ds, 3)}
        if stem is None:  # fixed axis
            rec.update(d_contact_mm=round(dc, 2), d_skel_mm=round(ds, 2), c_pad=None, face_depth_mm=round(3.0 + thick, 2),
                       source="recomputed (fixed axis; no design row)")
        else:
            r = rows[stem]
            pads = yaml.safe_load((YAML_DIR / f"cfl_q120_{stem}.yaml").read_text())["pads"]
            rec.update(d_contact_mm=round(float(r["contact_change_mm"]), 2),
                       d_skel_mm=round(float(r["skeleton_change"]) * L_F_MM, 2), c_pad=round(float(r["c_pad_mean"]), 3),
                       face_depth_mm=round(float(pads["mount_offset_mm"]) + float(pads["thickness_mm"]), 2),
                       face_depth_csv_mm=float(r["face_depth_proximal_mm"]), source="e18 design table")
            diffs.append(max(abs(rec["d_contact_mm"] - dc), abs(rec["d_skel_mm"] - ds)))
        out[key] = rec
        print(f"{key:22s} Dc {rec['d_contact_mm']:6.2f} ({dc:6.2f}) Ds {rec['d_skel_mm']:6.2f} ({ds:6.2f}) "
              f"C {rec['c_pad']} depth {rec['face_depth_mm']} h {thick:.1f} prox {length:.1f}")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({"hands": out, "max_abs_diff_design_vs_recomputed_mm": round(max(diffs), 3)}, indent=1) + "\n")
    print("max |design - recomputed| =", round(max(diffs), 3), "mm over", len(diffs), "CFL hands")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
