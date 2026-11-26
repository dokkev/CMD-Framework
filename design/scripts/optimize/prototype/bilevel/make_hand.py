#!/usr/bin/env python3
"""Bi-level hand design (docs/experimental/bilevel_hand.md): a lower-level finger as CoDe hand parameters.

    python scripts/optimize/prototype/bilevel/make_hand.py output/linkage/config_bilevel_L090.0/quick/portfolio_001/seeds.csv \
        [--role most_coverage] [--row N] [--out output/thumb_layout/bilevel/hands/L090.0.yaml]

The finger's design vector (the seeds.csv columns, in ``decode`` order) becomes the hand model's four-bar (A, A',
|A'B|, |AB'|, B' and C in the distal frame) and the pads by their spans; palm, MCP axes and thumb are the prototype's
(``base:``). The same mapping as the Study 3 hand maker, copied here so the prototype and study tracks stay apart.
Check: the hand model's fingertip C over [0, q_req] against the finger's own sweep, within 1e-6 mm.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[4]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.morphology.code_hand import load  # noqa: E402
from core.morphology.code_hand.model import four_bar  # noqa: E402
from core.morphology.mechanism.linkage.design import decode  # noqa: E402
from core.optimization.analysis.contact_kinematics import paths  # noqa: E402
from core.optimization.linkage.mechanism import resolve_preset  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

VECTOR = ("a_x_mm", "b_x_mm", "b_z_mm", "ap_x_mm", "ap_z_mm", "bp_x_mm", "bp_z_mm", "servo_tilt_deg", "bc_length_mm",
          "a_z_mm", "app_offset_mm", "bpp_offset_mm", "proximal_mount_offset_mm", "distal_mount_offset_mm")
PROTOTYPE = ROOT / "core" / "morphology" / "code_hand" / "prototype.yaml"
PAD_THICKNESS_MM = 4.0  # config.yaml goal.pads.*.height_mm
PAD_WIDTH_MM = 19.0  # the XM335 case width, as the other hand variants
RESOLUTION_MM = 0.1  # config.yaml prototyping_resolution_mm


def hand_dict(x) -> dict:
    finger = decode(x, RESOLUTION_MM)
    j = finger.joint
    a, ap, b, bp = j.a, j.ap, j.b, j.bp
    params = {"a_x_mm": a.x_mm, "a_z_mm": a.z_mm, "ap_x_mm": ap.x_mm, "ap_z_mm": ap.z_mm,
              "apb_mm": math.hypot(b.x_mm - ap.x_mm, b.z_mm - ap.z_mm),
              "abp_mm": math.hypot(bp.x_mm - a.x_mm, bp.z_mm - a.z_mm),
              "bp_x_mm": bp.x_mm - b.x_mm, "bp_z_mm": bp.z_mm - b.z_mm,
              "c_x_mm": finger.distal.end_mm, "c_z_mm": 0.0, "pip_servo_tilt_deg": float(x[7])}
    pads = {"proximal_start_mm": finger.proximal.start_mm, "proximal_end_mm": finger.proximal.end_mm,
            "distal_start_mm": finger.distal.start_mm, "distal_end_mm": finger.distal.end_mm,
            "mount_offset_mm": float(x[12]), "thickness_mm": PAD_THICKNESS_MM, "width_mm": PAD_WIDTH_MM}
    if abs(float(x[12]) - float(x[13])) > 1e-9:
        pads["distal_mount_offset_mm"] = float(x[13])
    return {"finger": {k: round(float(v), 6) for k, v in params.items()},
            "pads": {k: round(float(v), 6) for k, v in pads.items()}}


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("seeds", type=Path)
    p.add_argument("--role", default="most_coverage")
    p.add_argument("--row", type=int, default=None, help="take this row (0-based) instead of the role")
    p.add_argument("--out", type=Path, default=None)
    args = p.parse_args(argv)
    rows = list(csv.DictReader(args.seeds.open()))
    row = rows[args.row] if args.row is not None else next(r for r in rows if args.role in r["seed_role"].split(";"))
    x = np.array([float(row[k]) for k in VECTOR])
    data = hand_dict(x)
    tag = args.seeds.parts[-4] if len(args.seeds.parts) >= 4 else "bilevel"
    out = args.out or ROOT / "output" / "thumb_layout" / "bilevel" / "hands" / f"{tag}.yaml"
    out.parent.mkdir(parents=True, exist_ok=True)
    doc = {"base": os.path.relpath(PROTOTYPE, out.parent), **data,
           "source": {"track": "prototype (bi-level lower level)", "seeds": str(args.seeds), "role": row["seed_role"],
                      "run": row["run"], "candidate": row["candidate"], "c_pad_mean": float(row["c_pad_mean"]),
                      "ma_wc": float(row["ma_wc"]), "finger_length_mm": float(row["finger_length_mm"]),
                      "design": [float(v) for v in x]}}
    out.write_text("# Bi-level lower-level finger as hand parameters: written by "
                   "scripts/optimize/prototype/bilevel/make_hand.py\n" + yaml.safe_dump(doc, sort_keys=False))
    # check: the hand model's fingertip C against the finger's own sweep over [0, q_req]
    config_path = ROOT / "scripts" / "optimize" / "prototype" / "bilevel" / f"{tag}.yaml"
    values = read_config_file(config_path)
    base = mechanism(values.pop("mechanism"))
    resolve_preset(values)
    values.pop("output", None)
    values, _ = base.split_settings(values)
    config = base.config_from_mapping(values)
    ref = paths(base, x, config)
    hand, _ = load(out)
    tips = np.array([four_bar(hand.finger, math.degrees(v)).c for v in np.asarray(ref["q"])])
    worst = float(np.max(np.linalg.norm(tips - np.asarray(ref["tip"]), axis=1)))
    print(out, "C_pad", round(float(row["c_pad_mean"]), 3), "L_f", round(float(row["finger_length_mm"]), 1),
          f"fingertip check max {worst:.2e} mm")
    if worst > 1e-6:
        out.unlink()
        raise SystemExit(f"fingertip mismatch {worst:.3g} mm: not written")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
