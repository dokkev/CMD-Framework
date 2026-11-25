# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3 measurement (problem_formulation A.6.4): which candidate design invariant predicts the interface-measurement
error (Study 3)?

    uv run python scripts/study3_measurement/invariants.py

For every redesign: the change in contact-surface motion (Delta_contact), the skeleton's change (Delta_skel), the
coverage, the pad-thickness change and the transmission change (max over q in [0, 120] deg of |r_H(q)/r_O(q) - 1|,
r = dtheta/dq of the index finger's PIP). Spearman rho of each with the error under the reused flexion command for
pad friction (dose_A_all.json) and pad stiffness (comp.json), with a hand bootstrap 95% interval, and the interval of
rho(E, contact) - |rho(E, candidate)| (thirteenth experiment, 45 hands on the grid: dose_A-grid_expanded.json). Writes
``logs/study3/two_pad/invariants.json``.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dose_analysis as da  # noqa: E402

HAND_DIR = Path(__file__).resolve().parents[4] / "design" / "output" / "sim" / "study3" / "hands"


def hand_file(hand: str) -> Path:
    if hand == "O":
        return HAND_DIR / "cfl_q120_h3to7_h0.json"
    if hand in ("R", "U"):
        return HAND_DIR / f"cfl_q120_h3to7_h{hand.lower()}.json"
    stem = hand if hand.startswith("h3to") else f"h3to7_{hand}"
    return HAND_DIR / f"cfl_q120_{stem}.json"


def transmission_change(hand: str) -> float:
    from code_isaaclab.joint_map import flexion_maps

    mo = flexion_maps(json.loads(hand_file("O").read_text()))["index"]
    mh = flexion_maps(json.loads(hand_file(hand).read_text()))["index"]
    qs = np.radians(np.arange(1.0, 120.0, 1.0))
    return float(max(abs(float(mh.ratio(q)) / float(mo.ratio(q)) - 1.0) for q in qs))


def boot(x, e, n=10_000, seed=0):
    rng = np.random.default_rng(seed)
    v = []
    for _ in range(n):
        k = rng.integers(0, len(e), len(e))
        if len(set(k)) >= 4:
            v.append(da.spearman(x[k], e[k]))
    return [float(np.nanpercentile(v, 2.5)), float(np.nanpercentile(v, 97.5))]


def boot_diff(feats_x: dict, e, n=10_000, seed=0):
    """Interval of rho(E, contact) - |rho(E, candidate)| with hands resampled jointly for all candidates."""
    rng = np.random.default_rng(seed)
    v = {f: [] for f in feats_x if f != "contact"}
    for _ in range(n):
        k = rng.integers(0, len(e), len(e))
        if len(set(k)) < 4:
            continue
        rc = da.spearman(feats_x["contact"][k], e[k])
        for f in v:
            v[f].append(rc - abs(da.spearman(feats_x[f][k], e[k])))
    return {f: [float(np.nanpercentile(x, 2.5)), float(np.nanpercentile(x, 97.5)),
                float(np.nanmean(np.asarray(x) <= 0))] for f, x in v.items()}


def main() -> int:
    hands = {**da.HANDS, **da.EXTENSION, **da.THICK, **da.EXPAND}
    feats = {}
    for h, (dc, ds, cp) in hands.items():
        feats[h] = {"contact": dc, "skeleton": ds, "coverage": cp, "thickness": da.thickness(h) - 3.0,
                    "transmission": transmission_change(h)}
        print(f"{h:22s} " + " ".join(f"{k} {v:.3f}" for k, v in feats[h].items()))
    out = {"features": feats}
    fr = json.loads((da.ROOT / "dose_A_all.json").read_text())["hands"]
    cm = json.loads((da.ROOT / "comp.json").read_text())["hands"]
    tq = json.loads((da.ROOT / "dose_C_all.json").read_text())["hands"]
    gr = json.loads((da.ROOT / "dose_A-grid_all.json").read_text())["hands"]
    gx = json.loads((da.ROOT / "dose_A-grid_expanded.json").read_text())["hands"]
    for name, recs, key in (("friction", fr, "E_rel_median"), ("stiffness", cm, "E_median"),
                            ("friction_torque", tq, "E_rel_median"), ("friction_grid", gr, "E_rel_median"),
                            ("friction_grid_expanded", gx, "E_rel_median"),
                            # exploratory, after the thirteenth experiment's results: hands that keep two-pad contact
                            # at >= 80% of the grid's placements (the threshold's effect: report/study3/study3.md, 13th)
                            ("friction_grid_expanded_held", {h: r for h, r in gx.items() if r["gate0_fail_share"] < 0.2},
                             "E_rel_median")):
        hs = [h for h in recs if h in feats]
        e = np.array([recs[h][key] for h in hs])
        res = {}
        for f in ("contact", "skeleton", "coverage", "thickness", "transmission"):
            x = np.array([feats[h][f] for h in hs])
            res[f] = {"rho": da.spearman(x, e), "ci95": boot(x, e)}
        xs = {f: np.array([feats[h][f] for h in hs]) for f in res}
        out[name] = {"n": len(hs), **res, "contact_minus_abs_other_ci95_pnonpos": boot_diff(xs, e)}
        print(f"\n{name} (n {len(hs)}): " + "; ".join(
            f"{f} {r['rho']:+.2f} [{r['ci95'][0]:+.2f}, {r['ci95'][1]:+.2f}]" for f, r in res.items()))
        print("  contact - |other|: " + "; ".join(f"{f} [{c[0]:+.2f}, {c[1]:+.2f}] p {c[2]:.3f}"
                                                  for f, c in out[name]["contact_minus_abs_other_ci95_pnonpos"].items()))
    (da.ROOT / "invariants.json").write_text(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
