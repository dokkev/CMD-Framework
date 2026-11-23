"""Experiment 22 (in-hand rolling) analysis, as pre-registered in docs/experimental/separability.md: per hand H, over
placements and both sweeps where O and H pass Gate 0 at both mu and |D(O)| >= 0.5 deg, D = rotation(2.0) -
rotation(1.3), E(H) = median |D(H) - D(O)| / |D(O)|; grasps lost by the end are counted apart. Spearman rho(E,
D_contact) and rho(E, D_skel) with exact one-sided permutation p over the redesigned hands present.

    uv run python analyze_roll.py --contact point --tag e22 [--out file.json]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from analyze import DESIGN, spearman_perm

HERE = Path(__file__).resolve().parent
SWEEPS, RADII, MU = (8, 15), (15.0, 20.0), (1.3, 2.0)
MIN_EFFECT_DEG = 0.5


def load(hand, mu, contact, tag, s, r):
    p = HERE / "logs" / f"dkroll_{hand}_mu{mu:g}_{contact}-{tag}-s{s}_r{r:g}.json"
    return {(e["q0_deg"], round(e["x_c_mm"], 3)): e for e in json.loads(p.read_text())["envs"]} if p.exists() else None


def error(hand, contact, tag):
    rel, lost, n = [], 0, 0
    for s in SWEEPS:
        for r in RADII:
            runs = {(h, m): load(h, m, contact, tag, s, r) for h in ("O", hand) for m in MU}
            if any(v is None for v in runs.values()):
                return None
            for k in runs[("O", MU[0])]:
                es = {hm: runs[hm].get(k) for hm in runs}
                if any(e is None or not e["gate0"]["pass"] or e.get("roll") is None for e in es.values()):
                    continue
                d_o = es[("O", MU[1])]["roll"]["rotation_deg"] - es[("O", MU[0])]["roll"]["rotation_deg"]
                if abs(d_o) < MIN_EFFECT_DEG:
                    continue
                n += 1
                if not all(es[(hand, m)]["roll"]["held"] for m in MU):
                    lost += 1
                    continue
                d_h = es[(hand, MU[1])]["roll"]["rotation_deg"] - es[(hand, MU[0])]["roll"]["rotation_deg"]
                rel.append(abs(d_h - d_o) / abs(d_o))
    return {"E": float(np.median(rel)) if rel else None, "pairs": len(rel), "lost": lost, "candidates": n}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--contact", default="point")
    p.add_argument("--tag", default="e22")
    p.add_argument("--out", type=Path, default=None)
    args = p.parse_args()
    rows = []
    for hand, (dc, ds) in DESIGN.items():
        r = error(hand, args.contact, args.tag)
        if r is None or r["E"] is None:
            continue
        rows.append({"hand": hand, "d_contact": dc, "d_skel": ds, **r})
        print(f"{hand:22s} D_c {dc:4.2f} D_s {ds:5.2f} | rolling ({args.contact}): E {100 * r['E']:6.1f}% "
              f"({r['pairs']} pairs, {r['lost']} lost of {r['candidates']})")
    if len(rows) >= 3:
        rho, pv = spearman_perm([x["d_contact"] for x in rows], [x["E"] for x in rows])
        rho_s, pv_s = spearman_perm([x["d_skel"] for x in rows], [x["E"] for x in rows])
        print(f"rho(E, D_contact) {rho:.2f} (one-sided p {pv:.4f}); rho(E, D_skel) {rho_s:.2f} (p {pv_s:.4f}); "
              f"hands {len(rows)}")
    if args.out:
        args.out.write_text(json.dumps(rows, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
