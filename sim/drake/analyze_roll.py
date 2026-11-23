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
RADII, MU = (15.0, 20.0), (1.3, 2.0)
TASKS = {  # task: (file stem, variant tag prefix, variants, result key, metric, minimum |D(O)|, O must hold)
    "roll": ("dkroll", "s", (8, 15), "roll", "rotation_deg", 0.5, False),
    "pullin": ("dkpullin", "a", (3, 6), "pullin", "drawn_mm", 0.3, True),
    "dial": ("dkdial", "s", (8, 15), "dial", "rotation_deg", 0.3, True),
}
TASK = "roll"


def load(hand, mu, contact, tag, s, r):
    stem, pre = TASKS[TASK][0], TASKS[TASK][1]
    p = HERE / "logs" / f"{stem}_{hand}_mu{mu:g}_{contact}-{tag}-{pre}{s}_r{r:g}.json"
    return {(e["q0_deg"], round(e["x_c_mm"], 3)): e for e in json.loads(p.read_text())["envs"]} if p.exists() else None


def error(hand, contact, tag):
    _, _, variants, rk, metric, min_effect, o_holds = TASKS[TASK]
    rel, lost, n = [], 0, 0
    for s in variants:
        for r in RADII:
            runs = {(h, m): load(h, m, contact, tag, s, r) for h in ("O", hand) for m in MU}
            if any(v is None for v in runs.values()):
                return None
            for k in runs[("O", MU[0])]:
                es = {hm: runs[hm].get(k) for hm in runs}
                if any(e is None or not e["gate0"]["pass"] or e.get(rk) is None for e in es.values()):
                    continue
                if o_holds and not all(es[("O", m)][rk]["held"] for m in MU):
                    continue
                d_o = es[("O", MU[1])][rk][metric] - es[("O", MU[0])][rk][metric]
                if abs(d_o) < min_effect:
                    continue
                n += 1
                if not all(es[(hand, m)][rk]["held"] for m in MU):
                    lost += 1
                    continue
                d_h = es[(hand, MU[1])][rk][metric] - es[(hand, MU[0])][rk][metric]
                rel.append(abs(d_h - d_o) / abs(d_o))
    return {"E": float(np.median(rel)) if rel else None, "pairs": len(rel), "lost": lost, "candidates": n}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--task", choices=tuple(TASKS), default="roll")
    p.add_argument("--contact", default="point")
    p.add_argument("--tag", default="e22")
    p.add_argument("--out", type=Path, default=None)
    args = p.parse_args()
    global TASK
    TASK = args.task
    rows = []
    for hand, (dc, ds) in DESIGN.items():
        r = error(hand, args.contact, args.tag)
        if r is None or r["E"] is None:
            continue
        rows.append({"hand": hand, "d_contact": dc, "d_skel": ds, **r})
        print(f"{hand:22s} D_c {dc:4.2f} D_s {ds:5.2f} | {args.task} ({args.contact}): E {100 * r['E']:6.1f}% "
              f"({r['pairs']} pairs, {r['lost']} lost of {r['candidates']})", flush=True)
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
