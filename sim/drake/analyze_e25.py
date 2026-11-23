"""Experiment 25: hydroelastic pads as soft as they are thick, against the counterfactual reference O_h (O's geometry
with an h mm pad's stiffness). E'(H) = median |D(H) - D(O_h)| / |D(O_h)| over the placement-direction pairs where
both pass Gate 0 at both mu and have a limit; rho(E', D_contact), rho(E', D_skel); R against U per thickness.

    uv run python analyze_e25.py [--out logs/e25.json]
"""
import argparse
import json
from pathlib import Path

import numpy as np

from analyze import DESIGN, MU, RADII, key, spearman_perm

HERE = Path(__file__).resolve().parent


def thickness(hand: str) -> int:
    return int(hand[4]) if hand.startswith("h3to") else 7


def load(hand, mu, tag, r):
    p = HERE / "logs" / f"dk_{hand}_mu{mu:g}_hydro-{tag}_r{r:g}.json"
    return json.loads(p.read_text())["envs"] if p.exists() else None


def error(hand: str):
    h = thickness(hand)
    rel = []
    for r in RADII:
        runs = {("O", m): load("O", m, f"e25-ref{h}", r) for m in MU}
        runs.update({(hand, m): load(hand, m, "e25", r) for m in MU})
        if any(v is None for v in runs.values()):
            return None, 0
        by = {hm: {key(e): e for e in v} for hm, v in runs.items()}
        for k in by[("O", MU[0])]:
            es = [by[hm].get(k) for hm in by]
            if any(e is None or not e["gate0"]["pass"] or e["limit"].get("f_limit_n") is None for e in es):
                continue
            d_o = by[("O", MU[1])][k]["limit"]["f_limit_n"] - by[("O", MU[0])][k]["limit"]["f_limit_n"]
            d_h = by[(hand, MU[1])][k]["limit"]["f_limit_n"] - by[(hand, MU[0])][k]["limit"]["f_limit_n"]
            if abs(d_o) < 1e-9:
                continue
            rel.append(abs(d_h - d_o) / abs(d_o))
    return (float(np.median(rel)) if rel else None), len(rel)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--out", type=Path, default=None)
    args = p.parse_args()
    rows = []
    for hand, (dc, ds) in DESIGN.items():
        e, n = error(hand)
        if e is None:
            continue
        rows.append({"hand": hand, "d_contact": dc, "d_skel": ds, "E": e, "pairs": n, "thickness": thickness(hand)})
        print(f"{hand:22s} h {thickness(hand)} D_c {dc:4.2f} D_s {ds:5.2f} | E' {100 * e:6.1f}% ({n} pairs)")
    if len(rows) >= 3:
        rho, pv = spearman_perm([r["d_contact"] for r in rows], [r["E"] for r in rows])
        rs, ps = spearman_perm([r["d_skel"] for r in rows], [r["E"] for r in rows])
        print(f"rho(E', D_contact) {rho:.2f} (one-sided p {pv:.4f}); rho(E', D_skel) {rs:.2f} (p {ps:.4f}); "
              f"hands {len(rows)}")
    if args.out:
        args.out.write_text(json.dumps(rows, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
