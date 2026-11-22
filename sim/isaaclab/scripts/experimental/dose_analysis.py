# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Experimental: the pre-registered dose-response measures (``docs/experimental/separability.md``, fourth experiment).

    python3 scripts/experimental/dose_analysis.py [--condition A] [--mu 1.3 2.0]

For each redesigned hand H (latest ``two_pad_wrench.py`` run per hand, mu, condition, radius, impratio 1000), over the
placement-direction pairs where O and H both pass Gate 0 at both mu: the interface effect D(H, d) = F_limit(mu_hi) -
F_limit(mu_lo), the relative error |D(H, d) - D(O, d)| / |D(O, d)| and its median E(H) (pooled over directions and
radii; per direction too). Then over the redesigned hands: Spearman rho(E, D_c) with an exact one-sided permutation
p, rho(E, D_s), rho(E, C_pad), and the standardized least-squares fit E ~ D_c + D_s (descriptive). Also each hand's
Gate 0 failure share and its median distal force against O's (the same-command loading). Writes
``logs/experimental/two_pad/dose_<condition>.json``.
"""

from __future__ import annotations

import argparse
import glob
import itertools
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2] / "logs" / "experimental" / "two_pad"
DIRECTIONS = ("+x", "+x-z", "-z", "-x")
# design metrics of the pre-registered hands (e18; separability.md, fourth experiment, reselection)
HANDS = {"R": (0.86, 7.67, 0.725), "d1p5_minDs_s0": (1.60, 5.37, 0.706), "d2p25_minDs_s0": (2.33, 4.62, 0.700),
         "d2p25_maxDs_s0": (2.30, 9.17, 0.712), "d3_minDs_s0": (3.09, 3.83, 0.713), "U": (3.91, 3.38, 0.711),
         "d4p5_s0": (4.41, 11.33, 0.716), "d5p5_s0": (5.40, 12.17, 0.703)}
# added after the first results once the hand model took two mount depths (separability.md, fourth experiment, extension)
EXTENSION = {"d1p5_maxDs_s0": (1.46, 9.67, 0.707), "d3_maxDs_s0": (3.04, 11.55, 0.700)}


def latest(hand: str, mu: float, cond: str, radius: float) -> dict | None:
    found = None
    for f in sorted(glob.glob(str(ROOT / f"tp_{hand}_mu{mu:g}_{cond}_r{radius:g}_*.json"))):
        d = json.loads(Path(f).read_text())
        if d.get("impratio") == 1000.0:
            found = d
    return found


def ranks(x: np.ndarray) -> np.ndarray:
    order = np.argsort(x)
    r = np.empty(len(x))
    r[order] = np.arange(len(x))
    for v in np.unique(x):  # average ties
        m = x == v
        r[m] = r[m].mean()
    return r


def spearman(x, y) -> float:
    return float(np.corrcoef(ranks(np.asarray(x, float)), ranks(np.asarray(y, float)))[0, 1])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--condition", default="A", choices=("A", "B", "C"))
    parser.add_argument("--mu", type=float, nargs=2, default=(1.3, 2.0))
    parser.add_argument("--radii", type=float, nargs="+", default=(15.0, 20.0))
    parser.add_argument("--extended", action="store_true", help="also the two hands added after the first results")
    args = parser.parse_args()
    lo, hi = args.mu
    hands = {**HANDS, **EXTENSION} if args.extended else HANDS
    out = {"condition": args.condition, "mu": [lo, hi], "extended": args.extended, "hands": {}}
    rows = []
    for hand, (dc, ds, cp) in hands.items():
        rel, absolute, per_dir, force, fails, envs = [], [], {d: [] for d in DIRECTIONS}, [], 0, 0
        complete = True
        for r in args.radii:
            runs = {(h, m): latest(h, m, args.condition, r) for h in ("O", hand) for m in (lo, hi)}
            if any(v is None for v in runs.values()):
                complete = False
                continue
            key = lambda e: (e["q0_deg"], e["direction"])  # noqa: E731
            by = {hm: {key(e): e for e in d["envs"]} for hm, d in runs.items()}
            for m in (lo, hi):
                envs += len(by[(hand, m)])
                fails += sum(not e["gate0"]["pass"] for e in by[(hand, m)].values())
            for k, e_o_lo in by[("O", lo)].items():
                es = {hm: by[hm].get(k) for hm in by}
                if any(e is None or not e["gate0"]["pass"] or e["limit"]["f_limit_n"] is None for e in es.values()):
                    continue
                d_o = es[("O", hi)]["limit"]["f_limit_n"] - es[("O", lo)]["limit"]["f_limit_n"]
                d_h = es[(hand, hi)]["limit"]["f_limit_n"] - es[(hand, lo)]["limit"]["f_limit_n"]
                if abs(d_o) < 1e-9:
                    continue
                rel.append(abs(d_h - d_o) / abs(d_o))
                absolute.append(abs(d_h - d_o))
                per_dir[k[1]].append(abs(d_h - d_o) / abs(d_o))
                force.append(es[(hand, lo)]["pre"]["n_distal_total"] / max(es[("O", lo)]["pre"]["n_distal_total"], 1e-9))
        if not rel:
            print(f"{hand}: no valid pairs{'' if complete else ' (runs missing)'}")
            continue
        rec = {"D_c_mm": dc, "D_s_mm": ds, "C_pad": cp, "pairs": len(rel), "E_rel_median": float(np.median(rel)),
               "E_abs_median_n": float(np.median(absolute)),
               "E_rel_by_direction": {d: (float(np.median(v)) if v else None) for d, v in per_dir.items()},
               "distal_force_ratio_median": float(np.median(force)), "gate0_fail_share": fails / max(envs, 1),
               "complete": complete}
        out["hands"][hand] = rec
        rows.append((hand, dc, ds, cp, rec["E_rel_median"]))
        print(f"{hand:15s} D_c {dc:4.2f} D_s {ds:5.2f} C_pad {cp:.3f} | pairs {len(rel):3d} E {100 * rec['E_rel_median']:6.1f}% "
              f"({rec['E_abs_median_n']:.2f} N) | by dir " + " ".join(
                  f"{d}:{'-' if v is None else f'{100 * v:.0f}%'}" for d, v in rec["E_rel_by_direction"].items())
              + f" | N_d/N_d(O) {rec['distal_force_ratio_median']:.2f} | Gate 0 fail {100 * rec['gate0_fail_share']:.0f}%"
              + ("" if complete else " (incomplete)"))
    if len(rows) >= 4:
        names, dc, ds, cp, e = (np.array(v) if i else list(v) for i, v in enumerate(zip(*rows)))
        rho = spearman(dc, e)
        perms = [spearman(dc, np.array(p)) for p in itertools.permutations(e)]
        p = float(np.mean([v >= rho - 1e-12 for v in perms]))
        z = lambda v: (v - v.mean()) / v.std()  # noqa: E731
        coef, *_ = np.linalg.lstsq(np.c_[z(dc), z(ds), np.ones(len(e))], z(e), rcond=None)
        out["tests"] = {"n_hands": len(rows), "rho_E_Dc": rho, "p_one_sided": p, "rho_E_Ds": spearman(ds, e),
                        "rho_E_Cpad": spearman(cp, e), "std_coef_Dc": float(coef[0]), "std_coef_Ds": float(coef[1])}
        print(f"\nSpearman rho(E, D_c) = {rho:+.3f}, exact one-sided permutation p = {p:.4f} (n {len(rows)}); "
              f"rho(E, D_s) = {out['tests']['rho_E_Ds']:+.3f}; rho(E, C_pad) = {out['tests']['rho_E_Cpad']:+.3f}; "
              f"standardized fit E ~ {coef[0]:+.2f} D_c {coef[1]:+.2f} D_s")
    path = ROOT / f"dose_{args.condition}{'_extended' if args.extended else ''}.json"
    path.write_text(json.dumps(out, indent=1))
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
