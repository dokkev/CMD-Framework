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
# sixth experiment: four hands each at 3 -> 5 and 3 -> 9 mm (separability.md, sixth experiment)
THICK = {"h3to5_hr": (0.35, 3.55, 0.753), "h3to5_hu": (1.58, 2.02, 0.748), "h3to5_d3_s0": (2.92, 6.38, 0.750),
         "h3to5_d3_minDs_s0": (3.09, 0.38, 0.728), "h3to9_hr": (2.32, 9.37, 0.695), "h3to9_hu": (3.89, 7.17, 0.688),
         "h3to9_d3p5_maxDs_s0": (3.52, 12.72, 0.672), "h3to9_d5_minDs_s0": (5.08, 5.32, 0.673)}
BALANCED_7 = ("R", "U", "d3_minDs_s0", "d5p5_s0")


def thickness(hand: str) -> float:
    return float(hand[4]) if hand.startswith("h3to") else 7.0


def bound_mm(hand: str) -> float:  # the fixed-axis lower bound 2 sin(q_req/2) dh at q_req 120 deg
    import math
    return 2.0 * math.sin(math.radians(60.0)) * (thickness(hand) - 3.0)


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


def partial_spearman(y, x, z) -> float:
    """Rank correlation of y and x with z partialled out (Pearson on the residuals of the ranks)."""
    ry, rx, rz = (ranks(np.asarray(v, float)) for v in (y, x, z))
    A = np.c_[rz, np.ones(len(rz))]
    res = lambda v: v - A @ np.linalg.lstsq(A, v, rcond=None)[0]  # noqa: E731
    a, b = res(ry), res(rx)
    den = np.sqrt((a @ a) * (b @ b))
    return float(a @ b / den) if den > 0 else float("nan")


def bootstrap_ci(e, dc, ds, n: int = 10_000, seed: int = 0) -> dict:
    """95% percentile intervals over hands resampled with replacement (hand medians fixed)."""
    rng = np.random.default_rng(seed)
    e, dc, ds = (np.asarray(v, float) for v in (e, dc, ds))
    stats = {"rho_E_Dc": [], "rho_E_Ds": [], "partial_E_Dc_given_Ds": [], "partial_E_Ds_given_Dc": []}
    for _ in range(n):
        k = rng.integers(0, len(e), len(e))
        if len(set(k)) < 4:
            continue
        stats["rho_E_Dc"].append(spearman(dc[k], e[k]))
        stats["rho_E_Ds"].append(spearman(ds[k], e[k]))
        stats["partial_E_Dc_given_Ds"].append(partial_spearman(e[k], dc[k], ds[k]))
        stats["partial_E_Ds_given_Dc"].append(partial_spearman(e[k], ds[k], dc[k]))
    return {k: [float(np.nanpercentile(v, 2.5)), float(np.nanpercentile(v, 97.5))] for k, v in stats.items()}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--condition", default="A", choices=("A", "B", "C", "E", "A-grid"))
    parser.add_argument("--mu", type=float, nargs=2, default=(1.3, 2.0))
    parser.add_argument("--radii", type=float, nargs="+", default=(15.0, 20.0))
    parser.add_argument("--extended", action="store_true", help="also the two hands added after the first results")
    parser.add_argument("--set", default=None, choices=("all", "balanced"),
                        help="all: every hand at 5, 7 and 9 mm; balanced: four per thickness")
    args = parser.parse_args()
    lo, hi = args.mu
    hands = {**HANDS, **EXTENSION} if args.extended else HANDS
    if args.set == "all":
        hands = {**HANDS, **EXTENSION, **THICK}
    elif args.set == "balanced":
        hands = {**{k: HANDS[k] for k in BALANCED_7}, **THICK}
    out = {"condition": args.condition, "mu": [lo, hi], "extended": args.extended, "hands": {}}
    rows, rec_log = [], []
    for hand, (dc, ds, cp) in hands.items():
        rel, absolute, per_dir, force, fails, envs, logr = [], [], {d: [] for d in DIRECTIONS}, [], 0, 0, []
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
                if d_o > 0 and d_h > 0:
                    logr.append(abs(np.log(d_h / d_o)))
                absolute.append(abs(d_h - d_o))
                per_dir[k[1]].append(abs(d_h - d_o) / abs(d_o))
                force.append(es[(hand, lo)]["pre"]["n_distal_total"] / max(es[("O", lo)]["pre"]["n_distal_total"], 1e-9))
        if not rel:
            print(f"{hand}: no valid pairs{'' if complete else ' (runs missing)'}")
            continue
        rec = {"thickness_mm": thickness(hand), "D_c_mm": dc, "D_s_mm": ds, "C_pad": cp, "pairs": len(rel),
               "E_rel_median": float(np.median(rel)),
               "E_log_median": float(np.median(logr)) if logr else None,
               "E_abs_median_n": float(np.median(absolute)),
               "E_rel_by_direction": {d: (float(np.median(v)) if v else None) for d, v in per_dir.items()},
               "distal_force_ratio_median": float(np.median(force)), "gate0_fail_share": fails / max(envs, 1),
               "complete": complete}
        out["hands"][hand] = rec
        rows.append((hand, dc, ds, cp, rec["E_rel_median"]))
        rec_log.append(rec["E_log_median"])
        print(f"{hand:20s} h {thickness(hand):g} D_c {dc:4.2f} D_s {ds:5.2f} C_pad {cp:.3f} | pairs {len(rel):3d} E {100 * rec['E_rel_median']:6.1f}% "
              f"({rec['E_abs_median_n']:.2f} N) | by dir " + " ".join(
                  f"{d}:{'-' if v is None else f'{100 * v:.0f}%'}" for d, v in rec["E_rel_by_direction"].items())
              + f" | N_d/N_d(O) {rec['distal_force_ratio_median']:.2f} | Gate 0 fail {100 * rec['gate0_fail_share']:.0f}%"
              + ("" if complete else " (incomplete)"))
    if len(rows) >= 4:
        names, dc, ds, cp, e = (np.array(v) if i else list(v) for i, v in enumerate(zip(*rows)))
        rho = spearman(dc, e)
        if len(e) <= 8:
            perms = [spearman(dc, np.array(p)) for p in itertools.permutations(e)]
        else:  # random permutations, fixed seed
            rng = np.random.default_rng(0)
            perms = [spearman(dc, rng.permutation(e)) for _ in range(100_000)]
        p = float(np.mean([v >= rho - 1e-12 for v in perms]))
        dh = np.array([thickness(n) - 3.0 for n in names])
        norm = dc / np.array([bound_mm(n) for n in names])
        lg = np.array([np.nan if v is None else v for v in rec_log])
        z = lambda v: (v - v.mean()) / v.std()  # noqa: E731
        coef, *_ = np.linalg.lstsq(np.c_[z(dc), z(ds), np.ones(len(e))], z(e), rcond=None)
        out["tests"] = {"n_hands": len(rows), "rho_E_Dc": rho, "p_one_sided": p, "rho_E_Ds": spearman(ds, e),
                        "rho_E_Cpad": spearman(cp, e), "std_coef_Dc": float(coef[0]), "std_coef_Ds": float(coef[1])}
        if len(set(dh)) > 1:
            c3, *_ = np.linalg.lstsq(np.c_[z(dc), z(ds), z(dh), np.ones(len(e))], z(e), rcond=None)
            out["tests"] |= {"rho_E_Dc_over_bound": spearman(norm, e), "std_coef3_Dc_Ds_dh": [float(v) for v in c3[:3]]}
            print(f"rho(E, D_c / bound) = {spearman(norm, e):+.3f}; standardized fit E ~ {c3[0]:+.2f} D_c {c3[1]:+.2f} D_s "
                  f"{c3[2]:+.2f} dh")
            for t in sorted(set(dh)):
                m = dh == t
                if m.sum() >= 3:
                    print(f"  {t + 3:g} mm: n {m.sum()}, rho(E, D_c) = {spearman(dc[m], e[m]):+.3f}")
        pc, pd = partial_spearman(e, dc, ds), partial_spearman(e, ds, dc)
        ci = bootstrap_ci(e, dc, ds)
        out["tests"] |= {"partial_E_Dc_given_Ds": pc, "partial_E_Ds_given_Dc": pd, "bootstrap_ci95": ci}
        print(f"partial rank correlation: E-D_c | D_s {pc:+.3f} [{ci['partial_E_Dc_given_Ds'][0]:+.2f}, "
              f"{ci['partial_E_Dc_given_Ds'][1]:+.2f}]; E-D_s | D_c {pd:+.3f} [{ci['partial_E_Ds_given_Dc'][0]:+.2f}, "
              f"{ci['partial_E_Ds_given_Dc'][1]:+.2f}]; rho(E, D_c) 95% CI [{ci['rho_E_Dc'][0]:+.2f}, "
              f"{ci['rho_E_Dc'][1]:+.2f}] (hands resampled)")
        ok = ~np.isnan(lg)
        if ok.sum() >= 4:
            out["tests"] |= {"rho_Elog_Dc": spearman(dc[ok], lg[ok]), "rho_Elog_Ds": spearman(ds[ok], lg[ok])}
            print(f"symmetric |ln(D_H/D_O)|: rho with D_c {out['tests']['rho_Elog_Dc']:+.3f}, with D_s "
                  f"{out['tests']['rho_Elog_Ds']:+.3f}")
        print(f"\nSpearman rho(E, D_c) = {rho:+.3f}, exact one-sided permutation p = {p:.4f} (n {len(rows)}); "
              f"rho(E, D_s) = {out['tests']['rho_E_Ds']:+.3f}; rho(E, C_pad) = {out['tests']['rho_E_Cpad']:+.3f}; "
              f"standardized fit E ~ {coef[0]:+.2f} D_c {coef[1]:+.2f} D_s")
    tag = f"_{args.set}" if args.set else ("_extended" if args.extended else "")
    path = ROOT / f"dose_{args.condition}{tag}.json"
    path.write_text(json.dumps(out, indent=1))
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
