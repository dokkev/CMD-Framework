# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Attempt 9 (``docs/experimental/rl_pip_only.md``, pre-registered): which design quantity predicts how well the two
O finger-walk policies (training seeds 20, 21) transfer to the Study 3 hand grid.

    uv run python scripts/finger_relay/walk_grid_analysis.py [--runs logs/finger_walk/grid] [--out <json>] [--figure <png>]

Inputs: ``<runs>/<hand>_mu<mu>_seed<eval seed>.json`` (``eval_walk.py`` with both checkpoints: a list, seed 20 first)
and ``<runs>/hand_metrics.json`` (``walk_grid_metrics.py``).

Per hand and training seed (two eval seeds averaged):
- ratio = walked(H, mu 1.3) / walked(O, mu 1.3); drop = dropped share at mu 1.3;
- E = |D(H) - D(O)| / |D(O)|, D = walked(mu 2.0) - walked(mu 1.3);
- eval noise: ratio |w(94000) - w(95000)| / w(O), E |D(94000) - D(95000)| / |D(O)|.
Pooled = the two training seeds' mean.

Over the CFL hands (the grid and R, U, R5, U5, R9, U9; the fixed-axis family is reported apart):
- Spearman rho of ratio, E and drop with Delta_contact, Delta_skel, C_pad and the proximal face depth (exploratory:
  thickness, proximal pad length), two-sided permutation p (20,000, seed 0), hand-bootstrap 95 % CI (10,000, seed 0);
- partial rho(Y, Delta_contact | Delta_skel) (and | thickness): Pearson of the rank residuals, permutation of Y's
  residual, bootstrap CI; the bootstrap CI of |rho(Y, Dc)| - |rho(Y, Ds)|;
- the min/max-Delta_skel pairs at one Delta_contact target (|Delta Ds| >= 2 mm): Y(maxDs) - Y(minDs), median, sign
  count, Wilcoxon signed-rank two-sided p;
- the tolerance: a decreasing isotonic fit of the ratio on Delta_contact, and Dc50, the midpoint between the last hand
  whose fit is >= 0.5 and the first whose fit is < 0.5 (hand bootstrap CI).
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy import stats

SEEDS = (20, 21)
EVAL = (94000, 95000)
MU = ("1.3", "2.0")
PAIR_KEYS = ("R", "U", "R5", "U5", "R9", "U9")
PRED = ("d_contact_mm", "d_skel_mm", "c_pad", "face_depth_mm")
EXPLORE = ("thickness_mm", "proximal_pad_mm")
N_PERM, N_BOOT = 20000, 10000


def load(runs: Path, hand: str) -> dict | None:
    out = {}
    for mu in MU:
        for s in EVAL:
            p = runs / f"{hand}_mu{mu}_seed{s}.json"
            if not p.exists():
                return None
            res = json.loads(p.read_text())
            res = res if isinstance(res, list) else [res]
            for k, r in zip(SEEDS, res):
                assert f"_s{k}_" in r["checkpoint"], (p, r["checkpoint"])
                out[(k, mu, s)] = r
    return out


def outcomes(raw: dict, o: dict) -> dict:
    rec = {}
    for k in SEEDS:
        w = {(mu, s): raw[(k, mu, s)]["walked_mm_mean"] for mu in MU for s in EVAL}
        wo = {(mu, s): o[(k, mu, s)]["walked_mm_mean"] for mu in MU for s in EVAL}
        w13, wo13 = np.mean([w[("1.3", s)] for s in EVAL]), np.mean([wo[("1.3", s)] for s in EVAL])
        d = {s: w[("2.0", s)] - w[("1.3", s)] for s in EVAL}
        do = np.mean([wo[("2.0", s)] - wo[("1.3", s)] for s in EVAL])
        dm = np.mean(list(d.values()))
        rec[k] = {"walked_mu1.3": w13, "walked_mu2.0": float(np.mean([w[("2.0", s)] for s in EVAL])),
                  "ratio": w13 / wo13, "E": abs(dm - do) / abs(do), "D": dm, "D_O": do,
                  "drop": float(np.mean([raw[(k, "1.3", s)]["dropped_rate"] for s in EVAL])),
                  "goals": float(np.mean([raw[(k, "1.3", s)]["goals_per_episode"] for s in EVAL])),
                  "ratio_noise": abs(w[("1.3", 94000)] - w[("1.3", 95000)]) / wo13,
                  "E_noise": abs(d[94000] - d[95000]) / abs(do)}
    rec["pooled"] = {m: float(np.mean([rec[k][m] for k in SEEDS])) for m in rec[SEEDS[0]]}
    return rec


def _rank(x):
    return stats.rankdata(x)


def spearman(x, y):
    return float(stats.spearmanr(x, y).statistic)


def _corr_rows(a, b):
    """Pearson correlation of each row of a with each row of b (same shape)."""
    a = a - a.mean(axis=-1, keepdims=True)
    b = b - b.mean(axis=-1, keepdims=True)
    return (a * b).sum(-1) / np.sqrt((a * a).sum(-1) * (b * b).sum(-1))


def perm_p(x, y, rng):
    """Two-sided permutation p of Spearman rho (y's ranks permuted)."""
    rx, ry = _rank(x), _rank(y)
    r0 = float(np.corrcoef(rx, ry)[0, 1])
    perms = np.stack([rng.permutation(ry) for _ in range(N_PERM)])
    r = _corr_rows(perms, np.broadcast_to(rx, perms.shape))
    return float((np.sum(np.abs(r) >= abs(r0) - 1e-12) + 1) / (N_PERM + 1))


def boot_spearman(x, y, rng):
    idx = rng.integers(0, len(x), (N_BOOT, len(x)))
    r = _corr_rows(stats.rankdata(x[idx], axis=1), stats.rankdata(y[idx], axis=1))
    r = r[np.isfinite(r)]
    return [float(np.percentile(r, 2.5)), float(np.percentile(r, 97.5))]


def partial(y, x, z):
    """Spearman partial rho(y, x | z): Pearson of the rank residuals of y and x on z."""
    ry, rx, rz = _rank(y), _rank(x), _rank(z)
    A = np.c_[np.ones_like(rz), rz]
    ey = ry - A @ np.linalg.lstsq(A, ry, rcond=None)[0]
    ex = rx - A @ np.linalg.lstsq(A, rx, rcond=None)[0]
    return float(np.corrcoef(ey, ex)[0, 1]), ey, ex


def boot(n, fn, rng):
    vals = []
    for _ in range(N_BOOT):
        i = rng.integers(0, n, n)
        v = fn(i)
        if np.isfinite(v):
            vals.append(v)
    return [float(np.percentile(vals, 2.5)), float(np.percentile(vals, 97.5))]


def isotonic_decreasing(x, y):
    order = np.argsort(x, kind="stable")
    xs, ys = np.asarray(x)[order], -np.asarray(y, float)[order]
    vals = []
    for v in ys:  # pool adjacent violators on -y (increasing)
        vals.append([v, 1.0])
        while len(vals) > 1 and vals[-2][0] > vals[-1][0]:
            a, b = vals.pop(), vals.pop()
            vals.append([(a[0] * a[1] + b[0] * b[1]) / (a[1] + b[1]), a[1] + b[1]])
    fit = np.concatenate([[v] * int(w) for v, w in vals])
    return xs, -fit


def dc50(x, y):
    xs, fit = isotonic_decreasing(x, y)
    below = np.nonzero(fit < 0.5)[0]
    if len(below) == 0:
        return math.inf
    i = below[0]
    return float(xs[i]) if i == 0 else float(0.5 * (xs[i - 1] + xs[i]))


def correlations(hands, metrics, out, ys=("ratio", "E", "drop")):
    res = {}
    for scope in ("pooled",) + SEEDS:
        for yname in ys:
            y = np.array([out[h][scope][yname] for h in hands])
            for xname in PRED + EXPLORE:
                x = np.array([metrics[h][xname] for h in hands], float)
                if np.ptp(x) == 0:
                    continue
                rng = np.random.default_rng(0)
                rho = spearman(x, y)
                res[f"{scope}|{yname}|{xname}"] = {
                    "rho": rho, "p": perm_p(x, y, rng), "n": len(hands),
                    "ci": boot_spearman(x, y, np.random.default_rng(0))}
            dc = np.array([metrics[h]["d_contact_mm"] for h in hands])
            for zname in ("d_skel_mm", "thickness_mm", "c_pad"):
                z = np.array([metrics[h][zname] for h in hands], float)
                if np.ptp(z) == 0:
                    continue
                r, ey, ex = partial(y, dc, z)
                rng = np.random.default_rng(0)
                perms = np.stack([rng.permutation(ey) for _ in range(N_PERM)])
                hits = int(np.sum(np.abs(_corr_rows(perms, np.broadcast_to(ex, perms.shape))) >= abs(r) - 1e-12))
                res[f"{scope}|{yname}|partial_dc_given_{zname}"] = {
                    "rho": r, "p": (hits + 1) / (N_PERM + 1), "n": len(hands),
                    "ci": boot(len(hands), lambda i: partial(y[i], dc[i], z[i])[0], np.random.default_rng(0))}
            ds = np.array([metrics[h]["d_skel_mm"] for h in hands])
            idx = np.random.default_rng(0).integers(0, len(hands), (N_BOOT, len(hands)))
            ry = stats.rankdata(y[idx], axis=1)
            diff = (np.abs(_corr_rows(stats.rankdata(dc[idx], axis=1), ry))
                    - np.abs(_corr_rows(stats.rankdata(ds[idx], axis=1), ry)))
            diff = diff[np.isfinite(diff)]
            res[f"{scope}|{yname}|abs_dc_minus_abs_ds"] = {
                "value": abs(spearman(dc, y)) - abs(spearman(ds, y)),
                "ci": [float(np.percentile(diff, 2.5)), float(np.percentile(diff, 97.5))]}
    return res


def ds_pairs(metrics, out):
    rows = []
    for h in metrics:
        if not (h.startswith("g") and "_minDs_" in h):
            continue
        hm = h.replace("_minDs_", "_maxDs_")
        if hm not in out or h not in out:
            continue
        dds = metrics[hm]["d_skel_mm"] - metrics[h]["d_skel_mm"]
        if dds < 2.0:
            continue
        rows.append({"pair": h.replace("_minDs", ""), "d_contact_min_max": [metrics[h]["d_contact_mm"], metrics[hm]["d_contact_mm"]],
                     "d_skel_min_max": [metrics[h]["d_skel_mm"], metrics[hm]["d_skel_mm"]],
                     **{f"{y}_{sc}": out[hm][sc][y] - out[h][sc][y] for y in ("ratio", "E", "drop") for sc in ("pooled",) + SEEDS}})
    summ = {}
    for y in ("ratio", "E", "drop"):
        for sc in ("pooled",) + SEEDS:
            d = np.array([r[f"{y}_{sc}"] for r in rows])
            summ[f"{y}_{sc}"] = {"n": len(d), "median": float(np.median(d)), "mean": float(np.mean(d)),
                                 "positive": int((d > 0).sum()),
                                 "wilcoxon_p": float(stats.wilcoxon(d).pvalue) if len(d) and np.any(d != 0) else 1.0}
    return rows, summ


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=Path, default=Path("logs/finger_walk/grid"))
    ap.add_argument("--out", type=Path, default=Path("logs/finger_walk/grid/analysis.json"))
    ap.add_argument("--figure", type=Path)
    args = ap.parse_args()
    metrics = json.loads((args.runs / "hand_metrics.json").read_text())["hands"]
    raw_o = load(args.runs, "O")
    assert raw_o, "O's runs are missing"
    out, missing = {}, []
    for h in metrics:
        raw = load(args.runs, h)
        if raw is None:
            missing.append(h)
            continue
        out[h] = outcomes(raw, raw_o)
    o_rec = outcomes(raw_o, raw_o)
    cfl = [h for h in out if not h.startswith("FA")]
    fa = [h for h in out if h.startswith("FA")]
    res = {"missing": missing, "O": {k: {"walked_mu1.3": o_rec[k]["walked_mu1.3"], "walked_mu2.0": o_rec[k]["walked_mu2.0"],
                                         "D": o_rec[k]["D"]} for k in SEEDS},
           "hands": {h: {"metrics": metrics[h], **{str(k): v for k, v in out[h].items()}} for h in out},
           "n_cfl": len(cfl)}
    res["correlations"] = correlations(cfl, metrics, out)
    seven = [h for h in cfl if metrics[h]["thickness_mm"] == 7.0]
    res["correlations_7mm"] = correlations(seven, metrics, out, ys=("ratio", "E"))
    res["ds_pairs"], res["ds_pairs_summary"] = ds_pairs(metrics, out)
    tol = {}
    for sc in ("pooled",) + SEEDS:
        x = np.array([metrics[h]["d_contact_mm"] for h in cfl])
        y = np.array([out[h][sc]["ratio"] for h in cfl])
        tol[str(sc)] = {"dc50_mm": dc50(x, y), "ci": boot(len(cfl), lambda i: dc50(x[i], y[i]), np.random.default_rng(0))}
    res["tolerance"] = tol
    res["fixed_axis"] = {h: {"metrics": metrics[h], **{str(k): out[h][k] for k in ("pooled",) + SEEDS}} for h in fa}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(res, indent=1, default=float) + "\n")

    print(f"hands: {len(out)} ({len(cfl)} CFL, {len(fa)} fixed axis); missing {missing}")
    print("O walked mu1.3 / mu2.0 / D:", {k: [round(o_rec[k][m], 1) for m in ("walked_mu1.3", "walked_mu2.0", "D")] for k in SEEDS})
    for key, v in res["correlations"].items():
        if "rho" in v:
            print(f"{key:45s} rho {v['rho']:+.3f} p {v['p']:.4f} CI [{v['ci'][0]:+.2f}, {v['ci'][1]:+.2f}]")
        else:
            print(f"{key:45s} {v['value']:+.3f} CI [{v['ci'][0]:+.2f}, {v['ci'][1]:+.2f}]")
    print("7 mm only:")
    for key, v in res["correlations_7mm"].items():
        if "rho" in v and ("d_contact" in key or "d_skel" in key):
            print(f"  {key:43s} rho {v['rho']:+.3f} p {v['p']:.4f} CI [{v['ci'][0]:+.2f}, {v['ci'][1]:+.2f}] n {v['n']}")
    print("Ds pairs:", json.dumps(res["ds_pairs_summary"], indent=None))
    print("tolerance:", json.dumps(tol))
    for h in fa + PAIR_KEYS:
        if h in out:
            print(h, {k: {m: round(out[h][k][m], 3) for m in ("ratio", "E", "drop")} for k in ("pooled",) + SEEDS})
    if args.figure:
        figure(args.figure, cfl, fa, metrics, out)
    return 0


def figure(path: Path, cfl, fa, metrics, out):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {20: "#2a6fbb", 21: "#d9822b"}
    fig, axes = plt.subplots(2, 2, figsize=(9, 6.4), constrained_layout=True)
    for row, y in enumerate(("ratio", "E")):
        for col, x in enumerate(("d_contact_mm", "d_skel_mm")):
            ax = axes[row, col]
            for k in SEEDS:
                xs = [metrics[h][x] for h in cfl]
                ax.scatter(xs, [out[h][k][y] for h in cfl], s=14, color=colors[k], alpha=0.75, label=f"training seed {k}")
                ax.scatter([metrics[h][x] for h in fa], [out[h][k][y] for h in fa], s=30, marker="x", color=colors[k])
                for h in ("R", "U"):
                    ax.annotate(h, (metrics[h][x], out[h][k][y]), fontsize=8, xytext=(3, 3), textcoords="offset points")
            if y == "ratio":
                ax.axhline(0.5, color="0.6", lw=0.8, ls="--")
            ax.set_xlabel({"d_contact_mm": "Δ_contact [mm]", "d_skel_mm": "Δ_skel [mm]"}[x])
            ax.set_ylabel({"ratio": "transfer ratio (walked / O)", "E": "E (μ 1.3 → 2.0)"}[y])
            ax.grid(alpha=0.25)
    axes[0, 0].legend(fontsize=8, frameon=False)
    fig.suptitle("Finger walk: O policies on 68 CFL hands (dots) and the fixed-axis family (×)", fontsize=10)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    print("figure", path)


if __name__ == "__main__":
    raise SystemExit(main())
