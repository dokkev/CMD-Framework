# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3 measurement (problem_formulation A.6.4): the pre-registered impedance-sweep measures
(``docs/experimental/separability.md``, seventh experiment).

    python3 scripts/study3_measurement/impedance_analysis.py [--hands R U] [--scales 0.1 0.3 1 3 10]

For each flexion stiffness K_q = s K0 (condition ``D<s>`` runs of ``two_pad_wrench.py``, latest per hand, mu, radius,
impratio 1000) and hand H: over the placement-direction pairs where O and H pass Gate 0 at both mu (O at the same
stiffness), the relative error |D(H) - D(O)| / |D(O)| and the symmetric |ln(D(H)/D(O))|, their medians; R against U
pair by pair (one-sided Wilcoxon, E_R < E_U); the gap E_U - E_R and Spearman rho(K, gap) over the stiffnesses
(descriptive); each hand's servo torque over the XM335 stall torque (median of the per-environment maxima, and the
share of environments above 1). Prints a table, writes ``logs/experimental/two_pad/impedance.json`` and the figure
``report/study3/figures/study3_impedance_sweep.png`` (error against stiffness, conditions C and A marked).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dose_analysis as da  # noqa: E402

FIGURE = da.ROOT.parents[4] / "report" / "study3" / "figures" / "study3_impedance_sweep.png"


def pairs(hand: str, cond: str, radii=(15.0, 20.0)) -> dict:
    out = {}
    for r in radii:
        runs = {(h, m): da.latest(h, m, cond, r) for h in ("O", hand) for m in (1.3, 2.0)}
        if any(v is None for v in runs.values()):
            continue
        by = {k: {da.env_key(e): e for e in d["envs"]} for k, d in runs.items()}
        for key in by[("O", 1.3)]:
            es = {k: by[k].get(key) for k in by}
            if any(e is None or not e["gate0"]["pass"] or e["limit"]["f_limit_n"] is None for e in es.values()):
                continue
            d_o = es[("O", 2.0)]["limit"]["f_limit_n"] - es[("O", 1.3)]["limit"]["f_limit_n"]
            d_h = es[(hand, 2.0)]["limit"]["f_limit_n"] - es[(hand, 1.3)]["limit"]["f_limit_n"]
            if abs(d_o) < 1e-9:
                continue
            out[(r,) + key] = {"rel": abs(d_h - d_o) / abs(d_o),
                               "log": abs(np.log(d_h / d_o)) if d_o > 0 and d_h > 0 else np.nan,
                               "share": max(es[(hand, m)].get("servo_torque_share_max", np.nan) for m in (1.3, 2.0))}
    return out


def main() -> int:
    from scipy.stats import wilcoxon

    parser = argparse.ArgumentParser()
    parser.add_argument("--hands", nargs="+", default=("R", "U"))
    parser.add_argument("--scales", type=float, nargs="+", default=(0.1, 0.3, 1.0, 3.0, 10.0))
    args = parser.parse_args()
    out = {"scales": list(args.scales), "hands": {h: {} for h in args.hands}, "R_vs_U": {}}
    print(f"{'K/K0':>6} | " + " | ".join(f"{h}: E_rel  |ln|  pairs  stall-share" for h in args.hands) + " | R<U, p")
    for s in args.scales:
        cond = f"D{s:g}"
        got = {h: pairs(h, cond) for h in args.hands}
        cells = []
        for h in args.hands:
            v = got[h]
            if not v:
                cells.append(f"{h}: (none)")
                continue
            rel = np.array([x["rel"] for x in v.values()])
            lg = np.array([x["log"] for x in v.values()])
            sh = np.array([x["share"] for x in v.values()])
            out["hands"][h][f"{s:g}"] = {"pairs": len(v), "E_rel_median": float(np.median(rel)),
                                         "E_rel_q25": float(np.percentile(rel, 25)), "E_rel_q75": float(np.percentile(rel, 75)),
                                         "E_log_median": float(np.nanmedian(lg)), "stall_share_median": float(np.nanmedian(sh)),
                                         "above_stall": float(np.nanmean(sh > 1.0))}
            cells.append(f"{h}: {100 * np.median(rel):5.1f}% x{np.exp(np.nanmedian(lg)):.2f} {len(v):3d}  "
                         f"{np.nanmedian(sh):.2f} ({100 * np.nanmean(sh > 1):.0f}% >1)")
        line = f"{s:6g} | " + " | ".join(cells)
        if "R" in got and "U" in got:
            common = sorted(set(got["R"]) & set(got["U"]))
            a = np.array([got["R"][k]["rel"] for k in common])
            b = np.array([got["U"][k]["rel"] for k in common])
            if len(common) and np.any(a != b):
                p = float(wilcoxon(a, b, alternative="less").pvalue)
                out["R_vs_U"][f"{s:g}"] = {"pairs": len(common), "R_lt_U": int(np.sum(a < b)), "R_gt_U": int(np.sum(a > b)),
                                           "p_one_sided": p, "gap_median": float(np.median(b) - np.median(a))}
                line += f" | {np.sum(a < b)}/{len(common)}, p {p:.2g}"
        print(line)
    gaps = [(s, out["R_vs_U"][f"{s:g}"]["gap_median"]) for s in args.scales if f"{s:g}" in out["R_vs_U"]]
    if len(gaps) >= 3:
        out["rho_K_gap"] = da.spearman([g[0] for g in gaps], [g[1] for g in gaps])
        print(f"gap E_U - E_R over stiffness: {[(g[0], round(100 * g[1], 1)) for g in gaps]}; Spearman rho(K, gap) "
              f"= {out['rho_K_gap']:+.2f} (descriptive)")
    (da.ROOT / "impedance.json").write_text(json.dumps(out, indent=1))
    figure(out, args)
    return 0


def figure(out: dict, args) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"R": "#0969da", "U": "#cf222e"}
    labels = {"R": "R: contact trajectory kept (D_c 0.86 mm)", "U": "U: coverage only (D_c 3.91 mm)"}
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    for h in args.hands:
        rows = [(float(s), v) for s, v in out["hands"][h].items()]
        if not rows:
            continue
        rows.sort()
        k = np.array([r[0] for r in rows])
        med = np.array([100 * r[1]["E_rel_median"] for r in rows])
        lo = np.array([100 * r[1]["E_rel_q25"] for r in rows])
        hi = np.array([100 * r[1]["E_rel_q75"] for r in rows])
        c = colors.get(h, "#57606a")
        ax.fill_between(k, lo, hi, color=c, alpha=0.15, lw=0)
        ax.plot(k, med, "-o", color=c, lw=2, ms=5, label=labels.get(h, h))
    ax.set_xscale("log")
    ax.axvline(1.0, color="#8c959f", lw=0.8, ls=":")
    ax.text(1.05, ax.get_ylim()[1] * 0.97, "K0: condition A\n(position reuse)", fontsize=7.5, color="#57606a", va="top")
    ax.text(0.105, ax.get_ylim()[1] * 0.97, "→ K_q 0: condition C\n(torque reuse)", fontsize=7.5, color="#57606a", va="top")
    ax.set_xlabel("flexion stiffness K_q / K0 (same on every hand, K0 = 5 N m/rad)", fontsize=8.5, color="#57606a")
    ax.set_ylabel("error in the measured friction effect vs O (%)", fontsize=8.5, color="#57606a")
    ax.set_title("Interface-measurement error against actuation stiffness (median, band: IQR)", fontsize=9.5,
                 loc="left", color="#1f2328")
    ax.grid(color="#d0d7de", lw=0.5)
    ax.tick_params(labelsize=8, colors="#57606a")
    for s in ax.spines.values():
        s.set_color("#d0d7de")
    ax.legend(fontsize=8, frameon=False, loc="center left")
    fig.tight_layout()
    FIGURE.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURE, dpi=200)
    print(FIGURE)


if __name__ == "__main__":
    raise SystemExit(main())
