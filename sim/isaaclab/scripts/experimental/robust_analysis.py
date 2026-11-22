# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Experimental: the pre-registered robustness and shape-interface measures (``docs/experimental/separability.md``,
eighth experiment).

    python3 scripts/experimental/robust_analysis.py robust      # condition A under each robustness setting
    python3 scripts/experimental/robust_analysis.py shape-gate  # O: crowned minus flat, per direction
    python3 scripts/experimental/robust_analysis.py shape       # every hand: error in the shape effect

robust: for the baseline (impratio 1000, ke 2.5e3, 2 substeps; the main condition-A runs) and each tagged setting,
R and U against O at radius 15 mm: median E_R and E_U, R < U pair count, one-sided Wilcoxon p, and the median
distal force against O's. Verdict per setting: "kept" if p < 0.05 and median E_R < median E_U.
shape-gate: O's effect F_limit(crowned) - F_limit(flat) at mu 1.3 per direction (radii 15 and 20); passes if the
median is at least 0.2 N (3 load steps) in two or more directions.
shape: E_shape(H) = |D(H) - D(O)| / |D(O)| per hand (D = crowned minus flat at mu 1.3), rho(E, D_c), R vs U.
Writes ``logs/experimental/two_pad/robust.json`` / ``shape.json``.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dose_analysis as da  # noqa: E402

SETTINGS = ("baseline", "ke1e3", "ke1e4", "imp300", "imp3000", "sub4")
DIRECTIONS = ("+x", "+x-z", "-z", "-x")
STEP_N = 2.0 / 30.0


def latest_tagged(hand: str, mu: float, cond: str, radius: float) -> dict | None:
    import glob
    found = None
    for f in sorted(glob.glob(str(da.ROOT / f"tp_{hand}_mu{mu:g}_{cond}_r{radius:g}_*.json"))):
        found = json.loads(Path(f).read_text())
    return found


def run_for(setting: str, hand: str, mu: float, radius: float = 15.0) -> dict | None:
    if setting == "baseline":
        return da.latest(hand, mu, "A", radius)
    return latest_tagged(hand, mu, f"A-{setting}", radius)


def by_key(d: dict) -> dict:
    return {(e["q0_deg"], e["direction"]): e for e in d["envs"]}


def errors(get, hand: str) -> tuple[dict, list[float]]:
    """Per pair: relative error of hand against O for the effect get(hand) -> {key: D}; plus force ratios."""
    eo, eh = get("O"), get(hand)
    out = {}
    for k, d_o in eo.items():
        if k in eh and abs(d_o) > 1e-9:
            out[k] = abs(eh[k] - d_o) / abs(d_o)
    return out


def friction_effect(setting: str):
    def get(hand: str) -> dict:
        lo, hi = run_for(setting, hand, 1.3), run_for(setting, hand, 2.0)
        if lo is None or hi is None:
            return {}
        a, b = by_key(lo), by_key(hi)
        out = {}
        for k in a:
            e1, e2 = a[k], b.get(k)
            if e2 is None or not (e1["gate0"]["pass"] and e2["gate0"]["pass"]):
                continue
            if e1["limit"]["f_limit_n"] is None or e2["limit"]["f_limit_n"] is None:
                continue
            out[k] = e2["limit"]["f_limit_n"] - e1["limit"]["f_limit_n"]
        return out
    return get


def force_ratio(setting: str, hand: str) -> float:
    a, o = run_for(setting, hand, 1.3), run_for(setting, "O", 1.3)
    if a is None or o is None:
        return float("nan")
    ka, ko = by_key(a), by_key(o)
    r = [ka[k]["pre"]["n_distal_total"] / ko[k]["pre"]["n_distal_total"] for k in ka
         if k in ko and ka[k]["gate0"]["pass"] and ko[k]["gate0"]["pass"]]
    return float(np.median(r)) if r else float("nan")


def robust() -> int:
    from scipy.stats import wilcoxon
    out = {}
    print(f"{'setting':>9} | E_R     E_U    | R<U    p        | N_d/N_d(O): R    U    | verdict")
    for st in SETTINGS:
        get = friction_effect(st)
        er, eu = errors(get, "R"), errors(get, "U")
        common = sorted(set(er) & set(eu))
        if not common:
            print(f"{st:>9} | (missing)")
            continue
        a, b = np.array([er[k] for k in common]), np.array([eu[k] for k in common])
        p = float(wilcoxon(a, b, alternative="less").pvalue) if np.any(a != b) else float("nan")
        kept = bool(p < 0.05 and np.median(a) < np.median(b))
        fr, fu = force_ratio(st, "R"), force_ratio(st, "U")
        out[st] = {"pairs": len(common), "E_R": float(np.median(a)), "E_U": float(np.median(b)),
                   "R_lt_U": int(np.sum(a < b)), "p": p, "force_ratio_R": fr, "force_ratio_U": fu, "kept": kept}
        print(f"{st:>9} | {100 * np.median(a):5.1f}% {100 * np.median(b):6.1f}% | {np.sum(a < b):2d}/{len(common):2d} "
              f"{p:8.2g} | {fr:4.2f} {fu:4.2f} | {'kept' if kept else 'NOT kept'}")
    (da.ROOT / "robust.json").write_text(json.dumps(out, indent=1))
    return 0


def shape_effect(hand: str) -> dict:
    out = {}
    for r in (15.0, 20.0):
        flat, crowned = da.latest(hand, 1.3, "A", r), da.latest(hand, 1.3, "A-crowned", r)
        if flat is None or crowned is None:
            continue
        a, b = by_key(flat), by_key(crowned)
        for k in a:
            e1, e2 = a[k], b.get(k)
            if e2 is None or not (e1["gate0"]["pass"] and e2["gate0"]["pass"]):
                continue
            if e1["limit"]["f_limit_n"] is None or e2["limit"]["f_limit_n"] is None:
                continue
            out[(r,) + k] = e2["limit"]["f_limit_n"] - e1["limit"]["f_limit_n"]
    return out


def shape_gate() -> int:
    eff = shape_effect("O")
    passed = 0
    for d in DIRECTIONS:
        v = [x for k, x in eff.items() if k[2] == d]
        med = float(np.median(v)) if v else float("nan")
        ok = abs(med) >= 3 * STEP_N - 1e-9
        passed += ok
        print(f"O shape effect {d:5s}: n {len(v):2d}, median {med:+.3f} N ({med / STEP_N:+.1f} steps) "
              f"{'>= 3 steps' if ok else ''}")
    print(f"Gate: {passed} of 4 directions >= 3 steps -> {'PASS' if passed >= 2 else 'FAIL'}")
    return 0


def shape() -> int:
    from scipy.stats import wilcoxon
    eo = shape_effect("O")
    hands = {"R": da.HANDS["R"], "U": da.HANDS["U"], "d1p5_minDs_s0": da.HANDS["d1p5_minDs_s0"],
             "d1p5_maxDs_s0": da.EXTENSION["d1p5_maxDs_s0"], "d2p25_minDs_s0": da.HANDS["d2p25_minDs_s0"],
             "d3_minDs_s0": da.HANDS["d3_minDs_s0"]}
    rows, per = [], {}
    for h, (dc, ds, cp) in hands.items():
        eh = shape_effect(h)
        e = {k: abs(eh[k] - eo[k]) / abs(eo[k]) for k in eo if k in eh and abs(eo[k]) > 1e-9}
        if not e:
            continue
        per[h] = e
        rows.append((h, dc, float(np.median(list(e.values()))), len(e)))
        print(f"{h:15s} D_c {dc:4.2f} | pairs {len(e):3d} E_shape {100 * np.median(list(e.values())):6.1f}%")
    out = {"hands": {h: {"D_c_mm": dc, "E_median": e, "pairs": n} for h, dc, e, n in rows}}
    if len(rows) >= 4:
        import itertools
        dc = np.array([r[1] for r in rows]); e = np.array([r[2] for r in rows])
        rho = da.spearman(dc, e)
        perms = [da.spearman(dc, np.array(p)) for p in itertools.permutations(e)]
        p = float(np.mean([v >= rho - 1e-12 for v in perms]))
        out["rho_E_Dc"], out["p_one_sided"] = rho, p
        print(f"rho(E_shape, D_c) = {rho:+.3f}, exact one-sided p = {p:.4f} (n {len(rows)})")
    if "R" in per and "U" in per:
        common = sorted(set(per["R"]) & set(per["U"]))
        a, b = np.array([per["R"][k] for k in common]), np.array([per["U"][k] for k in common])
        if len(common) and np.any(a != b):
            pw = float(wilcoxon(a, b, alternative="less").pvalue)
            out["R_vs_U"] = {"pairs": len(common), "R_lt_U": int(np.sum(a < b)), "p": pw}
            print(f"R vs U: {np.sum(a < b)}/{len(common)} R < U, Wilcoxon p {pw:.2g}")
    (da.ROOT / "shape.json").write_text(json.dumps(out, indent=1))
    return 0


def comp_effect(hand: str) -> dict:
    """Pad compliance: F_limit(ke 1e3) - F_limit(ke 2.5e3, the condition-A baseline), mu 1.3, radii 15 and 20."""
    out = {}
    for r in (15.0, 20.0):
        stiff, soft = da.latest(hand, 1.3, "A", r), latest_tagged(hand, 1.3, "A-ke1e3", r)
        if stiff is None or soft is None:
            continue
        a, b = by_key(stiff), by_key(soft)
        for k in a:
            e1, e2 = a[k], b.get(k)
            if e2 is None or not (e1["gate0"]["pass"] and e2["gate0"]["pass"]):
                continue
            if e1["limit"]["f_limit_n"] is None or e2["limit"]["f_limit_n"] is None:
                continue
            out[(r,) + k] = e2["limit"]["f_limit_n"] - e1["limit"]["f_limit_n"]
    return out


def comp() -> int:
    eo = comp_effect("O")
    hands = {**da.HANDS, **da.EXTENSION, **da.THICK}
    rows = []
    for h, (dc, ds, cp) in hands.items():
        eh = comp_effect(h)
        e = [abs(eh[k] - eo[k]) / abs(eo[k]) for k in eo if k in eh and abs(eo[k]) > 1e-9]
        if not e:
            print(f"{h:20s} no valid pairs")
            continue
        rows.append((h, dc, ds, float(np.median(e)), len(e)))
        print(f"{h:20s} h {da.thickness(h):g} D_c {dc:4.2f} D_s {ds:5.2f} | pairs {len(e):3d} E_comp {100 * np.median(e):6.1f}%")
    out = {"hands": {h: {"D_c_mm": dc, "D_s_mm": ds, "E_median": e, "pairs": n} for h, dc, ds, e, n in rows}}
    if len(rows) >= 4:
        dc, ds, e = (np.array([r[i] for r in rows]) for i in (1, 2, 3))
        rho = da.spearman(dc, e)
        rng = np.random.default_rng(0)
        perms = [da.spearman(dc, rng.permutation(e)) for _ in range(100_000)]
        p = float(np.mean([v >= rho - 1e-12 for v in perms]))
        pc, pd = da.partial_spearman(e, dc, ds), da.partial_spearman(e, ds, dc)
        ci = da.bootstrap_ci(e, dc, ds)
        out |= {"n_hands": len(rows), "rho_E_Dc": rho, "p_one_sided": p, "rho_E_Ds": da.spearman(ds, e),
                "partial_E_Dc_given_Ds": pc, "partial_E_Ds_given_Dc": pd, "bootstrap_ci95": ci}
        print(f"rho(E_comp, D_c) = {rho:+.3f} [{ci['rho_E_Dc'][0]:+.2f}, {ci['rho_E_Dc'][1]:+.2f}], one-sided p = {p:.4f} "
              f"(n {len(rows)}); rho(E, D_s) = {out['rho_E_Ds']:+.3f}; partial E-D_c | D_s {pc:+.3f} "
              f"[{ci['partial_E_Dc_given_Ds'][0]:+.2f}, {ci['partial_E_Dc_given_Ds'][1]:+.2f}], E-D_s | D_c {pd:+.3f} "
              f"[{ci['partial_E_Ds_given_Dc'][0]:+.2f}, {ci['partial_E_Ds_given_Dc'][1]:+.2f}]")
    (da.ROOT / "comp.json").write_text(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "robust"
    raise SystemExit({"robust": robust, "shape-gate": shape_gate, "shape": shape, "comp": comp}[mode]())
