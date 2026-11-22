# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3 measurement (problem_formulation A.6.4): the pre-registered measures from ``two_pad_wrench.py`` runs (latest
run per hand, mu, condition, radius). See ``docs/experimental/separability.md`` (third experiment, Stage B).

    python3 scripts/study3_measurement/two_pad_analysis.py --mu 1.3 2.0 [--condition B] [--validate]

--validate: the friction check on O alone (condition A): placements passing Gate 0 at mu_lo (needs >= 8) and the
ratio of the slipping pad's |f_t|/|f_n| at slip, hi over lo (needs 1.3-1.7).

Otherwise, for the condition: Gate 0 failure fractions per hand and mu; the common valid set (placement and
direction passing Gate 0 on all three hands at both mu, with a load limit at both); per direction the effect
D(H, d) = F_limit(H, mu_hi, d) - F_limit(H, mu_lo, d), E(H, d) = |D(H, d) - D(O, d)| (median of E / |D(O)|, and the
share of pairs with E_R < E_U); the measured grasp flexion change q*(H) - q*(O) against e16's prediction; the
normals' angle against q*; N_p / N_d; the limiting events. Writes ``logs/experimental/two_pad/summary_<cond>.json``.
"""

from __future__ import annotations

import argparse
import glob
import json
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2] / "logs" / "experimental" / "two_pad"
HANDS = ("O", "R", "U")
DIRECTIONS = ("+x", "+x-z", "-z", "-x")


def load() -> dict:
    runs = {}
    for f in sorted(glob.glob(str(ROOT / "tp_*.json"))):
        d = json.loads(Path(f).read_text())
        if d.get("impratio") != 1000.0:
            continue
        r = d["envs"][0]["rho_mm"]
        runs[(d["hand"], d["mu"], d["condition"], r)] = d | {"file": Path(f).name}  # latest wins
    return runs


def by_key(d: dict) -> dict:
    return {(e["rho_mm"], e["q0_deg"], e["direction"]): e for e in d["envs"]}


def validate(runs: dict, lo: float, hi: float) -> None:
    for r in sorted({k[3] for k in runs}):
        if not all(("O", m, "A", r) in runs for m in (lo, hi)):
            continue
        print(f"radius {r:g} mm, O, condition A:")
        for m in (lo, hi):
            envs = runs[("O", m, "A", r)]["envs"]
            placements = {}
            for e in envs:
                placements.setdefault(e["q0_deg"], []).append(e["gate0"]["pass"])
            ok = sorted(q for q, v in placements.items() if all(v))
            mu = [e["limit"]["mu_eff_at_slip"] for e in envs if e["gate0"]["pass"] and e["limit"]["event"] == "slip"
                  and e["limit"]["mu_eff_at_slip"] is not None]
            print(f"  mu {m:g}: Gate 0 placements {len(ok)}/{len(placements)} (q0 {ok}); slipping-pad |f_t|/|f_n| "
                  f"median {np.median(mu) if mu else float('nan'):.3f} (n {len(mu)})")
        mus = [np.median([e["limit"]["mu_eff_at_slip"] for e in runs[("O", m, "A", r)]["envs"]
                          if e["gate0"]["pass"] and e["limit"]["event"] == "slip"
                          and e["limit"]["mu_eff_at_slip"] is not None] or [np.nan]) for m in (lo, hi)]
        print(f"  ratio hi/lo {mus[1] / mus[0]:.3f} (accept 1.3-1.7)")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mu", type=float, nargs=2, required=True)
    parser.add_argument("--condition", default="B", choices=("A", "B"))
    parser.add_argument("--validate", action="store_true")
    args = parser.parse_args()
    lo, hi = args.mu
    runs = load()
    if args.validate:
        validate(runs, lo, hi)
        return 0
    cond = args.condition
    radii = sorted({k[3] for k in runs if k[2] == cond})
    summary = {"condition": cond, "mu": [lo, hi], "runs": {}, "failure": {}, "directions": {}, "pairs": []}
    pairs = []
    for r in radii:
        if not all((h, m, cond, r) in runs for h in HANDS for m in (lo, hi)):
            print(f"radius {r:g}: incomplete, skipped")
            continue
        env = {(h, m): by_key(runs[(h, m, cond, r)]) for h in HANDS for m in (lo, hi)}
        for (h, m), d in env.items():
            summary["runs"][f"{h}_{m:g}_r{r:g}"] = runs[(h, m, cond, r)]["file"]
            fails = Counter()
            for e in d.values():
                g = e["gate0"]
                if not g["pass"]:
                    fails["any"] += 1
                    for k in ("pads", "still", "angle", "matched"):
                        fails[k] += not g[k]
            summary["failure"].setdefault(f"{h}_{m:g}", Counter()).update({**fails, "envs": len(d)})
        for key in env[("O", lo)]:
            es = {(h, m): env[(h, m)].get(key) for h in HANDS for m in (lo, hi)}
            if any(e is None for e in es.values()):
                continue
            valid = all(e["gate0"]["pass"] and e["limit"]["f_limit_n"] is not None for e in es.values())
            f = {hm: e["limit"]["f_limit_n"] for hm, e in es.items()}
            rec = {"rho_mm": key[0], "q0_deg": key[1], "direction": key[2], "valid": valid,
                   "q_star": {f"{h}_{m:g}": float(np.mean(es[(h, m)]["pre"]["q_star_deg"])) for h in HANDS for m in (lo, hi)},
                   "q_pred": {h: es[(h, lo)]["q_star_predicted_deg"] for h in HANDS},
                   "angle_err": max(abs(a - b) for e in es.values() for a, b in
                                    zip(e["pre"]["normal_angle_deg"], e["pre"]["q_finger_deg"])),
                   "np_nd": {f"{h}_{m:g}": es[(h, m)]["pre"]["n_proximal_total"] / max(es[(h, m)]["pre"]["n_distal_total"], 1e-9)
                             for h in HANDS for m in (lo, hi)},
                   "n_distal": {f"{h}_{m:g}": es[(h, m)]["pre"]["n_distal_total"] for h in HANDS for m in (lo, hi)},
                   "event": {f"{h}_{m:g}": es[(h, m)]["limit"]["event"] for h in HANDS for m in (lo, hi)},
                   "f_limit": {f"{h}_{m:g}": f[(h, m)] for h in HANDS for m in (lo, hi)}}
            if valid:
                dd = {h: f[(h, hi)] - f[(h, lo)] for h in HANDS}
                rec |= {"effect": dd, "E_R": abs(dd["R"] - dd["O"]), "E_U": abs(dd["U"] - dd["O"])}
            pairs.append(rec)
    summary["pairs"] = pairs
    print(f"condition {cond}, mu {lo:g} -> {hi:g}")
    print("Gate 0 failures (envs):")
    for k, c in summary["failure"].items():
        print(f"  {k}: {c.get('any', 0)}/{c['envs']} (pads {c.get('pads', 0)}, still {c.get('still', 0)}, "
              f"angle {c.get('angle', 0)}, matched {c.get('matched', 0)})")
    valid = [p for p in pairs if p["valid"]]
    print(f"common valid pairs: {len(valid)}/{len(pairs)} "
          f"(placements {len({(p['rho_mm'], p['q0_deg']) for p in valid})})")
    for dname in DIRECTIONS:
        v = [p for p in valid if p["direction"] == dname]
        if not v:
            print(f"  {dname:5s}: none")
            continue
        d_o = np.array([p["effect"]["O"] for p in v])
        er, eu = np.array([p["E_R"] for p in v]), np.array([p["E_U"] for p in v])
        rel_r, rel_u = er / np.abs(d_o), eu / np.abs(d_o)
        share = float(np.mean(er < eu))
        summary["directions"][dname] = {"n": len(v), "D_O_median": float(np.median(d_o)),
                                        "E_R_rel_median": float(np.median(rel_r)), "E_U_rel_median": float(np.median(rel_u)),
                                        "E_R_median": float(np.median(er)), "E_U_median": float(np.median(eu)),
                                        "share_E_R_lt_E_U": share}
        print(f"  {dname:5s}: n {len(v):2d}  D(O) {np.median(d_o):6.2f} N  E_R {np.median(er):5.2f} N "
              f"({100 * np.median(rel_r):5.1f}%)  E_U {np.median(eu):5.2f} N ({100 * np.median(rel_u):5.1f}%)  "
              f"E_R<E_U in {100 * share:.0f}%")
    if valid:
        for h in ("R", "U"):
            meas = np.array([p["q_star"][f"{h}_{lo:g}"] - p["q_star"][f"O_{lo:g}"] for p in valid])
            pred = np.array([p["q_pred"][h] - p["q_pred"]["O"] for p in valid])
            print(f"q*({h}) - q*(O) at mu {lo:g}: measured median {np.median(meas):+.2f} deg, e16 predicted "
                  f"{np.median(pred):+.2f} deg (median |measured - predicted| {np.median(np.abs(meas - pred)):.2f})")
        print(f"normals' angle vs q*: max error {max(p['angle_err'] for p in valid):.3f} deg")
        for h in HANDS:
            print(f"N_p/N_d {h}: median {np.median([p['np_nd'][f'{h}_{lo:g}'] for p in valid]):.3f}; events at mu {lo:g}: "
                  f"{dict(Counter(p['event'][f'{h}_{lo:g}'] for p in valid))}")
    summary["failure"] = {k: dict(v) for k, v in summary["failure"].items()}
    path = ROOT / f"summary_{cond}.json"
    path.write_text(json.dumps(summary, indent=1))
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
