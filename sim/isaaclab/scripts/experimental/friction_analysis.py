# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Experimental: the separability measures from ``friction_slip.py`` runs (latest run per hand, mu, condition,
radius; runs that predate the two torque directions are skipped).

    python3 scripts/experimental/friction_analysis.py [--radius_mm 15] [--threshold 0.5]

Per environment (O's contact flexion q0 x torque direction) and condition (A: fixed command, B: force-matched to O):
- the interface effect D(H) = T_slip(H, mu_hi) - T_slip(H, mu_lo);
- the raw artefact T(H) = T_slip(H, mu_lo) - T_slip(O, mu_lo) for R and U;
- the separability error E(H) = |D(H) - D(O)|, also as a fraction of D(O).
Also the effective friction T_slip / (rho N_slip) (a check: the same on every hand at one mu) and the preload.
Prints tables and writes ``logs/experimental/friction/summary_r<radius>.json``.
"""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2] / "logs" / "experimental" / "friction"
HANDS = ("O", "R", "U")


def load(radius: int) -> dict:
    runs = {}
    for f in sorted(glob.glob(str(ROOT / f"fric_*_r{radius}_*.json"))):
        d = json.loads(Path(f).read_text())
        if "torque_sign" not in d["envs"][0]:
            continue
        runs[(d["hand"], d["mu"], d["condition"])] = d | {"file": Path(f).name}  # sorted by time: latest wins
    return runs


def slip(d: dict, threshold: str) -> np.ndarray:
    return np.array([np.nan if e["t_slip_by_threshold"][threshold] is None else e["t_slip_by_threshold"][threshold]
                     for e in d["envs"]])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--radius_mm", type=int, default=15)
    parser.add_argument("--threshold", default="0.5", help="slip speed [rad/s]: 0.25, 0.5 or 1")
    args = parser.parse_args()
    runs = load(args.radius_mm)
    mus = sorted({k[1] for k in runs})
    lo, hi = mus[0], mus[-1]
    rho = args.radius_mm * 1e-3
    summary = {"radius_mm": args.radius_mm, "threshold_rad_s": float(args.threshold), "mu": [lo, hi],
               "runs": sorted(d["file"] for d in runs.values()), "conditions": {}}
    any_run = next(iter(runs.values()))
    labels = [f"q{e['q0_deg']}{'+' if e['torque_sign'] > 0 else '-'}" for e in any_run["envs"]]
    for cond in ("A", "B"):
        if not all((h, m, cond) in runs for h in HANDS for m in (lo, hi)):
            print(f"condition {cond}: incomplete, skipped")
            continue
        t = {(h, m): slip(runs[(h, m, cond)], args.threshold) for h in HANDS for m in (lo, hi)}
        pre = {(h, m): np.array([e["force_total_n"] for e in runs[(h, m, cond)]["envs"]]) for h in HANDS for m in (lo, hi)}
        n_slip = {(h, m): np.array([e["force_at_slip_n"] or np.nan for e in runs[(h, m, cond)]["envs"]])
                  for h in HANDS for m in (lo, hi)}
        q_cmd = {h: np.array([e["q_cmd_deg"] for e in runs[(h, lo, cond)]["envs"]]) for h in HANDS}
        effect = {h: t[(h, hi)] - t[(h, lo)] for h in HANDS}
        out = {"labels": labels}
        for h in HANDS:
            out[h] = {"t_slip_lo": t[(h, lo)].tolist(), "t_slip_hi": t[(h, hi)].tolist(), "effect": effect[h].tolist(),
                      "preload_lo_n": pre[(h, lo)].tolist(), "preload_hi_n": pre[(h, hi)].tolist(),
                      "q_cmd_lo_deg": q_cmd[h].tolist(),
                      "mu_eff_lo": (t[(h, lo)] / (rho * n_slip[(h, lo)])).tolist(),
                      "mu_eff_hi": (t[(h, hi)] / (rho * n_slip[(h, hi)])).tolist()}
        for h in ("R", "U"):
            err = np.abs(effect[h] - effect["O"])
            out[h] |= {"artefact": (t[(h, lo)] - t[("O", lo)]).tolist(), "error": err.tolist(),
                       "error_rel": (err / np.abs(effect["O"])).tolist(),
                       "error_median": float(np.nanmedian(err)),
                       "error_rel_median": float(np.nanmedian(err / np.abs(effect["O"])))}
        summary["conditions"][cond] = out

        print(f"\n## condition {cond}  (mu {lo:g} vs {hi:g}, radius {args.radius_mm} mm, slip at {args.threshold} rad/s)")
        print("env    | " + " | ".join(f"{h}: q_cmd  N_pre  T_lo   T_hi   D     mu_eff" for h in HANDS))
        for k, lab in enumerate(labels):
            cells = [f"{h}: {q_cmd[h][k]:5.1f} {pre[(h, lo)][k]:5.1f} {t[(h, lo)][k]:.3f} {t[(h, hi)][k]:.3f} "
                     f"{effect[h][k]:.3f} {out[h]['mu_eff_lo'][k]:.2f}/{out[h]['mu_eff_hi'][k]:.2f}" for h in HANDS]
            print(f"{lab:6s} | " + " | ".join(cells))
        for h in ("R", "U"):
            print(f"E_{h}: median {out[h]['error_median']:.4f} N m ({100 * out[h]['error_rel_median']:.0f}% of D(O)); "
                  f"per env {np.round(out[h]['error'], 3).tolist()}; artefact median "
                  f"{np.nanmedian(out[h]['artefact']):.3f} N m")
    path = ROOT / f"summary_r{args.radius_mm}.json"
    path.write_text(json.dumps(summary, indent=1))
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
