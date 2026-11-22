# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3 measurement (problem_formulation A.6.4): the R-versus-U test of the fifteenth and sixteenth experiments.

    python3 scripts/study3_measurement/padk_analysis.py [--condition A-padk|A-hydro]

A-padk: pad contact stiffness falling with thickness (fifteenth). A-hydro: hydroelastic pads with the reference's pad
mechanics on every hand (sixteenth). For R and U at each thickness (the same pads),
the error E against O per placement and direction over the pairs where O, R and U all hold; the pre-registered
one-sided paired Wilcoxon test of E_R < E_U; and, as an exploratory reading, the signed effect differences
Delta(H) - Delta(O), whose common pad offset cancels in R - U. Writes
``logs/experimental/two_pad/pairs_<condition>.json`` (``padk.json`` for A-padk, as first written).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from scipy.stats import wilcoxon

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dose_analysis as da  # noqa: E402

PAIRS = {5: ("h3to5_hr", "h3to5_hu"), 7: ("R", "U"), 9: ("h3to9_hr", "h3to9_hu")}


def effects(hand: str, cond: str, lo: float = 1.3, hi: float = 2.0) -> dict:
    out = {}
    for r in (15.0, 20.0):
        runs = {m: da.latest(hand, m, cond, r) for m in (lo, hi)}
        if any(v is None for v in runs.values()):
            continue
        by = {m: {da.env_key(e): e for e in d["envs"]} for m, d in runs.items()}
        for k, e_lo in by[lo].items():
            e_hi = by[hi].get(k)
            if e_hi is None or not (e_lo["gate0"]["pass"] and e_hi["gate0"]["pass"]):
                continue
            if e_lo["limit"]["f_limit_n"] is None or e_hi["limit"]["f_limit_n"] is None:
                continue
            out[(r, *k)] = e_hi["limit"]["f_limit_n"] - e_lo["limit"]["f_limit_n"]
    return out


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--condition", default="A-padk", choices=("A-padk", "A-hydro", "A"))
    cond = parser.parse_args().condition
    o = effects("O", cond)
    result = {"condition": cond}
    for t, (hr, hu) in PAIRS.items():
        r, u = effects(hr, cond), effects(hu, cond)
        keys = [k for k in o if k in r and k in u and abs(o[k]) > 1e-9]
        e_r = np.array([abs(r[k] - o[k]) / abs(o[k]) for k in keys])
        e_u = np.array([abs(u[k] - o[k]) / abs(o[k]) for k in keys])
        s_r = np.array([r[k] - o[k] for k in keys])
        s_u = np.array([u[k] - o[k] for k in keys])
        test = wilcoxon(e_r, e_u, alternative="less", zero_method="zsplit") if len(keys) >= 6 else None
        result[f"{t}mm"] = {
            "pairs": len(keys), "E_R_median": float(np.median(e_r)), "E_U_median": float(np.median(e_u)),
            "R_less_share": float(np.mean(e_r < e_u)),
            "wilcoxon_R_less_p": None if test is None else float(test.pvalue),
            "signed_R_median_n": float(np.median(s_r)), "signed_U_median_n": float(np.median(s_u)),
            "R_minus_U_median_n": float(np.median(s_r - s_u)),
        }
        print(t, json.dumps(result[f"{t}mm"]))
    name = "padk.json" if cond == "A-padk" else f"pairs_{cond}.json"
    (da.ROOT / name).write_text(json.dumps(result, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
