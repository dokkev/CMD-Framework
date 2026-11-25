# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3 measurement (problem_formulation A.6.4): the seventeenth experiment (report/study3/study3.md).

    python3 scripts/study3_measurement/padk_matched.py

The fifteenth experiment's redesigns (pads whose contact stiffness falls with thickness, condition ``A-padk``) are
compared with O at the same pad stiffness (``A-padk-ref<h>``): the counterfactual of carrying that pad without the
redesign. Per hand, E' = median |Delta(H) - Delta(O_h)| / |Delta(O_h)| over the placements and directions where both
hold. Pre-registered tests: rho(E', Delta_contact) > 0 (one-sided permutation), E'_R < E'_U at each thickness
(one-sided paired Wilcoxon), and the five candidates' rho with hand-bootstrap intervals. Also the reference's
validity: valid pairs and the effective friction at slip. Writes ``logs/study3/two_pad/padk_matched.json``.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from scipy.stats import wilcoxon

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dose_analysis as da  # noqa: E402
from invariants import boot  # noqa: E402
from padk_analysis import PAIRS, effects  # noqa: E402

HANDS = {**da.HANDS, **da.EXTENSION, **da.THICK}


def permutation_p(x: np.ndarray, y: np.ndarray, n: int = 100_000, seed: int = 0) -> float:
    rng = np.random.default_rng(seed)
    obs = da.spearman(x, y)
    hits = sum(da.spearman(x, rng.permutation(y)) >= obs - 1e-12 for _ in range(n))
    return (hits + 1) / (n + 1)


def validity(h: int) -> dict:
    cond = f"A-padk-ref{h}"
    mu_eff = {1.3: [], 2.0: []}
    gate = {1.3: 0, 2.0: 0}
    for mu in (1.3, 2.0):
        for r in (15.0, 20.0):
            run = da.latest("O", mu, cond, r)
            for e in run["envs"]:
                gate[mu] += e["gate0"]["pass"]
                if e["gate0"]["pass"] and e["limit"].get("mu_eff_at_slip") is not None:
                    mu_eff[mu].append(e["limit"]["mu_eff_at_slip"])
    o = effects("O", cond)
    return {"gate_pass": gate, "valid_effects": len(o),
            "mu_eff_median": {m: (float(np.median(v)) if v else None) for m, v in mu_eff.items()},
            "delta_O_median_n": float(np.median(list(o.values()))) if o else None}


def main() -> int:
    feats = json.loads((da.ROOT / "invariants.json").read_text())["features"]
    out = {"validity": {f"{h}mm": validity(h) for h in (5, 7, 9)}, "hands": {}}
    refs = {h: effects("O", f"A-padk-ref{h}") for h in (5, 7, 9)}
    per_pair = {}
    for hand in HANDS:
        h = int(da.thickness(hand))
        eh = effects(hand, "A-padk")
        o = refs[h]
        keys = [k for k in o if k in eh and abs(o[k]) > 1e-9]
        if not keys:
            continue
        rel = np.array([abs(eh[k] - o[k]) / abs(o[k]) for k in keys])
        per_pair[hand] = dict(zip(keys, rel, strict=True))
        out["hands"][hand] = {"thickness_mm": h, "pairs": len(keys), "E_median": float(np.median(rel))}
    hs = list(out["hands"])
    e = np.array([out["hands"][h]["E_median"] for h in hs])
    tests = {}
    for f in ("contact", "skeleton", "coverage", "thickness", "transmission"):
        x = np.array([feats[h][f] for h in hs])
        tests[f] = {"rho": da.spearman(x, e), "ci95": boot(x, e)}
    dc = np.array([feats[h]["contact"] for h in hs])
    tests["contact"]["p_one_sided"] = permutation_p(dc, e)
    out["candidates"] = tests
    out["n_hands"] = len(hs)
    pairs = {}
    for t, (hr, hu) in PAIRS.items():
        if hr not in per_pair or hu not in per_pair:
            continue
        keys = [k for k in per_pair[hr] if k in per_pair[hu]]
        er = np.array([per_pair[hr][k] for k in keys])
        eu = np.array([per_pair[hu][k] for k in keys])
        test = wilcoxon(er, eu, alternative="less", zero_method="zsplit")
        pairs[f"{t}mm"] = {"pairs": len(keys), "E_R_median": float(np.median(er)), "E_U_median": float(np.median(eu)),
                           "R_less_share": float(np.mean(er < eu)), "wilcoxon_R_less_p": float(test.pvalue)}
    out["R_vs_U"] = pairs
    (da.ROOT / "padk_matched.json").write_text(json.dumps(out, indent=1))
    print(json.dumps({k: out[k] for k in ("validity", "n_hands", "candidates", "R_vs_U")}, indent=1))
    for hand, rec in out["hands"].items():
        print(f"{hand:22s} h {rec['thickness_mm']} pairs {rec['pairs']:3d} E' {100 * rec['E_median']:6.1f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
