# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3 measurement (problem_formulation A.6.4): the eighteenth experiment (separability.md).

    python3 scripts/study3_measurement/torque_grid_analysis.py

Condition C (O's flexion torque reused) on the hand-independent grid, for the 27 redesigns of the thirteenth
experiment, which were never run under torque reuse. Per hand: E = median |Delta(H) - Delta(O)| / |Delta(O)| over
the placements and directions where both hold, and the force deviation |N_d(H)/N_d(O) - 1| (median over the same
pairs, at mu 1.3). Pre-registered tests: the partial rank correlation of E with Delta_contact given the force
deviation, with a hand-bootstrap 95% interval (above zero?); rho(E, Delta_contact) > 0 by a one-sided permutation
test; the five candidates' rho with intervals. Writes ``logs/study3/two_pad/torque_grid.json``.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dose_analysis as da  # noqa: E402
from invariants import boot  # noqa: E402
from padk_matched import permutation_p  # noqa: E402

COND = "C-grid"


def runs_by_key(hand: str, mu: float) -> dict:
    out = {}
    for r in (15.0, 20.0):
        d = da.latest(hand, mu, COND, r)
        if d is None:
            return {}
        out.update({(r, *da.env_key(e)): e for e in d["envs"]})
    return out


def main() -> int:
    feats = json.loads((da.ROOT / "invariants.json").read_text())["features"]
    o = {m: runs_by_key("O", m) for m in (1.3, 2.0)}
    hands = {}
    for hand in da.EXPAND:
        h = {m: runs_by_key(hand, m) for m in (1.3, 2.0)}
        if not h[1.3] or not h[2.0]:
            print(f"{hand}: runs missing")
            continue
        rel, force = [], []
        for k in o[1.3]:
            es = [o[1.3].get(k), o[2.0].get(k), h[1.3].get(k), h[2.0].get(k)]
            if any(e is None or not e["gate0"]["pass"] or e["limit"]["f_limit_n"] is None for e in es):
                continue
            d_o = es[1]["limit"]["f_limit_n"] - es[0]["limit"]["f_limit_n"]
            d_h = es[3]["limit"]["f_limit_n"] - es[2]["limit"]["f_limit_n"]
            if abs(d_o) < 1e-9:
                continue
            rel.append(abs(d_h - d_o) / abs(d_o))
            force.append(es[2]["pre"]["n_distal_total"] / max(es[0]["pre"]["n_distal_total"], 1e-9))
        if rel:
            hands[hand] = {"pairs": len(rel), "E_median": float(np.median(rel)),
                           "force_ratio_median": float(np.median(force))}
    hs = list(hands)
    e = np.array([hands[h]["E_median"] for h in hs])
    dc = np.array([feats[h]["contact"] for h in hs])
    ratio = np.array([hands[h]["force_ratio_median"] for h in hs])
    dev = np.abs(ratio - 1.0)

    def partial(k):
        return da.partial_spearman(e[k], dc[k], dev[k])

    rng = np.random.default_rng(0)
    vals = []
    for _ in range(10_000):
        k = rng.integers(0, len(hs), len(hs))
        if len(set(k)) >= 4:
            vals.append(partial(k))
    out = {"condition": COND, "n_hands": len(hs), "hands": hands,
           "force_ratio_range": [float(ratio.min()), float(ratio.max())],
           "partial_E_Dc_given_force": da.partial_spearman(e, dc, dev),
           "partial_E_Dc_given_force_ci95": [float(np.nanpercentile(vals, 2.5)), float(np.nanpercentile(vals, 97.5))],
           "rho_E_Dc": da.spearman(dc, e), "rho_E_Dc_p_one_sided": permutation_p(dc, e),
           "rho_E_force_dev": da.spearman(dev, e), "candidates": {}}
    for f in ("contact", "skeleton", "coverage", "thickness", "transmission"):
        x = np.array([feats[h][f] for h in hs])
        out["candidates"][f] = {"rho": da.spearman(x, e), "ci95": boot(x, e)}
    (da.ROOT / "torque_grid.json").write_text(json.dumps(out, indent=1))
    print(json.dumps({k: v for k, v in out.items() if k != "hands"}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
