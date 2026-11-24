# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3 measurement (problem_formulation A.6.4): does Delta_contact act only through the contact force?

    python3 scripts/study3_measurement/mediation.py

For the reused flexion command (A, dose_A_all.json) and the reused flexion torque (C, dose_C_all.json): the force
deviation |N_d(H)/N_d(O) - 1| (median distal force ratio over the valid pairs) as a sixth quantity, its Spearman rho
with the error, rho(Delta_contact, force ratio), and the partial rank correlations E-Delta_contact given the force
deviation and E-force deviation given Delta_contact, each with a hand-bootstrap 95% interval. Also the Spearman
correlations among the five candidates over the 45 hands of invariants.json. Writes
``logs/study3/two_pad/mediation.json``.
"""

from __future__ import annotations

import itertools
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dose_analysis as da  # noqa: E402


def interval(fn, n=10_000, seed=0):
    rng = np.random.default_rng(seed)
    size = fn.size
    v = []
    for _ in range(n):
        k = rng.integers(0, size, size)
        if len(set(k)) >= 4:
            v.append(fn(k))
    return [float(np.nanpercentile(v, 2.5)), float(np.nanpercentile(v, 97.5))]


def condition(name: str) -> dict:
    hands = json.loads((da.ROOT / f"{name}.json").read_text())["hands"]
    keys = [h for h, r in hands.items() if r.get("E_rel_median") is not None
            and r.get("distal_force_ratio_median") is not None]
    e = np.array([hands[h]["E_rel_median"] for h in keys])
    dc = np.array([hands[h]["D_c_mm"] for h in keys])
    ratio = np.array([hands[h]["distal_force_ratio_median"] for h in keys])
    dev = np.abs(ratio - 1.0)

    def partial_dc(k):
        return da.partial_spearman(e[k], dc[k], dev[k])

    def partial_force(k):
        return da.partial_spearman(e[k], dev[k], dc[k])

    partial_dc.size = partial_force.size = len(keys)
    return {
        "n_hands": len(keys),
        "force_ratio_range": [float(ratio.min()), float(ratio.max())],
        "rho_E_force_dev": da.spearman(dev, e),
        "rho_Dc_force_ratio": da.spearman(dc, ratio),
        "rho_E_Dc": da.spearman(dc, e),
        "partial_E_Dc_given_force": da.partial_spearman(e, dc, dev),
        "partial_E_Dc_given_force_ci95": interval(partial_dc),
        "partial_E_force_given_Dc": da.partial_spearman(e, dev, dc),
        "partial_E_force_given_Dc_ci95": interval(partial_force),
    }


def main() -> int:
    out = {"A_reused_command": condition("dose_A_all"), "C_reused_torque": condition("dose_C_all")}
    feats = json.loads((da.ROOT / "invariants.json").read_text())["features"]
    names = list(next(iter(feats.values())))
    cols = {f: np.array([feats[h][f] for h in feats]) for f in names}
    out["candidate_rho_45_hands"] = {f"{a}-{b}": da.spearman(cols[a], cols[b])
                                     for a, b in itertools.combinations(names, 2)}
    (da.ROOT / "mediation.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
