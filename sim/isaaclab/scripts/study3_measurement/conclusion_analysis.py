# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 4 (problem_formulation A.6.5): does a redesign change an interface study's conclusion?

    python3 scripts/study3_measurement/conclusion_analysis.py [--set study3|grid]

The interface change: O's stiff 3 mm pads to a thicker pad whose contact stiffness falls with thickness
(2.5 kN/m x 3 mm / h). Its true effect on the holding load (the limit load at mu 1.3 and 2.0) is measured on O
carrying the soft pad (``A-padk-ref<h>``: no redesign); an interface study instead measures it on a hand redesigned
to carry the pad (``A-padk``). Per hand: the median measured and true change relative to O's load, the relative
error of the measured change, and the share of placements and directions where its sign differs from the true one
(a reversed conclusion). ``--set study3``: the fifteenth and seventeenth experiments' runs (post hoc reading);
``--set grid``: the nineteenth experiment (grid placements, tags ``grid-padk`` and ``grid-padk-ref7``). Writes
``logs/study3/two_pad/conclusion_<set>.json``.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dose_analysis as da  # noqa: E402
from invariants import boot  # noqa: E402
from padk_matched import permutation_p  # noqa: E402

SETS = {
    "study3": {"hands": {**da.HANDS, **da.EXTENSION, **da.THICK}, "base": "A", "hand": "A-padk",
               "ref": lambda h: f"A-padk-ref{h}"},
    "grid": {"hands": da.EXPAND, "base": "A-grid", "hand": "A-grid-padk", "ref": lambda h: "A-grid-padk-ref7"},
}


def loads(hand: str, cond: str, mu: float) -> dict:
    out = {}
    for r in (15.0, 20.0):
        d = da.latest(hand, mu, cond, r)
        if d is None:
            continue
        for e in d["envs"]:
            if e["gate0"]["pass"] and e["limit"]["f_limit_n"] is not None:
                out[(mu, r, *da.env_key(e))] = e["limit"]["f_limit_n"]
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--set", default="study3", choices=tuple(SETS))
    spec = SETS[parser.parse_args().set]
    feats = json.loads((da.ROOT / "invariants.json").read_text())["features"]
    base = {**loads("O", spec["base"], 1.3), **loads("O", spec["base"], 2.0)}
    out = {"set": spec["base"], "hands": {}}
    for hand in spec["hands"]:
        h = int(da.thickness(hand))
        ref = {**loads("O", spec["ref"](h), 1.3), **loads("O", spec["ref"](h), 2.0)}
        got = {**loads(hand, spec["hand"], 1.3), **loads(hand, spec["hand"], 2.0)}
        keys = [k for k in base if k in ref and k in got]
        if not keys:
            continue
        b = np.array([base[k] for k in keys])
        true = np.array([ref[k] for k in keys]) - b
        meas = np.array([got[k] for k in keys]) - b
        out["hands"][hand] = {
            "thickness_mm": h, "pairs": len(keys),
            "true_change_pct": float(100 * np.median(true) / np.median(b)),
            "measured_change_pct": float(100 * np.median(meas) / np.median(b)),
            "reversed_share": float(np.mean(np.sign(meas) != np.sign(true))),
            "relative_error": float(np.median(np.abs(meas - true)) / max(abs(float(np.median(true))), 1e-9)),
        }
    hs = list(out["hands"])
    dc = np.array([feats[h]["contact"] for h in hs])
    for key in ("reversed_share", "relative_error"):
        v = np.array([out["hands"][h][key] for h in hs])
        out[f"rho_{key}_Dc"] = {"rho": da.spearman(dc, v), "ci95": boot(dc, v), "p_one_sided": permutation_p(dc, v)}
    out["n_hands"] = len(hs)
    name = "study3" if spec["base"] == "A" else "grid"
    (da.ROOT / f"conclusion_{name}.json").write_text(json.dumps(out, indent=1))
    for h in hs:
        r = out["hands"][h]
        print(f"{h:22s} h {r['thickness_mm']} Dc {feats[h]['contact']:.2f} n {r['pairs']:3d} true "
              f"{r['true_change_pct']:+5.0f}% measured {r['measured_change_pct']:+5.0f}% reversed "
              f"{100 * r['reversed_share']:3.0f}%")
    print({k: v for k, v in out.items() if k.startswith("rho_")})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
