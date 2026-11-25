# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3 measurement (problem_formulation A.6.4): the fixed-axis contrast family in Study 3 (report/study3/study3.md, 14th
experiment).

    python3 scripts/study3_measurement/fa_analysis.py

The error of each fixed-axis redesign (5, 7, 9 mm pads, the 3 mm finger's skeleton kept exactly) against the 3 mm
fixed-axis finger, under the reused flexion command (runs tagged ``fa``), computed as in ``dose_analysis.py``: the
median over placements and directions of |Delta(H) - Delta(O)| / |Delta(O)|, Delta the change of the limit load from
mu 1.3 to 2.0, over placements where all four runs pass Gate 0. Writes ``logs/study3/two_pad/fa.json``.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dose_analysis as da  # noqa: E402

REF, HANDS, COND = "fa_h3", ("fa_h5", "fa_h7", "fa_h9"), "A-fa"


def main() -> int:
    out = {"reference": REF, "condition": COND, "hands": {}}
    for hand in HANDS:
        dh = float(hand[-1]) - 3.0
        rel, force, envs, fails = [], [], 0, 0
        for r in (15.0, 20.0):
            runs = {(h, m): da.latest(h, m, COND, r) for h in (REF, hand) for m in (1.3, 2.0)}
            if any(v is None for v in runs.values()):
                print(f"{hand} r{r:g}: runs missing")
                continue
            key = da.env_key
            by = {hm: {key(e): e for e in d["envs"]} for hm, d in runs.items()}
            for m in (1.3, 2.0):
                envs += len(by[(hand, m)])
                fails += sum(not e["gate0"]["pass"] for e in by[(hand, m)].values())
            for k in by[(REF, 1.3)]:
                es = {hm: by[hm].get(k) for hm in by}
                if any(e is None or not e["gate0"]["pass"] or e["limit"]["f_limit_n"] is None for e in es.values()):
                    continue
                d_o = es[(REF, 2.0)]["limit"]["f_limit_n"] - es[(REF, 1.3)]["limit"]["f_limit_n"]
                d_h = es[(hand, 2.0)]["limit"]["f_limit_n"] - es[(hand, 1.3)]["limit"]["f_limit_n"]
                if abs(d_o) < 1e-9:
                    continue
                rel.append(abs(d_h - d_o) / abs(d_o))
                force.append(es[(hand, 1.3)]["pre"]["n_distal_total"] / max(es[(REF, 1.3)]["pre"]["n_distal_total"], 1e-9))
        rec = {"thickness_mm": dh + 3.0, "D_c_mm": 2.0 * math.sin(math.radians(60.0)) * dh, "D_s_mm": 0.0,
               "pairs": len(rel), "E_rel_median": float(np.median(rel)) if rel else None,
               "E_rel_iqr": [float(np.percentile(rel, 25)), float(np.percentile(rel, 75))] if rel else None,
               "distal_force_ratio_median": float(np.median(force)) if force else None,
               "gate0_fail_share": fails / max(envs, 1)}
        out["hands"][hand] = rec
        print(f"{hand}: D_c {rec['D_c_mm']:.2f} mm, D_s 0 | pairs {rec['pairs']} | E "
              + ("-" if not rel else f"{100 * rec['E_rel_median']:.1f}% (IQR {100 * rec['E_rel_iqr'][0]:.0f}-"
                 f"{100 * rec['E_rel_iqr'][1]:.0f}%) | N_d/N_d(ref) {rec['distal_force_ratio_median']:.2f}")
              + f" | Gate 0 fail {100 * rec['gate0_fail_share']:.0f}%")
    path = da.ROOT / "fa.json"
    path.write_text(json.dumps(out, indent=1))
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
