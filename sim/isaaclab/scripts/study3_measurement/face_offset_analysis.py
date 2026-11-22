# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3 measurement, exploratory (v16 review): which part of Delta_contact predicts the error?

    python3 scripts/study3_measurement/face_offset_analysis.py

Delta_contact split into the distal face's offset along its normal and the slide of the same face point along it
(``design/output/study/exploratory/e20_face_offset/summary.csv``). Spearman rho of each with the error, with a
hand-bootstrap interval, in every Study 3 setting. Writes ``logs/experimental/two_pad/face_offset.json``.
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dose_analysis as da  # noqa: E402
from invariants import boot  # noqa: E402

E20 = da.ROOT.parents[4] / "design" / "output" / "study" / "exploratory" / "e20_face_offset" / "summary.csv"
SETTINGS = {"friction": ("dose_A_all.json", "E_rel_median"), "stiffness": ("comp.json", "E_median"),
            "grid": ("dose_A-grid_expanded.json", "E_rel_median"), "soft": ("padk_matched.json", "E_median"),
            "torque": ("dose_C_all.json", "E_rel_median")}


def sim_name(stem: str) -> str:
    """e20's hand stem (``h3to7_hr``) to the simulation's hand name (``R``)."""
    if stem.startswith("h3to7_"):
        rest = stem.removeprefix("h3to7_")
        return {"hr": "R", "hu": "U"}.get(rest, rest)
    return stem


def main() -> int:
    with E20.open(encoding="utf-8") as stream:
        geo = {sim_name(r["hand"]): {k: float(r[k]) for k in ("contact_mm", "normal_mm", "tangential_mm")}
               for r in csv.DictReader(stream)}
    out = {}
    for name, (file, key) in SETTINGS.items():
        recs = json.loads((da.ROOT / file).read_text())["hands"]
        hs = [h for h in recs if h in geo and recs[h].get(key) is not None]
        e = np.array([recs[h][key] for h in hs])
        out[name] = {"n": len(hs)}
        for f in ("contact_mm", "normal_mm", "tangential_mm"):
            x = np.array([geo[h][f] for h in hs])
            out[name][f] = {"rho": da.spearman(x, e), "ci95": boot(x, e)}
        print(f"{name:9s} n {len(hs):2d} " + "  ".join(
            f"{f[:-3]} {out[name][f]['rho']:+.2f} [{out[name][f]['ci95'][0]:+.2f}, {out[name][f]['ci95'][1]:+.2f}]"
            for f in ("contact_mm", "normal_mm", "tangential_mm")))
    (da.ROOT / "face_offset.json").write_text(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
