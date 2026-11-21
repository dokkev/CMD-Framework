# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Experimental: the separability measures from ``separability_quasistatic.py`` runs (latest run per hand, shape,
radius).

    python3 scripts/experimental/separability_analysis.py

Per condition (cylinder radius x O's contact flexion q0) and hand, from the fingers' mean distal-pad force:
- J1: commanded flexion at first distal contact (force > 0.05 N) [deg];
- J2: distal force at commanded 110 and 120 deg [N];
- J3: grasp stiffness, the slope of the distal force over the 5 deg after first contact [N/deg].

Then, per measure: the interface effect E(hand) = J(crowned) - J(flat); the thickness artefact
T(hand) = J(hand, flat) - J(O, flat) for R and U (zero if the thicker pads changed nothing the object meets: the
contact faces are the same); the separability S(hand) = E(hand) - E(O) (zero if the interface effect measured after
the redesign equals the one measured on O). Prints a table and writes ``logs/experimental/separability/summary.json``.
"""

from __future__ import annotations

import glob
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2] / "logs" / "experimental" / "separability"
THRESHOLD_N = 0.05


def load() -> dict:
    runs = {}
    for f in sorted(glob.glob(str(ROOT / "sep_*.json"))):
        d = json.loads(Path(f).read_text())
        runs[(d["hand"], d["shape"], d["radius_mm"])] = d  # sorted by time: the latest wins
    return runs


def measures(d: dict) -> list[dict]:
    q = np.array([r["q_cmd_deg"] for r in d["rows"]])
    force = np.array([r["pad_force_n"] for r in d["rows"]])  # (steps, envs, pads)
    out = []
    for e, q0 in enumerate(d["q0_deg"]):
        distal = force[:, e, 1::2].mean(axis=1)
        steps = {}
        for qq, f in zip(q, distal):  # last value at each commanded flexion (end of its hold)
            steps[qq] = f
        qs = np.array(sorted(steps))
        fs = np.array([steps[v] for v in qs])
        touching = np.nonzero(fs > THRESHOLD_N)[0]
        j1 = float(qs[touching[0]]) if len(touching) else float("nan")
        window = (qs >= j1) & (qs <= j1 + 5.0)
        j3 = float(np.polyfit(qs[window], fs[window], 1)[0]) if window.sum() >= 3 else float("nan")
        out.append({"q0": q0, "J1_contact_deg": j1, "J2_force_110_n": float(np.interp(110.0, qs, fs)),
                    "J2_force_120_n": float(fs[-1]), "J3_stiffness_n_per_deg": j3,
                    "loop_mm_max": float(max(r["loop_mm"][e] for r in d["rows"]))})
    return out


def main() -> int:
    runs = load()
    keys = ("J1_contact_deg", "J2_force_110_n", "J2_force_120_n", "J3_stiffness_n_per_deg")
    table = {}
    for (hand, shape, radius), d in runs.items():
        for m in measures(d):
            table[(hand, shape, radius, m["q0"])] = m
    conds = sorted({(r, q) for (_, _, r, q) in table})
    summary = {"conditions": len(conds), "measures": {}}
    print(f"{len(conds)} conditions (radius, q0); loop residual max {max(m['loop_mm_max'] for m in table.values()):.3f} mm")
    for key in keys:
        rows = []
        for r, q0 in conds:
            get = lambda h, s: table.get((h, s, r, q0), {}).get(key, np.nan)  # noqa: E731
            e = {h: get(h, "crowned") - get(h, "flat") for h in "ORU"}
            rows.append({"radius": r, "q0": q0, "J_O_flat": get("O", "flat"),
                         "E_O": e["O"], "E_R": e["R"], "E_U": e["U"],
                         "T_R": get("R", "flat") - get("O", "flat"), "T_U": get("U", "flat") - get("O", "flat"),
                         "S_R": e["R"] - e["O"], "S_U": e["U"] - e["O"]})
        a = {k: np.array([row[k] for row in rows], dtype=float) for k in rows[0] if k not in ("radius", "q0")}
        med = lambda x: float(np.nanmedian(np.abs(x)))  # noqa: E731
        mx = lambda x: float(np.nanmax(np.abs(x)))  # noqa: E731
        summary["measures"][key] = {"rows": rows, **{f"median_abs_{k}": med(v) for k, v in a.items()},
                                    **{f"max_abs_{k}": mx(v) for k, v in a.items()}}
        print(f"\n{key}: median |.| (max |.|) over conditions")
        print(f"  J(O, flat) {med(a['J_O_flat']):.2f}   interface effect E_O {med(a['E_O']):.3f} ({mx(a['E_O']):.3f}),"
              f" E_R {med(a['E_R']):.3f}, E_U {med(a['E_U']):.3f}")
        print(f"  thickness artefact T_R {med(a['T_R']):.3f} ({mx(a['T_R']):.3f})   T_U {med(a['T_U']):.3f} ({mx(a['T_U']):.3f})")
        print(f"  separability S_R {med(a['S_R']):.3f} ({mx(a['S_R']):.3f})   S_U {med(a['S_U']):.3f} ({mx(a['S_U']):.3f})")
    (ROOT / "summary.json").write_text(json.dumps(summary, indent=1, default=float) + "\n")
    print(ROOT / "summary.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
