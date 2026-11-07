#!/usr/bin/env python3
"""E09: the hidden transmission and kinematic change from morphology adaptation.

For every Study 2 (bounded) target a reference had to adapt to (its final
design in ``summary_best.csv``), the adapted design is compared with the
reference carrying the same target module. Both use the Study 3 adapter
(existing FK), so the module's own effect cancels and only the morphology
change remains. The quantities, over phi = 0, 30, 60, 90 and 120 deg:
- the servo torque per unit normal force at the distal face centre, PIP
  column (|J^T n|, m), and at the proximal face centre, MCP column;
- the path of the distal face's outer end, a fingertip proxy;
- phi_max.
Reported: the largest relative change of each lever, and the largest path
deviation divided by the reference finger length, against the Study 2
eps_E and eps_Q.
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
from core.optimization.analysis.transmission import Transmission  # noqa: E402

SUMMARY = common.ROOT / "output/study/study2_morphology_adaptation/summary_best.csv"
PHIS = (0.0, 30.0, 60.0, 90.0, 120.0)


def levers(name, x, config):
    t = Transmission(name, x, config)
    distal, proximal, tip = [], [], []
    for phi in PHIS:
        rad = np.radians(phi)
        _, n_d = t.contact(rad, "distal", 0.5)
        _, n_p = t.contact(rad, "proximal", 0.5)
        distal.append(abs(float(t.jacobian(rad, "distal", 0.5)[:, 1] @ n_d)))
        proximal.append(abs(float(t.jacobian(rad, "proximal", 0.5)[:, 0] @ n_p)))
        tip.append(t.contact(rad, "distal", 0.0)[0])
    return np.array(distal), np.array(proximal), np.array(tip)


def main():
    refs, cat = common.references(), common.catalog()
    out = common.output("e09_hidden_transmission")
    rows = []
    with SUMMARY.open() as stream:
        best = list(csv.DictReader(stream))
    for row in best:
        if row["status"] in ("frozen_feasible", "not_found") or not row["final_x"]:
            continue
        ref = refs[row["reference_id"]]
        modules = cat[row["target"]].modules
        x_ref, config = common.carried(ref, modules)
        base = {"reference_id": ref.reference_id, "target": row["target"], "upper_bound": ref.upper_bound,
                "eps_E": float(row["final_external_cost"]), "eps_Q": float(row["final_mechanism_cost"] or 0.0)}
        if x_ref is None:  # the reference cannot even seat this module: compare against its baseline-module seat
            x_ref, config = common.carried(ref, cat["baseline"].modules)
            base["reference_module"] = "baseline"
        else:
            base["reference_module"] = row["target"]
        x_new = tuple(float(v) for v in row["final_x"].split(";"))
        try:
            d0, p0, tip0 = levers(ref.mechanism, x_ref, config)
            d1, p1, tip1 = levers(ref.mechanism, x_new, common.with_modules(ref.config, modules))
        except ValueError as error:
            rows.append({**base, "error": str(error)})
            continue
        length = common.shared.partitions.partition(ref.mechanism).external(ref.x, ref.config)["finger_length_mm"]
        rows.append({**base,
                     "distal_lever_ref_mm": float(d0.mean() * 1000), "distal_lever_new_mm": float(d1.mean() * 1000),
                     "distal_lever_change": float(np.max(np.abs(d1 - d0) / d0)),
                     "proximal_lever_change": float(np.max(np.abs(p1 - p0) / p0)),
                     "tip_path_deviation": float(np.max(np.linalg.norm(tip1 - tip0, axis=1)) * 1000 / length),
                     "tip_path_deviation_minus_eps_E": float(np.max(np.linalg.norm(tip1 - tip0, axis=1)) * 1000
                                                             / length - float(row["final_external_cost"]))})
    common.finish(out, rows, {"phis_deg": PHIS, "summary": str(SUMMARY.relative_to(common.ROOT)),
                              "lever": "|J^T n| at face centres: PIP column (distal), MCP column (proximal)"})
    for r in rows:
        print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})


if __name__ == "__main__":
    main()
