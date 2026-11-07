#!/usr/bin/env python3
"""E02 follow-up: does the effective pivot's depth relative to the contact
faces explain the thick-pad fold result?

CoDe's crossed four-bar ICR (the crossing of A'B and AB') is traced over
0-150 deg. Fixed-pivot designs are then given a counterfactual "palmar pivot":
both pads are raised by h mm toward the joint axis (negative mounting drop,
``mount_policy: searched``). This is equivalent to moving the axis h mm toward
the contact surface. The test covers the pin-only ideal 2R and the physical
2R-DD (servo on the axis) and belt 2R (pulley on the axis). The negative
drop is a hypothetical construction: it puts the segment line inside the pad
volume.
"""
from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
import e02_polycentric_ablation as e02  # noqa: E402
from core.optimization.analysis.tradeoff import Modules  # noqa: E402
from core.optimization.analysis.transmission import Transmission  # noqa: E402
from core.optimization.mechanism import mechanism  # noqa: E402

THICKNESS = (5.0, 8.0, 10.0, 12.0)
RAISE = tuple(float(v) for v in range(0, 9))
DROPS = {"revolute_idealized": ("proximal_drop_mm", "distal_drop_mm"),
         "revolute": ("proximal_drop_mm", "distal_drop_mm"),
         "revolute_parallel": ("proximal_drop_mm", "distal_drop_mm")}


def icr_path(code, modules):
    x, config = common.carried(code, modules)
    finger = Transmission("linkage", x, config).finger
    a = np.array([finger.joint.a.x_mm, finger.joint.a.z_mm])
    ap = np.array([finger.joint.ap.x_mm, finger.joint.ap.z_mm])
    rows = []
    for phi in range(0, 151, 5):
        s = finger.sweep_pip([np.radians(phi)])
        if len(s.q_pip_rad) != 1:
            break
        b, bp = s.b[0], s.bp[0]
        d1, d2 = b - ap, bp - a
        t = np.linalg.solve(np.array([d1, -d2]).T, a - ap)
        icr = ap + t[0] * d1
        rows.append({"phi_deg": phi, "icr_x_mm": float(icr[0]), "icr_z_mm": float(icr[1])})
    return rows


def raised(found, x, config, h):
    x = list(x)
    for name in DROPS[found.name]:
        x[found.variable_names.index(name)] = -h
    return tuple(x)


def main():
    refs = common.references()
    code = refs["code_matched"]
    out = common.output("e02b_palmar_pivot")
    ideal, ideal_config = e02.ideal_2r_config()
    ideal_part, code_part = (common.shared.partitions.partition(n) for n in ("revolute_idealized", "linkage"))
    rows = []
    for t in THICKNESS:
        modules = Modules(20.0, t, 20.0, t)
        x_code, carried_code = common.carried(code, modules)
        external = code_part.external(x_code, carried_code)
        cases = []
        config = common.with_modules(ideal_config, modules)
        coordinates, template = ideal_part.template(external, "linkage", x_code, config)
        cases.append(("ideal_2r_at_icr0", ideal, ideal_part.compose(external, coordinates, template, config), config))
        for rid in ("2r_dd_tilt90_matched", "belt_2r_matched"):
            x_ref, config_ref = common.carried(refs[rid], modules)
            cases.append((rid, mechanism(refs[rid].mechanism), x_ref, config_ref))
        for name, found, x, config in cases:
            if x is None:
                rows.append({"thickness_mm": t, "design": name, "raise_mm": None, "mounted": False})
                continue
            searched = common.with_goal(config, dataclasses.replace(common.goal_of(config), mount_policy="searched"))
            for h in RAISE:
                xr = raised(found, x, searched, h)
                row = {"thickness_mm": t, "design": name, "raise_mm": h, "mounted": True}
                for flexion in (120.0, 150.0):
                    e = found.evaluate(xr, common.shared.with_required_flexion(searched, flexion))
                    row[f"feasible_{flexion:g}"] = bool(e.feasible)
                    row[f"failing_{flexion:g}"] = ";".join(sorted(k for k, v in e.constraints.items() if v > 0))
                row["phi_max_deg"] = float(found.report(found.evaluate(xr, searched))["phi_max_deg"])
                rows.append(row)
    path = icr_path(code, Modules(20.0, 8.0, 20.0, 8.0))
    common.shared.write_csv(out / "code_icr_path.csv", path)
    common.finish(out, rows, {"thickness_mm": THICKNESS, "raise_mm": RAISE},
                  counterfactual="negative drops raise the pads toward the axis: a hypothetical palmar pivot")
    for r in rows:
        print({k: r.get(k) for k in ("thickness_mm", "design", "raise_mm", "feasible_120", "feasible_150",
                                     "failing_150")})
    print(path[::6])


if __name__ == "__main__":
    main()
