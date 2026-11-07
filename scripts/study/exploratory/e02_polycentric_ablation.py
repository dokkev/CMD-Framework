#!/usr/bin/env python3
"""E02: polycentric ablation of the thick-pad fold cost.

CoDe (matched reference, frozen) is compared with an idealized 2R
(``revolute_idealized``: no servo on the joint) built to the same external
morphology, with its pin swept around CoDe's extended ICR. Both pads are
20 mm long; the thickness sweeps 1-12 mm. The ideal 2R's pads are mounted
either at its own policy depth (no hardware, so 0) or at CoDe's policy depth
for that module, so that the pad faces sit where CoDe's sit. The ideal 2R has
no actuator packaging: it isolates the articulation, and it is optimistic.
"""
from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
from core.optimization.analysis.tradeoff import Modules  # noqa: E402
from core.optimization.linkage.mechanism import resolve_preset  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

THICKNESS = (1.0, 3.0, 5.0, 8.0, 10.0, 12.0)
PIN_OFFSETS = tuple(float(v) for v in range(-8, 9))
FLEXIONS = (90.0, 120.0, 150.0)
LENGTH = 20.0


def ideal_2r_config():
    values = read_config_file(common.ROOT / "scripts/study/2r_dd_tilt0.yaml")
    values.pop("mechanism")
    values.pop("output", None)
    found = mechanism("revolute_idealized")
    resolve_preset(values)
    values, _ = found.split_settings(values)
    return found, found.config_from_mapping(values)


def verdicts(found, x, config):
    row = {}
    evaluation = found.evaluate(x, config)
    report = found.report(evaluation)
    row["phi_max_deg"] = float(report["phi_max_deg"])
    row["stop_reason"] = report.get("stop_reason")
    row["coverage"] = float(report["pad_coverage"])
    for flexion in FLEXIONS:
        e = found.evaluate(x, common.shared.with_required_flexion(config, flexion))
        row[f"feasible_{flexion:g}"] = bool(e.feasible)
        row[f"failing_{flexion:g}"] = ";".join(sorted(k for k, v in e.constraints.items() if v > 0))
    return row


def main():
    refs = common.references()
    code = refs["code_matched"]
    linkage = mechanism("linkage")
    ideal, ideal_config = ideal_2r_config()
    part = common.shared.partitions.partition("revolute_idealized")
    code_part = common.shared.partitions.partition("linkage")
    out = common.output("e02_polycentric_ablation")
    rows = []
    for t in THICKNESS:
        modules = Modules(LENGTH, t, LENGTH, t)
        x_code, carried_code = common.carried(code, modules)
        base = {"thickness_mm": t}
        if x_code is None:
            rows.append({**base, "design": "code", "mounted": False})
            continue
        values = code_part.values(x_code)
        depths = (values[code_part.depths[0]], values[code_part.depths[1]])
        external = code_part.external(x_code, carried_code)
        rows.append({**base, "design": "code", "mounted": True, "pin_offset_mm": 0.0,
                     "proximal_depth_mm": depths[0], "distal_depth_mm": depths[1],
                     **verdicts(linkage, x_code, carried_code)})
        for mode in ("policy", "code_depth"):
            config = common.with_modules(ideal_config, modules)
            if mode == "code_depth":
                config = common.with_goal(config, dataclasses.replace(common.goal_of(config), mount_policy="searched"))
            for offset in PIN_OFFSETS:
                e = dict(external, articulation_mm=external["articulation_mm"] + offset)
                coordinates, template = part.template(e, "linkage", x_code, config)
                x = part.compose(e, coordinates, template, config)
                row = {**base, "design": f"ideal_2r_{mode}", "pin_offset_mm": offset}
                if x is None:
                    rows.append({**row, "mounted": False})
                    continue
                x = list(x)
                names = ideal.variable_names
                if mode == "code_depth":
                    x[names.index("proximal_drop_mm")], x[names.index("distal_drop_mm")] = depths
                v = dict(zip(names, x))
                rows.append({**row, "mounted": True, "proximal_depth_mm": v["proximal_drop_mm"],
                             "distal_depth_mm": v["distal_drop_mm"], **verdicts(ideal, tuple(x), config)})
    common.finish(out, rows, {"thickness_mm": THICKNESS, "pin_offsets_mm": PIN_OFFSETS, "pad_length_mm": LENGTH,
                              "flexions_deg": FLEXIONS},
                  icr0_mm=code_part.external(code.x, code.config)["articulation_mm"],
                  ablation="revolute_idealized: no joint servo; same E; pin swept about CoDe's extended ICR")
    for r in rows:
        if r.get("design") == "code" or r.get("pin_offset_mm") in (0.0,) or r.get("pin_offset_mm") is None:
            print({k: r.get(k) for k in ("thickness_mm", "design", "pin_offset_mm", "phi_max_deg", "feasible_120",
                                         "feasible_150", "proximal_depth_mm", "stop_reason")})


if __name__ == "__main__":
    main()
