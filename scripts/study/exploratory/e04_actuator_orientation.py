#!/usr/bin/env python3
"""E04: how much does the accommodation verdict depend on the actuator's orientation?

For each physical architecture, the matched reference is frozen (E and Q held)
and only ``servo_tilt_deg`` is swept from -30 to 120 deg. Each tilt is judged
on the Study 1 catalog at 120 deg with Study 1's ``assembly``; the policy
depths follow the servo. The 2R-DD and RCJ start from their 0 deg
references; their 90 deg references differ only in tilt, so they are points
of the same sweep. A tilt at which the bare reference (baseline modules) is
infeasible is recorded, not excluded.
"""
from __future__ import annotations

import dataclasses
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

TILTS = tuple(float(t) for t in range(-30, 121, 10))
BASES = ("code_matched", "2r_dd_tilt0_matched", "belt_2r_matched", "rcj_tilt0_matched")
FLEXION = 120.0
_STATE = {}


def _init():
    _STATE["refs"] = common.references()
    _STATE["catalog"] = common.shared.module_catalog(common.shared.read_config_file(common.ROOT / common.shared.CATALOG)) \
        if hasattr(common.shared, "read_config_file") else list(common.catalog().values())


def _job(args):
    rid, tilt = args
    ref = _STATE["refs"][rid]
    names = common.mechanism(ref.mechanism).variable_names
    x = list(ref.x)
    x[names.index("servo_tilt_deg")] = tilt
    # the tilt is the goal's actuation input (pip_servo_tilt_rad), excluded from the comparison rules
    goal = common.goal_of(ref.config)
    config = common.with_goal(ref.config, dataclasses.replace(
        goal, pip_servo_tilt_rad=math.radians(tilt), bounds={**goal.bounds, "servo_tilt_deg": (tilt, tilt)}))
    ref = dataclasses.replace(ref, x=tuple(x), config=config)
    rows = []
    for pair in _STATE["catalog"]:
        row = common.study1.assembly(ref, pair, [FLEXION])[0]
        rows.append({"reference_id": rid, "tilt_deg": tilt, "module_id": pair.module_id,
                     **{k: row.get(k) for k in ("mounted", "feasible", "failure", "phi_max_deg")}})
    return rows


def main():
    out = common.output("e04_actuator_orientation")
    jobs = [(rid, t) for rid in BASES for t in TILTS]
    with ProcessPoolExecutor(8, initializer=_init) as pool:
        rows = [r for chunk in pool.map(_job, jobs) for r in chunk]
    summary = []
    for rid in BASES:
        for t in TILTS:
            sel = [r for r in rows if r["reference_id"] == rid and r["tilt_deg"] == t]
            base = next(r for r in sel if r["module_id"] == "baseline")
            summary.append({"reference_id": rid, "tilt_deg": t, "carried": sum(bool(r["feasible"]) for r in sel),
                            "mounted": sum(bool(r["mounted"]) for r in sel), "pairs": len(sel),
                            "baseline_feasible": bool(base["feasible"]), "baseline_failure": base["failure"]})
    common.shared.write_csv(out / "summary.csv", summary)
    common.finish(out, rows, {"tilts_deg": TILTS, "bases": BASES, "flexion_deg": FLEXION})
    for s in summary:
        print(s["reference_id"], s["tilt_deg"], s["carried"], s["mounted"], s["baseline_feasible"], s["baseline_failure"][:50])


if __name__ == "__main__":
    main()
