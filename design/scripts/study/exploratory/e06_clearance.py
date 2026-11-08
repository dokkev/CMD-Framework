#!/usr/bin/env python3
"""E06: does the Study 1 ranking survive a non-zero body clearance?

The shared ``packaging.body_clearance_mm`` (0 in the study configs: touching
allowed) is raised to 0.25, 0.5, 1.0 and 1.5 mm for every reference at once.
The Study 1 catalog is then re-judged at 120 and 150 deg with Study 1's
policy depths, then each pad lowered (0.05 mm steps, at most 2 mm) until the
evaluator's own seat checks pass: the policy does not apply the clearance
where the evaluators do, so without this the policy depth reads as an
unmountable INVALID design (registry revision). Clearance enters every
check that reads ``body_clearance_mm`` in the evaluators and the mounting
policy (listed in the report).
"""
from __future__ import annotations

import dataclasses
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

CLEARANCES = (0.0, 0.25, 0.5, 1.0, 1.5)
FLEXIONS = (120.0, 150.0)
_STATE = {}


def _init():
    _STATE["refs"] = common.references()
    _STATE["catalog"] = list(common.catalog().values())


STATIC = ("proximal_support_mm", "distal_support_mm", "pad_drop_mm")
"""The evaluators' static checks on a pad's seat (its support room and drop)."""
SEAT_STEP, SEAT_MAX = 0.05, 2.0


VARIANTS = ("all", "pins_exempt")
"""``pins_exempt``: pin housings shrunk by the clearance (diameter - 2c), so the
clearance applies everywhere except at pin housings: it isolates the single
static pin contact that E05 found in several references."""


def seated(ref, pair, clearance, variant="all"):
    """The policy depths, then both pads lowered in 0.05 mm steps until the
    evaluator's own seat checks pass (at most 2 mm). Needed because the policy
    does not apply the body clearance where the evaluators do (see report);
    the same rule for every mechanism."""
    found = common.mechanism(ref.mechanism)
    part = common.shared.partitions.partition(ref.mechanism)
    changes = {"body_clearance_mm": clearance}
    if variant == "pins_exempt":
        packaging = common.goal_of(ref.config).packaging
        changes["pivot_clearance_diameter_mm"] = packaging.pivot_clearance_diameter_mm - 2.0 * clearance
        for pin in ("a", "ap", "b", "bp"):  # the linkage sizes each pin housing itself
            own = getattr(packaging, f"{pin}_pivot_diameter_mm", None)
            if own is not None:
                changes[f"{pin}_pivot_diameter_mm"] = own - 2.0 * clearance
    ref = dataclasses.replace(ref, config=common.with_packaging(ref.config, **changes))
    x, config = common.carried(ref, pair.modules)
    if x is None:
        return None, None, None
    config = common.with_goal(config, dataclasses.replace(common.goal_of(config), mount_policy="searched"))
    slots = [found.variable_names.index(n) for n in part.depths]
    for step in range(int(SEAT_MAX / SEAT_STEP) + 1):
        y = list(x)
        for i in slots:
            y[i] += step * SEAT_STEP
        e = found.evaluate(tuple(y), config)
        if e is not None and all(e.constraints.get(n, 0.0) <= 0.0 for n in STATIC):
            return tuple(y), config, step * SEAT_STEP
    return None, config, None


def _job(args):
    rid, clearance, variant = args
    ref = _STATE["refs"][rid]
    found = common.mechanism(ref.mechanism)
    rows = []
    for pair in _STATE["catalog"]:
        x, config, lowered = seated(ref, pair, clearance, variant)
        for flexion in FLEXIONS:
            row = {"reference_id": rid, "variant": variant, "clearance_mm": clearance, "module_id": pair.module_id,
                   "required_flexion_deg": flexion, "lowered_mm": lowered}
            if x is None:
                rows.append({**row, "mounted": False, "feasible": False, "failure": "not_mounted"})
                continue
            e = found.evaluate(x, common.shared.with_required_flexion(config, flexion))
            mounting = [n for n in common.study1.MOUNTING if e.constraints.get(n, 0.0) > 0.0]
            real = {k: v for k, v in e.constraints.items() if v > 0.0}
            rows.append({**row, "mounted": not mounting, "feasible": bool(e.feasible) and not mounting,
                         "failure": ";".join(sorted(real, key=lambda k: (real[k] >= 100.0, -real[k])))})
    return rows


def main():
    out = common.output("e06_clearance")
    refs = common.references()
    jobs = [(rid, c, v) for rid in refs for c in CLEARANCES for v in VARIANTS]
    with ProcessPoolExecutor(8, initializer=_init) as pool:
        rows = [r for chunk in pool.map(_job, jobs) for r in chunk]
    summary = []
    for rid in refs:
        for v in VARIANTS:
          for c in CLEARANCES:
            for f in FLEXIONS:
                sel = [r for r in rows if r["reference_id"] == rid and r["clearance_mm"] == c
                       and r["required_flexion_deg"] == f and r["variant"] == v]
                summary.append({"reference_id": rid, "variant": v, "clearance_mm": c, "flexion_deg": f,
                                "carried": sum(bool(r["feasible"]) for r in sel),
                                "mounted": sum(bool(r["mounted"]) for r in sel)})
    common.shared.write_csv(out / "summary.csv", summary)
    common.finish(out, rows, {"clearances_mm": CLEARANCES, "flexions_deg": FLEXIONS, "variants": VARIANTS,
                              "seat_step_mm": SEAT_STEP, "seat_max_mm": SEAT_MAX})
    for rid in refs:
        for v in VARIANTS:
            print(rid, v, [(s["clearance_mm"], s["flexion_deg"], s["carried"]) for s in summary
                           if s["reference_id"] == rid and s["variant"] == v])


if __name__ == "__main__":
    main()
