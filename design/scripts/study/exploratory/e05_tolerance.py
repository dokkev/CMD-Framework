#!/usr/bin/env python3
"""E05: do feasible assemblies stay feasible under small geometric perturbations?

Each matched reference carries a module at its policy depths (feasible at
120 deg in Study 1). Every design variable except the tilt then gets
independent Gaussian noise of sigma = 0.1, 0.2 and 0.4 mm; the tilt gets
sigma = 0.5 deg. There are 200 draws per (reference, module, sigma), from a
fixed seed. The depths are not perturbed: after each draw the mounting policy re-seats
each pad on the perturbed hardware, as an assembled module would sit. The
finger-length spec range is widened to 60-120 mm (a spec, not a physical
check).
The linkage's link-length grid is refined from 0.1 to 0.001 mm, so that the
noise is not rounded away. The draws are Monte Carlo samples of a model, not
hardware replicates.
"""
from __future__ import annotations

import dataclasses
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
from core.optimization.mounting import mounting  # noqa: E402

MODULES = ("baseline", "thick", "asym_long_distal")
SIGMAS = (0.1, 0.2, 0.4)
DRAWS = 200
TILT_SIGMA = 0.5
FLEXION = 120.0
SEED = 20261107
_STATE = {}


def _init():
    _STATE["refs"] = common.references()
    _STATE["catalog"] = common.catalog()


def prepared(ref, modules):
    x, config = common.carried(ref, modules)
    if x is None:
        return None, None
    goal = common.goal_of(config)
    # the finger-length range is a design spec, not a physical check: a perturbed finger may be 90.03 mm
    config = common.with_goal(config, dataclasses.replace(goal, extended_length_bounds_mm=(60.0, 120.0)))
    config = common.shared.with_required_flexion(config, FLEXION)
    if hasattr(config, "link_length_resolution_mm"):
        config = dataclasses.replace(config, link_length_resolution_mm=0.001)
    return x, config


def _job(args):
    rid, module, sigma, index = args
    ref = _STATE["refs"][rid]
    found = common.mechanism(ref.mechanism)
    x, config = prepared(ref, _STATE["catalog"][module].modules)
    if x is None:
        return {"reference_id": rid, "module": module, "sigma_mm": sigma, "mounted": False}
    nominal = found.evaluate(x, config)
    rng = np.random.default_rng([SEED, index])
    names = found.variable_names
    bounds = getattr(config, "bounds", {}) or {}
    part = common.shared.partitions.partition(ref.mechanism)
    held = [n for n in names if n in bounds and n != "servo_tilt_deg" and bounds[n][0] == bounds[n][1]]
    held += [n for n in part.depths if n not in held]  # depths: re-seated by the mounting policy below
    # variables the formulation holds fixed (e.g. A and B on the link line) are not perturbed
    scale = np.array([0.0 if n in held else TILT_SIGMA if n == "servo_tilt_deg" else sigma for n in names])
    feasible, failing = 0, {}
    for _ in range(DRAWS):
        y = np.asarray(x, dtype=float) + rng.normal(0.0, 1.0, len(x)) * scale
        seated = mounting(ref.mechanism, tuple(y), config).depths_mm
        if seated is None:
            failing["no_depth"] = failing.get("no_depth", 0) + 1
            continue
        for slot, depth in zip(part.depths, seated):
            y[names.index(slot)] = depth
        e = found.evaluate(tuple(y), config)
        if e is None:
            failing["invalid"] = failing.get("invalid", 0) + 1
            continue
        if e.feasible:
            feasible += 1
        else:
            # name the first real violation: 100.0 is the INVALID sentinel the evaluators cascade
            real = {k: v for k, v in e.constraints.items() if 0.0 < v < 100.0}
            worst = max(real, key=real.get) if real else max(e.constraints, key=e.constraints.get)
            failing[worst] = failing.get(worst, 0) + 1
    report = found.report(nominal)
    return {"reference_id": rid, "module": module, "sigma_mm": sigma, "mounted": True,
            "nominal_feasible": bool(nominal.feasible), "draws": DRAWS, "feasible_draws": feasible,
            "p_feasible": feasible / DRAWS, "held_variables": ";".join(held), "nominal_clearance_mm": float(report.get("pad_clearance_mm", math.nan)),
            "worst_constraints": ";".join(f"{k}:{v}" for k, v in sorted(failing.items(), key=lambda kv: -kv[1]))}


def wilson(k, n, z=1.96):
    if not n:
        return (math.nan, math.nan)
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return centre - half, centre + half


def main():
    out = common.output("e05_tolerance")
    refs = common.references()
    jobs = [(rid, m, s, i) for i, (rid, m, s) in enumerate((rid, m, s) for rid in refs for m in MODULES for s in SIGMAS)]
    with ProcessPoolExecutor(8, initializer=_init) as pool:
        rows = list(pool.map(_job, jobs))
    for row in rows:
        if row.get("mounted"):
            row["ci_low"], row["ci_high"] = wilson(row["feasible_draws"], row["draws"])
    common.finish(out, rows, {"modules": MODULES, "sigmas_mm": SIGMAS, "draws": DRAWS, "tilt_sigma_deg": TILT_SIGMA,
                              "flexion_deg": FLEXION, "seed": SEED, "link_length_resolution_mm": 0.001,
                              "mount_policy": "policy depths re-seated after each perturbation",
                              "extended_length_bounds_mm": (60.0, 120.0)})
    for r in rows:
        print(r["reference_id"], r["module"], r["sigma_mm"], r.get("nominal_feasible"), r.get("p_feasible"),
              round(r.get("nominal_clearance_mm", math.nan), 2) if r.get("mounted") else "", (r.get("worst_constraints") or "")[:70])


if __name__ == "__main__":
    main()
