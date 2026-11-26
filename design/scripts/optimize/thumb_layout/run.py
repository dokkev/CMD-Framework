#!/usr/bin/env python3
"""NSGA-II over the thumb layout (CMC servo x, z; fingertip extension) for one finger morphology and IP orientation.

    OPTIMIZE_WORKERS=3 python scripts/optimize/thumb_layout/run.py --hand h0 --orientation distal --seed 0 \
        [--envelope strict|relaxed] [--population 32] [--generations 25]
    python scripts/optimize/thumb_layout/run.py --hand h0 --baseline      # the prototype's thumb on this morphology

Objectives (both minimised): ``-functional_score`` (the functional Kapandji level with its tie-break) and
``-lever_ratio`` (min of the IP and MCP pad levers relative to the prototype's). The final front's designs are also
scored on All Hands Up's touch Kapandji, the pinch angle and the minimum graspable diameter (reported, not optimised).
Constraints ``G <= 0``: the IP and MCP pad levers no worse than 0.7x the prototype's (pad force
per torque), and the palm envelope (``Evaluator.envelope``: RAA sweep clear of the abduction servos, RO servo under the
hood's top and above the envelope's floor). The formulation: ``docs/experimental/thumb_layout_opt.md``.

Writes ``output/thumb_layout/<hand>/<envelope>/<orientation>/seed<N>/``: ``evaluations.csv`` (every design evaluated),
``pareto.csv`` (the final feasible non-dominated set) and ``run.json``.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.morphology.code_hand import load  # noqa: E402
from core.optimization.optimizer import OPTIMIZERS, OptimizerSettings, initial_population  # noqa: E402
from core.optimization.thumb_layout import Evaluator, ThumbDesign, lever_arms  # noqa: E402

HANDS = ROOT / "output" / "sim" / "study3" / "hands"
STEP_MM = 0.5
ENVELOPES = {
    # x, z bounds of the CMC servo frame [mm] and the floor depth below the palm's palmar face [mm]
    "strict": {"x": (-71.0, -49.0), "z": (-16.5, -3.5), "floor_mm": 20.0},
    "relaxed": {"x": (-80.0, -25.0), "z": (-46.0, -3.5), "floor_mm": 50.0},
}
EXTENSION_MM = (15.0, 45.0)
LEVER_RATIO = 0.7

_EVALUATOR: Evaluator | None = None


def _hand_path(hand: str) -> Path:
    return HANDS / f"cfl_q120_h3to7_{hand}.yaml"


def _init(hand: str, floor_mm: float) -> None:
    global _EVALUATOR
    parameters, _ = load(_hand_path(hand))
    _EVALUATOR = Evaluator(parameters, floor_mm=floor_mm)


def evaluate_row(args) -> dict:
    values, orientation, proto_levers = args
    x, z, e, *angles = values
    design = ThumbDesign(float(x), float(z), float(e), orientation, *(float(a) for a in angles))
    ev = _EVALUATOR.evaluate(design, touch=False, pinch=False)
    levers = lever_arms(design)
    row = {"cmc_x_mm": x, "cmc_z_mm": z, "extension_mm": e, "ip_orientation": orientation,
           **{f"base_{k}_deg": a for k, a in zip(("yaw", "pitch", "roll"), angles)},
           "functional_level": ev.functional_level, "functional_score": round(ev.functional_score, 4),
           "functional_reached": ev.functional_reached,
           "ip_lever_ratio": round(proto_levers["ip"] / levers["ip"], 4),
           "mcp_lever_ratio": round(proto_levers["mcp"] / levers["mcp"], 4),
           "lever_ratio": round(min(proto_levers["ip"] / levers["ip"], proto_levers["mcp"] / levers["mcp"]), 4),
           "mcp_to_ip_mm": design.mcp_to_ip_mm, "ee_mm": design.ee_mm,
           "levels": " ".join(f"K{k}" for k, r in ev.functional.items() if r.reached)}
    row.update({k: v for k, v in ev.envelope.items()})
    row["misses"] = " | ".join(f"K{k}: {r.blocker}" for k, r in ev.functional.items() if not r.reached)
    return row


def report_row(args) -> dict:
    """The reported metrics of a front design: the touch Kapandji, the pinch, the minimum graspable diameter."""
    values, orientation = args
    x, z, e, *angles = values
    design = ThumbDesign(float(x), float(z), float(e), orientation, *(float(a) for a in angles))
    ev = _EVALUATOR.evaluate(design, functional=False)
    grasp = _EVALUATOR.min_grasp_diameter(design)
    return {"touch_level": ev.kapandji_level, "touch_reached": ev.reached, "pinch_deg": round(min(ev.pinch_deg, 180.0), 2),
            "pinch_index_deg": round(ev.pinch["index"].residual, 2), "pinch_middle_deg": round(ev.pinch["middle"].residual, 2),
            "min_grasp_mm": grasp["min_diameter_mm"]}


def constraints(row: dict) -> list[float]:
    g = [LEVER_RATIO - row["ip_lever_ratio"], LEVER_RATIO - row["mcp_lever_ratio"],
         -row["raa_sweep_clearance_mm"], -row["ro_below_top_mm"], -row["ro_above_floor_mm"]]
    if "base_yaw_deg" in row:  # orientation free: the RO servo body itself must clear the palm's servos and fingers
        g.append(-row["ro_servo_clearance_mm"])
    return g


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--hand", default="h0", help="finger morphology: h0 (O), hr (R), hu (U)")
    parser.add_argument("--orientation", choices=("distal", "proximal"), default="distal")
    parser.add_argument("--envelope", choices=tuple(ENVELOPES), default="strict")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--population", type=int, default=32)
    parser.add_argument("--generations", type=int, default=25)
    parser.add_argument("--extension-max", type=float, default=EXTENSION_MM[1], help="upper bound of e [mm]")
    parser.add_argument("--free-orientation", type=float, default=0.0,
                        help="exploratory: base yaw/pitch/roll offsets free within +-this [deg] (beyond the original list)")
    parser.add_argument("--tag", default="", help="appended to the envelope folder name")
    parser.add_argument("--baseline", action="store_true", help="evaluate the prototype thumb and print it")
    args = parser.parse_args(argv)
    env = ENVELOPES[args.envelope]
    parameters, _ = load(_hand_path(args.hand))
    proto = ThumbDesign.prototype(parameters)
    proto_levers = lever_arms(proto)
    if args.baseline:
        _init(args.hand, env["floor_mm"])
        ev = _EVALUATOR.evaluate(proto)
        print(json.dumps(ev.summary(), indent=1, default=float))
        print(json.dumps(_EVALUATOR.min_grasp_diameter(proto), indent=1))
        return 0

    from pymoo.core.problem import Problem

    lo = np.array([env["x"][0], env["z"][0], EXTENSION_MM[0]])
    hi = np.array([env["x"][1], env["z"][1], args.extension_max])
    step = np.full(3, STEP_MM)
    if args.free_orientation:
        lo = np.concatenate((lo, np.full(3, -args.free_orientation)))
        hi = np.concatenate((hi, np.full(3, args.free_orientation)))
        step = np.concatenate((step, np.ones(3)))
    n_var = len(lo)
    steps = np.round((hi - lo) / step)
    workers = int(os.environ.get("OPTIMIZE_WORKERS") or 3)
    pool = Pool(workers, initializer=_init, initargs=(args.hand, env["floor_mm"]))
    cache: dict[tuple, dict] = {}
    rows: list[dict] = []

    class ThumbProblem(Problem):
        def __init__(self):
            super().__init__(n_var=n_var, n_obj=2, n_ieq_constr=6 if args.free_orientation else 5,
                             xl=np.zeros(n_var), xu=steps)

        def _evaluate(self, x, out, *a, **k):
            designs = [tuple(np.round(lo + np.round(v) * step, 3)) for v in x]
            todo = [d for d in dict.fromkeys(designs) if d not in cache]
            for d, row in zip(todo, pool.map(evaluate_row, [(d, args.orientation, proto_levers) for d in todo])):
                cache[d] = row
                rows.append(row)
            out["F"] = np.array([[-cache[d]["functional_score"], -cache[d]["lever_ratio"]] for d in designs])
            out["G"] = np.array([constraints(cache[d]) for d in designs])

    problem = ThumbProblem()
    settings = OptimizerSettings("nsga2", args.population, args.generations, args.seed)
    warm = []
    proto_x = np.array([proto.cmc_x_mm, proto.cmc_z_mm, proto.extension_mm, 0.0, 0.0, 0.0][:n_var])
    if args.orientation == "distal" and np.all((lo <= proto_x) & (proto_x <= hi)):
        warm = [np.round((proto_x - lo) / step)]
    initial = initial_population(problem, np.array(warm).reshape(-1, n_var), settings)
    start = time.time()
    result = OPTIMIZERS["nsga2"].minimize(problem, initial, settings)
    out = ROOT / "output" / "thumb_layout" / args.hand / (args.envelope + args.tag) / args.orientation / f"seed{args.seed}"
    out.mkdir(parents=True, exist_ok=True)
    keys = list(rows[0])
    with (out / "evaluations.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        w.writerows(rows)
    pareto = [cache[tuple(np.round(lo + np.round(v) * step, 3))] for v in result.x]
    pareto = [r for r in pareto if max(constraints(r)) <= 0.0]
    pareto = sorted({tuple(v for k, v in r.items() if k.startswith(("cmc", "ext", "base"))): r for r in pareto}.values(),
                    key=lambda r: -r["functional_score"])
    keyed = [tuple(v for k, v in r.items() if k.startswith(("cmc", "ext", "base"))) for r in pareto]
    for r, extra in zip(pareto, pool.map(report_row, [(k, args.orientation) for k in keyed])):
        r.update(extra)
    pool.close()
    with (out / "pareto.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, list(pareto[0]) if pareto else keys)
        w.writeheader()
        w.writerows(pareto)
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    (out / "run.json").write_text(json.dumps({
        "hand": args.hand, "orientation": args.orientation, "envelope": {args.envelope: env},
        "extension_mm": (EXTENSION_MM[0], args.extension_max), "free_orientation_deg": args.free_orientation,
        "steps": step.tolist(), "settings": settings.to_json(),
        "evaluations": len(rows), "seconds": round(time.time() - start, 1), "commit": commit,
        "prototype": proto.__dict__, "prototype_levers_mm": proto_levers}, indent=1) + "\n")
    print(f"{out}: {len(rows)} evaluations in {time.time() - start:.0f} s; Pareto:")
    for r in pareto:
        angles = " ".join(f"{r[k]:+.0f}" for k in ("base_yaw_deg", "base_pitch_deg", "base_roll_deg") if k in r)
        print(f"  x {r['cmc_x_mm']:7.2f} z {r['cmc_z_mm']:7.2f} e {r['extension_mm']:5.1f} {angles} F-K{r['functional_level']} {r['functional_score']:.3f} "
              f"n {r['functional_reached']}  levers {r['ip_lever_ratio']:.2f}/{r['mcp_lever_ratio']:.2f}  touch K{r['touch_level']} "
              f"n {r['touch_reached']}  pinch {r['pinch_deg']:5.1f}  grasp {r['min_grasp_mm']}  | {r['levels']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
