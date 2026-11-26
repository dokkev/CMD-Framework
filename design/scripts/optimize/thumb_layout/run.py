#!/usr/bin/env python3
"""NSGA-II over the thumb layout (CMC servo x, z; fingertip extension) for one finger morphology and IP orientation.

    OPTIMIZE_WORKERS=3 python scripts/optimize/thumb_layout/run.py --hand h0 --orientation distal --seed 0 \
        [--envelope strict|relaxed] [--population 32] [--generations 25]
    python scripts/optimize/thumb_layout/run.py --hand h0 --baseline      # the prototype's thumb on this morphology

Objectives (both minimised): ``-kapandji_score`` and ``pinch_deg`` (the worse of the thumb–index and thumb–middle best
angles from facing). Constraints ``G <= 0``: the IP and MCP pad levers no worse than 0.7x the prototype's (pad force
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
    x, z, e, orientation, proto_levers = args
    design = ThumbDesign(float(x), float(z), float(e), orientation)
    ev = _EVALUATOR.evaluate(design)
    levers = lever_arms(design)
    row = {"cmc_x_mm": x, "cmc_z_mm": z, "extension_mm": e, "ip_orientation": orientation,
           "kapandji_score": round(ev.kapandji_score, 4), "reached": ev.reached,
           "pinch_deg": round(min(ev.pinch_deg, 180.0), 3),
           "pinch_index_deg": round(ev.pinch["index"].residual, 3), "pinch_middle_deg": round(ev.pinch["middle"].residual, 3),
           "ip_lever_ratio": round(proto_levers["ip"] / levers["ip"], 4),
           "mcp_lever_ratio": round(proto_levers["mcp"] / levers["mcp"], 4),
           "mcp_to_ip_mm": design.mcp_to_ip_mm, "ee_mm": design.ee_mm,
           "targets": " ".join(k for k, r in ev.kapandji.items() if r.reached)}
    row.update({k: v for k, v in ev.envelope.items()})
    row["angles"] = " ".join(f"{k}:{r.angle_deg:.1f}" for k, r in ev.kapandji.items())
    return row


def constraints(row: dict) -> list[float]:
    return [LEVER_RATIO - row["ip_lever_ratio"], LEVER_RATIO - row["mcp_lever_ratio"],
            -row["raa_sweep_clearance_mm"], -row["ro_below_top_mm"], -row["ro_above_floor_mm"]]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--hand", default="h0", help="finger morphology: h0 (O), hr (R), hu (U)")
    parser.add_argument("--orientation", choices=("distal", "proximal"), default="distal")
    parser.add_argument("--envelope", choices=tuple(ENVELOPES), default="strict")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--population", type=int, default=32)
    parser.add_argument("--generations", type=int, default=25)
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
        return 0

    from pymoo.core.problem import Problem

    lo = np.array([env["x"][0], env["z"][0], EXTENSION_MM[0]])
    hi = np.array([env["x"][1], env["z"][1], EXTENSION_MM[1]])
    steps = np.round((hi - lo) / STEP_MM)
    workers = int(os.environ.get("OPTIMIZE_WORKERS") or 3)
    pool = Pool(workers, initializer=_init, initargs=(args.hand, env["floor_mm"]))
    cache: dict[tuple, dict] = {}
    rows: list[dict] = []

    class ThumbProblem(Problem):
        def __init__(self):
            super().__init__(n_var=3, n_obj=2, n_ieq_constr=5, xl=np.zeros(3), xu=steps)

        def _evaluate(self, x, out, *a, **k):
            designs = [tuple(np.round(lo + np.round(v) * STEP_MM, 3)) for v in x]
            todo = [d for d in dict.fromkeys(designs) if d not in cache]
            for d, row in zip(todo, pool.map(evaluate_row, [(*d, args.orientation, proto_levers) for d in todo])):
                cache[d] = row
                rows.append(row)
            out["F"] = np.array([[-cache[d]["kapandji_score"], cache[d]["pinch_deg"]] for d in designs])
            out["G"] = np.array([constraints(cache[d]) for d in designs])

    problem = ThumbProblem()
    settings = OptimizerSettings("nsga2", args.population, args.generations, args.seed)
    warm = []
    if args.orientation == "distal" and np.all((lo <= [proto.cmc_x_mm, proto.cmc_z_mm, proto.extension_mm])
                                               & ([proto.cmc_x_mm, proto.cmc_z_mm, proto.extension_mm] <= hi)):
        warm = [np.round((np.array([proto.cmc_x_mm, proto.cmc_z_mm, proto.extension_mm]) - lo) / STEP_MM)]
    initial = initial_population(problem, np.array(warm).reshape(-1, 3), settings)
    start = time.time()
    result = OPTIMIZERS["nsga2"].minimize(problem, initial, settings)
    pool.close()
    out = ROOT / "output" / "thumb_layout" / args.hand / args.envelope / args.orientation / f"seed{args.seed}"
    out.mkdir(parents=True, exist_ok=True)
    keys = list(rows[0])
    with (out / "evaluations.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        w.writerows(rows)
    pareto = [cache[tuple(np.round(lo + np.round(v) * STEP_MM, 3))] for v in result.x]
    pareto = [r for r in pareto if max(constraints(r)) <= 0.0]
    pareto = sorted({(r["cmc_x_mm"], r["cmc_z_mm"], r["extension_mm"]): r for r in pareto}.values(),
                    key=lambda r: -r["kapandji_score"])
    with (out / "pareto.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        w.writerows(pareto)
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    (out / "run.json").write_text(json.dumps({
        "hand": args.hand, "orientation": args.orientation, "envelope": {args.envelope: env},
        "extension_mm": EXTENSION_MM, "step_mm": STEP_MM, "settings": settings.to_json(),
        "evaluations": len(rows), "seconds": round(time.time() - start, 1), "commit": commit,
        "prototype": proto.__dict__, "prototype_levers_mm": proto_levers}, indent=1) + "\n")
    print(f"{out}: {len(rows)} evaluations in {time.time() - start:.0f} s; Pareto:")
    for r in pareto:
        print(f"  x {r['cmc_x_mm']:7.2f} z {r['cmc_z_mm']:7.2f} e {r['extension_mm']:5.1f}  score {r['kapandji_score']:.3f} "
              f"pinch {r['pinch_deg']:6.2f}  levers {r['ip_lever_ratio']:.2f}/{r['mcp_lever_ratio']:.2f}  {r['targets']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
