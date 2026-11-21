#!/usr/bin/env python3
"""E11: does taking the pad arrangement out of the search help the CFL converge?

    python scripts/study/exploratory/e11_placement_ablation.py [--flexion 120] [--thickness 7]
        [--seeds 0,1,2,3,4,5] [--population 200] [--generations 300] [--methods A,B,C,E,F]

One CFL condition of Study 1, the same goal, evaluator and objective, several ways:

- A, baseline: the GA over the whole design vector (Study 1 as it is);
- B, reduced: the GA over the linkage alone, each candidate's pad arrangement
  set by ``core.optimization.analysis.placement`` (Study 0's relations) before
  it is scored;
- C, warm start: the baseline GA, its first population's designs given their
  placed arrangement first;
- E, closure coordinates: the GA over the coordinates in which the four-bar
  always closes to the required flexion (``closure_search``), half of its
  first population feasible designs drawn in those coordinates through the
  closed-form transmission-angle filter (their cost is counted);
- G, conditional starts: the baseline GA and coordinates, its starts E14's
  feasible linkages (linkage first, pads drawn given the linkage);
- H, neighbour starts: the baseline GA and coordinates, its starts Study 1's
  designs of the same flexion for the other thicknesses;
- I, both: H's neighbour designs first, then G's sampled linkages, up to
  half a population;
- F, closure starts only: the baseline GA and coordinates, given E's starts
  (what the starts do apart from the coordinates).

Two budgets are reported, since a candidate of B costs more than one of A:
the best coverage against the candidates evaluated (does removing the two
variables help the search itself), and against the wall-clock time (is it
faster). Per seed: the final best, and the candidates and seconds to reach
``--target``. Writes ``output/study/exploratory/e11_placement_ablation/``.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT, ROOT / "scripts" / "study"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import shared  # noqa: E402
from core.optimization.analysis.closure_search import ClosureMechanism  # noqa: E402
from core.optimization.analysis.placement import PlacedMechanism, place  # noqa: E402
from core.optimization.optimizer import OptimizerSettings, initial_population  # noqa: E402
from core.optimization.study import MechanismProblem, run_study  # noqa: E402

OUTPUT = ROOT / "output" / "study" / "exploratory" / "e11_placement_ablation"


def condition(flexion: float, thickness: float):
    found, config = shared.load_config("scripts/study/cfl.yaml", ROOT, None, (
        f"goal.pads.proximal.height_mm={thickness:g}", f"goal.pads.distal.height_mm={thickness:g}"))
    return found, shared.with_required_flexion(config, flexion)


def placed_starts(base, config, settings: OptimizerSettings) -> tuple[list, float]:
    """The baseline's first population (same seed), each design with its placed arrangement; the seconds it took."""

    started = time.time()
    problem = MechanismProblem(base, config)
    found = []
    for steps in initial_population(problem, np.empty((0, problem.n_var)), settings):
        x = problem.to_mm(steps)
        if x is None:
            continue
        try:
            design, evaluation, _ = place(base, x, config)
        except ValueError:
            continue
        if design is not None and evaluation is not None and evaluation.feasible:
            found.append(design)
    return found, time.time() - started


_DRAW = {}


def _drawn(job):
    """A worker's draws in the closure coordinates: those the closed-form
    filter keeps and the evaluator then accepts; and how many reached the evaluator."""

    seed, count = job
    mechanism, config = _DRAW["mechanism"], _DRAW["config"]
    lower, upper = (np.array(v) for v in mechanism.search_box(config))
    steps = np.array(mechanism.search_steps(config))
    rng, found, evaluated = np.random.default_rng(seed), [], 0
    for _ in range(count):
        coordinates = lower + np.round(rng.random(len(lower)) * (upper - lower) / steps) * steps
        if not mechanism.accepts(coordinates, config):
            continue
        x = mechanism.to_design(coordinates, config)
        if x is None:
            continue
        evaluated += 1
        try:
            evaluation = mechanism.evaluate(x, config)
        except ValueError:
            continue
        if evaluation is not None and evaluation.feasible:
            found.append(x)
    return found, evaluated


def closure_starts(base, config, settings: OptimizerSettings, workers: int = 8, batch: int = 4000, most: int = 400):
    """Half a population of feasible designs drawn in the closure coordinates
    (``closure_search``): uniform in their box, kept where the transmission
    angle stays in range (closed form), then checked by the evaluator. Returns
    the designs, the seconds and the evaluator calls it took."""

    import multiprocessing

    started = time.time()
    _DRAW.update(mechanism=ClosureMechanism(base), config=config)
    wanted, found, spent = settings.population_size // 2, [], 0
    with multiprocessing.get_context("fork").Pool(workers) as pool:
        for round_ in range(most):
            jobs = [(100000 * (settings.seed + 1) + workers * round_ + w, batch) for w in range(workers)]
            for designs, evaluated in pool.map(_drawn, jobs):
                found += designs
                spent += evaluated
            if len(found) >= wanted:
                break
    return found[:wanted], time.time() - started, spent


CONDITIONAL = OUTPUT.parent / "e14_conditional_pads"


def conditional_starts(base, settings: OptimizerSettings):
    """E14's feasible linkages (pads drawn given the linkage) as starts: at
    most half a population, the same for every seed. Their cost is E14's
    measured CPU time per feasible linkage, as seconds on one core."""

    with (CONDITIONAL / "feasible_conditional.csv").open(encoding="utf-8", newline="") as stream:
        designs = [tuple(float(row[name]) for name in base.variable_names) for row in csv.DictReader(stream)]
    with (CONDITIONAL / "summary.csv").open(encoding="utf-8", newline="") as stream:
        each = next(float(row["cpu_s_per_feasible"]) for row in csv.DictReader(stream) if row["kind"] == "conditional")
    designs = designs[: settings.population_size // 2]
    return designs, each * len(designs), 0


def neighbour_starts(base, flexion: float, thickness: float, thicknesses=(3.0, 5.0, 7.0, 9.0)) -> list:
    """Study 1's designs of the same flexion for the other thicknesses (their
    design vectors as they are; under this thickness they need not be feasible)."""

    found = []
    for other in thicknesses:
        if other == thickness:
            continue
        tag = f"cfl_q{flexion:g}" + ("" if other == 5.0 else f"_h{other:g}")
        portfolios = sorted((ROOT / "output" / "study" / "linkage" / tag).glob("portfolio_*/seeds.csv"))
        if not portfolios:
            continue
        with portfolios[-1].open(encoding="utf-8", newline="") as stream:
            rows = [row for row in csv.DictReader(stream) if "balanced" in row.get("seed_role", "")]
        if rows:
            found.append(tuple(float(rows[0][name]) for name in base.variable_names))
    return found


def curve(history, generations: int) -> np.ndarray:
    """The best coverage after each generation (NaN before the first feasible design)."""

    best = np.full(generations + 1, np.nan)
    if history is None or not len(history.generation):
        return best
    values = -np.asarray(history.objectives).reshape(len(history.generation), -1)[:, 0]
    for g in range(generations + 1):
        seen = values[np.asarray(history.generation) <= g]
        if len(seen):
            best[g] = seen.max()
    return best


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--flexion", type=float, default=120.0)
    parser.add_argument("--thickness", type=float, default=7.0)
    parser.add_argument("--seeds", default="0,1,2,3,4,5")
    parser.add_argument("--population", type=int, default=200)
    parser.add_argument("--generations", type=int, default=300)
    parser.add_argument("--methods", default="A,B,C,E,F")
    parser.add_argument("--target", type=float, default=0.715, help="coverage whose first reach is timed")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--tag", default="", help="suffix of the files written (runs<tag>.csv, curves<tag>.csv)")
    args = parser.parse_args(argv)
    base, config = condition(args.flexion, args.thickness)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    rows, curves = [], []
    for method in args.methods.split(","):
        for seed in (int(s) for s in args.seeds.split(",")):
            settings = OptimizerSettings("ga", args.population, args.generations, seed)
            search, starts, before, extra = base, [], 0.0, 0
            if method == "B":
                search = PlacedMechanism(base)
            elif method == "C":
                starts, before = placed_starts(base, config, settings)
            elif method == "I":
                sampled, before, extra = conditional_starts(base, settings)
                near = neighbour_starts(base, args.flexion, args.thickness)
                starts = near + sampled[: settings.population_size // 2 - len(near)]
            elif method == "H":
                starts, before, extra = neighbour_starts(base, args.flexion, args.thickness), 0.0, 0
            elif method == "G":
                starts, before, extra = conditional_starts(base, settings)
            elif method in ("E", "F"):
                starts, before, extra = closure_starts(base, config, settings, args.workers)
                if method == "E":
                    search = ClosureMechanism(base)
            started = time.time()
            result = run_study(search, config, settings, initial_designs=starts, workers=args.workers)
            seconds = time.time() - started + before
            best = curve(result.history, args.generations)
            per_generation = np.asarray(result.history.evaluations) if result.history is not None else np.array([])
            candidates = np.cumsum(per_generation) if len(per_generation) else np.arange(args.generations + 1)
            reached = np.flatnonzero(best >= args.target)
            first = int(reached[0]) if len(reached) else None
            final = float(np.nanmax(best)) if np.isfinite(best).any() else float("nan")
            rows.append({"method": method, "seed": seed, "final_c_pad": final, "seconds": seconds,
                         "candidates": int(result.evaluation_count) + extra, "warm_starts": len(starts),
                         "generation_to_target": "" if first is None else first,
                         "candidates_to_target": "" if first is None else int(candidates[min(first, len(candidates) - 1)]),
                         "seconds_to_target": "" if first is None else seconds * (first + 1) / (args.generations + 1)})
            curves += [{"method": method, "seed": seed, "generation": g, "best_c_pad": float(v)}
                       for g, v in enumerate(best) if np.isfinite(v)]
            print(f"{method} seed {seed}: final {final:.4f}, {seconds:.0f} s, {result.evaluation_count} candidates"
                  + ("" if first is None else f", {args.target} at generation {first}"), flush=True)
            for name, table in (("runs", rows), ("curves", curves)):
                if not table:  # no feasible design yet: nothing to draw a curve from
                    continue
                with (OUTPUT / f"{name}{args.tag}.csv").open("w", encoding="utf-8", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(table[0]))
                    writer.writeheader()
                    writer.writerows(table)
    print(f"{'method':6s} {'runs':>4s} {'best':>7s} {'median':>7s} {'worst':>7s} {'seconds':>8s} {'reached target':>15s}")
    for method in args.methods.split(","):
        own = [r for r in rows if r["method"] == method]
        finals = np.array([r["final_c_pad"] for r in own])
        hit = [r for r in own if r["generation_to_target"] != ""]
        print(f"{method:6s} {len(own):4d} {np.nanmax(finals):7.4f} {np.nanmedian(finals):7.4f} {np.nanmin(finals):7.4f} "
              f"{np.median([r['seconds'] for r in own]):8.0f} {len(hit):9d} of {len(own)}")
    print(f"wrote {OUTPUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
