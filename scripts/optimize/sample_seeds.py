#!/usr/bin/env python3
"""Sample a study's search box for feasible designs, to warm-start its runs.

    python scripts/optimize/sample_seeds.py <config.yaml> [samples] [target] [seed] [from]

1. Draws ``samples`` (default 200000) points uniformly in the mechanism's
   search coordinates (on their grid) and evaluates them on every core (a
   design failing a static check is not swept, so most cost little).
2. Grows the feasible ones (with those of an earlier sampling of the config,
   if any, or of a ``from`` file: each taken through the config's own search
   coordinates, so a design of another study becomes its nearest of this
   one): each round perturbs designs of the pool by :data:`SPREAD` of every
   coordinate's range and keeps the feasible, until the pool holds ``target``
   (default 1000) or a round adds none.

Writes the pool, best first by the study's first objective, to
output/<mechanism>/<config stem>/samples/seeds.csv: a file the config's
``solver.optimizer.warm_start`` can name. Prints the share of the uniform
draws failing each constraint (a design may fail several).
"""

from __future__ import annotations

import collections
import csv
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402
from core.optimization.study import MechanismProblem  # noqa: E402

CHUNK = 500
UNSWEPT = 99.0
"""Constraint values at or above this were not checked (an earlier stage failed)."""
SPREAD = 0.05
"""Standard deviation of a growing step, as a share of each coordinate's range."""
ROUND = 20_000
"""Perturbed designs per growing round."""

_state: dict = {}


def _load(path: Path):
    values = read_config_file(path)
    found = mechanism(values.pop("mechanism", None) or path.resolve().parent.name)
    values, _ = found.split_settings(values)
    return found, found.config_from_mapping(values)


def _init(path: str) -> None:
    found, config = _load(Path(path))
    found.prepare(config)
    _state.update(found=found, config=config, problem=MechanismProblem(found, config))


def _evaluate(task: tuple[int, int, np.ndarray | None]) -> tuple[list[tuple[float, ...]], collections.Counter, int]:
    """``count`` draws: uniform, or about ``parents`` (grid steps) when given."""

    seed, count, parents = task
    found, config, problem = _state["found"], _state["config"], _state["problem"]
    rng = np.random.default_rng(seed)
    width = problem.xu - problem.xl
    feasible, failing, decoded = [], collections.Counter(), 0
    for _ in range(count):
        # The problem's variables count grid steps.
        if parents is None:
            steps = problem.xl + rng.random(problem.n_var) * width
        else:
            steps = parents[rng.integers(len(parents))] + rng.normal(0.0, SPREAD, problem.n_var) * width
        x = problem.to_mm(np.round(np.clip(steps, problem.xl, problem.xu)))
        if x is None:
            continue
        decoded += 1
        evaluation = found.evaluate(x, config)
        if evaluation is None:
            continue
        if evaluation.feasible:
            feasible.append(tuple(float(value) for value in x))
        failing.update(name for name, value in evaluation.constraints.items() if 0.0 < value < UNSWEPT)
    return feasible, failing, decoded


def _run(pool, tasks) -> tuple[list[tuple[float, ...]], collections.Counter, int]:
    designs, failing, decoded = [], collections.Counter(), 0
    for feasible, fails, count in pool.map(_evaluate, tasks):
        designs.extend(feasible)
        failing.update(fails)
        decoded += count
    return designs, failing, decoded


def _chunks(seed: int, count: int, parents: np.ndarray | None) -> list[tuple[int, int, np.ndarray | None]]:
    return [(seed + index, min(CHUNK, count - index * CHUNK), parents) for index in range(-(-count // CHUNK))]


def main(argv: list[str]) -> int:
    if not argv or argv[0].startswith("-"):
        print(__doc__.strip())
        return 2
    path = Path(argv[0]).resolve()
    samples = int(argv[1]) if len(argv) > 1 else 200_000
    target = int(argv[2]) if len(argv) > 2 else 1000
    seed = int(argv[3]) if len(argv) > 3 else 0
    source = Path(argv[4]) if len(argv) > 4 else None
    found, config = _load(path)
    problem = MechanismProblem(found, config)
    output = REPOSITORY_ROOT / "output" / found.name / path.stem / "samples" / "seeds.csv"
    pool_designs: dict[tuple[float, ...], None] = {}
    found.prepare(config)
    for start_file in (output, source):  # earlier pools, as this config searches them
        if start_file is None or not start_file.exists():
            continue
        rows = list(csv.DictReader(start_file.open(encoding="utf-8", newline="")))
        for row in rows:
            x = problem.to_mm(problem.to_steps(found.read_design(row)))
            if x is None:
                continue
            x = tuple(float(value) for value in x)
            evaluation = found.evaluate(x, config)
            if evaluation is not None and evaluation.feasible:
                pool_designs[x] = None
        print(f"{len(pool_designs)} feasible designs after {start_file} ({len(rows)} rows)")

    start = time.time()
    with ProcessPoolExecutor(os.cpu_count() or 1, initializer=_init, initargs=(str(path),)) as pool:
        if samples:
            designs, failing, decoded = _run(pool, _chunks(seed * 1_000_003, samples, None))
            pool_designs.update(dict.fromkeys(designs))
            print(f"uniform: {samples} draws, {decoded} decoded, {len(designs)} feasible "
                  f"({time.time() - start:.0f} s)")
            for name, count in failing.most_common(8):
                print(f"  {name:28s} {100.0 * count / max(decoded, 1):5.1f} %")
        for round_index in range(1, 1000):
            if not pool_designs or len(pool_designs) >= target:
                break
            parents = np.array([problem.to_steps(x) for x in pool_designs], dtype=float)
            designs, _, _ = _run(pool, _chunks(seed * 1_000_003 + round_index * 7919, ROUND, parents))
            before = len(pool_designs)
            pool_designs.update(dict.fromkeys(designs))
            print(f"grow {round_index}: {len(designs)} of {ROUND} feasible, pool {before} -> {len(pool_designs)} "
                  f"({time.time() - start:.0f} s)")
            if len(pool_designs) == before:
                break
    if not pool_designs:
        print("no feasible design")
        return 1

    objectives = found.active_objectives(config)
    rows = []
    for x in pool_designs:
        evaluation = found.evaluate(x, config)
        rows.append((found.objective_values(evaluation, objectives), x, found.report(evaluation)))
    rows.sort(key=lambda row: row[0])
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow((*found.variable_names, *rows[0][2]))
        for _, x, report in rows:
            writer.writerow((*x, *report.values()))
    print(f"wrote {len(rows)} designs -> {output.relative_to(REPOSITORY_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
