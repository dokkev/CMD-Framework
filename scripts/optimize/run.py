#!/usr/bin/env python3
"""Run the multi-objective optimization of one finger mechanism and save its feasible Pareto set.

Pick the mechanism (``core/optimization/mechanism.py``: ``MECHANISMS``) and
edit its ``scripts/optimize/<mechanism>/config.yaml`` (or pass another with
``--config``); its ``optimizer`` table picks the algorithm
(``core/optimization/optimizer.py``: ``OPTIMIZERS``). Then run, e.g.

    python scripts/optimize/run.py --mechanism linkage --seed 0
    python scripts/optimize/run.py --mechanism linkage --algorithm smsemoa
    python scripts/optimize/run.py --mechanism linkage --portfolio

``--portfolio`` runs complementary studies and merges them: every optimizer
(or ``--algorithms``), ``--runs-per-algorithm`` times, each from its own seed
(the config's, then the next ones), since the seed sets the starting
population and runs from one population fill the same part of the front. Each
run is saved as usual; ``output/<mechanism>/portfolio_NNN`` gets the verified,
merged ``seeds.csv`` (as ``select_seeds.py`` writes it) and ``portfolio.json``.

Each run writes ``run.json`` and ``pareto.csv`` to a new
``output/<mechanism>/run_NNN``; ``select_seeds.py`` verifies them and marks the
seeds, ``plot_pareto.py`` plots fronts (of several mechanisms together).
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import os
import subprocess
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.mechanism import MECHANISMS, mechanism  # noqa: E402
from core.optimization.optimizer import OPTIMIZERS, portfolio  # noqa: E402
from core.optimization.seeds import print_selection, select_seeds, write_seeds  # noqa: E402
from core.optimization.study import StudyResult, load_run, load_yaml, run_study, save_result  # noqa: E402


SCRIPTS = Path(__file__).resolve().parent
OUTPUT_ROOT = REPOSITORY_ROOT / "output"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mechanism", choices=sorted(MECHANISMS), default="linkage")
    parser.add_argument("--config", type=Path,
                        help="study settings (default: scripts/optimize/<mechanism>/config.yaml)")
    parser.add_argument("--algorithm", choices=sorted(OPTIMIZERS), help="optimizer (default: the config's)")
    parser.add_argument("--seed", type=int, help="optimizer seed (default: the config's)")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                        help="override a config entry, e.g. motion.required_rom_deg=150 (repeatable)")
    parser.add_argument("--warm-start", type=Path, nargs="+", default=(), metavar="CSV",
                        help="pareto.csv or seeds.csv files whose designs seed up to half the "
                        "population (re-evaluated, so earlier formulations may warm-start)")
    parser.add_argument("--output-root", type=Path,
                        help="where run_NNN goes (default: output/<mechanism>)")
    parser.add_argument("--workers", type=int, default=os.cpu_count() or 1,
                        help="processes evaluating each generation (default: %(default)s)")
    parser.add_argument("--portfolio", action="store_true",
                        help="run every optimizer from its own seed and merge the runs")
    parser.add_argument("--algorithms", nargs="+", choices=sorted(OPTIMIZERS),
                        help="the portfolio's optimizers (default: all of them)")
    parser.add_argument("--runs-per-algorithm", type=int, default=1, metavar="N",
                        help="portfolio runs of each optimizer (default: %(default)s)")
    parser.add_argument("--step-deg", type=float, default=0.05,
                        help="motion step the portfolio is verified on (default: %(default)s)")
    args = parser.parse_args(argv)
    if args.portfolio and args.algorithm is not None:
        parser.error("--portfolio picks its optimizers with --algorithms, not --algorithm")
    if not args.portfolio and (args.algorithms or args.runs_per_algorithm != 1):
        parser.error("--algorithms and --runs-per-algorithm need --portfolio")
    found = mechanism(args.mechanism)
    config, settings = load_yaml(found, args.config or SCRIPTS / found.name / "config.yaml", args.set)
    if args.algorithm is not None:
        settings = dataclasses.replace(settings, algorithm=args.algorithm)
    if args.seed is not None:
        settings = dataclasses.replace(settings, seed=args.seed)

    initial = [
        found.read_design(row)
        for path in args.warm_start
        for row in csv.DictReader(path.open(encoding="utf-8", newline=""))
    ]
    root = args.output_root or OUTPUT_ROOT / found.name
    if not args.portfolio:
        run_one(found, config, settings, initial, root, args.workers)
        return 0

    plan = portfolio(settings, args.algorithms or list(OPTIMIZERS), args.runs_per_algorithm)
    print(f"Portfolio: {', '.join(f'{s.algorithm}/{s.seed}' for s in plan)}\n")
    directories = [run_one(found, config, each, initial, root, args.workers) for each in plan]
    runs = [load_run(directory) for directory in directories]
    selection = select_seeds(runs, math.radians(args.step_deg))
    output = _next_directory(root, "portfolio")
    write_seeds(selection, output / "seeds.csv")
    (output / "portfolio.json").write_text(json.dumps({
        "runs": [directory.name for directory in directories],
        "optimizers": [each.to_json() for each in plan],
        "verification_step_deg": args.step_deg,
        "source_git_commit": _git_commit(),
    }, indent=2) + "\n", encoding="utf-8")
    print("Portfolio, verified and merged:")
    print_selection(selection, found, output / "seeds.csv")
    return 0


def run_one(found, config, settings, initial, root: Path, workers: int) -> Path:
    result = run_study(found, config, settings, initial_designs=initial, workers=workers)
    output = save_result(result, _next_directory(root, "run"), source_git_commit=_git_commit())
    print(f"Optimizer: {settings.algorithm} (seed {settings.seed}), evaluations: {result.evaluation_count}")
    print(f"Feasible Pareto candidates: {len(result.pareto)}")
    print_representatives(result)
    print(f"Output: {output}\n")
    return output


def print_representatives(result: StudyResult) -> None:
    """The representative seeds of the front, on the optimizer's coarse grid
    (``select_seeds.py`` verifies them on a fine one)."""

    found = result.mechanism
    names = found.active_objectives(result.config)
    rows = [found.report(candidate.evaluation) for candidate in result.pareto]
    scores = [found.objective_values(candidate.evaluation, names) for candidate in result.pareto]
    roles = found.seed_roles(scores, rows, names)
    if not roles:
        return
    print("Representative seeds (coarse grid; select_seeds.py verifies them):")
    print(f"  {'role':18s} {'#':>4s} {found.summary_header()}")
    for role in found.role_order(names):
        index = roles[role]
        print(f"  {role:18s} {index + 1:4d} {found.summary_line(rows[index])}")


def _next_directory(root: Path, prefix: str) -> Path:
    index = 1
    while (root / f"{prefix}_{index:03d}").exists():
        index += 1
    return root / f"{prefix}_{index:03d}"


def _git_commit() -> str | None:
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPOSITORY_ROOT,
            check=True, capture_output=True, text=True, timeout=3.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return completed.stdout.strip() or None


if __name__ == "__main__":
    raise SystemExit(main())
