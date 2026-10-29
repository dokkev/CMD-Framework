#!/usr/bin/env python3
"""Run the multi-objective optimization of one finger study, as its config file says.

    python scripts/optimize/run.py                                     # scripts/optimize/linkage/config.yaml
    python scripts/optimize/run.py scripts/optimize/revolute/config.yaml
    python scripts/optimize/run.py <config> --append   # into a folder that has runs already

The config file is the only input. Its folder names the mechanism
(``scripts/optimize/<mechanism>/``; a top-level ``mechanism:`` entry
overrides it), and it sets the optimizer (``solver.optimizer`` for the
``goal`` / ``solver`` studies, ``optimizer`` for the others), the seeds and
any warm start. For another condition, write another config file (``base:``
extends one).

``solver.optimizer.preset`` picks one of ``solver.optimizer.presets`` (e.g.
``quick`` or ``search``); its runs go to ``output/<mechanism>/<config
name>/<preset>/``. A top-level ``output:`` entry (a folder from the
repository root) puts them in ``<output>/<mechanism>/<config name>/`` instead. The optimizer's ``algorithm`` may list several (e.g. ``[nsga2, smsemoa,
agemoea]``): each runs every seed. One run per algorithm and seed is saved
to ``output/<mechanism>/<config name>/run_NNN``
(``run.json``, ``pareto.csv``). The runs are then verified on the fine grid
and merged into ``portfolio_NNN`` (``seeds.csv``, the representative seeds
first, and ``portfolio.json``). Every CPU evaluates.

A folder that already holds runs or portfolios is not run into (no earlier
result is mixed with, or mistaken for, a new one) unless ``--append`` is
given; then the new runs and portfolio take the next numbers.
"""

from __future__ import annotations

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

from core.optimization.linkage.mechanism import resolve_preset  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402
from core.optimization.seeds import print_selection, select_seeds, write_seeds  # noqa: E402
from core.optimization.study import StudyResult, load_run, run_study, save_result  # noqa: E402


SCRIPTS = Path(__file__).resolve().parent
OUTPUT_ROOT = REPOSITORY_ROOT / "output"


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    append = "--append" in argv
    argv = [arg for arg in argv if arg != "--append"]
    if len(argv) > 1 or (argv and argv[0].startswith("-")):
        print(__doc__.strip())
        return 2
    path = Path(argv[0]) if argv else SCRIPTS / "linkage" / "config.yaml"
    values = read_config_file(path)
    found = mechanism(values.pop("mechanism", None) or path.resolve().parent.name)
    output_root = values.pop("output", None)
    preset = resolve_preset(values)
    algorithms = _algorithms(values)
    values, settings = found.split_settings(values)
    config = found.config_from_mapping(values)
    seeds = tuple(getattr(config, "seeds", ()) or (settings.seed,))
    # The population takes at most half its size of warm-start designs.
    initial = warm_designs(found, getattr(config, "warm_start", ()) or ())[: settings.population_size // 2]
    root = (REPOSITORY_ROOT / str(output_root) / found.name / path.stem if output_root is not None
            else OUTPUT_ROOT / found.name / path.stem / (preset or ""))
    earlier = sorted(p.name for pattern in ("run_*", "portfolio_*") for p in root.glob(pattern)) if root.exists() else []
    if earlier and not append:
        print(f"{root} already holds {', '.join(earlier[:4])}{', ...' if len(earlier) > 4 else ''}: not run "
              "(move them, or run with --append to add to them)", file=sys.stderr)
        return 3
    workers = os.cpu_count() or 1
    print(f"{found.name}: {path}{f' (preset {preset})' if preset else ''} -> {root}; "
          f"seeds {', '.join(map(str, seeds))}"
          + (f"; warm start from {len(initial)} designs" if initial else "") + "\n")

    directories = [run_one(found, config, dataclasses.replace(settings, algorithm=algorithm, seed=seed), initial,
                           root, workers)
                   for algorithm in algorithms or (settings.algorithm,) for seed in seeds]
    runs = [load_run(directory) for directory in directories]
    step_rad = getattr(config, "verify_step_rad", math.radians(0.05))
    selection = select_seeds(runs, step_rad)
    output = _next_directory(root, "portfolio")
    write_seeds(selection, output / "seeds.csv")
    (output / "portfolio.json").write_text(json.dumps({
        "config": str(path),
        "runs": [directory.name for directory in directories],
        "optimizers": [run.settings.to_json() for run in runs],
        "verification_step_deg": math.degrees(step_rad),
        "source_git_commit": _git_commit(),
    }, indent=2) + "\n", encoding="utf-8")
    print("Verified and merged:")
    print_selection(selection, found, output / "seeds.csv")
    return 0


def _algorithms(values: dict) -> tuple[str, ...]:
    """The optimizers a config lists (``algorithm`` as a list), left as the
    first for the settings; ``()``: one, as it is."""

    for table in (values.get("solver", {}).get("optimizer") if isinstance(values.get("solver"), dict) else None,
                  values.get("optimizer")):
        if isinstance(table, dict) and isinstance(table.get("algorithm"), list):
            algorithms = tuple(str(name) for name in table["algorithm"])
            if not algorithms:
                raise ValueError("the optimizer's algorithm list is empty")
            table["algorithm"] = algorithms[0]
            return algorithms
    return ()


def warm_designs(found, patterns) -> list[list[float]]:
    """The designs of the warm-start files (paths from the repository root; a
    ``*`` matches several, absent ones are skipped), taken in turns from each
    file so that every file's first (best) rows come first: the population
    takes at most half its size of them."""

    files = [path for pattern in patterns
             for path in (sorted(REPOSITORY_ROOT.glob(pattern)) if "*" in pattern else [REPOSITORY_ROOT / pattern])
             if path.exists()]
    lists = [list(csv.DictReader(path.open(encoding="utf-8", newline=""))) for path in files]
    designs = []
    for index in range(max(map(len, lists), default=0)):
        designs.extend(found.read_design(rows[index]) for rows in lists if index < len(rows))
    return designs


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
    print("Representative seeds (coarse grid; verified after the runs):")
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
