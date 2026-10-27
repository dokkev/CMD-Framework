#!/usr/bin/env python3
"""Run the multi-objective optimization of one finger study, as its config file says.

    python scripts/optimize/run.py                                     # scripts/optimize/linkage/config.yaml
    python scripts/optimize/run.py scripts/optimize/revolute/config.yaml

The config file is the only input. Its folder names the mechanism
(``scripts/optimize/<mechanism>/``; a top-level ``mechanism:`` entry
overrides it), and it sets the optimizer (``solver.optimizer`` for the
``goal`` / ``solver`` studies, ``optimizer`` for the others), the seeds and
any warm start. For another condition, write another config file (``base:``
extends one).

One run per seed is saved to ``output/<mechanism>/<config name>/run_NNN``
(``run.json``, ``pareto.csv``). The runs are then verified on the fine grid
and merged into ``portfolio_NNN`` (``seeds.csv``, the representative seeds
first, and ``portfolio.json``). Every CPU evaluates.
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

from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402
from core.optimization.seeds import print_selection, select_seeds, write_seeds  # noqa: E402
from core.optimization.study import StudyResult, load_run, run_study, save_result  # noqa: E402


SCRIPTS = Path(__file__).resolve().parent
OUTPUT_ROOT = REPOSITORY_ROOT / "output"


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    if len(argv) > 1 or (argv and argv[0].startswith("-")):
        print(__doc__.strip())
        return 2
    path = Path(argv[0]) if argv else SCRIPTS / "linkage" / "config.yaml"
    values = read_config_file(path)
    found = mechanism(values.pop("mechanism", None) or path.resolve().parent.name)
    values, settings = found.split_settings(values)
    config = found.config_from_mapping(values)
    seeds = tuple(getattr(config, "seeds", ()) or (settings.seed,))
    initial = [found.read_design(row)
               for warm in getattr(config, "warm_start", ()) or ()
               for row in csv.DictReader((REPOSITORY_ROOT / warm).open(encoding="utf-8", newline=""))]
    root = OUTPUT_ROOT / found.name / path.stem
    workers = os.cpu_count() or 1
    print(f"{found.name}: {path} -> {root}; seeds {', '.join(map(str, seeds))}"
          + (f"; warm start from {len(initial)} designs" if initial else "") + "\n")

    directories = [run_one(found, config, dataclasses.replace(settings, seed=seed), initial, root, workers)
                   for seed in seeds]
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
