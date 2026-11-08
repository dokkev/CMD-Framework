#!/usr/bin/env python3
"""Verify Pareto candidates of one mechanism's runs on a fine grid and mark the seeds.

    python scripts/optimize/select_seeds.py output/linkage/run_001 output/linkage/run_002

Each candidate is re-evaluated with its motion sampled every ``--step-deg``
(the optimizer uses a coarse grid, so a condition can fail between samples).
The verified, mutually non-dominated candidates are written with a
``seed_role``, for the objectives the runs optimized:

- the best at each objective, e.g. ``best_transmission`` (smallest ``k_max``)
  and ``best_curl`` (largest ``phi_safe_max``);
- ``balanced``: the closest to the ideal point with each objective scaled to
  ``[0, 1]`` over the verified front (the compromise of them all);
- ``pareto``: every other verified non-dominated candidate.

A candidate that holds several roles lists them all, joined by ``;``. The runs
must be of one mechanism, share their study settings (the mechanism's config;
the optimizer and its settings may differ) and use its current formulation;
earlier runs can only warm-start a new one. ``run.py`` runs a config's seeds
and merges them in one go (``core/optimization/seeds.py``).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.seeds import print_selection, select_seeds, write_seeds  # noqa: E402
from core.optimization.study import load_run  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path, help="run directories of one mechanism")
    parser.add_argument("--step-deg", type=float, default=0.05)
    parser.add_argument("--output", type=Path,
                        help="default: output/<mechanism>/seeds.csv")
    args = parser.parse_args(argv)

    runs = [load_run(directory) for directory in args.runs]
    try:
        selection = select_seeds(runs, math.radians(args.step_deg))
    except ValueError as error:
        parser.error(str(error))
    found = runs[0].mechanism
    output = args.output or REPOSITORY_ROOT / "output" / found.name / "seeds.csv"
    write_seeds(selection, output)
    print_selection(selection, found, output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
