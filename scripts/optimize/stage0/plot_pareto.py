#!/usr/bin/env python3
"""Plot the Stage-0 Pareto front over the objectives the runs optimized.

    python scripts/optimize/stage0/plot_pareto.py output/stage0/run_00{1..6}

All runs' Pareto candidates are pooled. The first objective is the vertical
axis and the second the horizontal one; a third is the colour (by default:
k_max against the pad length, coloured by phi_safe_max). The runs must use
the current formulation and the same objectives.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.stage0.objectives import OBJECTIVES  # noqa: E402
from core.optimization.stage0.run import FORMULATION, load_formulation, load_objectives  # noqa: E402


def axis_label(name: str) -> str:
    objective = OBJECTIVES[name]
    return f"{objective.label}  ({'maximize' if objective.larger_is_better else 'minimize'})"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=Path("output/stage0/pareto.png"))
    args = parser.parse_args(argv)

    names = load_objectives(args.runs[0])
    rows = []
    for run in args.runs:
        if (formulation := load_formulation(run)) != FORMULATION:
            parser.error(f"{run} used the {formulation!r} formulation, not {FORMULATION!r}")
        if load_objectives(run) != names:
            parser.error(f"{run} optimized {load_objectives(run)}, not {names}")
        with (run / "pareto.csv").open(newline="") as stream:
            rows += list(csv.DictReader(stream))
    if len(names) < 2:
        parser.error(f"one objective ({names[0]}) has no front to plot")

    def column(name: str) -> list[float]:
        return [float(row[OBJECTIVES[name].column]) for row in rows]

    figure, axis = plt.subplots(figsize=(8, 5.5), constrained_layout=True)
    colour = {"c": column(names[2]), "cmap": "viridis"} if len(names) > 2 else {}
    points = axis.scatter(column(names[1]), column(names[0]), s=36, **colour)
    if len(names) > 2:
        figure.colorbar(points, ax=axis, label=axis_label(names[2]))
    axis.set(xlabel=axis_label(names[1]), ylabel=axis_label(names[0]), title="Stage-0 Pareto front")
    axis.grid(True, color="#e6e8eb")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=140)
    print(f"saved {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
