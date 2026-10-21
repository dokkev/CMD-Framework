#!/usr/bin/env python3
"""Plot Pareto fronts over the objectives the runs optimized, one colour per mechanism.

    python scripts/optimize/plot_pareto.py output/linkage/run_046 output/rcj/run_001

The runs' Pareto candidates are pooled per mechanism. The first objective is
the vertical axis and the second the horizontal one. With one mechanism a
third objective is the colour; with several, the colour is the mechanism, so
their fronts compare under the same pad requirements. The runs must use their
mechanism's current formulation and the same objectives (by name).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.study import load_run  # noqa: E402


def axis_label(objective) -> str:
    return f"{objective.label}  ({'maximize' if objective.larger_is_better else 'minimize'})"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=Path("output/pareto.png"))
    args = parser.parse_args(argv)

    runs = [load_run(directory) for directory in args.runs]
    names = runs[0].objectives
    pooled: dict[str, tuple] = {}  # mechanism name: (mechanism, rows)
    for run in runs:
        if not run.current:
            parser.error(f"{run.directory} used the {run.formulation!r} formulation, "
                         f"not {run.mechanism.formulation!r}")
        if run.objectives != names:
            parser.error(f"{run.directory} optimized {run.objectives}, not {names}")
        pooled.setdefault(run.mechanism.name, (run.mechanism, []))[1].extend(run.candidates())
    if len(names) < 2:
        parser.error(f"one objective ({names[0]}) has no front to plot")

    def column(found, rows, name: str) -> list[float]:
        return [float(row[found.objectives[name].column]) for row in rows]

    figure, axis = plt.subplots(figsize=(8, 5.5), constrained_layout=True)
    found, rows = next(iter(pooled.values()))
    if len(pooled) == 1:
        colour = {"c": column(found, rows, names[2]), "cmap": "viridis"} if len(names) > 2 else {}
        points = axis.scatter(column(found, rows, names[1]), column(found, rows, names[0]), s=36, **colour)
        if len(names) > 2:
            figure.colorbar(points, ax=axis, label=axis_label(found.objectives[names[2]]))
        title = f"{found.name.replace('_', ' ').capitalize()} Pareto front"
    else:
        for name, (each, each_rows) in pooled.items():
            axis.scatter(column(each, each_rows, names[1]), column(each, each_rows, names[0]),
                         s=36, label=name)
        axis.legend(title="mechanism")
        title = "Pareto fronts by mechanism"
    axis.set(xlabel=axis_label(found.objectives[names[1]]), ylabel=axis_label(found.objectives[names[0]]),
             title=title)
    axis.grid(True, color="#e6e8eb")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=140)
    print(f"saved {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
