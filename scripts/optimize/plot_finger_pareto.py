#!/usr/bin/env python3
"""Plot the Pareto fronts of finger study runs: worst transmission quality vs AB.

    python scripts/optimize/plot_finger_pareto.py outputs/finger/run_00{1..6}

Each run gets its own colour. The margin axis is the worst transmission
margin, ``asin`` of the quality objective.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=Path("outputs/finger/pareto.png"))
    args = parser.parse_args(argv)

    figure, axis = plt.subplots(figsize=(8, 5.5), constrained_layout=True)
    for run in args.runs:
        with (run / "pareto.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        axis.scatter(
            [float(row["ab_mm"]) for row in rows],
            [math.degrees(math.asin(float(row["worst_transmission_quality"]))) for row in rows],
            facecolor="none", s=36, label=f"{run.parent.name}/{run.name}",
        )
    axis.axhline(30.0, color="#888", linewidth=0.8, linestyle="--")
    axis.set(xlabel="AB in the extended pose [mm]  (minimize)",
             ylabel="worst transmission margin over the range [deg]  (maximize)",
             title="Finger study Pareto fronts")
    axis.grid(True, color="#e6e8eb")
    axis.legend(frameon=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=140)
    print(f"saved {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
