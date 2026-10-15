#!/usr/bin/env python3
"""Plot the Pareto fronts of finger study runs: worst transmission quality vs AB.

    python scripts/optimize/plot_finger_pareto.py outputs/finger/run_00{1..6}

Each run gets its own colour. The vertical axis is the run's first objective:
the worst transmission margin (``asin`` of the quality), lever or torque ratio.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=Path("outputs/finger/pareto.png"))
    args = parser.parse_args(argv)

    figure, axis = plt.subplots(figsize=(8, 5.5), constrained_layout=True)
    objective = None
    for run in args.runs:
        objective = json.loads((run / "run.json").read_text())["config"].get("objective", "transmission")
        with (run / "pareto.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        if objective == "torque_ratio":
            values = [float(row["torque_ratio_max"]) for row in rows]
        elif objective == "lever":
            values = [float(row["lever_max_mm"]) for row in rows]
        else:
            values = [math.degrees(math.asin(float(row["worst_transmission_quality"]))) for row in rows]
        axis.scatter([float(row["ab_mm"]) for row in rows], values,
                     facecolor="none", s=36, label=f"{run.parent.name}/{run.name}")
    ylabel = {
        "torque_ratio": "worst torque ratio k = dphi/dtheta over the workspace  (minimize)",
        "lever": "worst lever r over the workspace [mm]  (minimize)",
    }.get(objective, "worst transmission margin over the range [deg]  (maximize)")
    if objective == "transmission":
        axis.axhline(30.0, color="#888", linewidth=0.8, linestyle="--")
    axis.set(xlabel="AB in the extended pose [mm]  (minimize)", ylabel=ylabel,
             title="Finger study Pareto fronts")
    axis.grid(True, color="#e6e8eb")
    axis.legend(frameon=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=140)
    print(f"saved {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
