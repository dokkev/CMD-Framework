#!/usr/bin/env python3
"""Plot the Stage-0 Pareto front: k_max against OA + BC, coloured by phi_safe_max.

    python scripts/optimize/plot_stage0_pareto.py outputs/stage0/run_00{1..6}

All runs' Pareto candidates are pooled; the third objective is the colour.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=Path("outputs/stage0/pareto.png"))
    args = parser.parse_args(argv)

    rows = []
    for run in args.runs:
        with (run / "pareto.csv").open(newline="") as stream:
            rows += list(csv.DictReader(stream))
    figure, axis = plt.subplots(figsize=(8, 5.5), constrained_layout=True)
    points = axis.scatter(
        [float(row["structural_length_mm"]) for row in rows],
        [float(row["torque_ratio_max"]) for row in rows],
        c=[float(row["phi_safe_deg"]) for row in rows], cmap="viridis", s=36,
    )
    figure.colorbar(points, ax=axis, label="phi_safe_max [deg]  (maximize)")
    axis.set(xlabel="OA + BC [mm]  (maximize)",
             ylabel="worst torque ratio k_max over the usable range  (minimize)",
             title="Stage-0 Pareto front")
    axis.grid(True, color="#e6e8eb")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=140)
    print(f"saved {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
