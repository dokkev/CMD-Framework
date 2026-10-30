"""The accommodation study's results: the shortest finger each mechanism
needs to carry a module pair through each required flexion
(``scripts/optimize/accommodation``).

    python -m visualize.study.accommodation [output folder] [P] [D]

``P`` and ``D``: the module pair, ``<length>x<thickness>`` (default the
reference, 25x5 each). Left: ``L*`` (``|OC|``, extended) against the required
flexion, 2R and CoDe; whiskers: the seeds' own bests; a flexion where a
mechanism's runs found no feasible design marked at the top ("none found
within the bounds", not a proof that none exists). Right: each mechanism's shortest
finger at each required flexion, drawn at that flexion, one scale (each
column the same flexion). Saves output/accommodation/accommodation_P<P>_D<D>.png
and the numbers as .csv.
"""

from __future__ import annotations

import csv
import importlib.util
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from visualize import sketch  # noqa: E402
from visualize.finger import finger_from_row  # noqa: E402
from visualize.fronts import latest_portfolio, read_rows  # noqa: E402
from visualize.style import COLLIDE, COLORS, INK, MUTED, REPOSITORY_ROOT, axes_style, use  # noqa: E402

STUDY_DIR = REPOSITORY_ROOT / "scripts/optimize/accommodation"
NAMES = {"revolute": "2R (servo on the joint)", "linkage": "CoDe"}


def study():
    spec = importlib.util.spec_from_file_location("accommodation_configs", STUDY_DIR / "make_configs.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def module_of(text: str) -> tuple[int, int]:
    length, height = text.split("x")
    return int(length), int(height)


def shortest(config_dir: Path):
    """The verified front's shortest finger (its row) and each run's own shortest (``None``: no front)."""

    front = latest_portfolio(config_dir)
    if front is None:
        return None, []
    rows = read_rows(front)
    seeds = []
    for run in sorted(config_dir.glob("run_*")):
        pareto = read_rows(run / "pareto.csv") if (run / "pareto.csv").exists() else []
        if pareto:
            seeds.append(min(float(r["finger_length_mm"]) for r in pareto))
    return min(rows, key=lambda r: float(r["finger_length_mm"]), default=None), seeds


def main(argv: list[str]) -> int:
    use("paper")
    source = REPOSITORY_ROOT / (argv[0] if argv else "output/accommodation")
    configs = study()
    proximal = module_of(argv[1]) if len(argv) > 1 else configs.REFERENCE
    distal = module_of(argv[2]) if len(argv) > 2 else configs.REFERENCE
    flexions = configs.FLEXIONS_DEG
    mechanisms = configs.MECHANISMS
    figure = plt.figure(figsize=(13.5, 5.6), constrained_layout=True)
    grid = figure.add_gridspec(len(mechanisms), len(flexions) + 1, width_ratios=(1.6, *([1.0] * len(flexions))))
    curve = figure.add_subplot(grid[:, 0])
    axes_style(curve)
    table = []
    for r, mechanism in enumerate(mechanisms):
        points, missing = [], []
        for c, flexion in enumerate(flexions):
            stem = configs.name(mechanism, proximal, distal, flexion)
            config_dir = source / mechanism / stem
            axis = figure.add_subplot(grid[r, c + 1])
            sketch.frame(axis, (-14.0, 124.0), (-80.0, 40.0))
            if r == 0:
                axis.set_title(f"φ_req = {flexion}°", color=INK, fontsize=9)
            if c == 0:
                axis.text(-12.0, 37.0, NAMES[mechanism], color=COLORS[mechanism], fontsize=9, fontweight="bold",
                          va="top")
            row, seeds = shortest(config_dir)
            ran = latest_portfolio(config_dir) is not None
            entry = {"mechanism": mechanism, "flexion_deg": flexion, "proximal": f"{proximal[0]}x{proximal[1]}",
                     "distal": f"{distal[0]}x{distal[1]}", "runs": len(seeds),
                     "status": "not run" if not ran else ("none found" if row is None else "ok")}
            if row is None:
                none = entry["status"] == "none found"
                axis.text(0.5, 0.45, "no feasible design found\nwithin the tested bounds" if none else "not run yet",
                          transform=axis.transAxes, ha="center", color=COLLIDE if none else MUTED, fontsize=8)
                if none:
                    missing.append(flexion)
                table.append(entry)
                continue
            length = float(row["finger_length_mm"])
            finger = finger_from_row(mechanism, row, config_dir)
            finger.draw(axis, float(flexion))
            k = float(row["torque_ratio_max"])
            axis.text(0.02, 0.02, f"L* {length:.1f} mm, k {k:.2f}", transform=axis.transAxes, fontsize=7.5,
                      color=INK)
            points.append((flexion, length, min(seeds) if seeds else length, max(seeds) if seeds else length))
            table.append({**entry, "shortest_mm": f"{length:.2f}", "k_max": f"{k:.3f}",
                          "seed_min_mm": f"{min(seeds):.2f}" if seeds else "",
                          "seed_max_mm": f"{max(seeds):.2f}" if seeds else "",
                          "run": row["run"], "candidate": row["candidate"]})
        if points:
            f, length, low, high = (np.array(v) for v in zip(*points))
            curve.plot(f, length, color=COLORS[mechanism], marker="o", linewidth=1.8, label=NAMES[mechanism])
            curve.vlines(f, low, high, color=COLORS[mechanism], linewidth=1.0)
        for flexion in missing:
            curve.scatter([flexion], [157.0], marker="x", color=COLORS[mechanism], s=40, zorder=4)
            curve.text(flexion, 152.0, "none\nfound", color=COLORS[mechanism], fontsize=7, ha="center", va="top")
    curve.set_xticks(flexions)
    curve.set_xlabel("required flexion φ_req [deg]", color=INK)
    curve.set_ylabel("shortest finger L* = |OC| [mm]", color=INK)
    curve.set_ylim(40.0, 160.0)
    curve.legend(frameon=False, fontsize=8, loc="upper left")
    curve.set_title(f"modules: proximal {proximal[0]} x {proximal[1]} mm, distal {distal[0]} x {distal[1]} mm",
                    color=INK, fontsize=9, loc="left")
    figure.suptitle("Accommodation (comparison model): the shortest finger carrying both modules through φ_req; "
                    "clearance 0, links as 3 mm capsules, dorsal space 34 mm, MCP 0°",
                    color=MUTED, fontsize=8.5, x=0.01, ha="left")
    out_dir = source
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"accommodation_P{proximal[0]}x{proximal[1]}_D{distal[0]}x{distal[1]}.png"
    figure.savefig(out, dpi=150)
    columns = list(dict.fromkeys(key for row in table for key in row))
    with out.with_suffix(".csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(table)
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
