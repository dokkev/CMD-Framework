"""How hard pads are to fit: the linkage against the serial 2R on one goal.

    python -m visualize.comparison.pad_fit output/linkage/<study>/portfolio_001 \\
        output/revolute/<study>/portfolio_001 output/comparison/coverage_by_flexion.csv \\
        --at-deg 120 --output output/comparison.png

Each argument is a merged, verified front (``run.py`` with several seeds):
its ``seeds.csv`` and the runs ``portfolio.json`` lists. Both must share the
goal (the 2R may optimize fewer objectives: its ``k`` is 1).

- Left: the most effective pad coverage each mechanism keeps up to each
  flexion (``scripts/optimize/coverage_by_flexion.py``'s table: pads judged on ``[0, phi]``),
  the linkage's points labelled with their ``k_max``.
- Right: the linkage's price for it, ``k_max`` against the coverage (the
  2R drives its joint directly, ``k = 1``).
- Bottom: at ``--at-deg``, each mechanism's design of that table,
  extended (faint) and flexed there, MCP at 0.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from core.morphology.linkage import FingerLinkage  # noqa: E402
from core.morphology.linkage.packaging import distal_pad_polygon, proximal_pad_polygon  # noqa: E402
from core.optimization.linkage.design import decode  # noqa: E402
from core.optimization.revolute import geometry  # noqa: E402
from visualize import solids  # noqa: E402
from visualize.fronts import column, load_front  # noqa: E402
from visualize.motion import index_at, servo_sweep  # noqa: E402
from visualize.style import (COLORS, DISTAL_PAD, GRID, INK, MUTED, NAMES, OUTPUT, PROXIMAL_PAD,  # noqa: E402
                             SERVO, axes_style, use)


def envelope(rows) -> tuple[np.ndarray, np.ndarray]:
    """The most coverage at each flexion or more: a step down in ``phi_max``."""

    phi, coverage = column(rows, "phi_max_deg"), column(rows, "pad_coverage")
    order = np.argsort(-phi)
    best = np.maximum.accumulate(coverage[order])
    return phi[order], best


def best_at(rows, phi_deg: float) -> dict | None:
    """The design with the most coverage among those reaching ``phi_deg``."""

    reaching = [row for row in rows if float(row["phi_max_deg"]) >= phi_deg - 1e-9]
    return max(reaching, key=lambda row: float(row["pad_coverage"]), default=None)


def load_table(path: Path) -> dict[str, list[dict]]:
    """``coverage_by_flexion.py``'s table, by mechanism, in flexion order."""

    table: dict[str, list[dict]] = {}
    for row in csv.DictReader(path.open(encoding="utf-8", newline="")):
        table.setdefault(row["mechanism"], []).append(row)
    for rows in table.values():
        rows.sort(key=lambda row: float(row["flexion_deg"]))
    return table


def goal_of(front):
    config = front["config"]
    return config if front["mechanism"].name == "linkage" else config.goal


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("linkage", type=Path, help="the linkage's merged front (portfolio_NNN)")
    parser.add_argument("revolute", type=Path, help="the 2R's merged front (portfolio_NNN)")
    parser.add_argument("table", type=Path, help="coverage_by_flexion.py's CSV")
    parser.add_argument("--at-deg", type=float, default=120.0, help="flexion the designs are compared at")
    parser.add_argument("--output", type=Path, default=OUTPUT / "comparison.png")
    args = parser.parse_args(argv)

    fronts = {"linkage": load_front(args.linkage), "revolute": load_front(args.revolute)}
    for name, front in fronts.items():
        if front["mechanism"].name != name:
            parser.error(f"{name}: {front['mechanism'].name} front given")
    goal = goal_of(fronts["linkage"])
    other = dataclasses.replace(goal_of(fronts["revolute"]), seeds=goal.seeds, objectives=goal.objectives)
    if other != goal:  # the search settings aside
        print("warning: the two fronts were not run on the same goal", file=sys.stderr)

    use("report")
    figure = plt.figure(figsize=(13, 10.5), facecolor="white")
    grid = figure.add_gridspec(2, 2, height_ratios=(1.0, 1.1), hspace=0.3, wspace=0.32)
    table = load_table(args.table)
    plot_coverage(figure.add_subplot(grid[0, 0]), table, args.at_deg)
    plot_price(figure.add_subplot(grid[0, 1]), fronts, math.degrees(goal.required_rom_rad))
    for index, name in enumerate(("linkage", "revolute")):
        axis = figure.add_subplot(grid[1, index])
        row = next((r for r in table.get(name, []) if abs(float(r["flexion_deg"]) - args.at_deg) < 1e-6), None)
        if row is None:
            axis.text(0.5, 0.5, f"no design reaches {args.at_deg:.0f}°", ha="center", transform=axis.transAxes)
            axis.set_axis_off()
            continue
        row = {**row, "x": [float(v) for v in row["x"].split()]}
        (draw_linkage if name == "linkage" else draw_revolute)(axis, fronts[name], row, math.radians(args.at_deg))
        axis.set_title(f"{NAMES[name]}, most pad to {args.at_deg:.0f}°: coverage {float(row['pad_coverage']):.2f} "
                       f"(P {float(row['proximal_effective_mm']):.0f} + D {float(row['distal_effective_mm']):.0f} mm), "
                       f"k_max {float(row['torque_ratio_max']):.2f}", color=INK, fontsize=10, loc="left")
    low, high = goal.extended_length_bounds_mm
    figure.suptitle(f"How much pad fits up to a flexion, same goal: finger |OC| {low:.0f}-{high:.0f} mm, pads min {goal.minimum_proximal_support_mm:.0f}"
                    f" + {goal.minimum_distal_support_mm:.0f} mm (height {goal.packaging.proximal_pad.height_mm:.0f}),"
                    f" XM335 servos, no clearance",
                    color=INK, fontsize=11, x=0.06, ha="left")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=130, bbox_inches="tight")

    print(f"saved {args.output}")
    return 0


def _axes(axis) -> None:
    axes_style(axis, GRID, 0.8)


def plot_coverage(axis, table, at_deg) -> None:
    _axes(axis)
    for name in ("linkage", "revolute"):
        rows = table.get(name, [])
        phi, coverage = column(rows, "flexion_deg"), column(rows, "pad_coverage")
        axis.plot(phi, coverage, color=COLORS[name], linewidth=2.2, marker="o" if name == "linkage" else "s",
                  markersize=7, markeredgecolor="white", label=NAMES[name])
        if name == "linkage":
            for row in rows:
                axis.annotate(f"k {float(row['torque_ratio_max']):.1f}",
                              (float(row["flexion_deg"]), float(row["pad_coverage"])),
                              xytext=(0, 8), textcoords="offset points", ha="center", color=MUTED, fontsize=8)
        elif len(phi) and phi.max() < 180.0:
            step = float(np.diff(phi).min()) if len(phi) > 1 else 10.0
            axis.annotate(f"no design\nat {phi.max() + step:.0f}°", (phi.max(), coverage[-1]), xytext=(8, -18),
                          textcoords="offset points", color=COLORS[name], fontsize=9)
    axis.axvline(at_deg, color=MUTED, linewidth=1.0, linestyle="--")
    axis.set_xlabel("flexion the pads must stay usable to [deg]")
    axis.set_ylabel("most effective pad coverage  (higher is better)")
    axis.legend(loc="lower left", frameon=False, fontsize=9)


def plot_price(axis, fronts, required_deg) -> None:
    _axes(axis)
    for name, front in fronts.items():
        rows = front["rows"]
        axis.scatter(column(rows, "torque_ratio_max"), column(rows, "pad_coverage"), s=12,
                     color=COLORS[name], marker="o" if name == "linkage" else "s", alpha=0.55,
                     edgecolors="none", label=f"{NAMES[name]} ({len(rows)} designs)")
    axis.set_xlabel("k_max: servo torque per distal moment  (lower is better)")
    axis.set_ylabel(f"effective pad coverage on [0, {required_deg:.0f}°]  (higher is better)")
    axis.set_title(f"the fronts, {required_deg:.0f}° required", color=MUTED, fontsize=9, loc="left")
    axis.legend(loc="lower right", frameon=False, fontsize=9, markerscale=2.0)


def _frame(axis) -> None:
    solids.plane_axes(axis, 0.8)
    axis.set_xlabel("X / distal [mm]")
    axis.set_ylabel("Z / dorsal [mm]")


def draw_linkage(axis, front, row, phi_rad) -> None:
    """The linkage extended (faint) and where ``phi`` reaches ``phi_rad``."""

    config = front["config"]
    linkage = FingerLinkage(decode(row["x"], config.link_length_resolution_mm))
    packaging = config.packaging
    _, sweep = servo_sweep(linkage)
    index = int(index_at(sweep, phi_rad))
    solids.patch(axis, packaging.mcp_footprint().outline, SERVO, 0.9, zorder=0.5)
    solids.patch(axis, packaging.pip_footprint(linkage).outline, SERVO, 0.9, zorder=0.5)
    solids.patch(axis, proximal_pad_polygon(linkage, packaging), PROXIMAL_PAD, 0.9)
    for alpha, i in ((0.25, 0), (1.0, index)):
        b, c, bp = sweep.b[i], sweep.c[i], sweep.bp[i]
        solids.patch(axis, distal_pad_polygon(linkage, packaging, b[None], c[None])[0], DISTAL_PAD, 0.9 * alpha)
        solids.linkage_bars(axis, linkage, b, c, bp, COLORS["linkage"], alpha, width=2.2)
    _frame(axis)


def draw_revolute(axis, front, row, phi_rad) -> None:
    """The 2R extended (faint) and flexed by ``phi_rad``."""

    config = front["config"]
    geo = geometry(row["x"], config)
    packaging = config.goal.packaging
    solids.patch(axis, packaging.mcp_footprint().outline, SERVO, 0.9, zorder=0.5)
    solids.patch(axis, geo.distal_servo.outline, SERVO, 0.9, zorder=0.5)
    solids.patch(axis, geo.proximal_pad, PROXIMAL_PAD, 0.9)
    a = (geo.a_x_mm, 0.0)
    solids.line(axis, ((0.0, 0.0), a), COLORS["revolute"], width=2.2)
    for alpha, phi in ((0.25, 0.0), (1.0, phi_rad)):
        solids.patch(axis, geo.distal_pad([phi])[0], DISTAL_PAD, 0.9 * alpha)
        solids.line(axis, (a, geo.tip([phi])[0]), COLORS["revolute"], alpha, width=2.2)
    _frame(axis)


if __name__ == "__main__":
    raise SystemExit(main())
