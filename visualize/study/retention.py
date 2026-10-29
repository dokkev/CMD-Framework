"""Plot contact-retention runs: the fronts by required ROM, and the maximin designs.

    python -m visualize.study.retention output/open_chain_packaged/run_001 output/rolling_contact_joint/run_001 ...

Runs are grouped by mechanism and required ROM (runs of one group are pooled).
``fronts.png``: ``R* = max min(r_p, r_d)`` against the required ROM, and each
ROM's ``r_p``-``r_d`` fronts, the maximin designs starred. ``designs.png``: each
group's maximin design, extended (grey) and at the required ROM (in colour),
with its kept modules (filled) inside their nominal envelopes (dashed).
Candidates are as the runs saved them, on the optimizer's grid.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Polygon as PolygonPatch

from core.optimization.retention import RetentionMechanism, front_retention
from core.optimization.retention.geometry import Disk, Housing, Polygon, Segment
from core.optimization.study import load_run
from visualize.style import DISTAL_PAD, LIGHT_GRID, OUTPUT, PROXIMAL_PAD, SERVO

COLORS = {"open_chain_idealized": "#94a3b8", "open_chain_packaged": "#2563eb",
          "rolling_contact_joint": "#e11d48", "linkage": "#16a34a"}
LABELS = {"open_chain_idealized": "Idealized 2R", "open_chain_packaged": "Packaged 2R",
          "rolling_contact_joint": "Packaged RCJ (provisional)", "linkage": "Packaged CoDe"}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=OUTPUT / "retention")
    args = parser.parse_args(argv)

    groups: dict[tuple[str, int], dict] = {}
    for directory in args.runs:
        run = load_run(directory)
        if not isinstance(run.mechanism, RetentionMechanism):
            parser.error(f"{directory} is not a retention run")
        config = run.config
        rom = round(math.degrees(config.required_rom_rad))
        group = groups.setdefault((run.mechanism.name, rom),
                                  {"mechanism": run.mechanism, "config": config, "rows": []})
        group["rows"].extend(run.candidates())
    roms = sorted({rom for _, rom in groups})
    names = [name for name in COLORS if any(name == key[0] for key in groups)]
    args.output.mkdir(parents=True, exist_ok=True)
    plot_fronts(groups, roms, names, args.output / "fronts.png")
    plot_designs(groups, roms, names, args.output / "designs.png")
    print(f"{'mechanism':24s} " + " ".join(f"{rom:>6d}°" for rom in roms) + "   R* = max min(r_p, r_d)")
    for name in names:
        cells = [f"{front_retention(groups[(name, rom)]['rows']):7.3f}" if (name, rom) in groups else "      -"
                 for rom in roms]
        print(f"{name:24s} " + " ".join(cells))
    print(f"saved {args.output / 'fronts.png'} and {args.output / 'designs.png'}")
    return 0


def maximin(rows):
    return max(rows, key=lambda r: (min(float(r["proximal_retention"]), float(r["distal_retention"])),
                                    float(r["proximal_retention"]) + float(r["distal_retention"])),
               default=None)


def plot_fronts(groups, roms, names, path: Path) -> None:
    figure, axes = plt.subplots(1, len(roms) + 1, figsize=(4.2 * (len(roms) + 1), 4.2),
                                constrained_layout=True)
    summary = axes[0]
    for name in names:
        values = [front_retention(groups[(name, rom)]["rows"]) if (name, rom) in groups else np.nan
                  for rom in roms]
        summary.plot(roms, values, "o-", color=COLORS[name], label=LABELS[name])
    summary.set(xlabel="required ROM [deg]", ylabel="R* = max min(r_p, r_d)", ylim=(-0.03, 1.03),
                xticks=roms, title="Retention vs required ROM")
    summary.legend(fontsize=8, loc="lower left")
    for axis, rom in zip(axes[1:], roms):
        for name in names:
            rows = groups.get((name, rom), {}).get("rows", [])
            if not rows:
                axis.plot([], [], "x", color=COLORS[name], label=f"{LABELS[name]}: infeasible")
                continue
            rp = [float(r["proximal_retention"]) for r in rows]
            rd = [float(r["distal_retention"]) for r in rows]
            order = np.argsort(rp)
            axis.plot(np.array(rp)[order], np.array(rd)[order], ".-", color=COLORS[name], alpha=0.8,
                      label=f"{LABELS[name]}: R*={front_retention(rows):.2f}")
            best = maximin(rows)
            axis.plot(float(best["proximal_retention"]), float(best["distal_retention"]), "*",
                      color=COLORS[name], markersize=14, markeredgecolor="black")
        axis.plot([0, 1], [0, 1], ":", color="#9ca3af", linewidth=0.8)
        axis.set(xlim=(-0.03, 1.03), ylim=(-0.03, 1.03), xlabel="r_p (proximal kept / nominal)",
                 ylabel="r_d (distal kept / nominal)", title=f"Required ROM {rom}°", aspect="equal")
        axis.legend(fontsize=7, loc="lower left")
        for axis_ in (axis, summary):
            axis_.grid(True, color=LIGHT_GRID)
    figure.savefig(path, dpi=140)
    plt.close(figure)


def pose_solids(mechanism, config, x, flexion_rad: float):
    """The design's solids at one distal flexion, and its kept and nominal modules."""

    posed = dataclasses.replace(config, required_rom_rad=max(flexion_rad, 1e-9), rom_step_rad=max(flexion_rad, 1e-9))
    geometry = dict(zip(mechanism.geometry_names, x))
    kept = mechanism.module_outlines(x, geometry, posed)
    full = tuple(dataclasses.replace(m, line_mm=m.nominal_mm, palmar_mm=m.nominal_mm) for m in kept)
    return (mechanism.solids(geometry, kept, posed).solids,
            mechanism.solids(geometry, full, posed).solids)


def draw(axis, solids, nominal, packaging, colour: str, faded: bool) -> None:
    alpha = 0.25 if faded else 1.0
    for name, solid in solids.items():
        if isinstance(solid, Polygon):
            fill = PROXIMAL_PAD if name.startswith("proximal") else DISTAL_PAD
            outline = nominal[name].points[-1]
            axis.add_patch(PolygonPatch(outline, closed=True, fill=False, linestyle="--",
                                        edgecolor="#6b7280", linewidth=0.7, alpha=alpha))
            axis.add_patch(PolygonPatch(solid.points[-1], closed=True, facecolor=fill,
                                        edgecolor="#111111", linewidth=0.8, alpha=alpha))
        elif isinstance(solid, Segment):
            start, end = solid.start[-1], solid.end[-1]
            axis.plot([start[0], end[0]], [start[1], end[1]], color=colour if not faded else "#9ca3af",
                      linewidth=2.2, alpha=alpha, solid_capstyle="round")
        elif isinstance(solid, Disk):
            axis.add_patch(Circle(solid.centers[-1], solid.radius_mm, fill=False,
                                  edgecolor=colour if not faded else "#9ca3af", linewidth=1.0, alpha=alpha))
        elif isinstance(solid, Housing):
            axis.add_patch(PolygonPatch(solid.footprint.outline, closed=True, facecolor=SERVO,
                                        edgecolor="#4b5563", linewidth=0.8, alpha=0.8 * alpha))


def plot_designs(groups, roms, names, path: Path) -> None:
    figure, axes = plt.subplots(len(names), len(roms), figsize=(3.6 * len(roms), 3.4 * len(names)),
                                squeeze=False, constrained_layout=True)
    for row, name in enumerate(names):
        for column, rom in enumerate(roms):
            axis = axes[row][column]
            axis.set_aspect("equal")
            axis.set(xlim=(-30, 115), ylim=(-75, 40))
            axis.tick_params(labelsize=7)
            group = groups.get((name, rom))
            best = maximin(group["rows"]) if group else None
            if column == 0:
                axis.set_ylabel(LABELS[name], fontsize=9)
            if best is None:
                axis.set_title(f"{rom}°: no feasible design", fontsize=9)
                axis.text(0.5, 0.5, "infeasible", transform=axis.transAxes, ha="center", va="center",
                          color="#9ca3af", fontsize=14)
                continue
            mechanism, config = group["mechanism"], group["config"]
            x = mechanism.read_design(best)
            packaging = config.packaging
            if getattr(mechanism, "packaged", True):
                axis.add_patch(PolygonPatch(packaging.mcp_footprint().outline, closed=True,
                                            facecolor="#e5e7eb", edgecolor="#6b7280", linewidth=0.8))
            for flexion, faded in ((0.0, True), (config.required_rom_rad, False)):
                kept, nominal = pose_solids(mechanism, config, x, flexion)
                draw(axis, kept, nominal, packaging, COLORS[name], faded)
            axis.set_title(f"{rom}°: r_p {float(best['proximal_retention']):.2f}, "
                           f"r_d {float(best['distal_retention']):.2f}", fontsize=9)
            axis.grid(True, color="#f1f2f4")
    figure.suptitle("Maximin designs: extended (grey) and at the required ROM; dashed: nominal module",
                    fontsize=10)
    figure.savefig(path, dpi=140)
    plt.close(figure)


if __name__ == "__main__":
    raise SystemExit(main())
