#!/usr/bin/env python3
"""Study 1's designs, drawn: for one required flexion, each architecture's
best design for each pad thickness, and its pad coverage.

    python scripts/study/study1_thickness_coverage/figure_designs.py [--flexions 90,120,150]

(a) Columns: architectures; rows: pad thickness. Each panel the design for
that thickness, drawn at the required flexion, its pad coverage
``(l_p + l_d) / L_f`` under it; dotted: the distal segment's centre of
rotation from extension. One scale for every panel. A condition with no
feasible design is left empty. (b) Pad coverage against the thickness, one
line per architecture (dashed: the idealized upper bounds).

The search maximizes the cylinder coverage ``C_pad`` (summary.csv,
``figure.py``); the pad coverage shown here is the more direct reading.
Writes ``report/study1/figures/study1_designs_q<q>.png``.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.gridspec import GridSpec  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT, ROOT / "scripts" / "study", ROOT / "scripts" / "study" / "study2_morphology_adaptation"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import one_finger  # noqa: E402
from core.optimization.mechanism import read_config_file  # noqa: E402
from visualize import sketch  # noqa: E402
from visualize.finger import view_of  # noqa: E402
from visualize.style import COLORS, GRID, INK, MARKERS, MUTED, use  # noqa: E402

OUT = ROOT.parent / "report" / "study1" / "figures"
SUMMARY = ROOT / "output" / "study" / "study1_thickness_coverage" / "summary.csv"
ARCH = [("cfl", "CFL"), ("2r_remote", "2R-remote"), ("2r_dd", "2R-DD"), ("rcj", "RCJ"),
        ("cfl_ideal", "CFL ideal*"), ("rcj_ideal", "RCJ ideal*")]
WIDTH = 7.16  # in: a double column
LABEL_MM = 11.0  # a strip under every drawing for its coverage


def pad_coverages() -> dict[tuple[str, int, int], float]:
    with SUMMARY.open(encoding="utf-8", newline="") as stream:
        return {(r["architecture"], int(float(r["required_flexion_deg"])), int(float(r["thickness_mm"]))):
                float(r["pad_coverage"]) for r in csv.DictReader(stream)}


def draw(flexion: int, config, coverage) -> Path:
    thicknesses = [int(h) for h in config["thickness_mm"]]
    designs = {}
    for key, label in ARCH:
        for thickness, reference in one_finger.study1_designs(key, config["architectures"][key], float(flexion),
                                                              config).items():
            found = coverage.get((label, flexion, int(thickness)))
            if found is not None:  # the summary leaves out a condition without a feasible design
                designs[(key, int(thickness))] = (reference, view_of(reference.mechanism, reference.config,
                                                                     reference.x), found)
    probe_figure, probe = plt.subplots()
    for _, view, _ in designs.values():
        view.draw(probe, float(flexion), hit=set(), style="sketch")
    box = probe.dataLim
    plt.close(probe_figure)
    xlim, ylim = (box.x0 - 1.5, box.x1 + 1.5), (box.y0 - 1.5 - LABEL_MM, box.y1 + 1.5)
    panel_w = WIDTH / len(ARCH)
    panel_h = panel_w * (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])
    chart_h = 1.3
    height = 0.28 + len(thicknesses) * panel_h + 0.42 + chart_h + 0.3
    figure = plt.figure(figsize=(WIDTH, height))
    top = 1.0 - 0.28 / height
    sketches = GridSpec(len(thicknesses), len(ARCH), figure=figure, left=0.045, right=0.995, top=top,
                        bottom=top - len(thicknesses) * panel_h / height, wspace=0.0, hspace=0.0)
    colors = {key: COLORS[config["architectures"][key]["mechanism"]] for key, _ in ARCH}
    for b, (key, label) in enumerate(ARCH):
        for a, thickness in enumerate(thicknesses):
            axis = figure.add_subplot(sketches[a, b])
            sketch.frame(axis, xlim, ylim)
            centre = 0.5 * (xlim[0] + xlim[1])
            if (key, thickness) in designs:
                _, view, value = designs[(key, thickness)]
                view.draw(axis, float(flexion), hit=set(), style="sketch")
                sketch.icr_path(axis, view.icr(np.linspace(0.0, float(flexion), 61)), size=6, linewidth=0.7)
                axis.text(centre, ylim[0] + 0.45 * LABEL_MM, f"{100 * value:.0f}%", ha="center", va="center",
                          fontsize=8, fontweight="bold", color=INK)
            else:
                axis.text(centre, 0.5 * (ylim[0] + ylim[1]), "no feasible\ndesign", ha="center", va="center",
                          fontsize=7, color=MUTED)
            if a == 0:
                axis.set_title(label, fontsize=8, color=colors[key], fontweight="bold", pad=2)
            if b == 0:
                axis.text(-0.06, 0.5, f"{thickness} mm", transform=axis.transAxes, rotation=90, ha="center",
                          va="center", fontsize=8, color=INK)
    split = sketches[0, 4].get_position(figure).x0  # the idealized columns set apart
    figure.add_artist(plt.Line2D([split, split], [sketches[-1, 4].get_position(figure).y0 + 0.01, top - 0.005],
                                 transform=figure.transFigure, color=GRID, linewidth=0.8))
    figure.text(0.006, top, "(a)", fontsize=9, fontweight="bold", va="top")
    figure.text(0.012, top - len(thicknesses) * panel_h / height / 2, "pad thickness", rotation=90, fontsize=8,
                va="center", ha="center", color=MUTED)

    chart = figure.add_axes((0.085, 0.3 / height + 0.28 / height, 0.62, chart_h / height))
    for key, label in ARCH:
        spec = config["architectures"][key]
        points = [(h, 100 * designs[(key, h)][2]) for h in thicknesses if (key, h) in designs]
        if points:
            chart.plot(*zip(*points), color=colors[key], marker=MARKERS.get(spec["mechanism"], "o"), markersize=4.5,
                       linewidth=1.4, linestyle=(0, (3, 2)) if spec.get("upper_bound") else "-", label=label)
    chart.set_xticks(thicknesses, [f"{h} mm" for h in thicknesses])
    chart.set_xlim(thicknesses[0] - 0.6, thicknesses[-1] + 0.6)
    chart.set_ylim(45, 88)
    chart.set_xlabel(f"pad thickness ($q_{{req}}$ = {flexion}°)", fontsize=8)
    chart.set_ylabel("pad coverage (%)", fontsize=8)
    chart.tick_params(labelsize=7, length=2)
    chart.grid(color=GRID, linewidth=0.6)
    chart.set_axisbelow(True)
    for side in ("top", "right"):
        chart.spines[side].set_visible(False)
    chart.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), frameon=False, fontsize=7, handlelength=2.6)
    figure.text(0.006, (0.3 + 0.28 + chart_h) / height + 0.01, "(b)", fontsize=9, fontweight="bold", va="top")
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"study1_designs_q{flexion}.png"
    figure.savefig(path, dpi=300)
    plt.close(figure)
    return path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--flexions", default="90,120,150")
    args = parser.parse_args(argv)
    use("paper")
    config, coverage = read_config_file(one_finger.DEFAULT_CONFIG), pad_coverages()
    for flexion in (int(q) for q in args.flexions.split(",")):
        print(draw(flexion, config, coverage))
    return 0


if __name__ == "__main__":
    sys.exit(main())
