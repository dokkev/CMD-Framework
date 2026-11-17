#!/usr/bin/env python3
"""Study 1 with the link-in-front occlusion rule on and off: counts, the subset check and the appendix figure.

    python scripts/study/study1_fixed_morphology/summarize_faceoff.py

Reads the two runs of the same Study 1, ``config.yaml`` (ON, the study's rule) and ``config_faceoff.yaml``
(OFF, ``goal.loss.links_block_face: false`` on every reference). They differ in that rule only. Collisions,
ROM and the fold rule are the same, and links through a pad block count as routed around it in both. OFF is
an idealized comparison condition: no link geometry was designed or checked for it.

Writes ``faceoff_counts.csv`` and ``faceoff_changes.csv`` to the OFF run's folder and
``report/figures/study1_faceoff/study1_faceoff.png``. Stops if any assembly feasible ON fails OFF
(F_ON must be a subset of F_OFF).
"""

from __future__ import annotations

import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

ROOT = Path(__file__).resolve().parents[4]
DESIGN = ROOT / "design"
ON = DESIGN / "output/study/study1_fixed_morphology_tilt_sweep"
OFF = DESIGN / "output/study/study1_fixed_morphology_tilt_sweep_faceoff"
FIGURES = ROOT / "report/figures/study1_faceoff"
FLEXIONS = (90.0, 120.0, 150.0)
# The comparison set: each architecture's physical actuations at the nominal CoDe tilt, then the bound.
REFERENCES = [
    ("code_matched__tilt15", "CoDe (15°)"),
    ("belt_2r_matched__tilt15", "Belt 2R (15°)"),
    ("2r_dd_tilt0_matched__tilt0", "2R-DD (0°)"),
    ("2r_dd_tilt0_matched__tilt90", "2R-DD (90°)"),
    ("rcj_tilt0_matched__tilt0", "RCJ (0°)"),
    ("rcj_tilt0_matched__tilt90", "RCJ (90°)"),
    ("rcj_ideal_matched", "Ideal RCJ (bound)"),
]
INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e4e2dc"
COLOR_ON, COLOR_OFF = "#2a78d6", "#eb6834"
CELL = {"both": "#9ec5f4", "off_only": COLOR_OFF, "neither": "#f1efea", "not_mounted": "#ffffff"}


def load(folder: Path) -> dict[str, dict[str, str]]:
    with (folder / "assembly_results.csv").open(encoding="utf-8") as stream:
        return {row["assembly_id"]: row for row in csv.DictReader(stream)}


def feasible(row) -> bool:
    return row["feasible"] == "True"


def main() -> int:
    on, off = load(ON), load(OFF)
    if on.keys() != off.keys():
        raise SystemExit("the ON and OFF runs do not hold the same assemblies")
    broken = sorted(k for k in on if feasible(on[k]) and not feasible(off[k]))
    if broken:
        raise SystemExit(f"F_ON is not a subset of F_OFF: {broken[:5]} ({len(broken)})")

    counts = defaultdict(lambda: [0, 0, 0])
    changes = []
    for key, row in on.items():
        cell = counts[(row["reference_id"], float(row["required_flexion_deg"]))]
        cell[0] += feasible(row)
        cell[1] += feasible(off[key])
        cell[2] += row["mounted"] == "True"
        if feasible(off[key]) and not feasible(row):
            changes.append({k: row[k] for k in ("reference_id", "mechanism", "required_flexion_deg", "module_id",
                                                "proximal_length_mm", "proximal_thickness_mm",
                                                "distal_length_mm", "distal_thickness_mm")}
                           | {"on_failure": row["failure"],
                              "on_proximal_effective_mm": row["proximal_effective_mm"],
                              "on_distal_effective_mm": row["distal_effective_mm"],
                              "off_proximal_effective_mm": off[key]["proximal_effective_mm"],
                              "off_distal_effective_mm": off[key]["distal_effective_mm"]})
    rows = [{"reference_id": ref, "required_flexion_deg": flexion, "mounted": m, "n_on": a, "n_off": b,
             "delta_n": b - a} for (ref, flexion), (a, b, m) in sorted(counts.items())]
    for name, table in (("faceoff_counts.csv", rows), ("faceoff_changes.csv", changes)):
        with (OFF / name).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(table[0]))
            writer.writeheader()
            writer.writerows(table)

    figure(on, off, counts)
    print(f"{'reference':22s}" + "".join(f"{f:>16.0f}°" for f in FLEXIONS))
    for ref, label in REFERENCES:
        print(f"{label:22s}" + "".join(f"{counts[(ref, f)][0]:>8d} -> {counts[(ref, f)][1]:<3d}({counts[(ref, f)][1] - counts[(ref, f)][0]:+d})"
                                        for f in FLEXIONS))
    print(f"wrote {OFF / 'faceoff_counts.csv'}, {OFF / 'faceoff_changes.csv'}, {FIGURES / 'study1_faceoff.png'}")
    return 0


def figure(on, off, counts) -> None:
    plt.rcParams.update({"font.size": 8, "font.family": "DejaVu Sans", "axes.edgecolor": MUTED,
                         "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED, "text.color": INK})
    fig = plt.figure(figsize=(10.5, 6.6), facecolor="white")
    grid = fig.add_gridspec(2, 3, height_ratios=[1.0, 1.0], hspace=0.55, wspace=0.12)

    # (a): feasible module pairs, ON and OFF, at each required flexion.
    for column, flexion in enumerate(FLEXIONS):
        axis = fig.add_subplot(grid[0, column])
        for y, (ref, _) in enumerate(REFERENCES):
            a, b, _ = counts[(ref, flexion)]
            axis.plot([a, b], [y, y], color=GRID, linewidth=2, zorder=1)
            axis.scatter([a], [y], s=34, color=COLOR_ON, zorder=3, edgecolor="white", linewidth=1)
            axis.scatter([b], [y], s=34, color=COLOR_OFF, zorder=2, marker="D", edgecolor="white", linewidth=1)
            if b != a:
                axis.annotate(f"+{b - a}", (b, y), xytext=(6, 0), textcoords="offset points", va="center",
                              fontsize=8, color=INK)
        axis.set_yticks(range(len(REFERENCES)))
        axis.set_yticklabels([label for _, label in REFERENCES] if column == 0 else [])
        axis.invert_yaxis()
        axis.set_xlim(-1, 44)
        axis.set_title(f"Required flexion {flexion:.0f}°", fontsize=9, loc="left")
        axis.set_xlabel("Feasible module pairs (of 71)")
        axis.grid(axis="x", color=GRID, linewidth=0.6)
        axis.set_axisbelow(True)
        for side in ("top", "right"):
            axis.spines[side].set_visible(False)
    fig.legend(handles=[plt.Line2D([], [], marker="o", color=COLOR_ON, linestyle="", label="ON: link-in-front occlusion"),
                        plt.Line2D([], [], marker="D", color=COLOR_OFF, linestyle="", label="OFF: occlusion ignored")],
               loc="upper right", bbox_to_anchor=(0.98, 0.995), ncol=2, frameon=False)
    fig.text(0.01, 0.975, "(a)  Feasible module pairs at a fixed morphology", fontsize=10, weight="bold")

    # (b): CoDe's accommodation map, ON vs OFF, at 120° and 150°: groups x thickness x length.
    lengths = (15, 20, 25, 30, 35, 40, 45)
    thicknesses = (3, 5, 8)
    sub = grid[1, :].subgridspec(2, 3, hspace=0.35, wspace=0.12)
    for r, flexion in enumerate((120.0, 150.0)):
        for c, group in enumerate(("proximal", "distal", "combined")):
            axis = fig.add_subplot(sub[r, c])
            for i, thickness in enumerate(thicknesses):
                for j, length in enumerate(lengths):
                    key = f"code_matched__tilt15__{group}_L{length}_T{thickness}__phi{flexion:g}"
                    a, b = on[key], off[key]
                    state = ("not_mounted" if a["mounted"] != "True" else "both" if feasible(a)
                             else "off_only" if feasible(b) else "neither")
                    axis.add_patch(plt.Rectangle((j, i), 0.94, 0.9, color=CELL[state],
                                                 ec=GRID if state == "not_mounted" else None, lw=0.6))
                    if state == "off_only":
                        axis.text(j + 0.47, i + 0.45, "+", ha="center", va="center", color="white", fontsize=9,
                                  weight="bold")
            axis.set_xlim(0, len(lengths))
            axis.set_ylim(0, len(thicknesses))
            axis.set_xticks([j + 0.47 for j in range(len(lengths))])
            axis.set_xticklabels(lengths if r == 1 else [])
            axis.set_yticks([i + 0.45 for i in range(len(thicknesses))])
            axis.set_yticklabels(thicknesses if c == 0 else [])
            axis.tick_params(length=0)
            for spine in axis.spines.values():
                spine.set_visible(False)
            if r == 0:
                axis.set_title(f"{group.capitalize()} pad varied", fontsize=9, loc="left")
            if r == 1:
                axis.set_xlabel("Module length (mm)")
            if c == 0:
                axis.set_ylabel(f"{flexion:.0f}°\nthickness (mm)")
    fig.text(0.01, 0.47, "(b)  CoDe (15°): which module pairs are carried, ON vs OFF", fontsize=10, weight="bold")
    fig.legend(handles=[Patch(color=CELL["both"], label="feasible ON and OFF"),
                        Patch(color=CELL["off_only"], label="feasible OFF only (+)"),
                        Patch(color=CELL["neither"], label="infeasible in both"),
                        Patch(facecolor=CELL["not_mounted"], edgecolor=GRID, label="does not mount")],
               loc="upper right", bbox_to_anchor=(0.98, 0.485), ncol=4, frameon=False)
    fig.text(0.01, 0.005, "2D planar model. OFF ignores links in front of a contact face; it is an idealized "
             "condition, not a designed or verified link routing.\nOther checks (collisions, ROM, fold) "
             "unchanged. In (a) a single dot means ON = OFF.",
             fontsize=7.5, color=MUTED)
    FIGURES.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURES / "study1_faceoff.png", dpi=200, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
