"""Paper figure (Scientific): what the coverage gain comes from, where it
grows, and (pending) what interfaces one morphology carries.

    python -m visualize.paper.fig_scientific [height_mm] [flexion_deg] [source]

(a) At one condition, the coverage as a waterfall: the 2R with its servo on
the joint, then moving the actuator off it (``revolute_parallel`` -
``revolute``), then the moving centre of rotation (``linkage`` -
``revolute_parallel``), to CoDe; the idealized 2R (no joint hardware) a
dashed line, the RCJ a point beside. Whiskers: the seeds' spread where
there are several. Each bar's ``k_max`` and extended centre of rotation's
share of |OC| below it. (b) Over the grid (interface thickness x required
flexion): the polycentric gain and the relocation gain, coverage points of
|OC| on one diverging scale about 0; a cell missing either design is grey
with an x. (c) The interface envelope: pending its definition (Checkpoint
C). PROVISIONAL while the loss rule and the runs are. Saves
output/paper/fig_scientific_h<h>_flex<f>.png / .pdf and the numbers as .csv.
"""

from __future__ import annotations

import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import TwoSlopeNorm  # noqa: E402

from visualize.paper.collect import STUDY, read  # noqa: E402
from visualize.style import COLLIDE, COLORS, GRID, INK, MUTED, SHORT_LABELS, axes_style, save, use  # noqa: E402

DIVERGING = "RdBu"
"""Red losses, blue gains, white 0."""


def cell(table, mechanism: str, height: int, flexion: int) -> dict | None:
    entry = next((r for r in table if r["mechanism"] == mechanism and r["thickness_mm"] == height
                  and r["flexion_deg"] == flexion), None)
    return entry if entry and entry.get("coverage_pct") is not None else None


def spread(entry: dict | None) -> tuple[float, float] | None:
    if entry is None or not entry.get("runs") or entry["runs"] < 2:
        return None
    return entry["seed_min_pct"], entry["seed_max_pct"]


def waterfall(axis, table, height: int, flexion: int) -> list[dict]:
    axes_style(axis, GRID)
    base, moved, code = (cell(table, m, height, flexion) for m in ("revolute", "revolute_parallel", "linkage"))
    ideal, rcj = cell(table, "revolute_idealized", height, flexion), cell(table, "rcj", height, flexion)
    rows = []
    steps = [("2R", base, None), ("+ actuator moved", moved, base), ("+ moving centre", code, moved), ("CoDe", code, None)]
    for i, (label, entry, before) in enumerate(steps):
        if entry is None or (before is None and label != "2R" and label != "CoDe"):
            axis.text(i, 2.0, "no design", ha="center", color=MUTED, fontsize=7, rotation=90)
            continue
        if before is None:  # a total
            mechanism = "revolute" if label == "2R" else "linkage"
            axis.bar(i, entry["coverage_pct"], color=COLORS[mechanism], width=0.6)
            whisker = spread(entry)
            if whisker:
                axis.vlines(i, *whisker, color=INK, linewidth=1.0)
            axis.text(i, entry["coverage_pct"] + 1.0, f"{entry['coverage_pct']:.0f}%", ha="center", fontsize=8)
            axis.text(i, -9.0, f"k {entry['k_max']:.2f}\nICR₀ {entry['icr0_share']:.2f}", ha="center",
                      fontsize=6.5, color=MUTED)
            rows.append({"step": label, "value_pct": entry["coverage_pct"]})
        elif before is not None:
            gain = entry["coverage_pct"] - before["coverage_pct"]
            low, high = sorted((before["coverage_pct"], entry["coverage_pct"]))
            axis.bar(i, high - low, bottom=low, color="#4f8fdc" if gain >= 0 else COLLIDE, width=0.6, alpha=0.85)
            # The gain's spread, conservatively: the seeds' worst and best combinations.
            a, b = spread(before), spread(entry)
            if a and b:
                axis.vlines(i, b[0] - a[1] + before["coverage_pct"], b[1] - a[0] + before["coverage_pct"],
                            color=INK, linewidth=1.0)
            axis.text(i, high + 1.0, f"{gain:+.1f}", ha="center", fontsize=8)
            mechanism = "revolute_parallel" if "actuator" in label else "linkage"
            axis.text(i, -9.0, f"k {entry['k_max']:.2f}\nICR₀ {entry['icr0_share']:.2f}", ha="center",
                      fontsize=6.5, color=MUTED)
            rows.append({"step": label, "value_pct": gain})
    if ideal:
        axis.axhline(ideal["coverage_pct"], color=COLORS["revolute_idealized"], linestyle=(0, (4, 2)), linewidth=1.2)
        axis.text(4.55, ideal["coverage_pct"] + 0.8, f"2R ideal {ideal['coverage_pct']:.0f}%", ha="right",
                  fontsize=7, color=COLORS["revolute_idealized"])
        rows.append({"step": "2R ideal (line)", "value_pct": ideal["coverage_pct"]})
    if rcj:
        axis.scatter([4.0], [rcj["coverage_pct"]], marker="^", color=COLORS["rcj"], s=40, zorder=4)
        axis.text(4.0, rcj["coverage_pct"] + 2.0, f"RCJ\n{rcj['coverage_pct']:.0f}%", ha="center", fontsize=7,
                  color=COLORS["rcj"])
        rows.append({"step": "RCJ (point)", "value_pct": rcj["coverage_pct"]})
    axis.set_xticks(range(5), ["2R", "+ actuator\nmoved", "+ moving\ncentre", "CoDe", ""], fontsize=7.5)
    axis.set_ylim(-14.0, 100.0)
    axis.set_xlim(-0.6, 4.6)
    axis.set_ylabel("contact interface (P + D) / |OC| [%]", color=INK)
    axis.set_title(f"(a) where the gain comes from: {height} mm, {flexion}°", color=INK, fontsize=9, loc="left")
    return rows


def gains(table, first: str, second: str) -> np.ndarray:
    heights, flexions = sorted(STUDY.THICKNESSES_MM), sorted(STUDY.FLEXIONS_DEG)
    values = np.full((len(heights), len(flexions)), np.nan)
    for r, h in enumerate(heights):
        for c, f in enumerate(flexions):
            a, b = cell(table, first, h, f), cell(table, second, h, f)
            if a and b:
                values[r, c] = b["coverage_pct"] - a["coverage_pct"]
    return values


def heatmap(axis, values: np.ndarray, title: str, norm) -> object:
    heights, flexions = sorted(STUDY.THICKNESSES_MM), sorted(STUDY.FLEXIONS_DEG)
    shown = axis.imshow(np.ma.masked_invalid(values), cmap=DIVERGING, norm=norm, origin="lower", aspect="auto")
    axis.set_facecolor("#e3e2dd")
    for r in range(len(heights)):
        for c in range(len(flexions)):
            if np.isnan(values[r, c]):
                axis.text(c, r, "×", ha="center", va="center", color=MUTED, fontsize=11)
            else:
                dark = abs(norm(values[r, c]) - 0.5) > 0.3
                axis.text(c, r, f"{values[r, c]:+.0f}", ha="center", va="center", fontsize=8,
                          color="white" if dark else INK)
    axis.set_xticks(range(len(flexions)), [f"{f}°" for f in flexions])
    axis.set_yticks(range(len(heights)), [f"{h} mm" for h in heights])
    axis.set_xlabel("required flexion", color=INK)
    axis.set_title(title, color=INK, fontsize=8.5, loc="left")
    return shown


def main(argv: list[str]) -> int:
    use("paper")
    height = int(argv[0]) if argv else 5
    flexion = int(argv[1]) if len(argv) > 1 else 150
    source = argv[2] if len(argv) > 2 else "comparison_quick"
    table = read(source)
    figure = plt.figure(figsize=(12.0, 3.9), constrained_layout=True)
    grid = figure.add_gridspec(1, 4, width_ratios=(1.35, 1.0, 1.0, 0.9))
    rows = waterfall(figure.add_subplot(grid[0]), table, height, flexion)
    polycentric = gains(table, "revolute_parallel", "linkage")
    relocation = gains(table, "revolute", "revolute_parallel")
    limit = max(5.0, float(np.nanmax(np.abs(np.concatenate((polycentric.ravel(), relocation.ravel()))))))
    norm = TwoSlopeNorm(vcenter=0.0, vmin=-limit, vmax=limit)
    left = figure.add_subplot(grid[1])
    shown = heatmap(left, polycentric, "(b) moving centre\nCoDe − 2R belt [%p]", norm)
    left.set_ylabel("interface thickness", color=INK)
    right = figure.add_subplot(grid[2])
    heatmap(right, relocation, "actuator moved\n2R belt − 2R [%p]", norm)
    right.set_yticklabels([])
    figure.colorbar(shown, ax=[left, right], shrink=0.8, label="gain [%p of |OC|]")
    pending = figure.add_subplot(grid[3])
    pending.axis("off")
    pending.text(0.5, 0.5, "(c) interface envelope\nat fixed morphology\n\npending its definition\n(Checkpoint C)",
                 ha="center", va="center", color=MUTED, fontsize=9, transform=pending.transAxes)
    figure.suptitle(f"PROVISIONAL: {table[0]['source']} (one seed each, unconverged), loss rule "
                    f"{table[0]['loss_rule']}", color=COLLIDE, fontsize=8.5, x=0.01, ha="left")
    out = save(figure, f"fig_scientific_h{height}_flex{flexion}")
    heights, flexions = sorted(STUDY.THICKNESSES_MM), sorted(STUDY.FLEXIONS_DEG)
    with out.with_suffix(".csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["panel", "item", "thickness_mm", "flexion_deg", "value_pct"])
        for row in rows:
            writer.writerow(["a", row["step"], height, flexion, f"{row['value_pct']:.3f}"])
        for name, values in (("b_polycentric", polycentric), ("b_relocation", relocation)):
            for r, h in enumerate(heights):
                for c, f in enumerate(flexions):
                    writer.writerow([name, "", h, f, "" if np.isnan(values[r, c]) else f"{values[r, c]:.3f}"])
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
