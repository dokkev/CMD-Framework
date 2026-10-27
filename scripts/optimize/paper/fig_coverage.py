#!/usr/bin/env python3
"""Paper figure: at one extended finger length, the share of it each
mechanism can give to contact interfaces usable all through the required
flexion, one panel per interface thickness.

    python scripts/optimize/paper/fig_coverage.py

Each point is the verified front's best ``(P + D) / |OC|`` of one interface
coverage config (scripts/optimize/interface_coverage); the whisker spans the
three seeds' own bests (their runs' fronts, on the optimization grid). An x
on the axis: no design carries the interfaces there. Saves
output/paper/fig_coverage.png / .pdf and the table as .csv.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "interface_coverage"))
import fingers  # noqa: E402
from make_configs import PAD_MIN_MM, FINGER_RANGE_MM, FLEXIONS_DEG, THICKNESSES_MM  # noqa: E402
from style import COLORS, DASHES, INK, MARKERS, MUTED, axes_style, save  # noqa: E402


def table() -> list[dict]:
    rows = []
    for mechanism in fingers.MECHANISMS:
        for height in THICKNESSES_MM:
            for flexion in FLEXIONS_DEG:
                row = fingers.best_row(mechanism, height, flexion)
                seeds = fingers.seed_bests(mechanism, height, flexion)
                rows.append({
                    "mechanism": mechanism, "pad_height_mm": height, "flexion_deg": flexion,
                    "pad_total_mm": fingers.pad_total(row),
                    "coverage_pct": None if row is None else 100.0 * float(row["pad_coverage"]),
                    "finger_length_mm": None if row is None else float(row["finger_length_mm"]),
                    "proximal_mm": None if row is None else float(row["proximal_effective_mm"]),
                    "distal_mm": None if row is None else float(row["distal_effective_mm"]),
                    "k_max": None if row is None else float(row["torque_ratio_max"]),
                    # Each seed's best, as a share of |OC| (its runs keep |OC| within FINGER_RANGE_MM).
                    "seed_min_pct": None if not seeds else 100.0 * min(seeds) / FINGER_RANGE_MM[1],
                    "seed_max_pct": None if not seeds else 100.0 * max(seeds) / FINGER_RANGE_MM[1],
                    "has_front": fingers.latest_front(mechanism, height, flexion) is not None,
                })
    return rows


def main() -> int:
    rows = table()
    heights = sorted(THICKNESSES_MM)
    figure, axes = plt.subplots(1, len(heights), figsize=(10.5, 3.6), sharey=True, constrained_layout=True)
    for axis, height in zip(axes, heights):
        axes_style(axis)
        for mechanism in fingers.MECHANISMS:
            mine = sorted((r for r in rows if r["mechanism"] == mechanism and r["pad_height_mm"] == height),
                          key=lambda r: r["flexion_deg"])
            found = [r for r in mine if r["coverage_pct"] is not None]
            if not found and not any(r["has_front"] for r in mine):
                continue  # not run yet
            axis.plot([r["flexion_deg"] for r in found], [r["coverage_pct"] for r in found],
                      color=COLORS[mechanism], linestyle=DASHES[mechanism], marker=MARKERS[mechanism],
                      markersize=6, markeredgecolor="white", linewidth=1.8, label=fingers.SHORT_LABELS[mechanism],
                      zorder=3)
            for r in found:
                if r["seed_min_pct"] is not None and r["seed_max_pct"] - r["seed_min_pct"] > 0.5:
                    axis.vlines(r["flexion_deg"], r["seed_min_pct"], r["seed_max_pct"], color=COLORS[mechanism],
                                linewidth=1.0, alpha=0.6, zorder=2)
            for r in mine:
                if r["coverage_pct"] is None and r["has_front"]:
                    axis.scatter([r["flexion_deg"]], [0.0], marker="x", color=COLORS[mechanism], s=40, zorder=3,
                                 clip_on=False)
        axis.set_title(f"interfaces {height} mm thick", color=INK, fontsize=9.5, loc="left")
        axis.set_xlabel("required flexion φ [deg]")
        axis.set_xticks(sorted(FLEXIONS_DEG))
        axis.set_xlim(min(FLEXIONS_DEG) - 8, max(FLEXIONS_DEG) + 8)
    axes[0].set_ylim(0.0, 100.0)
    axes[0].set_ylabel("usable contact interface (P + D) / |OC| [%]")
    legend = {}
    for axis in axes:
        for handle, label in zip(*axis.get_legend_handles_labels()):
            legend.setdefault(label, handle)
    if legend:
        figure.legend(list(legend.values()), list(legend), loc="outside lower center", ncol=len(legend),
                      frameon=False, fontsize=9)
    figure.suptitle(f"|OC| = {FINGER_RANGE_MM[1]:.0f} mm, |OA| 40–60 mm, each interface ≥ {PAD_MIN_MM} mm; "
                    "x: no feasible design", color=MUTED, fontsize=9, x=0.01, ha="left")
    out = save(figure, "fig_coverage")
    with (out.with_suffix(".csv")).open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    def cell(r: dict) -> str:
        if r["coverage_pct"] is None:
            return "x" if r["has_front"] else "·"
        return f"{r['coverage_pct']:.0f}% ({r['proximal_mm']:.0f}+{r['distal_mm']:.0f})"

    print("coverage:   " + "  ".join(f"{fingers.SHORT_LABELS[m]:>14s}" for m in fingers.MECHANISMS))
    for height in heights:
        for flexion in sorted(FLEXIONS_DEG):
            cells = [cell(next(r for r in rows if r["mechanism"] == m and r["pad_height_mm"] == height
                               and r["flexion_deg"] == flexion)) for m in fingers.MECHANISMS]
            print(f"h{height:<2d} {flexion:3d}°  " + "  ".join(f"{c:>14s}" for c in cells))
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
