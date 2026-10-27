#!/usr/bin/env python3
"""Plot the pad trade-off study: at one finger length, the most effective pad
each mechanism keeps up to a required flexion, one panel per pad height.

    python scripts/optimize/pad_tradeoff/plot.py

Reads each config's latest merged front (output/<mechanism>/<config>/portfolio_NNN/seeds.csv,
verified) and takes its most pad. Saves output/comparison/pad_tradeoff.png
and the table as CSV, and prints it.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
REPOSITORY_ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
from make_configs import FINGER_RANGE_MM, FLEXIONS_DEG, HEIGHTS_MM, MECHANISMS, name  # noqa: E402

COLORS = {"linkage": "#2a78d6", "revolute": "#eb6834"}
LABELS = {"linkage": "CoDe linkage", "revolute": "Serial 2R"}
MARKERS = {"linkage": "o", "revolute": "s"}
INK, MUTED, GRID = "#1f1f1e", "#6b6a66", "#e6e5e0"


def most_pad(mechanism: str, stem: str) -> dict | None:
    fronts = sorted((REPOSITORY_ROOT / "output" / mechanism / stem).glob("portfolio_*/seeds.csv"))
    if not fronts:
        return None
    rows = list(csv.DictReader(fronts[-1].open(encoding="utf-8", newline="")))
    return max(rows, key=lambda row: float(row["pad_coverage"]), default=None)


def main() -> int:
    table = []
    for mechanism in MECHANISMS:
        for height in HEIGHTS_MM:
            for flexion in FLEXIONS_DEG:
                row = most_pad(mechanism, name(mechanism, height, flexion))
                value = (lambda key: float(row[key]) if row else None)  # noqa: E731
                table.append({"mechanism": mechanism, "pad_height_mm": height, "flexion_deg": flexion,
                              "pad_coverage": value("pad_coverage"),
                              "proximal_effective_mm": value("proximal_effective_mm"),
                              "distal_effective_mm": value("distal_effective_mm"),
                              "finger_length_mm": value("finger_length_mm"),
                              "phi_max_deg": value("phi_max_deg"), "torque_ratio_max": value("torque_ratio_max")})

    def total(r: dict) -> float | None:
        return None if r["pad_coverage"] is None else r["proximal_effective_mm"] + r["distal_effective_mm"]

    plt.rcParams.update({"font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
                         "xtick.color": MUTED, "ytick.color": MUTED})
    heights = sorted(HEIGHTS_MM)
    figure, axes = plt.subplots(1, len(heights), figsize=(4.4 * len(heights), 4.4), sharey=True,
                                facecolor="white", constrained_layout=True)
    for axis, height in zip(axes, heights):
        axis.grid(color=GRID, linewidth=0.8)
        axis.set_axisbelow(True)
        for side in ("top", "right"):
            axis.spines[side].set_visible(False)
        for mechanism in MECHANISMS:
            rows = sorted((r for r in table if r["mechanism"] == mechanism and r["pad_height_mm"] == height),
                          key=lambda r: r["flexion_deg"])
            found = [r for r in rows if total(r) is not None]
            axis.plot([r["flexion_deg"] for r in found], [total(r) for r in found], color=COLORS[mechanism],
                      marker=MARKERS[mechanism], markersize=7, markeredgecolor="white", linewidth=2.0,
                      label=LABELS[mechanism])
            for r in rows:
                if total(r) is None:
                    axis.scatter([r["flexion_deg"]], [0.0], marker="x", color=COLORS[mechanism], s=60, zorder=3)
        axis.set_title(f"pads {height} mm tall", color=INK, fontsize=10, loc="left")
        axis.set_xlabel("flexion required [deg]")
        axis.set_xticks(sorted(FLEXIONS_DEG))
        axis.set_ylim(bottom=0.0)
    axes[0].set_ylabel("effective pad length P + D [mm]\n(usable all through the flexion)")
    axes[0].legend(loc="lower left", frameon=False, fontsize=9)
    figure.suptitle(f"Same finger (|OC| {FINGER_RANGE_MM[1]:.0f} mm): the pad each mechanism keeps as it must flex further",
                    color=INK, fontsize=11, x=0.01, ha="left")
    output = REPOSITORY_ROOT / "output" / "comparison"
    output.mkdir(parents=True, exist_ok=True)
    figure.savefig(output / "pad_tradeoff.png", dpi=130)
    with (output / "pad_tradeoff.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(table[0]))
        writer.writeheader()
        writer.writerows(table)

    def cell(r: dict) -> str:
        if total(r) is None:
            return "-"
        return f"{r['proximal_effective_mm']:.1f}+{r['distal_effective_mm']:.1f}"

    print(f"{'height':>6s} {'flexion':>8s}  {'linkage P+D':>12s}  {'2R P+D':>12s}")
    for height in heights:
        for flexion in sorted(FLEXIONS_DEG):
            cells = [cell(next(r for r in table if r["mechanism"] == m and r["pad_height_mm"] == height
                               and r["flexion_deg"] == flexion)) for m in MECHANISMS]
            print(f"{height:>4d}mm {flexion:7d}°  {cells[0]:>12s}  {cells[1]:>12s}")
    print(f"saved {output / 'pad_tradeoff.png'} and {output / 'pad_tradeoff.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
