#!/usr/bin/env python3
"""Plot the pad-cost study: the shortest finger that carries given pads up to a
flexion, the linkage against the serial 2R.

    python scripts/optimize/pad_cost/plot.py

Reads each config's latest merged front (output/<mechanism>/<config>/portfolio_NNN/seeds.csv,
verified), takes its shortest finger, and draws one panel per pad size: the
least |OC| against the required flexion, a mark where no design was found
within the finger range. Also prints the table and writes it as CSV.
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
from make_configs import FINGER_RANGE_MM, FLEXIONS_DEG, MECHANISMS, PADS_MM, name  # noqa: E402

COLORS = {"linkage": "#2a78d6", "revolute": "#eb6834"}
LABELS = {"linkage": "CoDe linkage", "revolute": "Serial 2R"}
MARKERS = {"linkage": "o", "revolute": "s"}
INK, MUTED, GRID = "#1f1f1e", "#6b6a66", "#e6e5e0"
REFERENCE_MM = 100.0


def shortest(mechanism: str, stem: str) -> dict | None:
    fronts = sorted((REPOSITORY_ROOT / "output" / mechanism / stem).glob("portfolio_*/seeds.csv"))
    if not fronts:
        return None
    rows = list(csv.DictReader(fronts[-1].open(encoding="utf-8", newline="")))
    return min(rows, key=lambda row: float(row["finger_length_mm"]), default=None)


def main() -> int:
    table = []
    for mechanism in MECHANISMS:
        for pads in PADS_MM:
            for flexion in FLEXIONS_DEG:
                row = shortest(mechanism, name(mechanism, pads, flexion))
                table.append({"mechanism": mechanism, "proximal_pad_mm": pads[0], "distal_pad_mm": pads[1],
                              "flexion_deg": flexion,
                              "finger_length_mm": float(row["finger_length_mm"]) if row else None,
                              "phi_max_deg": float(row["phi_max_deg"]) if row else None,
                              "torque_ratio_max": float(row["torque_ratio_max"]) if row else None})

    plt.rcParams.update({"font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
                         "xtick.color": MUTED, "ytick.color": MUTED})
    figure, axes = plt.subplots(1, len(PADS_MM), figsize=(4.6 * len(PADS_MM), 4.6), sharey=True,
                                facecolor="white", constrained_layout=True)
    top = FINGER_RANGE_MM[1]
    for axis, pads in zip(axes, sorted(PADS_MM)):
        axis.grid(color=GRID, linewidth=0.8)
        axis.set_axisbelow(True)
        for side in ("top", "right"):
            axis.spines[side].set_visible(False)
        axis.axhline(REFERENCE_MM, color=MUTED, linewidth=1.0, linestyle="--")
        for mechanism in MECHANISMS:
            rows = [r for r in table if r["mechanism"] == mechanism and (r["proximal_pad_mm"], r["distal_pad_mm"]) == pads]
            rows.sort(key=lambda r: r["flexion_deg"])
            found = [r for r in rows if r["finger_length_mm"] is not None]
            axis.plot([r["flexion_deg"] for r in found], [r["finger_length_mm"] for r in found],
                      color=COLORS[mechanism], marker=MARKERS[mechanism], markersize=7, markeredgecolor="white",
                      linewidth=2.2, label=LABELS[mechanism])
            for r in rows:
                if r["finger_length_mm"] is None:
                    axis.scatter([r["flexion_deg"]], [top], marker="x", color=COLORS[mechanism], s=60, zorder=3)
                    axis.annotate(f"none ≤ {top:.0f} mm", (r["flexion_deg"], top), xytext=(0, -14),
                                  textcoords="offset points", ha="center", color=COLORS[mechanism], fontsize=8)
        axis.set_title(f"pads {pads[0]} + {pads[1]} mm (usable all through)", color=INK, fontsize=10, loc="left")
        axis.set_xlabel("flexion the finger must reach [deg]")
        axis.set_xticks(sorted(FLEXIONS_DEG))
    axes[0].set_ylabel("shortest finger |OC| that carries them [mm]")
    axes[0].legend(loc="upper left", frameon=False, fontsize=9)
    figure.suptitle("What fitting the pads costs: a longer finger, or less flexion "
                    f"(dashed: |OC| {REFERENCE_MM:.0f} mm)", color=INK, fontsize=11, x=0.01, ha="left")
    output = REPOSITORY_ROOT / "output" / "comparison"
    output.mkdir(parents=True, exist_ok=True)
    figure.savefig(output / "pad_cost.png", dpi=130)
    with (output / "pad_cost.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(table[0]))
        writer.writeheader()
        writer.writerows(table)

    print(f"{'pads':>8s} {'flexion':>8s}  {'linkage |OC|':>13s}  {'2R |OC|':>9s}")
    for pads in sorted(PADS_MM):
        for flexion in sorted(FLEXIONS_DEG):
            cells = []
            for mechanism in MECHANISMS:
                r = next(r for r in table if r["mechanism"] == mechanism and r["flexion_deg"] == flexion
                         and (r["proximal_pad_mm"], r["distal_pad_mm"]) == pads)
                cells.append("-" if r["finger_length_mm"] is None else f"{r['finger_length_mm']:.1f}")
            print(f"{pads[0]:>3d}+{pads[1]:<4d} {flexion:7d}°  {cells[0]:>13s}  {cells[1]:>9s}")
    print(f"saved {output / 'pad_cost.png'} and {output / 'pad_cost.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
