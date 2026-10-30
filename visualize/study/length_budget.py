"""Where each mechanism's shortest finger spends its length: the 2R against
CoDe carrying the same modules (the accommodation study's designs).

    python -m visualize.study.length_budget [output folder] [P] [D] [flexion ...]

Per required flexion (a row): each mechanism's shortest finger drawn
extended and at the flexion, one scale, then its extended length unfolded
along the finger: the MCP servo's clearance, the proximal module, the
joint region carrying no module (between the modules' inner ends, where the
joint, its housing and, for CoDe, the four-bar's span sit; negative where
the modules overlap along the finger), the distal module. Markers: the
joints (2R: A; CoDe: A and B) and the extended centre of rotation.
Saves <output folder>/length_budget_P<P>_D<D>.png and the numbers as .csv.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from core.optimization.analysis.loss import scene  # noqa: E402
from core.optimization.mechanism import mechanism as mechanism_of  # noqa: E402
from visualize import sketch  # noqa: E402
from visualize.finger import finger_from_row  # noqa: E402
from visualize.fronts import latest_portfolio, read_rows  # noqa: E402
from visualize.style import COLLIDE, COLORS, INK, MUTED, REPOSITORY_ROOT, use  # noqa: E402
from visualize.study.accommodation import NAMES, module_of, shortest, study  # noqa: E402

PARTS = (("MCP clearance", "#d9d8d3", None), ("proximal module", "#7fb77e", None),
         ("joint region, no module", "#f2c14e", "//"), ("distal module", "#7fb77e", "\\\\"))


def budget(mechanism: str, row: dict, config_dir: Path) -> dict:
    """The extended finger's length, part by part (mm), and its joints' ``x``."""

    finger = finger_from_row(mechanism, row, config_dir)
    built = scene(mechanism, finger.config, finger.x, [0.0])
    proximal = np.asarray(built.solids["distal_pad"]["other_pad"].polygons[0])
    distal = np.asarray(built.solids["proximal_pad"]["other_pad"].polygons[0])[0]
    length = float(row["finger_length_mm"])
    start, p_end = float(proximal[:, 0].min()), float(proximal[:, 0].max())
    d_start, d_end = float(distal[:, 0].min()), float(distal[:, 0].max())
    v = dict(zip(mechanism_of(mechanism).variable_names, finger.x))
    joints = {"A": v["a_x_mm"]}
    if mechanism == "linkage":
        joints["B"] = v["b_x_mm"]
    joints["ICR0"] = float(row["icr0_share"]) * length
    return {"finger": finger, "length": length, "parts": (start, p_end - start, d_start - p_end, d_end - d_start),
            "tip": length - d_end, "joints": joints, "k": float(row["torque_ratio_max"])}


def main(argv: list[str]) -> int:
    use("paper")
    source = REPOSITORY_ROOT / (argv[0] if argv else "output/accommodation_quick")
    configs = study()
    proximal = module_of(argv[1]) if len(argv) > 1 else configs.REFERENCE
    distal = module_of(argv[2]) if len(argv) > 2 else configs.REFERENCE
    flexions = [int(f) for f in argv[3:]] or [120, 150]
    mechanisms = configs.MECHANISMS
    figure = plt.figure(figsize=(15.0, 3.3 * len(flexions)), constrained_layout=True)
    grid = figure.add_gridspec(len(flexions), 2 * len(mechanisms) + 1,
                               width_ratios=(*([1.0] * (2 * len(mechanisms))), 2.2))
    rows = []
    for r, flexion in enumerate(flexions):
        bars = figure.add_subplot(grid[r, -1])
        for m, mechanism in enumerate(mechanisms):
            stem = configs.name(mechanism, proximal, distal, flexion)
            config_dir = source / mechanism / stem
            axes = [figure.add_subplot(grid[r, 2 * m + i]) for i in range(2)]
            for axis in axes:
                sketch.frame(axis, (-14.0, 124.0), (-80.0, 40.0))
            row, _ = shortest(config_dir)
            if row is None:
                axes[0].text(0.5, 0.5, "not run yet" if latest_portfolio(config_dir) is None else
                             "no feasible design found", transform=axes[0].transAxes, ha="center", color=MUTED)
                continue
            found = budget(mechanism, row, config_dir)
            for axis, pose in zip(axes, (0.0, float(flexion))):
                found["finger"].draw(axis, pose)
                axis.set_title(f"φ = {pose:.0f}°", color=INK, fontsize=8, loc="left")
            axes[0].text(-12.0, 37.0, f"{NAMES[mechanism]}\nL* {found['length']:.1f} mm, k {found['k']:.2f}",
                         color=COLORS[mechanism], fontsize=8, fontweight="bold", va="top")
            y, left = len(mechanisms) - 1 - m, 0.0
            for (label, color, hatch), width in zip(PARTS, found["parts"]):
                bars.barh(y, width, left=left, color=color if width >= 0 else COLLIDE, hatch=hatch, height=0.5,
                          edgecolor="white", linewidth=0.8, label=label if (r, m) == (0, 0) else None)
                if abs(width) >= 4.0:
                    bars.text(left + width / 2.0, y, f"{width:.0f}", ha="center", va="center", fontsize=7.5,
                              color=INK)
                left += width
            for name, x in found["joints"].items():
                marker = "^" if name == "ICR0" else "o"
                bars.scatter([x], [y + 0.36], marker=marker, s=22, color=COLORS[mechanism], zorder=4)
                bars.text(x, y + 0.48, "ICR₀" if name == "ICR0" else name, ha="center", fontsize=6.5,
                          color=COLORS[mechanism])
            rows.append({"mechanism": mechanism, "flexion_deg": flexion, "finger_length_mm": f"{found['length']:.2f}",
                         **{label: f"{width:.2f}" for (label, _, _), width in zip(PARTS, found["parts"])},
                         "tip_mm": f"{found['tip']:.2f}",
                         **{f"{name}_x_mm": f"{x:.2f}" for name, x in found["joints"].items()},
                         "k_max": f"{found['k']:.3f}", "run": row["run"], "candidate": row["candidate"]})
        bars.set_yticks(range(len(mechanisms)), [NAMES[m] for m in reversed(mechanisms)], fontsize=7.5)
        bars.set_xlim(0.0, 125.0)
        bars.set_ylim(-0.6, len(mechanisms) - 0.2)
        bars.set_xlabel("along the extended finger from O [mm]", color=INK)
        bars.set_title(f"φ_req = {flexion}°: where the length goes", color=INK, fontsize=9, loc="left")
        for side in ("top", "right"):
            bars.spines[side].set_visible(False)
        if r == 0:
            bars.legend(fontsize=7, frameon=False, loc="lower right", ncol=2)
    figure.suptitle(f"DRAFT: {source.name} (unconverged): shortest finger carrying proximal {proximal[0]} x "
                    f"{proximal[1]} mm and distal {distal[0]} x {distal[1]} mm modules through φ_req",
                    color=COLLIDE, fontsize=9, x=0.01, ha="left")
    out = source / f"length_budget_P{proximal[0]}x{proximal[1]}_D{distal[0]}x{distal[1]}.png"
    source.mkdir(parents=True, exist_ok=True)
    figure.savefig(out, dpi=150)
    if rows:
        with out.with_suffix(".csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(dict.fromkeys(k for row in rows for k in row)))
            writer.writeheader()
            writer.writerows(rows)
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
