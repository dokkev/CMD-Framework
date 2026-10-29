"""Paper figure (Visual): where each mechanism loses its contact faces, at one
requirement, its best design drawn through the motion.

    python -m visualize.paper.fig_visual [height_mm] [flexion_deg] [source]

Rows: the mechanisms, in the ablation's order, each its best design (the
most pad coverage of the collected table, ``visualize.paper.collect``; its
``k_max`` and extended centre of rotation's share of |OC| by the label).
Columns 1-3: extended, half the required flexion and the required flexion,
one scale, O aligned; each contact face green where an object
``goal.loss.access_depth_mm`` deep reaches it, red x where something stands closer
in front (the accessibility profile, ``core.optimization.analysis.loss``);
the centre of rotation's path from extension, dotted (a fixed pivot's: a
point). Column 4: the finger's length, unfolded: the faces kept through the
motion (the objective's rule, the table's ``loss_rule``), what the motion
takes, and the rest. PROVISIONAL while the loss rule and the runs are
(drafts from the quick runs). Saves output/paper/fig_visual_h<h>_flex<f>.png
/ .pdf and the rows as .csv.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from core.optimization.analysis.loss import access_points, scene  # noqa: E402
from visualize import sketch  # noqa: E402
from visualize.finger import finger_from_row  # noqa: E402
from visualize.fronts import latest_portfolio, read_rows  # noqa: E402
from visualize.paper.collect import ACCESS_DEPTH_MM, read  # noqa: E402
from visualize.style import (COLLIDE, COLORS, INK, LABELS, MECHANISMS, MUTED, PAPER, REPOSITORY_ROOT,  # noqa: E402
                             save, use)

USABLE, LOST, REST = "#4caf50", COLLIDE, "#d9d8d3"
XLIM, YLIM = (-14.0, 106.0), (-64.0, 30.0)


def design_row(entry: dict) -> tuple[dict[str, str], Path]:
    """The front's row the table names, and its config's folder."""

    config_dir = REPOSITORY_ROOT / Path(entry["run"]).parent
    rows = read_rows(latest_portfolio(config_dir))
    run = Path(entry["run"]).name
    row = next(r for r in rows if r["run"].split("/")[-1] == run and r["candidate"] == entry["candidate"])
    return row, config_dir


def faces_by_access(axis, built, index: int, depth_mm: float) -> None:
    """Each face's probed points, green where reached, red where not."""

    for side in ("proximal_pad", "distal_pad"):
        points, blocked = access_points(built, side, depth_mm)
        p, b = points[index], blocked[index]
        axis.plot(p[~b, 0], p[~b, 1], linestyle="none", marker="s", markersize=2.2, color=USABLE, zorder=6)
        axis.plot(p[b, 0], p[b, 1], linestyle="none", marker="x", markersize=3.0, color=LOST, zorder=6)


def main(argv: list[str]) -> int:
    use("paper")
    height = int(argv[0]) if argv else 5
    flexion = int(argv[1]) if len(argv) > 1 else 150
    source = argv[2] if len(argv) > 2 else "comparison_quick"
    table = [r for r in read(source) if r["thickness_mm"] == height and r["flexion_deg"] == flexion]
    poses = (0.0, flexion / 2.0, float(flexion))
    figure, axes = plt.subplots(len(MECHANISMS), 4, figsize=(11.5, 2.15 * len(MECHANISMS)), squeeze=False,
                                gridspec_kw={"width_ratios": (1, 1, 1, 1.35)}, constrained_layout=True)
    out_rows = []
    rule = next((r["loss_rule"] for r in table if r.get("loss_rule")), "?")
    depth = next((r["access_depth_mm"] for r in table if r.get("access_depth_mm")), ACCESS_DEPTH_MM)
    for row_axes, mechanism in zip(axes, MECHANISMS):
        entry = next((r for r in table if r["mechanism"] == mechanism), None)
        for axis in row_axes[:3]:
            sketch.frame(axis, XLIM, YLIM)
        strip = row_axes[3]
        if entry is None or entry.get("coverage_pct") is None:
            row_axes[0].text(0.5, 0.5, "no feasible design" if entry and entry["has_front"] else "not run",
                             transform=row_axes[0].transAxes, ha="center", color=MUTED)
            strip.axis("off")
            row_axes[0].set_title(LABELS[mechanism], color=COLORS[mechanism], fontsize=8.5, loc="left")
            continue
        row, config_dir = design_row(entry)
        finger = finger_from_row(mechanism, row, config_dir)
        built = scene(mechanism, finger.config, finger.x, poses)
        for index, (axis, pose) in enumerate(zip(row_axes[:3], poses)):
            finger.draw(axis, pose)
            faces_by_access(axis, built, index, depth)
            if pose > 0.0:
                path = finger.icr(np.linspace(0.0, pose, 61))
                sketch.icr_path(axis, path, size=14, linewidth=1.1)
            axis.set_title(f"φ = {pose:.0f}°", color=INK, fontsize=8, loc="left")
        row_axes[0].set_title(f"{LABELS[mechanism]}\nk_max {entry['k_max']:.2f}, ICR₀ at {entry['icr0_share']:.2f} |OC|",
                              color=COLORS[mechanism], fontsize=8, loc="left")
        # The finger unfolded: the faces kept, what the motion takes (the objective's rule), the rest.
        length = entry["finger_length_mm"]
        proximal, distal = entry["proximal_mm"], entry["distal_mm"]
        faces = finger.pads_mm
        motion = max(faces[0] - proximal, 0.0) + max(faces[1] - distal, 0.0)
        parts = [("proximal face kept", proximal, USABLE, None), ("distal face kept", distal, USABLE, "//"),
                 ("lost to the motion", motion, LOST, "xx"),
                 ("no face", max(length - proximal - distal - motion, 0.0), REST, None)]
        left = 0.0
        for label, width, color, hatch in parts:
            strip.barh(0, width, left=left, color=color, hatch=hatch, edgecolor="white", linewidth=0.8, height=0.35,
                       label=label)
            left += width
        strip.set_xlim(0.0, 100.0)
        strip.set_ylim(-1.2, 1.0)
        strip.set_yticks([])
        strip.text(proximal + distal, 0.25, f"{entry['coverage_pct']:.0f}%", ha="right", va="bottom", fontsize=8,
                   color=INK)
        strip.set_xlabel("unfolded finger [mm]" if mechanism == MECHANISMS[-1] else "", color=INK)
        for side in ("top", "right", "left"):
            strip.spines[side].set_visible(False)
        access = entry.get("access_lost_pct_at_flexion")
        strip.text(0.0, -0.75, f"P {proximal:.0f} + D {distal:.0f} mm kept to {flexion}°\n"
                   f"at {flexion}°: {access:.0f}% out of reach ({depth:g} mm object)",
                   color=MUTED, fontsize=7, va="center")
        out_rows.append({**{k: entry[k] for k in ("mechanism", "coverage_pct", "proximal_mm", "distal_mm", "k_max",
                                                    "icr0_share", "finger_length_mm", "run", "candidate",
                                                    "source", "loss_rule")},
                         "lost_to_motion_mm": motion, "access_depth_mm": depth, "access_lost_pct": access,
                         "selection": "the most pad coverage of the verified front (one seed, quick run)"})
    handles = next((a.get_legend_handles_labels() for a in axes[:, 3] if a.get_legend_handles_labels()[0]), None)
    if handles:
        figure.legend(*handles, loc="outside lower right", ncol=4, frameon=False, fontsize=7)
    figure.suptitle(f"PROVISIONAL: quick runs, loss rule {rule}\n"
                    f"Interfaces {height} mm thick, {flexion}° required, |OC| 100 mm. Faces: green reachable, "
                    f"red x out of reach of a {depth:g} mm object; dotted: centre of rotation",
                    color=COLLIDE, fontsize=8, x=0.01, ha="left")
    out = save(figure, f"fig_visual_h{height}_flex{flexion}")
    with out.with_suffix(".csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(out_rows[0]))
        writer.writeheader()
        writer.writerows(out_rows)
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
