"""Paper figure: what each mechanism's pad costs, at the trade-off study's
best designs.

    python -m visualize.paper.fig_cost [height_mm] [flexion_deg]

(a) ``k(phi) = d(phi)/d(theta)``, the joint servo's torque per unit of
distal moment, over the required flexion (the 2R's is 1, the RCJ's 2, the
linkage's varies; reported, not limited). (b) The
finger's thickness at the joint, extended: from the joint servo's top to the
proximal pad's face. (c) ``k_max`` of each best design against the required
flexion. Defaults: 5 mm pads, 150 deg for (a) and (b). Saves
output/paper/fig_cost_h<height>_flex<flexion>.png / .pdf.
"""

from __future__ import annotations

import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from visualize import studies  # noqa: E402
from visualize.finger import Finger  # noqa: E402
from visualize.style import (COLORS, DASHES, INK, MARKERS, MECHANISMS, MUTED, SHORT_LABELS, axes_style,  # noqa: E402
                             save, use)

FLEXIONS_DEG = studies.grid("interface_coverage").FLEXIONS_DEG
TORQUE_RATIO_MAX = 2.0
"""The RCJ's k, drawn for reference."""


def thickness_mm(finger: Finger) -> float:
    """Dorsal-palmar extent at the joint, extended: the joint's top (the servo,
    or the idealized 2R's pin) down to the proximal pad's face."""

    if finger.mechanism == "linkage":
        top = float(finger.packaging.pip_footprint(finger.state).outline[:, 1].max())
    elif finger.mechanism in ("rcj", "revolute_parallel"):
        top = float(finger.state.servo.outline[:, 1].max())
    elif finger.mechanism == "revolute":
        top = float(finger.state.distal_servo.outline[:, 1].max())
    else:
        top = finger.config.joint_radius_mm
    return top - float(finger.proximal_pad()[:, 1].min())


def main(argv: list[str]) -> int:
    use("paper")
    height = int(argv[0]) if argv else 5
    flexion = int(argv[1]) if len(argv) > 1 else 150
    found = [(m, studies.best(m, height, flexion)) for m in MECHANISMS]
    found = [(m, f) for m, f in found if f is not None]
    figure, axes = plt.subplots(1, 3, figsize=(10.5, 3.2), constrained_layout=True,
                                gridspec_kw={"width_ratios": (1.3, 0.9, 1.3)})
    k_axis, depth_axis, kmax_axis = axes
    for axis in axes:
        axes_style(axis)

    phi = np.linspace(0.0, flexion, 301)
    for mechanism, finger in found:
        k_axis.plot(phi, finger.k(phi), color=COLORS[mechanism], linestyle=DASHES[mechanism], linewidth=1.8,
                    label=SHORT_LABELS[mechanism])
    k_axis.axhline(TORQUE_RATIO_MAX, color=MUTED, linewidth=0.8, linestyle="--")
    k_axis.set_xlabel("flexion φ [deg]")
    k_axis.set_ylabel("k = dφ/dθ  (servo torque per distal moment)")
    k_axis.set_xlim(0.0, flexion)
    k_axis.set_ylim(0.0, TORQUE_RATIO_MAX * 1.25)
    k_axis.set_title(f"(a) transmission, best designs at {flexion}°", color=INK, fontsize=9, loc="left")
    k_axis.legend(frameon=False, fontsize=8, loc="lower right")

    names = [SHORT_LABELS[m] for m, _ in found]
    depths = [thickness_mm(f) for _, f in found]
    bars = depth_axis.bar(names, depths, color=[COLORS[m] for m, _ in found], width=0.6)
    for bar, depth in zip(bars, depths):
        depth_axis.annotate(f"{depth:.1f}", (bar.get_x() + bar.get_width() / 2, depth), ha="center", va="bottom",
                            fontsize=8, color=INK, xytext=(0, 2), textcoords="offset points")
    depth_axis.set_ylabel("thickness at the joint [mm]")
    depth_axis.set_title("(b) finger thickness, extended", color=INK, fontsize=9, loc="left")
    depth_axis.tick_params(axis="x", labelsize=8)

    for mechanism in MECHANISMS:
        points = [(f, float(row["torque_ratio_max"])) for f in sorted(FLEXIONS_DEG)
                  if (row := studies.best_row(mechanism, height, f)) is not None]
        if points:
            kmax_axis.plot(*zip(*points), color=COLORS[mechanism], linestyle=DASHES[mechanism],
                           marker=MARKERS[mechanism], markersize=5, markeredgecolor="white", linewidth=1.5,
                           label=SHORT_LABELS[mechanism])
    kmax_axis.axhline(TORQUE_RATIO_MAX, color=MUTED, linewidth=0.8, linestyle="--")
    kmax_axis.set_xticks(sorted(FLEXIONS_DEG))
    kmax_axis.set_xlabel("required flexion [deg]")
    kmax_axis.set_ylabel("k_max")
    kmax_axis.set_ylim(0.0, TORQUE_RATIO_MAX * 1.25)
    kmax_axis.set_title(f"(c) k_max of the most-pad designs, {height} mm pads", color=INK, fontsize=9, loc="left")
    out = save(figure, f"fig_cost_h{height}_flex{flexion}")
    for (mechanism, finger), depth in zip(found, depths):
        print(f"{mechanism:>18s}: thickness {depth:.1f} mm, k_max {float(finger.row['torque_ratio_max']):.2f}")
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
