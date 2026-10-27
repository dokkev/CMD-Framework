#!/usr/bin/env python3
"""Paper figure: the four mechanisms side by side at one requirement, each its
study's best design (most pad), drawn to one scale.

    python scripts/optimize/paper/fig_mechanisms.py [height_mm] [flexion_deg]

Top row: extended (0 deg), the solids the checks see. Bottom row: the
distal pad every 15 deg from 0 to the required flexion, darker as it
flexes, over the finger at that flexion; the dotted curve is the distal
segment's instantaneous centre of rotation relative to the proximal (the
2R's stays on its pin, the RCJ's rolls along its rollers' contact, CoDe's
moves with its four-bar), its start a ring. Defaults: 5 mm pads, 150 deg.
Saves output/paper/fig_mechanisms_h<height>_flex<flexion>.png / .pdf.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import fingers  # noqa: E402
from fingers import sketch  # noqa: E402
from style import COLORS, INK, MUTED, save  # noqa: E402

XLIM, YLIM = (-14.0, 104.0), (-62.0, 30.0)


def frame(axis) -> None:
    sketch._sketch_axes(axis)
    axis.set_xlim(*XLIM)
    axis.set_ylim(*YLIM)


def main(argv: list[str]) -> int:
    height = int(argv[0]) if argv else 5
    flexion = int(argv[1]) if len(argv) > 1 else 150
    found = [(m, fingers.best(m, height, flexion)) for m in fingers.MECHANISMS]
    found = [(m, f) for m, f in found if f is not None]
    if not found:
        print("no fronts yet")
        return 1
    figure, axes = plt.subplots(2, len(found), figsize=(3.1 * len(found), 5.2), squeeze=False,
                                constrained_layout=True)
    for c, (mechanism, finger) in enumerate(found):
        top, bottom = axes[0, c], axes[1, c]
        finger.draw(top, 0.0)
        frame(top)
        p, d = finger.pads_mm
        top.set_title(f"{finger.label}\nP {p:.0f} + D {d:.0f} mm, k ≤ {float(finger.row['torque_ratio_max']):.2f}",
                      color=INK, fontsize=8.5, loc="left")
        top.title.set_color(COLORS[mechanism])

        end = min(float(flexion), finger.reachable_deg())
        finger.draw(bottom, end)
        shades = LinearSegmentedColormap.from_list("flexion", ["#ffffff", COLORS[mechanism]])
        for pose in np.arange(0.0, end + 1e-9, 15.0):
            bottom.add_patch(Polygon(finger.distal_pad(pose), closed=True, facecolor=shades(0.2 + 0.8 * pose / 180.0),
                                     edgecolor=MUTED, linewidth=0.4, alpha=0.8, zorder=2.4))
        phi = np.linspace(0.0, end, 181)
        centre = finger.icr(phi)
        bottom.plot(centre[:, 0], centre[:, 1], linestyle=":", color=INK, linewidth=1.3, zorder=6)
        bottom.scatter(*centre[0], s=22, facecolor="white", edgecolor=INK, linewidth=1.0, zorder=7)
        bottom.scatter(*centre[-1], s=22, color=INK, zorder=7)
        frame(bottom)
        bottom.set_title(f"0–{end:.0f}°, safe to {finger.phi_max_deg:.0f}°", color=INK, fontsize=8.5, loc="left")
    figure.suptitle(f"Best design of each mechanism: |OC| 100 mm, pads {height} mm thick, {flexion}° required. "
                    "Dotted: instantaneous centre of rotation of the distal segment (ring: extended)",
                    color=MUTED, fontsize=8.5, x=0.01, ha="left")
    out = save(figure, f"fig_mechanisms_h{height}_flex{flexion}")
    for mechanism, finger in found:
        print(f"{mechanism:>18s}: x {tuple(round(v, 1) for v in finger.x)}, pads {finger.pads_mm[0]:.1f} + "
              f"{finger.pads_mm[1]:.1f}, safe to {finger.phi_max_deg:.1f}")
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
