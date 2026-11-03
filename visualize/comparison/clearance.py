"""The clearance of every checked pair as the finger flexes, the linkage
against the serial 2R with the same finger and pads, with sketches.

    python -m visualize.comparison.clearance [seeds.csv] [role]

The designs are ``visualize.comparison.nominal``'s: a representative linkage
seed and the 2R of at most its length with its effective pads that flexes
the most (its distal pad's drop searched). Each row sketches the finger at
0 deg, 90 deg and its most flexion (servos as boxes, servo-driven joints
yellow, links red, pads green), then plots each pair's gap against the
flexion: below 0 the pair collides, and the first such flexion ends the
motion. Saves ``output/comparison/clearance.png``.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from visualize import sketch  # noqa: E402
from visualize.comparison.designs import DEFAULT_SEEDS, linkage_seed, matched_revolute  # noqa: E402
from visualize.finger import FingerView, view_of  # noqa: E402
from visualize.finger.mechanism import FourBarView  # noqa: E402
from visualize.gaps import STEP_DEG, clearance_axes  # noqa: E402
from visualize.style import COLORS, INK, save_comparison  # noqa: E402


def main(argv: list[str]) -> int:
    seeds = Path(argv[0]) if argv else DEFAULT_SEEDS
    role = argv[1] if len(argv) > 1 else "balanced"
    row, config, linkage = linkage_seed(seeds, role)
    proximal, distal = round(float(row["proximal_effective_mm"])), round(float(row["distal_effective_mm"]))
    finger = round(float(row["finger_length_mm"]), 1)
    _, revolute_config, (x, revolute) = matched_revolute(finger, proximal, distal)
    views = {"linkage": FingerView("linkage", FourBarView(linkage, config.packaging), row),
             "revolute": view_of("revolute", revolute_config, x)}
    linkage_max, revolute_max = float(row["phi_max_deg"]), math.degrees(revolute.phi_max_rad)
    phi_l = np.arange(0.0, views["linkage"].reachable_deg() + 1e-9, STEP_DEG)
    phi_r = np.arange(0.0, 180.0 + 1e-9, STEP_DEG)
    margins_l, margins_r = views["linkage"].margins(phi_l), views["revolute"].margins(phi_r)

    figure = plt.figure(figsize=(17.0, 8.0), facecolor="white", constrained_layout=True)
    grid = figure.add_gridspec(2, 4, width_ratios=(1.0, 1.0, 1.0, 1.25))
    colors: dict[str, str] = {}
    for r, (name, phis, margins, phi_max) in enumerate((("linkage", phi_l, margins_l, linkage_max),
                                                         ("revolute", phi_r, margins_r, revolute_max))):
        sketches = [figure.add_subplot(grid[r, c]) for c in range(3)]
        for axis, pose in zip(sketches, (0.0, 90.0, phi_max)):
            views[name].draw(axis, pose)
            sketch.frame(axis, (-16.0, 102.0), (-60.0, 30.0))
            axis.set_title(f"{pose:.0f}°", color=INK, fontsize=10, loc="left")
        if name == "linkage":
            label = f"CoDe linkage: |OC| {finger:.0f} mm, k_max {float(row['torque_ratio_max']):.2f}"
            title = f"CoDe linkage: safe to {linkage_max:.0f}°"
        else:
            label = (f"Serial 2R: |OC| {revolute.finger_length_mm:.0f} mm (|OA| {x[0]:.1f}, "
                     f"distal pad {x[4]:.1f} mm below A-C), k 1")
            title = f"Serial 2R: safe to {revolute_max:.0f}° ({revolute.collision_pair.replace('_', ' ')})"
        sketches[0].set_title(f"{label}\n0°", color=INK, fontsize=10, loc="left")
        sketches[0].title.set_color(COLORS[name])
        clearance_axes(figure.add_subplot(grid[r, 3]), phis, margins, phi_max, colors, title)
    figure.suptitle(f"Clearance of every checked pair as the finger flexes: same finger (|OC| ≤ {finger:.0f} mm) "
                    f"and pads ({proximal} + {distal} mm usable, "
                    f"{config.packaging.proximal_pad.height_mm:.0f} mm tall); shaded: past the safe range",
                    color=INK, fontsize=11, x=0.01, ha="left")
    output = save_comparison(figure, "clearance")
    print(f"linkage {role}: |OC| {finger}, pads {proximal} + {distal}, safe to {linkage_max:.1f} deg; "
          f"2R {x}: safe to {revolute_max:.1f} deg ({revolute.collision_pair})")
    print(f"saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
