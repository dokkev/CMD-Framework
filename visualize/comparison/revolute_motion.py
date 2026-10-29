"""One serial 2R design over a flexion range, sketched as ``visualize.comparison.clearance``'s.

    python -m visualize.comparison.revolute_motion [seeds.csv] [role] [to_deg]

Defaults: the pad trade-off study's 2R at 90 deg with 5 mm pads
(output/revolute/revolute_h5_flex90/portfolio_*/seeds.csv), its most_pad,
0 to 90 deg. Snapshots every 30 deg, then the distal pad every 10 deg on
one sketch, and each pair's gap against the flexion. Saves
``output/comparison/revolute_motion.png``.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import core.morphology.linkage.packaging as packaging_module  # noqa: E402
from core.optimization.revolute import geometry  # noqa: E402
from core.optimization.revolute.evaluate import _margins  # noqa: E402
from visualize import gaps, sketch  # noqa: E402
from visualize.fronts import latest_portfolio, read_rows, run_of, with_role  # noqa: E402
from visualize.motion import collisions  # noqa: E402
from visualize.style import COLORS, INK, OUTPUT, save_comparison  # noqa: E402

XLIM, YLIM = (-16.0, 105.0), (-60.0, 32.0)


def main(argv: list[str]) -> int:
    packaging_module.EXACT_BAND_MM = 1.0e6  # every gap exact
    seeds = Path(argv[0]) if argv else latest_portfolio(OUTPUT / "revolute/revolute_h5_flex90")
    role = argv[1] if len(argv) > 1 else "most_pad"
    to_deg = float(argv[2]) if len(argv) > 2 else 90.0
    row = with_role(read_rows(seeds), role)
    run = run_of(seeds, row)
    config = run.config
    x = run.mechanism.read_design(row)
    geo = geometry(x, config)
    packaging = config.goal.packaging

    poses = np.arange(0.0, to_deg + 1e-9, 30.0)
    phi = np.arange(0.0, 180.0 + 1e-9, gaps.STEP_DEG)
    margins = _margins(geo, np.radians(phi), config)
    figure = plt.figure(figsize=(4.0 * (len(poses) + 1), 8.2), facecolor="white", constrained_layout=True)
    grid = figure.add_gridspec(2, len(poses) + 1)
    for c, pose in enumerate(poses):
        axis = figure.add_subplot(grid[0, c])
        k = int(np.argmin(np.abs(phi - pose)))
        sketch.draw_revolute(axis, geo, packaging, math.radians(pose), collisions(margins, k))
        sketch.frame(axis, XLIM, YLIM)
        axis.set_title(f"{pose:.0f}°", color=INK, fontsize=10, loc="left")

    # The distal pad every 10 deg, darker with flexion, on one sketch.
    axis = figure.add_subplot(grid[0, len(poses)])
    trail = np.arange(0.0, to_deg + 1e-9, 10.0)
    sketch.draw_revolute(axis, geo, packaging, 0.0, set())
    sketch.pad_trail(axis, [geo.distal_pad([math.radians(pose)])[0] for pose in trail],
                     0.25 + 0.75 * trail / max(to_deg, 1.0), COLORS["revolute"], alpha=0.85, linewidth=0.5,
                     zorder=2.5)
    sketch.frame(axis, XLIM, YLIM)
    axis.set_title(f"distal pad every 10°, 0–{to_deg:.0f}°", color=INK, fontsize=10, loc="left")

    phi_max = float(row["phi_max_deg"])
    gaps.clearance_axes(figure.add_subplot(grid[1, :]), phi, margins, phi_max, {},
                        f"gap of each checked pair; safe to {phi_max:.0f}° ({row['collision_pair'].replace('_', ' ')})")
    study = seeds.parent.parent.name
    figure.suptitle(
        f"Serial 2R ({study}, {role}): |OA| {x[0]:.1f}, |AC| {x[1]:.1f} mm, "
        f"pads {float(row['proximal_effective_mm']):.1f} + {float(row['distal_effective_mm']):.1f} mm, "
        f"distal pad {x[4]:.1f} mm below A-C (finger {float(row['distal_depth_mm']):.1f} mm deep there)",
        color=INK, fontsize=11, x=0.01, ha="left")
    output = save_comparison(figure, "revolute_motion")
    print(f"{study} {role}: x {x}, safe to {phi_max:.1f} deg")
    print(f"saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
