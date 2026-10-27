#!/usr/bin/env python3
"""One serial 2R design over a flexion range, sketched as plot_clearance.py's.

    python scripts/optimize/plot_revolute_motion.py [seeds.csv] [role] [to_deg]

Defaults: the pad trade-off study's 2R at 90 deg with 5 mm pads
(output/revolute/revolute_h5_flex90/portfolio_*/seeds.csv), its most_pad,
0 to 90 deg. Snapshots every 30 deg, then the distal pad every 10 deg on
one sketch, and each pair's gap against the flexion. Saves
``output/comparison/revolute_motion.png``.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

HERE = Path(__file__).resolve().parent
REPOSITORY_ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPOSITORY_ROOT))
import plot_clearance as sketch  # noqa: E402

import core.morphology.linkage.packaging as packaging_module  # noqa: E402
from core.optimization.revolute import geometry  # noqa: E402
from core.optimization.revolute.evaluate import _margins  # noqa: E402
from core.optimization.study import load_run  # noqa: E402

ORANGE = "#eb6834"


def main(argv: list[str]) -> int:
    packaging_module.EXACT_BAND_MM = 1.0e6  # every gap exact
    default = sorted((REPOSITORY_ROOT / "output/revolute/revolute_h5_flex90").glob("portfolio_*/seeds.csv"))
    seeds = Path(argv[0]) if argv else default[-1]
    role = argv[1] if len(argv) > 1 else "most_pad"
    to_deg = float(argv[2]) if len(argv) > 2 else 90.0
    row = next(r for r in csv.DictReader(seeds.open(encoding="utf-8", newline="")) if role in r["seed_role"].split(";"))
    run = load_run(seeds.parent.parent / row["run"].split("/")[-1])
    config = run.config
    x = run.mechanism.read_design(row)
    geo = geometry(x, config)
    packaging = config.goal.packaging

    poses = np.arange(0.0, to_deg + 1e-9, 30.0)
    phi = np.arange(0.0, 180.0 + 1e-9, sketch.STEP_DEG)
    margins = _margins(geo, np.radians(phi), config)
    figure = plt.figure(figsize=(4.0 * (len(poses) + 1), 8.2), facecolor="white", constrained_layout=True)
    grid = figure.add_gridspec(2, len(poses) + 1)
    for c, pose in enumerate(poses):
        axis = figure.add_subplot(grid[0, c])
        k = int(np.argmin(np.abs(phi - pose)))
        sketch.draw_revolute(axis, geo, packaging, math.radians(pose), sketch._hits(margins, k))
        sketch._sketch_axes(axis)
        axis.set_xlim(-16.0, 105.0)
        axis.set_ylim(-60.0, 32.0)
        axis.set_title(f"{pose:.0f}°", color=sketch.INK, fontsize=10, loc="left")

    # The distal pad every 10 deg, darker with flexion, on one sketch.
    axis = figure.add_subplot(grid[0, len(poses)])
    shades = LinearSegmentedColormap.from_list("flexion", ["#ffffff", ORANGE])
    targets = np.arange(0.0, to_deg + 1e-9, 10.0)
    sketch.draw_revolute(axis, geo, packaging, 0.0, set())
    for pose in targets:
        polygon = geo.distal_pad([math.radians(pose)])[0]
        axis.add_patch(Polygon(polygon, closed=True, facecolor=shades(0.25 + 0.75 * pose / max(to_deg, 1.0)),
                               edgecolor=sketch.MUTED, linewidth=0.5, alpha=0.85, zorder=2.5))
    sketch._sketch_axes(axis)
    axis.set_xlim(-16.0, 105.0)
    axis.set_ylim(-60.0, 32.0)
    axis.set_title(f"distal pad every 10°, 0–{to_deg:.0f}°", color=sketch.INK, fontsize=10, loc="left")

    phi_max = float(row["phi_max_deg"])
    sketch.clearance_axes(figure.add_subplot(grid[1, :]), phi, margins, phi_max, {},
                          f"gap of each checked pair; safe to {phi_max:.0f}° ({row['collision_pair'].replace('_', ' ')})")
    figure.suptitle(
        f"Serial 2R ({seeds.parent.parent.name}, {role}): |OA| {x[0]:.1f}, |AC| {x[1]:.1f} mm, "
        f"pads {float(row['proximal_effective_mm']):.1f} + {float(row['distal_effective_mm']):.1f} mm, "
        f"distal pad {x[4]:.1f} mm below A-C (finger {float(row['distal_depth_mm']):.1f} mm deep there)",
        color=sketch.INK, fontsize=11, x=0.01, ha="left")
    output = REPOSITORY_ROOT / "output" / "comparison" / "revolute_motion.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=130)
    print(f"{seeds.parent.parent.name} {role}: x {x}, safe to {phi_max:.1f} deg")
    print(f"saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
