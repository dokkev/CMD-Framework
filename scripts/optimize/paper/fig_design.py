#!/usr/bin/env python3
"""One design of a run's verified front, drawn through its motion.

    python scripts/optimize/paper/fig_design.py <mechanism> <run dir> [seed role]

``run dir``: the study's output directory (its latest portfolio is read);
``seed role``: the representative to draw (default ``balanced``). Panels: the
finger extended, then at every 30 deg of distal flexion up to the most it
reaches, over its distal pad every 15 deg; the dotted curve is the distal
segment's instantaneous centre of rotation, its start a ring.
Saves output/paper/fig_design_<run>_<role>.png / .pdf.
"""

from __future__ import annotations

import csv
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
from fig_mechanisms import frame  # noqa: E402
from style import COLORS, INK, MUTED, save  # noqa: E402

STEP_DEG = 30.0
COLUMNS = 4


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        return 2
    mechanism, run_dir = argv[0], Path(argv[1]).resolve()
    role = argv[2] if len(argv) > 2 else "balanced"
    front = sorted(run_dir.glob("portfolio_*/seeds.csv"))[-1]
    row = next((r for r in csv.DictReader(front.open(encoding="utf-8", newline="")) if r["seed_role"] == role), None)
    if row is None:
        print(f"no {role} in {front}")
        return 1
    finger = fingers.finger_from_row(mechanism, row, run_dir)
    end = min(finger.phi_max_deg, finger.reachable_deg())
    poses = [*np.arange(0.0, end - 1e-9, STEP_DEG), end]
    rows = -(-len(poses) // COLUMNS)
    figure, axes = plt.subplots(rows, COLUMNS, figsize=(3.4 * COLUMNS, 3.0 * rows), squeeze=False,
                                constrained_layout=True)
    for axis in axes.flat[len(poses):]:
        axis.axis("off")
    shades = LinearSegmentedColormap.from_list("flexion", ["#ffffff", COLORS[mechanism]])
    for axis, pose in zip(axes.flat, poses):
        finger.draw(axis, pose)
        for ghost in np.arange(0.0, pose + 1e-9, 15.0):
            axis.add_patch(Polygon(finger.distal_pad(ghost), closed=True, facecolor=shades(0.2 + 0.8 * ghost / 180.0),
                                   edgecolor=MUTED, linewidth=0.4, alpha=0.6, zorder=2.4))
        if pose > 0.0:
            centre = finger.icr(np.linspace(0.0, pose, 121))
            axis.plot(centre[:, 0], centre[:, 1], linestyle=":", color=INK, linewidth=1.2, zorder=6)
            axis.scatter(*centre[0], s=18, facecolor="white", edgecolor=INK, linewidth=1.0, zorder=7)
        frame(axis)
        axis.set_title(f"φ = {pose:.0f}°", color=INK, fontsize=9, loc="left")
    p, d = finger.pads_mm
    figure.suptitle(f"{fingers.LABELS[mechanism]}, {run_dir.name} {role} ({row['run']}): "
                    f"P {p:.1f} + D {d:.1f} mm = {100 * float(row['pad_coverage']):.0f}%, "
                    f"k ≤ {float(row['torque_ratio_max']):.2f}, safe to {float(row['safe_reach_deg']):.0f}°, "
                    f"curl {float(row['curl_deg']):.0f}°", color=MUTED, fontsize=8.5, x=0.01, ha="left")
    out = save(figure, f"fig_design_{run_dir.name}_{role}")
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
