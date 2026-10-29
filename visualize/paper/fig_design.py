"""One design of a run's verified front, drawn through its motion.

    python -m visualize.paper.fig_design <mechanism> <run dir> [seed role]

``run dir``: the study's output directory (its latest portfolio is read);
``seed role``: the representative to draw (default ``balanced``). Panels: the
finger extended, then at every 30 deg of distal flexion up to the most it
reaches, over its distal pad every 15 deg; the dotted curve is the distal
segment's instantaneous centre of rotation, its start a ring.
Saves output/paper/fig_design_<run>_<role>.png / .pdf.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from visualize import sketch  # noqa: E402
from visualize.finger import finger_from_row  # noqa: E402
from visualize.fronts import latest_portfolio, read_rows, with_role  # noqa: E402
from visualize.style import COLORS, INK, LABELS, MUTED, save, use  # noqa: E402

STEP_DEG = 30.0
COLUMNS = 4


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        return 2
    use("paper")
    mechanism, run_dir = argv[0], Path(argv[1]).resolve()
    role = argv[2] if len(argv) > 2 else "balanced"
    front = latest_portfolio(run_dir)
    row = with_role(read_rows(front), role)
    if row is None:
        print(f"no {role} in {front}")
        return 1
    finger = finger_from_row(mechanism, row, run_dir)
    end = min(finger.phi_max_deg, finger.reachable_deg())
    poses = [*np.arange(0.0, end - 1e-9, STEP_DEG), end]
    rows = -(-len(poses) // COLUMNS)
    figure, axes = plt.subplots(rows, COLUMNS, figsize=(3.4 * COLUMNS, 3.0 * rows), squeeze=False,
                                constrained_layout=True)
    for axis in axes.flat[len(poses):]:
        axis.axis("off")
    for axis, pose in zip(axes.flat, poses):
        finger.draw(axis, pose)
        trail = np.arange(0.0, pose + 1e-9, 15.0)
        sketch.pad_trail(axis, [finger.distal_pad(ghost) for ghost in trail], 0.2 + 0.8 * trail / 180.0,
                         COLORS[mechanism], alpha=0.6)
        if pose > 0.0:
            sketch.icr_path(axis, finger.icr(np.linspace(0.0, pose, 121)), size=18, linewidth=1.2, end=None)
        sketch.frame(axis)
        axis.set_title(f"φ = {pose:.0f}°", color=INK, fontsize=9, loc="left")
    p, d = finger.pads_mm
    figure.suptitle(f"{LABELS[mechanism]}, {run_dir.name} {role} ({row['run']}): "
                    f"P {p:.1f} + D {d:.1f} mm = {100 * float(row['pad_coverage']):.0f}%, "
                    f"k ≤ {float(row['torque_ratio_max']):.2f}, safe to {float(row['safe_reach_deg']):.0f}°, "
                    f"curl {float(row['curl_deg']):.0f}°", color=MUTED, fontsize=8.5, x=0.01, ha="left")
    out = save(figure, f"fig_design_{run_dir.name}_{role}")
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
