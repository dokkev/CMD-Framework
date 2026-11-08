"""One design of a run's verified front, drawn through its motion.

    python -m visualize.paper.fig_design <mechanism> <run dir> [seed role]

``run dir``: the study's output directory (its latest portfolio is read);
``seed role``: the representative to draw (default ``balanced``). Panels: the
finger extended, then at every 30 deg of distal flexion up to the most it
reaches, over its distal pad every 15 deg; the dotted curve is the distal
segment's instantaneous centre of rotation, its start a ring
(``visualize.finger.figure.motion_panel``; ``python -m visualize.finger``
draws any design so).
Saves output/paper/fig_design_<run>_<role>.png / .pdf.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from visualize.finger import view_from_row  # noqa: E402
from visualize.finger.figure import motion_panel, poses_to  # noqa: E402
from visualize.fronts import latest_portfolio, read_rows, with_role  # noqa: E402
from visualize.style import LABELS, MUTED, save, use  # noqa: E402

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
    finger = view_from_row(row, run_dir, mechanism)
    poses = poses_to(min(finger.phi_max_deg, finger.reachable_deg()), STEP_DEG)
    rows = -(-len(poses) // COLUMNS)
    figure, axes = plt.subplots(rows, COLUMNS, figsize=(3.4 * COLUMNS, 3.0 * rows), squeeze=False,
                                constrained_layout=True)
    for axis in axes.flat[len(poses):]:
        axis.axis("off")
    for axis, pose in zip(axes.flat, poses):
        motion_panel(axis, finger, pose)
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
