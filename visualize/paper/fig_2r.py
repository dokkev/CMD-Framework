"""The serial 2R's best design at every interface thickness and required
flexion of the interface coverage study, drawn to one scale.

    python -m visualize.paper.fig_2r [mechanism]

``mechanism``: ``revolute`` (default, the servo on the joint) or
``revolute_idealized``. Each panel: the finger at the required flexion over
its distal pad every 15 deg from extension, darker as it flexes, the pads'
lengths, the coverage, the servo's tilt and the pads' drops in the title.
A cell without a front yet reads "running"; one with no feasible design, "infeasible".
Saves output/paper/fig_2r_<mechanism>.png / .pdf.
"""

from __future__ import annotations

import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from core.optimization.revolute.config import VARIABLE_NAMES as REVOLUTE_NAMES  # noqa: E402
from visualize import sketch, studies  # noqa: E402
from visualize.style import COLORS, INK, LABELS, MUTED, save, use  # noqa: E402

STUDY = studies.grid("interface_coverage")


def panel(axis, mechanism: str, height: int, flexion: int) -> str:
    sketch.frame(axis)
    if studies.latest_front(mechanism, height, flexion) is None:
        axis.text(0.5, 0.5, "running", transform=axis.transAxes, ha="center", color=MUTED)
        return ""
    finger = studies.best(mechanism, height, flexion)
    if finger is None:
        axis.text(0.5, 0.5, "infeasible", transform=axis.transAxes, ha="center", color=INK)
        return ""
    end = min(float(flexion), finger.reachable_deg())
    finger.draw(axis, end)
    trail = np.arange(0.0, end + 1e-9, 15.0)
    sketch.pad_trail(axis, [finger.distal_pad(pose) for pose in trail], 0.2 + 0.8 * trail / 180.0,
                     COLORS[mechanism])
    v = dict(zip(REVOLUTE_NAMES, finger.x))
    p, d = finger.pads_mm
    axis.set_title(f"P {p:.0f} + D {d:.0f} mm = {float(finger.row['pad_coverage']) * 100:.0f}%\n"
                   + (f"tilt {v['servo_tilt_deg']:.0f}°, " if finger.config.joint_servo else "")
                   + f"drop P {v['proximal_drop_mm']:.1f} / D {v['distal_drop_mm']:.1f}",
                   color=INK, fontsize=8, loc="left")
    return (f"h{height:>2d} {flexion}°: {100 * float(finger.row['pad_coverage']):.0f}% (P {p:.1f} + D {d:.1f}), "
            f"|OA| {v['a_x_mm']:.1f}, tilt {v['servo_tilt_deg']:.0f}, drops P {v['proximal_drop_mm']:.1f} "
            f"D {v['distal_drop_mm']:.1f}, gap past A {v['bpp_from_a_mm']:.1f}, safe to {finger.phi_max_deg:.1f}")


def main(argv: list[str]) -> int:
    use("paper")
    mechanism = argv[0] if argv else "revolute"
    heights, flexions = sorted(STUDY.THICKNESSES_MM), sorted(STUDY.FLEXIONS_DEG)
    figure, axes = plt.subplots(len(heights), len(flexions), figsize=(3.1 * len(flexions), 2.9 * len(heights)),
                                squeeze=False, constrained_layout=True)
    lines = []
    for r, height in enumerate(heights):
        for c, flexion in enumerate(flexions):
            line = panel(axes[r, c], mechanism, height, flexion)
            if line:
                lines.append(line)
            if c == 0:
                axes[r, c].text(-0.08, 0.5, f"{height} mm", transform=axes[r, c].transAxes, rotation=90,
                                ha="right", va="center", color=INK)
            if r == 0:
                axes[r, c].text(0.5, 1.32, f"{flexion}° required", transform=axes[r, c].transAxes,
                                ha="center", color=INK)
    figure.suptitle(f"{LABELS[mechanism]}: best design per interface thickness (rows) and required "
                    "flexion (columns); pads off the joint, each ≥ 20 mm, |OC| 100 mm",
                    color=MUTED, fontsize=8.5, x=0.01, ha="left")
    out = save(figure, f"fig_2r_{mechanism}")
    print("\n".join(lines))
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
