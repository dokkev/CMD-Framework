"""Where a linkage's pads end against its centre of rotation.

    python -m visualize.paper.fig_icr_pads <run dir> [seed role]

``run dir``: a linkage study's output directory (its latest portfolio);
``seed role``: the design to draw (default ``balanced``). Panels at 0, 50,
100 and 150 deg of distal flexion: the finger, the instantaneous centre of
rotation's path from extension (dotted) with its current point (filled),
the vertical through the extended centre (dashed: the proximal pad's limit,
``goal.pads.proximal.past_icr_max_mm``) and the pads' joint-side ends
(``A''``, ``B''``). Saves output/paper/fig_icr_pads_<run>_<role>.png / .pdf.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from core.morphology.mechanism.linkage.measures import extended_icr_x  # noqa: E402
from visualize import sketch  # noqa: E402
from visualize.finger import view_from_row  # noqa: E402
from visualize.fronts import latest_portfolio, read_rows, with_role  # noqa: E402
from visualize.style import COLORS, INK, MUTED, save, use  # noqa: E402

POSES_DEG = (0.0, 50.0, 100.0, 150.0)


def main(argv: list[str]) -> int:
    if not argv:
        print(__doc__)
        return 2
    use("paper")
    run_dir = Path(argv[0]).resolve()
    role = argv[1] if len(argv) > 1 else "balanced"
    row = with_role(read_rows(latest_portfolio(run_dir)), role)
    finger = view_from_row(row, run_dir, "linkage")
    p = finger.state
    icr_x = extended_icr_x((p.joint.a.x_mm, p.joint.a.z_mm), (p.joint.ap.x_mm, p.joint.ap.z_mm), (p.joint.b.x_mm, p.joint.b.z_mm), (p.joint.bp.x_mm, p.joint.bp.z_mm))
    app_x, bpp_x = p.proximal_end.x_mm, p.distal_start.x_mm
    end = min(finger.phi_max_deg, finger.reachable_deg())
    figure, axes = plt.subplots(1, len(POSES_DEG), figsize=(3.3 * len(POSES_DEG), 3.2), constrained_layout=True)
    for axis, pose in zip(axes, POSES_DEG):
        pose = min(pose, end)
        finger.draw(axis, pose, style="sketch")
        sketch.icr_path(axis, finger.icr(np.linspace(0.0, max(pose, 1e-6), 121)), size=26, end=COLORS["rcj"])
        axis.axvline(icr_x, color=COLORS["revolute"], linestyle=(0, (4, 2)), linewidth=1.0, zorder=1)
        axis.scatter([app_x], [-p.joint.a.z_mm - 3.0], marker="|", s=120, color=COLORS["revolute"], zorder=9)
        sketch.frame(axis)
        axis.set_title(f"φ = {pose:.0f}°", color=INK, fontsize=9, loc="left")
    axes[0].text(icr_x + 1.0, 24.0, f"extended ICR x = {icr_x:.1f}", color=COLORS["revolute"], fontsize=7.5)
    axes[0].text(app_x - 1.0, -14.0, f"A'' {app_x:.1f}", color=COLORS["revolute"], fontsize=7.5, ha="right")
    axes[0].text(bpp_x + 1.0, -14.0, f"B'' {bpp_x:.1f}", color=MUTED, fontsize=7.5)
    pads = finger.pads_mm
    figure.suptitle(f"{run_dir.name} {role}: pads {pads[0]:.1f} + {pads[1]:.1f} mm against the centre of rotation "
                    "(dotted: its path from extension, ring; dashed: the extended one's x)",
                    color=MUTED, fontsize=8.5, x=0.01, ha="left")
    out = save(figure, f"fig_icr_pads_{run_dir.name}_{role}")
    print(f"ICR x {icr_x:.1f}, A'' {app_x:.1f}, B'' {bpp_x:.1f}, A {p.joint.a.x_mm:.1f}, B {p.joint.b.x_mm:.1f}")
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
