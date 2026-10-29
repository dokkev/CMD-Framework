"""Why the serial 2R loses its pads: the region the distal pad sweeps while
flexing, the linkage against the 2R with the same finger and pads.

    python -m visualize.comparison.sweep [seeds.csv] [role]

The designs are ``visualize.comparison.nominal``'s: a representative linkage
seed and the 2R of at most its length with its effective pads that flexes
the most. The distal pad (the linkage's with its carrier) is drawn every
STEP_DEG from extension up to each one's most flexion, shaded by flexion;
the 2R's next pose, where it collides, outlined red; and the circle about
``A`` the 2R's distal solids sweep, which its proximal pad must stay out of.
Saves ``output/comparison/sweep.png``.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402

from core.morphology.linkage.packaging import distal_carrier_polygon, distal_pad_polygon, proximal_pad_polygon  # noqa: E402
from core.optimization.revolute import geometry  # noqa: E402
from core.optimization.revolute.evaluate import _margins  # noqa: E402
from visualize import solids  # noqa: E402
from visualize.comparison.designs import DEFAULT_SEEDS, linkage_seed, matched_revolute  # noqa: E402
from visualize.motion import index_at, servo_sweep  # noqa: E402
from visualize.style import COLLIDE, COLORS, INK, PROXIMAL_PAD, SERVO, save_comparison, shades  # noqa: E402

STEP_DEG = 10.0
CARRIER = "#9aa3b5"


def _frame(axis, title: str) -> None:
    solids.plane_axes(axis)
    axis.set_title(title, color=INK, fontsize=10, loc="left")
    axis.set_xlabel("x [mm]")


def draw_linkage(axis, linkage, packaging, phi_max_deg: float, color: str) -> None:
    _, sweep = servo_sweep(linkage)
    cmap = shades(color)
    targets = np.arange(0.0, phi_max_deg + 1e-9, STEP_DEG)
    for phi in targets:
        i = int(index_at(sweep, math.radians(phi)))
        b, c = sweep.b[i][None], sweep.c[i][None]
        shade = cmap(0.25 + 0.75 * phi / max(targets[-1], 1.0))
        solids.patch(axis, distal_carrier_polygon(linkage, packaging, b, c)[0], CARRIER, alpha=0.35, width=0.4)
        solids.patch(axis, distal_pad_polygon(linkage, packaging, b, c)[0], shade, alpha=0.8, width=0.5)
        solids.line(axis, (b[0], c[0]), color, alpha=0.25)
    solids.patch(axis, packaging.mcp_footprint().outline, SERVO, zorder=0.5)
    solids.patch(axis, packaging.pip_footprint(linkage).outline, SERVO, zorder=0.5)
    solids.patch(axis, proximal_pad_polygon(linkage, packaging), PROXIMAL_PAD, zorder=2)
    o, a, ap = (0.0, 0.0), (linkage.a.x_mm, linkage.a.z_mm), (linkage.ap.x_mm, linkage.ap.z_mm)
    solids.line(axis, (o, a, ap, o), color)
    solids.joints(axis, active=(o, ap), passive=(a,))


def draw_revolute(axis, geo, config, phi_max_deg: float, color: str) -> None:
    packaging = config.goal.packaging
    cmap = shades(color)
    targets = np.arange(0.0, phi_max_deg + 1e-9, STEP_DEG)
    a = np.array((geo.a_x_mm, 0.0))
    axis.add_patch(Circle(a, geo.distal_reach_mm, fill=False, edgecolor=color, linewidth=1.2,
                          linestyle=(0, (4, 3)), zorder=4))
    for phi in targets:
        rad = math.radians(phi)
        shade = cmap(0.25 + 0.75 * phi / max(targets[-1], 1.0))
        solids.patch(axis, geo.distal_pad([rad])[0], shade, alpha=0.8, width=0.5)
        solids.line(axis, (a, geo.tip([rad])[0]), color, alpha=0.25)
    # The first pose past the most flexion: where it collides.
    phi = np.arange(phi_max_deg, 180.0 + 1e-9, 0.05)
    margins = _margins(geo, np.radians(phi), config)
    least = np.min(np.stack(list(margins.values())), axis=0)
    bad = np.flatnonzero(least < 0.0)
    if bad.size:
        rad = math.radians(phi[bad[0]])
        solids.patch(axis, geo.distal_pad([rad])[0], "none", edge=COLLIDE, width=1.8, zorder=6)
    solids.patch(axis, packaging.mcp_footprint().outline, SERVO, zorder=0.5)
    solids.patch(axis, geo.distal_servo.outline, SERVO, zorder=0.5)
    solids.patch(axis, geo.proximal_pad, PROXIMAL_PAD, zorder=2)
    solids.line(axis, ((0.0, 0.0), a), color)
    solids.joints(axis, active=((0.0, 0.0), a), passive=())


def main(argv: list[str]) -> int:
    seeds = Path(argv[0]) if argv else DEFAULT_SEEDS
    role = argv[1] if len(argv) > 1 else "balanced"
    row, config, linkage = linkage_seed(seeds, role)
    proximal, distal = round(float(row["proximal_effective_mm"])), round(float(row["distal_effective_mm"]))
    finger = round(float(row["finger_length_mm"]), 1)
    _, revolute_config, (x, revolute) = matched_revolute(finger, proximal, distal)
    geo = geometry(x, revolute_config)
    linkage_max, revolute_max = float(row["phi_max_deg"]), math.degrees(revolute.phi_max_rad)

    figure, axes = plt.subplots(1, 2, figsize=(11.0, 6.2), facecolor="white", sharex=True, sharey=True,
                                constrained_layout=True)
    draw_linkage(axes[0], linkage, config.packaging, linkage_max, COLORS["linkage"])
    draw_revolute(axes[1], geo, revolute_config, revolute_max, COLORS["revolute"])
    _frame(axes[0], f"CoDe linkage: 0-{linkage_max:.0f}°, the distal pad curls under the proximal one")
    _frame(axes[1], f"Serial 2R: 0-{revolute_max:.0f}° ({revolute.collision_pair.replace('_', ' ')}); dashed: what A-C sweeps")
    axes[0].set_ylabel("z [mm]")
    figure.suptitle(f"Same finger (|OC| ≤ {finger:.0f} mm) and pads ({proximal} + {distal} mm, "
                    f"{config.packaging.proximal_pad.height_mm:.0f} mm tall), distal pad every {STEP_DEG:.0f}°\n"
                    "darker: more flexion; grey: the linkage's distal carrier; red: the 2R's first collision",
                    color=INK, fontsize=10, x=0.01, ha="left")
    output = save_comparison(figure, "sweep", dpi=140, bbox_inches="tight")
    print(f"linkage {role}: |OC| {finger}, pads {proximal} + {distal}, to {linkage_max:.1f} deg; "
          f"2R |OA| {x[0]}, |AC| {x[1]}, to {revolute_max:.1f} deg ({revolute.collision_pair}), "
          f"reach {geo.distal_reach_mm:.1f} mm")
    print(f"saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
