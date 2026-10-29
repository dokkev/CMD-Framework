"""A nominal linkage against the serial 2R with the same finger and pads, side by side.

    python -m visualize.comparison.nominal [seeds.csv] [role]

The linkage is a representative seed of a merged front (default: the
``paper`` preset's ``balanced``). The 2R gets the same finger length (at
most) and the same pads, its effective lengths: its only free choice left is
``|OA|``, taken to reach the most flexion. Both are drawn at the same
flexions: extended, 90 deg, the 2R's most and the linkage's most, MCP at 0;
a pad that collides there is outlined red. Saves ``output/comparison/nominal.png``.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from core.morphology.linkage.packaging import distal_pad_polygon, pad_margins, proximal_pad_polygon  # noqa: E402
from core.optimization.revolute import geometry  # noqa: E402
from core.optimization.revolute.evaluate import _margins  # noqa: E402
from visualize import solids  # noqa: E402
from visualize.comparison.designs import DEFAULT_SEEDS, linkage_seed, matched_revolute  # noqa: E402
from visualize.motion import index_at, servo_sweep  # noqa: E402
from visualize.style import (COLLIDE, COLORS, DISTAL_PAD, INK, MUTED, PROXIMAL_PAD, SERVO,  # noqa: E402
                             save_comparison)


def draw_linkage(axis, linkage, packaging, phi_rad: float) -> str:
    _, sweep = servo_sweep(linkage)
    i = int(index_at(sweep, phi_rad))
    b, c, bp = sweep.b[i], sweep.c[i], sweep.bp[i]
    margins = pad_margins(linkage, packaging, b[None], c[None], bp[None], (0.0,))
    collides = {name for pair, margin in margins.items() if margin[0] < 0.0 for name in pair}
    solids.patch(axis, packaging.mcp_footprint().outline, SERVO, zorder=0.5)
    solids.patch(axis, packaging.pip_footprint(linkage).outline, SERVO, zorder=0.5)
    solids.phalanx(axis, b, c)
    for name, polygon, color in (("proximal_pad", proximal_pad_polygon(linkage, packaging), PROXIMAL_PAD),
                                 ("distal_pad", distal_pad_polygon(linkage, packaging, b[None], c[None])[0], DISTAL_PAD)):
        hit = name in collides
        solids.patch(axis, polygon, color, edge=COLLIDE if hit else MUTED, width=2.0 if hit else 0.8)
    solids.linkage_bars(axis, linkage, b, c, bp, COLORS["linkage"])
    o, a, ap = (0.0, 0.0), (linkage.a.x_mm, linkage.a.z_mm), (linkage.ap.x_mm, linkage.ap.z_mm)
    solids.joints(axis, active=(o, ap), passive=(a, b, bp))
    return f"{math.degrees(sweep.total_flexion_rad[i]):.0f}°" + ("  collides" if collides else "")


def draw_revolute(axis, geo, config, phi_rad: float) -> str:
    packaging = config.goal.packaging
    margins = _margins(geo, np.array([phi_rad]), config)
    collides = {name for pair, margin in margins.items() if margin[0] < 0.0 for name in pair}
    solids.patch(axis, packaging.mcp_footprint().outline, SERVO, zorder=0.5)
    solids.patch(axis, geo.distal_servo.outline, SERVO, zorder=0.5)
    solids.phalanx(axis, (geo.a_x_mm, 0.0), geo.tip([phi_rad])[0])
    pads = [("proximal_pad", geo.proximal_pad, PROXIMAL_PAD), ("distal_pad", geo.distal_pad([phi_rad])[0], DISTAL_PAD)]
    for name, polygon, color in pads:
        hit = name in collides
        solids.patch(axis, polygon, color, edge=COLLIDE if hit else MUTED, width=2.0 if hit else 0.8)
    a = (geo.a_x_mm, 0.0)
    solids.line(axis, ((0.0, 0.0), a), COLORS["revolute"])
    solids.line(axis, (a, geo.tip([phi_rad])[0]), COLORS["revolute"])
    solids.joints(axis, active=((0.0, 0.0), a), passive=())
    short = {"distal_pad": "D pad", "proximal_pad": "P pad", "pip_servo": "PIP servo",
             "mcp_servo": "MCP servo"}
    hits = sorted({"–".join(short[name] for name in pair) for pair, margin in margins.items() if margin[0] < 0.0})
    lines = [", ".join(hits[i:i + 2]) for i in range(0, len(hits), 2)]  # two a line
    return f"{math.degrees(phi_rad):.0f}°" + ("  collides:\n" + "\n".join(lines) if hits else "")


def main(argv: list[str]) -> int:
    seeds = Path(argv[0]) if argv else DEFAULT_SEEDS
    role = argv[1] if len(argv) > 1 else "balanced"
    row, config, linkage = linkage_seed(seeds, role)
    proximal, distal = round(float(row["proximal_effective_mm"])), round(float(row["distal_effective_mm"]))
    finger = round(float(row["finger_length_mm"]), 1)
    _, revolute_config, (x, revolute) = matched_revolute(finger, proximal, distal)
    geo = geometry(x, revolute_config)

    linkage_max, revolute_max = float(row["phi_max_deg"]), math.degrees(revolute.phi_max_rad)
    poses = [0.0, 90.0, revolute_max, linkage_max]
    if abs(revolute_max - 90.0) < 0.5:  # no column twice: one between instead
        poses[2] = 0.5 * (90.0 + linkage_max)
    figure, axes = plt.subplots(2, len(poses), figsize=(4.0 * len(poses), 7.6), facecolor="white",
                                sharex=True, sharey=True, constrained_layout=True)
    for column, phi in enumerate(poses):
        top = draw_linkage(axes[0, column], linkage, config.packaging, math.radians(phi))
        bottom = draw_revolute(axes[1, column], geo, revolute_config, math.radians(phi))
        for axis, text in ((axes[0, column], top), (axes[1, column], bottom)):
            solids.plane_axes(axis)
            axis.set_title(text, color=COLLIDE if "collides" in text else INK, fontsize=10, loc="left")
    axes[0, 0].set_ylabel(f"CoDe linkage\n|OC| {float(row['finger_length_mm']):.0f} mm, k_max "
                          f"{float(row['torque_ratio_max']):.2f}, max {linkage_max:.0f}°", color=COLORS["linkage"])
    axes[1, 0].set_ylabel(f"Serial 2R\n|OC| {revolute.finger_length_mm:.0f} mm (|OA| {x[0]:.1f}), k 1, "
                          f"max {revolute_max:.0f}°", color=COLORS["revolute"])
    axes[0, -1].legend(handles=[
        plt.Line2D([], [], marker="o", linestyle="", markersize=8, markerfacecolor=solids.ACTIVE, markeredgecolor=INK,
                   label="active joint (servo)"),
        plt.Line2D([], [], marker="o", linestyle="", markersize=8, markerfacecolor=solids.PASSIVE, markeredgecolor=INK,
                   label="passive joint")], loc="lower right", frameon=False, fontsize=8)
    figure.suptitle(f"Same finger (|OC| ≤ {finger:.0f} mm) and pads ({proximal} + {distal} mm usable, "
                    f"{config.packaging.proximal_pad.height_mm:.0f} mm thick): the 2R stops at "
                    f"{revolute_max:.0f}° ({revolute.collision_pair.replace('_', ' ')})", color=INK, fontsize=11, x=0.01, ha="left")
    output = save_comparison(figure, "nominal")
    print(f"linkage {role}: |OC| {finger}, pads {proximal} + {distal}, max {linkage_max:.1f} deg; "
          f"2R: |OA| {x[0]}, |AC| {x[1]}, max {revolute_max:.1f} deg ({revolute.collision_pair})")
    print(f"saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
