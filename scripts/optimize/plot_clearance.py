#!/usr/bin/env python3
"""The clearance of every checked pair as the finger flexes, the linkage
against the serial 2R with the same finger and pads, with sketches.

    python scripts/optimize/plot_clearance.py [output/linkage/config/portfolio_001/seeds.csv] [role]

The designs are plot_nominal.py's: a representative linkage seed and the 2R
of at most its length with its effective pads that flexes the most (its
distal pad's drop searched). Each row sketches the finger at 0 deg, 90 deg
and its most flexion (servos as boxes, servo-driven joints yellow, links
red, pads green), then plots each pair's gap against the flexion: below 0
the pair collides, and the first such flexion ends the motion. Saves
``output/comparison/clearance.png``.
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
from matplotlib.patches import Circle, Polygon  # noqa: E402

HERE = Path(__file__).resolve().parent
REPOSITORY_ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPOSITORY_ROOT))
import plot_nominal as nominal  # noqa: E402

import core.morphology.linkage.packaging as packaging_module  # noqa: E402
from core.morphology.linkage import FingerLinkage  # noqa: E402
from core.morphology.linkage.packaging import distal_pad_polygon, pad_margins, proximal_pad_polygon  # noqa: E402
from core.optimization.linkage.design import decode  # noqa: E402
from core.optimization.revolute import geometry  # noqa: E402
from core.optimization.revolute.evaluate import _margins  # noqa: E402
from core.optimization.study import load_run  # noqa: E402

LINK, JOINT, JOINT_EDGE, NODE = "#ff0a1e", "#ffff88", "#d01020", "#111111"
PAD, PAD_EDGE, SERVO_EDGE = "#d5e8d4", "#82b366", "#1f1f1e"
INK, MUTED, GRID = "#1f1f1e", "#6b6a66", "#e6e5e0"
STEP_DEG = 0.25
SHOWN_BELOW_MM = 15.0
"""A pair is plotted when it comes this close somewhere in [0, 180 deg]."""
SHORT = {"distal_pad": "D pad", "proximal_pad": "P pad", "pip_servo": "PIP servo", "mcp_servo": "MCP servo",
         "a_pin": "A pin", "ap_pin": "A' pin", "b_pin": "B pin", "bp_pin": "B' pin", "distal_carrier": "D carrier",
         "proximal_roller": "P roller", "distal_roller": "D roller"}
PAIR_COLORS = ["#2a78d6", "#eb6834", "#1f9e74", "#8a5cd1", "#c9a227", "#d63d8f", "#5b6b7f", "#41a7c9"]
"""Fixed order, one per pair as first met."""


def _sketch_axes(axis) -> None:
    axis.set_aspect("equal")
    axis.axis("off")


def _servo(axis, outline) -> None:
    axis.add_patch(Polygon(np.asarray(outline), closed=True, facecolor="white", edgecolor=SERVO_EDGE,
                           linewidth=1.2, joinstyle="round", zorder=1))


def _pad(axis, polygon, hit: bool = False) -> None:
    axis.add_patch(Polygon(np.asarray(polygon), closed=True, facecolor=PAD, edgecolor="#d03238" if hit else PAD_EDGE,
                           linewidth=2.0 if hit else 1.0, zorder=2))


def _links(axis, *chains) -> None:
    for chain in chains:
        points = np.asarray(chain, dtype=float)
        axis.plot(points[:, 0], points[:, 1], color=LINK, linewidth=3.0, solid_capstyle="round", zorder=3)


def _active(axis, *centers, radius: float = 4.0) -> None:
    for center in centers:
        axis.add_patch(Circle(center, radius, facecolor=JOINT, edgecolor=JOINT_EDGE, linewidth=1.2, zorder=4))
        axis.add_patch(Circle(center, 0.9, facecolor=NODE, edgecolor="none", zorder=5))


def _nodes(axis, *points) -> None:
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    axis.scatter(points[:, 0], points[:, 1], s=16, color=NODE, zorder=5)


def _ground(axis, outline) -> None:
    """Hatching along the MCP servo's back: the palm."""

    outline = np.asarray(outline)
    x = outline[:, 0].min()
    low, high = outline[:, 1].min(), outline[:, 1].max()
    for z in np.arange(low, high + 1e-9, (high - low) / 6.0):
        axis.plot((x, x - 4.0), (z, z - 4.0), color=SERVO_EDGE, linewidth=0.9, zorder=0.5)


def _hits(margins: dict, index: int) -> set[str]:
    return {name for pair, margin in margins.items() if margin[index] < 0.0 for name in pair}


def linkage_sweep(linkage, packaging, phi_max_deg: float):
    step = math.radians(STEP_DEG)
    sense = 1.0 if linkage.sweep_servo([0.0, step]).q_pip_rad[-1] >= 0.0 else -1.0
    sweep = linkage.sweep_servo(sense * np.arange(0.0, math.pi, math.radians(0.05)))
    targets = np.arange(0.0, 180.0 + 1e-9, STEP_DEG)
    total = np.degrees(sweep.total_flexion_rad)
    index = np.array([int(np.flatnonzero(total >= t - 1e-9)[0]) if (total >= t - 1e-9).any() else len(total) - 1
                      for t in targets])
    phi = total[index]
    margins = pad_margins(linkage, packaging, sweep.b[index], sweep.c[index], sweep.bp[index], (0.0,))
    return phi, margins, sweep, index


def draw_linkage(axis, linkage, packaging, sweep, i: int, hit: set[str]) -> None:
    b, c, bp = sweep.b[i], sweep.c[i], sweep.bp[i]
    mcp = packaging.mcp_footprint().outline
    _ground(axis, mcp)
    _servo(axis, mcp)
    _servo(axis, packaging.pip_footprint(linkage).outline)
    _pad(axis, proximal_pad_polygon(linkage, packaging), "proximal_pad" in hit)
    _pad(axis, distal_pad_polygon(linkage, packaging, b[None], c[None])[0], "distal_pad" in hit)
    o, a, ap = (0.0, 0.0), (linkage.a.x_mm, linkage.a.z_mm), (linkage.ap.x_mm, linkage.ap.z_mm)
    _links(axis, (o, a), (o, ap), (ap, b), (a, bp), (bp, b, c, bp))
    _nodes(axis, a, b, bp, c)
    _active(axis, o, ap)


def draw_revolute(axis, geo, packaging, phi_rad: float, hit: set[str]) -> None:
    mcp = packaging.mcp_footprint().outline
    _ground(axis, mcp)
    _servo(axis, mcp)
    _servo(axis, geo.distal_servo.outline)
    _pad(axis, geo.proximal_pad, "proximal_pad" in hit)
    _pad(axis, geo.distal_pad([phi_rad])[0], "distal_pad" in hit)
    a = np.array((geo.a_x_mm, 0.0))
    near, far = geo.posed(geo.distal_mount, [phi_rad])[0]
    _links(axis, ((0.0, 0.0), a), (a, near, far, a))  # the distal phalanx: a truss to the pad's ends
    _nodes(axis, near, far)
    _active(axis, (0.0, 0.0), a)


def clearance_axes(axis, phi, margins: dict, phi_max: float, colors: dict, title: str) -> None:
    axis.grid(color=GRID, linewidth=0.8)
    axis.set_axisbelow(True)
    for side in ("top", "right"):
        axis.spines[side].set_visible(False)
    axis.axhline(0.0, color=INK, linewidth=1.0)
    axis.axvspan(phi_max, 180.0, color="#f3d6d6", alpha=0.5, linewidth=0)
    for pair, margin in margins.items():
        if not np.isfinite(margin).any() or np.nanmin(margin[np.isfinite(margin)]) > SHOWN_BELOW_MM:
            continue  # far all along: not part of the story
        label = " – ".join(SHORT.get(name, name) for name in pair)
        axis.plot(phi, np.where(np.isfinite(margin), margin, np.nan), color=colors.setdefault(
            label, PAIR_COLORS[len(colors) % len(PAIR_COLORS)]), linewidth=1.8, label=label)
    axis.set_xlim(0.0, 180.0)
    axis.set_ylim(-10.0, 30.0)
    axis.set_xticks(range(0, 181, 30))
    axis.set_xlabel("flexion [deg]")
    axis.set_ylabel("gap [mm]")
    axis.set_title(title, color=INK, fontsize=10, loc="left")
    axis.legend(loc="upper right", frameon=False, fontsize=8)


def main(argv: list[str]) -> int:
    # Every gap exact (the checks bound the far ones from below only).
    packaging_module.EXACT_BAND_MM = 1.0e6
    seeds = Path(argv[0]) if argv else REPOSITORY_ROOT / "output/linkage/config/portfolio_001/seeds.csv"
    role = argv[1] if len(argv) > 1 else "balanced"
    rows = list(csv.DictReader(seeds.open(encoding="utf-8", newline="")))
    row = next(r for r in rows if role in r["seed_role"].split(";"))
    run = load_run(REPOSITORY_ROOT / "output" / "linkage" / seeds.parent.parent.name / row["run"].split("/")[-1])
    config = run.config
    linkage = FingerLinkage(decode(run.mechanism.read_design(row), config.link_length_resolution_mm))
    proximal, distal = round(float(row["proximal_effective_mm"])), round(float(row["distal_effective_mm"]))
    finger = round(float(row["finger_length_mm"]), 1)
    _, revolute_config, (x, revolute) = nominal.nominal_revolute(finger, proximal, distal)
    geo = geometry(x, revolute_config)
    linkage_max, revolute_max = float(row["phi_max_deg"]), math.degrees(revolute.phi_max_rad)

    phi_l, margins_l, sweep, index = linkage_sweep(linkage, config.packaging, linkage_max)
    phi_r = np.arange(0.0, 180.0 + 1e-9, STEP_DEG)
    margins_r = _margins(geo, np.radians(phi_r), revolute_config)

    figure = plt.figure(figsize=(17.0, 8.0), facecolor="white", constrained_layout=True)
    grid = figure.add_gridspec(2, 4, width_ratios=(1.0, 1.0, 1.0, 1.25))
    colors: dict[str, str] = {}
    for r, (name, phis, margins, phi_max) in enumerate((("linkage", phi_l, margins_l, linkage_max),
                                                         ("revolute", phi_r, margins_r, revolute_max))):
        sketches = [figure.add_subplot(grid[r, c]) for c in range(3)]
        for axis, pose in zip(sketches, (0.0, 90.0, phi_max)):
            k = int(np.argmin(np.abs(phis - pose)))
            hit = _hits(margins, k)
            if name == "linkage":
                draw_linkage(axis, linkage, config.packaging, sweep, index[k], hit)
            else:
                draw_revolute(axis, geo, revolute_config.goal.packaging, math.radians(pose), hit)
            _sketch_axes(axis)
            axis.set_xlim(-16.0, 102.0)
            axis.set_ylim(-60.0, 30.0)
            axis.set_title(f"{pose:.0f}°", color=INK, fontsize=10, loc="left")
        if name == "linkage":
            label = f"CoDe linkage: |OC| {finger:.0f} mm, k_max {float(row['torque_ratio_max']):.2f}"
            title = f"CoDe linkage: safe to {linkage_max:.0f}°"
        else:
            label = (f"Serial 2R: |OC| {revolute.finger_length_mm:.0f} mm (|OA| {x[0]:.1f}, "
                     f"distal pad {x[4]:.1f} mm below A-C), k 1")
            title = f"Serial 2R: safe to {revolute_max:.0f}° ({revolute.collision_pair.replace('_', ' ')})"
        sketches[0].set_title(f"{label}\n0°", color=INK, fontsize=10, loc="left")
        sketches[0].title.set_color(nominal.COLORS[name])
        clearance_axes(figure.add_subplot(grid[r, 3]), phis, margins, phi_max, colors, title)
    figure.suptitle(f"Clearance of every checked pair as the finger flexes: same finger (|OC| ≤ {finger:.0f} mm) "
                    f"and pads ({proximal} + {distal} mm usable, "
                    f"{config.packaging.proximal_pad.height_mm:.0f} mm tall); shaded: past the safe range",
                    color=INK, fontsize=11, x=0.01, ha="left")
    output = REPOSITORY_ROOT / "output" / "comparison" / "clearance.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=130)
    print(f"linkage {role}: |OC| {finger}, pads {proximal} + {distal}, safe to {linkage_max:.1f} deg; "
          f"2R {x}: safe to {revolute_max:.1f} deg ({revolute.collision_pair})")
    print(f"saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
