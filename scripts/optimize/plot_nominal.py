#!/usr/bin/env python3
"""A nominal linkage against the serial 2R with the same finger and pads, side by side.

    python scripts/optimize/plot_nominal.py [output/linkage/config/portfolio_001/seeds.csv] [role]

The linkage is a representative seed of a merged front (default: the base
study's ``balanced``). The 2R gets the same finger length (at most) and the
same pads, its effective lengths: its only free choice left is ``|OA|``,
taken to reach the most flexion. Both are drawn at the same flexions:
extended, 90 deg, the 2R's most and the linkage's most, MCP at 0; a pad that
collides there is outlined red. Saves ``output/comparison/nominal.png``.
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
from matplotlib.patches import Polygon  # noqa: E402

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.morphology import Fingertip, FingertipParameters  # noqa: E402
from core.morphology.linkage import FingerLinkage  # noqa: E402
from core.morphology.linkage.packaging import distal_pad_polygon, pad_margins, proximal_pad_polygon  # noqa: E402
from core.optimization.linkage.design import decode  # noqa: E402
from core.optimization.mechanism import mechanism  # noqa: E402
from core.optimization.revolute import geometry  # noqa: E402
from core.optimization.revolute.evaluate import _margins  # noqa: E402
from core.optimization.study import load_run, load_yaml  # noqa: E402

COLORS = {"linkage": "#2a78d6", "revolute": "#eb6834"}
INK, MUTED, GRID, SERVO, COLLIDE = "#1f1f1e", "#6b6a66", "#e6e5e0", "#c9ced6", "#d03238"
ACTIVE, PASSIVE = "#f5c518", "white"
PHALANX = "#e8e4da"
"""Joint fills: driven by a servo, or free."""
PROXIMAL_PAD, DISTAL_PAD = "#f7b6bd", "#b4bdf0"


def nominal_revolute(finger_mm: float, proximal_mm: float, distal_mm: float):
    """The 2R of at most ``finger_mm`` with these pads that flexes the most."""

    found = mechanism("revolute")
    config, _ = load_yaml(found, REPOSITORY_ROOT / "scripts/optimize/revolute/config.yaml", [
        f"goal.pads.proximal.length_mm=[{proximal_mm}, {proximal_mm}]",
        f"goal.pads.distal.length_mm=[{distal_mm}, {distal_mm}]",
        "goal.motion.dip_flexion_deg=10", f"goal.link.finger_length_mm=[40, {finger_mm}]"])
    start = config.goal.packaging.proximal_pad_start_mm
    low, high = config.bounds["distal_drop_mm"]
    tilt = config.bounds["servo_tilt_deg"][0]
    best = None

    def scan(lengths, oas, drops) -> None:
        nonlocal best
        for length in lengths:
            for a in oas:
                ac = length - a
                if ac < 25.0:
                    continue
                for drop in drops:
                    x = tuple(round(float(v), 1) for v in (a, ac, start + proximal_mm, ac - distal_mm, drop,
                                                            config.joint_radius_mm, tilt))
                    e = found.evaluate(x, config)
                    if e.feasible and (best is None or (e.phi_max_rad, -e.finger_length_mm)
                                       > (best[1].phi_max_rad, -best[1].finger_length_mm)):
                        best = (x, e)

    # Coarse over |OC|, |OA| and the distal pad's drop, then fine about the best.
    scan(np.arange(finger_mm - 10.0, finger_mm + 1e-9, 2.0), np.arange(20.0, finger_mm, 2.0),
         np.arange(low, high + 1e-9, 2.0))
    if best is not None:
        a, ac, _, _, drop = best[0]
        scan(np.arange(max(a + ac - 2.0, finger_mm - 10.0), min(a + ac + 2.0, finger_mm) + 1e-9, 0.5),
             np.arange(a - 2.0, a + 2.0 + 1e-9, 0.5), np.arange(max(drop - 2.0, low), min(drop + 2.0, high) + 1e-9, 0.5))
    return found, config, best


def _axes(axis) -> None:
    axis.set_aspect("equal")
    axis.grid(color=GRID, linewidth=0.6)
    axis.set_axisbelow(True)
    for side in ("top", "right"):
        axis.spines[side].set_visible(False)


def _patch(axis, points, color, alpha=0.9, edge=MUTED, width=0.8, zorder=1.0) -> None:
    axis.add_patch(Polygon(np.asarray(points), closed=True, facecolor=color, alpha=alpha,
                           edgecolor=edge, linewidth=width, zorder=zorder))


def _phalanx(axis, joint, tip, color=PHALANX) -> None:
    """The distal phalanx: the fingertip's side silhouette (``core.morphology.Fingertip``)
    as long as its segment, from its joint along it."""

    joint, tip = np.asarray(joint, dtype=float), np.asarray(tip, dtype=float)
    length = float(np.linalg.norm(tip - joint))
    fingertip = Fingertip(FingertipParameters(total_length_mm=length))
    outline = np.array(fingertip.side_silhouette_mm(16), dtype=float)
    outline[:, 1] -= fingertip.rotation_axis_z_mm
    e = (tip - joint) / length
    n = np.array((-e[1], e[0]))
    points = joint + outline[:, :1] * e + outline[:, 1:] * n
    axis.add_patch(Polygon(points, closed=True, facecolor=color, edgecolor=MUTED, linewidth=0.8, zorder=0.3))


def _joints(axis, active, passive) -> None:
    """The servo-driven joints filled yellow, the free ones white."""

    for points, fill in ((passive, PASSIVE), (active, ACTIVE)):
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        axis.scatter(points[:, 0], points[:, 1], s=70, facecolor=fill, edgecolor=INK, linewidth=1.4, zorder=5)


def _line(axis, points, color, alpha=1.0) -> None:
    points = np.asarray(points)
    axis.plot(points[:, 0], points[:, 1], color=color, linewidth=2.4, alpha=alpha, solid_capstyle="round", zorder=3)


def draw_linkage(axis, linkage, packaging, phi_rad: float) -> str:
    step = math.radians(0.05)
    sense = 1.0 if linkage.sweep_servo([0.0, step]).q_pip_rad[-1] >= 0.0 else -1.0
    sweep = linkage.sweep_servo(sense * np.arange(0.0, math.pi, step))
    reached = np.flatnonzero(sweep.total_flexion_rad >= phi_rad - 1e-9)
    i = int(reached[0]) if reached.size else len(sweep.total_flexion_rad) - 1
    b, c, bp = sweep.b[i], sweep.c[i], sweep.bp[i]
    margins = pad_margins(linkage, packaging, b[None], c[None], bp[None], (0.0,))
    collides = {name for pair, margin in margins.items() if margin[0] < 0.0 for name in pair}
    _patch(axis, packaging.mcp_footprint().outline, SERVO, zorder=0.5)
    _patch(axis, packaging.pip_footprint(linkage).outline, SERVO, zorder=0.5)
    _phalanx(axis, b, c)
    for name, polygon, color in (("proximal_pad", proximal_pad_polygon(linkage, packaging), PROXIMAL_PAD),
                                 ("distal_pad", distal_pad_polygon(linkage, packaging, b[None], c[None])[0], DISTAL_PAD)):
        hit = name in collides
        _patch(axis, polygon, color, edge=COLLIDE if hit else MUTED, width=2.0 if hit else 0.8)
    o, a, ap = (0.0, 0.0), (linkage.a.x_mm, linkage.a.z_mm), (linkage.ap.x_mm, linkage.ap.z_mm)
    _line(axis, (o, a, ap, o), COLORS["linkage"])
    _line(axis, (a, bp), INK)
    _line(axis, (ap, b), MUTED)
    _line(axis, (bp, b, c, bp), COLORS["linkage"])
    _joints(axis, active=(o, ap), passive=(a, b, bp))
    return f"{math.degrees(sweep.total_flexion_rad[i]):.0f}°" + ("  collides" if collides else "")


def draw_revolute(axis, geo, config, phi_rad: float) -> str:
    packaging = config.goal.packaging
    margins = _margins(geo, np.array([phi_rad]), config)
    collides = {name for pair, margin in margins.items() if margin[0] < 0.0 for name in pair}
    _patch(axis, packaging.mcp_footprint().outline, SERVO, zorder=0.5)
    _patch(axis, geo.distal_servo.outline, SERVO, zorder=0.5)
    _phalanx(axis, (geo.a_x_mm, 0.0), geo.tip([phi_rad])[0])
    solids = [("proximal_pad", geo.proximal_pad, PROXIMAL_PAD), ("distal_pad", geo.distal_pad([phi_rad])[0], DISTAL_PAD)]
    for name, polygon, color in solids:
        hit = name in collides
        _patch(axis, polygon, color, edge=COLLIDE if hit else MUTED, width=2.0 if hit else 0.8)
    a = (geo.a_x_mm, 0.0)
    _line(axis, ((0.0, 0.0), a), COLORS["revolute"])
    _line(axis, (a, geo.tip([phi_rad])[0]), COLORS["revolute"])
    _joints(axis, active=((0.0, 0.0), a), passive=())
    short = {"distal_pad": "D pad", "proximal_pad": "P pad", "pip_servo": "PIP servo",
             "mcp_servo": "MCP servo"}
    hits = sorted({"–".join(short[name] for name in pair) for pair, margin in margins.items() if margin[0] < 0.0})
    lines = [", ".join(hits[i:i + 2]) for i in range(0, len(hits), 2)]  # two a line
    return f"{math.degrees(phi_rad):.0f}°" + ("  collides:\n" + "\n".join(lines) if hits else "")


def main(argv: list[str]) -> int:
    seeds = Path(argv[0]) if argv else REPOSITORY_ROOT / "output/linkage/config/portfolio_001/seeds.csv"
    role = argv[1] if len(argv) > 1 else "balanced"
    rows = list(csv.DictReader(seeds.open(encoding="utf-8", newline="")))
    row = next(r for r in rows if role in r["seed_role"].split(";"))
    run = load_run(REPOSITORY_ROOT / "output" / "linkage" / seeds.parent.parent.name / row["run"].split("/")[-1])
    config = run.config
    linkage = FingerLinkage(decode(run.mechanism.read_design(row), config.link_length_resolution_mm))
    proximal, distal = round(float(row["proximal_effective_mm"])), round(float(row["distal_effective_mm"]))
    finger = round(float(row["finger_length_mm"]), 1)
    _, revolute_config, (x, revolute) = nominal_revolute(finger, proximal, distal)
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
            _axes(axis)
            axis.set_title(text, color=COLLIDE if "collides" in text else INK, fontsize=10, loc="left")
    axes[0, 0].set_ylabel(f"CoDe linkage\n|OC| {float(row['finger_length_mm']):.0f} mm, k_max "
                          f"{float(row['torque_ratio_max']):.2f}, max {linkage_max:.0f}°", color=COLORS["linkage"])
    axes[1, 0].set_ylabel(f"Serial 2R\n|OC| {revolute.finger_length_mm:.0f} mm (|OA| {x[0]:.1f}), k 1, "
                          f"max {revolute_max:.0f}°", color=COLORS["revolute"])
    axes[0, -1].legend(handles=[
        plt.Line2D([], [], marker="o", linestyle="", markersize=8, markerfacecolor=ACTIVE, markeredgecolor=INK,
                   label="active joint (servo)"),
        plt.Line2D([], [], marker="o", linestyle="", markersize=8, markerfacecolor=PASSIVE, markeredgecolor=INK,
                   label="passive joint")], loc="lower right", frameon=False, fontsize=8)
    figure.suptitle(f"Same finger (|OC| ≤ {finger:.0f} mm) and pads ({proximal} + {distal} mm usable, "
                    f"{config.packaging.proximal_pad.height_mm:.0f} mm thick): the 2R stops at "
                    f"{revolute_max:.0f}° ({revolute.collision_pair.replace('_', ' ')})", color=INK, fontsize=11, x=0.01, ha="left")
    output = REPOSITORY_ROOT / "output" / "comparison" / "nominal.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=130)
    print(f"linkage {role}: |OC| {finger}, pads {proximal} + {distal}, max {linkage_max:.1f} deg; "
          f"2R: |OA| {x[0]}, |AC| {x[1]}, max {revolute_max:.1f} deg ({revolute.collision_pair})")
    print(f"saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
