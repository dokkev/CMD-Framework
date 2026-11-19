#!/usr/bin/env python3
"""Figure: why CoDe's Study 1 result depends on the pivot size and the 2Rs' does not.

    python scripts/study/study1_fixed_morphology/figure_pivot_sweep.py

Top: each 15-degree reference extended (the baseline modules), with A's pin drawn at every swept diameter
(3-9 mm), red where it overlaps a servo body. Bottom: the feasible module pairs per diameter
(``output/study/study1_pivot_sweep/pivot_counts.csv``, from ``pivot_sweep.py``).

Writes ``report/figures/study1_pivot_sweep/study1_pivot_sweep.png``.
"""

from __future__ import annotations

import csv
import importlib
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402
from shapely.geometry import Point  # noqa: E402
from shapely.geometry import Polygon as ShapelyPolygon  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run as study1  # noqa: E402

DESIGN = study1.REPOSITORY_ROOT
sys.path.insert(0, str(DESIGN))
from core.morphology.contact.loss import joint_scene, linkage_scene  # noqa: E402
from core.morphology.mechanism.linkage.design import decode  # noqa: E402
from core.morphology.mechanism.linkage.packaging import pip_footprint  # noqa: E402
from core.optimization.mechanism import read_config_file  # noqa: E402
from visualize.style import COLORS, DISTAL_PAD, INK, MUTED, PROXIMAL_PAD, SERVO, SHORT_LABELS  # noqa: E402

shared = study1.shared
OUT = DESIGN.parent / "report/figures/study1_pivot_sweep/study1_pivot_sweep.png"
COUNTS = DESIGN / "output/study/study1_pivot_sweep/pivot_counts.csv"
DIAMETERS = (3, 4, 5, 6, 7, 8, 9)
REFERENCES = (("code_matched__tilt15", "linkage"), ("belt_2r_matched__tilt15", "revolute_parallel"),
              ("2r_dd_tilt0_matched__tilt15", "revolute"))
CLEAR, HIT = "#2a78d6", "#d03238"


def scene_of(reference):
    """The reference with the baseline modules, extended: its solids, A, the servo that A must clear (or None)."""

    pairs = {p.module_id: p for p in shared.module_catalog(read_config_file(DESIGN / CONFIG["modules"]))}
    x, carried = study1.frozen_design(reference, pairs["baseline"].modules)
    if reference.mechanism == "linkage":
        finger = decode(x, carried.link_length_resolution_mm)
        point = lambda p: np.array([[p.x_mm, p.z_mm]])  # noqa: E731
        scene = linkage_scene(finger, carried.packaging, point(finger.joint.b), point(finger.tip),
                              point(finger.joint.bp), [0.0])
        a = np.array((finger.joint.a.x_mm, finger.joint.a.z_mm))
        ap = np.array((finger.joint.ap.x_mm, finger.joint.ap.z_mm))
        return scene, a, ap, pip_footprint(carried.packaging, finger).outline
    module = importlib.import_module(f"core.optimization.{reference.mechanism}.evaluate")
    geo = module.geometry(x, carried)
    scene = joint_scene(reference.mechanism, geo, carried.goal.packaging, [0.0])
    joint = getattr(geo, "joint", geo)
    a = np.array((joint.a_x_mm, 0.0))
    if reference.mechanism == "revolute_parallel":
        return scene, a, np.asarray(geo.ap, dtype=float), geo.servo.outline
    return scene, a, a, None  # the 2R-DD: A is the servo's own shaft


def room_mm(a, outline) -> float:
    """The largest pin diameter at ``a`` clear of ``outline`` (inf: no servo to clear)."""

    return float("inf") if outline is None else 2.0 * ShapelyPolygon(outline).exterior.distance(Point(a))


def draw_scene(ax, reference_id, kind):
    scene, a, ap, servo = scene_of(REFS[reference_id])
    solids = scene.solids["proximal_pad"]
    ax.add_patch(Polygon(scene.solids["distal_pad"]["other_pad"].polygons[0], closed=True, fc=PROXIMAL_PAD,
                         ec="none", zorder=1))
    ax.add_patch(Polygon(solids["other_pad"].polygons[0][0] if np.ndim(solids["other_pad"].polygons[0]) == 3
                         else solids["other_pad"].polygons[0], closed=True, fc=DISTAL_PAD, ec="none", zorder=1))
    for name in ("mcp_servo", "servo"):
        for poly in solids[name].polygons:
            ax.add_patch(Polygon(np.asarray(poly).reshape(-1, 2), closed=True, fc=SERVO, ec="#9aa1ab", lw=0.8,
                                 alpha=0.9, zorder=2))
    for seg in solids["link"].segments + solids["pulley"].segments:
        s = np.asarray(seg).reshape(2, 2)
        ax.plot(*s.T, color=INK, lw=1.4, zorder=3, solid_capstyle="round")
    for centre, radius in solids["pulley"].disks:
        ax.add_patch(Circle(np.asarray(centre).reshape(2), radius, fill=False, ec=MUTED, lw=1.0, ls=":", zorder=3))
    room = room_mm(a, servo)
    for d in DIAMETERS:
        hit = d > room
        ax.add_patch(Circle(a, d / 2, fill=False, ec=HIT if hit else CLEAR, lw=1.6 if d == 6 else 0.9,
                            ls="-" if d == 6 else (0, (2, 1.5)), zorder=5))
    ax.plot(*a, "o", color=INK, ms=2.5, zorder=6)
    ax.annotate("A", a, xytext=(4, -10), textcoords="offset points", fontsize=9, weight="bold")
    if servo is not None and not np.allclose(ap, a):
        ax.plot(*ap, "+", color=INK, ms=7, zorder=6)
        ax.annotate("A' (PIP servo)", ap, xytext=(4, 6), textcoords="offset points", fontsize=8, color=MUTED)
    ax.set_aspect("equal")
    ax.set_xlim(a[0] - 26, a[0] + 22)
    ax.set_ylim(a[1] - 22, a[1] + 20)
    ax.set_xticks([]), ax.set_yticks([])
    for side in ax.spines.values():
        side.set_color("#d9d8d3")
    if servo is None:
        note = "A is the servo's own shaft:\nthe pin never meets a servo body"
    else:
        note = (f"|AA'| = {np.linalg.norm(a - ap):.2f} mm\nroom for A's pin: ⌀{room:.1f} mm"
                + ("" if room > max(DIAMETERS) else f"\n(7-9 mm overlap the servo)"))
    ax.text(0.03, 0.03, note, transform=ax.transAxes, fontsize=8.5, va="bottom",
            bbox=dict(fc="white", ec="#d9d8d3", lw=0.6, pad=3))
    ax.set_title(f"{SHORT_LABELS[kind]} (15°), extended", fontsize=10, loc="left", color=COLORS[kind])
    return room


def main() -> int:
    global CONFIG, REFS
    CONFIG = read_config_file(HERE / "config.yaml")
    REFS = {r.reference_id: r for r in shared.load_references(("matched",), DESIGN, CONFIG["references"],
                                                              CONFIG["servo_tilt_sweep"])}
    with COUNTS.open(encoding="utf-8") as stream:
        counts = {(r["reference_id"], float(r["required_flexion_deg"])): r for r in csv.DictReader(stream)}

    fig = plt.figure(figsize=(13, 8.4))
    grid = fig.add_gridspec(2, 3, height_ratios=(1.25, 1), hspace=0.16, wspace=0.12)
    for col, (reference_id, kind) in enumerate(REFERENCES):
        draw_scene(fig.add_subplot(grid[0, col]), reference_id, kind)

    bottom = grid[1, :].subgridspec(1, 3, wspace=0.22)
    for col, flexion in enumerate((90.0, 120.0, 150.0)):
        ax = fig.add_subplot(bottom[0, col])
        for reference_id, kind in REFERENCES:
            row = counts[(reference_id, flexion)]
            ax.plot(DIAMETERS, [int(row[f"feasible_d{d}"]) for d in DIAMETERS], color=COLORS[kind], lw=2,
                    marker="o", ms=5, label=SHORT_LABELS[kind], zorder=3)
        ax.axvline(6, color=MUTED, lw=0.8, ls=":", zorder=1)
        ax.axvspan(6.06, 9.4, color=HIT, alpha=0.06, lw=0, zorder=0)
        ax.set_xlim(2.6, 9.4), ax.set_ylim(-2, 40)
        ax.set_xticks(DIAMETERS)
        ax.set_xlabel("pivot diameter [mm] (A, A', B, B' alike)", fontsize=9)
        ax.set_title(f"required flexion {flexion:.0f}°", fontsize=10, loc="left")
        ax.grid(color="#eeeeee", lw=0.6), ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        if col == 0:
            ax.set_ylabel("feasible module pairs (of 71)", fontsize=9)
            ax.text(6.15, 38, "study value", fontsize=7.5, color=MUTED)
            ax.legend(frameon=False, fontsize=8.5, loc="center left")
    fig.text(0.5, 0.965, "Study 1 pivot sweep: CoDe's A pin sits 0.03 mm from the PIP servo; the 2Rs' A pin has room",
             ha="center", fontsize=11.5, weight="bold")
    fig.text(0.5, 0.94, "Top: A's pin at 3-9 mm (blue clear, red overlapping a servo; solid = 6 mm). "
             "Bottom: Study 1 feasible pairs per diameter (shaded: past CoDe's room).",
             ha="center", fontsize=9, color=MUTED)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=160, bbox_inches="tight", facecolor="white")
    print("wrote", OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
