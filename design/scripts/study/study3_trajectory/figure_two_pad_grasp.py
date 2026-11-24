#!/usr/bin/env python3
"""Figure (playground experiment, not the SSoT): the two-pad grasp of a free cylinder on O, R and U.

    python scripts/study/study3_trajectory/figure_two_pad_grasp.py [--rho 15] [--q0 105] [--condition A] [--mu 1.3]

One placement of the third experiment (``report/study3/experiments.md``): the cylinder put where O's two pads
touch it at q0, each hand closed by the same flexion command (condition A), the grasp settled with the cylinder free
in the plane. Drawn in the proximal contact face's frame (x along the face from the MCP axis, palm up): the pads from
each design's own kinematics at the grasp flexion q* the simulation measured (the distal face turns by q* against the
proximal one on every design), the cylinder, contact points and the contact forces on it from the simulation
(``sim/isaaclab/logs/study3/two_pad``, latest impratio-1000 run), O's cylinder dashed on R and U, and the four
load directions. Writes ``report/study3/figures/study3_two_pad_grasp.png``.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT, ROOT / "scripts" / "study", ROOT / "scripts" / "study" / "exploratory"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
import e15_transmission_change as e15  # noqa: E402
import e16_cylinder_regrasp as e16  # noqa: E402
from core.optimization.analysis.contact_kinematics import paths  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

LOGS = ROOT.parent / "sim" / "isaaclab" / "logs" / "study3" / "two_pad"
HANDS = ROOT / "output" / "sim" / "study3" / "hands"
OUT = ROOT.parent / "report" / "study3" / "figures" / "study3_two_pad_grasp.png"
LABELS = {"O": ("O", "reference, 3 mm pads"), "R": ("R", "7 mm, contact trajectory kept"),
          "U": ("U", "7 mm, coverage only")}
METRICS = {"O": None, "R": (0.86, 7.67), "U": (3.91, 3.38)}  # D_c, D_s [mm] (e18 / Study 2)
FORCE_MM_PER_N = 0.7
DIRECTIONS = {"+x": (1.0, 0.0), "+x−z": (math.sqrt(0.5), -math.sqrt(0.5)), "−z": (0.0, -1.0), "−x": (-1.0, 0.0)}


def sim_state(hand: str, mu: float, cond: str, rho: float, q0: float) -> dict:
    found = None
    for f in sorted(glob.glob(str(LOGS / f"tp_{hand}_mu{mu:g}_{cond}_r{rho:g}_*.json"))):
        d = json.loads(Path(f).read_text())
        if d.get("impratio") == 1000.0:
            found = d
    if found is None:
        raise FileNotFoundError(f"no run for {hand} mu {mu:g} {cond} r{rho:g}")
    return next(e for e in found["envs"] if e["q0_deg"] == q0 and e["direction"] == "-x")["pre"]


def main(argv=None) -> int:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, FancyArrowPatch, Polygon

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--rho", type=float, default=15.0)
    parser.add_argument("--q0", type=float, default=105.0)
    parser.add_argument("--condition", default="A")
    parser.add_argument("--mu", type=float, default=1.3)
    args = parser.parse_args(argv)

    config = read_config_file(ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    designs = e15.study1.study1_designs("cfl", config["architectures"]["cfl"], 120.0, config)
    base = mechanism(designs[3.0].mechanism)
    geometry = {}
    for hand, stem, cfg, h in (("O", "h0", designs[3.0].config, 3.0), ("R", "hr", designs[7.0].config, 7.0),
                               ("U", "hu", designs[7.0].config, 7.0)):
        x = tuple(yaml.safe_load((HANDS / f"cfl_q120_h3to7_{stem}.yaml").read_text())["source"]["design"])
        p = paths(base, x, cfg, samples=e16.SAMPLES)
        (prox, l2) = e16.pads(base, x, cfg)
        geometry[hand] = (np.asarray(p["q"]), np.asarray(p["contact"]), prox, l2, h)

    ink, muted, grid = "#1f2328", "#6e7781", "#d0d7de"
    pad_fc, pad_ec = "#f6d38a", "#9a6700"
    force_c, obj_fc = "#0550ae", "#eaeef2"
    fig, axes = plt.subplots(1, 3, figsize=(12.0, 5.0), sharey=True)
    flip = np.array([1.0, -1.0])  # face frame (object at z < 0) -> drawing (palm up: object above the proximal pad)
    o_state = sim_state("O", args.mu, args.condition, args.rho, args.q0)
    o_centre = np.asarray(o_state["centre_face_mm"]) * flip
    for ax, hand in zip(axes, ("O", "R", "U")):
        q, contact, prox, l2, h = geometry[hand]
        st = sim_state(hand, args.mu, args.condition, args.rho, args.q0)
        q_star = math.radians(float(np.mean(st["q_finger_deg"])))
        c = np.array([np.interp(q_star, q, contact[:, 0]), np.interp(q_star, q, contact[:, 1])])
        n = np.array([-math.sin(q_star), -math.cos(q_star)])  # distal face normal, toward the object
        t = np.array([math.cos(q_star), -math.sin(q_star)])
        # proximal pad (face z = 0, body on z in [0, h]) and distal pad (face through c along t, body behind it)
        ax.add_patch(Polygon([(prox[0], 0), (prox[1], 0), (prox[1], -h), (prox[0], -h)], closed=True,
                             fc=pad_fc, ec=pad_ec, lw=1.0, zorder=2))
        face = [c + s * t for s in (-l2, 0.0)]
        ax.add_patch(Polygon([p * flip for p in (face[0], face[1], face[1] - h * n, face[0] - h * n)], closed=True,
                             fc=pad_fc, ec=pad_ec, lw=1.0, zorder=2))
        centre = np.asarray(st["centre_face_mm"]) * flip
        if hand != "O":
            ax.add_patch(Circle(o_centre, args.rho, fill=False, ec=muted, lw=0.9, ls=(0, (3, 2)), zorder=3))
        ax.add_patch(Circle(centre, args.rho, fc=obj_fc, ec=ink, lw=1.1, zorder=3))
        ax.plot(*centre, marker="+", color=ink, ms=6, zorder=4)
        # contact forces on the cylinder (all three fingers' total along the middle finger's normal)
        pads = {p["pad"]: p for p in st["pads"]}
        for side, total in (("proximal", st["n_proximal_total"]), ("distal", st["n_distal_total"])):
            pad = pads[f"middle_{side}"]
            if not pad["point_face_mm"]:
                continue
            point = np.asarray(pad["point_face_mm"]) * flip
            unit = np.asarray(pad["normal_face"]) * flip
            unit = unit / np.linalg.norm(unit)
            ax.add_patch(FancyArrowPatch(point, point + FORCE_MM_PER_N * total * unit, arrowstyle="-|>", mutation_scale=9,
                                         color=force_c, lw=1.4, zorder=5))
            ax.plot(*point, "o", color=force_c, ms=3.5, zorder=5)
        name, sub = LABELS[hand]
        metric = "" if METRICS[hand] is None else f"\nD_c {METRICS[hand][0]:.2f} mm, D_s {METRICS[hand][1]:.2f} mm"
        ax.set_title(f"{name}: {sub}{metric}", fontsize=9.5, loc="left", color=ink)
        shift = np.linalg.norm(centre - o_centre)
        ax.text(0.03, 0.97, f"q* {math.degrees(q_star):.1f}°\nN_d {st['n_distal_total']:.1f} N"
                + ("" if hand == "O" else f"\ncylinder moved {shift:.1f} mm"), transform=ax.transAxes,
                fontsize=8.5, color=ink, va="top")
        ax.set_aspect("equal")
        ax.set_xlim(-2, 58)
        ax.set_ylim(-12, 42)
        ax.grid(color=grid, lw=0.5)
        ax.tick_params(labelsize=7.5, colors=muted)
        for s in ax.spines.values():
            s.set_color(grid)
        ax.set_xlabel("along the proximal contact face from the MCP axis (mm)", fontsize=8, color=muted)
    axes[0].set_ylabel("normal to the proximal face (mm)", fontsize=8, color=muted)
    # load directions, on O's panel
    ax = axes[0]
    origin = np.array([46.0, 34.0])
    for label, d in DIRECTIONS.items():
        v = np.asarray(d) * flip * 6.0
        ax.add_patch(FancyArrowPatch(origin, origin + v, arrowstyle="-|>", mutation_scale=8, color=muted, lw=1.0))
        ax.text(*(origin + 1.35 * v), label, fontsize=7.5, color=muted, ha="center", va="center")
    ax.text(origin[0], origin[1] + 9.5, "load on the cylinder", fontsize=7.5, color=muted, ha="center")
    fig.suptitle(f"Two-pad grasp of a free cylinder (ρ {args.rho:g} mm) placed where O's pads touch it at "
                 f"{args.q0:g}°, same flexion command (condition {args.condition}, μ {args.mu:g})\n"
                 f"Arrows: total contact force on the cylinder from each pad ({FORCE_MM_PER_N:g} mm per N); "
                 "dashed circle: O's cylinder",
                 fontsize=9, color=ink, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0.02, 1, 0.91))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=200)
    print(OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
