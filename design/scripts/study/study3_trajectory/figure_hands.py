#!/usr/bin/env python3
"""Study 3: the three CFL fingers (O = H0, R = H_R, U = H_U) and their trajectories over the flexion.

    python scripts/study/study3_trajectory/figure_hands.py [--flexion 120] [--to 7]

Top row, one panel per finger in the proximal link frame (O at the MCP axis, the link line on z = 0, palmar
down): the linkage and pads at flexions 0, 40, 80 and 120 deg, and the fingertip C's path over [0, q_req].
Bottom row:
- (d) the fingertip paths of the three fingers together (skeleton motion);
- (e) the relative contact trajectories together: the point under the fingertip on the distal contact face, in
  the proximal contact face's frame (each finger's own pad depth), so that the 3 mm and the 7 mm fingers are
  compared face to face;
- (f) how far R and U are from O at each flexion, for the fingertip (dashed) and the contact point (solid).

Writes ``report/study3/figures/study3_hands_q<q>_h3to<h>.png``. Reads saved designs only.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT, ROOT / "scripts" / "study", ROOT / "scripts" / "study" / "exploratory"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import e15_transmission_change as e15  # noqa: E402
from core.morphology.mechanism.linkage.design import decode  # noqa: E402
from core.optimization.analysis.contact_kinematics import paths  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

COLORS = {"O": "#5f6368", "R": "#1f6fd1", "U": "#d9640f"}
LABELS = {"O": "O: 3 mm design", "R": "R: redesigned, trajectory kept", "U": "U: redesigned, coverage only"}
POSES_DEG = (0.0, 40.0, 80.0, 120.0)


def designs(flexion: float, to: float):
    config = read_config_file(ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    spec = config["architectures"]["cfl"]
    found = e15.study1.study1_designs("cfl", spec, flexion, config)
    h0, goal = found[3.0], found[to]
    with (e15.STUDY2 / "front.csv").open(encoding="utf-8", newline="") as stream:
        rows = [r for r in csv.DictReader(stream) if r["architecture"] == "cfl"
                and float(r["required_flexion_deg"]) == flexion and float(r["to_mm"]) == to]
    x_r = tuple(float(v) for v in max(rows, key=lambda r: float(r["c_pad_mean"]))["x"].split(";"))
    folder = e15.study1.condition_folder("cfl", spec, flexion, to, float(config["native_thickness_mm"]))
    x_u = e15.balanced_x(sorted(folder.glob("portfolio_*"))[0])
    return mechanism(h0.mechanism), {"O": (h0.x, h0.config, 3.0), "R": (x_r, goal.config, to), "U": (x_u, goal.config, to)}


def draw_finger(ax, x, config, thickness: float, color: str, q_req: float) -> None:
    finger = decode(x, config.link_length_resolution_mm)
    joint = finger.joint
    mount = float(x[12]) if len(x) > 12 else 3.0
    a, ap = np.array([joint.a.x_mm, joint.a.z_mm]), np.array([joint.ap.x_mm, joint.ap.z_mm])
    # proximal pad (fixed in this frame)
    p0, p1 = finger.proximal.start_mm, finger.proximal.end_mm
    ax.fill([p0, p1, p1, p0], [-mount, -mount, -mount - thickness, -mount - thickness], color=color, alpha=0.18, lw=0)
    ax.plot([0, a[0]], [0, a[1]], color=color, lw=1.6)
    ax.plot([a[0], ap[0]], [a[1], ap[1]], color=color, lw=1.0)
    qs = np.radians(POSES_DEG)
    sweep = decode(x, config.link_length_resolution_mm).sweep_pip(qs)
    for i, q in enumerate(qs[: len(sweep.b)]):
        b, bp, tip = sweep.b[i], sweep.bp[i], sweep.tip[i]
        d = (tip - b) / np.linalg.norm(tip - b)
        n = np.array([d[1], -d[0]])  # palmar normal of the distal segment
        alpha = 0.35 + 0.65 * (i + 1) / len(qs)
        s0, s1 = finger.distal.start_mm, finger.distal.end_mm
        corners = [b + s0 * d - mount * n, b + s1 * d - mount * n, b + s1 * d - (mount + thickness) * n,
                   b + s0 * d - (mount + thickness) * n]
        ax.fill(*np.array(corners).T, color=color, alpha=0.18 * alpha, lw=0)
        ax.plot([a[0], bp[0]], [a[1], bp[1]], color=color, lw=0.8, alpha=alpha)        # driver A-B'
        ax.plot([ap[0], b[0]], [ap[1], b[1]], color=color, lw=0.8, alpha=alpha, ls=":")  # coupler A'-B
        ax.plot([bp[0], b[0], tip[0]], [bp[1], b[1], tip[1]], color=color, lw=1.6, alpha=alpha)
        ax.annotate(f"{POSES_DEG[i]:g}°", tip, textcoords="offset points", xytext=(4, 2), fontsize=7, color="#444")
    q = np.linspace(0.0, q_req, 121)
    tips = decode(x, config.link_length_resolution_mm).sweep_pip(q).tip
    ax.plot(tips[:, 0], tips[:, 1], color=color, lw=1.0, ls="--")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--flexion", type=float, default=120.0)
    parser.add_argument("--to", type=float, default=7.0)
    args = parser.parse_args(argv)
    base, hands = designs(args.flexion, args.to)
    q_req = math.radians(args.flexion)
    traj = {k: paths(base, x, cfg, samples=241) for k, (x, cfg, _) in hands.items()}
    d_c = {k: np.linalg.norm(np.asarray(traj[k]["contact"]) - np.asarray(traj["O"]["contact"]), axis=1) for k in hands}
    d_s = {k: np.linalg.norm(np.asarray(traj[k]["tip"]) - np.asarray(traj["O"]["tip"]), axis=1) for k in hands}
    qdeg = np.degrees(np.asarray(traj["O"]["q"]))

    plt.rcParams.update({"font.size": 8, "axes.spines.top": False, "axes.spines.right": False})
    fig = plt.figure(figsize=(12, 8.2), constrained_layout=True)
    grid = fig.add_gridspec(2, 3, height_ratios=[1.0, 1.0])
    top = [fig.add_subplot(grid[0, i]) for i in range(3)]
    for ax, (k, (x, cfg, h)) in zip(top, hands.items()):
        draw_finger(ax, x, cfg, h, COLORS[k], q_req)
        ax.set_title(f"({'abc'[list(hands).index(k)]}) {LABELS[k]}, {h:g} mm pads", loc="left", color=COLORS[k])
        ax.set_aspect("equal")
        ax.set_xlim(-5, 100)
        ax.set_ylim(-60, 30)
        ax.axhline(0, color="#bbb", lw=0.5)
        ax.set_xlabel("x from the MCP axis (mm)")
    top[0].set_ylabel("z, dorsal up (mm)")

    ax = fig.add_subplot(grid[1, 0])
    for k in hands:
        tip = np.asarray(traj[k]["tip"])
        ax.plot(tip[:, 0], tip[:, 1], color=COLORS[k], lw=1.6, label=k)
    ax.set_title("(d) fingertip C over 0-120° (skeleton)", loc="left")
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("z (mm)")
    ax.legend(frameon=False)

    ax = fig.add_subplot(grid[1, 1])
    for k in hands:
        c = np.asarray(traj[k]["contact"])
        ax.plot(c[:, 0], c[:, 1], color=COLORS[k], lw=1.6, label=k)
        for qd in (0, 40, 80, 120):
            i = int(np.argmin(np.abs(qdeg - qd)))
            ax.plot(*c[i], "o", ms=3, color=COLORS[k])
            if k == "O":
                ax.annotate(f"{qd}°", c[i], textcoords="offset points", xytext=(5, -2), fontsize=7, color="#444")
    ax.set_title("(e) relative contact trajectory (point under the tip,\n     in the proximal contact face's frame)", loc="left")
    ax.set_aspect("equal")
    ax.set_xlabel("along the proximal face (mm)")
    ax.set_ylabel("off the proximal face (mm)")
    ax.legend(frameon=False)

    ax = fig.add_subplot(grid[1, 2])
    for k in ("R", "U"):
        ax.plot(qdeg, d_c[k], color=COLORS[k], lw=1.8, label=f"{k}: contact point (D_c {d_c[k].max():.2f} mm)")
        ax.plot(qdeg, d_s[k], color=COLORS[k], lw=1.2, ls="--", label=f"{k}: fingertip (D_s {d_s[k].max():.2f} mm)")
    ax.set_title("(f) distance from O at the same flexion", loc="left")
    ax.set_xlabel("flexion q (deg)")
    ax.set_ylabel("mm")
    ax.set_xlim(0, args.flexion)
    ax.set_ylim(bottom=0)
    ax.legend(frameon=False, fontsize=7)
    ax.grid(alpha=0.3)

    fig.suptitle(f"CFL fingers for q_req {args.flexion:g}°: O (3 mm) and its two redesigns for {args.to:g} mm pads",
                 x=0.01, ha="left", fontsize=10)
    out = ROOT.parent / "report" / "study3" / "figures" / f"study3_hands_q{args.flexion:g}_h3to{args.to:g}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=170)
    print(out)
    for k in ("R", "U"):
        print(f"{k}: D_c {d_c[k].max():.2f} mm, D_s (fingertip) {d_s[k].max():.2f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
