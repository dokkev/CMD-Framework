# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Figure (playground experiment): Newton renders of the two-pad grasp on O, R and U, side by side.

    python3 scripts/experimental/figure_grasp_render.py [--rho 15] [--q0 105] [--condition A] [--mu 1.3]

Reads the renders ``two_pad_wrench.py --render logs/experimental/two_pad/render --render_only`` wrote (one
placement: the cylinder where O's pads touch it at q0, all hands under the same command) and annotates each with the
grasp the logged run measured (q*, total distal force, how far the cylinder moved from O's). Writes
``report/study3/figures/study3_two_pad_grasp_render.png``.
"""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np

SIM = Path(__file__).resolve().parents[2]
RENDERS = SIM / "logs" / "experimental" / "two_pad" / "render"
OUT = SIM.parents[1] / "report" / "study3" / "figures" / "study3_two_pad_grasp_render.png"
TITLES = {"O": "O: reference, 3 mm pads", "R": "R: 7 mm, contact trajectory kept\nD_c 0.86 mm, D_s 7.67 mm",
          "U": "U: 7 mm, coverage only\nD_c 3.91 mm, D_s 3.38 mm"}


def state(hand: str, mu: float, cond: str, rho: float, q0: float) -> dict:
    found = None
    for f in sorted(glob.glob(str(SIM / "logs" / "experimental" / "two_pad" / f"tp_{hand}_mu{mu:g}_{cond}_r{rho:g}_*.json"))):
        d = json.loads(Path(f).read_text())
        if d.get("impratio") == 1000.0:
            found = d
    return next(e for e in found["envs"] if e["q0_deg"] == q0 and e["direction"] == "-x")["pre"]


def main() -> int:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    parser = argparse.ArgumentParser()
    parser.add_argument("--rho", type=float, default=15.0)
    parser.add_argument("--q0", type=float, default=105.0)
    parser.add_argument("--condition", default="A")
    parser.add_argument("--mu", type=float, default=1.3)
    parser.add_argument("--crop", type=int, nargs=4, default=(230, 230, 1180, 900), help="left top right bottom [px]")
    args = parser.parse_args()
    o_centre = np.asarray(state("O", args.mu, args.condition, args.rho, args.q0)["centre_face_mm"])
    fig, axes = plt.subplots(1, 3, figsize=(12.0, 4.3))
    for ax, hand in zip(axes, ("O", "R", "U")):
        image = Image.open(RENDERS / f"tp_{hand}_mu{args.mu:g}_{args.condition}_r{args.rho:g}_env0.png").crop(args.crop)
        ax.imshow(image)
        ax.set_axis_off()
        st = state(hand, args.mu, args.condition, args.rho, args.q0)
        shift = float(np.linalg.norm(np.asarray(st["centre_face_mm"]) - o_centre))
        ax.set_title(TITLES[hand], fontsize=9.5, loc="left", color="#1f2328")
        note = f"q* {np.mean(st['q_finger_deg']):.1f}°   distal force {st['n_distal_total']:.1f} N"
        if hand != "O":
            note += f"\ncylinder {shift:.1f} mm from O's"
        ax.text(0.02, -0.02, note, transform=ax.transAxes, fontsize=8.5, color="#1f2328", va="top")
    fig.suptitle(f"Newton (MuJoCo-Warp): the same cylinder (ρ {args.rho:g} mm, placed where O's pads touch it at "
                 f"{args.q0:g}°) under the same flexion command (μ {args.mu:g}), settled and free in the plane",
                 fontsize=9, color="#1f2328", x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0.06, 1, 0.93))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=200)
    print(OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
