"""Experiment 40's fingertip slider on a rail, on hand O (slider_rail.py frames -> logs/slider_frames_O_d<draw>.json):
the index finger at the frame's flexion, middle and ring curled away, the rail (a bar on its sprung mount, at its
simulated height) with end blocks, the slider block at its simulated travel, its start outlined faintly, a zigzag
for the tether to the rail's distal end block, and a red arrow along the rail once it has moved. Three frames: the
sweep's start, its middle and the end of the hold. render3d is imported read-only.

    uv run python slider_rail.py frames --draws logs/slider_e40.json --draw 3
    uv run python render_slider.py --draw 3   ->  logs/render_slider{0,1,2}.png, logs/render_slider_strip.png
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import render3d as R
import slider_rail as S

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
BLOCK = [0.55, 0.70, 0.86, 1.0]
CAMERA = {"target_mm": (48.0, 0.0, -24.0), "dist": 0.26, "az_deg": 145.0, "el_deg": 24.0, "fov_deg": 33.0}
"""render3d's coin camera turned a little toward the index finger's outer side, so the rail runs across the image."""


def scene(fr, pl, folder, start):
    sc = R.Scene(folder)
    L, H = pl["len_mm"], S.BLOCK_H_MM
    half = S.DEPTH_MM / 2
    x0 = pl["x_c_mm"]
    z_c = pl["z_top_mm"] - 6.0 - H / 2 + fr["rail_z_mm"]  # the block's centre (world mm), with the rail's drop
    x = x0 - fr["travel_mm"]
    rail_z = z_c - H / 2
    # the rail: a bar under the block, with end blocks
    sc.extrude([np.array([x0 - 40, rail_z - 3]), np.array([x0 + 30, rail_z - 3]), np.array([x0 + 30, rail_z]),
                np.array([x0 - 40, rail_z])], -4.0, 4.0, R.MID)
    for xe in (x0 - 40, x0 + 26):
        sc.extrude([np.array([xe, rail_z - 3]), np.array([xe + 4, rail_z - 3]), np.array([xe + 4, rail_z + 6]),
                    np.array([xe, rail_z + 6])], -6.0, 6.0, R.MID)
    sc.extrude([np.array([x - L / 2, z_c - H / 2]), np.array([x + L / 2, z_c - H / 2]),
                np.array([x + L / 2, z_c + H / 2]), np.array([x - L / 2, z_c + H / 2])], -half, half, BLOCK,
               smooth_deg=0.0)
    # the tether: a zigzag from the block's distal face to the distal end block, in front (+y)
    a_, b_ = x + L / 2, x0 + 26
    n = 10
    pts = [np.array([a_ + (b_ - a_) * k / n, z_c + (1.2 if k % 2 else -1.2) * (0 < k < n)]) for k in range(n + 1)]
    for p_, q_ in zip(pts, pts[1:]):
        sc.bar(p_, q_, 0.35, half + 0.5, half + 1.3, R.STEEL)
    if fr["travel_mm"] > 0.5:
        x_s = x0
        for p_, q_ in ((np.array([x_s - L / 2, z_c - H / 2]), np.array([x_s + L / 2, z_c - H / 2])),
                       (np.array([x_s + L / 2, z_c - H / 2]), np.array([x_s + L / 2, z_c + H / 2])),
                       (np.array([x_s + L / 2, z_c + H / 2]), np.array([x_s - L / 2, z_c + H / 2])),
                       (np.array([x_s - L / 2, z_c + H / 2]), np.array([x_s - L / 2, z_c - H / 2]))):
            sc.bar(p_, q_, 0.35, half + 0.2, half + 0.9, R.GHOST)
        R.arrow(sc, np.array([x + L / 2 - 2, z_c + H / 2 + 4]), np.array([x - L / 2 - 6, z_c + H / 2 + 4]),
                half + 3.0, R.LOAD)
    return sc


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--draw", type=int, default=3)
    a = p.parse_args()
    d = json.loads((LOGS / f"slider_frames_O_d{a.draw}.json").read_text())
    frames, pl = d["frames"], d["placement"]
    n_app = int(round(d["opts"]["approach_s"] * 30))
    n_sw = int(round(d["opts"]["sweep_s"] * 30))
    picks = [frames[n_app - 1], frames[n_app + n_sw // 2], frames[-1]]
    out = []
    for i, fr in enumerate(picks):
        sc = scene(fr, pl, LOGS / "render3d" / f"slider{i}", picks[0])
        out.append(R.render_on_hand(sc, LOGS / f"render_slider{i}.png", math.radians(fr["q_deg"]), camera=CAMERA,
                                    ref=R.TIPDIAL_REF, fingers=R.TIPDIAL_FINGERS, curled=R.TIPDIAL_CURLED))
        print(f"frame {i}: q {fr['q_deg']:.1f} deg, travel {fr['travel_mm']:.2f} mm")
    R.strip(out, LOGS / "render_slider_strip.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
