"""Experiment 41's fingertip ball on hand O (ball_roll.py frames -> logs/ball_frames_O_d<draw>.json): the index finger
at the frame's flexion, middle and ring curled away, the ball at its simulated height and turn (painted in alternating
wedges about its axis so the turn shows), the socket's cup and base under it, and a red arc arrow for the turn once it
has turned. Three frames: the sweep's start, its middle and the end of the hold. render3d is imported read-only.

    uv run python ball_roll.py frames --draws logs/ball_e41.json --draw 3
    uv run python render_ball.py --draw 3   ->  logs/render_ball{0,1,2}.png, logs/render_ball_strip.png
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import render3d as R

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
BALL_A, BALL_B = [0.93, 0.78, 0.45, 1.0], [0.55, 0.70, 0.86, 1.0]
CAMERA = {"target_mm": (55.0, 0.0, -24.0), "dist": 0.26, "az_deg": 145.0, "el_deg": 24.0, "fov_deg": 33.0}
"""The slider's camera (render_slider.py): the index finger's outer side, the ball's wedges face the camera."""


def sphere(sc, c, r, turn_deg, n_wedge=8, n_lat=24, n_lon=96):
    """A sphere of radius r at c (x, y, z mm) in wedges about y, turned by turn_deg (positive: its top toward -x)."""
    off = math.radians(turn_deg)
    for w in range(n_wedge):
        verts, normals, faces = [], [], []
        a0, a1 = 2 * math.pi * w / n_wedge, 2 * math.pi * (w + 1) / n_wedge
        lons = np.linspace(a0, a1, n_lon // n_wedge + 1)
        lats = np.linspace(-math.pi / 2, math.pi / 2, n_lat + 1)  # latitude measured from the x-z plane, along y
        idx = {}
        for i, la in enumerate(lats):
            for j, lo in enumerate(lons):
                ang = lo + off
                nrm = np.array([math.cos(la) * math.cos(ang), math.sin(la), math.cos(la) * math.sin(ang)])
                idx[i, j] = len(verts)
                verts.append(np.asarray(c) + r * nrm)
                normals.append(nrm)
        for i in range(n_lat):
            for j in range(len(lons) - 1):
                a, b, cc, d = idx[i, j], idx[i, j + 1], idx[i + 1, j + 1], idx[i + 1, j]
                faces += [(a, b, cc), (a, cc, d)]
        v, nn = np.array(verts), np.array(normals)
        sc._write(v, nn, R.fix_winding(v, nn, faces), BALL_A if w % 2 == 0 else BALL_B)


def scene(fr, pl, folder):
    sc = R.Scene(folder)
    r = pl["r_mm"]
    x_c = pl["x_c_mm"]
    z_c = pl["z_c_mm"] - 6.0 + fr["socket_z_mm"]  # the ball's centre (world mm), with the socket's drop
    sphere(sc, (x_c, 0.0, z_c), r, fr["turn_deg"])
    # the socket: a cup ring around the ball's lower half and a base plate under it, both on the sprung seat

    sc.extrude([np.array([x_c - r - 3.0, z_c - r - 1.0]), np.array([x_c + r + 3.0, z_c - r - 1.0]),
                np.array([x_c + r + 3.0, z_c - 0.45 * r]), np.array([x_c + r + 1.0, z_c - 0.45 * r]),
                np.array([x_c + 0.7 * r, z_c - r + 0.2]), np.array([x_c - 0.7 * r, z_c - r + 0.2]),
                np.array([x_c - r - 1.0, z_c - 0.45 * r]), np.array([x_c - r - 3.0, z_c - 0.45 * r])],
               -r - 3.0, r + 3.0, R.MID, smooth_deg=0.0)
    sc.extrude([np.array([x_c - r - 10, z_c - r - 5.0]), np.array([x_c + r + 10, z_c - r - 5.0]),
                np.array([x_c + r + 10, z_c - r - 1.0]), np.array([x_c - r - 10, z_c - r - 1.0])],
               -r - 8.0, r + 8.0, R.TABLE, smooth_deg=0.0)
    if fr["turn_deg"] > 3.0:  # the turn: an arc arrow in front of the ball, its top toward the palm
        R.arc_arrow(sc, np.array([x_c, z_c]), r + 3.5, math.radians(40.0), math.radians(40.0 + min(fr["turn_deg"], 120)),
                    r + 1.5, R.LOAD)
    return sc


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--draw", type=int, default=3)
    a = p.parse_args()
    d = json.loads((LOGS / f"ball_frames_O_d{a.draw}.json").read_text())
    frames, pl = d["frames"], d["placement"]
    n_app = int(round(d["opts"]["approach_s"] * 30))
    n_sw = int(round(d["opts"]["sweep_s"] * 30))
    picks = [frames[n_app - 1], frames[n_app + n_sw // 2], frames[-1]]
    out = []
    for i, fr in enumerate(picks):
        sc = scene(fr, pl, LOGS / "render3d" / f"ball{i}")
        out.append(R.render_on_hand(sc, LOGS / f"render_ball{i}.png", math.radians(fr["q_deg"]), camera=CAMERA,
                                    ref=R.TIPDIAL_REF, fingers=R.TIPDIAL_FINGERS, curled=R.TIPDIAL_CURLED))
        print(f"frame {i}: q {fr['q_deg']:.1f} deg, turn {fr['turn_deg']:.1f} deg, socket {fr['socket_z_mm']:.1f} mm")
    R.strip(out, LOGS / "render_ball_strip.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
