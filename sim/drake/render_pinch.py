"""Experiment 33's pinch-rotate on hand O (pinch_rotate.py frames -> logs/pinch_frames_O_d<draw>.json): the index
finger at the frame's flexion, middle and ring curled away, the sphere (two-tone, so its turn shows) between the index
distal pad and the thumb pad (an abstract opposing pad on its squeeze spring, drawn as a pad on a block). Three frames:
the sweep's start, its middle and the end of the hold. render3d.py is imported read-only.

    uv run python pinch_rotate.py frames --draws logs/pinch_e33.json --draw 0
    uv run python render_pinch.py --draw 0     ->  logs/render_pinch{0,1,2}.png, logs/render_pinch_strip.png
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import render3d as R
import two_pad_drake as T
import pinch_rotate as PR

SPHERE_A, SPHERE_B = [0.93, 0.62, 0.30, 1.0], [0.35, 0.36, 0.42, 1.0]
THUMB_SKIN = [0.80, 0.70, 0.62, 1.0]
CAMERA = {"target_mm": (0.0, 0.0, 0.0), "dist": 0.22, "az_deg": 150.0, "el_deg": 22.0, "fov_deg": 30.0}
"""From the index finger's outer side (+y), a little in front of the fingertip and above: the distal pad, the sphere
and the thumb pad in profile, the curled middle and ring behind."""


def sphere_tris(c_xz, r, spin, n_lat=24, n_lon=48):
    """A UV sphere at (x, 0, z) mm, split into two halves by the plane through its centre that turns with it."""
    a, b = [], []
    axis = np.array([math.cos(spin), 0.0, -math.sin(spin)])  # in-plane direction fixed in the sphere
    ctr = np.array([c_xz[0], 0.0, c_xz[1]])
    def p(i, j):
        th, ph = math.pi * i / n_lat, 2 * math.pi * j / n_lon
        return ctr + r * np.array([math.sin(th) * math.cos(ph), math.cos(th), math.sin(th) * math.sin(ph)])
    for i in range(n_lat):
        for j in range(n_lon):
            q00, q01, q10, q11 = p(i, j), p(i, j + 1), p(i + 1, j), p(i + 1, j + 1)
            for tri in ((q00, q10, q11), (q00, q11, q01)):
                cen = sum(tri) / 3 - ctr
                (a if cen @ axis >= 0 else b).append(np.array(tri))
    return a, b


def build(fr: dict, pl: dict, folder: Path) -> R.Scene:
    sc = R.Scene(folder)
    sx, sz, spin = fr["sphere_mm_rad"]
    ta, tb = sphere_tris((sx, sz), pl["r_mm"], spin)
    for tris, rgba in ((ta, SPHERE_A), (tb, SPHERE_B)):
        # outward winding
        fixed = []
        for t in tris:
            nrm = np.cross(t[1] - t[0], t[2] - t[0])
            fixed.append(t if nrm @ (sum(t) / 3 - np.array([sx, 0.0, sz])) > 0 else t[[0, 2, 1]])
        sc.mesh(fixed, rgba)
    n = np.array(pl["n"])
    t = np.array([-n[1], n[0]])
    f = np.array(fr["thumb_mm"])  # the thumb pad's face centre (x, z)
    hl, th = PR.THUMB_LEN_MM / 2, PR.THUMB_THICK_MM
    pad = [f + hl * t, f - hl * t, f - hl * t + th * n, f + hl * t + th * n]
    w = T.finger_geometry("O")["pads"]["width_mm"] / 2
    sc.extrude(pad, -w, w, R.PAD)
    block = [f + (hl + 2) * t + th * n, f - (hl + 2) * t + th * n, f - (hl - 2) * t + (th + 14) * n,
             f + (hl - 2) * t + (th + 14) * n]
    sc.extrude(block, -w - 2, w + 2, THUMB_SKIN)
    if fr.get("contact_mm"):
        sc.cylinder(fr["contact_mm"], -w - 0.3, w + 0.3, 0.6, R.CONTACT)
    return sc


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--draw", type=int, default=0)
    p.add_argument("--hand", default="O")
    a = p.parse_args()
    d = json.loads((T.OUT / f"pinch_frames_{a.hand}_d{a.draw}.json").read_text())
    frames, pl = d["frames"], d["placement"]
    sweep = [f for f in frames if f["phase"] == "sweep"]
    n_sw = int(round(d["opts"]["sweep_s"] / T.DT_ENV))
    picks = [sweep[0], sweep[n_sw // 2], sweep[-1]]
    out = []
    c0 = picks[0]["sphere_mm_rad"]
    for i, fr in enumerate(picks):
        sc = build(fr, pl, T.OUT / "render3d" / f"pinch{i}")
        cam = {**CAMERA, "target_mm": (c0[0] - 4.0, 0.0, c0[1] + 6.0)}
        out.append(R.render_on_hand(sc, T.OUT / f"render_pinch{i}.png", math.radians(fr["q_deg"]), hand=a.hand,
                                    camera=cam, ref=R.TIPDIAL_REF, fingers=R.TIPDIAL_FINGERS,
                                    curled=R.TIPDIAL_CURLED))
        print(f"frame {i}: q {fr['q_deg']:.1f} deg, sphere turn {math.degrees(fr["sphere_mm_rad"][2] - c0[2]):+.2f} deg")
    R.strip(out, T.OUT / "render_pinch_strip.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
