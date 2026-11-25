"""Experiment 35's block tipped against a stop on hand O (box_pivot.py frames -> logs/pivot_frames_O_d<draw>.json):
the index finger at the frame's flexion, middle and ring curled away (render3d's coin and disk frames), the table at
its simulated height, the stop, the block at its simulated pose (its start outlined faintly), and a red arc arrow
once it tilts. Three frames: the sweep's start, where the tilt first reaches half its final value, and the end of the hold. render3d is imported read-only.

    uv run python box_pivot.py frames --draws logs/pivot_e35.json --draw 2
    uv run python render_pivot.py --draw 2   ->  logs/render_pivot{0,1,2}.png, logs/render_pivot_strip.png
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import box_pivot as B
import render3d as R

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
BLOCK = [0.55, 0.70, 0.86, 1.0]
BLOCK_EDGE = [0.33, 0.45, 0.60, 1.0]
CAMERA = {"target_mm": (45.0, 0.0, -30.0), "dist": 0.26, "az_deg": 160.0, "el_deg": 18.0, "fov_deg": 33.0}
"""From the index finger's outer side (+y), a little in front of the fingertip and above: the block tips in profile."""


def rect(c, w, h, ang):
    """The corners (x, z) of a w x h rectangle centred at c, turned by ang about +y (x' = c x + s z, z' = -s x + c z)."""
    cs, sn = math.cos(ang), math.sin(ang)
    out = []
    for dx, dz in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)):
        out.append(np.array([c[0] + cs * dx + sn * dz, c[1] - sn * dx + cs * dz]))
    return out


def scene(fr, pl, folder, start):
    sc = R.Scene(folder)
    half = B.DEPTH_MM / 2
    w, h = pl["w_mm"], pl["h_mm"]
    bx, bz, ang = fr["block_mm_rad"]
    tz = fr["table_z_mm"]
    # the table slab and the stop (on it, at the block's start palm-side edge)
    x0 = start["block_mm_rad"][0]
    sc.extrude([np.array([x0 - 45, tz - 6]), np.array([x0 + 35, tz - 6]), np.array([x0 + 35, tz]),
                np.array([x0 - 45, tz])], -half - 10, half + 10, R.TABLE)
    sx = x0 - w / 2
    sc.extrude([np.array([sx - B.STOP_W_MM, tz]), np.array([sx, tz]), np.array([sx, tz + B.STOP_H_MM]),
                np.array([sx - B.STOP_W_MM, tz + B.STOP_H_MM])], -half - 10, half + 10, R.MID)
    # the block, and its start outline (faint) once it has moved
    sc.extrude(rect((bx, bz), w, h, ang), -half, half, BLOCK, smooth_deg=0.0)
    turned = ang - start["block_mm_rad"][2]
    if abs(turned) > math.radians(0.5):
        s0 = start["block_mm_rad"]
        pts = rect((s0[0], s0[1] + (tz - start["table_z_mm"])), w, h, s0[2])
        for a, b in zip(pts, pts[1:] + pts[:1]):
            sc.bar(a, b, 0.35, half + 0.2, half + 0.9, R.GHOST)
        # the arc arrow about the pivot corner, in front (+y)
        pivot = rect((bx, bz), w, h, ang)[0]
        r = h + 6.0  # just above the block's top, about the pivot corner
        a0 = math.pi / 2 - (start["block_mm_rad"][2] - 0.0)  # straight up from the pivot at the start
        sgn = 1.0 if turned < 0 else -1.0  # ang < 0: the top moves toward -x (counter-clockwise in (x, z))
        span = max(abs(turned), math.radians(12.0))
        R.arc_arrow(sc, pivot, r, a0 - sgn * math.radians(8), a0 + sgn * (span + math.radians(10)), half + 3.0, R.LOAD)
    if fr.get("contact_mm") and fr.get("f_distal", 0) > 0.05:
        sc.cylinder(fr["contact_mm"], -half - 0.3, half + 0.3, 0.7, R.CONTACT)
    return sc


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--draw", type=int, default=5)
    a = p.parse_args()
    d = json.loads((LOGS / f"pivot_frames_O_d{a.draw}.json").read_text())
    frames, pl = d["frames"], d["placement"]
    n_app = int(round(d["opts"]["approach_s"] / (1.0 / 30.0)))
    n_sw = int(round(d["opts"]["sweep_s"] / (1.0 / 30.0)))
    end = frames[-1]["tilt_deg"]
    mid = next(f for f in frames[n_app:] if f["tilt_deg"] >= 0.5 * end)  # where the block has tipped half way
    picks = [frames[n_app - 1], mid, frames[-1]]
    out = []
    for i, fr in enumerate(picks):
        sc = scene(fr, pl, LOGS / "render3d" / f"pivot{i}", picks[0])
        out.append(R.render_on_hand(sc, LOGS / f"render_pivot{i}.png", math.radians(fr["q_deg"]), camera=CAMERA,
                                    ref=R.TIPDIAL_REF, fingers=R.TIPDIAL_FINGERS, curled=R.TIPDIAL_CURLED))
        print(f"frame {i}: q {fr['q_deg']:.1f} deg, tilt {fr['tilt_deg']:+.2f} deg")
    R.strip(out, LOGS / "render_pivot_strip.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
