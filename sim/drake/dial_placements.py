"""Placements for a cylinder of any radius on hand O: where O's two pad faces both touch it at a setpoint q0 (the rule
of design/output/sim/study3/two_pad_grasps.json). The cylinder rests on the proximal face (centre at depth
face + rho) and its centre lies rho from the distal face line at q0; x_c is solved along the finger.

    uv run python dial_placements.py --radius_mm 10 [--check]   ->  logs/grasps_r<rho>.json

--check recomputes the 15 and 20 mm placements and compares x_c with the grasp file.
"""
import argparse
import json
import math

import numpy as np
from scipy.optimize import brentq

import two_pad_drake as T


def x_contact(geom, qs, th, q0_deg, rho):
    pads = geom["pads"]
    face = pads["mount_offset_mm"] + pads["thickness_mm"]
    q = math.radians(q0_deg)
    theta = float(np.interp(q, qs, th))
    ap, b0 = geom["ap"], geom["b"]
    B = ap + T.rot(theta) @ (b0 - ap)
    R = T.rot(q)
    p0 = B + R @ np.array([0.0, -face])  # a point on the distal face line
    u = R @ np.array([1.0, 0.0])  # the line's direction
    n = R @ np.array([0.0, -1.0])  # its outward (palmar) normal
    zc = -(face + rho)

    def gap(x):
        return float(np.dot(np.array([x, zc]) - p0, n)) - rho

    xs = np.linspace(-20.0, 120.0, 1401)
    g = [gap(x) for x in xs]
    roots = [brentq(gap, xs[i], xs[i + 1]) for i in range(len(xs) - 1) if g[i] * g[i + 1] < 0]
    # the cylinder must sit proximal of the distal face, on the proximal pad
    roots = [x for x in roots if pads["proximal_start_mm"] <= x <= pads["proximal_end_mm"] + rho]
    return min(roots) if roots else None


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--radius_mm", type=float, default=10.0)
    p.add_argument("--check", action="store_true")
    a = p.parse_args()
    geom = T.finger_geometry("O")
    qs, th = T.servo_table(geom)
    if a.check:
        ref = json.loads(T.GRASPS.read_text())["grasps"]
        err = [abs(x_contact(geom, qs, th, g["q0_deg"], g["rho_mm"]) - g["x_c_mm"]) for g in ref]
        print(f"check against {len(ref)} placements: max |dx_c| {max(err):.3f} mm")
    out = []
    for q0 in range(35, 120, 5):
        x = x_contact(geom, qs, th, q0, a.radius_mm)
        if x is not None:
            out.append({"rho_mm": a.radius_mm, "q0_deg": float(q0), "x_c_mm": x})
    path = T.OUT / f"grasps_r{a.radius_mm:g}.json"
    path.write_text(json.dumps({"grasps": out}, indent=1))
    print(path, len(out), [round(g["x_c_mm"], 1) for g in out])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
