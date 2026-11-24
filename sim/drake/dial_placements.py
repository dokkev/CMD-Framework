"""Placements for a cylinder of any radius on hand O: where O's two pad faces both touch it at a setpoint q0 (the rule
of design/output/sim/study3/two_pad_grasps.json). The cylinder rests on the proximal face (centre at depth
face + rho) and its centre lies rho from the distal face line at q0; x_c is solved along the finger.

    uv run python dial_placements.py --radius_mm 10 [--check]   ->  logs/grasps_r<rho>.json

--check recomputes the 15 and 20 mm placements and compares x_c with the grasp file.

--tip y|z: experiment 28's fingertip dial instead, placed on O at its touch angle q_t (stored as q0_deg), in the
proximal-face frame (x along the face, z dorsal; the same coordinates on every hand):
  y: a dial in the finger's plane, tangent to O's distal pad at q_t at the pad point back_mm short of its tip end;
     centre = that point + rho along the pad's outward normal; its axle springs along that normal (axis_deg = q_t).
  z: a disk with its axis along z whose top face is at the depth of O's fingertip face point at q_t; its centre is
     x_back_mm proximal of that point and y_mm to the side of the finger's plane.

    uv run python dial_placements.py --tip y --radius_mm 8 12 --q_touch 20 35 50 --back_mm 3 8 [--out file]
    uv run python dial_placements.py --tip z --radius_mm 12 --q_touch 20 35 50 --x_back_mm 6 --y_mm 9 [--thick_mm 4]

--random N --seed S: experiment 29's randomized start conditions instead of a grid: N draws from the fixed
distributions RANDOM (independent uniforms; y: q_t, rho, back, k_r; z: q_t, rho, x_back, y_off, k_r, each drawn as a
vector of N in that order from numpy.random.default_rng([S, 1]) for y, [S, 2] for z). Each draw also carries its
torsional spring tip_kr (N m/rad) and its index 'draw'.

    uv run python dial_placements.py --tip y --random 64 --seed 29 --out logs/tipdial_y_e29.json
"""
import argparse
import json
import math
from pathlib import Path

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


# experiment 29's distributions (fixed before any data; docs/experimental/drake_dial_random.md): (low, high), uniform
RANDOM = {
    "y": {"q0_deg": (15.0, 70.0), "rho_mm": (9.0, 13.0), "back_mm": (9.0, 14.0), "tip_kr": (0.01, 0.03)},
    "z": {"q0_deg": (12.0, 40.0), "rho_mm": (13.0, 17.0), "x_back_mm": (8.0, 16.0), "y_mm": (6.0, 14.0),
          "tip_kr": (0.005, 0.015)},
}


def random_placements(tip: str, n: int, seed: int, thick_mm: float = 4.0) -> list[dict]:
    rng = np.random.default_rng([seed, 1 if tip == "y" else 2])
    cols = {k: rng.uniform(lo, hi, n) for k, (lo, hi) in RANDOM[tip].items()}
    geom = T.finger_geometry("O")
    qs, th = T.servo_table(geom)
    end = geom["pads"]["distal_end_mm"]
    out = []
    for i in range(n):
        d = {k: float(v[i]) for k, v in cols.items()}
        q = math.radians(d["q0_deg"])
        if tip == "y":
            nrm = T.rot(q) @ np.array([0.0, -1.0])
            c = T.reference_point(geom, qs, th, q, end - d["back_mm"]) + d["rho_mm"] * nrm
            out.append({"draw": i, **d, "x_c_mm": float(c[0]), "z_c_mm": float(c[1]), "axis_deg": d["q0_deg"]})
        else:
            tp = T.reference_point(geom, qs, th, q, "tip")
            out.append({"draw": i, **d, "x_c_mm": float(tp[0] - d["x_back_mm"]), "z_c_mm": float(tp[1] - thick_mm / 2),
                        "thick_mm": thick_mm})
    return out


def tip_placements(a) -> list[dict]:
    geom = T.finger_geometry("O")
    qs, th = T.servo_table(geom)
    end = geom["pads"]["distal_end_mm"]
    out = []
    for rho in a.radius_mm:
        for qt in a.q_touch:
            q = math.radians(qt)
            n = T.rot(q) @ np.array([0.0, -1.0])  # the distal pad's outward (palmar) normal
            if a.tip == "y":
                for b in a.back_mm:
                    c = T.reference_point(geom, qs, th, q, end - b) + rho * n
                    out.append({"rho_mm": rho, "q0_deg": float(qt), "x_c_mm": float(c[0]), "z_c_mm": float(c[1]),
                                "axis_deg": float(qt), "back_mm": b})
            else:
                tip = T.reference_point(geom, qs, th, q, "tip")
                for xb in a.x_back_mm:
                    for y in a.y_mm:
                        out.append({"rho_mm": rho, "q0_deg": float(qt), "x_c_mm": float(tip[0] - xb),
                                    "z_c_mm": float(tip[1] - a.thick_mm / 2), "y_mm": y, "x_back_mm": xb,
                                    "thick_mm": a.thick_mm})
    return out


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--radius_mm", type=float, nargs="+", default=[10.0])
    p.add_argument("--check", action="store_true")
    p.add_argument("--tip", choices=("y", "z"), default=None)
    p.add_argument("--q_touch", type=float, nargs="+", default=[20.0, 35.0, 50.0])
    p.add_argument("--back_mm", type=float, nargs="+", default=[3.0, 8.0])
    p.add_argument("--x_back_mm", type=float, nargs="+", default=[6.0])
    p.add_argument("--y_mm", type=float, nargs="+", default=[9.0])
    p.add_argument("--thick_mm", type=float, default=4.0)
    p.add_argument("--random", type=int, default=None, help="--tip: this many random draws (experiment 29)")
    p.add_argument("--seed", type=int, default=29)
    p.add_argument("--out", type=Path, default=None)
    a = p.parse_args()
    if a.tip:
        out = random_placements(a.tip, a.random, a.seed, a.thick_mm) if a.random else tip_placements(a)
        path = a.out or T.OUT / f"tipdial_{a.tip}.json"
        meta = {"seed": a.seed, "n": a.random, "ranges": RANDOM[a.tip]} if a.random else {}
        path.write_text(json.dumps({**meta, "grasps": out}, indent=1))
        print(path, len(out))
        for g in out:
            print("  ", {k: round(v, 2) if isinstance(v, float) else v for k, v in g.items()})
        return 0
    a.radius_mm = a.radius_mm[0]
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
