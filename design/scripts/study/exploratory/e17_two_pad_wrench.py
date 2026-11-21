#!/usr/bin/env python3
"""Experimental (playground, not the SSoT): does an interface effect on a two-pad grasp survive matching the force?

    python scripts/study/exploratory/e17_two_pad_wrench.py [--flexion 120] [--to 7] [--mu 0.8 1.2] [--radii 15 20]

The friction experiment (sim, axle-held cylinder) measured T_slip = mu rho N: an effect that multiplies the normal
force, so matching the force removed the hands' differences. Here the cylinder is held by both pads and is free in
the sagittal plane, and a force is applied to it in a direction; the load it takes before a contact slips or the
proximal contact opens depends on the contact normals, not only on the force.

Geometry (e16): the two contact faces turn against each other by q on every design, so at one flexion they form the
same wedge and the object meets the same normals. A redesign enters only through the flexion q* at which the hand
grasps a cylinder placed where H0 grasps it at q0 (the normals turn by dq = q* - q0) and where on the distal pad the
contact lands (ds). This script takes q* from e16 for every two-pad grasp on all three fingers' pads.

Statics (proximal-face frame, forces on the cylinder): proximal contact N_p (0, -1) + T_p (1, 0); distal contact
N_d n(q) + T_d t(q), n = (-sin q, -cos q), t = (cos q, -sin q). N_d is fixed (the PIP holds a matched contact force,
the force-matched condition); N_p is a reaction (MCP held); no moment is applied, so T_p + T_d = 0. With an applied
force F d, the three unknowns (N_p, T_p, T_d) are linear in F; the capacity F_max(d) is the largest F with N_p >= 0,
|T_p| <= mu N_p, |T_d| <= mu N_d. With F = 0 the grasp holds only if mu >= tan((180 deg - q) / 2): below that the
cylinder is squeezed out of the wedge (reported as not held).

Effect of the interface change mu_lo -> mu_hi: D(H, d) = F_max(H, mu_hi, d) - F_max(H, mu_lo, d); separability error
E(H, d) = |D(H, d) - D(H0, d)| / |D(H0, d)|. Everything is linear in N_d, so the ratios do not depend on it. Also
counts grasps where the holding itself flips (held on H0, not on the redesign, at one mu).
Reads saved designs only; writes nothing.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import e15_transmission_change as e15  # noqa: E402
import e16_cylinder_regrasp as e16  # noqa: E402
from core.optimization.analysis.contact_kinematics import paths  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

DIRECTIONS = {"+x (to tip)": 0, "+x-z": -45, "-z (off face)": -90, "-x-z": -135, "-x (to MCP)": 180, "-x+z": 135}


def capacity(q: float, mu: float, direction_rad: float, nd: float = 1.0) -> float:
    """F_max (in units of N_d) for an applied force along the direction; nan if not held without load."""
    n_p, t_p = np.array([0.0, -1.0]), np.array([1.0, 0.0])
    n_d, t_d = np.array([-math.sin(q), -math.cos(q)]), np.array([math.cos(q), -math.sin(q)])
    d = np.array([math.cos(direction_rad), math.sin(direction_rad)])
    a = np.array([[n_p[0], t_p[0], t_d[0]], [n_p[1], t_p[1], t_d[1]], [0.0, 1.0, 1.0]])
    x0 = np.linalg.solve(a, -np.array([*(nd * n_d), 0.0]))
    x1 = np.linalg.solve(a, -np.array([*d, 0.0]))
    rows = [(x0[0], x1[0]), (mu * x0[0] - x0[1], mu * x1[0] - x1[1]), (mu * x0[0] + x0[1], mu * x1[0] + x1[1]),
            (mu * nd - x0[2], -x1[2]), (mu * nd + x0[2], x1[2])]
    if min(g0 for g0, _ in rows) < -1e-9:
        return float("nan")
    limits = [-g0 / g1 for g0, g1 in rows if g1 < -1e-12]
    return min(limits) if limits else float("inf")


def grasps(flexion: float, to: float, radii: list[float]) -> list[dict]:
    """Two-pad grasps found on all three fingers' pads: q0 on H0, q* on H_R and H_U (as in e16)."""
    config = read_config_file(e15.REPOSITORY_ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    spec = config["architectures"]["cfl"]
    designs = e15.study1.study1_designs("cfl", spec, flexion, config)
    h0, goal = designs[3.0], designs[to]
    base = mechanism(h0.mechanism)
    import csv

    with (e15.STUDY2 / "front.csv").open(encoding="utf-8", newline="") as stream:
        rows = [r for r in csv.DictReader(stream) if r["architecture"] == "cfl"
                and float(r["required_flexion_deg"]) == flexion and float(r["to_mm"]) == to]
    x_r = tuple(float(v) for v in max(rows, key=lambda r: float(r["c_pad_mean"]))["x"].split(";"))
    folder = e15.study1.condition_folder("cfl", spec, flexion, to, float(config["native_thickness_mm"]))
    x_u = e15.balanced_x(sorted(folder.glob("portfolio_*"))[0])
    ref = paths(base, h0.x, h0.config, samples=e16.SAMPLES)
    q, c0 = np.asarray(ref["q"]), np.asarray(ref["contact"])
    pad0 = e16.pads(base, h0.x, h0.config)
    hands = {"R": x_r, "U": x_u}
    curves = {h: (np.asarray(paths(base, x, goal.config, samples=e16.SAMPLES)["contact"]), e16.pads(base, x, goal.config))
              for h, x in hands.items()}
    out = []
    for rho in radii:
        for q0 in np.arange(20.0, flexion + 0.1, 5.0):
            entry = {"rho_mm": rho, "q0_deg": float(q0), "O": float(q0)}
            for h, (cx, (prox, l2)) in curves.items():
                got = e16.regrasp(q, c0, cx, math.radians(q0), rho)
                if got is None:
                    entry = None
                    break
                dq, ds, x_c, s0, s_x = got
                if not (max(prox[0], pad0[0][0]) <= x_c <= min(prox[1], pad0[0][1]) and -pad0[1] <= s0 <= 0
                        and -l2 <= s_x <= 0):
                    entry = None
                    break
                entry[h] = float(q0 + dq)
                entry[f"ds_{h}"] = ds
            if entry is not None:
                out.append(entry)
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--flexion", type=float, default=120.0)
    parser.add_argument("--to", type=float, default=7.0)
    parser.add_argument("--mu", type=float, nargs=2, default=(0.8, 1.2))
    parser.add_argument("--radii", type=float, nargs="+", default=(15.0, 20.0))
    args = parser.parse_args(argv)
    lo, hi = args.mu
    found = grasps(args.flexion, args.to, list(args.radii))
    print(f"CFL, q_req {args.flexion:g} deg, 3 -> {args.to:g} mm; mu {lo:g} -> {hi:g}; {len(found)} two-pad grasps "
          f"(radii {', '.join(f'{r:g}' for r in args.radii)} mm, q0 every 5 deg)")
    dq = {h: np.array([g[h] - g["O"] for g in found]) for h in ("R", "U")}
    print(f"grasp flexion change |dq|: R median {np.median(np.abs(dq['R'])):.2f} max {np.abs(dq['R']).max():.2f} deg; "
          f"U median {np.median(np.abs(dq['U'])):.2f} max {np.abs(dq['U']).max():.2f} deg")
    print(f"holding needs mu >= tan((180 - q)/2): at q = 90/100/110/120 deg, "
          + "/".join(f"{math.tan(math.radians((180 - qq) / 2)):.2f}" for qq in (90, 100, 110, 120)))
    flips = {h: sum(1 for g in found for mu in (lo, hi)
                    if math.isnan(capacity(math.radians(g["O"]), mu, 0.0)) != math.isnan(capacity(math.radians(g[h]), mu, 0.0)))
             for h in ("R", "U")}
    print(f"holding flips against H0 (grasp x mu): R {flips['R']}, U {flips['U']} of {2 * len(found)}")
    print(f"\nper direction, over grasps held on all hands at both mu: effect D(O) [N per N of N_d], E_R, E_U "
          f"(|D(H) - D(O)| / D(O); median, max)")
    for name, deg in DIRECTIONS.items():
        rows = []
        for g in found:
            f = {h: [capacity(math.radians(g[h]), mu, math.radians(deg)) for mu in (lo, hi)] for h in ("O", "R", "U")}
            if any(math.isnan(v) or math.isinf(v) for vals in f.values() for v in vals):
                continue
            d = {h: f[h][1] - f[h][0] for h in f}
            if abs(d["O"]) < 1e-9:
                continue
            rows.append((d["O"], abs(d["R"] - d["O"]) / abs(d["O"]), abs(d["U"] - d["O"]) / abs(d["O"])))
        if not rows:
            print(f"  {name:14s}: no grasp held on all hands at both mu (or unbounded)")
            continue
        a = np.array(rows)
        print(f"  {name:14s}: n {len(a):3d}  D(O) {np.median(a[:, 0]):5.2f}  E_R {100 * np.median(a[:, 1]):5.1f}% "
              f"(max {100 * a[:, 1].max():5.1f}%)  E_U {100 * np.median(a[:, 2]):5.1f}% (max {100 * a[:, 2].max():5.1f}%)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
