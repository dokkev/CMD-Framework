#!/usr/bin/env python3
"""What a flexion-commanded controller must change to hold the same cylinder on a redesigned CFL finger.

    python scripts/study/exploratory/e16_cylinder_regrasp.py [--flexion 90] [--to 5]

Derivation (proximal contact face frame: the face is the line z = 0, the object on z < 0). At flexion q
the distal contact face passes through c(q), the point under the fingertip on it (the relative contact
trajectory), with outward normal n(q) = (-sin q, -cos q) and tangent t(q) = (cos q, -sin q). The two faces
turn against each other by q on any design, so at the same q two fingers' distal faces are parallel and
differ only by dc(q) = c_X(q) - c_0(q): a normal offset n.dc (the face moves into or away from a held
object) and a tangential offset t.dc (it slides along itself). No controller can remove these at the same
q: the relative pose is one point of a curve fixed by the mechanism (problem_formulation proposition 4).

A cylinder of radius rho touching both faces has its centre at (x_c, -rho) with n(q0).(centre - c(q0)) =
rho, so x_c = (rho + n_z rho + n.c) / n_x on the reference finger H0 at flexion q0. Held at that centre,
the cylinder is touched by finger X's distal face at the q* with n(q*).(centre - c_X(q*)) = rho; the
controller must change the flexion by dq = q* - q0, and the contact on the distal face moves by
ds = s_X - s_0, s = t.(centre - rho n - c) its position along the face from the point under the tip.

Only grasps with both contacts on pads count: the proximal contact x_c on both fingers' proximal pads
([start, end] of the proximal segment's support, from O along the proximal face) and the distal contact
within the distal pad (s in [-l2, 0], the pad laid back from the fingertip). Near the distal face's pole
foot the face barely moves toward a held object as it turns (the sensitivity is the distance along the face
from the contact to the pole's foot), so dq can be large for a small offset; the primary quantity is
therefore the normal offset at equal flexion, n.dc (positive: the face moves into the object, negative:
away from it), and dq, ds are reported where the re-grasp stays on the pads.

Reports, for H_R and H_U against H0 (designs as in e15): the offsets n.dc and t.dc at equal flexion over
the valid two-pad grasps, and dq and ds over cylinder radii and reference flexions. A self-check re-solves
H0 against itself (dq = 0).
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
from core.optimization.analysis.contact_kinematics import paths  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

SAMPLES = 1801


def frame(q):
    return np.stack((-np.sin(q), -np.cos(q)), axis=-1), np.stack((np.cos(q), -np.sin(q)), axis=-1)


def pads(base, x, config):
    from core.morphology.mechanism.linkage.design import decode

    finger = decode(x, config.link_length_resolution_mm)
    report = base.report(base.evaluate(x, config))
    return (finger.proximal.start_mm, finger.proximal.end_mm), float(report["distal_support_length_mm"])


def regrasp(q, c0, cx, q0: float, rho: float):
    """dq and ds for the cylinder that touches H0's faces at flexion q0 (None: no touch within range)."""
    i = int(np.argmin(np.abs(q - q0)))
    n0, t0 = frame(q[i])
    if abs(n0[0]) < 1e-6:
        return None
    x_c = (rho + n0[1] * rho + n0 @ c0[i]) / n0[0]
    centre = np.array([x_c, -rho])
    s0 = t0 @ (centre - rho * n0 - c0[i])
    n, t = frame(q)
    residual = np.einsum("ij,ij->i", n, centre - cx) - rho
    sign = np.sign(residual)
    crossings = np.nonzero(np.diff(sign))[0]
    if len(crossings) == 0:
        return None
    k = crossings[np.argmin(np.abs(q[crossings] - q0))]
    w = residual[k] / (residual[k] - residual[k + 1])
    q_star = q[k] + w * (q[k + 1] - q[k])
    c_star = cx[k] + w * (cx[k + 1] - cx[k])
    n_s, t_s = frame(np.array(q_star))
    s_x = t_s @ (centre - rho * n_s - c_star)
    return math.degrees(q_star - q[i]), float(s_x - s0), x_c, float(s0), float(s_x)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--flexion", type=float, default=90.0)
    parser.add_argument("--to", type=float, default=5.0)
    args = parser.parse_args(argv)
    config = read_config_file(e15.REPOSITORY_ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    spec = config["architectures"]["cfl"]
    designs = e15.study1.study1_designs("cfl", spec, args.flexion, config)
    h0, goal = designs[3.0], designs[args.to]
    base = mechanism(h0.mechanism)
    import csv
    with (e15.STUDY2 / "front.csv").open(encoding="utf-8", newline="") as stream:
        rows = [r for r in csv.DictReader(stream) if r["architecture"] == "cfl"
                and float(r["required_flexion_deg"]) == args.flexion and float(r["to_mm"]) == args.to]
    x_r = tuple(float(v) for v in max(rows, key=lambda r: float(r["c_pad_mean"]))["x"].split(";"))
    folder = e15.study1.condition_folder("cfl", spec, args.flexion, args.to, float(config["native_thickness_mm"]))
    x_u = e15.balanced_x(sorted(folder.glob("portfolio_*"))[0])

    ref = paths(base, h0.x, h0.config, samples=SAMPLES)
    q, c0 = np.asarray(ref["q"]), np.asarray(ref["contact"])
    n, t = frame(q)
    pad0 = pads(base, h0.x, h0.config)
    print(f"CFL, q_req {args.flexion:g} deg, 3 -> {args.to:g} mm. H0 pads: proximal {pad0[0][0]:.1f}..{pad0[0][1]:.1f} mm, "
          f"distal {pad0[1]:.1f} mm")
    hands = {"H0 (self-check)": (c0, pad0)}
    for name, x in (("H_R", x_r), ("H_U", x_u)):
        cx = np.asarray(paths(base, x, goal.config, samples=SAMPLES)["contact"])
        hands[name] = (cx, pads(base, x, goal.config))
    print(f"  {'rho':>4} {'q0':>4} | " + " | ".join(f"{name + ': n.dc  dq  ds':>28}" for name in hands))
    summary = {name: [] for name in hands}
    for rho in (10.0, 15.0, 20.0, 25.0, 30.0):
        for q0 in np.arange(20.0, args.flexion + 0.1, 10.0):
            i = int(np.argmin(np.abs(q - math.radians(q0))))
            cells, valid = [], True
            for name, (cx, (prox, l2)) in hands.items():
                got = regrasp(q, c0, cx, math.radians(q0), rho)
                if got is None:
                    cells.append(f"{'no touch':>28}"); valid = False; continue
                dq, ds, x_c, s0, s_x = got
                on = (max(prox[0], pad0[0][0]) <= x_c <= min(prox[1], pad0[0][1]) and -pad0[1] <= s0 <= 0 and -l2 <= s_x <= 0)
                dn = float(n[i] @ (cx[i] - c0[i]))
                if not on:
                    cells.append(f"{'off pad':>28}"); valid = False; continue
                summary[name].append((dn, dq, ds))
                cells.append(f"{dn:+6.2f} mm {dq:+6.2f} deg {ds:+6.2f} mm")
            if valid or True:
                print(f"  {rho:4.0f} {q0:4.0f} | " + " | ".join(cells))
    print("Over the two-pad grasps found on all three fingers' pads:")
    for name, vals in summary.items():
        if vals:
            a = np.array(vals)
            print(f"  {name}: {len(a)} grasps; |n.dc| max {np.abs(a[:, 0]).max():.2f} mm, median {np.median(np.abs(a[:, 0])):.2f}; "
                  f"|dq| max {np.abs(a[:, 1]).max():.2f} deg, median {np.median(np.abs(a[:, 1])):.2f}; "
                  f"|ds| max {np.abs(a[:, 2]).max():.2f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
