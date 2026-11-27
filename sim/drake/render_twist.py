"""Experiment 39's grasp-and-twist on hand O (twist_grasp.py): one placement re-simulated, three frames (the settled
grasp, half the limit torque, the limit), drawn as render3d's power-grasp frames (palm up, HOLD_ISO_CAMERA) with a red
arc arrow on the cylinder's near end for the torque (no linear load arrow); the end face's stripe shows the turn.
render3d, preload_match and twist_grasp are imported read-only.

    uv run python render_twist.py [--rho 15 --q0 95 --sense +spin]  ->  logs/render_twist{0,1,2}.png, _strip.png
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import preload_match as P
import render3d as R
import two_pad_drake as T
import twist_grasp as W

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"


def frames(rho, q0, sense):
    pl = next(g for r, g in W.places() if r == rho and g["q0_deg"] == q0)

    def settled():
        s, m_, gate, pre, c = P.pull_settle("O", 1.3, rho, pl, q0 + 5.0)
        assert gate["pass"], gate
        b = W.TwistStage("O", 1.3, rho, "point", 1.0e-3, False, c["k_theta"], c["d_theta"])
        b.plant.SetPositionsAndVelocities(b.pctx, s.plant.GetPositionsAndVelocities(s.pctx).copy())
        b.ctrl.theta_cmd = s.ctrl.theta_cmd
        b.sim.Initialize()
        return b

    def snap(s):
        m_ = s.measure()
        return {"distal": [0.0, 0.0, float(m_["q"])], "centre": (1e3 * m_["c"][[0, 2]]).tolist(),
                "spin": float(s.plant.GetJointByName("spin").get_angle(s.pctx)), "load_n": [0.0, 0.0]}
    s = settled()
    out = [snap(s)]
    lim, _ = W.twist_ramp(s, sense)
    s2 = settled()
    sgn = 1.0 if sense == "+spin" else -1.0
    for i in range(int(0.5 * lim["tau_limit_nm"] / T.FINGERS / (W.RATE_NM_S * T.DT_ENV))):
        s2.ctrl.torque = sgn * W.RATE_NM_S * T.DT_ENV * (i + 1)
        s2.step()
    out.append(snap(s2))
    out.append(snap(s))
    return out, lim


def torque_arrows(sc, c, rho, y_end, sgn, rgba=None):
    """A bold arc arrow (about 290 deg) just outside the cylinder's near end, and a straight arrow along the cylinder's
    axis (the torque vector, +y for +spin by the right-hand rule; -spin points out of the near end, the view's side), both in the
    load colour, so the torsional load reads at a glance."""
    rgba = rgba or R.LOAD
    r = rho + 5.0
    yy = y_end - 2.5
    a0 = math.radians(120.0)
    a1 = a0 - sgn * math.radians(290.0)
    ts = np.linspace(a0, a1, 48)
    pts = [c + r * np.array([math.cos(t), math.sin(t)]) for t in ts]
    for p, q in zip(pts[:-5], pts[1:-4]):
        sc.bar(p, q, 1.3, yy - 1.3, yy + 1.3, rgba)
    R.arrow(sc, pts[-6], pts[-1], yy, rgba, half_width=1.3, head_len=7.0, head_half=4.2)
    # the axis vector: a square shaft from the end face outward, then a tapered head
    out = sgn  # the torque vector along the spin axis (+y for +spin, right-hand rule); -spin points out of the near end
    shaft, head, w = 26.0, 9.0, 1.6
    sq = lambda h: [c + np.array([dx, dz]) for dx, dz in ((-h, -h), (h, -h), (h, h), (-h, h))]
    y0 = y_end - 1.0
    y1 = y0 - shaft if out < 0 else y0 + shaft
    sc.extrude(sq(w), min(y0, y1), max(y0, y1), rgba, smooth_deg=0.0)
    steps = 28
    for k in range(steps):
        h = 4.2 * (1.0 - k / steps)
        ya = y1 + out * head * k / steps
        yb = y1 + out * head * (k + 1) / steps
        sc.extrude(sq(max(h, 0.3)), min(ya, yb), max(ya, yb), rgba, smooth_deg=0.0)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--rho", type=float, default=15.0)
    p.add_argument("--q0", type=float, default=95.0)
    p.add_argument("--sense", default="+spin")
    a = p.parse_args()
    frs, lim = frames(a.rho, a.q0, a.sense)
    (LOGS / "twist_render_frames.json").write_text(json.dumps({"rho": a.rho, "q0": a.q0, "sense": a.sense,
                                                               "limit": lim, "frames": frs}, indent=1))
    print(f"limit {lim}")
    span = R.task_span()
    snapd = {"hold": {"rho_mm": a.rho}, "rho_mm": a.rho}
    paths = []
    sgn = 1.0 if a.sense == "+spin" else -1.0
    for i, st in enumerate(frs):
        sc = R.build("hold", st, snapd, LOGS / "render3d" / f"twist{i}", finger=False, span=span)
        if i > 0:  # the torque: a bold arc about the cylinder's axis, and the axis vector itself (right-hand rule)
            c = np.array(st["centre"])
            torque_arrows(sc, c, a.rho, span[0], sgn)
        paths.append(R.render_on_hand(sc, LOGS / f"render_twist{i}.png", st["distal"][2], camera=R.HOLD_ISO_CAMERA))
    R.strip(paths, LOGS / "render_twist_strip.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
