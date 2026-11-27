"""Grasp acquisition drawn from the fingertip (render only; the measured experiment 23 starts 3-6 mm from the grasp,
over the PIP when the finger is open): hand O, the cylinder started on the open finger's distal pad (--start_mm from the
proximal-face frame's origin; 62 mm, 6.6 mm past the pad's proximal end), the finger closing as experiment 23 (O's setpoint,
load toward the tip and the face). The thumb takes no part and stays at its neutral pose (--wrap swings it in as
a drawing pose only). Experiment 23b (the same start in the experiment) is a
TODO in report/study3.

    uv run python render_acquire_tip.py [--times 0 1 3] [--start_mm 62]  ->  logs/render_acquire_tip{0,1,2}.png, _strip.png
"""
import argparse
import math

import numpy as np

import render3d as R3
import render_task_times as RTT
import render_tasks as RT
import two_pad_drake as T

THUMB_OPEN = {"thumb_ro": 0.0, "thumb_raa": 0.0, "thumb_mcp": 0.0, "thumb_ip": 0.0}
THUMB_WRAP = {"thumb_ro": -70.0, "thumb_raa": 20.0, "thumb_mcp": -50.0, "thumb_ip": -40.0}  # over the palm, curled toward the object


def acquire_from(times, start_mm):
    geom, qs, th, q_cmd, theta_cmd, k, d, x0 = RT.setup()
    T.SPIN_DAMPING = 2.0
    s = T.Stage(RT.HAND, RT.MU, RT.RHO, "point", 1e-3, False, k, d)
    s.plant.GetJointByName("slide_x").set_translation(s.pctx, start_mm * 1e-3)
    s.plant.GetJointByName("slide_z").set_translation(s.pctx, x0[1])
    load = np.array([1.5, 1.5]) / T.FINGERS
    s.ctrl.load = load
    s.sim.Initialize()
    want = {RTT.steps(t): t for t in times}
    frames = {}
    for i in range(max(want) + 1):
        if i in want:
            frames[want[i]] = RT.frame(s, load)
        s.ctrl.theta_cmd = theta_cmd * min(1.0, (i + 1) / 60)
        s.step()
    T.SPIN_DAMPING = 0.0
    return frames


def thumb_at(f):
    return {k: THUMB_OPEN[k] + f * (THUMB_WRAP[k] - THUMB_OPEN[k]) for k in THUMB_OPEN}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--times", type=float, nargs="+", default=[0.0, 1.0, 3.0])
    p.add_argument("--start_mm", type=float, default=62.0,
                   help="on the open finger's distal pad (55.4-90.0 mm). From 66 mm on, the closing fingertip traps the "
                        "cylinder below it (centre ~20 mm under the two-pad grasp) instead of drawing it onto the proximal pad")
    p.add_argument("--wrap", type=float, nargs="+", default=None, help="thumb closure per frame, 0..1")
    a = p.parse_args()
    fr = acquire_from(a.times, a.start_mm)
    frames = [fr[t] for t in a.times]
    wrap = a.wrap or [0.0] * len(frames)  # the thumb takes no part: left at its neutral pose unless --wrap is given
    for t, st in zip(a.times, frames):
        print(f"t {t:g} s: centre {np.round(st['centre'], 2)} mm, q {math.degrees(st['distal'][2]):.1f} deg")
    R3.GONE_MM = 1.0e9  # drawn in ~50 mm from the fingertip: still in the hand, not "gone"
    snap = {"acquire": {"rho_mm": RT.RHO}, "rho_mm": RT.RHO}
    paths = []
    for i, st in enumerate(frames):
        ghost = frames[0]["centre"] if i > 0 else None
        sc = R3.build("acquire", st, snap, T.OUT / "render3d" / f"acquire_tip{i}", ghost=ghost,
                      turn_sign=math.copysign(1.0, frames[-1]["spin"] or 1.0), finger=False, span=R3.task_span())
        paths.append(R3.render_on_hand(sc, T.OUT / f"render_acquire_tip{i}.png", st["distal"][2],
                                       camera=R3.HOLD_ISO_CAMERA, thumb=thumb_at(wrap[i])))
    R3.strip(paths, T.OUT / "render_acquire_tip_strip.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
