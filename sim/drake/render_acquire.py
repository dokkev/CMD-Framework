"""Experiment 23's grasp acquisition on hand O (pad friction 1.3), drawn as the Fig. 4 strips: index, middle and ring
on the hand's URDF geometry, palm up, from render3d.HOLD_ISO_CAMERA (as the power-grasp hold). Frames at --times s
after the open finger starts closing (closing over 2 s, then held): open with the cylinder toward the fingertip under
its load, mid-closing, drawn in (the start outlined). Same placement and settings as render_task_times.acquire.

    uv run python render_acquire.py [--times 0 1 3]  ->  logs/render_acquire_iso{0,1,2}.png, _strip.png
"""
import argparse
import math

import render3d as R3
import render_task_times as RTT
import render_tasks as RT
import two_pad_drake as T


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--times", type=float, nargs="+", default=[0.0, 1.0, 3.0])
    a = p.parse_args()
    fr = RTT.acquire(a.times)
    frames = [fr[t] for t in a.times]
    snap = {"acquire": {"rho_mm": RT.RHO}, "rho_mm": RT.RHO}
    paths = []
    for i, st in enumerate(frames):
        ghost = frames[0]["centre"] if i > 0 else None
        sc = R3.build("acquire", st, snap, T.OUT / "render3d" / f"acquire_iso{i}", ghost=ghost,
                      turn_sign=math.copysign(1.0, frames[-1]["spin"] or 1.0), finger=False, span=R3.task_span())
        paths.append(R3.render_on_hand(sc, T.OUT / f"render_acquire_iso{i}.png", st["distal"][2],
                                       camera=R3.HOLD_ISO_CAMERA))
    R3.strip(paths, T.OUT / "render_acquire_iso_strip.png")
    for t, st in zip(a.times, frames):
        print(f"t {t:g} s: q {math.degrees(st['distal'][2]):.1f} deg, centre {st['centre']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
