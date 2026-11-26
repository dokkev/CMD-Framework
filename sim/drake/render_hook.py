"""Experiment 38's hook-and-pull on hand O (hook_pull.py): one placement re-simulated, three frames (the settled hook,
half the limit load, the limit), drawn as render3d's power-grasp frames (index, middle and ring hooking the bar, palm
up, HOLD_ISO_CAMERA), the bar made a drawer handle: two standoff posts from its ends with small feet, which move with it; the load arrow and, at the limit, the bar's start outlined. render3d, preload_match and pad_shape are imported
read-only.

    uv run python render_hook.py [--rho 6 --q0 100 --direction -z]  ->  logs/render_hook{0,1,2}.png, _strip.png
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import pad_shape as PS
import preload_match as P
import render3d as R
import two_pad_drake as T

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
PLATE = [0.80, 0.78, 0.74, 1.0]
HOOK_CAMERA = {"target_mm": (38.0, -4.0, -16.0), "dist": 0.25, "az_deg": -50.0, "el_deg": -30.0, "fov_deg": 33.0,
               "up": [0.0, 0.0, -1.0]}
"""Along the handle from the fingertips' side, below the palm (the hand palm up, as HOLD_ISO_CAMERA): the bar spans
the three hooked fingers with its posts at both ends, so the frames read as a drawer handle pulled out of the hook
rather than as the power grasp (from HOLD_ISO_CAMERA the bar end-on looked like the held cylinder)."""


def frames(rho, q0, direction):
    pl = next(p for p in json.loads((LOGS / "hook_places.json").read_text())["places"]
              if p["rho_mm"] == rho and p["q0_deg"] == q0)
    d = np.array(T.DIRECTIONS[direction])

    def settled():
        s, m_, gate, pre, c = P.pull_settle("O", 1.3, rho, pl, q0 + 10.0)
        assert gate["pass"], gate
        return s

    def snap(s, load):
        m_ = s.measure()
        return {"distal": [0.0, 0.0, float(m_["q"])], "centre": (1e3 * m_["c"][[0, 2]]).tolist(),
                "spin": float(s.plant.GetJointByName("spin").get_angle(s.pctx)), "load_n": (load * d).tolist()}
    s = settled()
    out = [snap(s, 0.0)]
    lim, _ = PS.ramp(s, s.measure(), direction)
    f_lim = lim["f_limit_n"]
    s2 = settled()
    for i in range(int(0.5 * f_lim / T.FINGERS / (T.RATE_N_S * T.DT_ENV))):
        s2.ctrl.load = T.RATE_N_S * T.DT_ENV * (i + 1) * d
        s2.step()
    out.append(snap(s2, 0.5 * f_lim))
    out.append(snap(s, f_lim))
    return out, lim


def handle(sc, c, rho, span, stand=16.0):
    """Two standoff posts from the bar's ends, just outside the fingers' span, down (-z) to small feet: a drawer
    handle (the drawer front itself is left out so that the hook stays visible), at the bar's centre c (x, z) mm."""
    y_lo, y_hi = span
    for y in (y_lo - 2.5, y_hi + 2.5):
        sc.extrude([np.array([c[0] - 2.5, c[1] - stand]), np.array([c[0] + 2.5, c[1] - stand]),
                    np.array([c[0] + 2.5, c[1]]), np.array([c[0] - 2.5, c[1]])], y - 2.5, y + 2.5, R.MID)
        sc.extrude([np.array([c[0] - 7.0, c[1] - stand - 3.0]), np.array([c[0] + 7.0, c[1] - stand - 3.0]),
                    np.array([c[0] + 7.0, c[1] - stand]), np.array([c[0] - 7.0, c[1] - stand])], y - 5.0, y + 5.0,
                   PLATE)


def load_arrow(sc, c, rho, load, y):
    """The load on the bar, drawn from its surface between two fingers (y, mm) so that the view along the handle
    shows it (render3d.build draws it beyond the fingers' span, hidden by a post in this view)."""
    load = np.asarray(load, float)
    if np.linalg.norm(load) == 0:
        return
    dirv = load / np.linalg.norm(load)
    tail, head = c + dirv * (rho + 0.5), c + dirv * (rho + 14.0)
    sc.bar(tail, head, 1.5, y - 2.5, y + 2.5, R.LOAD)  # thick across the bar: the view is along it
    nrm = np.array([-dirv[1], dirv[0]])
    sc.extrude([head + 4.5 * nrm, head - 4.5 * nrm, head + dirv * 7.5], y - 2.5, y + 2.5, R.LOAD, smooth_deg=0.0)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--rho", type=float, default=6.0)
    p.add_argument("--q0", type=float, default=100.0)
    p.add_argument("--direction", default="-z")
    p.add_argument("--camera", default=None, help="JSON overriding HOOK_CAMERA's keys")
    p.add_argument("--reuse", action="store_true", help="draw the frames saved in logs/hook_render_frames.json")
    p.add_argument("--name", default="hook", help="output stem: logs/render_<name>{0,1,2}.png, _strip.png")
    a = p.parse_args()
    if a.reuse:
        saved = json.loads((LOGS / "hook_render_frames.json").read_text())
        a.rho, a.q0, a.direction, lim, frs = (saved[k] for k in ("rho", "q0", "direction", "limit", "frames"))
    else:
        frs, lim = frames(a.rho, a.q0, a.direction)
        (LOGS / "hook_render_frames.json").write_text(json.dumps({"rho": a.rho, "q0": a.q0, "direction": a.direction,
                                                                  "limit": lim, "frames": frs}, indent=1))
    camera = {**HOOK_CAMERA, **(json.loads(a.camera) if a.camera else {})}
    print(f"limit {lim}")
    span = R.task_span()
    snapd = {"hold": {"rho_mm": a.rho}, "rho_mm": a.rho}
    paths = []
    for i, st in enumerate(frs):
        ghost = frs[1]["centre"] if i == 2 else None
        sc = R.build("hold", {**st, "load_n": [0.0, 0.0]}, snapd, LOGS / "render3d" / f"{a.name}{i}", ghost=ghost,
                     finger=False, span=span)
        handle(sc, np.array(st["centre"]), a.rho, span)
        load_arrow(sc, np.array(st["centre"]), a.rho, st["load_n"], 0.5 * (span[0] + span[1]))
        paths.append(R.render_on_hand(sc, LOGS / f"render_{a.name}{i}.png", st["distal"][2], camera=camera))
    R.strip(paths, LOGS / f"render_{a.name}_strip.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
