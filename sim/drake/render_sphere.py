"""Experiment 42's sphere acquisition on hand O (sphere_grasp.py; one draw re-simulated at mu 1.3): three frames (the
finger open with the sphere on the proximal face, mid-closing, the end of the hold), drawn as render3d's power-grasp
frames (index, middle and ring, palm up, HOLD_ISO_CAMERA). The sphere is painted in wedges about y so its turn shows;
its start is outlined faintly and the load (toward the face and the tip) is the red arrow. render3d and render_ball
are imported read-only.

    uv run python render_sphere.py [--draw 5]  ->  logs/render_sphere{0,1,2}.png, logs/render_sphere_strip.png
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import render3d as R
import render_ball as RB
import sphere_grasp as S
import two_pad_drake as T

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"


def frames(pl, mu=1.3):
    S.use_sphere()
    geom = T.finger_geometry("O")
    qs, th = T.servo_table(geom)
    q_cmd = math.radians(pl["q0_deg"])
    theta_cmd = float(np.interp(q_cmd, qs, th))
    ratio = float(np.interp(q_cmd, qs, np.gradient(th, qs)))
    T.SPIN_DAMPING = S.OPTS["spin_damping"]
    s = T.Stage("O", mu, pl["rho_mm"], "point", 1e-3, False, T.K_Q / ratio ** 2, T.D_Q / ratio ** 2)
    face = (geom["pads"]["mount_offset_mm"] + geom["pads"]["thickness_mm"]) * 1e-3
    start = np.array([pl["x_c_mm"] * 1e-3 + pl["offset_mm"] * 1e-3, -(face + pl["rho_mm"] * 1e-3)])
    s.plant.GetJointByName("slide_x").set_translation(s.pctx, start[0])
    s.plant.GetJointByName("slide_z").set_translation(s.pctx, start[1])
    load = np.array([S.OPTS["tip_load_n"], S.OPTS["face_load_n"]]) / T.FINGERS
    s.ctrl.load = load
    s.sim.Initialize()

    def snap():
        m_ = s.measure()
        return {"distal": [0.0, 0.0, float(m_["q"])], "centre": (1e3 * m_["c"][[0, 2]]).tolist(),
                "spin": float(s.plant.GetJointByName("spin").get_angle(s.pctx)), "load_n": [3.0, -3.0]}
    out = [snap()]
    n_close, n_hold = (int(round(S.OPTS[k] / T.DT_ENV)) for k in ("close_s", "hold_s"))
    for i in range(n_close + n_hold):
        s.ctrl.theta_cmd = theta_cmd * min(1.0, (i + 1) / n_close)
        s.step()
        if i == n_close // 2:
            out.append(snap())
    out.append(snap())
    return out


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--draw", type=int, default=5)
    a = p.parse_args()
    pl = next(x for x in json.loads((LOGS / "sphere_e42.json").read_text())["draws"] if x["draw"] == a.draw)
    frs = frames(pl)
    span = R.task_span()
    paths = []
    for i, st in enumerate(frs):
        sc = R.Scene(LOGS / "render3d" / f"sphere{i}")
        c = st["centre"]
        if i > 0:
            R.dashed_ring(sc, np.array(frs[0]["centre"]), pl["rho_mm"], -pl["rho_mm"] - 0.6, R.GHOST)
        RB.sphere(sc, (c[0], 0.0, c[1]), pl["rho_mm"], -math.degrees(st["spin"]))
        if i == 0:  # the load: toward the tip (+x) and the face (+z, dorsal), drawn in front of the sphere
            cc = np.array(c)
            R.arrow(sc, cc + np.array([pl["rho_mm"] + 2.0, 0.0]), cc + np.array([pl["rho_mm"] + 14.0, 0.0]),
                    -pl["rho_mm"] - 2.0, R.LOAD)
        paths.append(R.render_on_hand(sc, LOGS / f"render_sphere{i}.png", st["distal"][2], camera=R.HOLD_ISO_CAMERA))
        print(f"frame {i}: q {math.degrees(st['distal'][2]):.1f} deg, centre {c[0]:.1f}, {c[1]:.1f} mm")
    R.strip(paths, LOGS / "render_sphere_strip.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
