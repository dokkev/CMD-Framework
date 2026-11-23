"""Snapshots of the three tasks in Fig. 5, from Drake states of hand O (radius 15 mm, the q0 = 100 deg placement,
pad friction 1.3): holding against a load (after the release, with the load direction), grasp acquisition (start,
faint, and end), and the sprung dial (end). Pads, phalanx lines and the object are drawn from the simulated poses;
the linkage is not drawn. Writes logs/task_snapshots.json for paper/figures/fig_study3.py.

    uv run python render_tasks.py
"""
import json
import math

import numpy as np

import two_pad_drake as T

G = [g for g in json.loads(T.GRASPS.read_text())["grasps"] if g["rho_mm"] == 15.0][12]  # q0 = 100 deg
HAND, MU, RHO = "O", 1.3, 15.0


def pose(s):
    """Distal body pose (x, z, angle about +y) and the cylinder's centre and spin, in mm and rad."""
    Xd = s.plant.EvalBodyPoseInWorld(s.pctx, s.bodies["distal"])
    R = Xd.rotation().matrix()
    c = s.plant.EvalBodyPoseInWorld(s.pctx, s.bodies["roller"]).translation()
    spin = s.plant.GetJointByName("spin").get_angle(s.pctx)
    return {"distal": [1e3 * Xd.translation()[0], 1e3 * Xd.translation()[2], math.atan2(R[0, 2], R[0, 0])],
            "centre": [1e3 * c[0], 1e3 * c[2]], "spin": float(spin)}


def setup():
    geom = T.finger_geometry(HAND)
    qs, th = T.servo_table(geom)
    q_cmd = math.radians(G["q0_deg"] + 5.0)
    theta_cmd = float(np.interp(q_cmd, qs, th))
    ratio = float(np.interp(q_cmd, qs, np.gradient(th, qs)))
    face = (geom["pads"]["mount_offset_mm"] + geom["pads"]["thickness_mm"]) * 1e-3
    x0 = np.array([G["x_c_mm"] * 1e-3, -(face + RHO * 1e-3)])
    return geom, qs, th, q_cmd, theta_cmd, T.K_Q / ratio ** 2, T.D_Q / ratio ** 2, x0


def hold():
    geom, qs, th, q_cmd, theta_cmd, k, d, x0 = setup()
    s1 = T.Stage(HAND, MU, RHO, "point", 1e-3, True, k, d)
    s1.plant.GetJointByName("slide_x").set_translation(s1.pctx, x0[0])
    s1.plant.GetJointByName("slide_z").set_translation(s1.pctx, x0[1])
    s1.ctrl.hold_at = np.array([x0[0], x0[1], 0.0])
    s1.sim.Initialize()
    for i in range(70):
        s1.ctrl.theta_cmd = theta_cmd * min(1.0, (i + 1) / 40)
        s1.step()
    state = s1.plant.GetPositionsAndVelocities(s1.pctx).copy()
    s = T.Stage(HAND, MU, RHO, "point", 1e-3, False, k, d)
    s.plant.SetPositionsAndVelocities(s.pctx, state)
    s.ctrl.theta_cmd = theta_cmd
    s.sim.Initialize()
    for _ in range(120):
        s.step()
    return {"end": pose(s), "load": [1.0, 0.0]}


def acquire():
    geom, qs, th, q_cmd, theta_cmd, k, d, x0 = setup()
    T.SPIN_DAMPING = 2.0
    s = T.Stage(HAND, MU, RHO, "point", 1e-3, False, k, d)
    start = np.array([x0[0] + 6e-3, x0[1]])
    s.plant.GetJointByName("slide_x").set_translation(s.pctx, start[0])
    s.plant.GetJointByName("slide_z").set_translation(s.pctx, start[1])
    s.ctrl.load = np.array([1.5, 1.5]) / T.FINGERS
    s.sim.Initialize()
    first = pose(s)
    for i in range(120):
        s.ctrl.theta_cmd = theta_cmd * min(1.0, (i + 1) / 60)
        s.step()
    T.SPIN_DAMPING = 0.0
    return {"start": first, "end": pose(s), "load": [1.0, 1.0]}


def dial():
    geom, qs, th, q_cmd, theta_cmd, k, d, x0 = setup()
    T.DIAL_Z_STIFFNESS, T.DIAL_BRAKE_NM = 300.0, 0.0
    s = T.Stage(HAND, MU, RHO, "point", 1e-3, "xz", k, d)
    centre = np.array([x0[0], x0[1] - 1e-3])
    s.plant.GetJointByName("slide_x").set_translation(s.pctx, centre[0])
    s.plant.GetJointByName("slide_z").set_translation(s.pctx, centre[1])
    s.ctrl.hold_at = np.array([centre[0], centre[1], 0.0])
    s.sim.Initialize()
    q1 = min(q_cmd + math.radians(15.0), math.radians(120.0))
    for i in range(120):
        q = q1 * min(1.0, (i + 1) / 90)
        s.ctrl.theta_cmd = float(np.interp(q, qs, th))
        s.step()
    T.DIAL_Z_STIFFNESS = 0.0
    return {"end": pose(s), "axis": [1e3 * centre[0], 1e3 * centre[1]]}


def main():
    geom = T.finger_geometry(HAND)
    out = {"hand": HAND, "mu": MU, "rho_mm": RHO, "q0_deg": G["q0_deg"], "pads": geom["pads"],
           "b_mm": geom["b"].tolist(), "hold": hold(), "acquire": acquire(), "dial": dial()}
    path = T.OUT / "task_snapshots.json"
    path.write_text(json.dumps(out, indent=1))
    print(path)


if __name__ == "__main__":
    main()
