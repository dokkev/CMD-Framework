"""Frames of the three tasks in Fig. 4(a), from Drake states of hand O (radius 15 mm, the q0 = 100 deg placement,
pad friction 1.3), three per task in order: grasp pull-out (placed with the finger open; held after the release; the
-z load ramp after the grasp gives way, the cylinder drawn out), grasp acquisition (open; closing; drawn in), and the
sprung dial (open; turning; turned). Each frame keeps the distal pose, the cylinder's centre and spin, and the load
on the cylinder.
Writes logs/task_snapshots.json for render3d.py.

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


def frame(s, load=(0.0, 0.0)):
    return {**pose(s), "load_n": [float(v) * T.FINGERS for v in load]}


def hold():
    """Placed with the finger open, closed and released (the Gate 0 state), then the -z load ramp until the grasp
    gives way (slip, loss or collapse, as two_pad_drake), then on until the cylinder has moved 14 mm (or 40 steps)."""
    geom, qs, th, q_cmd, theta_cmd, k, d, x0 = setup()
    s1 = T.Stage(HAND, MU, RHO, "point", 1e-3, True, k, d)
    s1.plant.GetJointByName("slide_x").set_translation(s1.pctx, x0[0])
    s1.plant.GetJointByName("slide_z").set_translation(s1.pctx, x0[1])
    s1.ctrl.hold_at = np.array([x0[0], x0[1], 0.0])
    s1.sim.Initialize()
    frames = [frame(s1)]
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
    frames.append(frame(s))
    m0 = s.measure()
    c0 = m0["c"].copy()
    direction = np.array(T.DIRECTIONS["-z"])  # pulled away from the palm
    after = None
    for i in range(int(T.MAX_FORCE_N / (T.RATE_N_S * T.DT_ENV)) + 1):
        load = T.RATE_N_S * T.DT_ENV * (i + 1) * direction
        s.ctrl.load = load
        s.step()
        m_ = s.measure()
        gave = (1e3 * m_["slip"].max() > T.SLIP_MM_S or (m_["normal"] < T.LOSS_N).any()
                or 1e3 * np.linalg.norm((m_["c"] - c0)[[0, 2]]) > T.COLLAPSE_MM)
        if gave and after is None:
            after = 40  # keep the load on, so the frame shows the cylinder leaving the grasp
        if after is not None:
            after -= 1
            if after < 0 or 1e3 * np.linalg.norm((m_["c"] - c0)[[0, 2]]) > 14.0:
                break
    frames.append(frame(s, load))
    return {"frames": frames}


def acquire():
    geom, qs, th, q_cmd, theta_cmd, k, d, x0 = setup()
    T.SPIN_DAMPING = 2.0
    s = T.Stage(HAND, MU, RHO, "point", 1e-3, False, k, d)
    start = np.array([x0[0] + 6e-3, x0[1]])
    s.plant.GetJointByName("slide_x").set_translation(s.pctx, start[0])
    s.plant.GetJointByName("slide_z").set_translation(s.pctx, start[1])
    load = np.array([1.5, 1.5]) / T.FINGERS
    s.ctrl.load = load
    s.sim.Initialize()
    frames = [frame(s, load)]
    for i in range(120):
        s.ctrl.theta_cmd = theta_cmd * min(1.0, (i + 1) / 60)
        s.step()
        if i == 39:
            frames.append(frame(s, load))
    frames.append(frame(s, load))
    T.SPIN_DAMPING = 0.0
    return {"frames": frames}


def dial():
    geom, qs, th, q_cmd, theta_cmd, k, d, x0 = setup()
    T.DIAL_Z_STIFFNESS, T.DIAL_BRAKE_NM = 300.0, 0.0
    s = T.Stage(HAND, MU, RHO, "point", 1e-3, "xz", k, d)
    centre = np.array([x0[0], x0[1] - 1e-3])
    s.plant.GetJointByName("slide_x").set_translation(s.pctx, centre[0])
    s.plant.GetJointByName("slide_z").set_translation(s.pctx, centre[1])
    s.ctrl.hold_at = np.array([centre[0], centre[1], 0.0])
    s.sim.Initialize()
    frames = [frame(s)]
    q1 = min(q_cmd + math.radians(15.0), math.radians(120.0))
    for i in range(120):
        q = q1 * min(1.0, (i + 1) / 90)
        s.ctrl.theta_cmd = float(np.interp(q, qs, th))
        s.step()
        if i == 74:
            frames.append(frame(s))
    frames.append(frame(s))
    T.DIAL_Z_STIFFNESS = 0.0
    return {"frames": frames, "axis": [1e3 * centre[0], 1e3 * centre[1]]}


def main():
    geom = T.finger_geometry(HAND)
    out = {"hand": HAND, "mu": MU, "rho_mm": RHO, "q0_deg": G["q0_deg"], "pads": geom["pads"],
           "b_mm": geom["b"].tolist(), "hold": hold(), "acquire": acquire(), "dial": dial()}
    path = T.OUT / "task_snapshots.json"
    path.write_text(json.dumps(out, indent=1))
    print(path)


if __name__ == "__main__":
    main()
