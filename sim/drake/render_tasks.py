"""Frames of the three tasks in Fig. 4(a), from Drake states of hand O (radius 15 mm, the q0 = 100 deg placement,
pad friction 1.3), three per task in order: grasp pull-out (placed with the finger open; held after the release; the
-z load ramp after the grasp gives way, the cylinder drawn out), grasp acquisition (open; closing; drawn in), and the
sprung dial (open; turning; turned). Each frame keeps the distal pose, the cylinder's centre and spin, and the load
on the cylinder.
Also experiment 28z's fingertip disk (tipdial(): O, touch angle 25 deg, 10 mm toward the MCP, 12 mm to the side, mu
1.3) at the touch angle, mid-sweep and the end.
Writes logs/task_snapshots.json for render3d.py.

    uv run python render_tasks.py [--tipdial: only the disk's frames, added to the existing file]
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


def setup(g=None, rho=None):
    g, rho = g or G, rho or RHO
    geom = T.finger_geometry(HAND)
    qs, th = T.servo_table(geom)
    q_cmd = math.radians(g["q0_deg"] + 5.0)
    theta_cmd = float(np.interp(q_cmd, qs, th))
    ratio = float(np.interp(q_cmd, qs, np.gradient(th, qs)))
    face = (geom["pads"]["mount_offset_mm"] + geom["pads"]["thickness_mm"]) * 1e-3
    x0 = np.array([g["x_c_mm"] * 1e-3, -(face + rho * 1e-3)])
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


DIAL_RHO, DIAL_Q0 = 10.0, 70.0  # experiment 26: the 30-degree dial, one of its placements


def dial():
    """Experiment 26's dial: radius 10 mm, axis 3 mm off the proximal face, 300 N/m away from it, closing to
    min(q0 + 5 + 45 deg, 120 deg) over 3 s and held 1 s."""
    g = [x for x in json.loads((T.OUT / "grasps_r10.json").read_text())["grasps"] if x["q0_deg"] == DIAL_Q0][0]
    geom, qs, th, q_cmd, theta_cmd, k, d, x0 = setup(g, DIAL_RHO)
    T.DIAL_Z_STIFFNESS, T.DIAL_BRAKE_NM = 300.0, 0.0
    s = T.Stage(HAND, MU, DIAL_RHO, "point", 1e-3, "xz", k, d)
    centre = np.array([x0[0], x0[1] - 3e-3])
    s.plant.GetJointByName("slide_x").set_translation(s.pctx, centre[0])
    s.plant.GetJointByName("slide_z").set_translation(s.pctx, centre[1])
    s.ctrl.hold_at = np.array([centre[0], centre[1], 0.0])
    s.sim.Initialize()
    frames = [frame(s)]
    q1 = min(q_cmd + math.radians(45.0), math.radians(120.0))
    for i in range(120):
        q = q1 * min(1.0, (i + 1) / 90)
        s.ctrl.theta_cmd = float(np.interp(q, qs, th))
        s.step()
        if i == 79:
            frames.append(frame(s))
    frames.append(frame(s))
    T.DIAL_Z_STIFFNESS = 0.0
    return {"frames": frames, "axis": [1e3 * centre[0], 1e3 * centre[1]], "rho_mm": DIAL_RHO}


TIPDIAL = {"q0_deg": 25.0, "x_back_mm": 10.0, "y_mm": 12.0}
"""Experiment 28z's representative placement on O (logs/tipdial_z.json): touch angle 25 deg, the disk's centre 10 mm
toward the MCP from the fingertip, its axle 12 mm to the side; pad friction MU, run_e28.sh's settings (TIPDIAL_OPTS)."""
TIPDIAL_OPTS = {"tip_mode": "z", "tip_thick_mm": 4.0, "tip_kr": 0.01, "tip_dr": 1.0e-4, "dial_k": 60.0,
                "sweep_deg": 40.0, "approach_s": 1.0, "sweep_s": 3.0, "hold_s": 1.0}


def tipdial(times, placement=None, opts=None):
    """Experiment 28z's fingertip disk on O at ``placement``, frames at ``times`` [s from the start: 0 -> q_t over
    approach_s, q_t -> q_t + sweep over sweep_s, held hold_s]. The run is two_pad_drake.tipdial's (same settings and
    command), stepped here so frames can be kept; it is checked against two_pad_drake.tipdial's own rotation. Each frame
    keeps the distal pose, the disk's centre (x, y, z mm; y from the finger's plane), its spin about z, its rotation
    (deg, as two_pad_drake reports it) and the distal force [N]."""
    placement, opts = {**TIPDIAL, **(placement or {})}, {**TIPDIAL_OPTS, **(opts or {})}
    g = [x for x in json.loads((T.OUT / "tipdial_z.json").read_text())["grasps"]
         if all(abs(x[k] - v) < 1e-9 for k, v in placement.items())][0]
    rho = g["rho_mm"]
    saved = {k: getattr(T, k) for k in ("DIAL_Z_STIFFNESS", "TIP_MODE", "TIP_AXIS_DEG", "TIP_Y_MM", "TIP_THICK_MM",
                                         "TIP_KR", "TIP_DR", "SPIN_DAMPING", "DIAL_BRAKE_NM")}
    T.DIAL_Z_STIFFNESS, T.TIP_MODE, T.TIP_THICK_MM = opts["dial_k"], "z", opts["tip_thick_mm"]
    T.TIP_KR, T.TIP_DR, T.TIP_AXIS_DEG, T.TIP_Y_MM = opts["tip_kr"], opts["tip_dr"], 0.0, g["y_mm"]
    T.SPIN_DAMPING, T.DIAL_BRAKE_NM = 0.0, 0.0
    try:
        geom = T.finger_geometry(HAND)
        qs, th = T.servo_table(geom)
        q_start = math.radians(g["q0_deg"])  # O's touch angle, delta 0
        ratio = float(np.interp(q_start, qs, np.gradient(th, qs)))
        k, d = T.K_Q / ratio ** 2, T.D_Q / ratio ** 2
        reference = T.tipdial(HAND, MU, rho, "point", 1e-3, k, d, g, q_start, qs, th, opts)

        s = T.Stage(HAND, MU, rho, "point", 1e-3, "dial", k, d)
        face = geom["pads"]["mount_offset_mm"] + geom["pads"]["thickness_mm"]
        c = np.array([g["x_c_mm"], g["z_c_mm"] - face]) * 1e-3
        s.plant.GetJointByName("slide_x").set_translation(s.pctx, c[0])
        s.plant.GetJointByName("slide_z").set_translation(s.pctx, c[1])
        s.ctrl.hold_at = np.array([c[0], c[1], 0.0])
        s.sim.Initialize()
        spin = s.plant.GetJointByName("spin")
        q1 = min(q_start + math.radians(opts["sweep_deg"]), math.radians(120.0))
        n_app, n_sweep, n_hold = (int(round(opts[x] / T.DT_ENV)) for x in ("approach_s", "sweep_s", "hold_s"))
        want = {int(round(t / T.DT_ENV)): t for t in times}

        def keep(t):
            Xd = s.plant.EvalBodyPoseInWorld(s.pctx, s.bodies["distal"])
            R = Xd.rotation().matrix()
            pos = 1e3 * s.plant.EvalBodyPoseInWorld(s.pctx, s.bodies["roller"]).translation()
            force, _ = s.contacts()
            return {"t_s": t, "distal": [1e3 * Xd.translation()[0], 1e3 * Xd.translation()[2],
                                         math.atan2(R[0, 2], R[0, 0])],
                    "centre": pos.tolist(), "spin": float(spin.get_angle(s.pctx)),
                    "rotation_deg": -math.degrees(spin.get_angle(s.pctx)),
                    "distal_force_n": float(np.linalg.norm(force[1]))}

        frames = {}
        for i in range(n_app + n_sweep + n_hold + 1):
            if i in want:
                frames[want[i]] = keep(want[i])
            if i == n_app + n_sweep + n_hold:
                break
            if i < n_app:
                q = q_start * (i + 1) / n_app
            else:
                q = q_start + (q1 - q_start) * min(1.0, (i + 1 - n_app) / n_sweep)
            s.ctrl.theta_cmd = float(np.interp(q, qs, th))
            s.step()
        end = -math.degrees(spin.get_angle(s.pctx))
        assert abs(end - reference["rotation_deg"]) < 1e-6, (end, reference["rotation_deg"])
    finally:
        for k_, v in saved.items():
            setattr(T, k_, v)
    return {"frames": [frames[t] for t in times], "times_s": list(times), "rho_mm": rho,
            "thick_mm": opts["tip_thick_mm"], "placement": g, "rotation_end_deg": end,
            "touch_q_deg": reference["touch_q_deg"], "distal_force_max_n": reference["distal_force_max_n"]}


TIPDIAL_STRIP_S = (1.0, 2.5, 5.0)
"""render_tipdial{0,1,2}.png: at the touch angle (before the pad meets the disk), mid-sweep, and the end of the hold."""


def main():
    import sys
    if sys.argv[1:] == ["--tipdial"]:  # only the fingertip disk's frames, added to the existing snapshot file
        path = T.OUT / "task_snapshots.json"
        out = json.loads(path.read_text())
        out["tipdial"] = tipdial(TIPDIAL_STRIP_S)
        path.write_text(json.dumps(out, indent=1))
        print(path)
        return
    geom = T.finger_geometry(HAND)
    out = {"hand": HAND, "mu": MU, "rho_mm": RHO, "q0_deg": G["q0_deg"], "pads": geom["pads"],
           "b_mm": geom["b"].tolist(), "hold": hold(), "acquire": acquire(), "dial": dial(),
           "tipdial": tipdial(TIPDIAL_STRIP_S)}
    path = T.OUT / "task_snapshots.json"
    path.write_text(json.dumps(out, indent=1))
    print(path)


if __name__ == "__main__":
    main()
