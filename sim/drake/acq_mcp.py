"""Grasp acquisition from the distal pad (experimental, docs/experimental/drake_acquisition.md), in the Drake Study 3
replica with an MCP flexion DOF. Two variants of the same task, both open-loop and with O's commands on every hand:

  A  the PIP held at O's command (q0 + 5 deg) under the flexion impedance; the cylinder rests on the distal pad only;
     the MCP (an XM335-like position loop) flexes from -phi0 to 0, so the proximal pad comes round to the cylinder.
  B  the MCP held at 0; the cylinder rests on the distal pad only, the PIP starts at q0 + 5 - sweep and closes to
     q0 + 5, so the distal pad turns about the PIP and hands the cylinder proximally to the proximal pad.

The cylinder (radius rho) rides a planar slide (x, z, spin, the spin with experiment 23's bearing damping) under a
constant world load (gravity-like), set per placement from O's start pose: at angle beta0 from O's distal-pad inward
normal toward the PIP. Its start: on O's distal pad at a distal-frame point a mm toward the tip from where it would
sit in the two-pad corner at O's command; on another hand, the same world point moved back along the load line until
it just touches that hand's distal pad (dropped onto the pad). Planar finger as two_pad_drake.py (imported read-only):
x along the proximal phalanx at MCP 0, z dorsal; the four-bar is a tree closed by a distance constraint, now hung on
the proximal body, which turns about the MCP at the world origin.

    uv run python acq_mcp.py run --variant A --hand O --mu 1.3 --tag acqmcp-cal [--workers 3]
    uv run python acq_mcp.py analyze --variant A --tag acqmcp-A
    uv run python acq_mcp.py frames --variant A
"""
from __future__ import annotations

import argparse
import json
import math
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import two_pad_drake as T

OUT = T.OUT
MM = 1e-3
XM335_K, XM335_D = 11.854795309172589, 0.18558066949205912  # N m/rad, N m s/rad (design: xm335.py, servo.derived)
SETTLE_S = 0.5

# the pre-registered task (fixed on O alone; see the doc). Placements: (rho, q0) from the Study 3 grasp file, a (mm)
DEFAULTS = {
    "A": {"phi0_deg": 30.0, "beta0_deg": 40.0, "load_n": 6.0, "sweep_s": 2.0, "hold_s": 1.0, "spin_damping": 2.0,
          "q_sweep_deg": 0.0},
    "B": {"phi0_deg": 0.0, "beta0_deg": 40.0, "load_n": 1.5, "sweep_s": 2.0, "hold_s": 1.0, "spin_damping": 2.0,
          "q_sweep_deg": 30.0},
}
# B stops at q0 80: from q0 90 on, O's distal pad is pressed more than 1.5 mm after the capture (calibration)
PLACEMENTS = {"A": {"rho": (15.0, 20.0), "q0": (60.0, 70.0, 80.0, 90.0, 100.0), "a": (3.0, 6.0)},
              "B": {"rho": (15.0, 20.0), "q0": (60.0, 65.0, 70.0, 75.0, 80.0), "a": (3.0, 6.0)}}
THRESHOLD = {"capture_deg": 2.0, "x_end_mm": 0.3}  # |D(O)| for a pair (O only)
PEN_GATE_MM = 1.5


def kin(hand: str):
    g = T.finger_geometry(hand)
    qs, th = T.servo_table(g)
    return g, qs, th


def theta_of(qs, th, q):
    return float(np.interp(q, qs, th))


def distal_pose(g, qs, th, q, mcp):
    """Origin (B) and rotation of the distal frame in the world (m), at flexion q and MCP angle mcp (rad)."""
    b = (g["ap"] + T.rot(theta_of(qs, th, q)) @ (g["b"] - g["ap"])) * MM
    R = T.rot(mcp)
    return R @ b, R @ T.rot(q)


def corner_x(g, qs, th, q, rho_mm):
    """Distal-frame x of the cylinder's contact on the distal face when it touches both faces at flexion q (MCP 0)."""
    from scipy.optimize import brentq

    face = g["pads"]["mount_offset_mm"] + g["pads"]["thickness_mm"]
    o, R = distal_pose(g, qs, th, q, 0.0)

    def gap(x):  # the centre's height above the proximal face's offset line minus rho
        c = o / MM + R @ np.array([x, -face - rho_mm])
        return float(c[1] + face + rho_mm)

    return brentq(gap, -200.0, 200.0)


def start_state(hand: str, rho_mm: float, q0: float, a_mm: float, var: str, p: dict):
    """O's start centre and the load's unit direction (world, from O), then this hand's start centre dropped onto its
    distal pad along the load line. Returns a dict (m, rad)."""
    from scipy.optimize import brentq

    q_end = math.radians(q0 + 5.0)
    q_start = q_end - math.radians(p["q_sweep_deg"])
    mcp0 = -math.radians(p["phi0_deg"])
    go, qso, tho = kin("O")
    face_o = go["pads"]["mount_offset_mm"] + go["pads"]["thickness_mm"]
    x_s = corner_x(go, qso, tho, q_end, rho_mm) + a_mm  # O's distal-frame start point
    o, R = distal_pose(go, qso, tho, q_start, mcp0)
    c_o = o + R @ np.array([x_s, -face_o - rho_mm]) * MM
    n_in, t_pip = R @ np.array([0.0, 1.0]), R @ np.array([-1.0, 0.0])
    b0 = math.radians(p["beta0_deg"])
    u = math.cos(b0) * n_in + math.sin(b0) * t_pip
    g, qs, th = kin(hand)
    pads = g["pads"]
    m_, h = pads["mount_offset_mm"], pads["thickness_mm"]
    oh, Rh = distal_pose(g, qs, th, q_start, mcp0)

    def clearance(s):  # rim to H's distal pad box at c_o - s u (mm)
        c = (c_o - s * u) / MM
        loc = Rh.T @ (c - oh / MM)
        return T._rect_gap(loc[0], loc[1], pads["distal_start_mm"], pads["distal_end_mm"], -(m_ + h), -m_) - rho_mm

    # the last crossing from inside to clear as the centre backs away along -u (from 30 mm toward the pad, out to 50 mm)
    grid = np.arange(-0.03, 0.05, 1e-4)
    vals = np.array([clearance(x) for x in grid])
    idx = [i for i in range(len(grid) - 1) if vals[i] < 0 <= vals[i + 1]]
    s = brentq(clearance, grid[idx[-1]], grid[idx[-1] + 1], xtol=1e-10) if idx else float("nan")
    c = c_o - s * u
    # the start must touch the distal pad only: clearance to the proximal pad (in the proximal frame)
    loc_p = T.rot(-mcp0) @ c / MM
    face = m_ + h
    prox_gap = T._rect_gap(loc_p[0], loc_p[1], pads["proximal_start_mm"], pads["proximal_end_mm"], -face, -m_) - rho_mm
    loc_d = Rh.T @ (c / MM - oh / MM)
    return {"c": c, "u": u, "drop_mm": s / MM, "q_start": q_start, "q_end": q_end, "mcp0": mcp0,
            "prox_gap_mm": prox_gap, "distal_x_mm": float(loc_d[0]), "x_s_o_mm": x_s,
            "on_pad": bool(pads["distal_start_mm"] <= loc_d[0] <= pads["distal_end_mm"])}


def build(hand, mu, rho_mm, time_step, spin_damping, k_theta, d_theta):
    from pydrake.geometry import AddContactMaterial, Box, Cylinder, ProximityProperties
    from pydrake.math import RigidTransform, RotationMatrix
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction, DiscreteContactApproximation
    from pydrake.multibody.tree import (FixedOffsetFrame, PdControllerGains, PrismaticJoint, RevoluteJoint,
                                        SpatialInertia, UnitInertia)
    from pydrake.systems.framework import DiagramBuilder

    g = T.finger_geometry(hand)
    pads = g["pads"]
    m, h, w = pads["mount_offset_mm"] * MM, pads["thickness_mm"] * MM, pads["width_mm"] * MM
    builder = DiagramBuilder()
    plant, sg = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
    world = plant.world_body()
    y = np.array([0.0, 1.0, 0.0])
    at = lambda p2: np.array([p2[0], 0.0, p2[1]])
    rod = lambda mass, length: SpatialInertia(mass, np.zeros(3), UnitInertia.SolidBox(max(length, 1e-3), 0.004, 0.004))
    a, ap, b = g["a"] * MM, g["ap"] * MM, g["b"] * MM
    proximal = plant.AddRigidBody("proximal", rod(0.039, 0.044))
    mcp = plant.AddJoint(RevoluteJoint("mcp", world.body_frame(), proximal.body_frame(), y))
    plant.AddJointActuator("mcp", mcp).set_controller_gains(PdControllerGains(p=XM335_K, d=XM335_D))
    driver = plant.AddRigidBody("driver", rod(0.003, float(np.linalg.norm(b - ap))))
    distal = plant.AddRigidBody("distal", rod(0.01, 0.03))
    f_ap = plant.AddFrame(FixedOffsetFrame("at_ap", proximal.body_frame(), RigidTransform(at(ap))))
    f_b = plant.AddFrame(FixedOffsetFrame("b_on_driver", driver.body_frame(), RigidTransform(at(b - ap))))
    servo = plant.AddJoint(RevoluteJoint("servo", f_ap, driver.body_frame(), y))
    plant.AddJointActuator("servo", servo).set_controller_gains(PdControllerGains(p=k_theta, d=d_theta))
    plant.AddJoint(RevoluteJoint("pip_b", f_b, distal.body_frame(), y))
    plant.AddDistanceConstraint(proximal, at(a), distal, at(g["bp_local"] * MM), g["abp"] * MM)
    cx = plant.AddRigidBody("carriage_x", rod(0.002, 0.01))
    cz = plant.AddRigidBody("carriage_z", rod(0.002, 0.01))
    rho = rho_mm * MM
    roller = plant.AddRigidBody("roller", SpatialInertia(T.MASS_KG, np.zeros(3),
                                                         UnitInertia.SolidCylinder(rho, T.LENGTH_M, y)))
    plant.AddJoint(PrismaticJoint("slide_x", world.body_frame(), cx.body_frame(), [1.0, 0.0, 0.0]))
    plant.AddJoint(PrismaticJoint("slide_z", cx.body_frame(), cz.body_frame(), [0.0, 0.0, 1.0]))
    spin = plant.AddJoint(RevoluteJoint("spin", cz.body_frame(), roller.body_frame(), y))
    # experiment 23's bearing damping, taken relative to the proximal phalanx: an implicit PD (p 0) whose velocity
    # target is the MCP's commanded rate (with the MCP held, it is experiment 22/23's damper on the world)
    plant.AddJointActuator("spin", spin).set_controller_gains(PdControllerGains(p=0.0, d=spin_damping))

    def props(kind):
        pr = ProximityProperties()
        AddContactMaterial(point_stiffness=T.POINT_STIFFNESS if kind == "pad" else 1.0e8,
                           friction=CoulombFriction(mu, mu), properties=pr)
        return pr

    s0, s1 = pads["proximal_start_mm"] * MM, pads["proximal_end_mm"] * MM
    plant.RegisterCollisionGeometry(proximal, RigidTransform(at(((s0 + s1) / 2, -(m + h / 2)))), Box(s1 - s0, w, h),
                                    "proximal_pad", props("pad"))
    t0, t1 = pads["distal_start_mm"] * MM, pads["distal_end_mm"] * MM
    plant.RegisterCollisionGeometry(distal, RigidTransform(at(((t0 + t1) / 2, -(m + h / 2)))), Box(t1 - t0, w, h),
                                    "distal_pad", props("pad"))
    plant.RegisterCollisionGeometry(roller, RigidTransform(RotationMatrix.MakeXRotation(math.pi / 2)),
                                    Cylinder(rho, T.LENGTH_M), "roller", props("roller"))
    plant.Finalize()
    plant.mutable_gravity_field().set_gravity_vector(np.zeros(3))
    return builder, plant, sg, {"proximal": proximal, "distal": distal, "roller": roller}


class Sim:
    def __init__(self, hand, mu, rho_mm, time_step, spin_damping, k_theta, d_theta):
        from pydrake.systems.analysis import Simulator
        from pydrake.systems.framework import LeafSystem

        builder, plant, sg, bodies = build(hand, mu, rho_mm, time_step, spin_damping, k_theta, d_theta)
        self.plant, self.bodies = plant, bodies
        self.cmd = np.zeros(2)  # mcp, theta (actuator order)
        self.mcp_rate = 0.0  # the spin damper's velocity target
        self.load = np.zeros(2)
        nv = plant.num_velocities()
        ix, iz = plant.GetJointByName("slide_x").velocity_start(), plant.GetJointByName("slide_z").velocity_start()
        outer = self

        class Command(LeafSystem):
            def __init__(self):
                super().__init__()
                self.DeclareVectorOutputPort("desired", 6, self.desired)
                self.DeclareVectorOutputPort("load", nv, self.force)

            def desired(self, context, out):
                out.SetFromVector(np.array([outer.cmd[0], outer.cmd[1], 0.0, 0.0, 0.0, outer.mcp_rate]))

            def force(self, context, out):
                tau = np.zeros(nv)
                tau[ix], tau[iz] = outer.load
                out.SetFromVector(tau)

        c = builder.AddSystem(Command())
        builder.Connect(c.get_output_port(0), plant.get_desired_state_input_port(
            plant.GetModelInstanceByName("DefaultModelInstance")))
        builder.Connect(c.get_output_port(1), plant.get_applied_generalized_force_input_port())
        self.diagram = builder.Build()
        self.sim = Simulator(self.diagram)
        self.pctx = plant.GetMyMutableContextFromRoot(self.sim.get_mutable_context())
        plant.get_actuation_input_port().FixValue(self.pctx, np.zeros(3))
        self.t = 0.0

    def step(self):
        self.t += T.DT_ENV
        self.sim.AdvanceTo(self.t)

    def contacts(self):
        """Per pad (0 proximal, 1 distal): normal-ish force magnitude on the cylinder and the deepest penetration (mm)."""
        plant = self.plant
        roller, distal, prox = (self.bodies[k].index() for k in ("roller", "distal", "proximal"))
        res = plant.get_contact_results_output_port().Eval(self.pctx)
        force, depth, point = np.zeros((2, 3)), np.zeros(2), np.full((2, 3), np.nan)
        for i in range(res.num_point_pair_contacts()):
            c = res.point_pair_contact_info(i)
            pair = {c.bodyA_index(), c.bodyB_index()}
            if roller not in pair or not pair & {prox, distal}:
                continue
            k = 0 if prox in pair else 1
            f = c.contact_force() if c.bodyB_index() == roller else -c.contact_force()
            force[k] += f
            depth[k] = max(depth[k], c.point_pair().depth / MM)
            point[k] = c.contact_point()
        return force, depth, point

    def pose(self):
        plant, pctx = self.plant, self.pctx
        Xd = plant.EvalBodyPoseInWorld(pctx, self.bodies["distal"])
        R = Xd.rotation().matrix()
        c = plant.EvalBodyPoseInWorld(pctx, self.bodies["roller"]).translation()
        return {"distal": [Xd.translation()[0] / MM, Xd.translation()[2] / MM, math.atan2(R[0, 2], R[0, 0])],
                "centre": [c[0] / MM, c[2] / MM], "spin": float(plant.GetJointByName("spin").get_angle(pctx)),
                "mcp": float(plant.GetJointByName("mcp").get_angle(pctx))}


def run_one(task):
    try:
        return _run_one(task)
    except RuntimeError as err:
        try:
            out = _run_one(task, time_step=task[4] / 2)
            out["retried_time_step"] = task[4] / 2
            return out
        except RuntimeError as err2:
            hand, mu, rho_mm, q0, _, a_mm, var, p = task[:8]
            return {"rho_mm": rho_mm, "q0_deg": q0, "a_mm": a_mm, "solver_fail": str(err2)[:200]}


def _run_one(task, time_step=None, frames_at=None):
    hand, mu, rho_mm, q0, ts, a_mm, var, p, delta = task
    time_step = time_step or ts
    g, qs, th = kin(hand)
    st = start_state(hand, rho_mm, q0, a_mm, var, p)
    q_start, q_end = st["q_start"] + math.radians(delta), st["q_end"] + math.radians(delta)
    mcp0 = st["mcp0"]
    ratio = float(np.interp(q_end, qs, np.gradient(th, qs)))
    s = Sim(hand, mu, rho_mm, time_step, p["spin_damping"], T.K_Q / ratio ** 2, T.D_Q / ratio ** 2)
    pl, pctx = s.plant, s.pctx
    th0 = theta_of(qs, th, q_start)
    pl.GetJointByName("mcp").set_angle(pctx, mcp0)
    pl.GetJointByName("servo").set_angle(pctx, th0)
    pl.GetJointByName("pip_b").set_angle(pctx, q_start - th0)
    pl.GetJointByName("slide_x").set_translation(pctx, st["c"][0])
    pl.GetJointByName("slide_z").set_translation(pctx, st["c"][1])
    s.cmd[:] = [mcp0, th0]
    s.load = st["u"] * p["load_n"] / T.FINGERS
    s.sim.Initialize()
    face = g["pads"]["mount_offset_mm"] + g["pads"]["thickness_mm"]
    n_settle, n_sweep, n_hold = (int(round(x / T.DT_ENV)) for x in (SETTLE_S, p["sweep_s"], p["hold_s"]))
    frames = []

    def prox_frame():  # the cylinder in the proximal-face frame (mm): along the face, off the face (rim gap)
        ps = s.pose()
        loc = T.rot(-ps["mcp"]) @ np.array(ps["centre"])
        return float(loc[0]), float(-loc[1] - face - rho_mm)

    def distal_x(point):
        Xd = pl.EvalBodyPoseInWorld(pctx, s.bodies["distal"])
        return float((Xd.inverse() @ point)[0] / MM)

    max_f, max_d, cap, cap_t, trace = np.zeros(2), np.zeros(2), None, None, []
    dx_start, prev = None, (0.0, 0.0)
    n_total = n_settle + n_sweep + n_hold
    for i in range(n_total):
        k = i - n_settle
        frac = 0.0 if k < 0 else min(1.0, (k + 1) / n_sweep)
        mcp_c = mcp0 * (1.0 - frac)
        q_c = q_start + (q_end - q_start) * frac
        s.cmd[:] = [mcp_c, theta_of(qs, th, q_c)]
        s.mcp_rate = -mcp0 / p["sweep_s"] if 0 <= k < n_sweep else 0.0
        s.step()
        force, depth, point = s.contacts()
        fn = np.linalg.norm(force, axis=1)
        max_f, max_d = np.maximum(max_f, fn), np.maximum(max_d, depth)
        if i == n_settle - 1:
            settle = {"x": prox_frame(), "fn": fn.tolist(),
                      "distal_x": distal_x(point[1]) if np.isfinite(point[1]).all() else None}
            dx_start = settle["distal_x"]
        ang = math.degrees(mcp_c if var == "A" else q_c - q_end)  # A: MCP command; B: PIP command from its end
        if k >= 0 and cap is None and fn[0] > T.GATE_N:  # proximal pad first above 0.2 N: interpolated in the step
            w = (fn[0] - T.GATE_N) / max(fn[0] - prev[1], 1e-12)
            cap = ang - w * (ang - prev[0])
            cap_t = (k + 1 - w) * T.DT_ENV
        prev = (ang, float(fn[0]))
        if (i + 1) % 6 == 0:
            ps = s.pose()
            trace.append([round(s.t, 3), round(math.degrees(ps["mcp"]), 2), round(ps["distal"][2] * 180 / math.pi, 2),
                          *[round(v, 3) for v in prox_frame()], round(float(fn[0]), 3), round(float(fn[1]), 3)])
        if frames_at is not None and i in frames_at:
            frames.append({**s.pose(), "load_n": (s.load * T.FINGERS).tolist(), "step": i})
    force, depth, point = s.contacts()
    fn = np.linalg.norm(force, axis=1)
    x_end, off_end = prox_frame()
    ps = s.pose()
    out = {"rho_mm": rho_mm, "q0_deg": q0, "a_mm": a_mm, "start": {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                                                                  for k, v in st.items()},
           "settle": settle, "x_end_mm": x_end, "off_face_end_mm": off_end,
           "distal_x_end_mm": distal_x(point[1]) if np.isfinite(point[1]).all() else None,
           "distal_slide_mm": (distal_x(point[1]) - dx_start) if (dx_start is not None and np.isfinite(point[1]).all())
           else None,
           "rotation_deg": -math.degrees(ps["spin"]), "capture_deg": cap, "capture_s": cap_t,
           "held": bool((fn >= T.GATE_N).all()), "normal_end_n": fn.tolist(), "max_force_n": max_f.tolist(),
           "max_depth_mm": max_d.tolist(), "end_pen_mm_from_force": (fn / T.POINT_STIFFNESS / MM).tolist(), "max_pen_mm_from_force": (max_f / T.POINT_STIFFNESS / MM).tolist(),
           "q_end_deg": math.degrees(ps["distal"][2] - ps["mcp"]), "mcp_end_deg": math.degrees(ps["mcp"]),
           "trace": trace}
    if frames_at is not None:
        out["frames"] = frames
    return out


def placements(args):
    pl = PLACEMENTS[args.variant]
    rho, q0s, aa = (args.rho or pl["rho"]), (args.q0 or pl["q0"]), (args.a or pl["a"])
    return [(r, q0, a) for r in rho for q0 in q0s for a in aa]


def cmd_run(args):
    p = dict(DEFAULTS[args.variant])
    for k in p:
        v = getattr(args, k, None)
        if v is not None:
            p[k] = v
    tasks = [(args.hand, args.mu, r, q0, args.time_step, a, args.variant, p, args.delta_deg)
             for r, q0, a in placements(args)]
    t0 = time.time()
    with Pool(args.workers) as pool:
        envs = pool.map(run_one, tasks)
    name = f"acqmcp{args.variant}_{args.hand}_mu{args.mu:g}_{args.tag}.json"
    OUT.mkdir(exist_ok=True)
    (OUT / name).write_text(json.dumps({"hand": args.hand, "mu": args.mu, "variant": args.variant, "params": p,
                                        "delta_deg": args.delta_deg, "envs": envs}))
    held = sum(bool(e.get("held")) for e in envs)
    print(f"{args.variant} {args.hand} mu {args.mu}: held {held}/{len(envs)} in {time.time() - t0:.0f} s -> {name}")
    if args.verbose:
        for e in envs:
            if "solver_fail" in e:
                print("  FAIL", e)
                continue
            print(f"  r{e['rho_mm']:g} q0 {e['q0_deg']:g} a {e['a_mm']:g}: drop {e['start']['drop_mm']:+.2f} "
                  f"gap0 {e['start']['prox_gap_mm']:.2f} | cap {e['capture_deg']} | x_end {e['x_end_mm']:.2f} "
                  f"off {e['off_face_end_mm']:+.2f} slide {e['distal_slide_mm']} rot {e['rotation_deg']:.1f} "
                  f"held {e['held']} Fn {np.round(e['normal_end_n'], 2)} maxF {np.round(e['max_force_n'], 2)} "
                  f"depth {np.round(e['max_depth_mm'], 2)}")
    return 0


MEASURES = ("x_end_mm", "capture_deg", "distal_slide_mm", "rotation_deg")


def load_envs(var, hand, mu, tag):
    p = OUT / f"acqmcp{var}_{hand}_mu{mu:g}_{tag}.json"
    return {(e["rho_mm"], e["q0_deg"], e["a_mm"]): e for e in json.loads(p.read_text())["envs"]} if p.exists() else None


def pair_ok(e_lo, e_hi, measure):
    """Both runs exist, started on the distal pad only, end held by both pads and have the measure."""
    return all(e is not None and "solver_fail" not in e and e["held"] and e.get(measure) is not None
               and e["start"]["prox_gap_mm"] > 0 and e["start"]["on_pad"] for e in (e_lo, e_hi))


def cmd_analyze(args):
    from analyze import DESIGN, spearman_perm

    m = args.measure
    thr = THRESHOLD[m] if args.threshold is None else args.threshold
    o = [load_envs(args.variant, "O", mu, args.tag) for mu in (1.3, 2.0)]
    pen_o = max(max(e["max_force_n"]) / T.POINT_STIFFNESS / MM for d in o for e in d.values() if "solver_fail" not in e)
    print(f"O: largest force / 2.5 kN/m = {pen_o:.2f} mm")
    keys = [k for k in o[0] if pair_ok(o[0][k], o[1].get(k), m) and abs(o[1][k][m] - o[0][k][m]) >= thr]
    print(f"pairs (O holds at both mu, |D(O)| >= {thr}): {len(keys)}/{len(o[0])}")
    rows = []
    hands = args.hands or ["R", "U"] + [h for h in DESIGN if h not in ("R", "U")]
    for hand in hands:
        h = [load_envs(args.variant, hand, mu, args.tag) for mu in (1.3, 2.0)]
        if any(x is None for x in h):
            continue
        rel, lost, pen = [], 0, 0.0
        for k in keys:
            d_o = o[1][k][m] - o[0][k][m]
            if not pair_ok(h[0].get(k), h[1].get(k), m):
                lost += 1
                continue
            rel.append(abs((h[1][k][m] - h[0][k][m]) - d_o) / abs(d_o))
        pen = max(max(e["max_force_n"]) / T.POINT_STIFFNESS / MM for d in h for e in d.values()
                  if "solver_fail" not in e)
        if not rel:
            print(f"{hand:22s} no pairs (lost {lost})")
            continue
        dc, ds = DESIGN.get(hand, (None, None))
        depth = max(max(e["max_depth_mm"]) for d in h for e in d.values() if "solver_fail" not in e)
        rows.append({"hand": hand, "d_contact": dc, "d_skel": ds, "E": float(np.median(rel)), "pairs": len(rel),
                     "lost": lost, "E_lost_max": float(np.median(rel + [np.inf] * lost)), "pen_mm": pen,
                     "depth_mm": depth})
        print(f"{hand:22s} D_c {dc} D_s {ds} | E {100 * rows[-1]['E']:6.1f}% ({len(rel)} pairs, lost {lost}) "
              f"max depth {depth:.2f} mm, peak F/k {pen:.2f} mm")
    if args.noise_tag:
        nz = [load_envs(args.variant, "O", mu, args.noise_tag) for mu in (1.3, 2.0)]
        rel = [abs((nz[1][k][m] - nz[0][k][m]) - (o[1][k][m] - o[0][k][m])) / abs(o[1][k][m] - o[0][k][m])
               for k in keys if pair_ok(nz[0].get(k), nz[1].get(k), m)]
        print(f"noise floor (O, +0.05 deg): E {100 * np.median(rel):.2f}% (75th {100 * np.percentile(rel, 75):.2f}%, "
              f"{len(rel)} pairs)")
    rs = [r for r in rows if r["d_contact"] is not None]
    if len(rs) >= 3:
        rho, pv = spearman_perm([r["d_contact"] for r in rs], [r["E"] for r in rs])
        rho_s, pv_s = spearman_perm([r["d_skel"] for r in rs], [r["E"] for r in rs])
        print(f"rho(E, D_contact) {rho:.2f} (one-sided p {pv:.4f}); rho(E, D_skel) {rho_s:.2f} (p {pv_s:.4f}); "
              f"hands {len(rs)}")
        if any(r["lost"] for r in rs):  # lost pairs as the largest error (rank analysis)
            el = [r["E_lost_max"] if np.isfinite(r["E_lost_max"]) else 1e9 for r in rs]
            rho_l, pv_l = spearman_perm([r["d_contact"] for r in rs], el)
            print(f"  lost as largest: rho(E, D_contact) {rho_l:.2f} (p {pv_l:.4f})")
    if args.out:
        Path(args.out).write_text(json.dumps({"measure": m, "threshold": thr, "pairs": len(keys), "rows": rows},
                                             indent=1))
    return 0


def cmd_calib(args):
    """O only: every candidate measure's D(O) per placement, and the gates."""
    o = [load_envs(args.variant, "O", mu, args.tag) for mu in (1.3, 2.0)]
    print("placement | held 1.3/2.0 | " + " | ".join(f"{m}: lo, hi, D" for m in MEASURES) + " | max F (N), depth (mm)")
    for k in o[0]:
        lo, hi = o[0][k], o[1].get(k)
        if "solver_fail" in lo or hi is None or "solver_fail" in hi:
            print(k, "solver fail")
            continue
        cells = []
        for m in MEASURES:
            a, b = lo.get(m), hi.get(m)
            cells.append(f"{a if a is None else round(a, 2)}, {b if b is None else round(b, 2)}, "
                         f"{None if a is None or b is None else round(b - a, 2)}")
        print(k, lo["held"], hi["held"], " | ".join(cells), "|",
              np.round(np.maximum(lo["max_force_n"], hi["max_force_n"]), 2),
              np.round(np.maximum(lo["max_depth_mm"], hi["max_depth_mm"]), 2),
              "gap0", round(lo["start"]["prox_gap_mm"], 2))


def cmd_frames(args):
    """Three frames on O (mu 1.3): start, mid-capture, captured, drawn in the finger plane."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, FancyArrow, Polygon

    p = dict(DEFAULTS[args.variant])
    r, q0, a = args.frame_placement
    n_settle, n_sweep = int(round(SETTLE_S / T.DT_ENV)), int(round(p["sweep_s"] / T.DT_ENV))
    probe = _run_one(("O", 1.3, r, q0, 1e-3, a, args.variant, p, 0.0))
    cap_step = n_settle + int(round(probe["capture_s"] / T.DT_ENV)) - 1 if probe["capture_s"] else n_settle + n_sweep // 2
    mid = max(n_settle, cap_step - 1)  # the step before the proximal pad takes load: the cylinder sliding over
    n_total = n_settle + n_sweep + int(round(p["hold_s"] / T.DT_ENV))
    out = _run_one(("O", 1.3, r, q0, 1e-3, a, args.variant, p, 0.0), frames_at={n_settle - 1, mid, n_total - 1})
    g = T.finger_geometry("O")
    pads = g["pads"]
    m_, h = pads["mount_offset_mm"], pads["thickness_mm"]
    titles = ["start (on the distal pad)", "mid-capture (sliding, just before contact)", "captured (two pads)"]
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.8))
    for ax, fr, ttl in zip(axes, out["frames"], titles):
        mcp = fr["mcp"]
        Rm = T.rot(mcp)
        P = lambda pts: [Rm @ np.array(p_) for p_ in pts]
        link = [np.array([0.0, 4.0]), np.array([pads["proximal_end_mm"] + 4, 4.0]),
                np.array([pads["proximal_end_mm"] + 4, -m_]), np.array([0.0, -m_])]
        ax.add_patch(Polygon(P(link), closed=True, fc="#d9d9d9", ec="#555"))
        prox = [np.array([pads["proximal_start_mm"], -m_]), np.array([pads["proximal_end_mm"], -m_]),
                np.array([pads["proximal_end_mm"], -m_ - h]), np.array([pads["proximal_start_mm"], -m_ - h])]
        ax.add_patch(Polygon(P(prox), closed=True, fc="#4c8bd6", ec="#1f4f8c"))
        dx, dz, dth = fr["distal"]
        Rd = T.rot(dth)
        o = np.array([dx, dz])
        D = lambda pts: [o + Rd @ np.array(p_) for p_ in pts]
        t0, t1 = pads["distal_start_mm"], pads["distal_end_mm"]
        dl = [np.array([t0 - 2, 4.0]), np.array([t1 + 2, 4.0]), np.array([t1 + 2, -m_]), np.array([t0 - 2, -m_])]
        ax.add_patch(Polygon(D(dl), closed=True, fc="#d9d9d9", ec="#555"))
        ax.add_patch(Polygon(D([np.array([t0, -m_]), np.array([t1, -m_]), np.array([t1, -m_ - h]),
                                np.array([t0, -m_ - h])]), closed=True, fc="#e07b39", ec="#8c4515"))
        c = np.array(fr["centre"])
        ax.add_patch(Circle(c, r, fc="#f2f2f2", ec="#222", lw=1.5))
        ax.plot([c[0], c[0] + r * math.cos(-fr["spin"])], [c[1], c[1] + r * math.sin(-fr["spin"])], color="#222")
        u = np.array(fr["load_n"]) / max(np.linalg.norm(fr["load_n"]), 1e-9)
        ax.add_patch(FancyArrow(*(c - 14 * u), *(10 * u), width=0.8, color="#c0392b"))
        ax.plot(0, 0, "ko", ms=5)
        ax.set_title(f"{ttl}\nMCP {math.degrees(mcp):+.0f} deg, PIP {math.degrees(dth - mcp):.0f} deg", fontsize=10)
        ax.set_aspect("equal")
        ax.set_xlim(-10, 85)
        ax.set_ylim(-60, 40)
        ax.grid(alpha=0.2)
    label = {"A": "Variant A: MCP flexes, PIP held", "B": "Variant B: PIP closes, MCP held"}[args.variant]
    fig.suptitle(f"{label}; hand O, rho {r:g} mm, q0 {q0:g} deg, a {a:g} mm, mu 1.3 (blue: proximal pad, orange: "
                 f"distal pad, red: load)", fontsize=10)
    fig.tight_layout()
    path = OUT / f"acqmcp_frames_{args.variant}.png"
    fig.savefig(path, dpi=110)
    print(path)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--variant", choices=("A", "B"), required=True)
    r.add_argument("--hand", required=True)
    r.add_argument("--mu", type=float, required=True)
    r.add_argument("--rho", type=float, nargs="+", default=None)
    r.add_argument("--q0", type=float, nargs="+", default=None)
    r.add_argument("--a", type=float, nargs="+", default=None)
    r.add_argument("--time_step", type=float, default=1e-3)
    r.add_argument("--delta_deg", type=float, default=0.0, help="noise floor: shift O's PIP command")
    for k in DEFAULTS["A"]:
        r.add_argument(f"--{k}", type=float, default=None)
    r.add_argument("--workers", type=int, default=3)
    r.add_argument("--tag", required=True)
    r.add_argument("-v", "--verbose", action="store_true")
    a = sub.add_parser("analyze")
    a.add_argument("--variant", choices=("A", "B"), required=True)
    a.add_argument("--tag", required=True)
    a.add_argument("--measure", default="capture_deg")
    a.add_argument("--threshold", type=float, default=None, help="|D(O)| for a pair (default: THRESHOLD)")
    a.add_argument("--hands", nargs="+", default=None)
    a.add_argument("--noise_tag", default=None)
    a.add_argument("--out", default=None)
    c = sub.add_parser("calib")
    c.add_argument("--variant", choices=("A", "B"), required=True)
    c.add_argument("--tag", required=True)
    f = sub.add_parser("frames")
    f.add_argument("--variant", choices=("A", "B"), required=True)
    f.add_argument("--frame_placement", type=float, nargs=3, default=[15.0, 80.0, 6.0])
    args = ap.parse_args()
    return {"run": cmd_run, "analyze": cmd_analyze, "calib": cmd_calib, "frames": cmd_frames}[args.cmd](args)


if __name__ == "__main__":
    raise SystemExit(main())
