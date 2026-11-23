"""Study 3's two-pad grasp measurement (condition A) replicated in Drake, with point or hydroelastic contact.

Isaac Lab's hydroelastic path could not be used (docs/experimental/separability.md, sixteenth experiment, review): its
runs are not repeatable (E between identical runs 30%, against 2% for point contact under a 0.05 deg command change).
Drake's hydroelastic contact is the second contact model; its point contact first checks that this replica reproduces
the Isaac Lab measurement.

The finger is one planar CoDe finger (Isaac Lab grasps with three identical ones; loads, masses and thresholds here are
a third of theirs, and F_limit is reported times three, in the same units). Frame: x along the proximal phalanx from the
MCP, z dorsal, y along the cylinder's axis; the MCP is held at 0, so the proximal phalanx is the world. The crossed
four-bar is a tree world -(revolute at A')- driver A'B -(revolute at B)- distal, closed by a distance constraint
|A B'| = abp. Condition A: every hand gets O's command, flexion q0 + delta, through a flexion-space impedance
(K_Q = 5 N m/rad, D_Q = 0.08 N m s/rad) applied as a moment on the distal phalanx, which does the same virtual work as
the PIP servo's rescheduled gain K_Q / (dtheta/dq)^2 in Isaac Lab. The cylinder (radius rho) rides a planar slide
(x, z, spin) whose drives hold it while the finger closes, are released, and the grasp settles (Gate 0); a load then
ramps until the first of slip (a pad's tangential speed > 10 mm/s), loss (a pad's normal force < 0.2 N / 3) or
collapse (the cylinder moves 5 mm). Contact: only the two pads and the cylinder collide.

    uv run python two_pad_drake.py --hand O --mu 1.3 --radius_mm 15 --contact point [--only 2] [--workers 8]
"""
from __future__ import annotations

import argparse
import json
import math
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
HAND_DIR = ROOT / "design" / "output" / "sim" / "study3" / "hands"
GRASPS = ROOT / "design" / "output" / "sim" / "study3" / "two_pad_grasps.json"
OUT = Path(__file__).resolve().parent / "logs"

FINGERS = 3  # Isaac Lab's hand: index, middle, ring on one cylinder
K_Q, D_Q = 5.0, 0.08
DT_ENV = 1.0 / 30.0
LENGTH_M, MASS_KG = 0.07 / FINGERS, 0.05 / FINGERS
RATE_N_S, MAX_FORCE_N = 2.0 / FINGERS, 80.0 / FINGERS
LOSS_N, COLLAPSE_MM, ROLL_MM, SLIP_MM_S = 0.2 / FINGERS, 5.0, 1.0, 10.0
GATE_N, GATE_SPEED_MM_S, GATE_SPIN_RAD_S = 0.2, 2.0, 0.1
S2 = math.sqrt(0.5)
DIRECTIONS = {"+x": (1.0, 0.0), "+x-z": (S2, -S2), "-z": (0.0, -1.0), "-x": (-1.0, 0.0)}  # x along, z dorsal
POINT_STIFFNESS = 2.5e3  # N/m, Isaac Lab's shape ke
PAD_REFERENCE_M = 0.003  # every hand's pads get O's foundation stiffness: modulus / 3 mm (sixteenth experiment)
DIAL_BRAKE_NM = 0.0  # the dial's Coulomb-like brake torque (task 1)
DIAL_Z_STIFFNESS = 0.0  # N/m: the dial's axis on a soft spring off the proximal face (0: held stiffly)
SPIN_DAMPING = 0.0  # N m s/rad on the cylinder's spin (the rolling task's bearing); 0 for the limit measurement
MODULUS_PA = 1.0e5  # soft silicone; chosen on O alone so its settled pad force sits near the point model's


def hand_file(hand: str) -> Path:
    stem = {"O": "h3to7_h0", "R": "h3to7_hr", "U": "h3to7_hu"}.get(hand)
    if stem is None:
        stem = hand if hand.startswith("h3to") else f"h3to7_{hand}"
    return HAND_DIR / f"cfl_q120_{stem}.yaml"


def finger_geometry(hand: str) -> dict:
    """Pivots at q = 0 (mm, x along, z dorsal), link lengths and pads, from the hand's parameter file."""
    d = yaml.safe_load(hand_file(hand).read_text())
    f, p = d["finger"], d["pads"]
    a = np.array([f["a_x_mm"], f["a_z_mm"]])
    ap = np.array([f["ap_x_mm"], f["ap_z_mm"]])
    b = np.array([ap[0] + math.sqrt(f["apb_mm"] ** 2 - ap[1] ** 2), 0.0])  # on the link line, distal of A'
    bp = b + np.array([f["bp_x_mm"], f["bp_z_mm"]])
    return {"a": a, "ap": ap, "b": b, "bp_local": np.array([f["bp_x_mm"], f["bp_z_mm"]]), "abp": f["abp_mm"],
            "abp_check": float(np.linalg.norm(bp - a)), "pads": p}


def rot(t: float) -> np.ndarray:
    """Flexion by t in (x, z dorsal): a rotation about +y, taking +x toward -z (palmar)."""
    c, s_ = math.cos(t), math.sin(t)
    return np.array([[c, s_], [-s_, c]])


def servo_table(g: dict, q_max_deg: float = 125.0) -> tuple[np.ndarray, np.ndarray]:
    """The driver angle theta (about +y at A') that closes the four-bar at each flexion q, from 0 (rad, rad). For each
    q, B(theta) = A' + R(theta)(B0 - A') and B' = B + R(q) B'_local must keep |A B'| = abp; theta is followed
    continuously from 0."""
    from scipy.optimize import brentq

    a, ap, b0, bpl, abp = g["a"], g["ap"], g["b"], g["bp_local"], g["abp"]

    def gap(theta, q):
        bb = ap + rot(theta) @ (b0 - ap)
        return float(np.linalg.norm(bb + rot(q) @ bpl - a) - abp)

    qs = np.radians(np.arange(0.0, q_max_deg + 0.25, 0.25))
    th = [0.0]
    for q in qs[1:]:
        t0 = th[-1]
        lo, hi = t0 - 0.05, t0 + 0.05
        while gap(lo, q) * gap(hi, q) > 0:
            lo, hi = lo - 0.05, hi + 0.05
        th.append(brentq(gap, lo, hi, args=(q,), xtol=1e-12))
    return qs, np.array(th)


def _build(hand: str, mu: float, rho_mm: float, contact: str, time_step: float, hold: bool,
           k_theta: float, d_theta: float):
    from pydrake.geometry import (AddCompliantHydroelasticProperties, AddContactMaterial, AddRigidHydroelasticProperties,
                                  Box, Cylinder, ProximityProperties)
    from pydrake.math import RigidTransform, RotationMatrix
    from pydrake.multibody.plant import (AddMultibodyPlantSceneGraph, ContactModel, CoulombFriction,
                                         DiscreteContactApproximation)
    from pydrake.multibody.tree import (FixedOffsetFrame, PdControllerGains, PrismaticJoint, RevoluteJoint,
                                        SpatialInertia, UnitInertia)
    from pydrake.systems.framework import DiagramBuilder

    g = finger_geometry(hand)
    mm = 1e-3
    pads = g["pads"]
    m, h, w = pads["mount_offset_mm"] * mm, pads["thickness_mm"] * mm, pads["width_mm"] * mm
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
    plant.set_contact_model(ContactModel.kHydroelasticWithFallback if contact == "hydro" else ContactModel.kPoint)
    world = plant.world_body()
    y = np.array([0.0, 1.0, 0.0])

    def at(p2):  # (x, z) in m -> 3D
        return np.array([p2[0], 0.0, p2[1]])

    def rod(mass, length):
        return SpatialInertia(mass, np.zeros(3), UnitInertia.SolidBox(max(length, 1e-3), 0.004, 0.004))

    a, ap, b = g["a"] * mm, g["ap"] * mm, g["b"] * mm
    bp_local = g["bp_local"] * mm
    driver = plant.AddRigidBody("driver", rod(0.003, float(np.linalg.norm(b - ap))))
    distal = plant.AddRigidBody("distal", rod(0.01, 0.03))
    f_ap = plant.AddFrame(FixedOffsetFrame("at_ap", world.body_frame(), RigidTransform(at(ap))))
    f_b_on_driver = plant.AddFrame(FixedOffsetFrame("b_on_driver", driver.body_frame(), RigidTransform(at(b - ap))))
    servo = plant.AddJoint(RevoluteJoint("servo", f_ap, driver.body_frame(), y))
    # the PIP servo: an implicit PD on the driver, the flexion impedance mapped through dtheta/dq as in Isaac Lab
    plant.AddJointActuator("servo", servo).set_controller_gains(PdControllerGains(p=k_theta, d=d_theta))
    plant.AddJoint(RevoluteJoint("pip_b", f_b_on_driver, distal.body_frame(), y))
    # the proximal phalanx: a body welded to the world (a distance constraint on the world body itself crashes Drake)
    proximal = plant.AddRigidBody("proximal", rod(0.039, 0.044))
    plant.WeldFrames(world.body_frame(), proximal.body_frame())
    plant.AddDistanceConstraint(proximal, at(a), distal, at(bp_local), g["abp"] * mm)

    # cylinder on a planar slide: world -x-> carriage_x -z-> carriage_z -spin(y)-> roller
    cx = plant.AddRigidBody("carriage_x", rod(0.002, 0.01))
    cz = plant.AddRigidBody("carriage_z", rod(0.002, 0.01))
    rho = rho_mm * mm
    I_cyl = UnitInertia.SolidCylinder(rho, LENGTH_M, y)
    roller = plant.AddRigidBody("roller", SpatialInertia(MASS_KG, np.zeros(3), I_cyl))
    face = m + h  # proximal face depth below the link line
    joints = [plant.AddJoint(PrismaticJoint("slide_x", world.body_frame(), cx.body_frame(), [1.0, 0.0, 0.0])),
              plant.AddJoint(PrismaticJoint("slide_z", cx.body_frame(), cz.body_frame(), [0.0, 0.0, 1.0])),
              plant.AddJoint(RevoluteJoint("spin", cz.body_frame(), roller.body_frame(), y, damping=SPIN_DAMPING))]
    if hold:  # the drives that hold the cylinder while the finger closes (implicit PD; a second plant releases it)
        held = joints if hold is True else joints[:2]  # hold == "xz": a dial, free to turn only
        z_gains = (DIAL_Z_STIFFNESS, 5.0) if hold == "xz" and DIAL_Z_STIFFNESS > 0 else (1.0e5, 1.0e3)
        for j, (kp, kd) in zip(held, ((1.0e5, 1.0e3), z_gains, (10.0, 0.1))):
            plant.AddJointActuator(j.name(), j).set_controller_gains(PdControllerGains(p=kp, d=kd))
        if hold == "xz" and DIAL_BRAKE_NM > 0.0:  # the dial's brake: a damper that saturates at DIAL_BRAKE_NM
            plant.AddJointActuator("brake", joints[2], effort_limit=DIAL_BRAKE_NM).set_controller_gains(
                PdControllerGains(p=0.0, d=100.0))

    def props(kind: str, thickness: float = 0.0) -> ProximityProperties:
        pr = ProximityProperties()
        if contact == "hydro":
            if kind == "pad":
                # Drake's compliant box: pressure E * depth / (its smallest half-size) near its faces, so a foundation
                # stiffness k (N/m^3) needs E = k * thickness / 2
                AddCompliantHydroelasticProperties(resolution_hint=0.002,
                                                   hydroelastic_modulus=MODULUS_PA / PAD_REFERENCE_M * thickness / 2,
                                                   properties=pr)
            else:
                AddRigidHydroelasticProperties(resolution_hint=0.0005, properties=pr)
            AddContactMaterial(friction=CoulombFriction(mu, mu), properties=pr)
        else:
            k = POINT_STIFFNESS if kind == "pad" else 1.0e8  # the pair's stiffness is the pad's (springs in series)
            AddContactMaterial(point_stiffness=k, friction=CoulombFriction(mu, mu), properties=pr)
        return pr

    s0, s1 = pads["proximal_start_mm"] * mm, pads["proximal_end_mm"] * mm
    plant.RegisterCollisionGeometry(proximal, RigidTransform(at(((s0 + s1) / 2, -(m + h / 2)))), Box(s1 - s0, w, h),
                                    "proximal_pad", props("pad", h))
    t0, t1 = pads["distal_start_mm"] * mm, pads["distal_end_mm"] * mm
    plant.RegisterCollisionGeometry(distal, RigidTransform(at(((t0 + t1) / 2, -(m + h / 2)))), Box(t1 - t0, w, h),
                                    "distal_pad", props("pad", h))
    plant.RegisterCollisionGeometry(roller, RigidTransform(RotationMatrix.MakeXRotation(math.pi / 2)),
                                    Cylinder(rho, LENGTH_M), "roller", props("roller"))
    plant.Finalize()
    plant.mutable_gravity_field().set_gravity_vector(np.zeros(3))  # as Isaac Lab's measurement (gravity off)
    return builder, plant, scene_graph, {"driver": driver, "distal": distal, "roller": roller, "face": face,
                                         "proximal": proximal}


class Controller:
    """The settings the plant's inputs read: the servo's target angle, where the cylinder is held, the load."""

    def __init__(self):
        self.theta_cmd = 0.0
        self.load = np.zeros(2)
        self.hold_at = np.zeros(3)


def make_system(hand, mu, rho_mm, contact, time_step, hold, k_theta, d_theta):
    from pydrake.systems.analysis import Simulator
    from pydrake.systems.framework import LeafSystem

    builder, plant, scene_graph, bodies = _build(hand, mu, rho_mm, contact, time_step, hold, k_theta, d_theta)
    ctrl = Controller()
    nv = plant.num_velocities()
    n_act = plant.num_actuators()
    j_x, j_z = plant.GetJointByName("slide_x"), plant.GetJointByName("slide_z")
    ix, iz = j_x.velocity_start(), j_z.velocity_start()

    class Command(LeafSystem):
        def __init__(self):
            super().__init__()
            self.DeclareVectorOutputPort("desired", 2 * n_act, self.calc_desired)
            self.DeclareVectorOutputPort("load", nv, self.calc_load)

        def calc_desired(self, context, out):  # actuators in order: servo, then (if held) slide_x, slide_z, spin
            qd = [ctrl.theta_cmd] + (list(ctrl.hold_at)[: n_act - 1] if hold else [])
            out.SetFromVector(np.array(qd + [0.0] * n_act))

        def calc_load(self, context, out):
            tau = np.zeros(nv)
            tau[ix], tau[iz] = ctrl.load
            out.SetFromVector(tau)

    cmd = builder.AddSystem(Command())
    builder.Connect(cmd.get_output_port(0), plant.get_desired_state_input_port(plant.GetModelInstanceByName("DefaultModelInstance")))
    builder.Connect(cmd.get_output_port(1), plant.get_applied_generalized_force_input_port())
    diagram = builder.Build()
    sim = Simulator(diagram)
    ctx = sim.get_mutable_context()
    plant.get_actuation_input_port().FixValue(plant.GetMyMutableContextFromRoot(ctx), np.zeros(n_act))
    return diagram, sim, plant, bodies, ctrl, scene_graph


class Stage:
    """One built system and the measurements on it."""

    def __init__(self, hand, mu, rho_mm, contact, time_step, hold, k_theta, d_theta):
        self.diagram, self.sim, self.plant, self.bodies, self.ctrl, self.sg = make_system(
            hand, mu, rho_mm, contact, time_step, hold, k_theta, d_theta)
        self.pctx = self.plant.GetMyMutableContextFromRoot(self.sim.get_mutable_context())
        self.inspector = self.sg.model_inspector()
        self.t = 0.0

    def step(self):
        self.t += DT_ENV
        self.sim.AdvanceTo(self.t)

    def contacts(self):
        """Per pad (0 proximal, 1 distal): the force on the cylinder and its force-weighted point."""
        plant, pctx = self.plant, self.pctx
        roller, distal, prox = (self.bodies[k].index() for k in ("roller", "distal", "proximal"))
        res = plant.get_contact_results_output_port().Eval(pctx)
        force, point = np.zeros((2, 3)), np.zeros((2, 3))

        def add(body_a, body_b, f_on_a, p):
            pair = {body_a, body_b}
            if roller not in pair or not pair & {prox, distal}:
                return
            pad = 0 if prox in pair else 1
            f_cyl = f_on_a if body_a == roller else -f_on_a
            force[pad] += f_cyl
            point[pad] += np.linalg.norm(f_cyl) * p

        for i in range(res.num_point_pair_contacts()):
            c = res.point_pair_contact_info(i)
            add(c.bodyB_index(), c.bodyA_index(), c.contact_force(), c.contact_point())  # contact_force acts on B
        for i in range(res.num_hydroelastic_contacts()):
            c = res.hydroelastic_contact_info(i)
            surf = c.contact_surface()
            body_m = plant.GetBodyFromFrameId(self.inspector.GetFrameId(surf.id_M())).index()
            body_n = plant.GetBodyFromFrameId(self.inspector.GetFrameId(surf.id_N())).index()
            add(body_m, body_n, c.F_Ac_W().translational(), surf.centroid())  # F_Ac_W acts on M's body
        mag = np.linalg.norm(force, axis=1)
        with np.errstate(invalid="ignore", divide="ignore"):
            point = np.where(mag[:, None] > 1e-12, point / mag[:, None], np.nan)
        return force, point

    def measure(self):
        plant, pctx = self.plant, self.pctx
        force, point = self.contacts()
        Xr = plant.EvalBodyPoseInWorld(pctx, self.bodies["roller"])
        Vr = plant.EvalBodySpatialVelocityInWorld(pctx, self.bodies["roller"])
        Xd = plant.EvalBodyPoseInWorld(pctx, self.bodies["distal"])
        Vd = plant.EvalBodySpatialVelocityInWorld(pctx, self.bodies["distal"])
        R = Xd.rotation().matrix()
        qf = math.atan2(R[0, 2], R[0, 0])  # rotation about +y
        c, pd = Xr.translation(), Xd.translation()
        normal, slip, unit_n = np.zeros(2), np.zeros(2), np.zeros((2, 3))
        for k in range(2):
            if not np.all(np.isfinite(point[k])):
                continue
            r = c - point[k]
            r[1] = 0.0
            n = r / max(np.linalg.norm(r), 1e-12)  # the cylinder's true normal: contact point -> its axis
            unit_n[k] = n
            normal[k] = float(force[k] @ n)
            v_cyl = Vr.translational() + np.cross(Vr.rotational(), point[k] - c)
            v_pad = np.zeros(3) if k == 0 else Vd.translational() + np.cross(Vd.rotational(), point[k] - pd)
            rel = v_cyl - v_pad
            slip[k] = float(np.linalg.norm(rel - (rel @ n) * n))
        return {"force": force, "point": point, "normal": normal, "slip": slip, "n": unit_n, "c": c,
                "v": Vr.translational(), "w": Vr.rotational()[1], "q": qf}


def roll(s: "Stage", m0: dict, q_cmd: float, qs, th, opts: dict) -> dict:
    """In-hand rolling (task 2): from the settled grasp, the same open-loop flexion sweep on every hand, q_cmd ->
    q_cmd + sweep over sweep_s, then held for hold_s. Returns the cylinder's rotation (deg, + with flexion), its
    displacement in the proximal-face frame (mm: along the face toward the tip, away from the face), whether both pads
    still touch at the end, and the step at which the grasp was lost, if it was."""
    spin = s.plant.GetJointByName("spin")
    phi0 = spin.get_angle(s.pctx)
    c0 = m0["c"].copy()
    n_sweep, n_hold = int(round(opts["sweep_s"] / DT_ENV)), int(round(opts["hold_s"] / DT_ENV))
    lost_at = None
    q1 = min(q_cmd + math.radians(opts["sweep_deg"]), math.radians(120.0))  # never past the required flexion
    for i in range(n_sweep + n_hold):
        q = q_cmd + (q1 - q_cmd) * min(1.0, (i + 1) / n_sweep)
        s.ctrl.theta_cmd = float(np.interp(q, qs, th))
        s.step()
        m_ = s.measure()
        if lost_at is None and (m_["normal"] < LOSS_N).any():
            lost_at = i + 1
    m_ = s.measure()
    dc = 1e3 * (m_["c"] - c0)
    return {"rotation_deg": -math.degrees(spin.get_angle(s.pctx) - phi0), "along_mm": float(dc[0]),
            "off_face_mm": float(-dc[2]), "held": bool((m_["normal"] >= LOSS_N).all()), "lost_at_step": lost_at,
            "normal_end_n": m_["normal"].tolist(), "q_end_deg": math.degrees(m_["q"])}


def dial(hand, mu, rho_mm, contact, time_step, k_theta, d_theta, x0, q_cmd, qs, th, opts) -> dict:
    """Dial turning (task 1): the cylinder on a fixed axis (x, z held, free to turn against the bearing damping),
    gap_mm off the proximal face so that only the distal pad meets it; the same open-loop closing on every hand, from
    0 to min(O's command + sweep, 120 deg) over sweep_s, then held for hold_s. Returns the dial's rotation (deg, + as
    the flexion drives it) and the distal pad's final normal force."""
    s = Stage(hand, mu, rho_mm, contact, time_step, "xz", k_theta, d_theta)
    centre = np.array([x0[0], x0[1] - opts["gap_mm"] * 1e-3])
    s.plant.GetJointByName("slide_x").set_translation(s.pctx, centre[0])
    s.plant.GetJointByName("slide_z").set_translation(s.pctx, centre[1])
    s.ctrl.hold_at = np.array([centre[0], centre[1], 0.0])
    s.sim.Initialize()
    spin = s.plant.GetJointByName("spin")
    q1 = min(q_cmd + math.radians(opts["sweep_deg"]), math.radians(120.0))
    n_sweep, n_hold = int(round(opts["sweep_s"] / DT_ENV)), int(round(opts["hold_s"] / DT_ENV))
    touched = False
    for i in range(n_sweep + n_hold):
        q = q1 * min(1.0, (i + 1) / n_sweep)
        s.ctrl.theta_cmd = float(np.interp(q, qs, th))
        s.step()
        touched = touched or s.measure()["normal"][1] > GATE_N
    m_ = s.measure()
    return {"rotation_deg": -math.degrees(spin.get_angle(s.pctx)), "touched": bool(touched),
            "held": bool(touched), "distal_normal_end_n": float(m_["normal"][1]),
            "proximal_normal_end_n": float(m_["normal"][0]), "q_end_deg": math.degrees(m_["q"])}


def pullin(hand, mu, rho_mm, contact, time_step, k_theta, d_theta, x0, theta_cmd, opts) -> dict:
    """Grasp acquisition (task 3): the finger starts open (q = 0) with the cylinder on the proximal face, offset mm
    toward the tip of where O's distal pad meets it, under a constant load (toward the face, and toward the tip); the
    same open-loop closing to O's command on every hand (close_s), then held (hold_s). Returns how far the cylinder
    was drawn toward the MCP along the face (mm), whether both pads hold it at the end, and its final place."""
    s = Stage(hand, mu, rho_mm, contact, time_step, False, k_theta, d_theta)
    start = np.array([x0[0] + opts["offset_mm"] * 1e-3, x0[1]])
    s.plant.GetJointByName("slide_x").set_translation(s.pctx, start[0])
    s.plant.GetJointByName("slide_z").set_translation(s.pctx, start[1])
    s.ctrl.load = np.array([opts["tip_load_n"], opts["face_load_n"]]) / FINGERS
    s.sim.Initialize()
    n_close, n_hold = int(round(opts["close_s"] / DT_ENV)), int(round(opts["hold_s"] / DT_ENV))
    for i in range(n_close + n_hold):
        s.ctrl.theta_cmd = theta_cmd * min(1.0, (i + 1) / n_close)
        s.step()
    m_ = s.measure()
    c = m_["c"]
    return {"drawn_mm": float(1e3 * (start[0] - c[0])), "x_end_mm": float(1e3 * c[0]),
            "off_face_mm": float(1e3 * (start[1] - c[2])), "held": bool((m_["normal"] >= GATE_N).all()),
            "normal_end_n": m_["normal"].tolist(), "q_end_deg": math.degrees(m_["q"])}


def run_env(task):
    try:
        return _run_env(task)
    except RuntimeError:  # a solver failure: once more at half the time step (recorded)
        pass
    try:
        retry = list(task)
        retry[4] = task[4] / 2
        out = _run_env(tuple(retry))
        out["retried_time_step"] = retry[4]
        return out
    except RuntimeError as err:  # still failing: this placement only is lost
        hand, mu, rho_mm, contact, time_step, g, direction, delta = task[:8]
        return {"rho_mm": rho_mm, "q0_deg": g["q0_deg"], "x_c_mm": g["x_c_mm"], "direction": direction,
                "pre": {}, "gate0": {"pass": False, "solver_fail": str(err)[:200]},
                "limit": {"event": "solver_fail", "f_limit_n": None}}


def _run_env(task):
    hand, mu, rho_mm, contact, time_step, g, direction, delta, modulus, opts = task
    global MODULUS_PA, SPIN_DAMPING, DIAL_BRAKE_NM
    MODULUS_PA = modulus
    SPIN_DAMPING = opts.get("spin_damping", 0.0)
    DIAL_BRAKE_NM = opts.get("brake_nm", 0.0)
    global DIAL_Z_STIFFNESS
    DIAL_Z_STIFFNESS = opts.get("dial_k", 0.0)
    geom = finger_geometry(hand)
    qs, th = servo_table(geom)
    q_cmd = math.radians(g["q0_deg"] + delta)
    theta_cmd = float(np.interp(q_cmd, qs, th))
    ratio = float(np.interp(q_cmd, qs, np.gradient(th, qs)))  # dtheta/dq at the command
    k_theta, d_theta = K_Q / ratio ** 2, D_Q / ratio ** 2
    mm = 1e-3
    face = (geom["pads"]["mount_offset_mm"] + geom["pads"]["thickness_mm"]) * mm
    x0 = np.array([g["x_c_mm"] * mm, -(face + rho_mm * mm)])

    if opts.get("task") == "dial":
        return {"rho_mm": rho_mm, "q0_deg": g["q0_deg"], "x_c_mm": g["x_c_mm"], "direction": direction,
                "dial": dial(hand, mu, rho_mm, contact, time_step, k_theta, d_theta, x0, q_cmd, qs, th, opts),
                "gate0": {"pass": True}, "limit": {}}
    if opts.get("task") == "pullin":
        return {"rho_mm": rho_mm, "q0_deg": g["q0_deg"], "x_c_mm": g["x_c_mm"], "direction": direction,
                "pullin": pullin(hand, mu, rho_mm, contact, time_step, k_theta, d_theta, x0, theta_cmd, opts),
                "gate0": {"pass": True}, "limit": {}}
    # stage 1: the cylinder held, the finger closes to O's command (40 steps) and holds (30)
    s1 = Stage(hand, mu, rho_mm, contact, time_step, True, k_theta, d_theta)
    jx, jz = s1.plant.GetJointByName("slide_x"), s1.plant.GetJointByName("slide_z")
    jx.set_translation(s1.pctx, x0[0])
    jz.set_translation(s1.pctx, x0[1])
    s1.ctrl.hold_at = np.array([x0[0], x0[1], 0.0])
    s1.sim.Initialize()
    for i in range(40):
        s1.ctrl.theta_cmd = theta_cmd * (i + 1) / 40
        s1.step()
    for _ in range(30):
        s1.step()
    state = s1.plant.GetPositionsAndVelocities(s1.pctx).copy()

    # stage 2: the same finger, the cylinder free; settle 120 steps (Gate 0 over the last 10), then the load ramp
    s = Stage(hand, mu, rho_mm, contact, time_step, False, k_theta, d_theta)
    s.plant.SetPositionsAndVelocities(s.pctx, state)
    s.ctrl.theta_cmd = theta_cmd
    s.sim.Initialize()
    speed = spin = 0.0
    for k in range(120):
        s.step()
        if k >= 110:
            m_ = s.measure()
            speed = max(speed, 1e3 * float(np.linalg.norm(m_["v"][[0, 2]])))
            spin = max(spin, abs(m_["w"]))
    m_ = s.measure()
    pads_ok = bool((m_["normal"] >= GATE_N).all())
    still = speed < GATE_SPEED_MM_S and spin < GATE_SPIN_RAD_S
    ang = math.degrees(math.acos(float(np.clip(m_["n"][0] @ m_["n"][1], -1, 1)))) if pads_ok else float("nan")
    angle_ok = pads_ok and abs(ang - math.degrees(m_["q"])) <= 4.0
    pre = {"normal_n": m_["normal"].tolist(), "q_deg": math.degrees(m_["q"]), "q_cmd_deg": math.degrees(q_cmd),
           "normal_angle_deg": ang, "centre_mm": (1e3 * m_["c"][[0, 2]]).tolist(), "ratio": ratio}
    gate = {"pass": bool(pads_ok and still and angle_ok), "pads": bool(pads_ok), "still": bool(still),
            "angle": bool(angle_ok), "speed_mm_s": float(speed), "spin_rad_s": float(spin)}
    if opts.get("task") == "roll":
        return {"rho_mm": rho_mm, "q0_deg": g["q0_deg"], "x_c_mm": g["x_c_mm"], "direction": direction, "pre": pre,
                "gate0": gate, "roll": roll(s, m_, q_cmd, qs, th, opts) if gate["pass"] else None, "limit": {}}
    d = np.array(DIRECTIONS[direction])
    c0 = m_["c"].copy()
    limit = {"event": None, "f_limit_n": None, "mu_eff_geo_at_slip": None, "slip_pad": None, "roll_n": None}
    if gate["pass"]:
        steps = int(MAX_FORCE_N / (RATE_N_S * DT_ENV)) + 1
        for i in range(steps):
            f = RATE_N_S * DT_ENV * (i + 1)
            s.ctrl.load = f * d
            s.step()
            m_ = s.measure()
            disp = 1e3 * float(np.linalg.norm((m_["c"] - c0)[[0, 2]]))
            event = None
            k = int(np.argmax(m_["slip"]))
            if 1e3 * m_["slip"][k] > SLIP_MM_S:
                event = "slip"
                fn = m_["normal"][k]
                ft = float(np.linalg.norm(m_["force"][k] - fn * m_["n"][k]))
                limit["mu_eff_geo_at_slip"], limit["slip_pad"] = float(ft / max(abs(fn), 1e-9)), int(k)
            elif (m_["normal"] < LOSS_N).any():
                event = "loss"
            elif disp > COLLAPSE_MM:
                event = "collapse"
            if limit["roll_n"] is None and disp > ROLL_MM and event is None:
                limit["roll_n"] = float(f * FINGERS)
            if event:
                limit["event"], limit["f_limit_n"] = event, float(f * FINGERS)
                break
    return {"rho_mm": rho_mm, "q0_deg": g["q0_deg"], "x_c_mm": g["x_c_mm"], "direction": direction, "pre": pre,
            "gate0": gate, "limit": limit}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--hand", required=True)
    p.add_argument("--mu", type=float, required=True)
    p.add_argument("--radius_mm", type=float, default=15.0)
    p.add_argument("--contact", choices=("point", "hydro"), default="point")
    p.add_argument("--time_step", type=float, default=1.0e-3)
    p.add_argument("--delta_deg", type=float, default=5.0)
    p.add_argument("--only", type=int, default=None, help="first N placements")
    p.add_argument("--directions", nargs="+", default=list(DIRECTIONS))
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--task", choices=("limit", "roll", "pullin", "dial"), default="limit",
                   help="limit: the load ramp (Study 3); roll: in-hand rolling by an open-loop flexion sweep")
    p.add_argument("--sweep_deg", type=float, default=15.0)
    p.add_argument("--sweep_s", type=float, default=3.0)
    p.add_argument("--hold_s", type=float, default=1.0)
    p.add_argument("--spin_damping", type=float, default=0.0, help="N m s/rad on the cylinder's spin")
    p.add_argument("--offset_mm", type=float, default=5.0, help="pullin: start this far toward the tip")
    p.add_argument("--tip_load_n", type=float, default=0.5, help="pullin: load toward the tip (three fingers)")
    p.add_argument("--face_load_n", type=float, default=0.5, help="pullin: load toward the face (three fingers)")
    p.add_argument("--close_s", type=float, default=2.0)
    p.add_argument("--gap_mm", type=float, default=1.0, help="dial: its axis this far off the proximal face")
    p.add_argument("--brake_nm", type=float, default=0.0, help="dial: its brake torque (one finger)")
    p.add_argument("--dial_k", type=float, default=0.0, help="dial: its axis's spring away from the face [N/m]")
    p.add_argument("--modulus", type=float, default=MODULUS_PA, help="hydroelastic pads: modulus [Pa] (stiffness / 3 mm)")
    p.add_argument("--tag", default="")
    args = p.parse_args()
    grasps = [g for g in json.loads(GRASPS.read_text())["grasps"] if g["rho_mm"] == args.radius_mm][: args.only]
    opts = {"task": args.task, "sweep_deg": args.sweep_deg, "sweep_s": args.sweep_s, "hold_s": args.hold_s,
            "spin_damping": args.spin_damping, "offset_mm": args.offset_mm, "tip_load_n": args.tip_load_n,
            "face_load_n": args.face_load_n, "close_s": args.close_s, "gap_mm": args.gap_mm,
            "brake_nm": args.brake_nm, "dial_k": args.dial_k}
    directions = ["none"] if args.task in ("roll", "pullin", "dial") else args.directions
    tasks = [(args.hand, args.mu, args.radius_mm, args.contact, args.time_step, g, d, args.delta_deg, args.modulus, opts)
             for g in grasps for d in directions]
    t0 = time.time()
    with Pool(args.workers) as pool:
        envs = pool.map(run_env, tasks)
    ok = [e for e in envs if e["gate0"]["pass"]]
    print(f"{args.hand} mu {args.mu} r {args.radius_mm} {args.contact}: Gate 0 passed {len(ok)}/{len(envs)} "
          f"in {time.time() - t0:.0f} s")
    OUT.mkdir(exist_ok=True)
    stem = "dk" if args.task == "limit" else f"dk{args.task}"
    name = f"{stem}_{args.hand}_mu{args.mu:g}_{args.contact}{('-' + args.tag) if args.tag else ''}_r{args.radius_mm:g}.json"
    (OUT / name).write_text(json.dumps({"hand": args.hand, "mu": args.mu, "contact": args.contact,
                                        "time_step": args.time_step, "delta_deg": args.delta_deg,
                                        "modulus_pa": args.modulus if args.contact == "hydro" else None, "opts": opts,
                                        "envs": envs}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
