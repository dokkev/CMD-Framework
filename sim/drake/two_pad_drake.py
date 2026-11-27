"""Study 3's two-pad grasp measurement (condition A) replicated in Drake, with point or hydroelastic contact.

Isaac Lab's hydroelastic path could not be used (report/study3/study3.md, sixteenth experiment, review): its
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
collapse (the cylinder moves 5 mm). Contact: only the two pads and the cylinder collide. Experiment 27
(--setpoint contact_ref) keeps all of this but gives each hand its own flexion setpoint: the flexion at which its
distal-face point below the fingertip lands where O's lands at O's setpoint (contact_mid: the distal pad's middle).

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
# experiment 28, the fingertip dial (task 'tipdial'): 'y' a dial turning in the finger's plane (axis along y) on an axle
# sprung along O's distal-pad normal at the touch angle; 'z' a disk with its axis along z (normal to the proximal face),
# offset sideways in y, pressed face-on by the fingertip; both turn against a torsional spring
TIP_MODE = None  # None, 'y' or 'z'
TIP_AXIS_DEG = 0.0  # 'y': the axle's spring axis is O's distal-pad normal at this flexion
TIP_Y_MM, TIP_THICK_MM = 0.0, 4.0  # 'z': the disk's sideways offset and thickness
TIP_KR, TIP_DR = 0.0, 0.0  # the dial's torsional spring (N m/rad) and damping (N m s/rad)


def hand_file(hand: str) -> Path:
    stem = {"O": "h3to7_h0", "R": "h3to7_hr", "U": "h3to7_hu"}.get(hand)
    if stem is None:
        stem = hand if hand.startswith("h3to") else f"h3to7_{hand}"
    return HAND_DIR / f"cfl_q120_{stem}.yaml"


def finger_geometry(hand: str) -> dict:
    """Pivots at q = 0 (mm, x along, z dorsal), link lengths and pads, from the hand's parameter file."""
    d = yaml.safe_load(hand_file(hand).read_text())
    f, p = d["finger"], dict(d["pads"])
    a = np.array([f["a_x_mm"], f["a_z_mm"]])
    ap = np.array([f["ap_x_mm"], f["ap_z_mm"]])
    design = d.get("source", {}).get("design", [])
    if f["a_z_mm"] == 0.0 and (len(design) < 3 or design[2] == 0.0):  # the studies' hands: A and B on the link line
        b = np.array([ap[0] + math.sqrt(f["apb_mm"] ** 2 - ap[1] ** 2), 0.0])  # on the link line, distal of A'
        p["distal_mount_mm"] = p["mount_offset_mm"]  # both pads at mount_offset_mm, as in Study 3
    else:  # off the link line (pr_mr.md's MR hands): B by the CoDe hand model's closure, each pad at its own mount
        b = _closed_b(a, ap, np.array([f["bp_x_mm"], f["bp_z_mm"]]), f["apb_mm"], f["abp_mm"])
        p["distal_mount_mm"] = p.get("distal_mount_offset_mm", p["mount_offset_mm"])
    p["distal_face_mm"] = p["distal_mount_mm"] + p["thickness_mm"]
    bp = b + np.array([f["bp_x_mm"], f["bp_z_mm"]])
    return {"a": a, "ap": ap, "b": b, "bp_local": np.array([f["bp_x_mm"], f["bp_z_mm"]]), "abp": f["abp_mm"],
            "abp_check": float(np.linalg.norm(bp - a)), "pads": p}


def face_depth(hand: str) -> float:
    """The proximal contact face's depth below the link line (mm)."""
    pads = finger_geometry(hand)["pads"]
    return pads["mount_offset_mm"] + pads["thickness_mm"]


def _closed_b(a, ap, bp_local, apb, abp) -> np.ndarray:
    """B at q = 0 as design/core/morphology/code_hand/model.py's _reference_b: on the circle of radius |A'B| about A'
    and of radius |AB'| about A - B'_local, the branch counter-clockwise of A' -> that centre."""
    centre = a - bp_local
    u = centre - ap
    dist = float(np.linalg.norm(u))
    along = (apb ** 2 - abp ** 2 + dist ** 2) / (2.0 * dist)
    h = math.sqrt(max(apb ** 2 - along ** 2, 0.0))
    mid, perp = ap + along * u / dist, np.array([-u[1], u[0]]) / dist
    return max((mid + h * perp, mid - h * perp), key=lambda q: u[0] * (q - ap)[1] - u[1] * (q - ap)[0])


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


def _closure_gap(g: dict, theta: float, q: float) -> float:
    b = g["ap"] + rot(theta) @ (g["b"] - g["ap"])
    return float(np.linalg.norm(b + rot(q) @ g["bp_local"] - g["a"]) - g["abp"])


def reference_point(g: dict, qs: np.ndarray, th: np.ndarray, q: float, where: str | float = "tip") -> np.ndarray:
    """A point of the distal contact face at flexion q (rad), in the proximal face's frame (mm: origin at the foot of
    the MCP axis on the proximal face, x along it, z dorsal): 'tip' is the point below the fingertip (distal frame
    (distal_end, -face), the point Delta_contact follows), 'mid' the distal pad's middle. The four-bar is closed exactly
    at q, from the servo table's driver angle."""
    from scipy.optimize import brentq

    t0 = float(np.interp(q, qs, th))
    lo, hi = t0 - 0.01, t0 + 0.01
    while _closure_gap(g, lo, q) * _closure_gap(g, hi, q) > 0:
        lo, hi = lo - 0.01, hi + 0.01
    theta = brentq(lambda t: _closure_gap(g, t, q), lo, hi, xtol=1e-13)
    pads = g["pads"]
    face = pads["mount_offset_mm"] + pads["thickness_mm"]
    if isinstance(where, (int, float)):  # a distal-frame x along the pad (experiment 28's placements)
        x = float(where)
    else:
        x = pads["distal_end_mm"] if where == "tip" else (pads["distal_start_mm"] + pads["distal_end_mm"]) / 2
    b = g["ap"] + rot(theta) @ (g["b"] - g["ap"])
    return b + rot(q) @ np.array([x, -pads["distal_face_mm"]]) + np.array([0.0, face])


def contact_setpoint(hand: str, q_o_deg: float, where: str = "tip", q_max_deg: float = 125.0) -> dict:
    """Experiment 27's setpoint: the flexion at which the hand's reference point lands where O's lands at O's setpoint
    q_o, argmin_q |c_H(q) - c_O(q_o)| over the servo table's range, from kinematics only. Returns the setpoint (deg),
    the residual (mm) and whether the minimum sits on the range's bound."""
    from scipy.optimize import minimize_scalar

    g_o = finger_geometry("O")
    target = reference_point(g_o, *servo_table(g_o, q_max_deg), math.radians(q_o_deg), where)
    g = finger_geometry(hand)
    qs, th = servo_table(g, q_max_deg)
    dist = lambda q: float(np.linalg.norm(reference_point(g, qs, th, q, where) - target))
    grid = np.radians(np.arange(0.0, q_max_deg + 0.25, 0.5))
    k = int(np.argmin([dist(q) for q in grid]))
    lo, hi = grid[max(k - 1, 0)], grid[min(k + 1, len(grid) - 1)]
    res = minimize_scalar(dist, bounds=(lo, hi), method="bounded", options={"xatol": 1e-9})
    q = float(res.x)
    return {"where": where, "q_cmd_o_deg": q_o_deg, "q_cmd_deg": math.degrees(q), "residual_mm": dist(q),
            "at_bound": bool(q <= 1e-6 or math.degrees(q) >= q_max_deg - 1e-4),
            "target_mm": target.tolist()}


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
    tip = hold == "dial" and TIP_MODE in ("y", "z")
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
    zdisk = tip and TIP_MODE == "z"
    spin_axis = np.array([0.0, 0.0, 1.0]) if zdisk else y
    I_cyl = UnitInertia.SolidCylinder(rho, TIP_THICK_MM * mm if zdisk else LENGTH_M, spin_axis)
    roller = plant.AddRigidBody("roller", SpatialInertia(MASS_KG, np.zeros(3), I_cyl))
    face = m + h  # proximal face depth below the link line
    ax_x, ax_z, base = [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], world.body_frame()
    if tip and TIP_MODE == "y":  # slide_x along O's distal pad at the touch angle, slide_z (the spring) along -normal
        t_ = math.radians(TIP_AXIS_DEG)
        ax_x, ax_z = [math.cos(t_), 0.0, -math.sin(t_)], [math.sin(t_), 0.0, math.cos(t_)]
    if zdisk:  # the disk's axle sits TIP_Y_MM to the side of the finger's plane
        base = plant.AddFrame(FixedOffsetFrame("dial_base", world.body_frame(),
                                               RigidTransform([0.0, TIP_Y_MM * mm, 0.0])))
    joints = [plant.AddJoint(PrismaticJoint("slide_x", base, cx.body_frame(), ax_x)),
              plant.AddJoint(PrismaticJoint("slide_z", cx.body_frame(), cz.body_frame(), ax_z)),
              plant.AddJoint(RevoluteJoint("spin", cz.body_frame(), roller.body_frame(), spin_axis,
                                           damping=SPIN_DAMPING))]
    if tip:  # experiment 28: the axle held stiffly across, sprung along, the dial against a torsional spring
        for j, (kp, kd) in zip(joints, ((1.0e5, 1.0e3), (DIAL_Z_STIFFNESS, 5.0), (TIP_KR, TIP_DR))):
            plant.AddJointActuator(j.name(), j).set_controller_gains(PdControllerGains(p=kp, d=kd))
    elif hold:  # the drives that hold the cylinder while the finger closes (implicit PD; a second plant releases it)
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
    md = pads["distal_mount_mm"] * mm  # the distal pad's mount offset (m for the studies' hands)
    plant.RegisterCollisionGeometry(distal, RigidTransform(at(((t0 + t1) / 2, -(md + h / 2)))), Box(t1 - t0, w, h),
                                    "distal_pad", props("pad", h))
    if zdisk:  # Drake's cylinder axis is its frame's z: the disk as it is
        plant.RegisterCollisionGeometry(roller, RigidTransform(), Cylinder(rho, TIP_THICK_MM * mm), "roller",
                                        props("roller"))
    else:
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
    strokes = int(opts.get("strokes", 1))
    per_stroke = []
    for k in range(strokes):  # each stroke closes 0 -> q1 and holds; all but the last then open back to 0
        for i in range(n_sweep + n_hold):
            q = q1 * min(1.0, (i + 1) / n_sweep)
            s.ctrl.theta_cmd = float(np.interp(q, qs, th))
            s.step()
            touched = touched or s.measure()["normal"][1] > GATE_N
        per_stroke.append(-math.degrees(spin.get_angle(s.pctx)))
        if k + 1 < strokes:
            for i in range(n_sweep):
                q = q1 * max(0.0, 1.0 - (i + 1) / n_sweep)
                s.ctrl.theta_cmd = float(np.interp(q, qs, th))
                s.step()
            per_stroke.append(-math.degrees(spin.get_angle(s.pctx)))
    m_ = s.measure()
    return {"rotation_deg": -math.degrees(spin.get_angle(s.pctx)), "touched": bool(touched), "strokes": strokes,
            "rotation_after_deg": per_stroke,
            "held": bool(touched), "distal_normal_end_n": float(m_["normal"][1]),
            "proximal_normal_end_n": float(m_["normal"][0]), "q_end_deg": math.degrees(m_["q"])}


def _rect_gap(px: float, pz: float, x0: float, x1: float, z0: float, z1: float) -> float:
    """Distance (>= 0) from a point to an axis-aligned rectangle."""
    return math.hypot(max(x0 - px, 0.0, px - x1), max(z0 - pz, 0.0, pz - z1))


def tipdial(hand, mu, rho_mm, contact, time_step, k_theta, d_theta, g, q_start, qs, th, opts) -> dict:
    """The fingertip dial (task 'tipdial', experiment 28). The dial's axle is fixed in the hand's proximal-face frame
    at the placement's (x_c, z_c) (mm; z dorsal, so below the face is negative), the same coordinates on every hand;
    only the distal pad is meant to reach it. 'y': a dial in the finger's plane (axis y) on an axle sprung along O's
    distal-pad normal at the touch angle; 'z': a disk with its axis along z, TIP_Y_MM to the side, its top face at z_c +
    thickness / 2, sprung along z. Both turn against a torsional spring. The same open-loop flexion on every hand: 0 ->
    q_start over approach_s, q_start -> min(q_start + sweep, 120 deg) over sweep_s, held for hold_s. Returns the dial's
    rotation (deg), whether and at which flexion the distal pad first touched it, the largest proximal-pad force and the
    smallest proximal-pad clearance (mm, geometric) over the run."""
    s = Stage(hand, mu, rho_mm, contact, time_step, "dial", k_theta, d_theta)
    geom = finger_geometry(hand)
    pads = geom["pads"]
    m_mm = pads["mount_offset_mm"]
    face = m_mm + pads["thickness_mm"]
    # the placement's frame: the hand's own proximal face (anchor 'face', the paper's runs), or O's, i.e. O's coordinates
    # in F_1 (the link-line frame, the world here, the MCP held) on every hand (anchor 'f1'; docs/experimental/pr_mr.md)
    at = face if opts.get("anchor", "face") == "face" else face_depth("O")
    c = np.array([g["x_c_mm"], g["z_c_mm"] - at]) * 1e-3  # world (x, z), m
    if TIP_MODE == "y":
        t_ = math.radians(TIP_AXIS_DEG)
        hold = [float(c @ [math.cos(t_), -math.sin(t_)]), float(c @ [math.sin(t_), math.cos(t_)])]
    else:
        hold = [float(c[0]), float(c[1])]
    s.plant.GetJointByName("slide_x").set_translation(s.pctx, hold[0])
    s.plant.GetJointByName("slide_z").set_translation(s.pctx, hold[1])
    s.ctrl.hold_at = np.array([hold[0], hold[1], 0.0])
    s.sim.Initialize()
    spin = s.plant.GetJointByName("spin")
    q1 = min(q_start + math.radians(opts["sweep_deg"]), math.radians(120.0))
    n_app, n_sweep, n_hold = (int(round(opts[k] / DT_ENV)) for k in ("approach_s", "sweep_s", "hold_s"))
    touch_q, prox_f, clear, disp, dist_f_max = None, 0.0, float("inf"), 0.0, 0.0
    trace = []
    s0, s1 = pads["proximal_start_mm"], pads["proximal_end_mm"]
    half_w = pads["width_mm"] / 2
    for i in range(n_app + n_sweep + n_hold):
        if i < n_app:
            q = q_start * (i + 1) / n_app
        else:
            q = q_start + (q1 - q_start) * min(1.0, (i + 1 - n_app) / n_sweep)
        s.ctrl.theta_cmd = float(np.interp(q, qs, th))
        s.step()
        force, _ = s.contacts()
        fd, fp = float(np.linalg.norm(force[1])), float(np.linalg.norm(force[0]))
        dist_f_max, prox_f = max(dist_f_max, fd), max(prox_f, fp)
        pos = 1e3 * s.plant.EvalBodyPoseInWorld(s.pctx, s.bodies["roller"]).translation()
        disp = max(disp, float(np.linalg.norm(pos[[0, 2]] - 1e3 * c)))
        if TIP_MODE == "y":  # the dial's rim to the proximal pad (a rectangle in x, z)
            gap = _rect_gap(pos[0], pos[2], s0, s1, -face, -m_mm) - rho_mm
        else:  # the disk (a cylinder along z) to the proximal pad (a box)
            lat = math.hypot(max(s0 - pos[0], 0.0, pos[0] - s1), max(-half_w - pos[1], 0.0, pos[1] - half_w))
            top, bottom = pos[2] + TIP_THICK_MM / 2, pos[2] - TIP_THICK_MM / 2
            vert = max(-face - top, 0.0, bottom - (-m_mm))
            gap = math.hypot(max(lat - rho_mm, 0.0), vert)
        clear = min(clear, gap)
        if touch_q is None and fd > GATE_N:
            touch_q = math.degrees(s.measure()["q"])
        if (i + 1) % (1 if opts.get("curve") else 15) == 0:  # --curve: every step
            trace.append([round(math.degrees(q), 2), round(-math.degrees(spin.get_angle(s.pctx)), 3), round(fd, 4)])
    m_ = s.measure()
    force, _ = s.contacts()
    return {"rotation_deg": -math.degrees(spin.get_angle(s.pctx)), "touched": touch_q is not None,
            "held": touch_q is not None, "touch_q_deg": touch_q, "q_start_deg": math.degrees(q_start),
            "q_end_cmd_deg": math.degrees(q1), "q_end_deg": math.degrees(m_["q"]),
            "distal_force_end_n": float(np.linalg.norm(force[1])), "distal_force_max_n": dist_f_max,
            "proximal_force_max_n": prox_f, "proximal_clearance_min_mm": clear, "axle_disp_max_mm": disp,
            "trace": trace}


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
    trace = []  # --curve: [time s, drawn toward the MCP mm] every step (Fig. 4)
    for i in range(n_close + n_hold):
        s.ctrl.theta_cmd = theta_cmd * min(1.0, (i + 1) / n_close)
        s.step()
        if opts.get("curve"):
            trace.append([round((i + 1) * DT_ENV, 4), round(float(1e3 * (start[0] - s.measure()["c"][0])), 4)])
    m_ = s.measure()
    c = m_["c"]
    return {"drawn_mm": float(1e3 * (start[0] - c[0])), "x_end_mm": float(1e3 * c[0]),
            "off_face_mm": float(1e3 * (start[1] - c[2])), "held": bool((m_["normal"] >= GATE_N).all()),
            "normal_end_n": m_["normal"].tolist(), "q_end_deg": math.degrees(m_["q"]),
            **({"trace": trace} if opts.get("curve") else {})}


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
    global MODULUS_PA, SPIN_DAMPING, DIAL_BRAKE_NM, PAD_REFERENCE_M
    MODULUS_PA = modulus
    if opts.get("pad_kh_mm"):  # experiment 25: the foundation stiffness of a pad this thick (own: the hand's own)
        PAD_REFERENCE_M = (finger_geometry(hand)["pads"]["thickness_mm"] if opts["pad_kh_mm"] == "own"
                           else float(opts["pad_kh_mm"])) * 1e-3
    SPIN_DAMPING = opts.get("spin_damping", 0.0)
    DIAL_BRAKE_NM = opts.get("brake_nm", 0.0)
    global DIAL_Z_STIFFNESS, TIP_MODE, TIP_AXIS_DEG, TIP_Y_MM, TIP_THICK_MM, TIP_KR, TIP_DR
    DIAL_Z_STIFFNESS = opts.get("dial_k", 0.0)
    if opts.get("task") == "tipdial":
        TIP_MODE, TIP_THICK_MM = opts["tip_mode"], opts["tip_thick_mm"]
        TIP_KR, TIP_DR = g.get("tip_kr", opts["tip_kr"]), opts["tip_dr"]  # experiment 29: a spring per draw
        TIP_AXIS_DEG, TIP_Y_MM = g.get("axis_deg", 0.0), g.get("y_mm", 0.0)
    geom = finger_geometry(hand)
    qs, th = servo_table(geom)
    q_cmd = math.radians(g["q0_deg"] + delta)
    setpoint = None
    if opts.get("setpoint", "shared") != "shared":  # experiment 27: contact-referenced, 'contact_ref' or 'contact_mid'
        setpoint = contact_setpoint(hand, g["q0_deg"] + delta, "mid" if opts["setpoint"] == "contact_mid" else "tip")
        q_cmd = math.radians(setpoint["q_cmd_deg"])
    theta_cmd = float(np.interp(q_cmd, qs, th))
    ratio = float(np.interp(q_cmd, qs, np.gradient(th, qs)))  # dtheta/dq at the command
    k_theta, d_theta = K_Q / ratio ** 2, D_Q / ratio ** 2
    mm = 1e-3
    face = (geom["pads"]["mount_offset_mm"] + geom["pads"]["thickness_mm"]) * mm
    x0 = np.array([g["x_c_mm"] * mm, -(face + rho_mm * mm)])

    if opts.get("task") == "tipdial":  # starts at O's touch angle (+ delta: 0, or 0.05 for the noise floor)
        return {"rho_mm": rho_mm, "q0_deg": g["q0_deg"], "x_c_mm": g["x_c_mm"], "direction": direction,
                "placement": g,
                "dial": tipdial(hand, mu, rho_mm, contact, time_step, k_theta, d_theta, g, q_cmd, qs, th, opts),
                "gate0": {"pass": True}, "limit": {}}
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
    curve = []  # load-displacement: [load N (three fingers), cylinder displacement along the load mm, normals N]
    if gate["pass"]:
        steps = int(MAX_FORCE_N / (RATE_N_S * DT_ENV)) + 1
        for i in range(steps):
            f = RATE_N_S * DT_ENV * (i + 1)
            s.ctrl.load = f * d
            s.step()
            m_ = s.measure()
            disp = 1e3 * float(np.linalg.norm((m_["c"] - c0)[[0, 2]]))
            if opts.get("curve"):
                curve.append([round(f * FINGERS, 4), round(1e3 * float((m_["c"] - c0)[[0, 2]] @ d), 4),
                              round(disp, 4), [round(float(x), 4) for x in m_["normal"]]])
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
            "gate0": gate, "limit": limit, "setpoint": setpoint, **({"curve": curve} if opts.get("curve") else {})}


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
    p.add_argument("--task", choices=("limit", "roll", "pullin", "dial", "tipdial"), default="limit",
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
    p.add_argument("--grasps", type=Path, default=None, help="placements file (default: the Study 3 grasps)")
    p.add_argument("--strokes", type=int, default=1, help="dial: close-and-open strokes (the last one is not opened)")
    p.add_argument("--brake_nm", type=float, default=0.0, help="dial: its brake torque (one finger)")
    p.add_argument("--dial_k", type=float, default=0.0, help="dial: its axis's spring away from the face [N/m]")
    p.add_argument("--pad_kh_mm", default=None,
                   help="hydroelastic pads' foundation stiffness as modulus / this thickness (mm, or 'own': the hand's "
                        "own pad thickness); default 3 mm on every hand (experiment 21)")
    p.add_argument("--modulus", type=float, default=MODULUS_PA, help="hydroelastic pads: modulus [Pa] (stiffness / 3 mm)")
    p.add_argument("--setpoint", choices=("shared", "contact_ref", "contact_mid"), default="shared",
                   help="shared: O's flexion setpoint on every hand (condition A); contact_ref / contact_mid: each "
                        "hand's flexion that puts its distal-face point below the fingertip / at the pad's middle where "
                        "O's lands at O's setpoint (experiment 27)")
    p.add_argument("--map_only", action="store_true", help="print and save the setpoint mapping only, no simulation")
    p.add_argument("--tip_mode", choices=("y", "z"), default="y", help="tipdial: the dial's axis (experiment 28)")
    p.add_argument("--tip_kr", type=float, default=0.01, help="tipdial: torsional spring [N m/rad] (one finger)")
    p.add_argument("--tip_dr", type=float, default=1.0e-4, help="tipdial: torsional damping [N m s/rad]")
    p.add_argument("--tip_thick_mm", type=float, default=4.0, help="tipdial z: the disk's thickness")
    p.add_argument("--approach_s", type=float, default=1.0, help="tipdial: 0 -> touch angle over this time")
    p.add_argument("--any_radius", action="store_true",
                   help="every placement at its own rho_mm (experiment 29's random draws); the file is named _rany")
    p.add_argument("--curve", action="store_true", help="limit: also record the load-displacement curve")
    p.add_argument("--anchor", choices=("face", "f1"), default="face",
                   help="tipdial: place the dial in the hand's proximal-face frame (face) or at O's coordinates in F_1 (f1)")
    p.add_argument("--tag", default="")
    args = p.parse_args()
    grasps = [g for g in json.loads((args.grasps or GRASPS).read_text())["grasps"]
              if args.any_radius or g["rho_mm"] == args.radius_mm][: args.only]
    opts = {"task": args.task, "sweep_deg": args.sweep_deg, "sweep_s": args.sweep_s, "hold_s": args.hold_s,
            "spin_damping": args.spin_damping, "offset_mm": args.offset_mm, "tip_load_n": args.tip_load_n,
            "face_load_n": args.face_load_n, "close_s": args.close_s, "gap_mm": args.gap_mm,
            "brake_nm": args.brake_nm, "dial_k": args.dial_k, "pad_kh_mm": args.pad_kh_mm, "strokes": args.strokes,
            "setpoint": args.setpoint, "tip_mode": args.tip_mode, "tip_kr": args.tip_kr, "tip_dr": args.tip_dr,
            "tip_thick_mm": args.tip_thick_mm, "approach_s": args.approach_s, "curve": args.curve}
    if args.anchor != "face":  # only then recorded, so the face-anchored logs' opts stay as they were
        opts["anchor"] = args.anchor
    if args.setpoint != "shared":
        where = "mid" if args.setpoint == "contact_mid" else "tip"
        rows = [contact_setpoint(args.hand, q, where) for q in sorted({g["q0_deg"] + args.delta_deg for g in grasps})]
        print(f"{args.hand} {args.setpoint}: q_cmd,O -> q_cmd,H (shift) residual")
        for r in rows:
            print(f"  {r['q_cmd_o_deg']:6.2f} -> {r['q_cmd_deg']:7.3f} deg ({r['q_cmd_deg'] - r['q_cmd_o_deg']:+6.2f}) "
                  f"{r['residual_mm']:6.3f} mm{'  AT BOUND' if r['at_bound'] else ''}")
        OUT.mkdir(exist_ok=True)
        (OUT / f"setpoint_{args.setpoint}_{args.hand}_r{args.radius_mm:g}_d{args.delta_deg:g}.json").write_text(
            json.dumps(rows, indent=1))
        if args.map_only:
            return 0
    directions = ["none"] if args.task in ("roll", "pullin", "dial", "tipdial") else args.directions
    tasks = [(args.hand, args.mu, g["rho_mm"] if args.any_radius else args.radius_mm, args.contact, args.time_step, g,
              d, args.delta_deg, args.modulus, opts) for g in grasps for d in directions]
    t0 = time.time()
    with Pool(args.workers) as pool:
        envs = pool.map(run_env, tasks)
    ok = [e for e in envs if e["gate0"]["pass"]]
    r_name = "any" if args.any_radius else f"{args.radius_mm:g}"
    print(f"{args.hand} mu {args.mu} r {'any' if args.any_radius else args.radius_mm} {args.contact}: Gate 0 passed "
          f"{len(ok)}/{len(envs)} in {time.time() - t0:.0f} s")
    OUT.mkdir(exist_ok=True)
    stem = "dk" if args.task == "limit" else f"dk{args.task}"
    name = f"{stem}_{args.hand}_mu{args.mu:g}_{args.contact}{('-' + args.tag) if args.tag else ''}_r{r_name}.json"
    (OUT / name).write_text(json.dumps({"hand": args.hand, "mu": args.mu, "contact": args.contact,
                                        "time_step": args.time_step, "delta_deg": args.delta_deg,
                                        "modulus_pa": args.modulus if args.contact == "hydro" else None, "opts": opts,
                                        "envs": envs}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
