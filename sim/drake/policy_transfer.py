"""Sim-to-sim transfer: run the finger-walk (7b') and relay-dial policies, trained in Isaac Lab (Newton, MuJoCo-Warp)
on the original hand O, unchanged in Drake, on O and on the redesigns (docs/experimental/pr_mr.md).

The Drake environment reproduces the Isaac Lab task (``sim/isaaclab/src/code_isaaclab/tasks/finger_walk``,
``.../finger_relay``) term by term:

- **Hand.** The plain URDF of the hand (``sim/isaaclab/logs/urdf/<hand>_thumbpad.urdf``, written by
  ``scripts/code_hand/make_assets.py --geometry --thumb-pad --no-usd``), palm up at 0.5 m, the four-bar loops closed by
  ball constraints at the description's cut pins (as ``load_hand.py``), gravity on. Every servo carries the XM335
  armature (0.0031 kg m^2, Isaac Lab's ``xm335_actuator_cfg``). The held servos (MCP, abduction, thumb, a non-acting
  PIP) are implicit PD at the XM335 gains; the acting PIP servos get the DC-motor law Isaac Lab applies: torque
  ``K_q/r^2 (theta_d - theta) - D_q/r^2 theta_dot`` (``K_q`` 5 N m/rad, ``D_q`` 0.08 N m s/rad, ``r = dtheta/dq`` from
  the hand's four-bar table, rescheduled once per control step as ``FlexionEMAAction``), clipped to the torque-speed
  envelope (stall 1.03 N m, no-load 5.55 rad/s).
- **Contact.** Point contact (SAP). Each hand collider and the object get a contact stiffness of 1.3e4 N/m (6.5 kN/m in
  series, the ``pad13_condim4_v1`` model's effective stiffness); friction: pads 1.3 and object 1.3 (pad-object 1.3),
  other hand colliders 0.5 (hand-object 0.72 under Drake's combination; MuJoCo takes the larger, 1.3 and 0.5). The
  pads are the URDF's flat boxes; Isaac Lab's are domed (1 mm crown) and condim 4 with torsional friction: a known
  difference.
- **Control.** 30 Hz (``dt`` 1/120 x decimation 4), the plant at 1/1200 s. The action (3 or 2 PIP flexions in
  [-1, 1]) is clipped, mapped to 0-120 deg, smoothed by the EMA (alpha 0.5) from the last command, and sent as the
  servo angle theta(q).
- **Observations** (policy group, history 5, term-major, the first observation repeated at reset): the last raw action,
  the acting PIP flexions normalized over 0-120 deg, the task term (walk: the cylinder's face-frame x and height [cm],
  x rate [m/s, clip 1], target error [cm, clip 10]; relay: sin, cos of the dial angle since the start, goal error over
  pi, rate over 10), and the touch term (per distal body: contact force with the object, gain error +-10%, clipped at
  5 N and normalized, and the 0.05 N flag flipped with probability 0.05).
- **Actor.** The rsl_rl actor (MLP 256-128-64, ELU, the empirical observation normalizer with eps 0.01), exported to
  numpy (``export``, run in the Isaac Lab environment); the deterministic action is the mean.
- **Episode and score.** As ``eval_walk.py`` / ``eval_relay.py``: one 10 s episode from a random rest start (each acting
  PIP at an integer flexion 0-25 deg), 299 control steps; walk: the net distance walked toward the alternating targets
  (20 and 45 mm along the face), kept up to a drop; relay: the dial's rotation since the start [deg].

    cd sim/isaaclab && uv run python ../drake/policy_transfer.py export          # actors -> sim/drake/logs/policy_transfer
    cd sim/drake && uv run python policy_transfer.py run --task walk --hands O MRM SK U --episodes 24 --tag pt1
    cd sim/drake && uv run python policy_transfer.py check --task walk --hand O   # zero / constant action sanity runs

Writes ``logs/policy_transfer/<task>_<hand>_s<seed>_<tag>.json`` (per-episode scores) and prints the ratios to O.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
ISAAC = REPO / "sim" / "isaaclab"
OUT = HERE / "logs" / "policy_transfer"
HANDS = {"O": "h0", "R": "hr", "U": "hu", "MRM": "mrmid_free", "SK": "conv_e25"}
"""Isaac keys -> description stems (``cfl_q120_h3to7_<stem>``): CoDe = MO-Pad (MRM), Skeleton (SK), Coverage (U)."""
CKPT = {
    ("walk", 20): "code_finger_walk/2026-11-26_03-30-35_finger_walk_O_s20_walk_seg3/model_597.pt",
    ("walk", 21): "code_finger_walk/2026-11-26_05-23-33_finger_walk_O_s21_walks21_seg3/model_597.pt",
    ("relay", 20): "code_finger_relay/2026-11-26_01-50-50_finger_relay_O_s20_relay_seg2/model_398.pt",
    ("relay", 21): "code_finger_relay/2026-11-26_14-20-04_finger_relay_O_s21_relays21_seg2/model_398.pt",
}
HAND_ROOT = np.array([0.0, 0.0, 0.5])
ROOT_R = np.diag([1.0, -1.0, -1.0])  # palm up: half a turn about x
DT_ENV, SUB = 4.0 / 120.0, 40
STEPS = 299
K_Q, D_Q = 5.0, 0.08
STALL_NM, NO_LOAD = 1.03, 5.550147021341967
ARMATURE = 0.0031
K_CONTACT = 1.3e4
CONTACT = "hydro"
"""The contact model (``--contact``): hydro (compliant pads, rigid object, point fallback) or point."""
HYDRO_K = 1.0e5 / 0.003
"""Hydroelastic pads (the default contact): pressure per depth [Pa/m], the Drake study's 0.1 MPa over 3 mm
(two_pad_drake.MODULUS_PA / PAD_REFERENCE_M); the object is rigid, the other hand colliders fall back to point contact."""
PAD_MU, HAND_MU, OBJ_MU = 1.3, 0.5, 1.3
Q_LO, Q_HI = 0.0, math.radians(120.0)
TOUCH = {"max_force_n": 5.0, "threshold_n": 0.05, "force_noise": 0.10, "flip_prob": 0.05}
TASK = {
    "walk": {"acting": ("index", "middle", "ring"), "start_max_deg": 25},
    "relay": {"acting": ("index", "middle"), "start_max_deg": 25},
}
# walk (finger_walk/env_cfg.py)
W_RADIUS, W_LENGTH, W_MASS = 0.015, 0.070, 0.030
W_START_X, W_TARGETS = (0.028, 0.042), (0.020, 0.045)
W_SUCCESS, W_HOLD = 0.004, 3
W_X_MAX, W_Z_MIN, W_Y_MAX = 0.090, -0.010, 0.040
# relay (finger_relay/env_cfg.py)
R_QTOUCH, R_XBACK, R_YOUT, R_RADIUS, R_THICK = 30.0, 0.010, 0.012, 0.045, 0.004
R_MASS, R_SLIDE, R_KSLIDE, R_DSLIDE, R_KSPIN, R_DSPIN = 0.03, 0.025, 60.0, 1.0, 0.002, 0.001
R_GOAL, R_SUCCESS, R_HOLD = math.radians(30.0), math.radians(8.0), 3


# ------------------------------------------------------------------------------------------------ hand description
def description(hand: str) -> dict:
    return json.loads((REPO / "design" / "output" / "sim" / "study3" / "hands" /
                       f"cfl_q120_h3to7_{HANDS[hand]}.json").read_text())


def urdf(hand: str) -> Path:
    return ISAAC / "logs" / "urdf" / f"cfl_q120_h3to7_{HANDS[hand]}_thumbpad.urdf"


def closed_pose(desc: dict, digit: str, q_deg: float) -> dict:
    """The four-bar's tree joint angles [rad] at flexion ``q_deg`` (the description's table, ``Description.closed_pose``)."""
    t = desc["four_bar"][digit]
    return {f"{digit}_{k}": math.radians(float(np.interp(q_deg, t["pip_deg"], t[f"{k2}_deg"])))
            for k, k2 in (("a", "a"), ("pip_servo", "ap"), ("b", "b"))}


class Tables:
    """``FlexionTables``: per acting digit, q <-> theta [rad] and dtheta/dq."""

    def __init__(self, desc: dict, digits):
        self.q, self.th, self.r = {}, {}, {}
        for d in digits:
            t = desc["four_bar"][d]
            q, th = np.radians(t["pip_deg"]), np.radians(t["ap_deg"])
            o = np.argsort(q)
            q, th = q[o], th[o]
            r = np.empty_like(q)
            r[1:-1] = (th[2:] - th[:-2]) / (q[2:] - q[:-2])
            r[0], r[-1] = (th[1] - th[0]) / (q[1] - q[0]), (th[-1] - th[-2]) / (q[-1] - q[-2])
            self.q[d], self.th[d], self.r[d] = q, th, r

    def servo(self, d, q):
        return float(np.interp(q, self.q[d], self.th[d]))

    def flexion(self, d, th):
        xp, fp = self.th[d], self.q[d]
        if xp[0] > xp[-1]:
            xp, fp = xp[::-1], fp[::-1]
        return float(np.interp(th, xp, fp))

    def ratio(self, d, q):
        return float(np.interp(q, self.q[d], self.r[d]))


def face_frame(desc: dict, mcp_deg: float = 0.0):
    """``rotate_cube.grasp.face_frame``: the middle finger's proximal pad face at MCP ``mcp_deg``, in the world (the
    hand's root at ``HAND_ROOT``): origin and rotation (columns: x toward the tips, y, z the outward normal)."""
    fe = np.array(next(j for j in desc["joints"] if j["name"] == "middle_fe")["anchor"], float)
    pad = next(b for body in desc["bodies"] for b in (body.get("boxes") or []) if b["name"] == "middle_proximal_pad")
    face_z = float(pad["center"][2]) - float(pad["half_extents"][1])
    t = math.radians(mcp_deg)
    rot_y = np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])
    foot = fe + rot_y @ (np.array([fe[0], fe[1], face_z]) - fe)
    origin = HAND_ROOT + ROOT_R @ foot
    ex, ez = ROOT_R @ rot_y @ np.array([1.0, 0.0, 0.0]), ROOT_R @ rot_y @ np.array([0.0, 0.0, -1.0])
    return origin, np.stack((ex, np.cross(ez, ex), ez), axis=1)


def tip_point_face(desc: dict, finger: str, q_deg: float) -> np.ndarray:
    """``finger_relay.env_cfg.tip_point_face``: the distal pad face's tip-end centre at flexion ``q_deg``, face frame."""
    joints = {j["name"]: j for j in desc["joints"]}

    def tf(name, angle):
        j = joints[name]
        a, p = np.asarray(j["axis"], float), np.asarray(j["anchor"], float)
        a = a / np.linalg.norm(a)
        k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        R = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k
        m = np.eye(4)
        m[:3, :3], m[:3, 3] = R, p - R @ p
        return m

    pose = closed_pose(desc, finger, q_deg)
    m = tf(f"{finger}_fe", 0.0) @ tf(f"{finger}_pip_servo", pose[f"{finger}_pip_servo"]) @ tf(f"{finger}_b", pose[f"{finger}_b"])
    body = next(b for b in desc["bodies"] if b["name"] == f"{finger}_distal")
    box = next(b for b in body["boxes"] if b["name"] == f"{finger}_distal_pad")
    c, h = np.asarray(box["center"]), box["half_extents"]
    o, r = face_frame(desc, 0.0)
    p = (m @ np.array([c[0] + h[0], c[1], c[2] - h[1], 1.0]))[:3]
    return r.T @ (HAND_ROOT + ROOT_R @ p - o)


def dial_place_face() -> np.ndarray:
    o = description("O")
    tip = tip_point_face(o, "index", R_QTOUCH)
    side = np.sign(tip_point_face(o, "index", 0.0)[1] - tip_point_face(o, "middle", 0.0)[1])
    return np.array([tip[0] - R_XBACK, tip[1] + side * R_YOUT, tip[2]])


# ------------------------------------------------------------------------------------------------------- the actor
class Actor:
    def __init__(self, task: str, seed: int):
        z = np.load(OUT / f"actor_{task}_s{seed}.npz")
        self.mean, self.std = z["mean"], z["std"]
        self.layers = [(z[f"w{i}"], z[f"b{i}"]) for i in range(4)]

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        x = (obs - self.mean) / (self.std + 1e-2)
        for i, (w, b) in enumerate(self.layers):
            x = w @ x + b
            if i < 3:
                x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))
        return x[: len(x) // 2]  # the heteroscedastic head: (mean, log std); the mean


def export() -> int:
    """Run in the Isaac Lab environment (torch): each checkpoint's actor to ``logs/policy_transfer/actor_*.npz``."""
    import torch

    OUT.mkdir(parents=True, exist_ok=True)
    for (task, seed), rel in CKPT.items():
        sd = torch.load(ISAAC / "logs" / "rsl_rl" / rel, map_location="cpu", weights_only=False)["actor_state_dict"]
        arrays = {"mean": sd["obs_normalizer._mean"].numpy()[0], "std": sd["obs_normalizer._std"].numpy()[0]}
        for i, k in enumerate((0, 2, 4, 6)):
            arrays[f"w{i}"], arrays[f"b{i}"] = sd[f"mlp.{k}.weight"].numpy(), sd[f"mlp.{k}.bias"].numpy()
        np.savez(OUT / f"actor_{task}_s{seed}.npz", **arrays)
        print(task, seed, "obs", arrays["mean"].shape, "out", arrays["b3"].shape)
    return 0


# ------------------------------------------------------------------------------------------------------- the plant
def build(hand: str, task: str, contact: str = "hydro"):
    from pydrake.geometry import (AddCompliantHydroelasticProperties, AddContactMaterial, AddRigidHydroelasticProperties,
                                  CollisionFilterDeclaration, Cylinder, GeometrySet, ProximityProperties, RoleAssign)
    from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import AddMultibodyPlant, CoulombFriction, MultibodyPlantConfig
    from pydrake.multibody.tree import (FixedOffsetFrame, JointActuatorIndex, PdControllerGains, PrismaticJoint, RevoluteJoint,
                                        SpatialInertia, UnitInertia)
    from pydrake.systems.framework import DiagramBuilder, LeafSystem, BasicVector

    desc = description(hand)
    acting = TASK[task]["acting"]
    builder = DiagramBuilder()
    plant, sg = AddMultibodyPlant(MultibodyPlantConfig(time_step=DT_ENV / SUB, discrete_contact_approximation="sap",
                                                       contact_model={"point": "point", "hydro": "hydroelastic_with_fallback"}[contact]), builder)
    model = Parser(plant).AddModels(str(urdf(hand)))[0]
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("palm", model),
                     RigidTransform(RollPitchYaw(math.pi, 0.0, 0.0), HAND_ROOT))
    origin = {b["name"]: np.asarray(b["origin"], float) for b in desc["bodies"]}
    for cut in desc["loops"]:
        a, b = plant.GetBodyByName(cut["parent"], model), plant.GetBodyByName(cut["child"], model)
        anchor = np.asarray(cut["anchor"], float)
        plant.AddBallConstraint(a, anchor - origin[cut["parent"]], b, anchor - origin[cut["child"]])
    servo = desc["servo"]["derived"]
    pip_act = {}
    for k in range(plant.num_actuators()):
        act = plant.get_joint_actuator(JointActuatorIndex(k))
        act.set_default_rotor_inertia(ARMATURE)
        act.set_default_gear_ratio(1.0)
        name = act.joint().name()
        if name.endswith("_pip_servo") and name.split("_")[0] in acting:
            pip_act[name.split("_")[0]] = act  # explicit DC-motor torque (no PD)
        else:
            act.set_controller_gains(PdControllerGains(p=servo["stiffness_nm_per_rad"],
                                                       d=servo["back_emf_damping_nms_per_rad"]))
    objects = {}
    obj_model = plant.AddModelInstance("object")
    if task == "walk":
        inertia = UnitInertia.SolidCylinder(W_RADIUS, W_LENGTH, [0.0, 1.0, 0.0])
        body = plant.AddRigidBody("cylinder", obj_model, SpatialInertia(W_MASS, np.zeros(3), inertia))
        pr = ProximityProperties()
        if contact == "hydro":
            AddRigidHydroelasticProperties(resolution_hint=0.002, properties=pr)
        AddContactMaterial(point_stiffness=K_CONTACT, friction=CoulombFriction(OBJ_MU, OBJ_MU), properties=pr)
        plant.RegisterCollisionGeometry(body, RigidTransform(RotationMatrix.MakeXRotation(math.pi / 2)),
                                        Cylinder(W_RADIUS, W_LENGTH), "cylinder", pr)
        objects["object"] = body
    else:
        o, r = face_frame(desc, 0.0)
        pos = o + r @ dial_place_face()
        base = plant.AddFrame(FixedOffsetFrame("dial_base", plant.world_frame(), RigidTransform(RotationMatrix(r), pos)))
        slider = plant.AddRigidBody("slider", obj_model, SpatialInertia(0.005, np.zeros(3), UnitInertia.SolidSphere(0.005)))
        disk = plant.AddRigidBody("disk", obj_model, SpatialInertia(R_MASS, [0.0, 0.0, R_THICK / 2],
                                                        UnitInertia.SolidCylinder(R_RADIUS, R_THICK, [0, 0, 1]).ShiftFromCenterOfMass([0, 0, -R_THICK / 2])))
        slide = plant.AddJoint(PrismaticJoint("slide", base, slider.body_frame(), [0, 0, 1], 0.0, R_SLIDE))
        spin = plant.AddJoint(RevoluteJoint("spin", slider.body_frame(), disk.body_frame(), [0, 0, 1]))
        plant.AddJointActuator("slide", slide).set_controller_gains(PdControllerGains(p=R_KSLIDE, d=R_DSLIDE))
        plant.AddJointActuator("spin", spin).set_controller_gains(PdControllerGains(p=R_KSPIN, d=R_DSPIN))
        pr = ProximityProperties()
        if contact == "hydro":
            AddRigidHydroelasticProperties(resolution_hint=0.002, properties=pr)
        AddContactMaterial(point_stiffness=K_CONTACT, friction=CoulombFriction(OBJ_MU, OBJ_MU), properties=pr)
        plant.RegisterCollisionGeometry(disk, RigidTransform([0, 0, R_THICK / 2]), Cylinder(R_RADIUS, R_THICK), "disk", pr)
        objects.update(object=disk, slide=slide, spin=spin)
    # contact materials of the hand colliders (pads: the *_c3 boxes of the proximal and distal bodies, the thumb pad)
    inspector = sg.model_inspector()
    for body_index in plant.GetBodyIndices(model):
        body = plant.get_body(body_index)
        for gid in plant.GetCollisionGeometriesForBody(body):
            name = inspector.GetName(gid)
            pad = name.endswith("_c3") and body.name().endswith(("_proximal", "_distal"))
            pr = ProximityProperties()
            mu = PAD_MU if pad else HAND_MU
            if pad and contact == "hydro":  # a compliant layer: Drake's box pressure E * depth / (half thickness)
                h = inspector.GetShape(gid)
                half = min(h.width(), h.depth(), h.height()) / 2
                AddCompliantHydroelasticProperties(resolution_hint=0.002, hydroelastic_modulus=HYDRO_K * half,
                                                   properties=pr)
            AddContactMaterial(point_stiffness=K_CONTACT, friction=CoulombFriction(mu, mu), properties=pr)
            sg.AssignRole(plant.get_source_id(), gid, pr, RoleAssign.kReplace)
    manager = sg.collision_filter_manager()
    for x, y in desc["collision"]["filtered_pairs"]:
        gx = plant.GetCollisionGeometriesForBody(plant.GetBodyByName(x, model))
        gy = plant.GetCollisionGeometriesForBody(plant.GetBodyByName(y, model))
        if gx and gy:
            manager.Apply(CollisionFilterDeclaration().ExcludeBetween(GeometrySet(gx), GeometrySet(gy)))
    plant.Finalize()

    tables = Tables(desc, acting)
    act_index = {d: int(a.index()) for d, a in pip_act.items()}
    joint_of = {d: plant.GetJointByName(f"{d}_pip_servo", model) for d in acting}

    class ServoTorque(LeafSystem):
        """The acting PIP servos' DC-motor torque, from the plant's state at every plant step; gains and targets set
        once per control step (``Env.step``)."""

        def __init__(self):
            super().__init__()
            self.state = self.DeclareVectorInputPort("x", plant.num_multibody_states())
            self.DeclareVectorOutputPort("u", plant.num_actuators(), self.calc)
            self.target = {d: 0.0 for d in acting}
            self.kp = {d: 0.0 for d in acting}
            self.kd = {d: 0.0 for d in acting}

        def calc(self, context, out):
            x = self.state.Eval(context)
            u = np.zeros(plant.num_actuators())
            nq = plant.num_positions()
            for d in acting:
                j = joint_of[d]
                th, w = x[j.position_start()], x[nq + j.velocity_start()]
                tau = self.kp[d] * (self.target[d] - th) - self.kd[d] * w
                hi = min(max(STALL_NM * (1.0 - w / NO_LOAD), 0.0), STALL_NM)
                lo = max(min(STALL_NM * (-1.0 - w / NO_LOAD), 0.0), -STALL_NM)
                u[act_index[d]] = min(max(tau, lo), hi)
            out.SetFromVector(u)

    torque = builder.AddSystem(ServoTorque())
    builder.Connect(plant.get_state_output_port(), torque.state)
    builder.Connect(torque.get_output_port(0), plant.get_actuation_input_port())
    diagram = builder.Build()
    return {"diagram": diagram, "plant": plant, "model": model, "desc": desc, "tables": tables, "torque": torque,
            "acting": acting, "objects": objects, "joint_of": joint_of}


# --------------------------------------------------------------------------------------------------------- the env
class Env:
    def __init__(self, hand: str, task: str):
        from pydrake.systems.analysis import Simulator

        self.task, self.hand = task, hand
        self.b = build(hand, task, CONTACT)
        self.plant, self.model = self.b["plant"], self.b["model"]
        self.ctx = self.b["diagram"].CreateDefaultContext()
        self.pctx = self.plant.GetMyMutableContextFromRoot(self.ctx)
        self.sim = Simulator(self.b["diagram"], self.ctx)
        self.sim.Initialize()
        self.acting = self.b["acting"]
        self.desc, self.tab = self.b["desc"], self.b["tables"]
        self.face_o, self.face_r = face_frame(self.desc, 0.0)
        distal = {d: self.plant.GetBodyByName(f"{d}_distal", self.model).index() for d in self.acting}
        self.distal_of = {v: k for k, v in distal.items()}
        self.obj = self.b["objects"]["object"]
        sg = self.b["diagram"].GetSubsystemByName("scene_graph")
        insp = sg.model_inspector()
        self.body_of = {g: self.plant.GetBodyFromFrameId(insp.GetFrameId(g)).index()
                        for g in insp.GetAllGeometryIds() if insp.GetProximityProperties(g) is not None}

    # -- state readers
    def theta(self, d):
        return self.b["joint_of"][d].get_angle(self.pctx)

    def flexion(self, d):
        return self.tab.flexion(d, self.theta(d))

    def touch_forces(self) -> np.ndarray:
        res = self.plant.get_contact_results_output_port().Eval(self.pctx)
        f = {d: np.zeros(3) for d in self.acting}
        oi = self.obj.index()
        for i in range(res.num_point_pair_contacts()):
            c = res.point_pair_contact_info(i)
            a, b_ = c.bodyA_index(), c.bodyB_index()
            if oi in (a, b_):
                other = b_ if a == oi else a
                if other in self.distal_of:
                    f[self.distal_of[other]] += np.asarray(c.contact_force())
        for i in range(res.num_hydroelastic_contacts()):  # the pads' hydroelastic patches
            c = res.hydroelastic_contact_info(i)
            cs = c.contact_surface()
            bm, bn = (self.body_of[g] for g in (cs.id_M(), cs.id_N()))
            if oi in (bm, bn):
                other = bn if bm == oi else bm
                if other in self.distal_of:
                    f[self.distal_of[other]] += np.asarray(c.F_Ac_W().translational())
        return np.array([np.linalg.norm(f[d]) for d in self.acting])

    def obj_face(self):
        X = self.plant.EvalBodyPoseInWorld(self.pctx, self.obj)
        V = self.plant.EvalBodySpatialVelocityInWorld(self.pctx, self.obj)
        return self.face_r.T @ (X.translation() - self.face_o), self.face_r.T @ V.translational()

    # -- episode
    def reset(self, rng):
        from pydrake.math import RigidTransform, RotationMatrix
        from pydrake.multibody.math import SpatialVelocity

        plant, ctx = self.plant, self.pctx
        self.sim.get_mutable_context().SetTime(0.0)
        plant.SetPositions(ctx, self.model, np.zeros(plant.num_positions(self.model)))
        plant.SetVelocities(ctx, self.model, np.zeros(plant.num_velocities(self.model)))
        for d in self.desc["four_bar"]:
            q = float(rng.integers(0, TASK[self.task]["start_max_deg"] + 1)) if d in self.acting else 0.0
            for name, v in closed_pose(self.desc, d, q).items():
                plant.GetJointByName(name, self.model).set_angle(ctx, v)
        held = [plant.get_joint_actuator(i).joint().get_angle(ctx) for i in plant.GetJointActuatorIndices(self.model)]
        plant.get_desired_state_input_port(self.model).FixValue(ctx, np.concatenate([held, np.zeros(len(held))]))
        if self.task == "walk":
            x = rng.uniform(*W_START_X)
            pos = self.face_o + self.face_r @ np.array([x, 0.0, W_RADIUS + 0.0015])
            plant.SetFreeBodyPose(ctx, self.obj, RigidTransform(RotationMatrix(self.face_r), pos))
            plant.SetFreeBodySpatialVelocity(ctx, self.obj, SpatialVelocity(np.zeros(3), np.zeros(3)))
            self.k = int(rng.integers(0, len(W_TARGETS)))
            p, _ = self.obj_face()
            self.x, self.zh = p[0], p[2]
            self.err = W_TARGETS[self.k % 2] - self.x
            self.walked, self.hold, self.goals, self.alive = 0.0, 0, 0, True
        else:
            dial_model = self.b["objects"]["slide"].model_instance()
            self.b["objects"]["slide"].set_translation(ctx, 0.0)
            self.b["objects"]["spin"].set_angle(ctx, 0.0)
            plant.get_desired_state_input_port(dial_model).FixValue(ctx, np.zeros(4))
            self.start = self.b["objects"]["spin"].get_angle(ctx)
            self.th, self.goal, self.hold, self.goals = 0.0, R_GOAL, 0, 0
        self.prev_cmd = {d: self.flexion(d) for d in self.acting}  # EMA's previous command: the current flexion
        self.last_action = np.zeros(len(self.acting))
        self.hist = None
        self.rng = rng
        return self.observe(first=True)

    def task_obs(self):
        if self.task == "walk":
            _, v = self.obj_face()
            return np.array([self.x * 100, self.zh * 100, np.clip(v[0], -1, 1), np.clip(self.err * 100, -10, 10)])
        w = self.b["objects"]["spin"].get_angular_rate(self.pctx)
        err = np.clip(self.goal - self.th, -math.pi, math.pi) / math.pi
        return np.array([math.sin(self.th), math.cos(self.th), err, np.clip(w / 10.0, -1, 1)])

    def observe(self, first=False):
        f = self.touch_forces()
        gain = 1.0 + TOUCH["force_noise"] * (2 * self.rng.random(len(f)) - 1)
        force = np.clip(f * gain, 0.0, TOUCH["max_force_n"]) / TOUCH["max_force_n"]
        flag = (f > TOUCH["threshold_n"]) ^ (self.rng.random(len(f)) < TOUCH["flip_prob"])
        pip = np.array([2 * (self.flexion(d) - Q_LO) / (Q_HI - Q_LO) - 1 for d in self.acting])
        terms = [self.last_action.copy(), pip, self.task_obs(), np.concatenate([force, flag.astype(float)])]
        if first or self.hist is None:
            self.hist = [[t.copy() for _ in range(5)] for t in terms]
        else:
            for h, t in zip(self.hist, terms):
                h.pop(0)
                h.append(t)
        return np.concatenate([np.concatenate(h) for h in self.hist])

    def step(self, action):
        self.last_action = np.asarray(action, float).copy()
        a = np.clip(action, -1.0, 1.0)
        tq = self.b["torque"]
        for i, d in enumerate(self.acting):
            cmd = Q_LO + (a[i] + 1) / 2 * (Q_HI - Q_LO)
            cmd = 0.5 * cmd + 0.5 * self.prev_cmd[d]
            cmd = min(max(cmd, Q_LO), Q_HI)
            self.prev_cmd[d] = cmd
            tq.target[d] = self.tab.servo(d, cmd)
            r = self.tab.ratio(d, self.flexion(d))
            tq.kp[d], tq.kd[d] = K_Q / r**2, D_Q / r**2
        self.sim.AdvanceTo(self.sim.get_context().get_time() + DT_ENV)
        done = False
        if self.task == "walk":
            p, _ = self.obj_face()
            self.x, self.zh = p[0], p[2]
            err = W_TARGETS[self.k % 2] - self.x
            progress = abs(self.err) - abs(err)
            if (p[0] > W_X_MAX) or (p[2] < W_Z_MIN) or (abs(p[1]) > W_Y_MAX) or not np.isfinite(p).all():
                done = True  # dropped: the walk so far is kept (eval_walk takes the totals before the reset)
            else:
                self.walked += progress
                self.hold = self.hold + 1 if abs(err) < W_SUCCESS else 0
                if self.hold >= W_HOLD:
                    self.k += 1
                    self.goals += 1
                    self.hold = 0
                self.err = W_TARGETS[self.k % 2] - self.x
        else:
            self.th = self.b["objects"]["spin"].get_angle(self.pctx) - self.start
            self.hold = self.hold + 1 if abs(self.th - self.goal) < R_SUCCESS else 0
            if self.hold >= R_HOLD:
                self.goal += R_GOAL
                self.goals += 1
                self.hold = 0
        return self.observe(), done


def episode(args):
    task, hand, seed, ep, policy, const = args
    env = Env(hand, task)
    rng = np.random.default_rng(1000 * seed + ep)
    actor = Actor(task, seed) if policy else None
    obs = env.reset(rng)
    t0 = time.time()
    dropped = False
    for _ in range(STEPS):
        a = actor(obs) if actor else np.full(len(env.acting), const)
        obs, done = env.step(a)
        if done:
            dropped = True
            break
    score = 1e3 * env.walked if task == "walk" else math.degrees(env.th)
    return {"episode": ep, "score": float(score), "goals": int(env.goals), "dropped": dropped,
            "wall_s": round(time.time() - t0, 1)}


def cmd_run(a) -> int:
    from multiprocessing import Pool

    OUT.mkdir(parents=True, exist_ok=True)
    jobs = [(a.task, h, s, e, True, 0.0) for s in a.seeds for h in a.hands for e in range(a.episodes)
            if not (OUT / f"{a.task}_{h}_s{s}_{a.tag}.json").exists()]
    with Pool(a.workers) as pool:
        res = pool.map(episode, jobs, chunksize=1)
    by = {}
    for j, r in zip(jobs, res):
        by.setdefault((j[1], j[2]), []).append(r)
    for (h, s), rs in by.items():
        (OUT / f"{a.task}_{h}_s{s}_{a.tag}.json").write_text(json.dumps(
            {"task": a.task, "hand": h, "seed": s, "contact": CONTACT, "episodes": rs, "score_mean": float(np.mean([r["score"] for r in rs])),
             "dropped_rate": float(np.mean([r["dropped"] for r in rs]))}, indent=1))
    summarize(a.task, a.tag, a.seeds)
    return 0


def summarize(task, tag, seeds):
    for s in seeds:
        rows = {}
        for f in sorted(OUT.glob(f"{task}_*_s{s}_{tag}.json")):
            d = json.loads(f.read_text())
            rows[d["hand"]] = d
        if "O" not in rows:
            continue
        o = rows["O"]["score_mean"]
        print(f"{task} s{s}: O {o:.1f} | " + " | ".join(
            f"{h} {d['score_mean']:.1f} ({d['score_mean'] / o:.2f}, drop {d['dropped_rate']:.2f})" for h, d in rows.items() if h != "O"))


def cmd_check(a) -> int:
    for const in (-1.0, 0.0):
        r = episode((a.task, a.hand, 20, 0, False, const))
        print("constant action", const, r)
    r = episode((a.task, a.hand, 20, 0, True, 0.0))
    print("policy s20", r)
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("export")
    r = sub.add_parser("run")
    r.add_argument("--task", choices=tuple(TASK), required=True)
    r.add_argument("--hands", nargs="+", default=["O", "MRM", "SK", "U"])
    r.add_argument("--seeds", nargs="+", type=int, default=[20, 21])
    r.add_argument("--episodes", type=int, default=24)
    r.add_argument("--workers", type=int, default=6)
    r.add_argument("--tag", default="pt1")
    r.add_argument("--contact", choices=("hydro", "point"), default="hydro")
    c = sub.add_parser("check")
    c.add_argument("--task", choices=tuple(TASK), required=True)
    c.add_argument("--hand", default="O")
    c.add_argument("--contact", choices=("hydro", "point"), default="hydro")
    s = sub.add_parser("summary")
    s.add_argument("--task", choices=tuple(TASK), required=True)
    s.add_argument("--tag", default="pt1")
    s.add_argument("--seeds", nargs="+", type=int, default=[20, 21])
    a = p.parse_args()
    global CONTACT
    CONTACT = getattr(a, "contact", CONTACT)
    if a.cmd == "export":
        return export()
    if a.cmd == "run":
        return cmd_run(a)
    if a.cmd == "summary":
        summarize(a.task, a.tag, a.seeds)
        return 0
    return cmd_check(a)


if __name__ == "__main__":
    raise SystemExit(main())
