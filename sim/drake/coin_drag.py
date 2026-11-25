"""Experiment 30 (tag e30): the fingertip drags a coin toward the palm (docs/experimental/drake_coin.md).

The translational counterpart of experiment 28's fingertip disk (task tipdial z). The planar CoDe finger of
two_pad_drake.py (imported read-only: geometry, servo table, the flexion impedance of condition A; the MCP held at 0, so
the proximal phalanx is the world). Frame: x along the proximal phalanx from the MCP, z dorsal; every placement is given
in the hand's proximal-face frame (origin at the foot of the MCP axis on the proximal face), so world z = z_face - face.

- Table: a box below the finger whose top rests, with the coin on it, at the placement's height. It rides a vertical
  slide on a soft spring (K_TABLE, with its and the coin's weight carried by a constant force), so the fingertip's press
  is set by the spring, not by driving into the pad (experiment 28's axle spring; drake_acquisition.md's over-pressing).
- Coin: a flat disk (diameter d, 2 mm thick, 3 g) lying on the table; it slides in x and z and turns about y (y locked:
  every contact is in the finger's plane). It is tethered along x to its start by a soft spring (K_TETHER): the
  translational counterpart of the dial's torsional spring. Without it the coin follows the pad whatever mu is (the
  pad's mu is 1.3 or 2.0 against the table's 0.3), and mu has no effect (checked on O, calibration).
- Friction: pad-coin mu (1.3 or 2.0, the interface property), coin-table MU_TABLE fixed. Drake combines a pair's
  friction as 2 m1 m2 / (m1 + m2); the coin carries mu and the table the value that makes the coin-table pair MU_TABLE.
- Contact: point contact, pads 2.5 kN/m (the pair's stiffness is the pad's), coin and table rigid (1e8 N/m).
- Command (O's open loop on every hand): flexion 0 -> q_t over approach_s, q_t -> q_t + s over sweep_s, held hold_s;
  q_t is O's touch angle (O's fingertip face point meets the coin's top at q_t, before the height offset dz).
- Measure: the coin's travel toward the MCP (mm) at the end, and its largest travel; D = travel(2.0) - travel(1.3).

    uv run python coin_drag.py draws --n 64 --seed 30 --out logs/coin_e30.json
    uv run python coin_drag.py run --hand O --mu 1.3 --draws logs/coin_e30.json --tag e30 [--delta_deg 0.05]
    uv run python coin_drag.py analyze --tag e30 --noise_tag e30n
    uv run python coin_drag.py frames --draws logs/coin_e30.json --draw 0
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
MU_TABLE = 0.3
K_TABLE, D_TABLE = 60.0, 2.0  # N/m, N s/m: the table's vertical spring
K_TETHER, D_TETHER = 150.0, 0.5  # N/m, N s/m: the coin's tether along x
COIN_THICK_MM, COIN_MASS_KG = 2.0, 0.003
TABLE_MASS_KG = 0.02
FOOT_MM = 0.5  # the coin's sphere feet on the table
FOOT_K = 1e8  # N/m, each foot's point stiffness
SPIN_D = 0.0  # N m s/rad on the coin's rocking (about y)
TABLE_BACK_MM, TABLE_FRONT_MM = 45.0, 30.0  # the table's extent behind and ahead of the coin's centre
PEN_MM, POINT_K = 1.5, T.POINT_STIFFNESS * 1e-3  # the penetration gate (mm) and the pad's point stiffness (N/mm)
TOUCH_N, PROX_TOUCH_N = T.GATE_N, 1e-3
G = 9.81

# experiment 30's distributions (fixed before any redesigned-hand data): independent uniforms, drawn as vectors of N in
# this order from numpy.random.default_rng([seed, 30])
RANDOM = {"q_t_deg": (25.0, 50.0), "x_back_mm": (2.0, 7.0), "dz_mm": (-0.5, 0.5), "diam_mm": (20.0, 28.0),
          "sweep_deg": (15.0, 25.0)}


def make_draws(n: int, seed: int) -> list[dict]:
    rng = np.random.default_rng([seed, 30])
    cols = {k: rng.uniform(lo, hi, n) for k, (lo, hi) in RANDOM.items()}
    geom = T.finger_geometry("O")
    qs, th = T.servo_table(geom)
    out = []
    for i in range(n):
        d = {k: float(v[i]) for k, v in cols.items()}
        tip = T.reference_point(geom, qs, th, math.radians(d["q_t_deg"]), "tip")
        # the coin's centre (proximal-face frame): x_back proximal of O's fingertip face point at q_t; its top face at
        # that point's depth, raised by dz
        out.append({"draw": i, **d, "x_c_mm": float(tip[0] - d["x_back_mm"]),
                    "z_top_mm": float(tip[1] + d["dz_mm"])})
    return out


def pair_mu(mu_coin: float, target: float) -> float:
    """The table's friction that makes Drake's combined coin-table friction equal target."""
    return target * mu_coin / (2 * mu_coin - target)


def build(hand: str, mu: float, pl: dict, time_step: float, k_theta: float, d_theta: float, tether: float,
          foot_k: float = None, spin_d: float = None):
    from pydrake.geometry import (AddContactMaterial, Box, CollisionFilterDeclaration, Cylinder, GeometrySet,
                                  ProximityProperties, Sphere)
    from pydrake.math import RigidTransform
    from pydrake.multibody.plant import (AddMultibodyPlantSceneGraph, ContactModel, CoulombFriction,
                                         DiscreteContactApproximation)
    from pydrake.multibody.tree import (FixedOffsetFrame, PdControllerGains, PrismaticJoint, RevoluteJoint,
                                        SpatialInertia, UnitInertia)
    from pydrake.systems.framework import DiagramBuilder

    g = T.finger_geometry(hand)
    mm = 1e-3
    pads = g["pads"]
    m, h, w = pads["mount_offset_mm"] * mm, pads["thickness_mm"] * mm, pads["width_mm"] * mm
    face = m + h
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
    plant.set_contact_model(ContactModel.kPoint)
    world = plant.world_body()
    y = np.array([0.0, 1.0, 0.0])
    zax = np.array([0.0, 0.0, 1.0])

    def at(p2):
        return np.array([p2[0], 0.0, p2[1]])

    def rod(mass, length):
        return SpatialInertia(mass, np.zeros(3), UnitInertia.SolidBox(max(length, 1e-3), 0.004, 0.004))

    # the finger, as two_pad_drake._build
    a, ap, b = g["a"] * mm, g["ap"] * mm, g["b"] * mm
    bp_local = g["bp_local"] * mm
    driver = plant.AddRigidBody("driver", rod(0.003, float(np.linalg.norm(b - ap))))
    distal = plant.AddRigidBody("distal", rod(0.01, 0.03))
    f_ap = plant.AddFrame(FixedOffsetFrame("at_ap", world.body_frame(), RigidTransform(at(ap))))
    f_b = plant.AddFrame(FixedOffsetFrame("b_on_driver", driver.body_frame(), RigidTransform(at(b - ap))))
    servo = plant.AddJoint(RevoluteJoint("servo", f_ap, driver.body_frame(), y))
    plant.AddJointActuator("servo", servo).set_controller_gains(PdControllerGains(p=k_theta, d=d_theta))
    plant.AddJoint(RevoluteJoint("pip_b", f_b, distal.body_frame(), y))
    proximal = plant.AddRigidBody("proximal", rod(0.039, 0.044))
    plant.WeldFrames(world.body_frame(), proximal.body_frame())
    plant.AddDistanceConstraint(proximal, at(a), distal, at(bp_local), g["abp"] * mm)

    # the table: its body frame at its top face's middle (x of the coin), on a vertical spring
    r = pl["diam_mm"] / 2 * mm
    tk = COIN_THICK_MM * mm
    x_c = pl["x_c_mm"] * mm
    z_top = pl["z_top_mm"] * mm - face - tk  # the table's top (world), under the coin
    t_len = (TABLE_BACK_MM + TABLE_FRONT_MM) * mm
    table = plant.AddRigidBody("table", SpatialInertia(TABLE_MASS_KG, np.zeros(3),
                                                       UnitInertia.SolidBox(t_len, 0.04, 0.005)))
    f_tb = plant.AddFrame(FixedOffsetFrame("table_base", world.body_frame(), RigidTransform([x_c, 0.0, z_top])))
    table_z = plant.AddJoint(PrismaticJoint("table_z", f_tb, table.body_frame(), zax))
    plant.AddJointActuator("table_z", table_z).set_controller_gains(PdControllerGains(p=K_TABLE, d=D_TABLE))

    # the coin: world -x-> cx -z-> cz -spin(y)-> coin; tethered along x to its start
    cx = plant.AddRigidBody("coin_cx", rod(1e-4, 0.01))
    cz = plant.AddRigidBody("coin_cz", rod(1e-4, 0.01))
    coin = plant.AddRigidBody("coin", SpatialInertia(COIN_MASS_KG, np.zeros(3),
                                                     UnitInertia.SolidCylinder(r, tk, zax)))
    f_cb = plant.AddFrame(FixedOffsetFrame("coin_base", world.body_frame(),
                                           RigidTransform([x_c, 0.0, z_top + tk / 2])))
    slide_x = plant.AddJoint(PrismaticJoint("coin_x", f_cb, cx.body_frame(), [1.0, 0.0, 0.0]))
    plant.AddJoint(PrismaticJoint("coin_z", cx.body_frame(), cz.body_frame(), zax))
    plant.AddJoint(RevoluteJoint("coin_spin", cz.body_frame(), coin.body_frame(), y,
                                 damping=SPIN_D if spin_d is None else spin_d))
    plant.AddJointActuator("coin_x", slide_x).set_controller_gains(PdControllerGains(p=tether, d=D_TETHER))

    def props(k, mu_):
        pr = ProximityProperties()
        AddContactMaterial(point_stiffness=k, friction=CoulombFriction(mu_, mu_), properties=pr)
        return pr

    s0, s1 = pads["proximal_start_mm"] * mm, pads["proximal_end_mm"] * mm
    plant.RegisterCollisionGeometry(proximal, RigidTransform(at(((s0 + s1) / 2, -(m + h / 2)))), Box(s1 - s0, w, h),
                                    "proximal_pad", props(T.POINT_STIFFNESS, mu))
    t0, t1 = pads["distal_start_mm"] * mm, pads["distal_end_mm"] * mm
    plant.RegisterCollisionGeometry(distal, RigidTransform(at(((t0 + t1) / 2, -(m + h / 2)))), Box(t1 - t0, w, h),
                                    "distal_pad", props(T.POINT_STIFFNESS, mu))
    pad_ids = [g_ for g_ in plant.GetCollisionGeometriesForBody(proximal) + plant.GetCollisionGeometriesForBody(distal)]
    # the coin: a cylinder for the pads (as experiment 28's disk); its underside on the table through small sphere feet
    # (a flat cylinder face on a box face is degenerate for point contact: the coin sank 2 mm in 0.3 s)
    coin_id = plant.RegisterCollisionGeometry(coin, RigidTransform(), Cylinder(r, tk), "coin", props(1e8, mu))
    feet = []
    fr = FOOT_MM * mm
    spots = [(0.0, 0.0)] + [((r - fr) * math.cos(2 * math.pi * k / 12), (r - fr) * math.sin(2 * math.pi * k / 12))
                            for k in range(12)]
    for k, (fx, fy) in enumerate(spots):
        feet.append(plant.RegisterCollisionGeometry(coin, RigidTransform([fx, fy, -tk / 2 + fr]), Sphere(fr),
                                                    f"coin_foot{k}", props(FOOT_K if foot_k is None else foot_k, mu)))
    t_h = 0.01
    table_id = plant.RegisterCollisionGeometry(
        table, RigidTransform([(TABLE_FRONT_MM - TABLE_BACK_MM) / 2 * mm, 0.0, -t_h / 2]), Box(t_len, 0.06, t_h),
        "table", props(1e8, pair_mu(mu, MU_TABLE)))
    plant.Finalize()
    cfm = scene_graph.collision_filter_manager()
    cfm.Apply(CollisionFilterDeclaration().ExcludeBetween(GeometrySet([coin_id]), GeometrySet([table_id])))
    cfm.Apply(CollisionFilterDeclaration().ExcludeBetween(GeometrySet(feet), GeometrySet(pad_ids)))
    plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, -G])  # palm down over the table
    return builder, plant, scene_graph, {"distal": distal, "proximal": proximal, "coin": coin, "table": table,
                                         "face": face}


class Run:
    def __init__(self, hand, mu, pl, time_step, k_theta, d_theta, tether, foot_k=None, spin_d=None):
        from pydrake.systems.analysis import Simulator
        from pydrake.systems.framework import LeafSystem

        builder, plant, sg, bodies = build(hand, mu, pl, time_step, k_theta, d_theta, tether, foot_k, spin_d)
        self.plant, self.bodies = plant, bodies
        self.theta_cmd = 0.0
        nv, n_act = plant.num_velocities(), plant.num_actuators()
        iz = plant.GetJointByName("table_z").velocity_start()
        carried = (TABLE_MASS_KG + COIN_MASS_KG) * G
        outer = self

        class Command(LeafSystem):
            def __init__(self):
                super().__init__()
                self.DeclareVectorOutputPort("desired", 2 * n_act, self.calc_desired)
                self.DeclareVectorOutputPort("load", nv, self.calc_load)

            def calc_desired(self, context, out):  # servo, table_z, coin_x; then their velocities
                out.SetFromVector(np.array([outer.theta_cmd, 0.0, 0.0] + [0.0] * n_act))

            def calc_load(self, context, out):
                tau = np.zeros(nv)
                tau[iz] = carried  # the table's spring carries its and the coin's weight at its rest height
                out.SetFromVector(tau)

        cmd = builder.AddSystem(Command())
        builder.Connect(cmd.get_output_port(0),
                        plant.get_desired_state_input_port(plant.GetModelInstanceByName("DefaultModelInstance")))
        builder.Connect(cmd.get_output_port(1), plant.get_applied_generalized_force_input_port())
        self.diagram = builder.Build()
        self.sim = Simulator(self.diagram)
        self.pctx = plant.GetMyMutableContextFromRoot(self.sim.get_mutable_context())
        plant.get_actuation_input_port().FixValue(self.pctx, np.zeros(n_act))
        self.idx = {k: v.index() for k, v in bodies.items() if k != "face"}
        self.t = 0.0

    def step(self):
        self.t += T.DT_ENV
        self.sim.AdvanceTo(self.t)

    def contacts(self) -> dict:
        """Force on the coin from the distal pad (and its point), and the largest force of every other pair kind."""
        res = self.plant.get_contact_results_output_port().Eval(self.pctx)
        i = self.idx
        name = {i["distal"]: "distal", i["proximal"]: "proximal", i["coin"]: "coin", i["table"]: "table"}
        out = {"distal_coin": np.zeros(3), "distal_coin_pt": None, "proximal_any": 0.0, "distal_table": 0.0,
               "coin_table": 0.0}
        for k in range(res.num_point_pair_contacts()):
            c = res.point_pair_contact_info(k)
            a, b = name.get(c.bodyA_index()), name.get(c.bodyB_index())
            f = c.contact_force()  # on B
            pair = {a, b}
            if pair == {"distal", "coin"}:
                out["distal_coin"] = out["distal_coin"] + (f if b == "coin" else -f)
                out["distal_coin_pt"] = c.contact_point()
            elif "proximal" in pair:
                out["proximal_any"] += float(np.linalg.norm(f))
            elif pair == {"distal", "table"}:
                out["distal_table"] += float(np.linalg.norm(f))
            elif pair == {"coin", "table"}:
                out["coin_table"] += float(np.linalg.norm(f))
        return out

    def pose(self, body):
        X = self.plant.EvalBodyPoseInWorld(self.pctx, self.bodies[body])
        R = X.rotation().matrix()
        return np.array([X.translation()[0], X.translation()[2], math.atan2(R[0, 2], R[0, 0])])


def drag(hand, mu, pl, time_step, k_theta, d_theta, q_start, qs, th, opts, record=False) -> dict:
    tether = opts.get("tether", K_TETHER)
    s = Run(hand, mu, pl, time_step, k_theta, d_theta, tether, opts.get("foot_k"), opts.get("spin_d"))
    s.sim.Initialize()
    geom = T.finger_geometry(hand)
    pads = geom["pads"]
    face, m_mm = pads["mount_offset_mm"] + pads["thickness_mm"], pads["mount_offset_mm"]
    s0, s1 = pads["proximal_start_mm"], pads["proximal_end_mm"]
    x0 = s.pose("coin")[0]
    q1 = min(q_start + math.radians(opts["sweep_deg"]), math.radians(120.0))
    n_app, n_sweep, n_hold = (int(round(opts[k] / T.DT_ENV)) for k in ("approach_s", "sweep_s", "hold_s"))
    touch_q = None
    fd_max = prox_max = dtab_max = 0.0
    travel_max, clear = -1e9, float("inf")
    rad = pl["diam_mm"] / 2
    frames, trace = [], []
    for i in range(n_app + n_sweep + n_hold):
        q = q_start * (i + 1) / n_app if i < n_app else q_start + (q1 - q_start) * min(1.0, (i + 1 - n_app) / n_sweep)
        s.theta_cmd = float(np.interp(q, qs, th))
        s.step()
        c = s.contacts()
        fd = float(np.linalg.norm(c["distal_coin"]))
        fd_max, prox_max, dtab_max = max(fd_max, fd), max(prox_max, c["proximal_any"]), max(dtab_max, c["distal_table"])
        cp, tp = s.pose("coin"), s.pose("table")
        travel = 1e3 * (x0 - cp[0])
        travel_max = max(travel_max, travel)
        # geometric clearance (mm, x-z) from the proximal pad to the coin (its rectangle, tilt ignored) and the table top
        cxm, czm = 1e3 * cp[0], 1e3 * cp[1]
        gap_coin = math.hypot(max(s0 - (cxm + rad), 0.0, (cxm - rad) - s1),
                              max(-face - (czm + COIN_THICK_MM / 2), 0.0, (czm - COIN_THICK_MM / 2) - (-m_mm)))
        table_top = 1e3 * tp[1]
        tx0, tx1 = 1e3 * tp[0] - TABLE_BACK_MM, 1e3 * tp[0] + TABLE_FRONT_MM
        gap_table = math.hypot(max(s0 - tx1, 0.0, tx0 - s1), max(-face - table_top, 0.0))
        clear = min(clear, gap_coin, gap_table)
        if touch_q is None and fd > TOUCH_N:
            touch_q = math.degrees(q)
        if (i + 1) % 6 == 0:
            trace.append([round(s.t, 3), round(math.degrees(q), 2), round(travel, 3), round(fd, 4)])
        if record:
            dp = s.pose("distal")
            pt = c["distal_coin_pt"]
            frames.append({"t": s.t, "q_cmd_deg": math.degrees(q), "q_deg": math.degrees(dp[2]),
                           "distal_pose_mm_rad": [1e3 * dp[0], 1e3 * dp[1], dp[2]],
                           "coin_pose_mm_rad": [1e3 * cp[0], 1e3 * cp[1], cp[2]], "table_z_mm": 1e3 * tp[1],
                           "travel_mm": travel,
                           "contact_point_mm": None if pt is None or fd < 1e-6 else [1e3 * pt[0], 1e3 * pt[2]],
                           "contact_force_on_coin_n": [float(c["distal_coin"][0]), float(c["distal_coin"][2])]})
    cp, tp, dp = s.pose("coin"), s.pose("table"), s.pose("distal")
    c = s.contacts()
    out = {"travel_end_mm": float(1e3 * (x0 - cp[0])), "travel_max_mm": float(travel_max),
           "coin_lift_mm": float(1e3 * (cp[1] - tp[1]) - COIN_THICK_MM / 2),
           "coin_tilt_deg": float(math.degrees(cp[2])), "table_drop_mm": float(-(1e3 * tp[1] - (pl["z_top_mm"] - face - COIN_THICK_MM))),
           "touched": touch_q is not None, "touch_q_deg": touch_q, "q_start_deg": math.degrees(q_start),
           "q_end_cmd_deg": math.degrees(q1), "q_end_deg": math.degrees(dp[2]),
           "distal_force_end_n": float(np.linalg.norm(c["distal_coin"])), "distal_force_max_n": fd_max,
           "proximal_force_max_n": prox_max, "distal_table_force_max_n": dtab_max, "proximal_clearance_min_mm": clear,
           "trace": trace}
    if record:
        out["frames"] = frames
    return out


def run_env(task):
    try:
        return _run_env(task)
    except RuntimeError:
        pass
    try:
        retry = list(task)
        retry[3] = task[3] / 2
        out = _run_env(tuple(retry))
        out["retried_time_step"] = retry[3]
        return out
    except RuntimeError as err:
        return {"placement": task[2], "solver_fail": str(err)[:200], "drag": None}


def _run_env(task):
    hand, mu, pl, time_step, delta, opts = task[:6]
    record = len(task) > 6 and task[6]
    geom = T.finger_geometry(hand)
    qs, th = T.servo_table(geom)
    q_t = math.radians(pl["q_t_deg"] + delta)
    ratio = float(np.interp(q_t, qs, np.gradient(th, qs)))  # the gain map at the touch angle, as tipdial
    k_theta, d_theta = T.K_Q / ratio ** 2, T.D_Q / ratio ** 2
    o = {**opts, "sweep_deg": pl.get("sweep_deg", opts.get("sweep_deg", 20.0))}
    return {"placement": pl, "drag": drag(hand, mu, pl, time_step, k_theta, d_theta, q_t, qs, th, o, record)}


def out_path(hand, mu, tag):
    return OUT / f"dkcoin_{hand}_mu{mu:g}_point-{tag}.json"


def cmd_run(a) -> int:
    draws = json.loads(a.draws.read_text())["grasps"]
    if a.only is not None:
        draws = [d for d in draws if d["draw"] in a.only]
    opts = {"approach_s": a.approach_s, "sweep_s": a.sweep_s, "hold_s": a.hold_s, "tether": a.tether,
            "foot_k": a.foot_k, "spin_d": a.spin_d}
    if a.sweep_deg is not None:
        draws = [{**d, "sweep_deg": a.sweep_deg} for d in draws]
    tasks = [(a.hand, a.mu, d, a.time_step, a.delta_deg, opts) for d in draws]
    t0 = time.time()
    with Pool(a.workers) as pool:
        envs = pool.map(run_env, tasks)
    fails = sum(e["drag"] is None for e in envs)
    retried = sum("retried_time_step" in e for e in envs)
    print(f"{a.hand} mu {a.mu} {a.tag}: {len(envs)} draws in {time.time() - t0:.0f} s (solver fails {fails}, "
          f"retried {retried})", flush=True)
    out_path(a.hand, a.mu, a.tag).write_text(json.dumps({
        "hand": a.hand, "mu": a.mu, "tag": a.tag, "time_step": a.time_step, "delta_deg": a.delta_deg, "opts": opts,
        "k_table": K_TABLE, "mu_table": MU_TABLE, "envs": envs}))
    return 0


# ---------------------------------------------------------------- analysis (pre-registered in drake_coin.md)
MU = (1.3, 2.0)
MIN_D_MM = 0.3
METRIC = "travel_end_mm"


def load(hand, mu, tag):
    p = out_path(hand, mu, tag)
    if not p.exists():
        return None
    return {e["placement"]["draw"]: e["drag"] for e in json.loads(p.read_text())["envs"]}


def o_pairs(tag, metric=METRIC):
    """O's surviving draws (the pre-registered exclusions, first reason counted) and D(O) on them."""
    o = {m: load("O", m, tag) for m in MU}
    keep, excl = {}, {"solver": 0, "untouched": 0, "o_pen": 0, "o_proximal": 0, "o_pad_table": 0, "small_d": 0}
    for k in o[MU[0]]:
        ds = [o[m].get(k) for m in MU]
        if any(d is None for d in ds):
            excl["solver"] += 1
        elif not all(d["touched"] for d in ds):
            excl["untouched"] += 1
        elif any(d["distal_force_max_n"] / POINT_K > PEN_MM for d in ds):
            excl["o_pen"] += 1
        elif any(d["proximal_force_max_n"] > PROX_TOUCH_N for d in ds):
            excl["o_proximal"] += 1
        elif any(d["distal_table_force_max_n"] > PROX_TOUCH_N for d in ds):
            excl["o_pad_table"] += 1
        elif abs(ds[1][metric] - ds[0][metric]) < MIN_D_MM:
            excl["small_d"] += 1
        else:
            keep[k] = ds[1][metric] - ds[0][metric]
    return keep, excl, o


def hand_error(hand, tag, keep, metric=METRIC):
    h, _, htag = hand.partition("@")
    hh = {m: load(h, m, htag or tag) for m in MU}
    if any(v is None for v in hh.values()):
        return None
    rel, rel_sens, lost, over, prox, padtab, pen = [], [], 0, 0, 0, 0, []
    clear = []
    for k, d_o in keep.items():
        hs = [hh[m].get(k) for m in MU]
        if any(d is None or not d["touched"] for d in hs):
            lost += 1
            rel_sens.append(float("inf"))
            continue
        p = max(d["distal_force_max_n"] for d in hs) / POINT_K
        pen.append(p)
        clear.append(min(d["proximal_clearance_min_mm"] for d in hs))
        prox += any(d["proximal_force_max_n"] > PROX_TOUCH_N for d in hs)
        padtab += any(d["distal_table_force_max_n"] > PROX_TOUCH_N for d in hs)
        if p > PEN_MM:
            over += 1
            rel_sens.append(float("inf"))  # sensitivity: an over-pressed pair scored as the largest error
            continue
        e = abs((hs[1][metric] - hs[0][metric]) - d_o) / abs(d_o)
        rel.append(e)
        rel_sens.append(e)
    return {"E": float(np.median(rel)) if rel else None, "E_p75": float(np.percentile(rel, 75)) if rel else None,
            "E_over_as_max": float(np.median(rel_sens)) if rel_sens else None, "pairs": len(rel), "lost": lost,
            "over_pressed": over, "proximal_touched": prox, "pad_table": padtab, "candidates": len(keep),
            "pen_max_mm": float(max(pen)) if pen else None, "pen_median_mm": float(np.median(pen)) if pen else None,
            "proximal_clearance_min_mm": float(min(clear)) if clear else None}


def cmd_analyze(a) -> int:
    from analyze import DESIGN, spearman_perm

    keep, excl, o = o_pairs(a.tag, a.metric)
    d_o = np.array(list(keep.values()))
    print(f"O: {len(keep)}/{len(o[MU[0]])} draws survive; excluded {excl}; D(O) {d_o.min():.2f} .. {d_o.max():.2f} mm "
          f"(median {np.median(d_o):.2f}; negative {int((d_o < 0).sum())})")
    o_pen = max(max(o[m][k]["distal_force_max_n"] for m in MU) for k in keep) / POINT_K
    o_clear = min(min(o[m][k]["proximal_clearance_min_mm"] for m in MU) for k in keep)
    print(f"O on surviving draws: pen max {o_pen:.2f} mm, proximal clearance min {o_clear:.1f} mm")
    rows = []
    for hand, (dc, ds) in DESIGN.items():
        if a.hands and hand not in a.hands:
            continue
        r = hand_error(hand, a.tag, keep, a.metric)
        if r is None or r["E"] is None:
            continue
        rows.append({"hand": hand, "d_contact": dc, "d_skel": ds, **r})
        print(f"{hand:22s} D_c {dc:4.2f} D_s {ds:5.2f} | E {100 * r['E']:6.1f}% (over-pressed as max "
              f"{100 * r['E_over_as_max']:6.1f}%) {r['pairs']}/{r['candidates']} pairs; lost {r['lost']}, over-pressed "
              f"{r['over_pressed']}, proximal {r['proximal_touched']}, pad-table {r['pad_table']} | pen max "
              f"{r['pen_max_mm']:.2f} median {r['pen_median_mm']:.2f} mm, prox clear {r['proximal_clearance_min_mm']:.1f}"
              , flush=True)
    out = {"tag": a.tag, "metric": a.metric, "o_survive": len(keep), "o_excluded": excl, "o_pen_max_mm": o_pen,
           "o_proximal_clearance_min_mm": o_clear, "d_o_mm": {str(k): v for k, v in keep.items()}, "rows": rows}
    if a.noise_tag:
        nz = hand_error(f"O@{a.noise_tag}", a.tag, keep, a.metric)
        out["noise_floor"] = {"tag": a.noise_tag, **nz}
        print(f"noise floor (O, command +0.05 deg): E {100 * nz['E']:.2f}% (75th pct {100 * nz['E_p75']:.2f}%, "
              f"{nz['pairs']} pairs, over-pressed {nz['over_pressed']})")
    if len(rows) >= 3:
        for key in ("E", "E_over_as_max"):
            rho, pv = spearman_perm([x["d_contact"] for x in rows], [x[key] for x in rows])
            rho_s, pv_s = spearman_perm([x["d_skel"] for x in rows], [x[key] for x in rows])
            out[key + "_rho"] = {"contact": [rho, pv], "skel": [rho_s, pv_s]}
            print(f"[{key}] rho(E, D_contact) {rho:.2f} (one-sided permutation p {pv:.4f}); rho(E, D_skel) {rho_s:.2f} "
                  f"(p {pv_s:.4f}); hands {len(rows)}")
    if a.out:
        a.out.write_text(json.dumps(out, indent=1))
    return 0


# ---------------------------------------------------------------- frames for the figure and the render agent
def cmd_frames(a) -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle

    pl = next(d for d in json.loads(a.draws.read_text())["grasps"] if d["draw"] == a.draw)
    opts = {"approach_s": 1.0, "sweep_s": 3.0, "hold_s": 1.0, "tether": K_TETHER}
    env = _run_env(("O", a.mu, pl, 2.5e-4, 0.0, opts, True))
    fr = env["drag"]["frames"]
    i_touch = next(i for i, f in enumerate(fr) if f["contact_point_mm"] is not None)
    i_mid = i_touch + (len(fr) - 30 - i_touch) // 2  # halfway through the dragging sweep
    picks = {"start": 0, "dragging": i_mid, "end": len(fr) - 1}
    geom = T.finger_geometry("O")
    pads = geom["pads"]
    face = pads["mount_offset_mm"] + pads["thickness_mm"]
    snap = {"task": "coin drag (experiment 30, tag e30)", "hand": "O", "mu": a.mu, "draw": pl,
            "frame": "world = proximal phalanx frame (mm): x along it from the MCP, z dorsal; MCP held at 0 deg; "
                     "the finger plane is y = 0; angles about +y (pitch = flexion q)",
            "pads": pads, "proximal_face_z_mm": -face, "coin": {"diameter_mm": pl["diam_mm"],
                                                                "thickness_mm": COIN_THICK_MM, "mass_kg": COIN_MASS_KG},
            "table": {"top_rest_z_mm": pl["z_top_mm"] - face - COIN_THICK_MM, "x_extent_from_coin_mm":
                      [-TABLE_BACK_MM, TABLE_FRONT_MM], "spring_n_m": K_TABLE},
            "tether_n_m": K_TETHER, "mu_table": MU_TABLE,
            "result": {k: v for k, v in env["drag"].items() if k not in ("frames", "trace")},
            "snapshots": {k: fr[i] for k, i in picks.items()}}
    (OUT / "coin_snapshots.json").write_text(json.dumps(snap, indent=1))

    fig, axs = plt.subplots(1, 3, figsize=(12, 3.6), sharey=True)
    a_, ap_, b_ = geom["a"], geom["ap"], geom["b"]
    for ax, (name, i) in zip(axs, picks.items()):
        f = fr[i]
        x, z, pitch = f["distal_pose_mm_rad"]
        R = np.array([[math.cos(pitch), math.sin(pitch)], [-math.sin(pitch), math.cos(pitch)]])

        def dl(p):  # distal frame (x, z) -> world
            return np.array([x, z]) + R @ np.asarray(p)
        ax.add_patch(Rectangle((pads["proximal_start_mm"], -face), pads["proximal_end_mm"] - pads["proximal_start_mm"],
                               pads["thickness_mm"], color="#c98", alpha=0.8))
        ax.plot([0, b_[0]], [0, 0], color="0.3", lw=3)
        t0, t1, m = pads["distal_start_mm"], pads["distal_end_mm"], pads["mount_offset_mm"]
        ax.add_patch(Polygon([dl(p) for p in ((t0, -m), (t1, -m), (t1, -face), (t0, -face))], color="#c98", alpha=0.8))
        ax.plot(*np.array([dl((t0, 0)), dl((t1, 0))]).T, color="0.3", lw=3)
        cx, cz, cp = f["coin_pose_mm_rad"]
        rr, ht = pl["diam_mm"] / 2, COIN_THICK_MM / 2
        Rc = np.array([[math.cos(cp), math.sin(cp)], [-math.sin(cp), math.cos(cp)]])
        ax.add_patch(Polygon([np.array([cx, cz]) + Rc @ np.array(p) for p in ((-rr, -ht), (rr, -ht), (rr, ht), (-rr, ht))],
                             color="#d4a017"))
        tz = f["table_z_mm"]
        ax.add_patch(Rectangle((pl["x_c_mm"] - TABLE_BACK_MM, tz - 4), TABLE_BACK_MM + TABLE_FRONT_MM, 4, color="0.75"))
        ax.axvline(pl["x_c_mm"], color="0.6", ls=":", lw=0.8)
        if f["contact_point_mm"]:
            p, fv = np.array(f["contact_point_mm"]), np.array(f["contact_force_on_coin_n"])
            ax.annotate("", p + 6 * fv, p, arrowprops=dict(arrowstyle="<-", color="C3"))
        ax.set_title(f"{name}: t {f['t']:.2f} s, q {f['q_deg']:.1f} deg, travel {f['travel_mm']:.1f} mm", fontsize=9)
        ax.set_aspect("equal")
        ax.set_xlim(20, 105)
        ax.set_ylim(-45, 8)
        ax.set_xlabel("x along the proximal phalanx (mm)")
    axs[0].set_ylabel("z dorsal (mm)")
    fig.suptitle(f"Coin drag, O, mu {a.mu}, draw {a.draw} (q_t {pl['q_t_deg']:.1f} deg, sweep {pl['sweep_deg']:.1f} "
                 f"deg, coin {pl['diam_mm']:.1f} mm)", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / "coin_frames.png", dpi=130)
    print(OUT / "coin_frames.png", OUT / "coin_snapshots.json", env["drag"]["travel_end_mm"])
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("draws")
    d.add_argument("--n", type=int, default=64)
    d.add_argument("--seed", type=int, default=30)
    d.add_argument("--out", type=Path, required=True)
    r = sub.add_parser("run")
    r.add_argument("--hand", required=True)
    r.add_argument("--mu", type=float, required=True)
    r.add_argument("--draws", type=Path, required=True)
    r.add_argument("--only", type=int, nargs="+", default=None, help="these draw indices")
    r.add_argument("--tag", required=True)
    r.add_argument("--delta_deg", type=float, default=0.0)
    r.add_argument("--time_step", type=float, default=2.5e-4)
    r.add_argument("--approach_s", type=float, default=1.0)
    r.add_argument("--sweep_s", type=float, default=3.0)
    r.add_argument("--hold_s", type=float, default=1.0)
    r.add_argument("--sweep_deg", type=float, default=None, help="override every draw's sweep (calibration)")
    r.add_argument("--tether", type=float, default=K_TETHER)
    r.add_argument("--foot_k", type=float, default=FOOT_K)
    r.add_argument("--spin_d", type=float, default=SPIN_D)
    r.add_argument("--workers", type=int, default=4)
    an = sub.add_parser("analyze")
    an.add_argument("--tag", required=True)
    an.add_argument("--noise_tag", default=None)
    an.add_argument("--metric", default=METRIC)
    an.add_argument("--hands", nargs="+", default=None)
    an.add_argument("--out", type=Path, default=None)
    f = sub.add_parser("frames")
    f.add_argument("--draws", type=Path, required=True)
    f.add_argument("--draw", type=int, default=0)
    f.add_argument("--mu", type=float, default=1.3)
    a = p.parse_args()
    if a.cmd == "draws":
        out = make_draws(a.n, a.seed)
        a.out.write_text(json.dumps({"seed": a.seed, "n": a.n, "rng": f"numpy.random.default_rng([{a.seed}, 30])",
                                     "ranges": RANDOM, "grasps": out}, indent=1))
        print(a.out, len(out))
        return 0
    return {"run": cmd_run, "analyze": cmd_analyze, "frames": cmd_frames}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main())
