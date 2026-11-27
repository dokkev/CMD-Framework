"""Experiment 35: the fingertip tips a block against a stop (experimental; docs/experimental/drake_pivot.md).

A non-prehensile task with an extrinsic contact. The planar CoDe index finger of two_pad_drake.py (imported read-only:
geometry, servo table, condition A's flexion impedance; the MCP held at 0, so the proximal phalanx is the world;
palm down over a table, gravity on, as the coin drag). Frame: x along the proximal phalanx from the MCP, z dorsal;
placements in the hand's proximal-face frame (world z = z_face - face), made on O.

- Table: a box on a vertical spring (K_TABLE, its and the block's weight carried by a constant force), so the press is
  set by the spring, not by driving into the pad. A stop (a low block fixed on the table) touches the block's
  palm-side bottom edge.
- Block (width w along x, height h, mass M): on the table through sphere feet at its bottom edges, free in x, z and
  in tilt about y, a torsional return spring k_r about its centre (a spring-return tilting block; without it the pad
  always sticks and the tilt does not depend on the pads' mu, as the untethered coin). Block-table mu MU_TABLE.
- Command (O's open loop on every hand, condition A): flexion 0 -> q_t (O's fingertip face point meets the block's
  top, edge_mm inside its distal edge) over approach_s, q_t -> q_t + sweep over sweep_s (the fingertip drags the
  top toward the palm, and the stop holds the bottom: the block tips), held hold_s.
- Measure: the block's tilt at the end (deg, + top toward the palm); D = tilt(2.0) - tilt(1.3).

    uv run python box_pivot.py draws --n 64 --seed 35 --out logs/pivot_e35.json [--ranges JSON]
    uv run python box_pivot.py run --hands O --draws logs/pivot_e35.json --tag e35 [--only 12] [--delta_deg 0.05]
    uv run python box_pivot.py check --tag e35 [--hands R U] [--show_d]
    uv run python box_pivot.py analyze --tag e35 --noise_tag e35n --out logs/e35_pivot.json
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

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
MU = (1.3, 2.0)
G = 9.81
PEN_MM, POINT_K = 1.5, T.POINT_STIFFNESS * 1e-3
PROX_TOUCH_N = 1e-3
TABLE_MASS_KG = 0.02
FOOT_MM, FOOT_K = 0.5, 1e8
STOP_H_MM, STOP_W_MM = 3.0, 6.0
DEPTH_MM = 30.0  # the block's extent across the finger (y)
# experiment 35's distributions (fixed on O before any redesigned-hand data): independent uniforms drawn as vectors of
# N in this order from numpy.random.default_rng([seed, 35])
RANDOM = {"q_t_deg": (25.0, 40.0), "w_mm": (18.0, 26.0), "h_mm": (14.0, 22.0), "edge_mm": (2.0, 5.0),
          "sweep_deg": (20.0, 30.0), "k_r": (0.02, 0.04), "mass_g": (20.0, 40.0)}
DEFAULTS = {"approach_s": 1.0, "sweep_s": 3.0, "hold_s": 1.0, "k_table": 60.0, "d_table": 2.0, "mu_table": 0.5,
            "d_spin": 2e-4, "time_step": 2.5e-4}


def make_draws(n, seed, ranges=RANDOM):
    rng = np.random.default_rng([seed, 35])
    cols = {k: rng.uniform(lo, hi, n) for k, (lo, hi) in ranges.items()}
    g = T.finger_geometry("O")
    qs, th = T.servo_table(g)
    out = []
    for i in range(n):
        d = {k: float(v[i]) for k, v in cols.items()}
        tip = T.reference_point(g, qs, th, math.radians(d["q_t_deg"]), "tip")  # face frame
        # the block's top at the tip's depth; its distal edge edge_mm beyond the tip
        x_c = float(tip[0] + d["edge_mm"] - d["w_mm"] / 2)
        out.append({"draw": i, **d, "x_c_mm": x_c, "z_top_mm": float(tip[1])})
    return out


def pair_mu(mu_obj, target):
    return target * mu_obj / (2 * mu_obj - target)


def build(hand, mu, pl, opts, k_theta, d_theta):
    from pydrake.geometry import (AddContactMaterial, Box, CollisionFilterDeclaration, GeometrySet,
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
    plant, sg = AddMultibodyPlantSceneGraph(builder, time_step=opts["time_step"])
    plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
    plant.set_contact_model(ContactModel.kPoint)
    world = plant.world_body()
    y = np.array([0.0, 1.0, 0.0])
    zax = np.array([0.0, 0.0, 1.0])
    at = lambda p2: np.array([p2[0], 0.0, p2[1]])
    rod = lambda mass, L: SpatialInertia(mass, np.zeros(3), UnitInertia.SolidBox(max(L, 1e-3), 0.004, 0.004))

    a, ap, b = g["a"] * mm, g["ap"] * mm, g["b"] * mm
    driver = plant.AddRigidBody("driver", rod(0.003, float(np.linalg.norm(b - ap))))
    distal = plant.AddRigidBody("distal", rod(0.01, 0.03))
    f_ap = plant.AddFrame(FixedOffsetFrame("at_ap", world.body_frame(), RigidTransform(at(ap))))
    f_b = plant.AddFrame(FixedOffsetFrame("b_on_driver", driver.body_frame(), RigidTransform(at(b - ap))))
    servo = plant.AddJoint(RevoluteJoint("servo", f_ap, driver.body_frame(), y))
    plant.AddJointActuator("servo", servo).set_controller_gains(PdControllerGains(p=k_theta, d=d_theta))
    plant.AddJoint(RevoluteJoint("pip_b", f_b, distal.body_frame(), y))
    proximal = plant.AddRigidBody("proximal", rod(0.039, 0.044))
    plant.WeldFrames(world.body_frame(), proximal.body_frame())
    plant.AddDistanceConstraint(proximal, at(a), distal, at(g["bp_local"] * mm), g["abp"] * mm)

    W, H, M = pl["w_mm"] * mm, pl["h_mm"] * mm, pl["mass_g"] * 1e-3
    x_c = pl["x_c_mm"] * mm
    # the placement's frame: the hand's own proximal face (anchor 'face', experiment 35), or O's, i.e. O's coordinates
    # in F_1 (the link-line frame, the world here, the MCP held) on every hand (anchor 'f1'; docs/experimental/pr_mr.md)
    z_top = pl["z_top_mm"] * mm - (face if opts.get("anchor", "face") == "face" else T.face_depth("O") * mm)
    z_table = z_top - H  # the table's top (world)
    t_len, t_h = 0.08, 0.01
    table = plant.AddRigidBody("table", SpatialInertia(TABLE_MASS_KG, np.zeros(3),
                                                       UnitInertia.SolidBox(t_len, 0.04, t_h)))
    f_tb = plant.AddFrame(FixedOffsetFrame("table_base", world.body_frame(), RigidTransform([x_c, 0.0, z_table])))
    tj = plant.AddJoint(PrismaticJoint("table_z", f_tb, table.body_frame(), zax))
    plant.AddJointActuator("table_z", tj).set_controller_gains(PdControllerGains(p=opts["k_table"], d=opts["d_table"]))
    bx = plant.AddRigidBody("blk_x", rod(1e-4, 0.01))
    bz = plant.AddRigidBody("blk_z", rod(1e-4, 0.01))
    blk = plant.AddRigidBody("block", SpatialInertia(M, np.zeros(3), UnitInertia.SolidBox(W, DEPTH_MM * mm, H)))
    f_bb = plant.AddFrame(FixedOffsetFrame("blk_base", world.body_frame(), RigidTransform([x_c, 0.0, z_table + H / 2])))
    plant.AddJoint(PrismaticJoint("blk_x", f_bb, bx.body_frame(), [1.0, 0.0, 0.0]))
    plant.AddJoint(PrismaticJoint("blk_z", bx.body_frame(), bz.body_frame(), zax))
    tilt = plant.AddJoint(RevoluteJoint("tilt", bz.body_frame(), blk.body_frame(), y))
    plant.AddJointActuator("tilt", tilt).set_controller_gains(PdControllerGains(p=pl["k_r"], d=opts["d_spin"]))

    def props(k, mu_):
        pr = ProximityProperties()
        AddContactMaterial(point_stiffness=k, friction=CoulombFriction(mu_, mu_), properties=pr)
        return pr

    s0, s1 = pads["proximal_start_mm"] * mm, pads["proximal_end_mm"] * mm
    pid = plant.RegisterCollisionGeometry(proximal, RigidTransform(at(((s0 + s1) / 2, -(m + h / 2)))),
                                          Box(s1 - s0, w, h), "proximal_pad", props(T.POINT_STIFFNESS, mu))
    t0, t1 = pads["distal_start_mm"] * mm, pads["distal_end_mm"] * mm
    md = pads["distal_mount_mm"] * mm  # the distal pad's mount offset (m for the studies' hands)
    did = plant.RegisterCollisionGeometry(distal, RigidTransform(at(((t0 + t1) / 2, -(md + h / 2)))),
                                          Box(t1 - t0, w, h), "distal_pad", props(T.POINT_STIFFNESS, mu))
    blk_id = plant.RegisterCollisionGeometry(blk, RigidTransform(), Box(W, DEPTH_MM * mm, H), "block", props(1e8, mu))
    fr = FOOT_MM * mm
    mt = pair_mu(mu, opts["mu_table"])
    feet = [plant.RegisterCollisionGeometry(blk, RigidTransform([sx * (W / 2 - fr), sy * (DEPTH_MM * mm / 2 - fr),
                                                                 -H / 2 + fr]), Sphere(fr), f"foot{k}", props(FOOT_K, mt))
            for k, (sx, sy) in enumerate((sx, sy) for sx in (-1, 1) for sy in (-1, 1))]
    tab_id = plant.RegisterCollisionGeometry(table, RigidTransform([0.0, 0.0, -t_h / 2]), Box(t_len, 0.06, t_h),
                                             "table", props(1e8, mt))
    # the stop: on the table, touching the block's palm-side face at its bottom
    stop_id = plant.RegisterCollisionGeometry(
        table, RigidTransform([-W / 2 - STOP_W_MM * mm / 2, 0.0, STOP_H_MM * mm / 2]),
        Box(STOP_W_MM * mm, 0.06, STOP_H_MM * mm), "stop", props(1e8, mt))
    plant.Finalize()
    cfm = sg.collision_filter_manager()
    cfm.Apply(CollisionFilterDeclaration().ExcludeBetween(GeometrySet([blk_id]), GeometrySet([tab_id])))
    cfm.Apply(CollisionFilterDeclaration().ExcludeBetween(GeometrySet(feet), GeometrySet([pid, did, stop_id])))
    plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, -G])
    return builder, plant, sg, {"distal": distal, "proximal": proximal, "block": blk, "table": table}


class Run:
    def __init__(self, hand, mu, pl, opts, k_theta, d_theta):
        from pydrake.systems.analysis import Simulator
        from pydrake.systems.framework import LeafSystem

        builder, plant, sg, bodies = build(hand, mu, pl, opts, k_theta, d_theta)
        self.plant, self.bodies = plant, bodies
        self.theta_cmd = 0.0
        nv, n_act = plant.num_velocities(), plant.num_actuators()
        iz = plant.GetJointByName("table_z").velocity_start()
        carried = (TABLE_MASS_KG + pl["mass_g"] * 1e-3) * G
        outer = self

        class Command(LeafSystem):
            def __init__(self):
                super().__init__()
                self.DeclareVectorOutputPort("desired", 2 * n_act, self.calc_desired)
                self.DeclareVectorOutputPort("load", nv, self.calc_load)

            def calc_desired(self, context, out):  # servo, table_z, tilt
                out.SetFromVector(np.array([outer.theta_cmd, 0.0, 0.0] + [0.0] * n_act))

            def calc_load(self, context, out):
                tau = np.zeros(nv)
                tau[iz] = carried
                out.SetFromVector(tau)

        cmd = builder.AddSystem(Command())
        builder.Connect(cmd.get_output_port(0),
                        plant.get_desired_state_input_port(plant.GetModelInstanceByName("DefaultModelInstance")))
        builder.Connect(cmd.get_output_port(1), plant.get_applied_generalized_force_input_port())
        self.diagram = builder.Build()
        self.sim = Simulator(self.diagram)
        self.pctx = plant.GetMyMutableContextFromRoot(self.sim.get_mutable_context())
        plant.get_actuation_input_port().FixValue(self.pctx, np.zeros(n_act))
        self.idx = {k: v.index() for k, v in bodies.items()}
        self.t = 0.0

    def step(self):
        self.t += T.DT_ENV
        self.sim.AdvanceTo(self.t)

    def contacts(self):
        res = self.plant.get_contact_results_output_port().Eval(self.pctx)
        name = {v: k for k, v in self.idx.items()}
        out = {"distal_block": 0.0, "proximal_any": 0.0, "pad_table": 0.0, "distal_pt": None, "distal_f": np.zeros(3)}
        for k in range(res.num_point_pair_contacts()):
            c = res.point_pair_contact_info(k)
            pair = {name.get(c.bodyA_index()), name.get(c.bodyB_index())}
            fn = abs(float(c.contact_force() @ c.point_pair().nhat_BA_W))
            if pair == {"distal", "block"}:
                out["distal_block"] += fn
                out["distal_pt"] = c.contact_point()
            elif "proximal" in pair:
                out["proximal_any"] += fn
            elif pair == {"distal", "table"}:
                out["pad_table"] += fn
        return out

    def pose(self, body):
        X = self.plant.EvalBodyPoseInWorld(self.pctx, self.bodies[body])
        R = X.rotation().matrix()
        return np.array([X.translation()[0], X.translation()[2], math.atan2(R[0, 2], R[0, 0])])


def pivot(hand, mu, pl, opts, delta_deg=0.0, record=False):
    geom = T.finger_geometry(hand)
    qs, th = T.servo_table(geom)
    q_t = math.radians(pl["q_t_deg"] + delta_deg)
    ratio = float(np.interp(q_t, qs, np.gradient(th, qs)))
    k_theta, d_theta = T.K_Q / ratio ** 2, T.D_Q / ratio ** 2
    s = Run(hand, mu, pl, opts, k_theta, d_theta)
    s.sim.Initialize()
    tilt = s.plant.GetJointByName("tilt")
    W2, H2 = pl["w_mm"] / 2, pl["h_mm"] / 2

    def corner():
        """The block's palm-side bottom corner relative to the table's top (mm): it pivots against the stop."""
        bp, tp = s.pose("block"), s.pose("table")
        c, sn = math.cos(bp[2]), math.sin(bp[2])
        dx, dz = -W2, -H2  # body frame (x, z); rotation about +y: x' = c x + s z, z' = -s x + c z
        return np.array([1e3 * bp[0] + c * dx + sn * dz, 1e3 * (bp[1] - tp[1]) - sn * dx + c * dz])
    corner0 = corner()
    q1 = q_t + math.radians(pl["sweep_deg"])
    n_app, n_sw, n_hold = (int(round(opts[k] / T.DT_ENV)) for k in ("approach_s", "sweep_s", "hold_s"))
    fd_max = prox = padtab = 0.0
    touch = None
    trace, frames = [], []
    for i in range(n_app + n_sw + n_hold):
        q = q_t * (i + 1) / n_app if i < n_app else q_t + (q1 - q_t) * min(1.0, (i + 1 - n_app) / n_sw)
        s.theta_cmd = float(np.interp(q, qs, th))
        s.step()
        c = s.contacts()
        fd_max, prox, padtab = max(fd_max, c["distal_block"]), max(prox, c["proximal_any"]), max(padtab, c["pad_table"])
        if touch is None and c["distal_block"] > T.GATE_N:
            touch = math.degrees(q)
        ang = -math.degrees(tilt.get_angle(s.pctx))  # + top toward the palm (-x)
        if i >= n_app:
            trace.append([round(ang, 4), round(c["distal_block"], 4)])
        if record:
            bp, dp = s.pose("block"), s.pose("distal")
            pt = c["distal_pt"]
            frames.append({"t": s.t, "q_cmd_deg": math.degrees(q), "q_deg": math.degrees(dp[2]),
                           "block_mm_rad": [1e3 * bp[0], 1e3 * bp[1], bp[2]], "table_z_mm": 1e3 * s.pose("table")[1],
                           "tilt_deg": ang, "f_distal": c["distal_block"],
                           "contact_mm": None if pt is None else [1e3 * pt[0], 1e3 * pt[2]]})
    cn = corner()
    out = {"tilt_deg": -math.degrees(tilt.get_angle(s.pctx)), "tilt_max_deg": max(t[0] for t in trace),
           "slide_mm": float(cn[0] - corner0[0]), "lift_mm": float(cn[1] - corner0[1]),
           "touched": touch is not None, "touch_q_deg": touch, "distal_force_max_n": fd_max,
           "proximal_force_max_n": prox, "pad_table_force_max_n": padtab, "pen_mm": fd_max / POINT_K,
           "distal_force_end_n": s.contacts()["distal_block"], "trace": trace}
    if record:
        out["frames"] = frames
    return out


def run_task(task):
    hand, pl, opts, delta = task
    t0 = time.time()
    rec = {"draw": pl["draw"], "hand": hand}
    for mu in MU:
        try:
            rec[f"{mu:g}"] = pivot(hand, mu, pl, opts, delta)
        except RuntimeError as err:
            rec[f"{mu:g}"] = {"solver_fail": str(err)[:200]}
    rec["wall_s"] = time.time() - t0
    return rec


def _opts(dfile, a=None):
    o = {**DEFAULTS, **dfile.get("config", {})}
    if a is not None and getattr(a, "config", None):
        o.update(json.loads(a.config))
    return o


def cmd_run(a) -> int:
    d = json.loads(a.draws.read_text())
    draws = d["draws"][: a.only] if a.only else d["draws"]
    opts = _opts(d, a)
    if a.anchor != "face":  # only then recorded, so the face-anchored logs' opts stay as they were
        opts["anchor"] = a.anchor
    tasks = [(h, pl, opts, a.delta_deg) for h in a.hands for pl in draws]
    t0 = time.time()
    with Pool(a.workers) as pool:
        recs = pool.map(run_task, tasks, chunksize=1)
    for h in a.hands:
        rr = [r for r in recs if r["hand"] == h]
        (LOGS / f"pivot_{h}_{a.tag}.json").write_text(json.dumps({"hand": h, "tag": a.tag, "opts": opts,
                                                                   "delta_deg": a.delta_deg, "draws": rr}))
        print(f"{h} {a.tag}: {len(rr)} draws, cpu {sum(r['wall_s'] for r in rr):.0f} s", flush=True)
    print(f"done in {time.time() - t0:.0f} s (workers {a.workers})", flush=True)
    return 0


# ----------------------------------------------------------------------------------------------------- analysis
MIN_D_DEG = 0.3
METRIC = "tilt_deg"  # the pre-registered metric (the end tilt); "tilt_max_deg" is the secondary one
TOPPLE_DEG = 45.0


def _load(hand, tag):
    p = LOGS / f"pivot_{hand}_{tag}.json"
    return {r["draw"]: r for r in json.loads(p.read_text())["draws"]} if p.exists() else None


def lost(x):
    """The task failed for this run: the pad never touched, lost the block at the end, or the block toppled
    (tilt past TOPPLE_DEG at any time) or its pivot corner left the stop (moved more than 1 mm toward the palm or
    rose more than the stop's height off the table)."""
    return (not x["touched"]) or x["distal_force_end_n"] < T.LOSS_N or x["tilt_max_deg"] > TOPPLE_DEG or \
        x["slide_mm"] < -1.0 or x["lift_mm"] > STOP_H_MM


def o_keep(tag, metric=None):
    metric = metric or METRIC
    o = _load("O", tag)
    keep, excl = {}, {"o_solver": 0, "o_lost": 0, "o_pen": 0, "o_proximal": 0, "o_pad_table": 0, "small_d": 0}
    for k, r in sorted(o.items()):
        runs = [r[f"{mu:g}"] for mu in MU]
        if any("solver_fail" in x for x in runs):
            excl["o_solver"] += 1
        elif any(lost(x) for x in runs):
            excl["o_lost"] += 1
        elif any(x["pen_mm"] > PEN_MM for x in runs):
            excl["o_pen"] += 1
        elif any(x["proximal_force_max_n"] > PROX_TOUCH_N for x in runs):
            excl["o_proximal"] += 1
        elif any(x["pad_table_force_max_n"] > PROX_TOUCH_N for x in runs):
            excl["o_pad_table"] += 1
        elif abs(runs[1][metric] - runs[0][metric]) < MIN_D_DEG:
            excl["small_d"] += 1
        else:
            keep[k] = runs
    return keep, excl


def hand_error(hand, tag, keep, metric=None):
    metric = metric or METRIC
    h = _load(hand, tag)
    if h is None:
        return None
    rel, sens, pens = [], [], []
    cnt = {"solver": 0, "lost": 0, "over_pressed": 0, "proximal": 0, "pad_table": 0}
    for k, o in keep.items():
        runs = [h[k][f"{mu:g}"] for mu in MU]
        if any("solver_fail" in x for x in runs):
            cnt["solver"] += 1
            sens.append(math.inf)
            continue
        if any(lost(x) for x in runs):
            cnt["lost"] += 1
            sens.append(math.inf)
            continue
        pen = max(x["pen_mm"] for x in runs)
        pens.append(pen)
        if pen > PEN_MM:
            cnt["over_pressed"] += 1
            sens.append(math.inf)
            continue
        cnt["proximal"] += any(x["proximal_force_max_n"] > PROX_TOUCH_N for x in runs)
        cnt["pad_table"] += any(x["pad_table_force_max_n"] > PROX_TOUCH_N for x in runs)
        d_o = o[1][metric] - o[0][metric]
        e = abs((runs[1][metric] - runs[0][metric]) - d_o) / abs(d_o)
        rel.append(e)
        sens.append(e)
    return {"E": float(np.median(rel)) if rel else None, "E_p75": float(np.percentile(rel, 75)) if rel else None,
            "pairs": len(rel), "E_sens": float(np.median(sens)) if sens else None, **cnt,
            "pen_max_mm": float(max(pens)) if pens else None}


def deviation(tag, keep):
    """R and U minus O, the block's tilt in time from the sweep's start, per mu, over O's kept draws."""
    o = _load("O", tag)
    out = {}
    n_min = None
    for hand in ("R", "U"):
        h = _load(hand, tag)
        for mu in MU:
            ys = []
            for k in keep:
                xh = h[k][f"{mu:g}"]
                if "solver_fail" in xh or lost(xh) or max(h[k][f"{m:g}"].get("pen_mm", 0) for m in MU) > PEN_MM:
                    continue
                a_, b_ = np.array(o[k][f"{mu:g}"]["trace"])[:, 0], np.array(xh["trace"])[:, 0]
                n = min(len(a_), len(b_))
                ys.append(b_[:n] - a_[:n])
            n = min(map(len, ys))
            n_min = n if n_min is None else min(n_min, n)
            y = np.array([v[:n] for v in ys])
            out[f"{hand}_{mu:g}"] = {"median": np.median(y, 0).round(4).tolist(),
                                     "q25": np.percentile(y, 25, 0).round(4).tolist(),
                                     "q75": np.percentile(y, 75, 0).round(4).tolist(), "n": len(ys)}
    out["x"] = [round((i + 1) * T.DT_ENV, 4) for i in range(n_min)]
    return out


def cmd_analyze(a) -> int:
    from analyze import DESIGN, spearman_perm
    global METRIC
    METRIC = a.metric
    keep, excl = o_keep(a.tag)
    d_o = np.array([v[1][METRIC] - v[0][METRIC] for v in keep.values()])
    t_o = np.array([v[0]["tilt_deg"] for v in keep.values()])
    print(f"{a.tag}: O kept {len(keep)}, excluded {excl}; D(O) {d_o.min():.2f}/{np.median(d_o):.2f}/{d_o.max():.2f} "
          f"deg (negative {int((d_o < 0).sum())}); tilt(O, mu 1.3) {t_o.min():.1f}/{np.median(t_o):.1f}/{t_o.max():.1f}")
    out = {"tag": a.tag, "metric": METRIC, "o_kept": len(keep), "o_excluded": excl, "d_o_deg": d_o.tolist(),
           "rows": []}
    if a.noise_tag:
        nz = hand_error("O", a.noise_tag, keep)
        out["noise_floor"] = nz
        print(f"noise floor ({a.noise_tag}): E {100 * nz['E']:.2f}% (p75 {100 * nz['E_p75']:.2f}%), pairs {nz['pairs']}")
    for hand, (dc, ds) in DESIGN.items():
        r = hand_error(hand, a.tag, keep)
        if r is None:
            continue
        out["rows"].append({"hand": hand, "d_contact": dc, "d_skel": ds, **r})
        print(f"{hand:22s} Dc {dc:4.2f} Ds {ds:5.2f} | E {'-' if r['E'] is None else f'{100 * r[chr(69)]:5.1f}%'} | "
              f"pairs {r['pairs']}/{len(keep)} lost {r['lost']} over {r['over_pressed']} prox {r['proximal']} "
              f"padtab {r['pad_table']} | pen max {r['pen_max_mm'] or 0:.2f}", flush=True)
    rows = [r for r in out["rows"] if r["E"] is not None]
    if len(rows) >= 3:
        st = {}
        for key in ("E", "E_sens"):
            for cand in ("d_contact", "d_skel"):
                rho, p = spearman_perm([r[cand] for r in rows], [r[key] for r in rows])
                st[f"rho_{key}_{cand}"], st[f"p_{key}_{cand}"] = rho, p
            print(f"rho({key}, D_contact) {st[f'rho_{key}_d_contact']:.2f} (permutation p "
                  f"{st[f'p_{key}_d_contact']:.4f}); rho({key}, D_skel) {st[f'rho_{key}_d_skel']:.2f} "
                  f"(p {st[f'p_{key}_d_skel']:.4f}); hands {len(rows)}")
        out["stats"] = st
    if _load("R", a.tag) and _load("U", a.tag):
        out["deviation"] = deviation(a.tag, keep)
        for k in ("R_1.3", "U_1.3", "R_2", "U_2"):
            m = np.array(out["deviation"][k]["median"])
            i = int(np.argmax(np.abs(m)))
            print(f"deviation {k}: peak {m[i]:+.3f} deg at {out['deviation']['x'][i]:.2f} s, end {m[-1]:+.3f} deg, "
                  f"n {out['deviation'][k]['n']}")
    if a.out:
        a.out.write_text(json.dumps(out, indent=1, default=float))
    return 0


def cmd_check(a) -> int:
    for hand in a.hands:
        h = _load(hand, a.tag)
        rows = [h[k] for k in sorted(h)]
        fail = sum(any("solver_fail" in r[f"{mu:g}"] for mu in MU) for r in rows)
        ok = [r for r in rows if not any("solver_fail" in r[f"{mu:g}"] for mu in MU)]
        ls = sum(any(lost(r[f"{mu:g}"]) for mu in MU) for r in ok)
        pen = [max(r[f"{mu:g}"]["pen_mm"] for mu in MU) for r in ok]
        prox = sum(any(r[f"{mu:g}"]["proximal_force_max_n"] > PROX_TOUCH_N for mu in MU) for r in ok)
        pt = sum(any(r[f"{mu:g}"]["pad_table_force_max_n"] > PROX_TOUCH_N for mu in MU) for r in ok)
        line = (f"{hand} {a.tag}: {len(rows)} draws, solver {fail}, lost {ls}, proximal {prox}, pad-table {pt}, pen max "
                f"{max(pen):.2f} mm (> 1.5: {sum(p > PEN_MM for p in pen)}), median {np.median(pen):.2f}")
        if a.show_d:
            good = [r for r in ok if not any(lost(r[f"{mu:g}"]) for mu in MU)]
            dd = np.array([r["2"]["tilt_deg"] - r["1.3"]["tilt_deg"] for r in good])
            tt = np.array([r["1.3"]["tilt_deg"] for r in good])
            sl = np.array([r["1.3"]["slide_mm"] for r in good])
            line += (f"; tilt(1.3) {tt.min():.1f}/{np.median(tt):.1f}/{tt.max():.1f}; D {dd.min():.2f}/{np.median(dd):.2f}/"
                     f"{dd.max():.2f} deg (|D|<0.3: {int((abs(dd) < 0.3).sum())}); slide {sl.min():.1f}..{sl.max():.1f} mm")
        print(line)
        if a.verbose:
            for r in rows:
                x, y_ = r["1.3"], r["2"]
                if "solver_fail" in x or "solver_fail" in y_:
                    print(f"  draw {r['draw']}: solver fail")
                    continue
                print(f"  draw {r['draw']:2d}: tilt {x['tilt_deg']:6.2f}/{y_['tilt_deg']:6.2f} max {x['tilt_max_deg']:6.2f}"
                      f" slide {x['slide_mm']:+.2f} lift {x['lift_mm']:+.2f} pen {x['pen_mm']:.2f}/{y_['pen_mm']:.2f} "
                      f"fend {x['distal_force_end_n']:.2f} touch {x['touch_q_deg']} lost {lost(x)},{lost(y_)}")
    return 0


def cmd_frames(a) -> int:
    d = json.loads(a.draws.read_text())
    pl = next(x for x in d["draws"] if x["draw"] == a.draw)
    out = pivot(a.hand, a.mu, pl, _opts(d), record=True)
    p = LOGS / f"pivot_frames_{a.hand}_d{a.draw}.json"
    p.write_text(json.dumps({"hand": a.hand, "mu": a.mu, "placement": pl, "opts": _opts(d), **out}))
    print(f"tilt {out['tilt_deg']:.2f} deg -> {p.name}")
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    pd_ = sub.add_parser("draws")
    pd_.add_argument("--n", type=int, default=64)
    pd_.add_argument("--seed", type=int, default=35)
    pd_.add_argument("--out", type=Path, required=True)
    pd_.add_argument("--ranges", default=None)
    pd_.add_argument("--config", default=None)
    pr = sub.add_parser("run")
    pr.add_argument("--hands", nargs="+", default=["O"])
    pr.add_argument("--draws", type=Path, required=True)
    pr.add_argument("--tag", required=True)
    pr.add_argument("--only", type=int, default=None)
    pr.add_argument("--delta_deg", type=float, default=0.0)
    pr.add_argument("--config", default=None)
    pr.add_argument("--workers", type=int, default=6)
    pr.add_argument("--anchor", choices=("face", "f1"), default="face",
                    help="place the block in the hand's proximal-face frame (face) or at O's coordinates in F_1 (f1)")
    pa = sub.add_parser("analyze")
    pa.add_argument("--tag", required=True)
    pa.add_argument("--noise_tag", default=None)
    pa.add_argument("--out", type=Path, default=None)
    pa.add_argument("--metric", default="tilt_deg", choices=("tilt_deg", "tilt_max_deg"))
    pc = sub.add_parser("check")
    pc.add_argument("--tag", required=True)
    pc.add_argument("--hands", nargs="+", default=["O"])
    pc.add_argument("--show_d", action="store_true")
    pc.add_argument("--verbose", action="store_true")
    pf = sub.add_parser("frames")
    pf.add_argument("--draws", type=Path, required=True)
    pf.add_argument("--draw", type=int, default=0)
    pf.add_argument("--hand", default="O")
    pf.add_argument("--mu", type=float, default=1.3)
    a = p.parse_args()
    if a.cmd == "draws":
        ranges = {**RANDOM, **(json.loads(a.ranges) if a.ranges else {})}
        draws = make_draws(a.n, a.seed, ranges)
        cfg = {**DEFAULTS, **(json.loads(a.config) if a.config else {})}
        a.out.write_text(json.dumps({"seed": a.seed, "n": a.n, "rng": f"numpy.random.default_rng([{a.seed}, 35])",
                                     "ranges": ranges, "config": cfg, "draws": draws}, indent=1))
        print(f"{len(draws)} draws -> {a.out}")
        return 0
    return {"run": cmd_run, "analyze": cmd_analyze, "check": cmd_check, "frames": cmd_frames}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main())
