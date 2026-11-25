"""Experiment 36: pinch peg-in-hole by PIP extension (experimental; docs/experimental/drake_peg.md).

The peg is pinched between the index distal pad and the real thumb pad at the 33b pose (pinch3_pose.py: thumb_ro -90,
raa 10, mcp 45, ip -7.5 deg; index FE 72.5 deg), the thumb moving only along its pad normal with its servo compliance
(pinch3_pose: every thumb servo's XM335 position loop through its lever arm, 1525 N/m). One planar index finger
(two_pad_drake.py read-only: geometry, servo table, condition A's impedance; the proximal phalanx is the world, MCP
fixed at the pose). At this pose PIP extension moves the distal pad's contact toward the thumb (0.2-0.5 mm/deg) and
along the thumb's face (0.33-0.54 mm/deg, distal and dorsal in the finger frame): the index drives the pinched peg
along the thumb pad, out in front of the pinch, into a chamfered slot fixed relative to the palm.

- Peg: a capsule (radius r, length L) with its axis along the thumb face (direction u, the extension's sliding
  direction), its side touching both pads. Planar: it slides along u and across (the thumb normal) and tilts about y.
  A spring k_seat along u (the slot's spring-loaded seat; without a resistance the pad's friction would not change
  the outcome, as the untethered coin) resists its travel.
- Slot: two walls parallel to u, clearance c each side of the peg, entrance gap_mm ahead of the peg's leading end
  (placed on O's own peg path: the slot axis is O's pinch line), each wall with a 45 deg chamfer at the entrance;
  peg-wall mu MU_WALL. The thumb pad does not collide with the walls (the slot is in front of the thumb's tip).
- Command (O's open loop on every hand, condition A): the peg held while the finger closes 0 -> q_s (1 s) and holds
  0.5 s (the thumb's preload squeezes it); released 0.5 s; then q_s -> q_s - sweep (PIP extension) over sweep_s, held
  hold_s. Gains at q_s.
- Measure: the insertion depth (mm) of the peg's leading end past the slot's entrance at the end; D = depth(2.0) -
  depth(1.3). Lost: either pad's normal force on the peg < LOSS_N at the end or for 3 samples running; jam is not a
  failure (it is the outcome). Penetration: largest pad normal force / its stiffness.

    uv run python peg_pinch.py draws --n 64 --seed 36 --out logs/peg_e36.json [--ranges JSON]
    uv run python peg_pinch.py run --hands O --draws logs/peg_e36.json --tag e36 [--only 12]
    uv run python peg_pinch.py check --tag e36 [--hands R U] [--show_d] [--verbose]
    uv run python peg_pinch.py analyze --tag e36 --noise_tag e36n --out logs/e36_peg.json
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
PEN_MM = 1.5
POINT_K = T.POINT_STIFFNESS * 1e-3
PROX_TOUCH_N = 1e-3
LOST_RUN = 3
MU_WALL = 0.3
WALL_LEN_MM, WALL_T_MM, CHAMFER_MM = 16.0, 3.0, 3.0
PEG_MASS_KG = 0.01
THUMB_LEN_MM, THUMB_THICK_MM = 30.0, 3.0
D_THUMB = 0.5
HOLD_N_FACTOR = 1.25
PAD_CENTRE_X = -6.5
# experiment 36's distributions (fixed on O before any redesigned-hand data): independent uniforms drawn as vectors of
# N in this order from numpy.random.default_rng([seed, 36])
RANDOM = {"q_s_deg": (12.0, 22.0), "sweep_deg": (6.0, 10.0), "r_mm": (3.0, 4.5), "L_mm": (20.0, 26.0),
          "preload_n": (0.4, 0.8), "k_seat": (40.0, 80.0), "clear_mm": (0.3, 0.8), "gap_mm": (0.5, 2.0),
          "jitter_mm": (-1.0, 1.0)}
DEFAULTS = {"approach_s": 1.0, "settle_s": 0.5, "release_s": 0.5, "sweep_s": 2.0, "hold_s": 0.5, "fe_deg": 72.5,
            "time_step": 2.5e-4}


def make_draws(n, seed, ranges=RANDOM, fe_deg=72.5):
    import pinch3_pose as PP
    rng = np.random.default_rng([seed, 36])
    cols = {k: rng.uniform(lo, hi, n) for k, (lo, hi) in ranges.items()}
    g = T.finger_geometry("O")
    qs, th = T.servo_table(g)
    pose = PP.thumb_in_plane(fe_deg)
    nt = np.array(pose["thumb_normal_in_plane"])  # the thumb pad's outward normal (toward the index)
    u = -np.array([-nt[1], nt[0]])  # the insertion direction: along the thumb face, distal and dorsal
    out = []
    for i in range(n):
        d = {k: float(v[i]) for k, v in cols.items()}
        q = math.radians(d["q_s_deg"])
        p = T.reference_point(g, qs, th, q, PAD_CENTRE_X + d["jitter_mm"])  # face frame
        n_i = T.rot(q) @ np.array([0.0, -1.0])
        # the peg's axis lies along u at distance r from the thumb face; its side touches the index pad at p:
        # centre c with (c - p) . n_i = r (touching the index face) and c = thumb point + r nt, the thumb face line
        # through FK's face point. Solve: c = p + r n_i + s u for s such that c lies r off the thumb face line.
        fk = np.array(pose["thumb_face_mm"]) + np.array([0.0, 6.0])  # face frame
        # distance of a point x from the thumb face line along nt: (x - fk) . nt; want r
        # move along u (keeps the distance to the index contact's tangent unchanged only approximately; the index
        # face is not parallel): put the peg centre where it is r from the thumb face and r from the index face
        # the index face is inclined to the axis (the opposition angle), so it touches the capsule's nearer end:
        # that end's centre e = c - sigma L/2 u lies r off the index face, the axis r off the thumb face
        sigma = 1.0 if (u @ n_i) > 0 else -1.0
        a = np.array([n_i, nt])
        rhs = np.array([d["r_mm"] + p @ n_i + sigma * (d["L_mm"] / 2) * (u @ n_i), d["r_mm"] + fk @ nt])
        c = np.linalg.solve(a, rhs)
        thumb_c = c - d["r_mm"] * nt  # the thumb face's contact point
        lead = c + (d["L_mm"] / 2) * u  # the peg's leading end (centre of its cap)
        entry = lead + (d["r_mm"] + d["gap_mm"]) * u  # the slot's entrance, on the peg's axis line
        out.append({"draw": i, **d, "fe_deg": fe_deg, "c_mm": c.tolist(), "u": u.tolist(), "n": (-nt).tolist(),
                    "thumb_mm": thumb_c.tolist(), "entry_mm": entry.tolist(),
                    "thumb_shift_mm": float((thumb_c - fk) @ nt), "index_end": "rear" if sigma > 0 else "front",
                    "index_contact_along_pad_mm": float(((c - sigma * (d["L_mm"] / 2) * u) - p)
                                                        @ (T.rot(q) @ np.array([1.0, 0.0]))),
                    "opposition_deg": float(math.degrees(math.acos(float(np.clip(-(nt @ n_i), -1, 1)))))})
    return out, pose


def build(hand, mu, pl, opts, k_theta, d_theta, stage):
    from pydrake.geometry import (AddContactMaterial, Box, Capsule, CollisionFilterDeclaration, GeometrySet,
                                  ProximityProperties)
    from pydrake.math import RigidTransform, RotationMatrix
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

    u = np.array(pl["u"])
    nrm = np.array(pl["n"])  # from the peg toward the thumb (the thumb joint's + direction)
    to_w = lambda p_face: np.array(p_face) * mm - np.array([0.0, face])
    c, tp, entry = to_w(pl["c_mm"]), to_w(pl["thumb_mm"]), to_w(pl["entry_mm"])
    r, L = pl["r_mm"] * mm, pl["L_mm"] * mm

    # thumb: its frame's z along nrm (away from the peg), the face at its origin
    R_t = RotationMatrix.MakeYRotation(math.atan2(nrm[0], nrm[1]))
    thumb = plant.AddRigidBody("thumb", SpatialInertia(0.01, np.zeros(3), UnitInertia.SolidBox(0.03, 0.02, 0.003)))
    f_tb = plant.AddFrame(FixedOffsetFrame("thumb_base", world.body_frame(), RigidTransform(R_t, at(tp))))
    th_j = plant.AddJoint(PrismaticJoint("thumb_n", f_tb, thumb.body_frame(), [0.0, 0.0, 1.0]))
    plant.AddJointActuator("thumb_n", th_j).set_controller_gains(PdControllerGains(p=opts["k_thumb"], d=D_THUMB))

    # peg: slides along u (with the seat spring) and across (the thumb normal), tilts about y; its body x along u
    R_u = RotationMatrix.MakeYRotation(math.atan2(-u[1], u[0]))  # local x -> (u_x, 0, u_z)
    px = plant.AddRigidBody("peg_u", rod(1e-4, 0.01))
    pn = plant.AddRigidBody("peg_n", rod(1e-4, 0.01))
    peg = plant.AddRigidBody("peg", SpatialInertia(PEG_MASS_KG, np.zeros(3),
                                                   UnitInertia.SolidCylinder(r, L, [1.0, 0.0, 0.0])))
    f_pb = plant.AddFrame(FixedOffsetFrame("peg_base", world.body_frame(), RigidTransform(R_u, at(c))))
    ju = plant.AddJoint(PrismaticJoint("peg_u", f_pb, px.body_frame(), [1.0, 0.0, 0.0]))
    jn = plant.AddJoint(PrismaticJoint("peg_n", px.body_frame(), pn.body_frame(), [0.0, 0.0, 1.0]))
    jt = plant.AddJoint(RevoluteJoint("peg_tilt", pn.body_frame(), peg.body_frame(), y))
    if stage == "hold":
        plant.AddJointActuator("peg_u", ju).set_controller_gains(PdControllerGains(p=1e5, d=1e3))
        plant.AddJointActuator("peg_n", jn, effort_limit=HOLD_N_FACTOR * pl["preload_n"]).set_controller_gains(
            PdControllerGains(p=1e5, d=1e3))
        plant.AddJointActuator("peg_tilt", jt).set_controller_gains(PdControllerGains(p=10.0, d=0.1))
    else:
        plant.AddJointActuator("peg_u", ju).set_controller_gains(PdControllerGains(p=pl["k_seat"], d=0.5))

    def props(k, mu_):
        pr = ProximityProperties()
        AddContactMaterial(point_stiffness=k, friction=CoulombFriction(mu_, mu_), properties=pr)
        return pr

    s0, s1 = pads["proximal_start_mm"] * mm, pads["proximal_end_mm"] * mm
    pid = plant.RegisterCollisionGeometry(proximal, RigidTransform(at(((s0 + s1) / 2, -(m + h / 2)))),
                                          Box(s1 - s0, w, h), "proximal_pad", props(T.POINT_STIFFNESS, mu))
    t0, t1 = pads["distal_start_mm"] * mm, pads["distal_end_mm"] * mm
    did = plant.RegisterCollisionGeometry(distal, RigidTransform(at(((t0 + t1) / 2, -(m + h / 2)))),
                                          Box(t1 - t0, w, h), "distal_pad", props(T.POINT_STIFFNESS, mu))
    tt = THUMB_THICK_MM * mm
    tid = plant.RegisterCollisionGeometry(thumb, RigidTransform([0.0, 0.0, tt / 2]), Box(THUMB_LEN_MM * mm, w, tt),
                                          "thumb_pad", props(T.POINT_STIFFNESS, mu))
    # the capsule along its body x (Drake's capsule axis is its frame's z)
    peg_id = plant.RegisterCollisionGeometry(peg, RigidTransform(RotationMatrix.MakeYRotation(math.pi / 2)),
                                             Capsule(r, L), "peg", props(1e8, mu))
    # the slot: walls parallel to u on either side of the peg axis line, beyond the entrance; chamfers at the entry
    perp = np.array([-u[1], u[0]])  # in-plane normal of the axis line
    wl, wt, cl = WALL_LEN_MM * mm, WALL_T_MM * mm, pl["clear_mm"] * mm
    walls = []
    for sgn in (-1.0, 1.0):
        off = sgn * (r + cl + wt / 2)
        centre = entry + (wl / 2) * u + off * perp
        walls.append(plant.RegisterCollisionGeometry(world, RigidTransform(R_u, at(centre)),
                                                     Box(wl, 0.06, wt), f"wall{int(sgn)}", props(1e8, MU_WALL)))
        # the chamfer: a block turned 45 deg outward at the entrance
        ch = CHAMFER_MM * mm
        R_c = RotationMatrix.MakeYRotation(math.atan2(-u[1], u[0]) + sgn * math.radians(45.0))
        cc = entry - (ch / 2) * u + sgn * (r + cl + ch / 2) * perp + sgn * (wt / 2) * perp
        walls.append(plant.RegisterCollisionGeometry(world, RigidTransform(R_c, at(cc)),
                                                     Box(ch * 1.4, 0.06, wt), f"chamfer{int(sgn)}",
                                                     props(1e8, MU_WALL)))
    plant.Finalize()
    cfm = sg.collision_filter_manager()
    cfm.Apply(CollisionFilterDeclaration().ExcludeBetween(GeometrySet([tid]), GeometrySet(walls)))
    plant.mutable_gravity_field().set_gravity_vector(np.zeros(3))
    return builder, plant, sg, {"distal": distal, "proximal": proximal, "peg": peg, "thumb": thumb}, \
        {"walls": walls, "pads": [pid, did]}


class Run:
    def __init__(self, hand, mu, pl, opts, k_theta, d_theta, stage):
        from pydrake.systems.analysis import Simulator
        from pydrake.systems.framework import LeafSystem

        builder, plant, sg, bodies, self.geoms = build(hand, mu, pl, opts, k_theta, d_theta, stage)
        self.plant, self.bodies, self.sg = plant, bodies, sg
        self.theta_cmd = 0.0
        n_act = plant.num_actuators()
        rest = -pl["preload_n"] / opts["k_thumb"]
        outer = self

        class Command(LeafSystem):
            def __init__(self):
                super().__init__()
                self.DeclareVectorOutputPort("desired", 2 * n_act, self.calc)

            def calc(self, context, out):  # servo, thumb, then the peg's drives
                out.SetFromVector(np.array([outer.theta_cmd, rest] + [0.0] * (n_act - 2) + [0.0] * n_act))

        cmd = builder.AddSystem(Command())
        builder.Connect(cmd.get_output_port(0),
                        plant.get_desired_state_input_port(plant.GetModelInstanceByName("DefaultModelInstance")))
        self.diagram = builder.Build()
        self.sim = Simulator(self.diagram)
        self.pctx = plant.GetMyMutableContextFromRoot(self.sim.get_mutable_context())
        plant.get_actuation_input_port().FixValue(self.pctx, np.zeros(n_act))
        self.idx = {k: v.index() for k, v in bodies.items()}
        self.t = 0.0

    def step(self):
        self.t += T.DT_ENV
        self.sim.AdvanceTo(self.t)

    def forces(self):
        res = self.plant.get_contact_results_output_port().Eval(self.pctx)
        name = {v: k for k, v in self.idx.items()}
        out = {"distal": 0.0, "proximal": 0.0, "thumb": 0.0, "wall": 0.0, "pad_wall": 0.0}
        world = self.plant.world_body().index()
        for k in range(res.num_point_pair_contacts()):
            c = res.point_pair_contact_info(k)
            ba, bb = c.bodyA_index(), c.bodyB_index()
            fn = abs(float(c.contact_force() @ c.point_pair().nhat_BA_W))
            pair = {name.get(ba, "world" if ba == world else None), name.get(bb, "world" if bb == world else None)}
            if "peg" in pair:
                other = (pair - {"peg"}).pop()
                if other in ("distal", "proximal", "thumb"):
                    out[other] += fn
                elif other == "world":
                    out["wall"] += fn
            elif "world" in pair and pair & {"distal", "proximal"}:
                out["pad_wall"] += fn
        return out

    def peg(self):
        X = self.plant.EvalBodyPoseInWorld(self.pctx, self.bodies["peg"])
        R = X.rotation().matrix()
        return X.translation()[[0, 2]], math.atan2(R[0, 2], R[0, 0])


def peg_in(hand, mu, pl, opts, delta_deg=0.0, record=False):
    geom = T.finger_geometry(hand)
    qs, th = T.servo_table(geom)
    q_s = math.radians(pl["q_s_deg"] + delta_deg)
    ratio = float(np.interp(q_s, qs, np.gradient(th, qs)))
    k_theta, d_theta = opts.get("k_q", T.K_Q) / ratio ** 2, opts.get("d_q", T.D_Q) / ratio ** 2
    n_app, n_set, n_rel, n_sw, n_hold = (int(round(opts[k] / T.DT_ENV)) for k in
                                         ("approach_s", "settle_s", "release_s", "sweep_s", "hold_s"))
    face = geom["pads"]["mount_offset_mm"] + geom["pads"]["thickness_mm"]
    u = np.array(pl["u"])
    entry_w = np.array(pl["entry_mm"]) - np.array([0.0, face])
    pen, prox, padwall = 0.0, 0.0, 0.0
    thumb_k = POINT_K
    frames = []

    def depth(s):
        c, ang = s.peg()
        ax = np.array([math.cos(ang), -math.sin(ang)])  # the peg's body x in (x, z)
        lead = 1e3 * c + (pl["L_mm"] / 2 + pl["r_mm"]) * ax  # the tip of its leading cap
        return float((lead - entry_w) @ u)

    def log(s, phase, q):
        nonlocal pen, prox, padwall
        f = s.forces()
        pen = max(pen, f["distal"] / POINT_K, f["thumb"] / thumb_k, f["proximal"] / POINT_K)
        prox, padwall = max(prox, f["proximal"]), max(padwall, f["pad_wall"])
        if record:
            c, ang = s.peg()
            dp = s.plant.EvalBodyPoseInWorld(s.pctx, s.bodies["distal"])
            tpos = s.plant.EvalBodyPoseInWorld(s.pctx, s.bodies["thumb"]).translation()
            R = dp.rotation().matrix()
            frames.append({"t": s.t, "phase": phase, "q_cmd_deg": math.degrees(q),
                           "q_deg": math.degrees(math.atan2(R[0, 2], R[0, 0])),
                           "peg_mm_rad": [1e3 * c[0], 1e3 * c[1], ang], "thumb_mm": [1e3 * tpos[0], 1e3 * tpos[2]],
                           "depth_mm": depth(s), "f_distal": f["distal"], "f_thumb": f["thumb"], "f_wall": f["wall"]})
        return f

    s1 = Run(hand, mu, pl, opts, k_theta, d_theta, "hold")
    s1.sim.Initialize()
    for i in range(n_app + n_set):
        q = q_s * min(1.0, (i + 1) / n_app)
        s1.theta_cmd = float(np.interp(q, qs, th))
        s1.step()
        log(s1, "approach", q)
    state = s1.plant.GetPositionsAndVelocities(s1.pctx).copy()
    s = Run(hand, mu, pl, opts, k_theta, d_theta, "free")
    s.plant.SetPositionsAndVelocities(s.pctx, state)
    s.theta_cmd = float(np.interp(q_s, qs, th))
    s.sim.Initialize()
    for _ in range(n_rel):
        s.step()
        log(s, "release", q_s)
    d0 = depth(s)
    q1 = q_s - math.radians(pl["sweep_deg"])
    trace = []
    for i in range(n_sw + n_hold):
        q = q_s + (q1 - q_s) * min(1.0, (i + 1) / n_sw)
        s.theta_cmd = float(np.interp(q, qs, th))
        s.step()
        f = log(s, "sweep", q)
        trace.append([round(depth(s), 4), round(f["distal"], 4), round(f["thumb"], 4), round(f["wall"], 4)])
    f = s.forces()
    out = {"depth_mm": depth(s), "depth_start_mm": d0, "depth_max_mm": max(t[0] for t in trace),
           "lost": bool(f["distal"] < T.LOSS_N or f["thumb"] < T.LOSS_N), "pen_mm": pen,
           "proximal_max_n": prox, "pad_wall_max_n": padwall, "wall_end_n": f["wall"],
           "tilt_end_deg": math.degrees(s.peg()[1]), "trace": trace}
    if record:
        out["frames"] = frames
    return out


def is_lost(x):
    if x["lost"]:
        return True
    run = 0
    for _, fd, ft, _ in x["trace"]:
        run = run + 1 if min(fd, ft) < T.LOSS_N else 0
        if run >= LOST_RUN:
            return True
    return False


def run_task(task):
    hand, pl, opts, delta = task
    t0 = time.time()
    rec = {"draw": pl["draw"], "hand": hand}
    for mu in MU:
        try:
            rec[f"{mu:g}"] = peg_in(hand, mu, pl, opts, delta)
        except RuntimeError as err:
            rec[f"{mu:g}"] = {"solver_fail": str(err)[:200]}
    rec["wall_s"] = time.time() - t0
    return rec


def _opts(d, a=None):
    o = {**DEFAULTS, **d.get("config", {})}
    if a is not None and getattr(a, "config", None):
        o.update(json.loads(a.config))
    return o


def cmd_run(a) -> int:
    d = json.loads(a.draws.read_text())
    draws = d["draws"][: a.only] if a.only else d["draws"]
    opts = _opts(d, a)
    tasks = [(h, pl, opts, a.delta_deg) for h in a.hands for pl in draws]
    t0 = time.time()
    with Pool(a.workers) as pool:
        recs = pool.map(run_task, tasks, chunksize=1)
    for h in a.hands:
        rr = [r for r in recs if r["hand"] == h]
        (LOGS / f"peg_{h}_{a.tag}.json").write_text(json.dumps({"hand": h, "tag": a.tag, "opts": opts,
                                                                 "delta_deg": a.delta_deg, "draws": rr}))
        print(f"{h} {a.tag}: {len(rr)} draws, cpu {sum(r['wall_s'] for r in rr):.0f} s", flush=True)
    print(f"done in {time.time() - t0:.0f} s (workers {a.workers})", flush=True)
    return 0


# ----------------------------------------------------------------------------------------------------- analysis
MIN_D_MM = 0.3
METRIC = "depth_mm"


def _load(hand, tag):
    p = LOGS / f"peg_{hand}_{tag}.json"
    return {r["draw"]: r for r in json.loads(p.read_text())["draws"]} if p.exists() else None


def o_keep(tag):
    o = _load("O", tag)
    keep, excl = {}, {"o_solver": 0, "o_lost": 0, "o_pen": 0, "o_proximal": 0, "o_pad_wall": 0, "small_d": 0}
    for k, r in sorted(o.items()):
        runs = [r[f"{mu:g}"] for mu in MU]
        if any("solver_fail" in x for x in runs):
            excl["o_solver"] += 1
        elif any(is_lost(x) for x in runs):
            excl["o_lost"] += 1
        elif any(x["pen_mm"] > PEN_MM for x in runs):
            excl["o_pen"] += 1
        elif any(x["proximal_max_n"] > PROX_TOUCH_N for x in runs):
            excl["o_proximal"] += 1
        elif any(x["pad_wall_max_n"] > PROX_TOUCH_N for x in runs):
            excl["o_pad_wall"] += 1
        elif abs(runs[1][METRIC] - runs[0][METRIC]) < MIN_D_MM:
            excl["small_d"] += 1
        else:
            keep[k] = runs
    return keep, excl


def hand_error(hand, tag, keep):
    h = _load(hand, tag)
    if h is None:
        return None
    rel, sens, pens = [], [], []
    cnt = {"solver": 0, "lost": 0, "over_pressed": 0, "proximal": 0, "pad_wall": 0}
    for k, o in keep.items():
        runs = [h[k][f"{mu:g}"] for mu in MU]
        if any("solver_fail" in x for x in runs):
            cnt["solver"] += 1
            sens.append(math.inf)
            continue
        if any(is_lost(x) for x in runs):
            cnt["lost"] += 1
            sens.append(math.inf)
            continue
        pen = max(x["pen_mm"] for x in runs)
        pens.append(pen)
        if pen > PEN_MM:
            cnt["over_pressed"] += 1
            sens.append(math.inf)
            continue
        cnt["proximal"] += any(x["proximal_max_n"] > PROX_TOUCH_N for x in runs)
        cnt["pad_wall"] += any(x["pad_wall_max_n"] > PROX_TOUCH_N for x in runs)
        d_o = o[1][METRIC] - o[0][METRIC]
        e = abs((runs[1][METRIC] - runs[0][METRIC]) - d_o) / abs(d_o)
        rel.append(e)
        sens.append(e)
    return {"E": float(np.median(rel)) if rel else None, "E_p75": float(np.percentile(rel, 75)) if rel else None,
            "pairs": len(rel), "E_sens": float(np.median(sens)) if sens else None, **cnt,
            "pen_max_mm": float(max(pens)) if pens else None}


def deviation(tag, keep):
    o = _load("O", tag)
    out, n_min = {}, None
    for hand in ("R", "U"):
        h = _load(hand, tag)
        for mu in MU:
            ys = []
            for k in keep:
                xh = h[k][f"{mu:g}"]
                if "solver_fail" in xh or is_lost(xh) or max(h[k][f"{m:g}"].get("pen_mm", 0) for m in MU) > PEN_MM:
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
    keep, excl = o_keep(a.tag)
    d_o = np.array([v[1][METRIC] - v[0][METRIC] for v in keep.values()])
    dd = np.array([v[0][METRIC] for v in keep.values()])
    print(f"{a.tag}: O kept {len(keep)}, excluded {excl}; D(O) {d_o.min():.2f}/{np.median(d_o):.2f}/{d_o.max():.2f} mm "
          f"(negative {int((d_o < 0).sum())}); depth(O, mu 1.3) {dd.min():.1f}/{np.median(dd):.1f}/{dd.max():.1f}")
    out = {"tag": a.tag, "metric": METRIC, "o_kept": len(keep), "o_excluded": excl, "d_o_mm": d_o.tolist(), "rows": []}
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
              f"padwall {r['pad_wall']} | pen max {r['pen_max_mm'] or 0:.2f}", flush=True)
    rows = [r for r in out["rows"] if r["E"] is not None]
    if len(rows) >= 3:
        st = {}
        for key in ("E", "E_sens"):
            for cand in ("d_contact", "d_skel"):
                rho, p = spearman_perm([r[cand] for r in rows], [r[key] for r in rows])
                st[f"rho_{key}_{cand}"], st[f"p_{key}_{cand}"] = rho, p
            print(f"rho({key}, D_contact) {st[f'rho_{key}_d_contact']:.2f} (permutation p "
                  f"{st[f'p_{key}_d_contact']:.5f}); rho({key}, D_skel) {st[f'rho_{key}_d_skel']:.2f} "
                  f"(p {st[f'p_{key}_d_skel']:.4f}); hands {len(rows)}")
        out["stats"] = st
    if _load("R", a.tag) and _load("U", a.tag):
        out["deviation"] = deviation(a.tag, keep)
        for k in ("R_1.3", "U_1.3", "R_2", "U_2"):
            m = np.array(out["deviation"][k]["median"])
            i = int(np.argmax(np.abs(m)))
            print(f"deviation {k}: peak {m[i]:+.3f} mm at {out['deviation']['x'][i]:.2f} s, end {m[-1]:+.3f} mm, "
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
        ls = sum(any(is_lost(r[f"{mu:g}"]) for mu in MU) for r in ok)
        pen = [max(r[f"{mu:g}"]["pen_mm"] for mu in MU) for r in ok]
        prox = sum(any(r[f"{mu:g}"]["proximal_max_n"] > PROX_TOUCH_N for mu in MU) for r in ok)
        pw = sum(any(r[f"{mu:g}"]["pad_wall_max_n"] > PROX_TOUCH_N for mu in MU) for r in ok)
        line = (f"{hand} {a.tag}: {len(rows)} draws, solver {fail}, lost {ls}, proximal {prox}, pad-wall {pw}, pen max "
                f"{max(pen) if pen else 0:.2f} mm (> 1.5: {sum(p > PEN_MM for p in pen)}), median "
                f"{np.median(pen) if pen else 0:.2f}")
        if a.show_d:
            good = [r for r in ok if not any(is_lost(r[f"{mu:g}"]) for mu in MU)]
            if good:
                dd = np.array([r["2"][METRIC] - r["1.3"][METRIC] for r in good])
                de = np.array([r["1.3"][METRIC] for r in good])
                line += (f"; depth(1.3) {de.min():.2f}/{np.median(de):.2f}/{de.max():.2f}; D {dd.min():.2f}/"
                         f"{np.median(dd):.2f}/{dd.max():.2f} mm (|D|<0.3: {int((abs(dd) < 0.3).sum())})")
        print(line)
        if a.verbose:
            for r in rows:
                x, y_ = r["1.3"], r["2"]
                if "solver_fail" in x or "solver_fail" in y_:
                    print(f"  draw {r['draw']}: solver fail")
                    continue
                print(f"  draw {r['draw']:2d}: depth {x['depth_start_mm']:+.2f}->{x['depth_mm']:+.2f} / "
                      f"{y_['depth_start_mm']:+.2f}->{y_['depth_mm']:+.2f} pen {x['pen_mm']:.2f}/{y_['pen_mm']:.2f} "
                      f"wall {x['wall_end_n']:.2f}/{y_['wall_end_n']:.2f} tilt {x['tilt_end_deg']:+.1f} lost "
                      f"{is_lost(x)},{is_lost(y_)} padwall {x['pad_wall_max_n']:.2f}")
    return 0


def cmd_frames(a) -> int:
    d = json.loads(a.draws.read_text())
    pl = next(x for x in d["draws"] if x["draw"] == a.draw)
    out = peg_in(a.hand, a.mu, pl, _opts(d), record=True)
    p = LOGS / f"peg_frames_{a.hand}_d{a.draw}.json"
    p.write_text(json.dumps({"hand": a.hand, "mu": a.mu, "placement": pl, "opts": _opts(d), **out}))
    print(f"depth {out['depth_mm']:.2f} mm -> {p.name}")
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    pd_ = sub.add_parser("draws")
    pd_.add_argument("--n", type=int, default=64)
    pd_.add_argument("--seed", type=int, default=36)
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
    pa = sub.add_parser("analyze")
    pa.add_argument("--tag", required=True)
    pa.add_argument("--noise_tag", default=None)
    pa.add_argument("--out", type=Path, default=None)
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
        import pinch3_pose as PP
        ranges = {**RANDOM, **(json.loads(a.ranges) if a.ranges else {})}
        cfg = {**DEFAULTS, **(json.loads(a.config) if a.config else {})}
        draws, pose = make_draws(a.n, a.seed, ranges, cfg["fe_deg"])
        cfg["k_thumb"] = pose["k_thumb_n_m"]
        a.out.write_text(json.dumps({"seed": a.seed, "n": a.n, "rng": f"numpy.random.default_rng([{a.seed}, 36])",
                                     "ranges": ranges, "config": cfg, "pose": pose, "draws": draws}, indent=1))
        sh = np.array([x["thumb_shift_mm"] for x in draws])
        op = np.array([x["opposition_deg"] for x in draws])
        print(f"{len(draws)} draws -> {a.out}; thumb k {pose['k_thumb_n_m']:.0f} N/m; thumb shift from FK along its "
              f"normal {sh.min():+.1f}..{sh.max():+.1f} mm; opposition {op.min():.1f}..{op.max():.1f} deg")
        return 0
    return {"run": cmd_run, "analyze": cmd_analyze, "check": cmd_check, "frames": cmd_frames}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main())
