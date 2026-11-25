"""Experiment 33: thumb-index pinch, the index turns a sphere slightly while both keep contact (experimental;
docs/experimental/drake_pinch.md).

The planar CoDe index finger of two_pad_drake.py (imported read-only: geometry, servo table, condition A's flexion
impedance; the MCP held at 0, so the proximal phalanx is the world). Frame: x along the proximal phalanx from the MCP, z
dorsal; placements are given in the hand's proximal-face frame (world z = z_face - face), the same coordinates on every
hand, and are made on O.

- Sphere (radius r): in the finger's plane on a planar slide (x, z, spin about y), as the rolling task's cylinder. Its
  spin turns against a torsional spring k_r (and a small damper), so that the pads must grip it to turn it and the end
  rotation depends on the pads' mu (without it the sphere rolls freely and D ~ 0, as the untethered coin).
- Index: only its distal pad is meant to touch the sphere. At O's start flexion q_s the contact point is back_mm
  proximal of O's distal-pad tip end; the sphere's centre is r along O's distal-pad outward normal n from it.
- Thumb: an opposing pad identical on every hand (a box, the pads' point stiffness 2.5 kN/m and mu), on the far side of
  the sphere, its face normal -n, on a prismatic joint along n with a soft spring K_THUMB (N/m) whose rest position is
  preload / K_THUMB into the sphere: the spring, not the finger, sets the squeeze, and a hand whose face stands off
  differently only moves the thumb (penetration stays near preload / 2.5 kN/m). The thumb cannot move tangentially.
- Command (O's open loop on every hand, condition A): the sphere and thumb held while the finger closes 0 -> q_s over
  approach_s and holds settle_s; then a new plant with the sphere free (x, z, the spin's spring) settles release_s;
  then q_s -> q_s + sweep over sweep_s, held hold_s. Gains at q_s.
- Measure: the sphere's rotation from the sweep's start to the end (deg, + with flexion); D = rot(2.0) - rot(1.3);
  lost: at the end either pad's normal force (index distal or thumb, on the sphere) < LOSS_N; penetration: the largest
  pad force / 2.5 N/mm over the run (index distal, proximal, thumb).

    uv run python pinch_rotate.py draws --n 64 --seed 33 --out logs/pinch_e33.json
    uv run python pinch_rotate.py run --hands O --draws logs/pinch_e33.json --tag e33 [--delta_deg 0.05] [--only 8]
    uv run python pinch_rotate.py analyze --draws logs/pinch_e33.json --tag e33 --noise_tag e33n
    uv run python pinch_rotate.py frames --draws logs/pinch_e33.json --draw 0
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
PEN_MM, POINT_K = 1.5, T.POINT_STIFFNESS * 1e-3  # gate (mm), pad point stiffness (N/mm)
SPHERE_MASS_KG = T.MASS_KG
THUMB_MASS_KG = 0.01
THUMB_LEN_MM, THUMB_THICK_MM = 30.0, 3.0  # the thumb pad's extent along the sphere's surface and its thickness
K_THUMB, D_THUMB = 40.0, 0.5  # the thumb's squeeze spring (N/m) and damper (N s/m)
D_SPIN = 1.0e-4  # N m s/rad on the sphere's spin, with its spring
PROX_TOUCH_N = 1e-3
HOLD_N_FACTOR = 1.25  # the approach's hold along the squeeze axis: its force limit / preload
# experiment 33's distributions (fixed before any redesigned-hand data): independent uniforms drawn as vectors of N in
# this order from numpy.random.default_rng([seed, 33])
RANDOM = {"q_s_deg": (30.0, 55.0), "back_mm": (3.0, 8.0), "r_mm": (8.0, 12.0), "preload_n": (0.4, 0.8),
          "sweep_deg": (18.0, 28.0), "k_r": (0.25, 0.40)}
DEFAULTS = {"approach_s": 1.0, "settle_s": 0.5, "release_s": 0.5, "sweep_s": 2.0, "hold_s": 0.5, "k_thumb": K_THUMB}


def make_draws(n: int, seed: int, ranges: dict = RANDOM) -> list[dict]:
    rng = np.random.default_rng([seed, 33])
    cols = {k: rng.uniform(lo, hi, n) for k, (lo, hi) in ranges.items()}
    geom = T.finger_geometry("O")
    qs, th = T.servo_table(geom)
    end = geom["pads"]["distal_end_mm"]
    out = []
    for i in range(n):
        d = {k: float(v[i]) for k, v in cols.items()}
        q = math.radians(d["q_s_deg"])
        p = T.reference_point(geom, qs, th, q, end - d["back_mm"])  # O's contact point (proximal-face frame, mm)
        nrm = T.rot(q) @ np.array([0.0, -1.0])  # O's distal pad's outward normal
        c = p + d["r_mm"] * nrm
        out.append({"draw": i, **d, "c_mm": c.tolist(), "n": nrm.tolist(), "thumb_mm": (c + d["r_mm"] * nrm).tolist()})
    return out


def build(hand, mu, pl, time_step, k_theta, d_theta, stage, opts):
    """stage 'hold': the sphere held (x, z, spin) while the finger closes; 'free': the sphere free in x, z, its spin on
    the torsional spring. The thumb's spring acts in both."""
    from pydrake.geometry import AddContactMaterial, Box, ProximityProperties, Sphere
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
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
    plant.set_contact_model(ContactModel.kPoint)
    world = plant.world_body()
    y = np.array([0.0, 1.0, 0.0])

    def at(p2):
        return np.array([p2[0], 0.0, p2[1]])

    def rod(mass, length):
        return SpatialInertia(mass, np.zeros(3), UnitInertia.SolidBox(max(length, 1e-3), 0.004, 0.004))

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

    nrm = np.array(pl["n"])
    n3 = at(nrm)
    c = np.array(pl["c_mm"]) * mm - np.array([0.0, face])  # world (x, z)
    tp = np.array(pl["thumb_mm"]) * mm - np.array([0.0, face])
    r = pl["r_mm"] * mm

    # the thumb: its body frame on its face (local z = -n, toward the sphere... the face's outward normal), sliding
    # along n; joint coordinate 0 at the nominal contact, + away from the sphere
    ang = math.atan2(nrm[0], nrm[1])  # rotation about y taking local z (0,0,1) to n3
    R_t = RotationMatrix.MakeYRotation(ang)
    thumb = plant.AddRigidBody("thumb", SpatialInertia(THUMB_MASS_KG, np.zeros(3),
                                                       UnitInertia.SolidBox(0.03, 0.02, 0.003)))
    f_tb = plant.AddFrame(FixedOffsetFrame("thumb_base", world.body_frame(), RigidTransform(R_t, at(tp))))
    th_j = plant.AddJoint(PrismaticJoint("thumb_n", f_tb, thumb.body_frame(), [0.0, 0.0, 1.0]))
    plant.AddJointActuator("thumb_n", th_j).set_controller_gains(
        PdControllerGains(p=opts.get("k_thumb", K_THUMB), d=D_THUMB))

    # the sphere: on slides along O's pad tangent t and normal n (the squeeze axis), then spin about y
    cx = plant.AddRigidBody("sph_ct", rod(1e-4, 0.01))
    cz = plant.AddRigidBody("sph_cn", rod(1e-4, 0.01))
    sph = plant.AddRigidBody("sphere", SpatialInertia(SPHERE_MASS_KG, np.zeros(3), UnitInertia.SolidSphere(r)))
    f_sb = plant.AddFrame(FixedOffsetFrame("sph_base", world.body_frame(), RigidTransform(R_t, at(c))))
    jt = plant.AddJoint(PrismaticJoint("sph_t", f_sb, cx.body_frame(), [1.0, 0.0, 0.0]))
    jn = plant.AddJoint(PrismaticJoint("sph_n", cx.body_frame(), cz.body_frame(), [0.0, 0.0, 1.0]))
    js = plant.AddJoint(RevoluteJoint("spin", cz.body_frame(), sph.body_frame(), y))
    if stage == "hold":  # held along t and in spin; along n by a drive limited to HOLD_N_FACTOR x preload, which
        # holds the thumb's preload until the index arrives and then gives way, so the index cannot press the sphere
        # harder than (1 + HOLD_N_FACTOR) x preload against a hold (the thumb's spring sets the squeeze)
        plant.AddJointActuator("sph_t", jt).set_controller_gains(PdControllerGains(p=1.0e5, d=1.0e3))
        plant.AddJointActuator("sph_n", jn, effort_limit=HOLD_N_FACTOR * pl["preload_n"]).set_controller_gains(
            PdControllerGains(p=1.0e5, d=1.0e3))
        plant.AddJointActuator("spin", js).set_controller_gains(PdControllerGains(p=10.0, d=0.1))
    else:
        plant.AddJointActuator("spin", js).set_controller_gains(PdControllerGains(p=pl["k_r"], d=D_SPIN))

    def props(k):
        pr = ProximityProperties()
        AddContactMaterial(point_stiffness=k, friction=CoulombFriction(mu, mu), properties=pr)
        return pr

    s0, s1 = pads["proximal_start_mm"] * mm, pads["proximal_end_mm"] * mm
    plant.RegisterCollisionGeometry(proximal, RigidTransform(at(((s0 + s1) / 2, -(m + h / 2)))), Box(s1 - s0, w, h),
                                    "proximal_pad", props(T.POINT_STIFFNESS))
    t0, t1 = pads["distal_start_mm"] * mm, pads["distal_end_mm"] * mm
    plant.RegisterCollisionGeometry(distal, RigidTransform(at(((t0 + t1) / 2, -(m + h / 2)))), Box(t1 - t0, w, h),
                                    "distal_pad", props(T.POINT_STIFFNESS))
    tt = THUMB_THICK_MM * mm
    # the thumb pad's face on the thumb frame's origin, its body behind it (away from the sphere, + local z = +n)
    plant.RegisterCollisionGeometry(thumb, RigidTransform([0.0, 0.0, tt / 2]), Box(THUMB_LEN_MM * mm, w, tt),
                                    "thumb_pad", props(T.POINT_STIFFNESS))
    plant.RegisterCollisionGeometry(sph, RigidTransform(), Sphere(r), "sphere", props(1.0e8))
    plant.Finalize()
    plant.mutable_gravity_field().set_gravity_vector(np.zeros(3))
    return builder, plant, scene_graph, {"distal": distal, "proximal": proximal, "sphere": sph, "thumb": thumb}


class Run:
    def __init__(self, hand, mu, pl, time_step, k_theta, d_theta, stage, opts):
        from pydrake.systems.analysis import Simulator
        from pydrake.systems.framework import LeafSystem

        builder, plant, sg, bodies = build(hand, mu, pl, time_step, k_theta, d_theta, stage, opts)
        self.plant, self.bodies = plant, bodies
        self.theta_cmd = 0.0
        n_act = plant.num_actuators()
        # thumb rest: preload / k into the sphere (- along its joint, which points away from the sphere)
        rest = -pl["preload_n"] / opts.get("k_thumb", K_THUMB)
        outer = self

        class Command(LeafSystem):
            def __init__(self):
                super().__init__()
                self.DeclareVectorOutputPort("desired", 2 * n_act, self.calc)

            def calc(self, context, out):  # servo, thumb, then the sphere's (x, z, spin) or (spin)
                q = [outer.theta_cmd, rest] + [0.0] * (n_act - 2)
                out.SetFromVector(np.array(q + [0.0] * n_act))

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
        """Normal force magnitudes on the sphere from the index distal pad, the proximal pad and the thumb, and the
        distal contact point."""
        res = self.plant.get_contact_results_output_port().Eval(self.pctx)
        name = {v: k for k, v in self.idx.items()}
        out = {"distal": 0.0, "proximal": 0.0, "thumb": 0.0, "distal_pt": None}
        for k in range(res.num_point_pair_contacts()):
            c = res.point_pair_contact_info(k)
            pair = {name.get(c.bodyA_index()), name.get(c.bodyB_index())}
            if "sphere" not in pair:
                continue
            other = (pair - {"sphere"}).pop()
            if other in out:  # the normal component (penetration = normal force / point stiffness)
                out[other] += abs(float(c.contact_force() @ c.point_pair().nhat_BA_W))
                if other == "distal":
                    out["distal_pt"] = c.contact_point()
        return out

    def spin(self):
        return self.plant.GetJointByName("spin").get_angle(self.pctx)

    def pose(self, body):
        X = self.plant.EvalBodyPoseInWorld(self.pctx, self.bodies[body])
        R = X.rotation().matrix()
        return np.array([X.translation()[0], X.translation()[2], math.atan2(R[0, 2], R[0, 0])])


def pinch(hand, mu, pl, opts, delta_deg=0.0, record=False) -> dict:
    geom = T.finger_geometry(hand)
    qs, th = T.servo_table(geom)
    q_s = math.radians(pl["q_s_deg"] + delta_deg)
    ratio = float(np.interp(q_s, qs, np.gradient(th, qs)))
    k_theta, d_theta = T.K_Q / ratio ** 2, T.D_Q / ratio ** 2
    ts = opts.get("time_step", 1.0e-3)
    n_app, n_set, n_rel, n_sw, n_hold = (int(round(opts[k] / T.DT_ENV)) for k in
                                         ("approach_s", "settle_s", "release_s", "sweep_s", "hold_s"))
    pen, prox_max, frames = 0.0, 0.0, []
    peak = {"distal": 0.0, "thumb": 0.0}
    touch_q = None

    def log(s, phase, q):
        nonlocal pen, prox_max, touch_q
        f = s.forces()
        pen = max(pen, f["distal"] / POINT_K, f["thumb"] / POINT_K, f["proximal"] / POINT_K)
        prox_max = max(prox_max, f["proximal"])
        for k in peak:
            peak[k] = max(peak[k], f[k])
        if touch_q is None and f["distal"] > T.GATE_N:
            touch_q = math.degrees(q)
        if record:
            sp, dp, tp = s.pose("sphere"), s.pose("distal"), s.pose("thumb")
            pt = f["distal_pt"]
            frames.append({"t": s.t, "phase": phase, "q_cmd_deg": math.degrees(q), "q_deg": math.degrees(dp[2]),
                           "sphere_mm_rad": [1e3 * sp[0], 1e3 * sp[1], -s.spin()],
                           "distal_pose_mm_rad": [1e3 * dp[0], 1e3 * dp[1], dp[2]],
                           "thumb_mm": [1e3 * tp[0], 1e3 * tp[1]], "f_distal": f["distal"], "f_thumb": f["thumb"],
                           "f_proximal": f["proximal"],
                           "contact_mm": None if pt is None else [1e3 * pt[0], 1e3 * pt[2]]})
        return f

    s1 = Run(hand, mu, pl, ts, k_theta, d_theta, "hold", opts)
    s1.sim.Initialize()
    theta_s = float(np.interp(q_s, qs, th))
    for i in range(n_app + n_set):
        q = q_s * min(1.0, (i + 1) / n_app)
        s1.theta_cmd = float(np.interp(q, qs, th))
        s1.step()
        log(s1, "approach", q)
    state = s1.plant.GetPositionsAndVelocities(s1.pctx).copy()
    s = Run(hand, mu, pl, ts, k_theta, d_theta, "free", opts)
    s.plant.SetPositionsAndVelocities(s.pctx, state)
    s.theta_cmd = theta_s
    s.sim.Initialize()
    for _ in range(n_rel):
        s.step()
        f_rel = log(s, "release", q_s)
    spin0 = s.spin()
    c0 = s.pose("sphere")
    q1 = q_s + math.radians(pl["sweep_deg"])
    f_min = {"distal": 1e9, "thumb": 1e9}
    trace = []
    for i in range(n_sw + n_hold):
        q = q_s + (q1 - q_s) * min(1.0, (i + 1) / n_sw)
        s.theta_cmd = float(np.interp(q, qs, th))
        s.step()
        f = log(s, "sweep", q)
        for k in f_min:
            f_min[k] = min(f_min[k], f[k])
        trace.append([round(-math.degrees(s.spin() - spin0), 4), round(f["distal"], 4), round(f["thumb"], 4)])
    f = s.forces()
    sp = s.pose("sphere")
    out = {"rotation_deg": -math.degrees(s.spin() - spin0), "rotation_release_deg": -math.degrees(spin0),
           "lost": bool(f["distal"] < T.LOSS_N or f["thumb"] < T.LOSS_N),
           "lost_during": bool(min(f_min.values()) < T.LOSS_N), "f_min_n": f_min,
           "f_release_n": {"distal": f_rel["distal"], "thumb": f_rel["thumb"], "proximal": f_rel["proximal"]},
           "f_end_n": {"distal": f["distal"], "thumb": f["thumb"]}, "f_peak_n": peak, "pen_mm": pen,
           "proximal_max_n": prox_max, "touch_q_deg": touch_q,
           "sphere_shift_mm": (1e3 * (sp[:2] - c0[:2])).tolist(), "q_end_deg": math.degrees(s.pose("distal")[2]),
           "trace": trace}
    if record:
        out["frames"] = frames
    return out


def run_task(task):
    hand, pl, opts, delta = task
    t0 = time.time()
    rec = {"draw": pl["draw"], "hand": hand}
    for mu in MU:
        try:
            rec[f"{mu:g}"] = pinch(hand, mu, pl, opts, delta)
        except RuntimeError as err:
            rec[f"{mu:g}"] = {"solver_fail": str(err)[:200]}
    rec["wall_s"] = time.time() - t0
    return rec


def cmd_run(a) -> int:
    d = json.loads(a.draws.read_text())
    draws = d["draws"][: a.only] if a.only else d["draws"]
    if a.draw_ids:
        draws = [x for x in d["draws"] if x["draw"] in a.draw_ids]
    opts = {**DEFAULTS, **d.get("config", {}), "time_step": a.time_step}
    for k in ("k_thumb",):
        if getattr(a, k) is not None:
            opts[k] = getattr(a, k)
    tasks = [(hand, pl, opts, a.delta_deg) for hand in a.hands for pl in draws]
    t0 = time.time()
    with Pool(a.workers) as pool:
        recs = pool.map(run_task, tasks, chunksize=1)
    for hand in a.hands:
        rr = [r for r in recs if r["hand"] == hand]
        out = LOGS / f"pinch_{hand}_{a.tag}.json"
        out.write_text(json.dumps({"hand": hand, "tag": a.tag, "opts": opts, "delta_deg": a.delta_deg, "draws": rr}))
        cpu = sum(r["wall_s"] for r in rr)
        print(f"{hand} {a.tag}: {len(rr)} draws, cpu {cpu:.0f} s -> {out.name}", flush=True)
    print(f"done in {time.time() - t0:.0f} s (workers {a.workers})", flush=True)
    return 0


# ----------------------------------------------------------------------------------------------------- analysis
LOST_RUN = 3  # consecutive samples (0.1 s) below LOSS_N during the sweep and hold count as lost


def is_lost(x) -> bool:
    """Lost: at the end either pad's force on the sphere < LOSS_N, or LOST_RUN consecutive samples so during the sweep
    and hold. A single low sample is stick-slip chatter (counted by chatter())."""
    if x["lost"]:
        return True
    run = 0
    for _, fd, ft in x["trace"]:
        run = run + 1 if min(fd, ft) < T.LOSS_N else 0
        if run >= LOST_RUN:
            return True
    return False


def chatter(x) -> int:
    return sum(min(fd, ft) < T.LOSS_N for _, fd, ft in x["trace"])


def _load(hand, tag):
    p = LOGS / f"pinch_{hand}_{tag}.json"
    return {r["draw"]: r for r in json.loads(p.read_text())["draws"]} if p.exists() else None


def o_keep(tag):
    o = _load("O", tag)
    keep, excl = {}, {"o_solver": 0, "o_lost": 0, "o_pen": 0, "o_proximal": 0, "small_d": 0}
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
        elif abs(runs[1]["rotation_deg"] - runs[0]["rotation_deg"]) < 0.3:
            excl["small_d"] += 1
        else:
            keep[k] = runs
    return keep, excl


def hand_error(hand, tag, keep):
    h = _load(hand, tag)
    if h is None:
        return None
    rel, sens, pens = [], [], []
    cnt = {"solver": 0, "lost": 0, "over_pressed": 0, "proximal": 0, "chatter_pairs": 0}
    for k, o in keep.items():
        runs = [h[k][f"{mu:g}"] for mu in MU]
        if any("solver_fail" in x for x in runs):
            cnt["solver"] += 1
            sens.append(math.inf)
            continue
        pen = max(x["pen_mm"] for x in runs)
        pens.append(pen)
        cnt["chatter_pairs"] += any(chatter(x) for x in runs)
        if any(is_lost(x) for x in runs):
            cnt["lost"] += 1
            sens.append(math.inf)
            continue
        if pen > PEN_MM:
            cnt["over_pressed"] += 1
            sens.append(math.inf)
            continue
        cnt["proximal"] += any(x["proximal_max_n"] > PROX_TOUCH_N for x in runs)
        d_o = o[1]["rotation_deg"] - o[0]["rotation_deg"]
        d_h = runs[1]["rotation_deg"] - runs[0]["rotation_deg"]
        e = abs(d_h - d_o) / abs(d_o)
        rel.append(e)
        sens.append(e)
    return {"E": float(np.median(rel)) if rel else None, "E_p75": float(np.percentile(rel, 75)) if rel else None,
            "pairs": len(rel), "E_sens": float(np.median(sens)) if sens else None, **cnt,
            "pen_max_mm": float(max(pens)) if pens else None,
            "pen_median_mm": float(np.median(pens)) if pens else None}


def cmd_analyze(a) -> int:
    from analyze import DESIGN, spearman_perm
    keep, excl = o_keep(a.tag)
    d_o = np.array([o[1]["rotation_deg"] - o[0]["rotation_deg"] for o in keep.values()])
    rot_o = np.array([o[0]["rotation_deg"] for o in keep.values()])
    print(f"{a.tag}: O kept {len(keep)}, excluded {excl}; D(O) {d_o.min():.2f}/{np.median(d_o):.2f}/{d_o.max():.2f} deg "
          f"(negative {int((d_o < 0).sum())}); rot(O, mu 1.3) {rot_o.min():.2f}/{np.median(rot_o):.2f}/"
          f"{rot_o.max():.2f} deg")
    out = {"tag": a.tag, "o_kept": len(keep), "o_excluded": excl, "d_o_deg": d_o.tolist(), "rows": []}
    if a.noise_tag:
        nz = hand_error(f"O", a.noise_tag, keep)
        out["noise_floor"] = nz
        print(f"noise floor ({a.noise_tag}): E {100 * nz['E']:.2f}% (p75 {100 * nz['E_p75']:.2f}%), pairs {nz['pairs']}")
    rows = []
    for hand, (dc, ds) in DESIGN.items():
        if a.hands and hand not in a.hands:
            continue
        r = hand_error(hand, a.tag, keep)
        if r is None:
            continue
        rows.append({"hand": hand, "d_contact": dc, "d_skel": ds, **r})
        print(f"{hand:22s} Dc {dc:4.2f} Ds {ds:5.2f} | E {('-' if r['E'] is None else f'{100 * r[chr(69)]:5.1f}%')} "
              f"sens {('-' if r['E_sens'] in (None, math.inf) else f'{100 * r['E_sens']:5.1f}%')} | pairs "
              f"{r['pairs']}/{len(keep)} lost {r['lost']} (chatter {r['chatter_pairs']}) over {r['over_pressed']} prox {r['proximal']} | pen max "
              f"{r['pen_max_mm'] or 0:.2f} med {r['pen_median_mm'] or 0:.2f}", flush=True)
    out["rows"] = rows
    sc = [r for r in rows if r["E"] is not None]
    if len(sc) >= 3:
        st = {}
        for key in ("E", "E_sens"):
            rr = [r for r in sc if r[key] is not None]
            for cand in ("d_contact", "d_skel"):
                rho, p = spearman_perm([r[cand] for r in rr], [r[key] for r in rr])
                st[f"rho_{key}_{cand}"], st[f"p_{key}_{cand}"] = rho, p
            print(f"rho({key}, D_contact) {st[f'rho_{key}_d_contact']:.2f} (permutation p "
                  f"{st[f'p_{key}_d_contact']:.4f}); rho({key}, D_skel) {st[f'rho_{key}_d_skel']:.2f} "
                  f"(p {st[f'p_{key}_d_skel']:.4f}); hands {len(rr)}")
        out["stats"] = st
    if a.out:
        a.out.write_text(json.dumps(out, indent=1, default=float))
    return 0


def cmd_summary(a) -> int:
    """The Study 3 summary (report/study3/data/drake_e33_pinch.json): the analysis, and the sweep's rotation
    deviation from O over every kept draw in fig4_sync.json's 'deviation' structure (x = time from the sweep's start,
    s; per hand and mu the median, quartiles and n of rot_H(t) - rot_O(t), deg), plus O's own rotation curve."""
    from analyze import DESIGN, spearman_perm
    keep, excl = o_keep(a.tag)
    o = _load("O", a.tag)
    rows = []
    for hand, (dc, ds) in DESIGN.items():
        r = hand_error(hand, a.tag, keep)
        if r is not None:
            rows.append({"hand": hand, "d_contact": dc, "d_skel": ds, **r})
    rho = lambda key, cand: list(spearman_perm([r[cand] for r in rows], [r[key] for r in rows]))
    dev, n_min = {}, None
    for hand in ("R", "U"):
        h = _load(hand, a.tag)
        for mu in MU:
            ys = []
            for k in keep:
                x_o, x_h = o[k][f"{mu:g}"], h[k][f"{mu:g}"]
                if "solver_fail" in x_h or is_lost(x_h) or max(h[k][f"{m:g}"]["pen_mm"] for m in MU) > PEN_MM:
                    continue
                ro, rh = np.array(x_o["trace"])[:, 0], np.array(x_h["trace"])[:, 0]
                n = min(len(ro), len(rh))
                ys.append(rh[:n] - ro[:n])
            n = min(map(len, ys))
            n_min = n if n_min is None else min(n_min, n)
            y = np.array([v[:n] for v in ys])
            dev[f"{hand}_{mu:g}"] = {"median": np.round(np.median(y, 0), 4).tolist(),
                                     "q25": np.round(np.percentile(y, 25, 0), 4).tolist(),
                                     "q75": np.round(np.percentile(y, 75, 0), 4).tolist(), "n": len(ys)}
    dev["x"] = [round((i + 1) * T.DT_ENV, 4) for i in range(n_min)]
    o_rot = {}
    for mu in MU:
        y = np.array([np.array(keep[k][MU.index(mu)]["trace"])[:n_min, 0] for k in keep])
        o_rot[f"{mu:g}"] = {"median": np.round(np.median(y, 0), 4).tolist(),
                            "q25": np.round(np.percentile(y, 25, 0), 4).tolist(),
                            "q75": np.round(np.percentile(y, 75, 0), 4).tolist(), "n": len(y)}
    nz = hand_error("O", a.noise_tag, keep)
    out = {"tag": a.tag, "metric": "rotation_deg (sphere, from the sweep's start)", "o_survive": len(keep),
           "o_excluded": excl, "o_pen_max_mm": max(max(x["pen_mm"] for x in v) for v in keep.values()),
           "d_o_deg": {str(k): v[1]["rotation_deg"] - v[0]["rotation_deg"] for k, v in keep.items()},
           "rows": rows, "noise_floor": {"tag": a.noise_tag, **nz},
           "E_rho": {"contact": rho("E", "d_contact"), "skel": rho("E", "d_skel")},
           "E_over_as_max_rho": {"contact": rho("E_sens", "d_contact"), "skel": rho("E_sens", "d_skel")},
           "deviation": dev, "o_rotation": {"x": dev["x"], **o_rot},
           "note": "Experiment 33 (docs/experimental/drake_pinch.md): thumb-index pinch, the index distal pad turns a "
                   "sphere (k_r 0.25-0.40 N m/rad) against an identical thumb pad on a 40 N/m spring, 64 draws (seed "
                   "33), condition A, point contact, time step 0.25 ms. deviation: rot_H(t) - rot_O(t) per draw over "
                   "the sweep and hold (x: time from the sweep's start, s), median/q25/q75 over kept draws; "
                   "o_rotation: O's own rotation."}
    a.out.write_text(json.dumps(out, indent=1, default=float))
    for k in ("R_1.3", "U_1.3", "R_2", "U_2"):
        m = np.array(dev[k]["median"])
        print(f"{k}: median deviation peak {m[np.argmax(abs(m))]:+.3f} deg, end {m[-1]:+.3f} deg, n {dev[k]['n']}")
    print(f"O end rotation median: {o_rot['1.3']['median'][-1]:.2f} / {o_rot['2']['median'][-1]:.2f} deg -> {a.out}")
    return 0


def cmd_check(a) -> int:
    """O-only (or any hand) summary without E: lost, penetration, proximal contact, rotation, D."""
    for hand in a.hands:
        h = _load(hand, a.tag)
        rows = [h[k] for k in sorted(h)]
        lost = [any("solver_fail" in r[f"{mu:g}"] or is_lost(r[f"{mu:g}"]) for mu in MU) for r in rows]
        chat = sum(any("solver_fail" not in r[f"{mu:g}"] and chatter(r[f"{mu:g}"]) for mu in MU) for r in rows)
        fail = sum(any("solver_fail" in r[f"{mu:g}"] for mu in MU) for r in rows)
        ok = [r for r, l_ in zip(rows, lost) if not l_ and not any("solver_fail" in r[f"{mu:g}"] for mu in MU)]
        pen = [max(r[f"{mu:g}"]["pen_mm"] for mu in MU) for r in rows if "solver_fail" not in r["1.3"]]
        prox = sum(any(r[f"{mu:g}"].get("proximal_max_n", 0) > PROX_TOUCH_N for mu in MU) for r in rows)
        line = (f"{hand} {a.tag}: {len(rows)} draws, solver {fail}, lost {sum(lost)} (single-sample dropouts in {chat}), proximal {prox}, pen max "
                f"{max(pen):.2f} mm (> 1.5: {sum(p > PEN_MM for p in pen)}), median {np.median(pen):.2f}")
        if a.show_d:
            d = np.array([r["2"]["rotation_deg"] - r["1.3"]["rotation_deg"] for r in ok])
            rot = np.array([r["1.3"]["rotation_deg"] for r in ok])
            line += (f"; rot(1.3) {rot.min():.2f}/{np.median(rot):.2f}/{rot.max():.2f}; D {d.min():.2f}/"
                     f"{np.median(d):.2f}/{d.max():.2f} deg (|D| < 0.3: {int((abs(d) < 0.3).sum())})")
        print(line)
        if a.verbose:
            for r in rows:
                x, y_ = r["1.3"], r["2"]
                if "solver_fail" in x or "solver_fail" in y_:
                    print(f"  draw {r['draw']}: solver fail")
                    continue
                print(f"  draw {r['draw']:2d}: rot {x['rotation_deg']:7.2f} / {y_['rotation_deg']:7.2f} "
                      f"lost {x['lost']},{x['lost_during']}/{y_['lost']},{y_['lost_during']} pen {x['pen_mm']:.2f}/"
                      f"{y_['pen_mm']:.2f} f_rel d/t {x['f_release_n']['distal']:.2f}/{x['f_release_n']['thumb']:.2f}"
                      f" fmin d/t {x['f_min_n']['distal']:.2f}/{x['f_min_n']['thumb']:.2f} prox {x['proximal_max_n']:.3f}"
                      f" shift {x['sphere_shift_mm'][0]:+.1f},{x['sphere_shift_mm'][1]:+.1f}")
    return 0


def cmd_frames(a) -> int:
    d = json.loads(a.draws.read_text())
    pl = next(x for x in d["draws"] if x["draw"] == a.draw)
    opts = {**DEFAULTS, **d.get("config", {}), "time_step": a.time_step}
    out = pinch(a.hand, a.mu, pl, opts, record=True)
    path = LOGS / f"pinch_frames_{a.hand}_d{a.draw}.json"
    path.write_text(json.dumps({"hand": a.hand, "mu": a.mu, "placement": pl, "opts": opts, **out}))
    print(f"rotation {out['rotation_deg']:.2f} deg, lost {out['lost']}, pen {out['pen_mm']:.2f} mm -> {path.name}")
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    pd_ = sub.add_parser("draws")
    pd_.add_argument("--n", type=int, default=64)
    pd_.add_argument("--seed", type=int, default=33)
    pd_.add_argument("--out", type=Path, required=True)
    pr = sub.add_parser("run")
    pr.add_argument("--hands", nargs="+", default=["O"])
    pr.add_argument("--draws", type=Path, required=True)
    pr.add_argument("--tag", required=True)
    pr.add_argument("--delta_deg", type=float, default=0.0)
    pr.add_argument("--only", type=int, default=None)
    pr.add_argument("--draw_ids", type=int, nargs="+", default=None)
    pr.add_argument("--time_step", type=float, default=2.5e-4)
    pr.add_argument("--k_thumb", type=float, default=None)
    pr.add_argument("--workers", type=int, default=6)
    pa = sub.add_parser("analyze")
    pa.add_argument("--tag", required=True)
    pa.add_argument("--noise_tag", default=None)
    pa.add_argument("--hands", nargs="+", default=None)
    pa.add_argument("--out", type=Path, default=None)
    pc = sub.add_parser("check")
    pc.add_argument("--tag", required=True)
    pc.add_argument("--hands", nargs="+", default=["O"])
    pc.add_argument("--show_d", action="store_true")
    pc.add_argument("--verbose", action="store_true")
    ps = sub.add_parser("summary")
    ps.add_argument("--tag", default="e33")
    ps.add_argument("--noise_tag", default="e33n")
    ps.add_argument("--out", type=Path, required=True)
    pf = sub.add_parser("frames")
    pf.add_argument("--draws", type=Path, required=True)
    pf.add_argument("--draw", type=int, default=0)
    pf.add_argument("--hand", default="O")
    pf.add_argument("--mu", type=float, default=1.3)
    pf.add_argument("--time_step", type=float, default=2.5e-4)
    a = p.parse_args()
    if a.cmd == "draws":
        draws = make_draws(a.n, a.seed)
        a.out.write_text(json.dumps({"seed": a.seed, "n": a.n, "rng": f"numpy.random.default_rng([{a.seed}, 33])",
                                     "ranges": RANDOM, "config": DEFAULTS, "draws": draws}, indent=1))
        print(f"{len(draws)} draws -> {a.out}")
        return 0
    return {"run": cmd_run, "analyze": cmd_analyze, "check": cmd_check, "frames": cmd_frames,
            "summary": cmd_summary}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main())
