"""Experiment 41: the fingertip rolls a ball (a trackball; experimental, docs/experimental/drake_ball.md).

A ball of radius r sits in a socket below the fingertip: its centre rides a vertical spring (k_socket, the socket's
seat), and the ball turns about y against a torsional spring (k_r) and a little damping. Its turn about x stays zero
by the finger's symmetry, so it is not modelled. Only the distal pad is meant to reach it. The planar CoDe index finger
of two_pad_drake.py (read-only: geometry, servo table, condition A's impedance; MCP fixed, palm down, gravity on).
O's open loop on every hand: flexion 0 -> q_t over approach_s (at q_t O's distal pad is tangent to the ball, back_mm
short of the pad's tip end), q_t -> q_t + sweep over sweep_s (the pad rolls the ball's top toward the palm), held
hold_s. Measure: the ball's turn at the end (deg, positive when its top moves toward the MCP); D = turn(2.0) - turn(1.3).

    uv run python ball_roll.py draws --n 64 --seed 41 --out logs/ball_e41.json [--ranges JSON] [--config JSON]
    uv run python ball_roll.py run --hands O --draws logs/ball_e41.json --tag e41 [--only 12]
    uv run python ball_roll.py check --tag e41 [--hands R U] [--show_d]
    uv run python ball_roll.py analyze --tag e41 --noise_tag e41n --out logs/e41_ball.json
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
SEAT_MASS_KG, BALL_MASS_KG = 0.01, 0.01
RANDOM = {"q_t_deg": (20.0, 35.0), "r_mm": (8.0, 12.0), "back_mm": (2.0, 6.0), "sweep_deg": (22.0, 30.0),
          "k_r": (0.02, 0.04)}
DEFAULTS = {"approach_s": 1.0, "sweep_s": 4.0, "hold_s": 1.0, "k_socket": 150.0, "d_socket": 4.0, "d_r": 2.0e-4,
            "time_step": 2.5e-4}


def make_draws(n, seed, ranges=RANDOM):
    rng = np.random.default_rng([seed, 41])
    cols = {k: rng.uniform(lo, hi, n) for k, (lo, hi) in ranges.items()}
    g = T.finger_geometry("O")
    qs, th = T.servo_table(g)
    end = g["pads"]["distal_end_mm"]
    out = []
    for i in range(n):
        d = {k: float(v[i]) for k, v in cols.items()}
        q = math.radians(d["q_t_deg"])
        nrm = T.rot(q) @ np.array([0.0, -1.0])  # O's distal pad's outward (palmar) normal at q_t
        c = T.reference_point(g, qs, th, q, end - d["back_mm"]) + d["r_mm"] * nrm
        out.append({"draw": i, **d, "x_c_mm": float(c[0]), "z_c_mm": float(c[1])})
    return out


def build(hand, mu, pl, opts, k_theta, d_theta):
    from pydrake.geometry import AddContactMaterial, Box, ProximityProperties, Sphere
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
    y, zax = np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])
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

    r = pl["r_mm"] * mm
    c = [pl["x_c_mm"] * mm, 0.0, pl["z_c_mm"] * mm - face]  # the ball's centre (world)
    seat = plant.AddRigidBody("seat", SpatialInertia(SEAT_MASS_KG, np.zeros(3), UnitInertia.SolidBox(0.02, 0.02, 0.005)))
    f_s = plant.AddFrame(FixedOffsetFrame("socket", world.body_frame(), RigidTransform(c)))
    sz = plant.AddJoint(PrismaticJoint("socket_z", f_s, seat.body_frame(), zax))
    plant.AddJointActuator("socket_z", sz).set_controller_gains(PdControllerGains(p=opts["k_socket"],
                                                                                  d=opts["d_socket"]))
    ball = plant.AddRigidBody("ball", SpatialInertia(BALL_MASS_KG, np.zeros(3), UnitInertia.SolidSphere(r)))
    rj = plant.AddJoint(RevoluteJoint("roll", seat.body_frame(), ball.body_frame(), y))
    plant.AddJointActuator("roll", rj).set_controller_gains(PdControllerGains(p=pl["k_r"], d=opts["d_r"]))

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
    plant.RegisterCollisionGeometry(ball, RigidTransform(), Sphere(r), "ball", props(1e8, mu))
    plant.Finalize()
    plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, -G])
    return builder, plant, sg, {"distal": distal, "proximal": proximal, "ball": ball, "seat": seat}


class Run:
    def __init__(self, hand, mu, pl, opts, k_theta, d_theta):
        from pydrake.systems.analysis import Simulator
        from pydrake.systems.framework import LeafSystem
        builder, plant, sg, bodies = build(hand, mu, pl, opts, k_theta, d_theta)
        self.plant, self.bodies = plant, bodies
        self.theta_cmd = 0.0
        nv, n_act = plant.num_velocities(), plant.num_actuators()
        iz = plant.GetJointByName("socket_z").velocity_start()
        carried = (SEAT_MASS_KG + BALL_MASS_KG) * G
        outer = self

        class Command(LeafSystem):
            def __init__(self):
                super().__init__()
                self.DeclareVectorOutputPort("desired", 2 * n_act, self.calc_desired)
                self.DeclareVectorOutputPort("load", nv, self.calc_load)

            def calc_desired(self, context, out):
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

    def forces(self):
        res = self.plant.get_contact_results_output_port().Eval(self.pctx)
        name = {v: k for k, v in self.idx.items()}
        out = {"distal": 0.0, "proximal": 0.0}
        for k in range(res.num_point_pair_contacts()):
            c = res.point_pair_contact_info(k)
            pair = {name.get(c.bodyA_index()), name.get(c.bodyB_index())}
            fn = abs(float(c.contact_force() @ c.point_pair().nhat_BA_W))
            if pair == {"distal", "ball"}:
                out["distal"] += fn
            elif "proximal" in pair:
                out["proximal"] += fn
        return out


def roll(hand, mu, pl, opts, delta_deg=0.0, record=False):
    geom = T.finger_geometry(hand)
    qs, th = T.servo_table(geom)
    q_t = math.radians(pl["q_t_deg"] + delta_deg)
    ratio = float(np.interp(q_t, qs, np.gradient(th, qs)))
    k_theta, d_theta = T.K_Q / ratio ** 2, T.D_Q / ratio ** 2
    s = Run(hand, mu, pl, opts, k_theta, d_theta)
    s.sim.Initialize()
    rj = s.plant.GetJointByName("roll")
    szj = s.plant.GetJointByName("socket_z")
    q1 = q_t + math.radians(pl["sweep_deg"])
    n_app, n_sw, n_hold = (int(round(opts[k] / T.DT_ENV)) for k in ("approach_s", "sweep_s", "hold_s"))
    fd_max = prox = 0.0
    touch = None
    trace, frames = [], []
    for i in range(n_app + n_sw + n_hold):
        q = q_t * (i + 1) / n_app if i < n_app else q_t + (q1 - q_t) * min(1.0, (i + 1 - n_app) / n_sw)
        s.theta_cmd = float(np.interp(q, qs, th))
        s.step()
        f = s.forces()
        fd_max, prox = max(fd_max, f["distal"]), max(prox, f["proximal"])
        if touch is None and f["distal"] > T.GATE_N:
            touch = math.degrees(q)
        turn = -math.degrees(rj.get_angle(s.pctx))
        if i >= n_app:
            trace.append([round(turn, 4), round(f["distal"], 4)])
        if record:
            X = s.plant.EvalBodyPoseInWorld(s.pctx, s.bodies["distal"])
            R = X.rotation().matrix()
            frames.append({"t": s.t, "q_cmd_deg": math.degrees(q), "q_deg": math.degrees(math.atan2(R[0, 2], R[0, 0])),
                           "turn_deg": turn, "socket_z_mm": 1e3 * szj.get_translation(s.pctx), "f_distal": f["distal"]})
    f = s.forces()
    out = {"rotation_deg": -math.degrees(rj.get_angle(s.pctx)), "rotation_max_deg": max(t[0] for t in trace),
           "socket_drop_mm": -1e3 * szj.get_translation(s.pctx), "touched": touch is not None, "touch_q_deg": touch,
           "distal_force_max_n": fd_max, "distal_force_end_n": f["distal"], "proximal_force_max_n": prox,
           "pen_mm": fd_max / POINT_K, "trace": trace}
    if record:
        out["frames"] = frames
    return out


def run_task(task):
    hand, pl, opts, delta = task
    t0 = time.time()
    rec = {"draw": pl["draw"], "hand": hand}
    for mu in MU:
        try:
            rec[f"{mu:g}"] = roll(hand, mu, pl, opts, delta)
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
    t0 = time.time()
    with Pool(a.workers) as pool:
        recs = pool.map(run_task, [(h, pl, opts, a.delta_deg) for h in a.hands for pl in draws], chunksize=1)
    for h in a.hands:
        rr = [r for r in recs if r["hand"] == h]
        (LOGS / f"ball_{h}_{a.tag}.json").write_text(json.dumps({"hand": h, "tag": a.tag, "opts": opts,
                                                                  "delta_deg": a.delta_deg, "draws": rr}))
        print(f"{h} {a.tag}: {len(rr)} draws, cpu {sum(r['wall_s'] for r in rr):.0f} s", flush=True)
    print(f"done in {time.time() - t0:.0f} s (workers {a.workers})", flush=True)
    return 0


# ----------------------------------------------------------------------------------------------------- analysis
MIN_D = 0.3  # deg
METRIC = "rotation_deg"


def _load(hand, tag):
    p = LOGS / f"ball_{hand}_{tag}.json"
    return {r["draw"]: r for r in json.loads(p.read_text())["draws"]} if p.exists() else None


def lost(x):
    return (not x["touched"]) or x["distal_force_end_n"] < T.LOSS_N


def o_keep(tag):
    o = _load("O", tag)
    keep, excl = {}, {"o_solver": 0, "o_lost": 0, "o_pen": 0, "o_proximal": 0, "small_d": 0}
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
        elif abs(runs[1][METRIC] - runs[0][METRIC]) < MIN_D:
            excl["small_d"] += 1
        else:
            keep[k] = runs
    return keep, excl


def hand_error(hand, tag, keep):
    h = _load(hand, tag)
    if h is None:
        return None
    rel, sens, pens = [], [], []
    cnt = {"solver": 0, "lost": 0, "over_pressed": 0, "proximal": 0}
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
    keep, excl = o_keep(a.tag)
    d_o = np.array([v[1][METRIC] - v[0][METRIC] for v in keep.values()])
    tr = np.array([v[0][METRIC] for v in keep.values()])
    print(f"{a.tag}: O kept {len(keep)}, excluded {excl}; D(O) {d_o.min():.2f}/{np.median(d_o):.2f}/{d_o.max():.2f} deg "
          f"(negative {int((d_o < 0).sum())}); rotation(O, mu 1.3) {tr.min():.1f}/{np.median(tr):.1f}/{tr.max():.1f}")
    out = {"tag": a.tag, "metric": METRIC, "o_kept": len(keep), "o_excluded": excl, "d_o_deg": d_o.tolist(), "rows": []}
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
              f"pairs {r['pairs']}/{len(keep)} lost {r['lost']} over {r['over_pressed']} prox {r['proximal']} | pen max "
              f"{r['pen_max_mm'] or 0:.2f}", flush=True)
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
        line = (f"{hand} {a.tag}: {len(rows)} draws, solver {fail}, lost {ls}, proximal {prox}, pen max {max(pen):.2f} mm "
                f"(> 1.5: {sum(p > PEN_MM for p in pen)}), median {np.median(pen):.2f}")
        if a.show_d:
            good = [r for r in ok if not any(lost(r[f"{mu:g}"]) for mu in MU)]
            dd = np.array([r["2"][METRIC] - r["1.3"][METRIC] for r in good])
            tr = np.array([r["1.3"][METRIC] for r in good])
            dr = np.array([r["1.3"]["socket_drop_mm"] for r in good])
            line += (f"; rotation(1.3) {tr.min():.1f}/{np.median(tr):.1f}/{tr.max():.1f}; D {dd.min():.2f}/{np.median(dd):.2f}/"
                     f"{dd.max():.2f} deg (|D|<0.3: {int((abs(dd) < 0.3).sum())}); socket drop {dr.min():.1f}..{dr.max():.1f} mm")
        print(line)
    return 0


def cmd_frames(a) -> int:
    d = json.loads(a.draws.read_text())
    pl = next(x for x in d["draws"] if x["draw"] == a.draw)
    out = roll(a.hand, a.mu, pl, _opts(d), record=True)
    p = LOGS / f"ball_frames_{a.hand}_d{a.draw}.json"
    p.write_text(json.dumps({"hand": a.hand, "mu": a.mu, "placement": pl, "opts": _opts(d), **out}))
    print(f"rotation {out['rotation_deg']:.2f} deg -> {p.name}")
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    pd_ = sub.add_parser("draws")
    pd_.add_argument("--n", type=int, default=64)
    pd_.add_argument("--seed", type=int, default=41)
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
    pf = sub.add_parser("frames")
    pf.add_argument("--draws", type=Path, required=True)
    pf.add_argument("--draw", type=int, default=0)
    pf.add_argument("--hand", default="O")
    pf.add_argument("--mu", type=float, default=1.3)
    a = p.parse_args()
    if a.cmd == "draws":
        ranges = {**RANDOM, **(json.loads(a.ranges) if a.ranges else {})}
        cfg = {**DEFAULTS, **(json.loads(a.config) if a.config else {})}
        draws = make_draws(a.n, a.seed, ranges)
        a.out.write_text(json.dumps({"seed": a.seed, "n": a.n, "rng": f"numpy.random.default_rng([{a.seed}, 41])",
                                     "ranges": ranges, "config": cfg, "draws": draws}, indent=1))
        print(f"{len(draws)} draws -> {a.out}")
        return 0
    return {"run": cmd_run, "analyze": cmd_analyze, "check": cmd_check, "frames": cmd_frames}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main())
