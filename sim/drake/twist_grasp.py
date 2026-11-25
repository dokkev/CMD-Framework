"""Experiment 39: grasp-and-twist (experimental; docs/experimental/drake_twist.md).

The pull-out grasp (experiment 21: condition A, the cylinder where O's two pads touch it at q0, setpoint q0 + 5 deg,
held while closing, released, settled, Gate 0), then a torque about the cylinder's own axis ramps until the grasp
gives way: the rotational pull-out. Two senses: +spin (the side on the distal pad moves toward the fingertip) and
-spin. Limit: the first of slip (a pad's tangential speed > 10 mm/s), loss (a pad's normal force < 0.2/3 N),
collapse (the centre moves 5 mm) or a turn past 30 deg; the limit torque is reported times three (three fingers, as
F_limit). D = tau_limit(mu 2.0) - tau_limit(mu 1.3). two_pad_drake.py is imported read-only; its Stage is subclassed
only to add the torque to the applied generalized forces.

    uv run python twist_grasp.py run --hands O --tag e39 [--only 8]
    uv run python twist_grasp.py analyze --tag e39 --noise_tag e39n --out logs/e39_twist.json
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
SENSES = ("+spin", "-spin")
RATE_NM_S = 0.01 / T.FINGERS  # torque ramp per finger (N m/s): about 4 s to a typical limit
MAX_NM = 0.3 / T.FINGERS
TURN_DEG = 30.0
PEN_MM = 1.5


class TwistStage(T.Stage):
    """two_pad_drake.Stage with a torque about the cylinder's spin added to the applied generalized forces."""

    def __init__(self, hand, mu, rho_mm, contact, time_step, hold, k_theta, d_theta):
        from pydrake.systems.analysis import Simulator
        from pydrake.systems.framework import LeafSystem

        builder, plant, scene_graph, bodies = T._build(hand, mu, rho_mm, contact, time_step, hold, k_theta, d_theta)
        ctrl = T.Controller()
        ctrl.torque = 0.0
        nv, n_act = plant.num_velocities(), plant.num_actuators()
        ix = plant.GetJointByName("slide_x").velocity_start()
        iz = plant.GetJointByName("slide_z").velocity_start()
        isp = plant.GetJointByName("spin").velocity_start()

        class Command(LeafSystem):
            def __init__(self):
                super().__init__()
                self.DeclareVectorOutputPort("desired", 2 * n_act, self.calc_desired)
                self.DeclareVectorOutputPort("load", nv, self.calc_load)

            def calc_desired(self, context, out):
                qd = [ctrl.theta_cmd] + (list(ctrl.hold_at)[: n_act - 1] if hold else [])
                out.SetFromVector(np.array(qd + [0.0] * n_act))

            def calc_load(self, context, out):
                tau = np.zeros(nv)
                tau[ix], tau[iz] = ctrl.load
                tau[isp] = ctrl.torque
                out.SetFromVector(tau)

        cmd = builder.AddSystem(Command())
        builder.Connect(cmd.get_output_port(0),
                        plant.get_desired_state_input_port(plant.GetModelInstanceByName("DefaultModelInstance")))
        builder.Connect(cmd.get_output_port(1), plant.get_applied_generalized_force_input_port())
        self.diagram = builder.Build()
        self.sim = Simulator(self.diagram)
        ctx = self.sim.get_mutable_context()
        plant.get_actuation_input_port().FixValue(plant.GetMyMutableContextFromRoot(ctx), np.zeros(n_act))
        self.plant, self.bodies, self.ctrl, self.sg = plant, bodies, ctrl, scene_graph
        self.pctx = plant.GetMyMutableContextFromRoot(ctx)
        self.inspector = scene_graph.model_inspector()
        self.t = 0.0


def twist_ramp(s, sense):
    m0 = s.measure()
    c0 = m0["c"].copy()
    spin = s.plant.GetJointByName("spin")
    a0 = spin.get_angle(s.pctx)
    sgn = 1.0 if sense == "+spin" else -1.0
    limit = {"event": None, "tau_limit_nm": None}
    curve = []
    steps = int(MAX_NM / (RATE_NM_S * T.DT_ENV)) + 1
    for i in range(steps):
        tq = RATE_NM_S * T.DT_ENV * (i + 1)
        s.ctrl.torque = sgn * tq
        s.step()
        m_ = s.measure()
        turn = math.degrees(abs(spin.get_angle(s.pctx) - a0))
        disp = 1e3 * float(np.linalg.norm((m_["c"] - c0)[[0, 2]]))
        curve.append([round(tq * T.FINGERS, 5), round(turn, 4), round(disp, 4)])
        event = None
        k = int(np.argmax(m_["slip"]))
        if 1e3 * m_["slip"][k] > T.SLIP_MM_S:
            event = "slip"
        elif (m_["normal"] < T.LOSS_N).any():
            event = "loss"
        elif disp > T.COLLAPSE_MM:
            event = "collapse"
        elif turn > TURN_DEG:
            event = "turn"
        if event:
            limit = {"event": event, "tau_limit_nm": float(tq * T.FINGERS)}
            break
    return limit, curve


def job(task):
    hand, r, g, delta = task
    import preload_match as P
    t0 = time.time()
    rec = {"hand": hand, "rho_mm": r, "q0_deg": g["q0_deg"], "x_c_mm": g["x_c_mm"], "runs": {}}
    try:
        for mu in MU:
            s, m_, gate, pre, c = P.pull_settle(hand, mu, r, g, g["q0_deg"] + 5.0 + delta)
            pre["pen_mm"] = float(max(m_["normal"].max(), 0.0)) / 2.5
            run = {"gate0": gate, "pre": pre, "senses": {}}
            if gate["pass"]:
                state = s.plant.GetPositionsAndVelocities(s.pctx).copy()
                for sense in SENSES:
                    b = TwistStage(hand, mu, r, "point", 1.0e-3, False, c["k_theta"], c["d_theta"])
                    b.plant.SetPositionsAndVelocities(b.pctx, state)
                    b.ctrl.theta_cmd = s.ctrl.theta_cmd
                    b.sim.Initialize()
                    lim, curve = twist_ramp(b, sense)
                    run["senses"][sense] = {"limit": lim, "curve": curve}
            rec["runs"][f"{mu:g}"] = run
    except RuntimeError as err:
        rec["solver_fail"] = str(err)[:200]
    rec["wall_s"] = time.time() - t0
    return rec


RADII = (12.5, 15.0, 17.5, 20.0)


def places():
    """rho 12.5-20 mm x q0 80-115 deg (5 deg steps), the cylinder where O's two pads touch it at q0 (the
    two_pad_grasps rule, dial_placements.x_contact; at rho 15 and 20 these are the pull-out's placements)."""
    from dial_placements import x_contact
    g = T.finger_geometry("O")
    qs, th = T.servo_table(g)
    out = []
    for r in RADII:
        for q0 in np.arange(80.0, 115.0 + 1e-9, 5.0):
            x = x_contact(g, qs, th, float(q0), r)
            if x is not None:
                out.append((r, {"rho_mm": r, "q0_deg": float(q0), "x_c_mm": float(x)}))
    return out


def cmd_run(a) -> int:
    pl = places()[: a.only] if a.only else places()
    t0 = time.time()
    with Pool(a.workers) as pool:
        recs = pool.map(job, [(h, r, g, a.delta_deg) for h in a.hands for r, g in pl], chunksize=1)
    for h in a.hands:
        rr = [x for x in recs if x["hand"] == h]
        (LOGS / f"twist_{h}_{a.tag}.json").write_text(json.dumps({"hand": h, "tag": a.tag, "places": rr}))
        g0 = sum(x.get("runs", {}).get("1.3", {}).get("gate0", {}).get("pass", False) for x in rr)
        print(f"{h} {a.tag}: {len(rr)} placements, Gate 0 (mu 1.3) {g0}, cpu {sum(x['wall_s'] for x in rr):.0f} s",
              flush=True)
    print(f"done in {time.time() - t0:.0f} s (workers {a.workers})", flush=True)
    return 0


# ----------------------------------------------------------------------------------------------------- analysis
D_MIN = 3 * RATE_NM_S * T.DT_ENV * T.FINGERS  # three torque steps


def _load(hand, tag):
    p = LOGS / f"twist_{hand}_{tag}.json"
    if not p.exists():
        return None
    return {(x["rho_mm"], x["q0_deg"], round(x["x_c_mm"], 3)): x for x in json.loads(p.read_text())["places"]}


def _lim(rec, mu, s):
    if rec is None or rec.get("solver_fail") or not rec["runs"][f"{mu:g}"]["gate0"]["pass"]:
        return None
    return rec["runs"][f"{mu:g}"]["senses"][s]["limit"]["tau_limit_nm"]


def o_keep(tag):
    o = _load("O", tag)
    keep, excl = {}, {"o_solver": 0, "o_gate0": 0, "o_pen": 0, "o_no_limit": 0, "small_d": 0}
    for k, r in sorted(o.items()):
        for s in SENSES:
            if r.get("solver_fail"):
                excl["o_solver"] += 1
            elif not all(r["runs"][f"{mu:g}"]["gate0"]["pass"] for mu in MU):
                excl["o_gate0"] += 1
            elif any(r["runs"][f"{mu:g}"]["pre"]["pen_mm"] > PEN_MM for mu in MU):
                excl["o_pen"] += 1
            elif any(_lim(r, mu, s) is None for mu in MU):
                excl["o_no_limit"] += 1
            elif abs(_lim(r, 2.0, s) - _lim(r, 1.3, s)) < D_MIN - 1e-12:
                excl["small_d"] += 1
            else:
                keep[k + (s,)] = _lim(r, 2.0, s) - _lim(r, 1.3, s)
    return keep, excl


def hand_error(hand, tag, keep):
    h = _load(hand, tag)
    if h is None:
        return None
    rel, sens, pens = [], [], []
    cnt = {"solver": 0, "gate0": 0, "over_pressed": 0, "no_limit": 0}
    for k, d_o in keep.items():
        r = h.get(k[:3])
        if r is None or r.get("solver_fail"):
            cnt["solver"] += 1
            sens.append(math.inf)
            continue
        if not all(r["runs"][f"{mu:g}"]["gate0"]["pass"] for mu in MU):
            cnt["gate0"] += 1
            sens.append(math.inf)
            continue
        pen = max(r["runs"][f"{mu:g}"]["pre"]["pen_mm"] for mu in MU)
        pens.append(pen)
        if pen > PEN_MM:
            cnt["over_pressed"] += 1
            sens.append(math.inf)
            continue
        t = [_lim(r, mu, k[3]) for mu in MU]
        if any(x is None for x in t):
            cnt["no_limit"] += 1
            continue
        e = abs((t[1] - t[0]) - d_o) / abs(d_o)
        rel.append(e)
        sens.append(e)
    return {"E": float(np.median(rel)) if rel else None, "E_p75": float(np.percentile(rel, 75)) if rel else None,
            "pairs": len(rel), "E_sens": float(np.median(sens)) if sens else None, **cnt,
            "pen_max_mm": float(max(pens)) if pens else None}


def deviation(tag, keep):
    """R and U minus O: the torque held up to each turn of the cylinder (0-10 deg grid), per mu."""
    grid = np.linspace(0.0, 10.0, 101)

    def held(curve):
        c = np.asarray(curve)
        tt = np.maximum.accumulate(c[:, 1])
        return np.interp(grid, tt, c[:, 0], right=c[-1, 0])
    o = _load("O", tag)
    out = {"x": grid.round(3).tolist()}
    for hand in ("R", "U"):
        h = _load(hand, tag)
        for mu in MU:
            ys = []
            for k in keep:
                r = h.get(k[:3])
                if r is None or r.get("solver_fail") or not r["runs"][f"{mu:g}"]["gate0"]["pass"]:
                    continue
                ys.append(held(r["runs"][f"{mu:g}"]["senses"][k[3]]["curve"]) -
                          held(o[k[:3]]["runs"][f"{mu:g}"]["senses"][k[3]]["curve"]))
            y = np.array(ys)
            out[f"{hand}_{mu:g}"] = {"median": np.median(y, 0).round(5).tolist(),
                                     "q25": np.percentile(y, 25, 0).round(5).tolist(),
                                     "q75": np.percentile(y, 75, 0).round(5).tolist(), "n": len(ys)}
    return out


def cmd_analyze(a) -> int:
    from analyze import DESIGN, spearman_perm
    import collections
    keep, excl = o_keep(a.tag)
    d_o = np.array(list(keep.values()))
    print(f"{a.tag}: O pairs {len(keep)} ({dict(collections.Counter(k[3] for k in keep))}), excluded {excl}; D(O) "
          f"{d_o.min():+.4f}/{np.median(d_o):+.4f}/{d_o.max():+.4f} N m (negative {int((d_o < 0).sum())})")
    out = {"tag": a.tag, "o_pairs": len(keep), "o_excluded": excl, "d_o_nm": d_o.tolist(), "rows": []}
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
              f"pairs {r['pairs']}/{len(keep)} gate0 {r['gate0']} over {r['over_pressed']} no_limit {r['no_limit']} "
              f"| pen max {r['pen_max_mm'] or 0:.2f}", flush=True)
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
            print(f"deviation {k}: peak {m[i]:+.5f} N m at {out['deviation']['x'][i]:.1f} deg, end {m[-1]:+.5f}, "
                  f"n {out['deviation'][k]['n']}")
    if a.out:
        a.out.write_text(json.dumps(out, indent=1, default=float))
    return 0


def cmd_check(a) -> int:
    import collections
    for hand in a.hands:
        h = _load(hand, a.tag)
        g0 = sum(x["runs"]["1.3"]["gate0"]["pass"] for x in h.values() if "runs" in x)
        pens = [max(x["runs"][f"{mu:g}"]["pre"]["pen_mm"] for mu in MU) for x in h.values() if "runs" in x]
        ev = collections.Counter(x["runs"][f"{mu:g}"]["senses"][s]["limit"]["event"] for x in h.values() if "runs" in x
                                 for mu in MU if x["runs"][f"{mu:g}"]["gate0"]["pass"] for s in SENSES)
        line = f"{hand} {a.tag}: Gate 0 {g0}/{len(h)}, pen max {max(pens):.2f} mm, events {dict(ev)}"
        if a.show_d and hand == "O":
            keep, excl = o_keep(a.tag)
            d = np.array(list(keep.values()))
            line += (f"; O pairs {len(keep)} excl {excl}; D(O) {d.min():+.4f}/{np.median(d):+.4f}/{d.max():+.4f} N m"
                     if len(d) else f"; O pairs 0 excl {excl}")
        print(line)
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    pr = sub.add_parser("run")
    pr.add_argument("--hands", nargs="+", default=["O"])
    pr.add_argument("--tag", required=True)
    pr.add_argument("--only", type=int, default=None)
    pr.add_argument("--delta_deg", type=float, default=0.0)
    pr.add_argument("--workers", type=int, default=6)
    pa = sub.add_parser("analyze")
    pa.add_argument("--tag", required=True)
    pa.add_argument("--noise_tag", default=None)
    pa.add_argument("--out", type=Path, default=None)
    pc = sub.add_parser("check")
    pc.add_argument("--tag", required=True)
    pc.add_argument("--hands", nargs="+", default=["O"])
    pc.add_argument("--show_d", action="store_true")
    a = p.parse_args()
    return {"run": cmd_run, "analyze": cmd_analyze, "check": cmd_check}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main())
