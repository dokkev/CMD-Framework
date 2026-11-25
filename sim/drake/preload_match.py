"""Experiment 31, condition F: preload-matched, shared incremental command, no force feedback (experimental;
docs/experimental/drake_preload.md).

Condition A reuses O's flexion setpoint, so each redesigned hand starts from its own initial contact force. F removes
that difference once, before the motion: for each hand and placement the PIP setpoint is found by bisection (Illinois
false position) so that the settled distal normal force at mu 1.3 equals O's (the simulator's force stands in for a
load cell). Nothing is fed back afterwards: every hand then gets the same incremental flexion command dq(t) added to
its own matched setpoint, at both mu (the mu 1.3 setpoint is reused at mu 2.0).

Two tasks, both on two_pad_drake.py (imported read-only):
  pull: experiment 21's two-pad pull-out (O's placements, rho 15 / 20 mm, four load directions). Close and settle at
        the matched setpoint exactly as condition A, then dq = +inc_deg over inc_s, held hold_s, then the standard load
        ramp from that state, once per direction. Measures: F_limit, and the pads' normal forces over the increment,
        the hold and the ramp.
  disk: experiment 29's fingertip disk z (seed 29 draws). Close 0 -> q_set over approach_s and settle settle_s; q_set
        is matched so the settled distal normal force equals O's at q_t + pre_deg (O's own matched setpoint). Then the
        sweep q_set -> q_set + 40 deg over 3 s, held 1 s, as experiment 29. Measures: the disk's rotation, the distal
        normal force over the sweep and hold.

    uv run python preload_match.py check                       # reimplementation == conditions A / experiment 29 (O)
    uv run python preload_match.py pull --hands O --tag e31 --workers 6 [--only 3] [--inc_deg 5]
    uv run python preload_match.py disk --hands O --tag e31 --workers 6 [--draws 0 1 2] [--pre_deg 2]
    uv run python preload_match.py analyze --task pull --tag e31 --noise_tag e31n [--out logs/e31_pull.json]
"""
from __future__ import annotations

import argparse
import json
import math
import time
from functools import lru_cache
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import two_pad_drake as T

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
MU = (1.3, 2.0)
POINT_K = 2.5  # N/mm: a pad's point-contact stiffness (penetration = force / this)
PEN_MM = 1.5
TOL_REL, TOL_ABS = 0.02, 0.02  # matching tolerance: +-2 % or +-0.02 N, whichever is larger
HANDS = ["O", "R", "U", "h3to5_hr", "h3to5_hu", "h3to9_hr", "h3to9_hu", "d1p5_minDs_s0", "d2p25_minDs_s0",
         "d2p25_maxDs_s0", "d3_minDs_s0", "d4p5_s0", "d5p5_s0", "d1p5_maxDs_s0", "d3_maxDs_s0", "h3to5_d3_s0",
         "h3to5_d3_minDs_s0", "h3to9_d3p5_maxDs_s0", "h3to9_d5_minDs_s0"]


@lru_cache(maxsize=None)
def table(hand: str):
    """The hand's geometry and servo table, to 140 deg so that a command past 125 deg is still mapped (the table's
    entries up to 125 deg are those condition A used)."""
    g = T.finger_geometry(hand)
    qs, th = T.servo_table(g, 140.0)
    return g, qs, th


def gains(qs, th, q_cmd):
    ratio = float(np.interp(q_cmd, qs, np.gradient(th, qs)))
    return ratio, T.K_Q / ratio ** 2, T.D_Q / ratio ** 2


# ----------------------------------------------------------------------------------------------------- pull-out
def pull_settle(hand, mu, rho_mm, g, q_set_deg, time_step=1.0e-3):
    """Condition A's closing and settling at the flexion setpoint q_set_deg (the cylinder held while the finger closes
    40 + 30 steps, then free for 120): returns the Stage, the settled measurement, Gate 0 and the context."""
    geom, qs, th = table(hand)
    q_cmd = math.radians(q_set_deg)
    theta_cmd = float(np.interp(q_cmd, qs, th))
    ratio, k_theta, d_theta = gains(qs, th, q_cmd)
    face = (geom["pads"]["mount_offset_mm"] + geom["pads"]["thickness_mm"]) * 1e-3
    x0 = np.array([g["x_c_mm"] * 1e-3, -(face + rho_mm * 1e-3)])
    s1 = T.Stage(hand, mu, rho_mm, "point", time_step, True, k_theta, d_theta)
    s1.plant.GetJointByName("slide_x").set_translation(s1.pctx, x0[0])
    s1.plant.GetJointByName("slide_z").set_translation(s1.pctx, x0[1])
    s1.ctrl.hold_at = np.array([x0[0], x0[1], 0.0])
    s1.sim.Initialize()
    for i in range(40):
        s1.ctrl.theta_cmd = theta_cmd * (i + 1) / 40
        s1.step()
    for _ in range(30):
        s1.step()
    state = s1.plant.GetPositionsAndVelocities(s1.pctx).copy()
    s = T.Stage(hand, mu, rho_mm, "point", time_step, False, k_theta, d_theta)
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
    pads_ok = bool((m_["normal"] >= T.GATE_N).all())
    still = speed < T.GATE_SPEED_MM_S and spin < T.GATE_SPIN_RAD_S
    ang = math.degrees(math.acos(float(np.clip(m_["n"][0] @ m_["n"][1], -1, 1)))) if pads_ok else float("nan")
    angle_ok = pads_ok and abs(ang - math.degrees(m_["q"])) <= 4.0
    gate = {"pass": bool(pads_ok and still and angle_ok), "pads": pads_ok, "still": bool(still),
            "angle": bool(angle_ok), "speed_mm_s": float(speed), "spin_rad_s": float(spin)}
    pre = {"normal_n": m_["normal"].tolist(), "q_deg": math.degrees(m_["q"]), "q_cmd_deg": q_set_deg,
           "normal_angle_deg": ang, "ratio": ratio}
    return s, m_, gate, pre, {"qs": qs, "th": th, "k_theta": k_theta, "d_theta": d_theta}


def ramp(s, m0, direction, trace):
    """Condition A's load ramp from the Stage's current state (the same code as two_pad_drake._run_env); appends
    (distal, proximal) normal forces per step to trace."""
    d = np.array(T.DIRECTIONS[direction])
    c0 = m0["c"].copy()
    limit = {"event": None, "f_limit_n": None, "mu_eff_geo_at_slip": None, "slip_pad": None, "roll_n": None,
             "steps": 0}
    steps = int(T.MAX_FORCE_N / (T.RATE_N_S * T.DT_ENV)) + 1
    for i in range(steps):
        f = T.RATE_N_S * T.DT_ENV * (i + 1)
        s.ctrl.load = f * d
        s.step()
        m_ = s.measure()
        trace.append((round(float(m_["normal"][1]), 5), round(float(m_["normal"][0]), 5)))
        disp = 1e3 * float(np.linalg.norm((m_["c"] - c0)[[0, 2]]))
        event = None
        k = int(np.argmax(m_["slip"]))
        if 1e3 * m_["slip"][k] > T.SLIP_MM_S:
            event = "slip"
            fn = m_["normal"][k]
            ft = float(np.linalg.norm(m_["force"][k] - fn * m_["n"][k]))
            limit["mu_eff_geo_at_slip"], limit["slip_pad"] = float(ft / max(abs(fn), 1e-9)), int(k)
        elif (m_["normal"] < T.LOSS_N).any():
            event = "loss"
        elif disp > T.COLLAPSE_MM:
            event = "collapse"
        if limit["roll_n"] is None and disp > T.ROLL_MM and event is None:
            limit["roll_n"] = float(f * T.FINGERS)
        if event:
            limit["event"], limit["f_limit_n"] = event, float(f * T.FINGERS)
            break
    limit["steps"] = len(trace)
    return limit


def pull_run(hand, mu, rho_mm, g, q_set_deg, opts):
    """Condition F at one placement and mu: settle at q_set, the shared increment and hold (no load), then the load
    ramp in each direction from the held state. With inc_deg = 0 and hold_s = 0 this is condition A."""
    s, m_, gate, pre, c = pull_settle(hand, mu, rho_mm, g, q_set_deg, opts.get("time_step", 1.0e-3))
    out = {"rho_mm": rho_mm, "q0_deg": g["q0_deg"], "x_c_mm": g["x_c_mm"], "q_set_deg": q_set_deg, "pre": pre,
           "gate0": gate, "dirs": {}}
    if not gate["pass"]:
        return out
    q_set = math.radians(q_set_deg)
    n_inc, n_hold = int(round(opts["inc_s"] / T.DT_ENV)), int(round(opts["hold_s"] / T.DT_ENV))
    inc = math.radians(opts["inc_deg"])
    trace, speed = [], 0.0
    pen = max(m_["normal"].max(), 0.0) / POINT_K
    for i in range(n_inc + n_hold):
        q = q_set + inc * min(1.0, (i + 1) / n_inc) if n_inc else q_set + inc
        s.ctrl.theta_cmd = float(np.interp(q, c["qs"], c["th"]))
        s.step()
        m_ = s.measure()
        trace.append((round(float(m_["normal"][1]), 5), round(float(m_["normal"][0]), 5)))
        pen = max(pen, float(m_["normal"].max()) / POINT_K)
        if i >= n_inc + n_hold - 10:
            speed = max(speed, 1e3 * float(np.linalg.norm(m_["v"][[0, 2]])))
    held = bool((m_["normal"] >= T.GATE_N).all()) and speed < T.GATE_SPEED_MM_S if n_inc + n_hold else True
    out.update(inc_trace=trace, pen_pre_mm=pen, gate1={"pass": held, "speed_mm_s": speed,
                                                       "normal_n": m_["normal"].tolist(),
                                                       "q_deg": math.degrees(m_["q"])},
               q_cmd_end_deg=math.degrees(q_set + inc))
    if not held:
        return out
    state = s.plant.GetPositionsAndVelocities(s.pctx).copy()
    theta_end = s.ctrl.theta_cmd
    for direction in opts["directions"]:
        if n_inc + n_hold == 0 and len(opts["directions"]) == 1 and opts.get("no_branch"):
            b = s  # the reproduction check: continue the same simulator, exactly as condition A
        else:
            b = T.Stage(hand, mu, rho_mm, "point", opts.get("time_step", 1.0e-3), False, c["k_theta"], c["d_theta"])
            b.plant.SetPositionsAndVelocities(b.pctx, state)
            b.ctrl.theta_cmd = theta_end
            b.sim.Initialize()
        m0 = b.measure()
        rt = []
        limit = ramp(b, m0, direction, rt)
        out["dirs"][direction] = {"limit": limit, "ramp_trace": rt}
    return out


def match(f, target, x0, step, lo_bound, hi_bound, max_iter=30):
    """Illinois false position on f(x) = target, f increasing, from a bracket grown around x0 by step; stops when
    |f - target| <= 0.5 % of the target (a tighter aim than the tolerance) or the bracket is narrower than 1e-4 deg.
    Returns (x, f(x), evaluations, converged within the tolerance +-2 % or +-0.02 N, history)."""
    tol = max(TOL_REL * target, TOL_ABS)
    aim = max(0.005 * abs(target), 1e-5)
    hist = []

    def ev(x):
        y = f(x)
        hist.append((round(x, 6), round(y, 6)))
        return y - target

    lo, hi = max(x0 - step, lo_bound), min(x0 + step, hi_bound)
    flo, fhi = ev(lo), ev(hi)
    while flo > 0 and lo > lo_bound:
        hi, fhi = lo, flo
        lo = max(lo - step, lo_bound)
        flo = ev(lo)
    while fhi < 0 and hi < hi_bound:
        lo, flo = hi, fhi
        hi = min(hi + step, hi_bound)
        fhi = ev(hi)
    if flo > 0 or fhi < 0:  # no bracket in range
        best = min(hist, key=lambda h: abs(h[1] - target))
        return best[0], best[1], len(hist), False, hist
    side = 0
    for _ in range(max_iter):
        if min(abs(flo), abs(fhi)) <= aim:
            break
        if hi - lo < 1e-4:
            break
        x = (lo * fhi - hi * flo) / (fhi - flo)
        if not (lo < x < hi) or hi - lo > 4.0:  # bisection while the bracket is wide
            x = 0.5 * (lo + hi)
        fx = ev(x)
        if fx < 0:
            lo, flo = x, fx
            if side == -1:
                fhi /= 2
            side = -1
        else:
            hi, fhi = x, fx
            if side == 1:
                flo /= 2
            side = 1
    best = min(hist, key=lambda h: abs(h[1] - target))
    return best[0], best[1], len(hist), abs(best[1] - target) <= tol, hist


def a_target(rho_mm: float, q0: float, x_c: float):
    """O's settled distal normal force at this placement under condition A, mu 1.3 (experiment 21's log)."""
    envs = _a_log(rho_mm)
    for e in envs:
        if e["q0_deg"] == q0 and abs(e["x_c_mm"] - x_c) < 1e-6 and e["gate0"]["pass"]:
            return float(e["pre"]["normal_n"][1])
    return None


@lru_cache(maxsize=None)
def _a_log(rho_mm):
    return json.loads((LOGS / f"dk_O_mu1.3_point-e21_r{rho_mm:g}.json").read_text())["envs"]


def pull_task(task):
    hand, g, rho_mm, opts = task
    t0 = time.time()
    target = a_target(rho_mm, g["q0_deg"], g["x_c_mm"])
    base = g["q0_deg"] + 5.0
    rec = {"hand": hand, "rho_mm": rho_mm, "q0_deg": g["q0_deg"], "x_c_mm": g["x_c_mm"], "target_n": target}
    if target is None:
        rec["skip"] = "O fails Gate 0 under A at mu 1.3"
        return rec
    try:
        if hand == "O":  # O's matched setpoint is its condition-A setpoint (+ the noise-floor offset)
            q_set, f_set, n_ev, ok, hist = base + opts.get("offset_deg", 0.0), target, 0, True, []
        else:
            f = lambda q: float(pull_settle(hand, 1.3, rho_mm, g, q)[1]["normal"][1])
            q_set, f_set, n_ev, ok, hist = match(f, target, base, 4.0, 0.0, 135.0)
        rec["match"] = {"q_set_deg": q_set, "force_n": f_set, "evals": n_ev, "converged": bool(ok), "history": hist,
                        "shift_deg": q_set - base}
        rec["runs"] = {}
        for mu in MU:
            rec["runs"][f"{mu:g}"] = pull_run(hand, mu, rho_mm, g, q_set, opts)
    except RuntimeError as err:
        rec["solver_fail"] = str(err)[:200]
    rec["wall_s"] = time.time() - t0
    return rec


def pull_main(a):
    grasps = json.loads(T.GRASPS.read_text())["grasps"]
    opts = {"inc_deg": a.inc_deg, "inc_s": a.inc_s, "hold_s": a.hold_s, "directions": a.directions,
            "offset_deg": a.offset_deg}
    places = []
    for r in (15.0, 20.0):
        gs = [g for g in grasps if g["rho_mm"] == r and a_target(r, g["q0_deg"], g["x_c_mm"]) is not None]
        places += [(r, g) for g in gs][: a.only]
    for hand in a.hands:
        t0 = time.time()
        tasks = [(hand, g, r, opts) for r, g in places]
        with Pool(a.workers) as pool:
            recs = pool.map(pull_task, tasks, chunksize=1)
        name = LOGS / f"pm_pull_{hand}_{a.tag}.json"
        name.write_text(json.dumps({"hand": hand, "opts": opts, "places": recs}))
        conv = sum(r.get("match", {}).get("converged", False) for r in recs)
        print(f"{hand}: {len(recs)} placements, matched {conv}, {time.time() - t0:.0f} s -> {name.name}", flush=True)
    return 0


# ----------------------------------------------------------------------------------------------------- disk
def _set_disk_globals(g):
    T.TIP_MODE, T.TIP_THICK_MM = "z", 4.0
    T.TIP_KR, T.TIP_DR = g["tip_kr"], 1.0e-4
    T.TIP_AXIS_DEG, T.TIP_Y_MM = 0.0, g["y_mm"]
    T.DIAL_Z_STIFFNESS = 60.0


def disk_run(hand, mu, g, q_set_deg, opts, preload_only=False, gains_at=None):
    """The fingertip disk z with a preload phase: 0 -> q_set over approach_s, held settle_s, then q_set -> q_set +
    sweep over sweep_s, held hold_s (all at the gains of q_set). With settle_s = 0 and q_set = q_t this is
    two_pad_drake.tipdial (experiment 29). Returns the rotation (deg), the settled distal normal force (the pad's push
    on the disk along -z) and per-step traces over the sweep and hold."""
    _set_disk_globals(g)
    geom, qs, th = table(hand)
    q_set = math.radians(q_set_deg)
    _, k_theta, d_theta = gains(qs, th, math.radians(gains_at) if gains_at is not None else q_set)
    rho_mm = g["rho_mm"]
    s = T.Stage(hand, mu, rho_mm, "point", opts.get("time_step", 1.0e-3), "dial", k_theta, d_theta)
    pads = geom["pads"]
    m_mm = pads["mount_offset_mm"]
    face = m_mm + pads["thickness_mm"]
    c = np.array([g["x_c_mm"], g["z_c_mm"] - face]) * 1e-3
    s.plant.GetJointByName("slide_x").set_translation(s.pctx, float(c[0]))
    s.plant.GetJointByName("slide_z").set_translation(s.pctx, float(c[1]))
    s.ctrl.hold_at = np.array([float(c[0]), float(c[1]), 0.0])
    s.sim.Initialize()
    spin = s.plant.GetJointByName("spin")
    n_app, n_set = int(round(opts["approach_s"] / T.DT_ENV)), int(round(opts["settle_s"] / T.DT_ENV))
    n_sweep, n_hold = int(round(opts["sweep_s"] / T.DT_ENV)), int(round(opts["hold_s"] / T.DT_ENV))
    q1 = q_set + math.radians(opts["sweep_deg"])
    touch_q, prox_f, fmax, fmax_pre = None, 0.0, 0.0, 0.0
    trace, settled, rot_pre = [], None, 0.0
    total = n_app + n_set + (0 if preload_only else n_sweep + n_hold)
    for i in range(total):
        if i < n_app:
            q = q_set * (i + 1) / n_app
        elif i < n_app + n_set:
            q = q_set
        else:
            q = q_set + (q1 - q_set) * min(1.0, (i + 1 - n_app - n_set) / n_sweep)
        s.ctrl.theta_cmd = float(np.interp(q, qs, th))
        s.step()
        force, _ = s.contacts()
        fd, fp = float(np.linalg.norm(force[1])), float(np.linalg.norm(force[0]))
        fn = float(-force[1][2])  # the pad's push on the disk along -z, the disk face's normal
        fmax, prox_f = max(fmax, fd), max(prox_f, fp)
        if i < n_app + n_set:
            fmax_pre = max(fmax_pre, fd)
        if touch_q is None and fd > T.GATE_N:
            touch_q = math.degrees(s.measure()["q"])
        if i == n_app + n_set - 1:
            settled, rot_pre = fn, -math.degrees(spin.get_angle(s.pctx))
        if i >= n_app + n_set:
            trace.append((round(fn, 5), round(-math.degrees(spin.get_angle(s.pctx)), 4)))
    m_ = s.measure()
    rot = -math.degrees(spin.get_angle(s.pctx))
    return {"q_set_deg": q_set_deg, "settled_normal_n": settled, "rotation_pre_deg": rot_pre, "rotation_deg": rot,
            "rotation_sweep_deg": rot - rot_pre, "touched": touch_q is not None, "touch_q_deg": touch_q,
            "distal_force_max_n": fmax, "distal_force_max_pre_n": fmax_pre, "proximal_force_max_n": prox_f,
            "q_end_deg": math.degrees(m_["q"]), "trace": trace}


DISK_OPTS = {"approach_s": 1.0, "settle_s": 0.5, "sweep_s": 3.0, "hold_s": 1.0, "sweep_deg": 40.0}


def disk_task(task):
    hand, g, opts = task
    t0 = time.time()
    rec = {"hand": hand, "draw": g["draw"], "q_t_deg": g["q0_deg"]}
    try:
        q_o = g["q0_deg"] + opts["pre_deg"]
        target = disk_run("O", 1.3, g, q_o, opts, preload_only=True)["settled_normal_n"]
        rec["target_n"] = target
        goal = target * opts.get("target_scale", 1.0)
        if hand == "O" and opts.get("target_scale", 1.0) == 1.0:
            q_set, f_set, n_ev, ok, hist = q_o, target, 0, True, []
        else:
            f = lambda q: disk_run(hand, 1.3, g, q, opts, preload_only=True)["settled_normal_n"]
            q_set, f_set, n_ev, ok, hist = match(f, goal, q_o, 3.0, 0.0, 135.0)
        rec["match"] = {"q_set_deg": q_set, "force_n": f_set, "goal_n": goal, "evals": n_ev, "converged": bool(ok),
                        "history": hist, "shift_deg": q_set - q_o}
        rec["runs"] = {f"{mu:g}": disk_run(hand, mu, g, q_set, opts) for mu in MU}
    except RuntimeError as err:
        rec["solver_fail"] = str(err)[:200]
    rec["wall_s"] = time.time() - t0
    return rec


def disk_main(a):
    draws = json.loads((LOGS / "tipdial_z_e29.json").read_text())["grasps"]
    keep = set(a.draws) if a.draws else set(json.loads((LOGS / "e31_disk_draws.json").read_text())["draws"])
    draws = [g for g in draws if g["draw"] in keep]
    opts = {**DISK_OPTS, "pre_deg": a.pre_deg, "target_scale": a.target_scale}
    for hand in a.hands:
        t0 = time.time()
        with Pool(a.workers) as pool:
            recs = pool.map(disk_task, [(hand, g, opts) for g in draws], chunksize=1)
        name = LOGS / f"pm_disk_{hand}_{a.tag}.json"
        name.write_text(json.dumps({"hand": hand, "opts": opts, "draws": recs}))
        conv = sum(r.get("match", {}).get("converged", False) for r in recs)
        print(f"{hand}: {len(recs)} draws, matched {conv}, {time.time() - t0:.0f} s -> {name.name}", flush=True)
    return 0


# ----------------------------------------------------------------------------------------------------- check
def check_main(a):
    """The reimplementation reproduces condition A (experiment 21, O, mu 1.3, rho 15, q0 95, direction +x) and
    experiment 29 (O, z draw 0, mu 1.3) exactly."""
    grasps = [g for g in json.loads(T.GRASPS.read_text())["grasps"] if g["rho_mm"] == 15.0 and g["q0_deg"] == 95.0]
    g = grasps[0]
    for direction in ("+x", "-z"):
        ref = next(e for e in _a_log(15.0) if e["q0_deg"] == 95.0 and e["direction"] == direction)
        out = pull_run("O", 1.3, 15.0, g, 100.0, {"inc_deg": 0.0, "inc_s": 0.0, "hold_s": 0.0,
                                                  "directions": [direction], "no_branch": True})
        lim = out["dirs"][direction]["limit"]
        print(f"pull O q0 95 {direction}: settled {out['pre']['normal_n']} vs A {ref['pre']['normal_n']}; F_limit "
              f"{lim['f_limit_n']} ({lim['event']}) vs A {ref['limit']['f_limit_n']} ({ref['limit']['event']}); "
              f"exact: {out['pre']['normal_n'] == ref['pre']['normal_n'] and lim['f_limit_n'] == ref['limit']['f_limit_n']}")
        out = pull_run("O", 1.3, 15.0, g, 100.0, {"inc_deg": 0.0, "inc_s": 0.0, "hold_s": 0.0,
                                                  "directions": [direction]})
        lim = out["dirs"][direction]["limit"]
        print(f"   branched ramp (new simulator from the settled state): F_limit {lim['f_limit_n']} ({lim['event']})")
    draws = json.loads((LOGS / "tipdial_z_e29.json").read_text())["grasps"]
    ref = {e["placement"]["draw"]: e["dial"] for e in
           json.loads((LOGS / "dktipdial_O_mu1.3_point-e29z_rany.json").read_text())["envs"]}
    for d in (0, 1):
        g = draws[d]
        out = disk_run("O", 1.3, g, g["q0_deg"], {**DISK_OPTS, "settle_s": 0.0})
        r = ref[d]
        print(f"disk O draw {d}: rotation {out['rotation_deg']:.6f} vs e29 {r['rotation_deg']:.6f}; max force "
              f"{out['distal_force_max_n']:.6f} vs {r['distal_force_max_n']:.6f}; exact: "
              f"{out['rotation_deg'] == r['rotation_deg'] and out['distal_force_max_n'] == r['distal_force_max_n']}")
    return 0


# ----------------------------------------------------------------------------------------------------- analysis
def _fte(h, o):
    """Force-trajectory error of one run: time-mean |F_H - F_O| / time-mean |F_O| over the common steps."""
    n = min(len(h), len(o))
    if n == 0:
        return None
    h, o = np.asarray(h[:n], float), np.asarray(o[:n], float)
    den = float(np.mean(np.abs(o)))
    return float(np.mean(np.abs(h - o)) / den) if den > 1e-9 else None


def _load(task, hand, tag):
    p = LOGS / f"pm_{task}_{hand}_{tag}.json"
    return json.loads(p.read_text()) if p.exists() else None


def _a_pull_limits(hand):
    """Condition A (experiment 21, point): F_limit by (rho, q0, x_c, direction) and mu, None where Gate 0 failed."""
    out = {}
    for mu in MU:
        for r in (15.0, 20.0):
            p = LOGS / f"dk_{hand}_mu{mu:g}_point-e21_r{r:g}.json"
            if not p.exists():
                return None
            for e in json.loads(p.read_text())["envs"]:
                k = (e["rho_mm"], e["q0_deg"], round(float(e["x_c_mm"]), 3), e["direction"])
                ok = e["gate0"]["pass"] and e["limit"]["f_limit_n"] is not None
                out.setdefault(k, {})[mu] = e["limit"]["f_limit_n"] if ok else None
    return out


def pull_pairs(tag):
    """O's per-pair record under F: (rho, q0, x_c, direction) -> {mu: (F_limit, distal trace, proximal trace)}, and
    O's exclusions by the first reason."""
    o = _load("pull", "O", tag)
    pairs, excl = {}, {"o_solver": 0, "o_gate0": 0, "o_gate1": 0, "o_pen": 0, "o_no_limit": 0, "o_zero_d": 0}
    for rec in o["places"]:
        if "runs" not in rec:
            continue
        for direction in T.DIRECTIONS:
            k = (rec["rho_mm"], rec["q0_deg"], round(float(rec["x_c_mm"]), 3), direction)
            runs = [rec["runs"].get(f"{mu:g}") for mu in MU]
            if rec.get("solver_fail"):
                excl["o_solver"] += 1
            elif not all(x["gate0"]["pass"] for x in runs):
                excl["o_gate0"] += 1
            elif not all(x.get("gate1", {}).get("pass") for x in runs):
                excl["o_gate1"] += 1
            elif any(x["pen_pre_mm"] > PEN_MM for x in runs):
                excl["o_pen"] += 1
            elif any(x["dirs"][direction]["limit"]["f_limit_n"] is None for x in runs):
                excl["o_no_limit"] += 1
            else:
                lim = [x["dirs"][direction]["limit"]["f_limit_n"] for x in runs]
                if abs(lim[1] - lim[0]) < 1e-9:
                    excl["o_zero_d"] += 1
                    continue
                pairs[k] = {mu: _pull_series(x, direction) for mu, x in zip(MU, runs)}
    return pairs, excl


def _pull_series(run, direction):
    inc = run["inc_trace"]
    rt = run["dirs"][direction]["ramp_trace"]
    return (run["dirs"][direction]["limit"]["f_limit_n"], [x[0] for x in inc] + [x[0] for x in rt],
            [x[1] for x in inc] + [x[1] for x in rt], run["pen_pre_mm"])


def pull_hand(hand, tag, o_pairs, a_o=None):
    """E(H), FTE(H) and the validity record for one hand under F, and E under A over the same pairs."""
    h_tag = tag
    if "@" in hand:
        hand, _, h_tag = hand.partition("@")
    d = _load("pull", hand, h_tag)
    if d is None:
        return None
    by = {(r["rho_mm"], r["q0_deg"], round(float(r["x_c_mm"]), 3)): r for r in d["places"]}
    a_h = _a_pull_limits(hand) if a_o is not None else None
    rel, rel_sens, fte, fte_p, rel_a = [], [], [], [], []
    cnt = {"pairs_o": len(o_pairs), "unmatched": 0, "solver": 0, "not_held": 0, "over_pressed": 0, "no_limit": 0}
    shifts, prox_dev, mu_dev, evals, pens = {}, {}, {}, {}, []
    for k, o in o_pairs.items():
        rec = by.get(k[:3])
        if rec is None or rec.get("solver_fail"):
            cnt["solver"] += 1
            rel_sens.append(math.inf)
            continue
        m = rec["match"]
        if not m["converged"]:
            cnt["unmatched"] += 1
            rel_sens.append(math.inf)
            continue
        runs = [rec["runs"][f"{mu:g}"] for mu in MU]
        shifts[k[:3]], evals[k[:3]] = m["shift_deg"], m["evals"]
        n13, n20 = runs[0]["pre"]["normal_n"], runs[1]["pre"]["normal_n"]
        mu_dev[k[:3]] = n20[1] / max(n13[1], 1e-9) - 1
        o_rec = _o_place(tag, k[:3])
        if o_rec is not None:
            prox_dev[k[:3]] = n13[0] / max(o_rec["runs"]["1.3"]["pre"]["normal_n"][0], 1e-9) - 1
        if not all(x["gate0"]["pass"] and x.get("gate1", {}).get("pass") for x in runs):
            cnt["not_held"] += 1
            continue
        pen = max(x["pen_pre_mm"] for x in runs)
        pens.append(pen)
        if pen > PEN_MM:
            cnt["over_pressed"] += 1
            rel_sens.append(math.inf)
            continue
        hs = [_pull_series(x, k[3]) for x in runs]
        if any(x[0] is None for x in hs):
            cnt["no_limit"] += 1
            continue
        d_o, d_h = o[MU[1]][0] - o[MU[0]][0], hs[1][0] - hs[0][0]
        e = abs(d_h - d_o) / abs(d_o)
        rel.append(e)
        rel_sens.append(e)
        f_d = [_fte(hs[i][1], o[mu][1]) for i, mu in enumerate(MU)]
        f_p = [_fte(hs[i][2], o[mu][2]) for i, mu in enumerate(MU)]
        fte.append(float(np.mean(f_d)))
        fte_p.append(float(np.mean(f_p)))
        if a_h is not None:
            ao, ah = a_o.get(k, {}), a_h.get(k, {})
            if all(ao.get(mu) is not None and ah.get(mu) is not None for mu in MU):
                dao, dah = ao[MU[1]] - ao[MU[0]], ah[MU[1]] - ah[MU[0]]
                if abs(dao) > 1e-9:
                    rel_a.append(abs(dah - dao) / abs(dao))
    sh = np.array(list(shifts.values())) if shifts else np.array([np.nan])
    return {"E": float(np.median(rel)) if rel else None, "E_p75": float(np.percentile(rel, 75)) if rel else None,
            "pairs": len(rel), "E_sens": float(np.median(rel_sens)) if rel_sens else None,
            "FTE": float(np.median(fte)) if fte else None, "FTE_proximal": float(np.median(fte_p)) if fte_p else None,
            "E_A_same": float(np.median(rel_a)) if rel_a else None, "pairs_A_same": len(rel_a), **cnt,
            "shift_deg": [float(np.min(sh)), float(np.median(sh)), float(np.max(sh))],
            "evals_median": float(np.median(list(evals.values()))) if evals else None,
            "proximal_dev_median": float(np.median(np.abs(list(prox_dev.values())))) if prox_dev else None,
            "proximal_dev_range": [float(min(prox_dev.values())), float(max(prox_dev.values()))] if prox_dev else None,
            "mu2_settle_dev_max": float(np.max(np.abs(list(mu_dev.values())))) if mu_dev else None,
            "pen_max_mm": float(max(pens)) if pens else None}


@lru_cache(maxsize=None)
def _o_places(tag):
    d = _load("pull", "O", tag)
    return {(r["rho_mm"], r["q0_deg"], round(float(r["x_c_mm"]), 3)): r for r in d["places"]}


def _o_place(tag, k):
    return _o_places(tag).get(k)


def disk_series(run):
    return [x[0] for x in run["trace"]]


def disk_o(tag):
    o = _load("disk", "O", tag)
    keep, excl = {}, {"o_solver": 0, "untouched": 0, "o_pen": 0, "o_proximal": 0, "small_d": 0}
    for r in o["draws"]:
        if r.get("solver_fail"):
            excl["o_solver"] += 1
            continue
        runs = [r["runs"][f"{mu:g}"] for mu in MU]
        if not all(x["touched"] for x in runs):
            excl["untouched"] += 1
        elif any(x["distal_force_max_n"] / POINT_K > PEN_MM for x in runs):
            excl["o_pen"] += 1
        elif any(x["proximal_force_max_n"] > 1e-3 for x in runs):
            excl["o_proximal"] += 1
        elif abs(runs[1]["rotation_deg"] - runs[0]["rotation_deg"]) < 0.3:
            excl["small_d"] += 1
        else:
            keep[r["draw"]] = runs
    return keep, excl


def disk_hand(hand, tag, o_keep, a=None):
    h_tag = tag
    if "@" in hand:
        hand, _, h_tag = hand.partition("@")
    d = _load("disk", hand, h_tag)
    if d is None:
        return None
    by = {r["draw"]: r for r in d["draws"]}
    rel, rel_sens, rel_sw, fte, rel_a, pens = [], [], [], [], [], []
    cnt = {"pairs_o": len(o_keep), "unmatched": 0, "lost": 0, "over_pressed": 0, "proximal_touched": 0}
    shifts, mu_dev, evals = [], [], []
    a_o, a_h = (a if a else (None, None))
    for k, o in o_keep.items():
        rec = by.get(k)
        if rec is None or rec.get("solver_fail"):
            cnt["lost"] += 1
            rel_sens.append(math.inf)
            continue
        m = rec["match"]
        if not m["converged"]:
            cnt["unmatched"] += 1
            rel_sens.append(math.inf)
            continue
        runs = [rec["runs"][f"{mu:g}"] for mu in MU]
        shifts.append(m["shift_deg"])
        evals.append(m["evals"])
        mu_dev.append(runs[1]["settled_normal_n"] / max(runs[0]["settled_normal_n"], 1e-12) - 1)
        if not all(x["touched"] for x in runs):
            cnt["lost"] += 1
            rel_sens.append(math.inf)
            continue
        pen = max(x["distal_force_max_n"] for x in runs) / POINT_K
        pens.append(pen)
        if pen > PEN_MM:
            cnt["over_pressed"] += 1
            rel_sens.append(math.inf)
            continue
        cnt["proximal_touched"] += any(x["proximal_force_max_n"] > 1e-3 for x in runs)
        d_o = o[1]["rotation_deg"] - o[0]["rotation_deg"]
        d_h = runs[1]["rotation_deg"] - runs[0]["rotation_deg"]
        e = abs(d_h - d_o) / abs(d_o)
        rel.append(e)
        rel_sens.append(e)
        d_o_sw = o[1]["rotation_sweep_deg"] - o[0]["rotation_sweep_deg"]
        if abs(d_o_sw) >= 0.3:
            rel_sw.append(abs(runs[1]["rotation_sweep_deg"] - runs[0]["rotation_sweep_deg"] - d_o_sw) / abs(d_o_sw))
        fte.append(float(np.mean([_fte(disk_series(runs[i]), disk_series(o[i])) for i in range(2)])))
        if a_o is not None and a_h is not None:  # condition A (experiment 29 z) on the same draw, A's own rules
            ao, ah = [a_o[m_].get(("draw", k)) for m_ in MU], [a_h[m_].get(("draw", k)) for m_ in MU]
            if all(x is not None and x["touched"] for x in ao + ah) and \
                    max(x["distal_force_max_n"] for x in ah) / POINT_K <= PEN_MM:
                dao = ao[1]["rotation_deg"] - ao[0]["rotation_deg"]
                rel_a.append(abs(ah[1]["rotation_deg"] - ah[0]["rotation_deg"] - dao) / abs(dao))
    return {"E": float(np.median(rel)) if rel else None, "E_p75": float(np.percentile(rel, 75)) if rel else None,
            "pairs": len(rel), "E_sens": float(np.median(rel_sens)) if rel_sens else None,
            "E_sweep_only": float(np.median(rel_sw)) if rel_sw else None,
            "FTE": float(np.median(fte)) if fte else None,
            "E_A_same": float(np.median(rel_a)) if rel_a else None, "pairs_A_same": len(rel_a), **cnt,
            "shift_deg": [float(np.min(shifts)), float(np.median(shifts)), float(np.max(shifts))] if shifts else None,
            "evals_median": float(np.median(evals)) if evals else None,
            "mu2_settle_dev_max": float(np.max(np.abs(mu_dev))) if mu_dev else None,
            "pen_max_mm": float(max(pens)) if pens else None}


def analyze_main(a):
    from analyze import DESIGN, spearman_perm
    rows, out = [], {"task": a.task, "tag": a.tag}
    if a.task == "pull":
        o_pairs, excl = pull_pairs(a.tag)
        a_o = _a_pull_limits("O")
        out["o_pairs"], out["o_excluded"] = len(o_pairs), excl
        fn = lambda h: pull_hand(h, a.tag, o_pairs, a_o)
        a_pub = None
    else:
        import analyze_roll as AR
        AR.TASK, AR.RANDOM = "tipdial_z", True
        o_keep, excl = disk_o(a.tag)
        out["o_pairs"], out["o_excluded"] = len(o_keep), excl
        a_o = {m: AR.tip_load("O", m, "point", "e29z", "any") for m in MU}

        def fn(h):
            a_h = {m: AR.tip_load(h.partition("@")[0], m, "point", "e29z", "any") for m in MU}
            return disk_hand(h, a.tag, o_keep, (a_o, a_h) if all(a_h.values()) else None)
    print(f"{a.task} {a.tag}: O pairs {out['o_pairs']}, O excluded {out['o_excluded']}")
    for hand, (dc, ds) in DESIGN.items():
        if a.hands and hand not in a.hands:
            continue
        r = fn(hand)
        if r is None:
            continue
        rows.append({"hand": hand, "d_contact": dc, "d_skel": ds, **r})
        pct = lambda v: "   -  " if v is None else f"{100 * v:5.1f}%"
        print(f"{hand:22s} Dc {dc:4.2f} Ds {ds:5.2f} | E {pct(r['E'])} (A same {pct(r['E_A_same'])}) FTE "
              f"{pct(r['FTE'])} | pairs {r['pairs']}/{r['pairs_o']} unmatched {r['unmatched']} over "
              f"{r['over_pressed']} | shift {r['shift_deg'][0]:+.2f}/{r['shift_deg'][1]:+.2f}/{r['shift_deg'][2]:+.2f} "
              f"| pen max {r['pen_max_mm'] or 0:.2f}", flush=True)
    if a.noise_tag:
        nz = fn(f"O@{a.noise_tag}")
        out["noise_floor"] = {"tag": a.noise_tag, **nz}
        print(f"noise floor ({a.noise_tag}): E {100 * nz['E']:.2f}% (p75 {100 * nz['E_p75']:.2f}%), FTE "
              f"{100 * nz['FTE']:.2f}%, pairs {nz['pairs']}")
    sc = [r for r in rows if r["E"] is not None]
    if len(sc) >= 3:
        stats = {}
        for name, key in (("E", "E"), ("E_sens", "E_sens"), ("FTE", "FTE"), ("E_A_same", "E_A_same")):
            rr = [r for r in sc if r[key] is not None]
            for cand in ("d_contact", "d_skel"):
                rho, pv = spearman_perm([r[cand] for r in rr], [r[key] for r in rr])
                stats[f"rho_{name}_{cand}"], stats[f"p_{name}_{cand}"] = rho, pv
            stats[f"n_{name}"] = len(rr)
        out["stats"] = stats
        for name in ("E", "E_sens", "FTE", "E_A_same"):
            print(f"rho({name}, D_contact) {stats[f'rho_{name}_d_contact']:.2f} (permutation p "
                  f"{stats[f'p_{name}_d_contact']:.4f}); rho({name}, D_skel) {stats[f'rho_{name}_d_skel']:.2f} "
                  f"(p {stats[f'p_{name}_d_skel']:.4f}); hands {stats[f'n_{name}']}")
        ef = [r["E"] for r in sc if r["E_A_same"] is not None]
        ea = [r["E_A_same"] for r in sc if r["E_A_same"] is not None]
        out["overall"] = {"E_F_median_over_hands": float(np.median(ef)), "E_A_median_over_hands": float(np.median(ea)),
                          "hands_F_below_A": int(sum(f < x for f, x in zip(ef, ea))), "hands": len(ef)}
        print(f"overall: median over hands E_F {100 * np.median(ef):.1f}% vs E_A (same pairs) "
              f"{100 * np.median(ea):.1f}%; F < A in {out['overall']['hands_F_below_A']}/{len(ef)} hands")
    out["rows"] = rows
    if a.out:
        a.out.write_text(json.dumps(out, indent=1))
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    pp = sub.add_parser("pull")
    pp.add_argument("--hands", nargs="+", default=HANDS)
    pp.add_argument("--tag", default="e31")
    pp.add_argument("--workers", type=int, default=6)
    pp.add_argument("--only", type=int, default=None, help="first N placements per radius")
    pp.add_argument("--inc_deg", type=float, default=5.0, help="the shared increment (pre-registered +5 deg; +10 over-pressed O)")
    pp.add_argument("--inc_s", type=float, default=2.0)
    pp.add_argument("--hold_s", type=float, default=0.5)
    pp.add_argument("--offset_deg", type=float, default=0.0, help="O only: setpoint offset (noise floor 0.05)")
    pp.add_argument("--directions", nargs="+", default=list(T.DIRECTIONS))
    pd = sub.add_parser("disk")
    pd.add_argument("--hands", nargs="+", default=HANDS)
    pd.add_argument("--tag", default="e31")
    pd.add_argument("--workers", type=int, default=6)
    pd.add_argument("--draws", type=int, nargs="+", default=None)
    pd.add_argument("--pre_deg", type=float, default=2.0)
    pd.add_argument("--target_scale", type=float, default=1.0, help="noise floor: O matched to 1.02 x the target")
    sub.add_parser("check")
    pa = sub.add_parser("analyze")
    pa.add_argument("--task", choices=("pull", "disk"), required=True)
    pa.add_argument("--tag", default="e31")
    pa.add_argument("--noise_tag", default=None)
    pa.add_argument("--hands", nargs="+", default=None)
    pa.add_argument("--out", type=Path, default=None)
    a = p.parse_args()
    if a.cmd == "pull":
        return pull_main(a)
    if a.cmd == "disk":
        return disk_main(a)
    if a.cmd == "check":
        return check_main(a)
    return analyze_main(a)


if __name__ == "__main__":
    raise SystemExit(main())
