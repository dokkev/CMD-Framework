"""Experiment 43: a thumb-index pinch twisted about the pinch's normal axis (experimental; docs/experimental/drake_pinch_twist.md).

Experiment 33's pinch (pinch_rotate.py, imported read-only: the planar CoDe index, condition A's reused setpoint, the
same thumb on every hand, a soft squeeze spring so that the thumb, not the finger, sets the squeeze; the placements made
on O in each hand's proximal-face frame). Instead of sweeping the finger, the finger holds O's start flexion q_s and a
torque about the sphere's spin axis (y, normal to the finger's plane, i.e. the torsional load of a pinch) rises at
RATE until the pinch gives way: the sphere turns more than SLIP_DEG, either pad loses contact (LOST_RUN samples below
LOSS_N), or the sphere moves SHIFT_MM. tau_limit is the torque at that event; both directions (+: the distal pad's
side of the sphere toward the fingertip) are separate pairs.

- Measure: D = tau_limit(mu 2.0) - tau_limit(mu 1.3) per (draw, direction); E(H) = median |D(H) - D(O)| / |D(O)|.
- Gates as experiment 33: O's solver failures, lost before the ramp, penetration > 1.5 mm and proximal-pad contact
  remove a pair; |D(O)| below three torque steps removes it. A redesign's failures are counted and scored infinite in
  E_sens.

    uv run python pinch_twist.py draws --n 64 --seed 43 --out logs/ptwist_e43.json
    uv run python pinch_twist.py run --hands O --draws logs/ptwist_e43.json --tag e43 [--delta_deg 0.05] [--only 8]
    uv run python pinch_twist.py analyze --tag e43 --noise_tag e43n [--hands O R U conv_e25] [--out logs/e43_ptwist.json]
"""
from __future__ import annotations

import argparse
import json
import math
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import pinch_rotate as P
import two_pad_drake as T

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
MU = (1.3, 2.0)
RATE = 0.01  # N m/s
TAU_CAP = 0.08  # N m: no event below it -> no limit
SLIP_DEG, SHIFT_MM, LOST_RUN = 5.0, 5.0, 3
RANDOM = {"q_s_deg": (30.0, 55.0), "back_mm": (3.0, 8.0), "r_mm": (8.0, 12.0), "preload_n": (0.4, 0.8)}
DEFAULTS = {"approach_s": 1.0, "settle_s": 0.5, "release_s": 0.5, "k_thumb": P.K_THUMB}
DIRS = (1.0, -1.0)


def twist(hand, mu, pl, opts, delta_deg=0.0, record=False) -> dict:
    geom = T.finger_geometry(hand)
    qs, th = T.servo_table(geom)
    q_s = math.radians(pl["q_s_deg"] + delta_deg)
    ratio = float(np.interp(q_s, qs, np.gradient(th, qs)))
    k_theta, d_theta = T.K_Q / ratio ** 2, T.D_Q / ratio ** 2
    ts = opts.get("time_step", 2.5e-4)
    n_app, n_set, n_rel = (int(round(opts[k] / T.DT_ENV)) for k in ("approach_s", "settle_s", "release_s"))
    pl0 = {**pl, "k_r": 0.0}  # no spin spring: the pads alone hold the torque
    thumb_k = T.POINT_STIFFNESS * 1e-3
    pen, prox_max = 0.0, 0.0

    def watch(s):
        nonlocal pen, prox_max
        f = s.forces()
        pen = max(pen, f["distal"] / P.POINT_K, f["thumb"] / thumb_k, f["proximal"] / P.POINT_K)
        prox_max = max(prox_max, f["proximal"])
        return f

    s1 = P.Run(hand, mu, pl0, ts, k_theta, d_theta, "hold", opts)
    s1.sim.Initialize()
    theta_s = float(np.interp(q_s, qs, th))
    for i in range(n_app + n_set):
        q = q_s * min(1.0, (i + 1) / n_app)
        s1.theta_cmd = float(np.interp(q, qs, th))
        s1.step()
        watch(s1)
    held = s1.plant.GetPositionsAndVelocities(s1.pctx).copy()
    out = {"dirs": {}}
    for sgn in DIRS:
        s = P.Run(hand, mu, pl0, ts, k_theta, d_theta, "free", opts)
        s.plant.SetPositionsAndVelocities(s.pctx, held)
        s.theta_cmd = theta_s
        s.sim.Initialize()
        f = None
        for _ in range(n_rel):
            s.step()
            f = watch(s)
        lost_pre = f is None or f["distal"] < T.LOSS_N or f["thumb"] < T.LOSS_N
        spin0, c0 = s.spin(), s.pose("sphere")
        n_act = s.plant.num_actuators()
        tau, low, event, trace = 0.0, 0, None, []
        while tau < TAU_CAP and not lost_pre:
            tau += RATE * T.DT_ENV
            u = np.zeros(n_act)
            u[n_act - 1] = sgn * tau  # the spin actuator is the last in the free stage
            s.plant.get_actuation_input_port().FixValue(s.pctx, u)
            s.step()
            f = watch(s)
            turn = abs(math.degrees(s.spin() - spin0))
            shift = 1e3 * float(np.linalg.norm(s.pose("sphere")[:2] - c0[:2]))
            low = low + 1 if (f["distal"] < T.LOSS_N or f["thumb"] < T.LOSS_N) else 0
            if record:
                trace.append([round(tau, 5), round(turn, 4), round(f["distal"], 4), round(f["thumb"], 4)])
            if turn > SLIP_DEG:
                event = "slip"
            elif low >= LOST_RUN:
                event = "loss"
            elif shift > SHIFT_MM:
                event = "collapse"
            if event:
                break
        rec = {"tau_limit_nm": tau if event else None, "event": event, "lost_pre": lost_pre,
               "f_release_n": None if f is None else {"distal": f["distal"], "thumb": f["thumb"]}}
        if record:
            rec["trace"] = trace
        out["dirs"][f"{sgn:+g}"] = rec
    out["pen_mm"], out["proximal_max_n"] = pen, prox_max
    return out


def run_task(task):
    hand, pl, opts, delta = task
    t0 = time.time()
    rec = {"draw": pl["draw"], "hand": hand}
    for mu in MU:
        try:
            rec[f"{mu:g}"] = twist(hand, mu, pl, opts, delta)
        except RuntimeError as err:
            rec[f"{mu:g}"] = {"solver_fail": str(err)[:200]}
    rec["wall_s"] = time.time() - t0
    return rec


def cmd_draws(a) -> int:
    draws = P.make_draws(a.n, a.seed, RANDOM)
    a.out.write_text(json.dumps({"seed": a.seed, "ranges": RANDOM, "draws": draws}, indent=1))
    print(f"{len(draws)} draws -> {a.out}")
    return 0


def cmd_run(a) -> int:
    d = json.loads(a.draws.read_text())
    draws = d["draws"][: a.only] if a.only else d["draws"]
    opts = {**DEFAULTS, "time_step": a.time_step}
    tasks = [(hand, pl, opts, a.delta_deg) for hand in a.hands for pl in draws]
    t0 = time.time()
    with Pool(a.workers) as pool:
        recs = pool.map(run_task, tasks, chunksize=1)
    for hand in a.hands:
        rr = [r for r in recs if r["hand"] == hand]
        out = LOGS / f"ptwist_{hand}_{a.tag}.json"
        out.write_text(json.dumps({"hand": hand, "tag": a.tag, "opts": opts, "delta_deg": a.delta_deg, "draws": rr}))
        print(f"{hand} {a.tag}: {len(rr)} draws, cpu {sum(r['wall_s'] for r in rr):.0f} s -> {out.name}", flush=True)
    print(f"done in {time.time() - t0:.0f} s (workers {a.workers})", flush=True)
    return 0


def _load(hand, tag):
    p = LOGS / f"ptwist_{hand}_{tag}.json"
    return {r["draw"]: r for r in json.loads(p.read_text())["draws"]} if p.exists() else None


def _bad(x):
    return "solver_fail" in x


def o_keep(tag):
    o = _load("O", tag)
    keep, excl = {}, {"o_solver": 0, "o_lost_pre": 0, "o_pen": 0, "o_proximal": 0, "no_limit": 0, "small_d": 0}
    for k, r in sorted(o.items()):
        runs = [r[f"{mu:g}"] for mu in MU]
        if any(_bad(x) for x in runs):
            excl["o_solver"] += 2
            continue
        if any(x["pen_mm"] > P.PEN_MM for x in runs):
            excl["o_pen"] += 2
            continue
        if any(x["proximal_max_n"] > P.PROX_TOUCH_N for x in runs):
            excl["o_proximal"] += 2
            continue
        for dname in (f"{s:+g}" for s in DIRS):
            dd = [x["dirs"][dname] for x in runs]
            if any(y["lost_pre"] for y in dd):
                excl["o_lost_pre"] += 1
            elif any(y["tau_limit_nm"] is None for y in dd):
                excl["no_limit"] += 1
            elif abs(dd[1]["tau_limit_nm"] - dd[0]["tau_limit_nm"]) < 3 * RATE * T.DT_ENV:
                excl["small_d"] += 1
            else:
                keep[(k, dname)] = dd
    return keep, excl


def hand_error(hand, tag, keep):
    h = _load(hand, tag)
    if h is None:
        return None
    rel, sens, pens = [], [], []
    cnt = {"solver": 0, "lost": 0, "over_pressed": 0, "no_limit": 0, "proximal": 0}
    for (k, dname), o in keep.items():
        runs = [h[k][f"{mu:g}"] for mu in MU]
        if any(_bad(x) for x in runs):
            cnt["solver"] += 1
            sens.append(math.inf)
            continue
        pen = max(x["pen_mm"] for x in runs)
        pens.append(pen)
        dd = [x["dirs"][dname] for x in runs]
        if any(y["lost_pre"] for y in dd):
            cnt["lost"] += 1
            sens.append(math.inf)
            continue
        if pen > P.PEN_MM:
            cnt["over_pressed"] += 1
            sens.append(math.inf)
            continue
        if any(y["tau_limit_nm"] is None for y in dd):
            cnt["no_limit"] += 1
            sens.append(math.inf)
            continue
        cnt["proximal"] += any(x["proximal_max_n"] > P.PROX_TOUCH_N for x in runs)
        d_o = o[1]["tau_limit_nm"] - o[0]["tau_limit_nm"]
        d_h = dd[1]["tau_limit_nm"] - dd[0]["tau_limit_nm"]
        e = abs(d_h - d_o) / abs(d_o)
        rel.append(e)
        sens.append(e)
    return {"E": float(np.median(rel)) if rel else None, "E_p75": float(np.percentile(rel, 75)) if rel else None,
            "pairs": len(rel), "E_sens": float(np.median(sens)) if sens else None, **cnt,
            "pen_max_mm": float(max(pens)) if pens else None}


def cmd_analyze(a) -> int:
    from analyze import DESIGN, spearman_perm
    keep, excl = o_keep(a.tag)
    d_o = np.array([o[1]["tau_limit_nm"] - o[0]["tau_limit_nm"] for o in keep.values()])
    t_o = np.array([o[0]["tau_limit_nm"] for o in keep.values()])
    print(f"{a.tag}: O kept {len(keep)} pairs, excluded {excl}; D(O) {1e3 * d_o.min():.2f}/{1e3 * np.median(d_o):.2f}/"
          f"{1e3 * d_o.max():.2f} mN m (negative {int((d_o < 0).sum())}); tau(O, mu 1.3) {1e3 * t_o.min():.1f}/"
          f"{1e3 * np.median(t_o):.1f}/{1e3 * t_o.max():.1f} mN m")
    out = {"tag": a.tag, "o_kept": len(keep), "o_excluded": excl, "d_o_nm": d_o.tolist(), "rows": []}
    if a.noise_tag and _load("O", a.noise_tag):
        nz = hand_error("O", a.noise_tag, keep)
        out["noise_floor"] = nz
        print(f"noise floor ({a.noise_tag}): E {100 * nz['E']:.2f}% (p75 {100 * nz['E_p75']:.2f}%), pairs {nz['pairs']}")
    hands = dict(DESIGN)
    extra = {"conv_e25": (6.61, 0.34)}
    for hh in (a.hands or []):
        if hh in extra:
            hands[hh] = extra[hh]
    rows = []
    for hand, (dc, ds) in hands.items():
        if a.hands and hand not in a.hands:
            continue
        r = hand_error(hand, a.tag, keep)
        if r is None:
            continue
        rows.append({"hand": hand, "d_contact": dc, "d_skel": ds, **r})
        e = "-" if r["E"] is None else f"{100 * r['E']:5.1f}%"
        print(f"{hand:22s} Dc {dc:4.2f} Ds {ds:5.2f} | E {e} | pairs {r['pairs']}/{len(keep)} lost {r['lost']} "
              f"over {r['over_pressed']} no_limit {r['no_limit']} prox {r['proximal']} | pen max {r['pen_max_mm'] or 0:.2f}",
              flush=True)
    out["rows"] = rows
    sc = [r for r in rows if r["E"] is not None and r["hand"] in DESIGN]
    if len(sc) >= 3:
        st = {}
        for cand in ("d_contact", "d_skel"):
            rho, p = spearman_perm([r[cand] for r in sc], [r["E"] for r in sc])
            st[f"rho_{cand}"], st[f"p_{cand}"] = rho, p
        print(f"rho(E, D_contact) {st['rho_d_contact']:.2f} (p {st['p_d_contact']:.4f}); rho(E, D_skel) "
              f"{st['rho_d_skel']:.2f} (p {st['p_d_skel']:.4f}); {len(sc)} hands of analyze.DESIGN")
        out["stats"] = st
    if a.out:
        a.out.write_text(json.dumps(out, indent=1, default=float))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    pd_ = sub.add_parser("draws")
    pd_.add_argument("--n", type=int, default=64)
    pd_.add_argument("--seed", type=int, default=43)
    pd_.add_argument("--out", type=Path, required=True)
    pr = sub.add_parser("run")
    pr.add_argument("--hands", nargs="+", default=["O"])
    pr.add_argument("--draws", type=Path, required=True)
    pr.add_argument("--tag", required=True)
    pr.add_argument("--delta_deg", type=float, default=0.0)
    pr.add_argument("--only", type=int, default=None)
    pr.add_argument("--time_step", type=float, default=2.5e-4)
    pr.add_argument("--workers", type=int, default=4)
    pa = sub.add_parser("analyze")
    pa.add_argument("--tag", required=True)
    pa.add_argument("--noise_tag", default=None)
    pa.add_argument("--hands", nargs="+", default=None)
    pa.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    return {"draws": cmd_draws, "run": cmd_run, "analyze": cmd_analyze}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main())
