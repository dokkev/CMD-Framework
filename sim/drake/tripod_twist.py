"""Experiment 44: a tripod pinch twisted about the object's axis, the axis normal to the palm (experimental;
docs/experimental/drake_tripod_twist.md).

A short cylinder, its axis normal to the palm, is pinched between the thumb on one end face and the index and middle
fingertips on the other; a torque about that axis twists it. The thumb sits on the axis, so it resists the twist only
by torsional friction, which a point contact does not have: the index and middle fingers, s on either side of the
axis, carry the torque as a couple. Each sees a tangential load along its own distal pad, in its own flexion plane,
the index toward the fingertip and the middle toward the MCP (or the reverse). So one planar finger, loaded once in
each sense, gives both: tau(theta) = s * (F_+(s theta) + F_-(s theta)), and the tripod's limit is the largest tau
within THETA_MAX_DEG.

Experiment 33's pinch scene (pinch_rotate.py: the planar CoDe index, condition A's reused setpoint, the same thumb on
every hand, a soft squeeze spring so that the thumb sets the squeeze; placements made on O in each hand's proximal-face
frame). Changes: the object's face is a large sphere (radius 20-40 mm, the end face nearly flat); its spin is held
(the other finger and the thumb's face keep the cylinder from tilting); the thumb is frictionless (on the axis); the
object is driven along O's pad tangent t by a stiff spring whose rest point ramps at RATE, and the drive's force is
the finger's tangential load. Each finger carries the draw's preload (the thumb's squeeze is 2 x preload).

- Measure: tau_limit = max over theta <= THETA_MAX_DEG of s (F_+ + F_-); D = tau_limit(mu 2.0) - tau_limit(mu 1.3)
  per draw; E(H) = median |D(H) - D(O)| / |D(O)|.
- Gates as experiment 43: O's solver failures, a pad lost before the ramp, penetration > 1.5 mm and proximal-pad
  contact remove a draw; |D(O)| below 3 % of tau_limit(O, mu 1.3) removes it. A redesign's failures are scored
  infinite in E_sens.

    uv run python tripod_twist.py draws --n 64 --seed 44 --out logs/tripod_e44.json
    uv run python tripod_twist.py run --hands O --draws logs/tripod_e44.json --tag e44 [--delta_deg 0.05] [--only 8]
    uv run python tripod_twist.py analyze --tag e44 --noise_tag e44n [--hands ...] [--out logs/e44_tripod.json]
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
S_MM = 10.0  # half the index-middle spacing: each fingertip's distance from the twist axis
THETA_MAX_DEG = 10.0
TRAVEL_MM = S_MM * math.radians(THETA_MAX_DEG)  # the drive's ramp end along t
RATE = 1.0e-3  # m/s, the drive's rest point
K_DRIVE, D_DRIVE = 2.0e3, 2.0  # N/m, N s/m
LOST_RUN = 3
RANDOM = {"q_s_deg": (30.0, 55.0), "back_mm": (3.0, 8.0), "r_mm": (20.0, 40.0), "preload_n": (0.4, 0.8)}
DEFAULTS = {"approach_s": 1.0, "settle_s": 0.5, "release_s": 0.5, "k_thumb": P.K_THUMB, "thumb_mu": 0.0,
            "t_drive": (K_DRIVE, D_DRIVE)}
DIRS = (1.0, -1.0)  # + : the object's face toward O's fingertip along t
SPIN_HOLD = 10.0  # N m/rad on the object's spin (no tilt)


def load_curve(hand, mu, pl, opts, delta_deg=0.0) -> dict:
    geom = T.finger_geometry(hand)
    qs, th = T.servo_table(geom)
    q_s = math.radians(pl["q_s_deg"] + delta_deg)
    ratio = float(np.interp(q_s, qs, np.gradient(th, qs)))
    k_theta, d_theta = T.K_Q / ratio ** 2, T.D_Q / ratio ** 2
    ts = opts.get("time_step", 2.5e-4)
    n_app, n_set, n_rel = (int(round(opts[k] / T.DT_ENV)) for k in ("approach_s", "settle_s", "release_s"))
    pl0 = {**pl, "k_r": SPIN_HOLD}
    hold_opts = {k: v for k, v in opts.items() if k != "t_drive"}
    thumb_k = T.POINT_STIFFNESS * 1e-3
    pen, prox_max = 0.0, 0.0

    def watch(s):
        nonlocal pen, prox_max
        f = s.forces()
        pen = max(pen, f["distal"] / P.POINT_K, f["thumb"] / thumb_k, f["proximal"] / P.POINT_K)
        prox_max = max(prox_max, f["proximal"])
        return f

    s1 = P.Run(hand, mu, pl0, ts, k_theta, d_theta, "hold", hold_opts)
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
        jt = s.plant.GetJointByName("sph_t")
        n_act = s.plant.num_actuators()
        i_t = n_act - 2  # the t drive, then spin
        f = None
        for _ in range(n_rel):
            s.step()
            f = watch(s)
        lost_pre = f is None or f["distal"] < T.LOSS_N or f["thumb"] < T.LOSS_N
        x0 = jt.get_translation(s.pctx)
        cmd, low, lost, curve = 0.0, 0, False, []
        while not lost_pre and cmd < TRAVEL_MM * 1e-3 + 1e-12:
            cmd += RATE * T.DT_ENV
            u = np.zeros(n_act)
            u[i_t] = K_DRIVE * (x0 + sgn * cmd) + D_DRIVE * sgn * RATE  # fed forward onto the PD toward 0
            s.plant.get_actuation_input_port().FixValue(s.pctx, u)
            s.step()
            f = watch(s)
            x = jt.get_translation(s.pctx)
            force = K_DRIVE * (x0 + sgn * cmd - x)  # the spring's pull on the object = the finger's tangential load
            curve.append([round(1e3 * sgn * (x - x0), 5), round(sgn * force, 6)])
            low = low + 1 if (f["distal"] < T.LOSS_N or f["thumb"] < T.LOSS_N) else 0
            if low >= LOST_RUN:
                lost = True
                break
        out["dirs"][f"{sgn:+g}"] = {"lost_pre": lost_pre, "lost": lost, "curve": curve,
                                    "f_release_n": None if f is None else {"distal": f["distal"], "thumb": f["thumb"]}}
    out["pen_mm"], out["proximal_max_n"] = pen, prox_max
    out["tau_limit_nm"] = tau_limit(out)
    return out


def tau_limit(rec):
    """max over theta <= THETA_MAX_DEG of s (F_+(s theta) + F_-(s theta)); None if either sense was lost (before or
    during the ramp, before its curve reached the end)."""
    dd = [rec["dirs"][f"{s:+g}"] for s in DIRS]
    if any(d["lost_pre"] or not d["curve"] for d in dd):
        return None
    grid = np.linspace(0.0, TRAVEL_MM, 201)
    fs = []
    for d in dd:
        c = np.asarray(d["curve"])
        trav = np.maximum.accumulate(c[:, 0])
        reach = trav[-1]
        f = np.interp(grid, trav, c[:, 1])
        f[grid > reach] = 0.0 if d["lost"] else c[-1, 1]  # a lost finger holds nothing past its loss
        fs.append(f)
    return float(np.max(S_MM * 1e-3 * (fs[0] + fs[1])))


def run_task(task):
    hand, pl, opts, delta = task
    t0 = time.time()
    rec = {"draw": pl["draw"], "hand": hand}
    for mu in MU:
        try:
            r = load_curve(hand, mu, pl, opts, delta)
            for d in r["dirs"].values():  # thin the curves for the log
                d["curve"] = d["curve"][::10] + d["curve"][-1:]
            rec[f"{mu:g}"] = r
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
    for hand in a.hands:
        t0 = time.time()
        with Pool(a.workers) as pool:
            rr = pool.map(run_task, [(hand, pl, opts, a.delta_deg) for pl in draws], chunksize=1)
        out = LOGS / f"tripod_{hand}_{a.tag}.json"
        out.write_text(json.dumps({"hand": hand, "tag": a.tag, "opts": opts, "delta_deg": a.delta_deg, "draws": rr}))
        print(f"{hand} {a.tag}: {len(rr)} draws, cpu {sum(r['wall_s'] for r in rr):.0f} s, wall "
              f"{time.time() - t0:.0f} s -> {out.name}", flush=True)
    return 0


def _load(hand, tag):
    p = LOGS / f"tripod_{hand}_{tag}.json"
    return {r["draw"]: r for r in json.loads(p.read_text())["draws"]} if p.exists() else None


def _bad(x):
    return "solver_fail" in x


def o_keep(tag):
    o = _load("O", tag)
    keep, excl = {}, {"o_solver": 0, "o_lost": 0, "o_pen": 0, "o_proximal": 0, "small_d": 0}
    for k, r in sorted(o.items()):
        runs = [r[f"{mu:g}"] for mu in MU]
        if any(_bad(x) for x in runs):
            excl["o_solver"] += 1
        elif any(x["pen_mm"] > P.PEN_MM for x in runs):
            excl["o_pen"] += 1
        elif any(x["proximal_max_n"] > P.PROX_TOUCH_N for x in runs):
            excl["o_proximal"] += 1
        elif any(x["tau_limit_nm"] is None for x in runs):
            excl["o_lost"] += 1
        elif abs(runs[1]["tau_limit_nm"] - runs[0]["tau_limit_nm"]) < 0.03 * runs[0]["tau_limit_nm"]:
            excl["small_d"] += 1
        else:
            keep[k] = [x["tau_limit_nm"] for x in runs]
    return keep, excl


def hand_error(hand, tag, keep):
    h = _load(hand, tag)
    if h is None:
        return None
    rel, sens, pens = [], [], []
    cnt = {"solver": 0, "lost": 0, "over_pressed": 0, "proximal": 0}
    for k, o in keep.items():
        runs = [h[k][f"{mu:g}"] for mu in MU]
        if any(_bad(x) for x in runs):
            cnt["solver"] += 1
            sens.append(math.inf)
            continue
        pen = max(x["pen_mm"] for x in runs)
        pens.append(pen)
        if any(x["tau_limit_nm"] is None for x in runs):
            cnt["lost"] += 1
            sens.append(math.inf)
            continue
        if pen > P.PEN_MM:
            cnt["over_pressed"] += 1
            sens.append(math.inf)
            continue
        cnt["proximal"] += any(x["proximal_max_n"] > P.PROX_TOUCH_N for x in runs)
        d_o = o[1] - o[0]
        d_h = runs[1]["tau_limit_nm"] - runs[0]["tau_limit_nm"]
        e = abs(d_h - d_o) / abs(d_o)
        rel.append(e)
        sens.append(e)
    return {"E": float(np.median(rel)) if rel else None, "E_p75": float(np.percentile(rel, 75)) if rel else None,
            "pairs": len(rel), "E_sens": float(np.median(sens)) if sens else None, **cnt,
            "pen_max_mm": float(max(pens)) if pens else None}


EXTRA = {"conv_e25": (6.61, 0.34)}  # outside analyze.DESIGN: scored, not in the correlation


def cmd_analyze(a) -> int:
    from analyze import DESIGN, spearman_perm
    keep, excl = o_keep(a.tag)
    d_o = np.array([o[1] - o[0] for o in keep.values()])
    t_o = np.array([o[0] for o in keep.values()])
    print(f"{a.tag}: O kept {len(keep)} draws, excluded {excl}; D(O) {1e3 * d_o.min():.2f}/{1e3 * np.median(d_o):.2f}/"
          f"{1e3 * d_o.max():.2f} mN m (negative {int((d_o < 0).sum())}); tau(O, mu 1.3) {1e3 * t_o.min():.1f}/"
          f"{1e3 * np.median(t_o):.1f}/{1e3 * t_o.max():.1f} mN m")
    out = {"tag": a.tag, "o_kept": len(keep), "o_excluded": excl, "d_o_nm": d_o.tolist(), "rows": []}
    if a.noise_tag and _load("O", a.noise_tag):
        nz = hand_error("O", a.noise_tag, keep)
        out["noise_floor"] = nz
        print(f"noise floor ({a.noise_tag}): E {100 * nz['E']:.2f}% (p75 {100 * nz['E_p75']:.2f}%), pairs {nz['pairs']}")
    hands = {**DESIGN, **EXTRA}
    hands.update({hh: (float("nan"), float("nan")) for hh in (a.hands or []) if hh not in hands})
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
              f"over {r['over_pressed']} prox {r['proximal']} | pen max {r['pen_max_mm'] or 0:.2f}", flush=True)
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
    pd_.add_argument("--seed", type=int, default=44)
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
