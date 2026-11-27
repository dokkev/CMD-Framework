"""Experiment 45: a tripod pinch translating the object toward the palm (experimental;
docs/experimental/drake_pinch_translate.md).

The thumb holds one face of the object, the index and middle fingertips the other; the two fingers flex together and
drag the object along the thumb toward the palm. By symmetry one planar finger models it (each finger carries half
the object's load). Experiment 44's scene (tripod_twist.py / pinch_rotate.py): the thumb on experiment 33's soft
squeeze spring, frictionless (the object slides on it); the finger closes to O's q_s; the object's spin is held. The object is
tethered along O's pad tangent t by a spring K_T (without it the friction would not matter and D ~ 0, as the
untethered coin), so the pad drags it until the spring's pull passes mu N and the pad slips.

- Command (O's open loop on every hand, condition A): close 0 -> q_s + PRESS_DEG over approach_s, settle; release
  the object (free along t on the spring and along n); then flex by sweep_deg over sweep_s and hold hold_s.
- Measure: the object's travel along t from the sweep's start to the end (mm, + toward the palm... the sense the
  flexing pad drags it); D = travel(mu 2.0) - travel(mu 1.3); E(H) = median |D(H) - D(O)| / |D(O)|.
- Gates as 44b: solver, a pad lost (LOST_RUN samples below LOSS_N), penetration > 1.5 mm, proximal-pad contact;
  |D(O)| below 3 % of travel(O, mu 1.3) removes a draw.

    uv run python pinch_translate.py draws --n 64 --seed 45 --out logs/ptrans_e45.json
    uv run python pinch_translate.py run --hands O --draws logs/ptrans_e45.json --tag e45 [--delta_deg 0.05] [--only 8]
    uv run python pinch_translate.py analyze --tag e45 --noise_tag e45n [--hands ...] [--out logs/e45_ptrans.json]
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
import tripod_twist as TT
import two_pad_drake as T

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
MU = (1.3, 2.0)
K_T, D_T = 600.0, 2.0  # N/m, N s/m: the tether along t (O pilot, 4 draws: 200 never slips, 1000 always does)
LOST_RUN = 3
RANDOM = {"q_s_deg": (30.0, 55.0), "back_mm": (3.0, 8.0), "r_mm": (20.0, 40.0), "preload_n": (0.4, 0.8),
          "sweep_deg": (18.0, 28.0)}
DEFAULTS = {"approach_s": 1.0, "settle_s": 0.5, "release_s": 0.5, "sweep_s": 2.0, "hold_s": 0.5,
            "k_thumb": P.K_THUMB, "thumb_mu": 0.0, "press_deg": 0.0, "t_drive": (K_T, D_T)}
# the thumb on experiment 33's soft squeeze spring: a stiff thumb (44b's) blocks the object along n, the flexing pad
# then presses it (10 N, 4.5 mm penetration in a 4-draw O check) instead of dragging it, and nothing slips


def translate(hand, mu, pl, opts, delta_deg=0.0) -> dict:
    geom = T.finger_geometry(hand)
    qs, th = T.servo_table(geom)
    q_s = math.radians(pl["q_s_deg"] + delta_deg)
    q0 = q_s + math.radians(opts["press_deg"])
    q1 = q0 + math.radians(pl["sweep_deg"])
    ratio = float(np.interp(q0, qs, np.gradient(th, qs)))
    k_theta, d_theta = T.K_Q / ratio ** 2, T.D_Q / ratio ** 2
    ts = opts.get("time_step", 2.5e-4)
    n_app, n_set, n_rel, n_sw, n_hold = (int(round(opts[k] / T.DT_ENV)) for k in
                                         ("approach_s", "settle_s", "release_s", "sweep_s", "hold_s"))
    pl0 = {**pl, "k_r": TT.SPIN_HOLD}
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
    for i in range(n_app + n_set):
        s1.theta_cmd = float(np.interp(q0 * min(1.0, (i + 1) / n_app), qs, th))
        s1.step()
        watch(s1)
    held = s1.plant.GetPositionsAndVelocities(s1.pctx).copy()
    s = P.Run(hand, mu, pl0, ts, k_theta, d_theta, "free", opts)
    s.plant.SetPositionsAndVelocities(s.pctx, held)
    s.theta_cmd = float(np.interp(q0, qs, th))
    s.sim.Initialize()
    jt = s.plant.GetJointByName("sph_t")
    f = None
    for _ in range(n_rel):
        s.step()
        f = watch(s)
    lost_pre = f is None or f["distal"] < T.LOSS_N or f["thumb"] < T.LOSS_N
    x0 = jt.get_translation(s.pctx)
    low, lost, trace = 0, False, []
    if not lost_pre:
        for i in range(n_sw + n_hold):
            q = q0 + (q1 - q0) * min(1.0, (i + 1) / n_sw)
            s.theta_cmd = float(np.interp(q, qs, th))
            s.step()
            f = watch(s)
            low = low + 1 if (f["distal"] < T.LOSS_N or f["thumb"] < T.LOSS_N) else 0
            if i % 20 == 0:
                trace.append([round(math.degrees(q), 3), round(1e3 * (jt.get_translation(s.pctx) - x0), 5)])
            if low >= LOST_RUN:
                lost = True
                break
    travel = 1e3 * (jt.get_translation(s.pctx) - x0)
    return {"travel_mm": None if (lost_pre or lost) else float(travel), "lost_pre": lost_pre, "lost": lost,
            "pen_mm": pen, "proximal_max_n": prox_max, "trace": trace,
            "f_end_n": None if f is None else {"distal": f["distal"], "thumb": f["thumb"]}}


def run_task(task):
    hand, pl, opts, delta = task
    t0 = time.time()
    rec = {"draw": pl["draw"], "hand": hand}
    for mu in MU:
        try:
            rec[f"{mu:g}"] = translate(hand, mu, pl, opts, delta)
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
        out = LOGS / f"ptrans_{hand}_{a.tag}.json"
        out.write_text(json.dumps({"hand": hand, "tag": a.tag, "opts": opts, "delta_deg": a.delta_deg, "draws": rr}))
        print(f"{hand} {a.tag}: {len(rr)} draws, cpu {sum(r['wall_s'] for r in rr):.0f} s, wall "
              f"{time.time() - t0:.0f} s -> {out.name}", flush=True)
    return 0


def _load(hand, tag):
    p = LOGS / f"ptrans_{hand}_{tag}.json"
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
        elif any(x["travel_mm"] is None for x in runs):
            excl["o_lost"] += 1
        elif abs(runs[1]["travel_mm"] - runs[0]["travel_mm"]) < 0.03 * abs(runs[0]["travel_mm"]):
            excl["small_d"] += 1
        else:
            keep[k] = [x["travel_mm"] for x in runs]
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
        if any(x["travel_mm"] is None for x in runs):
            cnt["lost"] += 1
            sens.append(math.inf)
            continue
        if pen > P.PEN_MM:
            cnt["over_pressed"] += 1
            sens.append(math.inf)
            continue
        cnt["proximal"] += any(x["proximal_max_n"] > P.PROX_TOUCH_N for x in runs)
        d_o = o[1] - o[0]
        d_h = runs[1]["travel_mm"] - runs[0]["travel_mm"]
        e = abs(d_h - d_o) / abs(d_o)
        rel.append(e)
        sens.append(e)
    return {"E": float(np.median(rel)) if rel else None, "E_p75": float(np.percentile(rel, 75)) if rel else None,
            "pairs": len(rel), "E_sens": float(np.median(sens)) if sens else None, **cnt,
            "pen_max_mm": float(max(pens)) if pens else None}


def cmd_analyze(a) -> int:
    from analyze import DESIGN, spearman_perm
    keep, excl = o_keep(a.tag)
    d_o = np.array([o[1] - o[0] for o in keep.values()])
    t_o = np.array([o[0] for o in keep.values()])
    print(f"{a.tag}: O kept {len(keep)} draws, excluded {excl}; D(O) {d_o.min():.3f}/{np.median(d_o):.3f}/"
          f"{d_o.max():.3f} mm (negative {int((d_o < 0).sum())}); travel(O, mu 1.3) {t_o.min():.2f}/"
          f"{np.median(t_o):.2f}/{t_o.max():.2f} mm")
    out = {"tag": a.tag, "o_kept": len(keep), "o_excluded": excl, "d_o_mm": d_o.tolist(), "rows": []}
    if a.noise_tag and _load("O", a.noise_tag):
        nz = hand_error("O", a.noise_tag, keep)
        out["noise_floor"] = nz
        print(f"noise floor ({a.noise_tag}): E {100 * nz['E']:.2f}% (p75 {100 * nz['E_p75']:.2f}%), pairs {nz['pairs']}")
    hands = {**DESIGN, **TT.EXTRA}
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
    pd_.add_argument("--seed", type=int, default=45)
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
