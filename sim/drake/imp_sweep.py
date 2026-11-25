"""Experiment 37: the PIP flexion impedance K_Q swept over the Drake tasks (docs/experimental/drake_impedance.md).

Every task keeps its own simulation and analysis code; only two_pad_drake's K_Q (and D_Q = 0.08 sqrt(K_Q / 5), so the
damping ratio stays about the same) is set in each worker before the task runs. Logs are written in each task's own
format under the tag e37k<K>, so its own analysis reads them:
  pull  (21): condition A pull-out, point contact, the 32 placements at rho 15 and 20 mm, four directions; the ramp
        branched from the settled state (identical to the per-direction runs) with the load-displacement curve;
        dk_<hand>_mu<mu>_point-e37k<K>_r<rho>.json (analyze.error)
  disk  (29z): the fingertip disk's 64 random draws; dktipdial_<hand>_mu<mu>_point-e37k<K>_rany.json (analyze_roll)
  coin  (30b): the long coin drag's 64 draws; dkcoin_<hand>_mu<mu>_point-e37k<K>.json (coin_drag)
  pivot (35): the block pivot's 64 draws; pivot_<hand>_e37k<K>.json (box_pivot)
  shape (34): the pad shape's 80 placements x 2 shapes; shape_<hand>_e37k<K>.json (pad_shape)

    uv run python imp_sweep.py run --k 1 2.5 5 10 20 --tasks pull disk coin pivot shape --hands O R U
    uv run python imp_sweep.py check --k 5        # K = 5 reproduces the primary runs exactly (O, R, U)
    uv run python imp_sweep.py analyze --k 1 2.5 5 10 20 --out logs/e37_stage1.json
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
TASKS = ("pull", "disk", "coin", "pivot", "shape")
K0, D0 = 5.0, 0.08


def tag(k):
    return f"e37k{k:g}"


def set_k(k):
    T.K_Q, T.D_Q = float(k), D0 * math.sqrt(float(k) / K0)


# ----------------------------------------------------------------------------------------------------- one job
def job(args):
    """(k, task, hand, item) -> the task's own record for one placement or draw (both mu, or both shapes)."""
    k, task, hand, item = args
    set_k(k)
    t0 = time.time()
    if task == "pull":
        import pad_shape as PS
        import preload_match as P
        r, g = item
        out = {}
        for mu in MU:
            s, m_, gate, pre, c = P.pull_settle(hand, mu, r, g, g["q0_deg"] + 5.0)
            envs = []
            state = s.plant.GetPositionsAndVelocities(s.pctx).copy()
            theta = s.ctrl.theta_cmd
            for d in T.DIRECTIONS:
                e = {"rho_mm": r, "q0_deg": g["q0_deg"], "x_c_mm": g["x_c_mm"], "direction": d, "pre": pre,
                     "gate0": gate, "limit": {"event": None, "f_limit_n": None}}
                if gate["pass"]:
                    b = T.Stage(hand, mu, r, "point", 1.0e-3, False, c["k_theta"], c["d_theta"])
                    b.plant.SetPositionsAndVelocities(b.pctx, state)
                    b.ctrl.theta_cmd = theta
                    b.sim.Initialize()
                    lim, curve = PS.ramp(b, b.measure(), d)
                    e["limit"], e["curve"] = lim, curve
                envs.append(e)
            out[f"{mu:g}"] = envs
    elif task == "disk":
        out = {}
        opts = json.loads((LOGS / "dktipdial_O_mu1.3_point-e29z_rany.json").read_text())["opts"]
        for mu in MU:
            out[f"{mu:g}"] = T.run_env((hand, mu, item["rho_mm"], "point", 1.0e-3, item, "none", 0.0, T.MODULUS_PA,
                                        opts))
    elif task == "coin":
        import coin_drag as CD
        hdr = json.loads((LOGS / "dkcoin_O_mu1.3_point-e30b.json").read_text())
        out = {f"{mu:g}": CD.run_env((hand, mu, item, hdr["time_step"], 0.0, hdr["opts"])) for mu in MU}
    elif task == "pivot":
        import box_pivot as BP
        d = json.loads((LOGS / "pivot_e35.json").read_text())
        out = BP.run_task((hand, item, BP._opts(d), 0.0))
    elif task == "shape":
        import pad_shape as PS
        out = {sh: PS.run_task((hand, sh, item, 0.0)) for sh in PS.SHAPES}
    return {"k": k, "task": task, "hand": hand, "item": item if task != "pull" else
            {"rho_mm": item[0], "q0_deg": item[1]["q0_deg"]}, "out": out, "wall_s": time.time() - t0}


def items(task):
    if task == "pull":
        grasps = json.loads(T.GRASPS.read_text())["grasps"]
        return [(g["rho_mm"], g) for g in grasps if g["rho_mm"] in (15.0, 20.0)]
    if task == "disk":
        return json.loads((LOGS / "tipdial_z_e29.json").read_text())["grasps"]
    if task == "coin":
        return json.loads((LOGS / "coin_e30b.json").read_text())["grasps"]
    if task == "pivot":
        return json.loads((LOGS / "pivot_e35.json").read_text())["draws"]
    if task == "shape":
        import pad_shape as PS
        return json.loads(PS.PLACES.read_text())["places"]


def write(k, task, hand, recs):
    tg = tag(k)
    if task == "pull":
        for mu in MU:
            for r in (15.0, 20.0):
                envs = [e for rec in recs if rec["item"]["rho_mm"] == r for e in rec["out"][f"{mu:g}"]]
                (LOGS / f"dk_{hand}_mu{mu:g}_point-{tg}_r{r:g}.json").write_text(json.dumps(
                    {"hand": hand, "mu": mu, "contact": "point", "time_step": 1e-3, "delta_deg": 5.0, "k_q": k,
                     "envs": envs}))
    elif task == "disk":
        for mu in MU:
            (LOGS / f"dktipdial_{hand}_mu{mu:g}_point-{tg}_rany.json").write_text(json.dumps(
                {"hand": hand, "mu": mu, "contact": "point", "k_q": k, "envs": [rec["out"][f"{mu:g}"] for rec in recs]}))
    elif task == "coin":
        import coin_drag as CD
        for mu in MU:
            CD.out_path(hand, mu, tg).write_text(json.dumps(
                {"hand": hand, "mu": mu, "tag": tg, "k_q": k, "envs": [rec["out"][f"{mu:g}"] for rec in recs]}))
    elif task == "pivot":
        (LOGS / f"pivot_{hand}_{tg}.json").write_text(json.dumps(
            {"hand": hand, "tag": tg, "k_q": k, "draws": [rec["out"] for rec in recs]}))
    elif task == "shape":
        (LOGS / f"shape_{hand}_{tg}.json").write_text(json.dumps(
            {"hand": hand, "tag": tg, "mu": 1.3, "k_q": k, "runs": [x for rec in recs for x in rec["out"].values()]}))


def cmd_run(a) -> int:
    t_all = time.time()
    for task in a.tasks:
        its = items(task)
        if a.only:
            its = its[: a.only]
        jobs = [(k, task, h, it) for k in a.k for h in a.hands for it in its]
        t0 = time.time()
        with Pool(a.workers) as pool:
            recs = pool.map(job, jobs, chunksize=1)
        for k in a.k:
            for h in a.hands:
                write(k, task, h, [r for r in recs if r["k"] == k and r["hand"] == h])
        cpu = sum(r["wall_s"] for r in recs)
        print(f"{task}: K {a.k}, hands {a.hands}: {len(jobs)} jobs, cpu {cpu:.0f} s, wall {time.time() - t0:.0f} s",
              flush=True)
    print(f"done in {time.time() - t_all:.0f} s (workers {a.workers})", flush=True)
    return 0


# ----------------------------------------------------------------------------------------------------- analysis
FLOOR = {"pull": 1e-9, "disk": 0.3, "coin": 0.3, "pivot": 0.3, "shape": 0.6}  # each task's |D(O)| floor
UNIT = {"pull": "N", "disk": "deg", "coin": "mm", "pivot": "deg", "shape": "N"}


def task_result(task, tg, hands):
    """O's kept pairs and |D(O)|, and E (with counts) for the hands, through each task's own analysis."""
    if task == "pull":
        import analyze as A
        loader = lambda h, m, r: A.load_drake(h, m, "point", tg, r)
        out = {"hands": {}}
        # O's candidates and keeps (analyze.error's pair rule: both hands pass Gate 0 at both mu, limits exist)
        o = {m: {A.key(e): e for r in A.RADII for e in (loader("O", m, r) or [])} for m in MU}
        cand = [k_ for k_ in o[MU[0]]]
        kept = [k_ for k_ in cand if all(o[m].get(k_) and o[m][k_]["gate0"]["pass"] and
                                          o[m][k_]["limit"]["f_limit_n"] is not None for m in MU)]
        d_o = [o[2.0][k_]["limit"]["f_limit_n"] - o[1.3][k_]["limit"]["f_limit_n"] for k_ in kept]
        d_o = [x for x in d_o if abs(x) > 1e-9]
        out.update(o_candidates=len(cand), o_kept=len(d_o), d_o_abs_median=float(np.median(np.abs(d_o))) if d_o else None)
        for h in hands:
            e, n = A.error(loader, h)
            out["hands"][h] = {"E": e, "pairs": n}
        return out
    if task == "disk":
        import analyze_roll as AR
        AR.TASK, AR.RANDOM = "tipdial_z", True
        out = {"hands": {}}
        for h in hands:
            r = AR.tip_error(h, "point", tg)
            if r is None:
                continue
            out["hands"][h] = {"E": r["E"], "pairs": r["pairs"], "lost": r["lost"], "over_pressed": r["over_pressed"]}
            if h == "O" or "o_candidates" not in out:
                exc = r.get("o_excluded", {})
                out["o_candidates"] = r["candidates"] + sum(exc.values())
                out["o_kept"] = r["candidates"]
        o = {m: AR.tip_load("O", m, "point", tg, "any") for m in MU}
        d_o = []
        for k_ in o[MU[0]]:
            os_ = [o[m][k_] for m in MU]
            if all(d["touched"] for d in os_) and not any(d["distal_force_max_n"] / AR.POINT_K > AR.TIP_PEN_MM or
                                                          d["proximal_force_max_n"] > AR.PROX_TOUCH_N for d in os_):
                dd = os_[1]["rotation_deg"] - os_[0]["rotation_deg"]
                if abs(dd) >= 0.3:
                    d_o.append(dd)
        out["d_o_abs_median"] = float(np.median(np.abs(d_o))) if d_o else None
        return out
    if task == "coin":
        import coin_drag as CD
        keep, excl, o = CD.o_pairs(tg)
        out = {"o_candidates": len(o[MU[0]]), "o_kept": len(keep),
               "d_o_abs_median": float(np.median(np.abs(list(keep.values())))) if keep else None, "hands": {}}
        for h in hands:
            r = CD.hand_error(h, tg, keep)
            if r:
                out["hands"][h] = {"E": r["E"], "pairs": r["pairs"], "lost": r["lost"], "over_pressed": r["over_pressed"]}
        return out
    if task == "pivot":
        import box_pivot as BP
        keep, excl = BP.o_keep(tg)
        d_o = [v[1]["tilt_deg"] - v[0]["tilt_deg"] for v in keep.values()]
        out = {"o_candidates": len(BP._load("O", tg)), "o_kept": len(keep),
               "d_o_abs_median": float(np.median(np.abs(d_o))) if d_o else None, "hands": {}}
        for h in hands:
            r = BP.hand_error(h, tg, keep)
            if r:
                out["hands"][h] = {"E": r["E"], "pairs": r["pairs"], "lost": r["lost"], "over_pressed": r["over_pressed"]}
        return out
    if task == "shape":
        import pad_shape as PS
        keep, excl = PS.o_pairs(tg)
        cand = 4 * len({(r["rho_mm"], r["q0_deg"]) for r in json.loads((LOGS / f"shape_O_{tg}.json").read_text())["runs"]})
        d_o = [v[0] for v in keep.values()]
        out = {"o_candidates": cand, "o_kept": len(keep),
               "d_o_abs_median": float(np.median(np.abs(d_o))) if d_o else None, "hands": {}}
        for h in hands:
            r = PS.hand_error(h, tg, keep)
            if r:
                out["hands"][h] = {"E": r["E"], "pairs": r["pairs"], "gate0": r["gate0"],
                                   "over_pressed": r["over_pressed"]}
        return out


def o_passes(task, res):
    """The pre-registered "O's gates pass and D(O) clearly nonzero": O keeps at least half of its candidates and at
    least 20 pairs, and the median |D(O)| is at least three times the task's floor (pull: 3 load steps, 0.2 N)."""
    floor = 0.2 if task == "pull" else FLOOR[task]
    return bool(res["o_kept"] >= max(20, 0.5 * res["o_candidates"]) and res["d_o_abs_median"] is not None
                and res["d_o_abs_median"] >= 3 * floor)


def cmd_analyze(a) -> int:
    from analyze import DESIGN, spearman_perm
    table = {}
    for k in a.k:
        for task in a.tasks:
            tg = tag(k)
            hands = ["O", "R", "U"] + ([h for h in DESIGN if h not in ("R", "U")] if a.full else [])
            try:
                res = task_result(task, tg, hands)
            except (FileNotFoundError, TypeError, KeyError):
                continue
            if res is None:
                continue
            res["o_passes"] = o_passes(task, res)
            hr = res["hands"]
            er, eu = (hr.get(h, {}).get("E") for h in ("R", "U"))
            res["E_R_lt_E_U"] = bool(er is not None and eu is not None and er < eu)
            sc = [(DESIGN[h], hr[h]["E"]) for h in DESIGN if h in hr and hr[h]["E"] is not None]
            if a.full and len(sc) >= 3:
                rho, p = spearman_perm([x[0][0] for x in sc], [x[1] for x in sc])
                rs, ps = spearman_perm([x[0][1] for x in sc], [x[1] for x in sc])
                res.update(rho_dc=rho, p_dc=p, rho_ds=rs, p_ds=ps, hands_scored=len(sc))
            table[f"{task}@{k:g}"] = res
            pc = lambda v: "   -  " if v is None else f"{100 * v:5.1f}%"
            line = (f"K {k:>4g} {task:5s}: O {res['o_kept']}/{res['o_candidates']} kept, |D(O)| med "
                    f"{res['d_o_abs_median'] if res['d_o_abs_median'] is None else round(res['d_o_abs_median'], 2)} "
                    f"{UNIT[task]} -> O pass {res['o_passes']} | E_R {pc(er)} E_U {pc(eu)} (R<U {res['E_R_lt_E_U']})")
            if "rho_dc" in res:
                line += (f" | rho(E,Dc) {res['rho_dc']:.2f} p {res['p_dc']:.4f}; rho(E,Ds) {res['rho_ds']:.2f} "
                         f"p {res['p_ds']:.4f} ({res['hands_scored']} hands)")
            print(line, flush=True)
    if a.out:
        a.out.write_text(json.dumps(table, indent=1, default=float))
    return 0


def cmd_check(a) -> int:
    """K = 5 against the primary runs: E of R and U (and O's kept count) per task, which must be identical."""
    primary = {"pull": "e21", "disk": "e29z", "coin": "e30b", "pivot": "e35", "shape": "e34"}
    for task in a.tasks:
        new = task_result(task, tag(5.0), ["R", "U"])
        old = task_result(task, primary[task], ["R", "U"])
        same = all(new["hands"][h]["E"] == old["hands"][h]["E"] for h in ("R", "U")) and new["o_kept"] == old["o_kept"]
        print(f"{task}: K 5 E_R {new['hands']['R']['E']} E_U {new['hands']['U']['E']} O kept {new['o_kept']} | primary "
              f"{old['hands']['R']['E']} {old['hands']['U']['E']} {old['o_kept']} -> identical {same}")
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    pr = sub.add_parser("run")
    pr.add_argument("--k", type=float, nargs="+", required=True)
    pr.add_argument("--tasks", nargs="+", default=list(TASKS))
    pr.add_argument("--hands", nargs="+", default=["O", "R", "U"])
    pr.add_argument("--only", type=int, default=None)
    pr.add_argument("--workers", type=int, default=6)
    pa = sub.add_parser("analyze")
    pa.add_argument("--k", type=float, nargs="+", required=True)
    pa.add_argument("--tasks", nargs="+", default=list(TASKS))
    pa.add_argument("--full", action="store_true", help="all 18 hands (stage 2) and the correlations")
    pa.add_argument("--out", type=Path, default=None)
    pc = sub.add_parser("check")
    pc.add_argument("--tasks", nargs="+", default=list(TASKS))
    a = p.parse_args()
    return {"run": cmd_run, "analyze": cmd_analyze, "check": cmd_check}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main())
