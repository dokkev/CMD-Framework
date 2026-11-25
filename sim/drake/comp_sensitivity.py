"""Experiment 32: sensitivity of the redesign error to IMPERFECT contact-dependent compensation (experimental;
docs/experimental/drake_comp_sensitivity.md).

Two compensations that need contact information, each with an injected error, one factor at a time:
  F (experiment 31, preload_match.py): the PIP setpoint is matched so that the settled distal normal force equals O's.
    Force-estimation error: the hand is matched to (1 + e_F) F*, e_F = sign * level.
  C (experiment 27, two_pad_drake.contact_setpoint): the setpoint puts the hand's distal-face point below the fingertip
    where O's lands at O's setpoint, from kinematics. Contact-location error: the hand's reference point is taken
    sign * level mm along the distal face from the fingertip point (+ toward the tip); O's target point is unchanged.
  Joint position error (both): an encoder offset sign * level deg on the executed command (the setpoint and everything
    after it; the gains are scheduled at the executed command, as in the two modules).
Signs: per placement (pull-out) or draw (disk), +-1 with equal probability from numpy.random.default_rng([32, seed,
kind]) (kind 1 force, 2 location, 3 joint), the same draws for every hand and every level (common random numbers).

Tasks (imported read-only from preload_match.py and two_pad_drake.py):
  pull F: experiment 31's pull-out (matched setpoint, +5 deg shared increment, hold, ramp). Zero error = tag e31.
  pull C: experiment 27's pull-out (contact_ref setpoint, condition A's settle and ramp; the four directions branch from
          the settled state, which reproduces the per-direction runs exactly).
  disk F: experiment 31's fingertip disk at a preload pre_deg (O's setpoint q_t + pre_deg). pre 2 zero error = e31.
  disk C: the disk with the contact-referenced setpoint (H's fingertip point where O's lands at q_t + pre_deg), then the
          same sweep q_set -> q_set + 40 deg.
Logs: logs/pm_{pull|disk}_{hand}_{tag}.json in preload_match's record format, so its analysis reads them.

    uv run python comp_sensitivity.py check                      # zero error == experiments 31 and 27
    uv run python comp_sensitivity.py run --task pull --method F --conds force=0.05 joint=1 --seeds 1 2 --hands O R U
    uv run python comp_sensitivity.py run --task disk --method C --pre 12 --conds zero loc=1 --seeds 1 --hands O
    uv run python comp_sensitivity.py analyze --out logs/e32.json
"""
from __future__ import annotations

import argparse
import json
import math
import time
from functools import lru_cache
from multiprocessing import Pool

import numpy as np

import preload_match as P
import two_pad_drake as T

LOGS = P.LOGS
MU = P.MU
LEVELS = {"force": (0.05, 0.10, 0.20), "loc": (0.5, 1.0, 2.0), "joint": (0.5, 1.0, 2.0)}
UNITS = {"force": "", "loc": "mm", "joint": "deg"}
KIND = {"force": 1, "loc": 2, "joint": 3}
INC_DEG = 5.0  # experiment 31's increment
PULL_F_OPTS = {"inc_deg": INC_DEG, "inc_s": 2.0, "hold_s": 0.5, "directions": list(T.DIRECTIONS)}
PULL_C_OPTS = {"inc_deg": 0.0, "inc_s": 0.0, "hold_s": 0.0, "directions": list(T.DIRECTIONS)}


# ----------------------------------------------------------------------------------------------------- conditions
def parse_cond(text: str) -> dict:
    """'zero' or 'force=0.1,joint=1' -> {'force': 0.1, 'joint': 1.0} (absent kinds are 0)."""
    err = {k: 0.0 for k in KIND}
    if text != "zero":
        for part in text.split(","):
            k, v = part.split("=")
            err[k] = float(v)
    return err


def tag_of(task: str, method: str, pre: float | None, err: dict, seed: int | None, noise: str | None = None) -> str:
    fam = f"{task[0]}{method}" + (f"{pre:g}" if task == "disk" else "")
    if noise:
        return f"e32_{fam}_n"
    if not any(err.values()):
        if method == "F" and (task == "pull" or pre == 2.0):
            return "e31"  # experiment 31 is this family's zero-error run
        return f"e32_{fam}_zero"
    body = "_".join(f"{k}{err[k]:g}" for k in ("force", "loc", "joint") if err[k])
    return f"e32_{fam}_{body}_s{seed}"


def pull_places():
    """Experiment 31's placements (O passes Gate 0 under A at mu 1.3), in its order."""
    grasps = json.loads(T.GRASPS.read_text())["grasps"]
    out = []
    for r in (15.0, 20.0):
        out += [(r, g) for g in grasps if g["rho_mm"] == r and P.a_target(r, g["q0_deg"], g["x_c_mm"]) is not None]
    return out


def disk_draws():
    draws = json.loads((LOGS / "tipdial_z_e29.json").read_text())["grasps"]
    keep = set(json.loads((LOGS / "e31_disk_draws.json").read_text())["draws"])
    return [g for g in draws if g["draw"] in keep]


@lru_cache(maxsize=None)
def sign_table(seed: int, kind: str, n: int) -> tuple:
    return tuple(int(x) for x in np.random.default_rng([32, seed, KIND[kind]]).choice([-1, 1], size=n))


def signed(err: dict, seed: int, index: int, n: int) -> dict:
    """The signed errors of one placement / draw (index into its list of n)."""
    return {k: (sign_table(seed, k, n)[index] * v if v else 0.0) for k, v in err.items()}


# ----------------------------------------------------------------------------------------------------- setpoints
def contact_setpoint_err(hand: str, q_o_deg: float, dx_mm: float, q_max_deg: float = 125.0) -> dict:
    """Experiment 27's mapping with a contact-location error: the hand's reference point is its fingertip face point
    moved dx_mm along the distal face; O's target (its fingertip point at q_o) is unchanged. dx = 0 is exactly
    two_pad_drake.contact_setpoint(hand, q_o, 'tip')."""
    if dx_mm == 0.0:
        return T.contact_setpoint(hand, q_o_deg, "tip", q_max_deg)
    from scipy.optimize import minimize_scalar

    g_o = T.finger_geometry("O")
    target = T.reference_point(g_o, *T.servo_table(g_o, q_max_deg), math.radians(q_o_deg), "tip")
    g = T.finger_geometry(hand)
    qs, th = T.servo_table(g, q_max_deg)
    x = g["pads"]["distal_end_mm"] + dx_mm
    dist = lambda q: float(np.linalg.norm(T.reference_point(g, qs, th, q, x) - target))
    grid = np.radians(np.arange(0.0, q_max_deg + 0.25, 0.5))
    k = int(np.argmin([dist(q) for q in grid]))
    lo, hi = grid[max(k - 1, 0)], grid[min(k + 1, len(grid) - 1)]
    res = minimize_scalar(dist, bounds=(lo, hi), method="bounded", options={"xatol": 1e-9})
    q = float(res.x)
    return {"where": x, "q_cmd_o_deg": q_o_deg, "q_cmd_deg": math.degrees(q), "residual_mm": dist(q),
            "at_bound": bool(q <= 1e-6 or math.degrees(q) >= q_max_deg - 1e-4), "target_mm": target.tolist()}


# ----------------------------------------------------------------------------------------------------- runners
def pull_task(task):
    hand, idx, n, r, g, method, err, seed, base = task
    t0 = time.time()
    e = signed(err, seed, idx, n)
    target = P.a_target(r, g["q0_deg"], g["x_c_mm"])
    rec = {"hand": hand, "rho_mm": r, "q0_deg": g["q0_deg"], "x_c_mm": g["x_c_mm"], "target_n": target, "err": e}
    q_o = g["q0_deg"] + 5.0
    try:
        if method == "F":
            opts = PULL_F_OPTS
            if e["force"] == 0.0:
                if base is not None:
                    m = dict(base["match"])
                elif hand == "O":
                    m = {"q_set_deg": q_o, "force_n": target, "evals": 0, "converged": True, "history": [],
                         "shift_deg": 0.0}
                else:
                    raise ValueError(f"no zero-error record for {hand}")
            else:
                goal = target * (1.0 + e["force"])
                x0 = base["match"]["q_set_deg"] if base is not None else q_o
                f = lambda q: float(P.pull_settle(hand, 1.3, r, g, q)[1]["normal"][1])
                q_set, f_set, n_ev, ok, hist = P.match(f, goal, x0, 2.0, 0.0, 135.0)
                m = {"q_set_deg": q_set, "force_n": f_set, "goal_n": goal, "evals": n_ev, "converged": bool(ok),
                     "history": hist, "shift_deg": q_set - q_o}
        else:
            opts = PULL_C_OPTS
            sp = contact_setpoint_err(hand, q_o, e["loc"])
            m = {"q_set_deg": sp["q_cmd_deg"], "setpoint": sp, "evals": 0, "converged": True,
                 "shift_deg": sp["q_cmd_deg"] - q_o}
        m["q_exec_deg"] = m["q_set_deg"] + e["joint"]
        rec["match"] = m
        rec["runs"] = {f"{mu:g}": P.pull_run(hand, mu, r, g, m["q_exec_deg"], opts) for mu in MU}
    except RuntimeError as exc:
        rec["solver_fail"] = str(exc)[:200]
    rec["wall_s"] = time.time() - t0
    return rec


def disk_task(task):
    hand, idx, n, g, method, err, seed, base, pre = task
    t0 = time.time()
    e = signed(err, seed, idx, n)
    opts = {**P.DISK_OPTS, "pre_deg": pre, "target_scale": 1.0}
    q_o = g["q0_deg"] + pre
    rec = {"hand": hand, "draw": g["draw"], "q_t_deg": g["q0_deg"], "err": e}
    try:
        if method == "F":
            if not any(e.values()) and base is None:  # this family's zero-error run: experiment 31's code as is
                out = P.disk_task((hand, g, opts))
                out["err"] = e
                out["match"]["q_exec_deg"] = out["match"]["q_set_deg"]
                return out
            target = base["target_n"]
            rec["target_n"] = target
            if e["force"] == 0.0:
                m = dict(base["match"])
            else:
                goal = target * (1.0 + e["force"])
                f = lambda q: P.disk_run(hand, 1.3, g, q, opts, preload_only=True)["settled_normal_n"]
                q_set, f_set, n_ev, ok, hist = P.match(f, goal, base["match"]["q_set_deg"], 1.5, 0.0, 135.0)
                m = {"q_set_deg": q_set, "force_n": f_set, "goal_n": goal, "evals": n_ev, "converged": bool(ok),
                     "history": hist, "shift_deg": q_set - q_o}
        else:
            sp = contact_setpoint_err(hand, q_o, e["loc"])
            m = {"q_set_deg": sp["q_cmd_deg"], "setpoint": sp, "evals": 0, "converged": True,
                 "shift_deg": sp["q_cmd_deg"] - q_o}
        m["q_exec_deg"] = m["q_set_deg"] + e["joint"]
        rec["match"] = m
        rec["runs"] = {f"{mu:g}": P.disk_run(hand, mu, g, m["q_exec_deg"], opts) for mu in MU}
    except RuntimeError as exc:
        rec["solver_fail"] = str(exc)[:200]
    rec["wall_s"] = time.time() - t0
    return rec


def _noise_task_disk(task):
    """Experiment 31's disk noise floor at another preload: O matched to 1.02 x its own target."""
    hand, g, pre = task
    return P.disk_task((hand, g, {**P.DISK_OPTS, "pre_deg": pre, "target_scale": 1.02}))


def _base(task, hand, tag_zero):
    """The family's zero-error records of a hand, keyed by placement / draw (None if absent)."""
    d = P._load(task, hand, tag_zero)
    if d is None:
        return {}
    if task == "pull":
        return {(r["rho_mm"], r["q0_deg"], round(float(r["x_c_mm"]), 3)): r for r in d["places"] if "match" in r}
    return {r["draw"]: r for r in d["draws"] if "match" in r}


def run_main(a):
    places = pull_places() if a.task == "pull" else disk_draws()
    n = len(places) if a.task == "pull" else 64  # disk signs indexed by draw id (seed 29's 64 draws)
    jobs = []
    for text in a.conds:
        err = parse_cond(text)
        for seed in (a.seeds if any(err.values()) else [None]):
            tag = tag_of(a.task, a.method, a.pre, err, seed)
            zero = tag_of(a.task, a.method, a.pre, {k: 0.0 for k in KIND}, None)
            for hand in a.hands:
                out = LOGS / f"pm_{a.task}_{hand}_{tag}.json"
                if out.exists() and not a.force:
                    print(f"exists, skipped: {out.name}", flush=True)
                    continue
                if tag == "e31":
                    print(f"zero error of this family is experiment 31's log, not rerun: {out.name}", flush=True)
                    continue
                base = _base(a.task, hand, zero) if a.method == "F" and tag != zero else {}
                if a.task == "pull":
                    tasks = [(hand, i, n, r, g, a.method, err, seed or 0,
                              base.get((r, g["q0_deg"], round(float(g["x_c_mm"]), 3))))
                             for i, (r, g) in enumerate(places)]
                else:
                    tasks = [(hand, g["draw"], n, g, a.method, err, seed or 0, base.get(g["draw"]), a.pre)
                             for g in places]
                jobs.append((out, hand, tag, err, seed, tasks))
    t_all = time.time()
    flat = [(j, t) for j, job in enumerate(jobs) for t in job[5]]
    fn = pull_task if a.task == "pull" else disk_task
    with Pool(a.workers) as pool:
        recs = pool.map(fn, [t for _, t in flat], chunksize=1)
    for j, (out, hand, tag, err, seed, tasks) in enumerate(jobs):
        rr = [rec for (jj, _), rec in zip(flat, recs) if jj == j]
        meta = {"hand": hand, "tag": tag, "task": a.task, "method": a.method, "pre_deg": a.pre, "err": err,
                "seed": seed}
        key = "places" if a.task == "pull" else "draws"
        out.write_text(json.dumps({**meta, "opts": PULL_F_OPTS if a.method == "F" else PULL_C_OPTS, key: rr}
                                  if a.task == "pull" else {**meta, "opts": {**P.DISK_OPTS, "pre_deg": a.pre}, key: rr}))
        conv = sum(r.get("match", {}).get("converged", False) for r in rr)
        fail = sum("solver_fail" in r for r in rr)
        wall = sum(r.get("wall_s", 0.0) for r in rr)
        print(f"{tag} {hand}: {len(rr)} {key}, matched {conv}, solver fail {fail}, cpu {wall:.0f} s -> {out.name}",
              flush=True)
    print(f"done: {len(jobs)} files in {time.time() - t_all:.0f} s (workers {a.workers})", flush=True)
    return 0


def noise_main(a):
    """O-only noise floors for the new families: pull C and disk C, O's executed command +0.05 deg; disk F at a new
    preload, O matched to 1.02 F* (experiment 31's rules)."""
    tag = tag_of(a.task, a.method, a.pre, {}, None, noise="n")
    t0 = time.time()
    with Pool(a.workers) as pool:
        if a.task == "pull":
            places = pull_places()
            recs = pool.map(_noise_pull_c, [(r, g) for r, g in places], chunksize=1)
            body = {"places": recs}
        elif a.method == "F":
            recs = pool.map(_noise_task_disk, [("O", g, a.pre) for g in disk_draws()], chunksize=1)
            body = {"draws": recs}
        else:
            recs = pool.map(_noise_disk_c, [(g, a.pre) for g in disk_draws()], chunksize=1)
            body = {"draws": recs}
    out = LOGS / f"pm_{a.task}_O_{tag}.json"
    out.write_text(json.dumps({"hand": "O", "tag": tag, **body}))
    print(f"{tag}: {time.time() - t0:.0f} s -> {out.name}")
    return 0


def _noise_pull_c(task):
    r, g = task
    rec = pull_task(("O", 0, 1, r, g, "C", {"force": 0.0, "loc": 0.0, "joint": 0.0}, 0, None))
    if "match" in rec:  # rerun at +0.05 deg
        m = rec["match"]
        m["q_exec_deg"] = m["q_set_deg"] + 0.05
        rec["runs"] = {f"{mu:g}": P.pull_run("O", mu, r, g, m["q_exec_deg"], PULL_C_OPTS) for mu in MU}
    return rec


def _noise_disk_c(task):
    g, pre = task
    opts = {**P.DISK_OPTS, "pre_deg": pre}
    sp = T.contact_setpoint("O", g["q0_deg"] + pre, "tip")
    m = {"q_set_deg": sp["q_cmd_deg"], "setpoint": sp, "evals": 0, "converged": True, "shift_deg": 0.0,
         "q_exec_deg": sp["q_cmd_deg"] + 0.05}
    return {"hand": "O", "draw": g["draw"], "q_t_deg": g["q0_deg"], "match": m,
            "runs": {f"{mu:g}": P.disk_run("O", mu, g, m["q_exec_deg"], opts) for mu in MU}}


# ----------------------------------------------------------------------------------------------------- check
def _cmp_pull(rec, ref_places, label):
    k = (rec["rho_mm"], rec["q0_deg"], round(float(rec["x_c_mm"]), 3))
    same = True
    for mu, runs in rec["runs"].items():
        for d, v in runs["dirs"].items():
            a, b = v["limit"]["f_limit_n"], ref_places[k][mu][d]
            same &= a == b
            print(f"  {label} {k} mu {mu} {d}: F_limit {a} vs {b}")
    return same


def check_main(a):
    """Zero error reproduces experiment 31 (F: the logged matched setpoint, re-executed, gives the logged F_limit and
    traces; the disk likewise) and experiment 27 (C: the branched ramps equal the per-direction runs), bit for bit."""
    places = pull_places()
    ok = True
    # F pull: R at two placements, executed at e31's matched setpoint
    base = _base("pull", "R", "e31")
    for i in (2, 9):
        r, g = places[i]
        k = (r, g["q0_deg"], round(float(g["x_c_mm"]), 3))
        rec = pull_task(("R", i, len(places), r, g, "F", parse_cond("zero"), 1, base[k]))
        ref = {mu: {d: v["limit"]["f_limit_n"] for d, v in base[k]["runs"][mu]["dirs"].items()} for mu in base[k]["runs"]}
        same = _cmp_pull(rec, {k: ref}, "F R")
        same &= all([list(x) for x in rec["runs"][mu]["inc_trace"]] == base[k]["runs"][mu]["inc_trace"]
                    for mu in rec["runs"])
        print(f"F pull R {k}: exact {same}")
        ok &= same
    # F disk: U, draws 0 and 7, at e31's setpoint
    base = _base("disk", "U", "e31")
    for d in (0, 7):
        g = next(x for x in disk_draws() if x["draw"] == d)
        rec = disk_task(("U", d, 64, g, "F", parse_cond("zero"), 1, base[d], 2.0))
        same = all(rec["runs"][mu]["rotation_deg"] == base[d]["runs"][mu]["rotation_deg"] and
                   rec["runs"][mu]["distal_force_max_n"] == base[d]["runs"][mu]["distal_force_max_n"]
                   for mu in rec["runs"])
        print(f"F disk U draw {d}: rotation {[rec['runs'][m]['rotation_deg'] for m in rec['runs']]} vs "
              f"{[base[d]['runs'][m]['rotation_deg'] for m in base[d]['runs']]}; exact {same}")
        ok &= same
    # C pull: U and O at two placements vs experiment 27's logs
    for hand in ("U", "O"):
        ref = {}
        for mu in MU:
            for rr in (15.0, 20.0):
                for e in json.loads((LOGS / f"dk_{hand}_mu{mu:g}_point-e27_r{rr:g}.json").read_text())["envs"]:
                    k = (e["rho_mm"], e["q0_deg"], round(float(e["x_c_mm"]), 3))
                    ref.setdefault(k, {}).setdefault(f"{mu:g}", {})[e["direction"]] = e["limit"]["f_limit_n"]
                    ref[k].setdefault("sp", e["setpoint"]["q_cmd_deg"])
        for i in (1, 10):
            r, g = places[i]
            rec = pull_task((hand, i, len(places), r, g, "C", parse_cond("zero"), 1, None))
            k = (r, g["q0_deg"], round(float(g["x_c_mm"]), 3))
            same = rec["match"]["q_set_deg"] == ref[k]["sp"]
            same &= _cmp_pull(rec, ref, f"C {hand}")
            print(f"C pull {hand} {k}: setpoint {rec['match']['q_set_deg']} vs e27 {ref[k]['sp']}; exact {same}")
            ok &= same
    # location error 0 == the module's mapping; a 1 mm shift moves the setpoint
    for hand in ("R", "U"):
        s0 = contact_setpoint_err(hand, 100.0, 0.0)["q_cmd_deg"]
        s1, s2 = (contact_setpoint_err(hand, 100.0, dx)["q_cmd_deg"] for dx in (1e-12, 1.0))
        print(f"mapping {hand} q_o 100: dx 0 {s0:.6f}, dx 1e-12 (own code) {s1:.6f}, dx +1 mm {s2:.4f} deg")
    print(f"all exact: {ok}")
    return 0


# ----------------------------------------------------------------------------------------------------- analysis
FAMILIES = {  # name -> (task, method, pre, error kinds)
    "pull_F": ("pull", "F", None, ("force", "joint")),
    "pull_C": ("pull", "C", None, ("loc", "joint")),
}


def _rows(task, tag, ref, hands):
    out = {}
    for hand in hands:
        r = P.pull_hand(hand, tag, ref) if task == "pull" else P.disk_hand(hand, tag, ref)
        if r is not None:
            out[hand] = r
    return out


def analyze_main(a):
    from analyze import DESIGN, spearman_perm
    fams = dict(FAMILIES)
    for pre in a.disk_pre:
        fams[f"disk_F{pre:g}"] = ("disk", "F", pre, ("force", "joint"))
        fams[f"disk_C{pre:g}"] = ("disk", "C", pre, ("loc", "joint"))
    hands = list(DESIGN)
    result = {}
    pct = lambda v: "   -  " if v is None else f"{100 * v:5.1f}%"
    for fam, (task, method, pre, kinds) in fams.items():
        zero = tag_of(task, method, pre, {k: 0.0 for k in KIND}, None)
        if P._load(task, "O", zero) is None:
            continue
        if task == "pull":
            ref, excl = P.pull_pairs(zero)
        else:
            ref, excl = P.disk_o(zero)
        res = {"zero_tag": zero, "o_pairs": len(ref), "o_excluded": excl, "kinds": {}}
        z = _rows(task, zero, ref, hands + ["O"])
        ntag = "e31n" if (method == "F" and (task == "pull" or pre == 2.0)) else tag_of(task, method, pre, {}, None, "n")
        nz = _rows(task, ntag, ref, ["O"]).get("O") if P._load(task, "O", ntag) else None
        res["noise_floor"] = {"tag": ntag, **nz} if nz else None
        print(f"\n=== {fam} (zero {zero}; O pairs {len(ref)}; O excluded {excl}; noise "
              f"{pct(nz['E']) if nz else '-'} p75 {pct(nz['E_p75']) if nz else '-'})")
        for kind in kinds + ("comb",):
            levels = (0.0,) + LEVELS[kind] if kind != "comb" else (0.0,)
            per = {}  # (level, seed) -> rows
            per[(0.0, None)] = z
            for seed in a.seeds:
                for lv in levels[1:]:
                    tag = tag_of(task, method, pre, {**{k: 0.0 for k in KIND}, kind: lv}, seed)
                    if P._load(task, "O", tag) is not None or P._load(task, "R", tag) is not None:
                        per[(lv, seed)] = _rows(task, tag, ref, hands + ["O"])
                if kind == "comb":
                    err = {k: 0.0 for k in KIND}
                    err.update({"joint": 1.0, ("force" if method == "F" else "loc"): 1.0 if method == "C" else 0.10})
                    tag = tag_of(task, method, pre, err, seed)
                    if P._load(task, "R", tag) is not None:
                        per[("comb", seed)] = _rows(task, tag, ref, hands + ["O"])
            if kind == "comb":
                if len(per) > 1:
                    res["kinds"]["comb"] = _summ_comb(per, DESIGN, spearman_perm, pct)
                continue
            if len(per) < 2:
                continue
            res["kinds"][kind] = _summ_kind(kind, levels, per, a.seeds, DESIGN, spearman_perm, pct)
        result[fam] = res
    if a.out:
        a.out.write_text(json.dumps(result, indent=1, default=float))
    return 0


def _mean_e(per, hand, lv, seeds):
    vals = [per[(lv, s)][hand]["E"] for s in seeds if (lv, s) in per and hand in per[(lv, s)]
            and per[(lv, s)][hand]["E"] is not None]
    return float(np.mean(vals)) if vals else None


def crossover(levels, e_r, e_u):
    """The smallest error level at which E_U >= E_R, by linear interpolation of E_U - E_R between levels; 0 if it
    already holds without error, None (beyond the largest) if it never does."""
    d = [None if (r is None or u is None) else u - r for r, u in zip(e_r, e_u)]
    if d[0] is not None and d[0] >= 0:
        return 0.0, "at zero error (R <= U already)"
    for i in range(1, len(levels)):
        if d[i] is None or d[i - 1] is None:
            continue
        if d[i] >= 0:
            x = levels[i - 1] + (levels[i] - levels[i - 1]) * (-d[i - 1]) / (d[i] - d[i - 1])
            return float(x), "interpolated"
    return None, "beyond the largest level (U better throughout)"


def _summ_kind(kind, levels, per, seeds, DESIGN, spearman_perm, pct):
    unit = UNITS[kind]
    scale = 100.0 if kind == "force" else 1.0
    seeds = [s for s in seeds if any((lv, s) in per for lv in levels[1:])]
    out = {"levels": list(levels), "seeds": seeds, "per_level": [], "hands": {}}
    print(f"--- {kind} (levels {[f'{scale * l:g}{unit or "%"}' for l in levels]}; seeds {seeds})")
    # per-level table
    for lv in levels:
        keys = [(lv, None)] if lv == 0.0 else [(lv, s) for s in seeds if (lv, s) in per]
        row = {"level": lv}
        for key in keys:
            rows = per[key]
            sc = [h for h in DESIGN if h in rows and rows[h]["E"] is not None]
            rho, p = spearman_perm([DESIGN[h][0] for h in sc], [rows[h]["E"] for h in sc]) if len(sc) >= 3 else (None, None)
            rs, ps = spearman_perm([DESIGN[h][1] for h in sc], [rows[h]["E"] for h in sc]) if len(sc) >= 3 else (None, None)
            g = lambda h, f: rows.get(h, {}).get(f)
            over = {h: rows[h]["over_pressed"] for h in rows if rows[h]["over_pressed"]}
            unm = {h: rows[h]["unmatched"] for h in rows if rows[h]["unmatched"]}
            ent = {"seed": key[1], "E_R": g("R", "E"), "E_U": g("U", "E"), "E_O": g("O", "E"),
                   "FTE_R": g("R", "FTE"), "FTE_U": g("U", "FTE"),
                   "E_h5": [g("h3to5_hr", "E"), g("h3to5_hu", "E")], "E_h9": [g("h3to9_hr", "E"), g("h3to9_hu", "E")],
                   "rho_E_dc": rho, "p_E_dc": p, "rho_E_ds": rs, "p_E_ds": ps, "hands": len(sc),
                   "median_E_over_hands": float(np.median([rows[h]["E"] for h in sc])) if sc else None,
                   "over_pressed": over, "over_pressed_total": int(sum(over.values())), "unmatched": unm,
                   "unmatched_total": int(sum(unm.values())),
                   "pairs_R": g("R", "pairs"), "pairs_U": g("U", "pairs"),
                   "E_by_hand": {h: rows[h]["E"] for h in rows}}
            row.setdefault("by_seed", []).append(ent)
            print(f"  {scale * lv:5g}{unit:3s} s{key[1] or '-'}: E_R {pct(ent['E_R'])} E_U {pct(ent['E_U'])} "
                  f"(O {pct(ent['E_O'])}) FTE R/U {pct(ent['FTE_R'])}/{pct(ent['FTE_U'])} | 5mm {pct(ent['E_h5'][0])}/"
                  f"{pct(ent['E_h5'][1])} 9mm {pct(ent['E_h9'][0])}/{pct(ent['E_h9'][1])} | rho(E,Dc) "
                  f"{rho if rho is None else round(rho, 2)} p {p if p is None else round(p, 4)}; rho(E,Ds) "
                  f"{rs if rs is None else round(rs, 2)} | hands {len(sc)} med {pct(ent['median_E_over_hands'])} "
                  f"| over {ent['over_pressed_total']} unmatched {ent['unmatched_total']} pairs R/U "
                  f"{ent['pairs_R']}/{ent['pairs_U']}", flush=True)
        out["per_level"].append(row)
    # crossover on the seed-mean E, and per seed
    e_r = [_mean_e(per, "R", lv, seeds) if lv else per[(0.0, None)].get("R", {}).get("E") for lv in levels]
    e_u = [_mean_e(per, "U", lv, seeds) if lv else per[(0.0, None)].get("U", {}).get("E") for lv in levels]
    x, how = crossover(levels, e_r, e_u)
    out["crossover"] = {"level": x, "how": how, "E_R_mean": e_r, "E_U_mean": e_u, "per_seed": {}}
    for s in seeds:
        er = [per[(0.0, None)].get("R", {}).get("E")] + [per.get((lv, s), {}).get("R", {}).get("E") for lv in levels[1:]]
        eu = [per[(0.0, None)].get("U", {}).get("E")] + [per.get((lv, s), {}).get("U", {}).get("E") for lv in levels[1:]]
        out["crossover"]["per_seed"][s] = crossover(levels, er, eu)
    prim = out["crossover"]["per_seed"].get(1, (None, "-"))
    out["crossover"]["primary_seed1"] = prim
    print(f"  CROSSOVER (seed 1, primary): {prim[0] if prim[0] is None else round(scale * prim[0], 3)}{unit} ({prim[1]})")
    print(f"  crossover (seed mean): {x if x is None else round(scale * x, 3)}{unit} ({how}); per seed "
          f"{ {s: (None if v[0] is None else round(scale * v[0], 3)) for s, v in out['crossover']['per_seed'].items()} }")
    # slopes (pre-registered): least squares over the four points of seed 1 (zero and the three levels)
    slopes = {}
    for h in list(DESIGN) + ["O"]:
        pts = [(0.0, per[(0.0, None)].get(h, {}).get("E"))]
        pts += [(lv, per[(lv, 1)].get(h, {}).get("E")) for lv in levels[1:] if (lv, 1) in per]
        if any(e is None for _, e in pts) or len(pts) < 3:
            continue
        xs, ys = np.array([p[0] for p in pts]), np.array([p[1] for p in pts])
        slopes[h] = float(np.polyfit(xs, ys, 1)[0])
    sc = [h for h in DESIGN if h in slopes]
    if len(sc) >= 3:
        rc, pc = spearman_perm([DESIGN[h][0] for h in sc], [slopes[h] for h in sc])
        rs, ps = spearman_perm([DESIGN[h][1] for h in sc], [slopes[h] for h in sc])
    else:
        rc = pc = rs = ps = None
    out["slope"] = {"per_hand": slopes, "rho_dc": rc, "p_dc": pc, "rho_ds": rs, "p_ds": ps, "hands": len(sc)}
    sl = lambda h: "-" if h not in slopes else f"{slopes[h] * 100 / scale:+.2f}"
    print(f"  slope (E pct-points per {('%' if kind == 'force' else unit)}): R {sl('R')} U {sl('U')} O {sl('O')}; "
          f"rho(slope, Dc) {rc if rc is None else round(rc, 2)} (permutation p {pc if pc is None else round(pc, 4)}); "
          f"rho(slope, Ds) {rs if rs is None else round(rs, 2)} (p {ps if ps is None else round(ps, 4)}); hands {len(sc)}")
    # rho at the seed-mean E per level
    out["rho_mean_E"] = []
    for lv in levels[1:]:
        em = {h: _mean_e(per, h, lv, seeds) for h in DESIGN}
        hs = [h for h in DESIGN if em[h] is not None]
        if len(hs) >= 3:
            r_, p_ = spearman_perm([DESIGN[h][0] for h in hs], [em[h] for h in hs])
            out["rho_mean_E"].append({"level": lv, "rho_dc": r_, "p_dc": p_, "hands": len(hs)})
    return out


def _summ_comb(per, DESIGN, spearman_perm, pct):
    out = []
    for key, rows in per.items():
        if key[0] != "comb":
            continue
        sc = [h for h in DESIGN if h in rows and rows[h]["E"] is not None]
        rho, p = spearman_perm([DESIGN[h][0] for h in sc], [rows[h]["E"] for h in sc]) if len(sc) >= 3 else (None, None)
        over = int(sum(rows[h]["over_pressed"] for h in rows))
        ent = {"seed": key[1], "E_R": rows.get("R", {}).get("E"), "E_U": rows.get("U", {}).get("E"),
               "E_O": rows.get("O", {}).get("E"), "rho_E_dc": rho, "p_E_dc": p, "hands": len(sc),
               "over_pressed_total": over, "E_by_hand": {h: rows[h]["E"] for h in rows}}
        out.append(ent)
        print(f"--- combined s{key[1]}: E_R {pct(ent['E_R'])} E_U {pct(ent['E_U'])} (O {pct(ent['E_O'])}) rho(E,Dc) "
              f"{rho if rho is None else round(rho, 2)} p {p if p is None else round(p, 4)} hands {len(sc)} over {over}")
    return out


# ----------------------------------------------------------------------------------------------------- calibration
def precal_main(a):
    """O only: the disk's settled preload force (mu 1.3) at q_t + pre for each pre, and O's full run's largest
    penetration at each pre (both mu), over all kept draws."""
    draws = disk_draws()
    t0 = time.time()
    with Pool(a.workers) as pool:
        recs = pool.map(_precal, [(g, pre) for pre in a.pre_list for g in draws], chunksize=1)
    out = {}
    for pre in a.pre_list:
        rr = [r for r in recs if r["pre"] == pre]
        f = np.array([r["settled_n"] for r in rr])
        pen = np.array([r["pen_mm"] for r in rr])
        rot = np.array([r["d_deg"] for r in rr])
        prox = sum(r["prox"] > 1e-3 for r in rr)
        untouched = sum(not r["touched"] for r in rr)
        out[pre] = rr
        print(f"pre +{pre:g} deg: preload {f.min():.3f}/{np.median(f):.3f}/{f.max():.3f} N; pen max {pen.max():.2f} mm "
              f"(>1.5: {int((pen > 1.5).sum())}/{len(rr)}), median {np.median(pen):.2f}; D(O) "
              f"{rot.min():.2f}/{np.median(rot):.2f}/{rot.max():.2f} deg; |D|<0.3 {int((np.abs(rot) < 0.3).sum())}; "
              f"proximal {prox}; untouched {untouched}", flush=True)
    (LOGS / "e32_disk_precal.json").write_text(json.dumps({str(k): v for k, v in out.items()}))
    print(f"{time.time() - t0:.0f} s")
    return 0


def _precal(task):
    g, pre = task
    opts = {**P.DISK_OPTS, "pre_deg": pre}
    q = g["q0_deg"] + pre
    runs = [P.disk_run("O", mu, g, q, opts) for mu in MU]
    return {"draw": g["draw"], "pre": pre, "settled_n": runs[0]["settled_normal_n"],
            "pen_mm": max(r["distal_force_max_n"] for r in runs) / P.POINT_K,
            "d_deg": runs[1]["rotation_deg"] - runs[0]["rotation_deg"],
            "prox": max(r["proximal_force_max_n"] for r in runs), "touched": all(r["touched"] for r in runs)}


def main() -> int:
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    pr = sub.add_parser("run")
    pr.add_argument("--task", choices=("pull", "disk"), required=True)
    pr.add_argument("--method", choices=("F", "C"), required=True)
    pr.add_argument("--pre", type=float, default=None, help="disk: O's setpoint q_t + pre (deg)")
    pr.add_argument("--conds", nargs="+", required=True, help="'zero' or 'force=0.1', 'loc=1', 'joint=0.5', "
                                                               "'force=0.1,joint=1'")
    pr.add_argument("--seeds", type=int, nargs="+", default=[1])
    pr.add_argument("--hands", nargs="+", default=P.HANDS)
    pr.add_argument("--workers", type=int, default=6)
    pr.add_argument("--force", action="store_true", help="rerun even if the log exists")
    pn = sub.add_parser("noise")
    pn.add_argument("--task", choices=("pull", "disk"), required=True)
    pn.add_argument("--method", choices=("F", "C"), required=True)
    pn.add_argument("--pre", type=float, default=None)
    pn.add_argument("--workers", type=int, default=6)
    pc = sub.add_parser("precal")
    pc.add_argument("--pre_list", type=float, nargs="+", default=[2.0, 6.0, 8.0, 10.0, 12.0])
    pc.add_argument("--workers", type=int, default=6)
    sub.add_parser("check")
    pa = sub.add_parser("analyze")
    pa.add_argument("--disk_pre", type=float, nargs="+", default=[2.0])
    pa.add_argument("--seeds", type=int, nargs="+", default=[1, 2])
    pa.add_argument("--out", type=__import__("pathlib").Path, default=None)
    a = p.parse_args()
    if a.cmd == "run":
        if a.task == "disk" and a.pre is None:
            p.error("--pre is required for the disk")
        return run_main(a)
    if a.cmd == "noise":
        return noise_main(a)
    if a.cmd == "precal":
        return precal_main(a)
    if a.cmd == "check":
        return check_main(a)
    return analyze_main(a)


if __name__ == "__main__":
    raise SystemExit(main())
