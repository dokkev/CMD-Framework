"""Experiment 38: hook-and-pull (experimental; docs/experimental/drake_hook.md).

The flexed finger hooks a thin bar (a handle, radius rho 4-6 mm, free in the finger's plane) between its proximal and
distal pads, and a load ramps until the hook gives way: out of the hook along the palm normal (-z), diagonally
(+x-z) or distally along the finger (+x). Condition A as the pull-out (experiment 21): the bar placed where O's two pad
faces both touch it at q0 (the two_pad_grasps rule, dial_placements.x_contact, in each hand's proximal-face frame),
the bar held while the finger closes to q0 + 5 deg, released, settled (Gate 0), then the load ramp (2/3 N/s per
finger, first of slip, loss, collapse), each direction branched from the settled state. Point contact, K_Q 5, mu
1.3 and 2.0. Measure F_limit; D = F_limit(2.0) - F_limit(1.3); the load-displacement curve for the deviation.

    uv run python hook_pull.py place [--radii 4 5 6]
    uv run python hook_pull.py run --hands O --tag e38 [--only 6]
    uv run python hook_pull.py analyze --tag e38 --noise_tag e38n --out logs/e38_hook.json
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
DIRS = ("-z", "+x-z")  # out of the hook and diagonally; +x (along the finger) dropped on O: its failure event switches
PLACES = LOGS / "hook_places.json"
PEN_MM = 1.5


def cmd_place(a) -> int:
    from dial_placements import x_contact
    g = T.finger_geometry("O")
    qs, th = T.servo_table(g)
    rows = []
    for rho in a.radii:
        for q0 in np.arange(a.q_lo, a.q_hi + 1e-9, a.q_step):
            x = x_contact(g, qs, th, float(q0), rho)
            if x is not None:
                rows.append({"rho_mm": float(rho), "q0_deg": float(q0), "x_c_mm": float(x)})
    PLACES.write_text(json.dumps({"radii": a.radii, "places": rows}, indent=1))
    print(f"{len(rows)} placements -> {PLACES.name}: " +
          ", ".join(f"rho {r}: {sum(p['rho_mm'] == r for p in rows)}" for r in a.radii))
    return 0


def job(task):
    hand, pl, delta = task
    import pad_shape as PS
    import preload_match as P
    t0 = time.time()
    rho = pl["rho_mm"]
    rec = {"hand": hand, **pl, "runs": {}}
    try:
        for mu in MU:
            s, m_, gate, pre, c = P.pull_settle(hand, mu, rho, pl, pl["q0_deg"] + 5.0 + delta)
            pre["pen_mm"] = float(max(m_["normal"].max(), 0.0)) / 2.5
            run = {"gate0": gate, "pre": pre, "dirs": {}}
            if gate["pass"]:
                state = s.plant.GetPositionsAndVelocities(s.pctx).copy()
                for d in DIRS:
                    b = T.Stage(hand, mu, rho, "point", 1.0e-3, False, c["k_theta"], c["d_theta"])
                    b.plant.SetPositionsAndVelocities(b.pctx, state)
                    b.ctrl.theta_cmd = s.ctrl.theta_cmd
                    b.sim.Initialize()
                    lim, curve = PS.ramp(b, b.measure(), d)
                    run["dirs"][d] = {"limit": lim, "curve": curve}
            rec["runs"][f"{mu:g}"] = run
    except RuntimeError as err:
        rec["solver_fail"] = str(err)[:200]
    rec["wall_s"] = time.time() - t0
    return rec


def cmd_run(a) -> int:
    places = json.loads(PLACES.read_text())["places"]
    if a.only:
        places = places[:: max(1, len(places) // a.only)][: a.only]
    t0 = time.time()
    with Pool(a.workers) as pool:
        recs = pool.map(job, [(h, pl, a.delta_deg) for h in a.hands for pl in places], chunksize=1)
    for h in a.hands:
        rr = [r for r in recs if r["hand"] == h]
        (LOGS / f"hook_{h}_{a.tag}.json").write_text(json.dumps({"hand": h, "tag": a.tag, "places": rr}))
        g0 = {f"{mu:g}": sum(r.get("runs", {}).get(f"{mu:g}", {}).get("gate0", {}).get("pass", False) for r in rr)
              for mu in MU}
        print(f"{h} {a.tag}: {len(rr)} placements, Gate 0 {g0}, cpu {sum(r['wall_s'] for r in rr):.0f} s", flush=True)
    print(f"done in {time.time() - t0:.0f} s (workers {a.workers})", flush=True)
    return 0


# ----------------------------------------------------------------------------------------------------- analysis
D_MIN_N = 0.2  # three load steps


def _load(hand, tag):
    p = LOGS / f"hook_{hand}_{tag}.json"
    if not p.exists():
        return None
    return {(r["rho_mm"], r["q0_deg"]): r for r in json.loads(p.read_text())["places"]}


def _flim(rec, mu, d):
    if rec is None or rec.get("solver_fail"):
        return None
    run = rec["runs"][f"{mu:g}"]
    if not run["gate0"]["pass"]:
        return None
    return run["dirs"][d]["limit"]["f_limit_n"]


def o_keep(tag):
    o = _load("O", tag)
    keep, excl = {}, {"o_solver": 0, "o_gate0": 0, "o_pen": 0, "o_no_limit": 0, "small_d": 0}
    for k, r in sorted(o.items()):
        for d in DIRS:
            if r.get("solver_fail"):
                excl["o_solver"] += 1
            elif not all(r["runs"][f"{mu:g}"]["gate0"]["pass"] for mu in MU):
                excl["o_gate0"] += 1
            elif any(r["runs"][f"{mu:g}"]["pre"]["pen_mm"] > PEN_MM for mu in MU):
                excl["o_pen"] += 1
            elif any(_flim(r, mu, d) is None for mu in MU):
                excl["o_no_limit"] += 1
            elif abs(_flim(r, 2.0, d) - _flim(r, 1.3, d)) < D_MIN_N - 1e-9:
                excl["small_d"] += 1
            else:
                keep[k + (d,)] = _flim(r, 2.0, d) - _flim(r, 1.3, d)
    return keep, excl


def hand_error(hand, tag, keep):
    h = _load(hand, tag)
    if h is None:
        return None
    rel, sens, pens = [], [], []
    cnt = {"solver": 0, "gate0": 0, "over_pressed": 0, "no_limit": 0}
    for k, d_o in keep.items():
        r = h.get(k[:2])
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
        f = [_flim(r, mu, k[2]) for mu in MU]
        if any(x is None for x in f):
            cnt["no_limit"] += 1
            continue
        e = abs((f[1] - f[0]) - d_o) / abs(d_o)
        rel.append(e)
        sens.append(e)
    return {"E": float(np.median(rel)) if rel else None, "E_p75": float(np.percentile(rel, 75)) if rel else None,
            "pairs": len(rel), "E_sens": float(np.median(sens)) if sens else None, **cnt,
            "pen_max_mm": float(max(pens)) if pens else None}


def deviation(tag, keep):
    grid = np.linspace(0.0, 5.0, 101)

    def held(curve):
        c = np.asarray(curve)
        dd = np.maximum.accumulate(np.maximum(c[:, 1], 0.0))
        return np.interp(grid, dd, c[:, 0], right=c[-1, 0])
    o = _load("O", tag)
    out = {"x": grid.round(3).tolist()}
    for hand in ("R", "U"):
        h = _load(hand, tag)
        for mu in MU:
            ys = []
            for k in keep:
                r = h.get(k[:2])
                if r is None or r.get("solver_fail") or not r["runs"][f"{mu:g}"]["gate0"]["pass"]:
                    continue
                ys.append(held(r["runs"][f"{mu:g}"]["dirs"][k[2]]["curve"]) -
                          held(o[k[:2]]["runs"][f"{mu:g}"]["dirs"][k[2]]["curve"]))
            y = np.array(ys)
            out[f"{hand}_{mu:g}"] = {"median": np.median(y, 0).round(4).tolist(),
                                     "q25": np.percentile(y, 25, 0).round(4).tolist(),
                                     "q75": np.percentile(y, 75, 0).round(4).tolist(), "n": len(ys)}
    return out


def cmd_analyze(a) -> int:
    from analyze import DESIGN, spearman_perm
    keep, excl = o_keep(a.tag)
    d_o = np.array(list(keep.values()))
    import collections
    print(f"{a.tag}: O pairs {len(keep)} ({len({k[:2] for k in keep})} placements; by direction "
          f"{dict(collections.Counter(k[2] for k in keep))}), excluded {excl}; D(O) {d_o.min():+.2f}/"
          f"{np.median(d_o):+.2f}/{d_o.max():+.2f} N (negative {int((d_o < 0).sum())})")
    out = {"tag": a.tag, "o_pairs": len(keep), "o_excluded": excl, "d_o_n": d_o.tolist(), "rows": []}
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
            print(f"deviation {k}: peak {m[i]:+.3f} N at {out['deviation']['x'][i]:.2f} mm, end {m[-1]:+.3f} N, "
                  f"n {out['deviation'][k]['n']}")
    if a.out:
        a.out.write_text(json.dumps(out, indent=1, default=float))
    return 0


def cmd_check(a) -> int:
    for hand in a.hands:
        h = _load(hand, a.tag)
        g0 = {f"{mu:g}": sum(r["runs"][f"{mu:g}"]["gate0"]["pass"] for r in h.values() if "runs" in r) for mu in MU}
        pens = [max(r["runs"][f"{mu:g}"]["pre"]["pen_mm"] for mu in MU) for r in h.values() if "runs" in r]
        fail = sum("solver_fail" in r for r in h.values())
        line = (f"{hand} {a.tag}: {len(h)} placements, Gate 0 {g0}, solver {fail}, settle pen max {max(pens):.2f} mm "
                f"(> 1.5: {sum(p > PEN_MM for p in pens)})")
        if a.show_d:
            keep, excl = o_keep(a.tag) if hand == "O" else (None, None)
            if keep is not None:
                d = np.array(list(keep.values()))
                line += f"; O pairs {len(keep)} excl {excl}; D(O) {d.min():+.2f}/{np.median(d):+.2f}/{d.max():+.2f} N"
        print(line)
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    pp = sub.add_parser("place")
    pp.add_argument("--radii", type=float, nargs="+", default=[4.0, 5.0, 6.0])
    pp.add_argument("--q_lo", type=float, default=40.0)
    pp.add_argument("--q_hi", type=float, default=115.0)
    pp.add_argument("--q_step", type=float, default=5.0)
    pr = sub.add_parser("run")
    pr.add_argument("--hands", nargs="+", default=["O"])
    pr.add_argument("--tag", required=True)
    pr.add_argument("--only", type=int, default=None)
    pr.add_argument("--delta_deg", type=float, default=5.0, help="overclosure beyond q0 + 5 (the task's 10 deg: 5)")
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
    return {"place": cmd_place, "run": cmd_run, "analyze": cmd_analyze, "check": cmd_check}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main())
