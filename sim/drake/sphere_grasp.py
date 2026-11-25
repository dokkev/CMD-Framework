"""Experiment 42: grasp acquisition of a sphere (experimental; docs/experimental/drake_sphere.md).

Experiment 23's acquisition (two_pad_drake.py --task pullin, run unchanged) with a sphere in place of the cylinder: the
finger starts open (q = 0), the sphere rests on the proximal face offset_mm toward the tip of where O's two pads meet
it at the setpoint q0, under a constant load toward the face and toward the tip (1.5 N each, three-finger shares); the
same open-loop closing to O's command (2 s), held 1 s, spin damping 2.0 N m s/rad. The sphere stays on experiment 23's
planar slide (x, z, turn about y): the finger is planar, so the sphere's sideways motion is not modelled. Only the
geometry changes: the roller's collision shape is a sphere of radius rho and its inertia a solid sphere's (the mass is
experiment 23's). Measure: drawn_mm, how far the sphere was drawn toward the MCP; D = drawn(2.0) - drawn(1.3).

two_pad_drake.py is not edited: use_sphere() makes pydrake's Cylinder of the roller's length a Sphere (and its unit
inertia a sphere's) in this process and its forked workers.

    uv run python sphere_grasp.py draws --n 64 --seed 42 --out logs/sphere_e42.json
    uv run python sphere_grasp.py run --hands O --draws logs/sphere_e42.json --tag e42 [--only 12] [--contact hydro]
    uv run python sphere_grasp.py check --tag e42 [--hands R U] [--show_d]
    uv run python sphere_grasp.py analyze --tag e42 --noise_tag e42n --out logs/e42_sphere.json
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
from dial_placements import x_contact

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
MU = (1.3, 2.0)
RANDOM = {"rho_mm": (10.0, 16.0), "q0_deg": (85.0, 115.0), "offset_mm": (3.0, 6.0)}
OPTS = {"task": "pullin", "tip_load_n": 1.5, "face_load_n": 1.5, "spin_damping": 2.0, "close_s": 2.0, "hold_s": 1.0}
MIN_D_MM = 0.3
METRIC = "drawn_mm"


def use_sphere():
    import pydrake.geometry as G
    from pydrake.multibody.tree import UnitInertia
    if getattr(G, "_sphere_patch", False):
        return
    cyl, solid = G.Cylinder, UnitInertia.SolidCylinder

    def shape(radius, length):
        return G.Sphere(radius) if abs(length - T.LENGTH_M) < 1e-12 else cyl(radius, length)

    def inertia(radius, length, axis):
        return UnitInertia.SolidSphere(radius) if abs(length - T.LENGTH_M) < 1e-12 else solid(radius, length, axis)
    G.Cylinder = shape
    UnitInertia.SolidCylinder = staticmethod(inertia)
    G._sphere_patch = True


def make_draws(n, seed, ranges=RANDOM):
    rng = np.random.default_rng([seed, 42])
    cols = {k: rng.uniform(lo, hi, n) for k, (lo, hi) in ranges.items()}
    g = T.finger_geometry("O")
    qs, th = T.servo_table(g)
    out = []
    for i in range(n):
        d = {k: float(v[i]) for k, v in cols.items()}
        x = x_contact(g, qs, th, d["q0_deg"], d["rho_mm"])
        out.append({"draw": i, **d, "x_c_mm": None if x is None else float(x)})
    return out


def run_task(task):
    hand, pl, contact, time_step, delta, pad_kh = task
    t0 = time.time()
    rec = {"draw": pl["draw"], "hand": hand}
    opts = {**OPTS, "offset_mm": pl["offset_mm"]}
    if pad_kh:
        opts["pad_kh_mm"] = pad_kh
    for mu in MU:
        e = T.run_env((hand, mu, pl["rho_mm"], contact, time_step, pl, "none", delta, T.MODULUS_PA, opts))
        rec[f"{mu:g}"] = e.get("pullin") or {"solver_fail": e["gate0"].get("solver_fail", "?")}
    rec["wall_s"] = time.time() - t0
    return rec


def cmd_run(a) -> int:
    use_sphere()
    d = json.loads(a.draws.read_text())
    draws = [x for x in d["draws"] if x["x_c_mm"] is not None]
    draws = draws[: a.only] if a.only else draws
    pad_kh = None  # hydro: experiment 21's rule (every hand O's pad foundation stiffness)
    t0 = time.time()
    with Pool(a.workers) as pool:
        recs = pool.map(run_task, [(h, pl, a.contact, a.time_step, a.delta_deg, pad_kh) for h in a.hands for pl in draws],
                        chunksize=1)
    for h in a.hands:
        rr = [r for r in recs if r["hand"] == h]
        (LOGS / f"sphere_{h}_{a.tag}.json").write_text(json.dumps({"hand": h, "tag": a.tag, "contact": a.contact,
                                                                    "time_step": a.time_step, "opts": OPTS,
                                                                    "delta_deg": a.delta_deg, "draws": rr}))
        print(f"{h} {a.tag}: {len(rr)} draws, cpu {sum(r['wall_s'] for r in rr):.0f} s", flush=True)
    print(f"done in {time.time() - t0:.0f} s (workers {a.workers})", flush=True)
    return 0


# ----------------------------------------------------------------------------------------------------- analysis
def _load(hand, tag):
    p = LOGS / f"sphere_{hand}_{tag}.json"
    return {r["draw"]: r for r in json.loads(p.read_text())["draws"]} if p.exists() else None


def lost(x):
    return "solver_fail" in x or not x["held"]


def o_keep(tag, only=None):
    o = _load("O", tag)
    keep, excl = {}, {"o_solver": 0, "o_not_held": 0, "small_d": 0}
    for k, r in sorted(o.items()):
        if only is not None and k not in only:
            continue
        runs = [r[f"{mu:g}"] for mu in MU]
        if any("solver_fail" in x for x in runs):
            excl["o_solver"] += 1
        elif any(not x["held"] for x in runs):
            excl["o_not_held"] += 1
        elif abs(runs[1][METRIC] - runs[0][METRIC]) < MIN_D_MM:
            excl["small_d"] += 1
        else:
            keep[k] = runs
    return keep, excl


def hand_error(hand, tag, keep):
    h = _load(hand, tag)
    if h is None:
        return None
    rel, sens = [], []
    cnt = {"solver": 0, "lost": 0}
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
        d_o = o[1][METRIC] - o[0][METRIC]
        e = abs((runs[1][METRIC] - runs[0][METRIC]) - d_o) / abs(d_o)
        rel.append(e)
        sens.append(e)
    return {"E": float(np.median(rel)) if rel else None, "E_p75": float(np.percentile(rel, 75)) if rel else None,
            "pairs": len(rel), "E_sens": float(np.median(sens)) if sens else None, **cnt}


def cmd_analyze(a) -> int:
    from analyze import DESIGN, spearman_perm
    only = set(range(a.first)) if a.first else None
    keep, excl = o_keep(a.tag, only)
    d_o = np.array([v[1][METRIC] - v[0][METRIC] for v in keep.values()])
    dr = np.array([v[0][METRIC] for v in keep.values()])
    print(f"{a.tag}: O kept {len(keep)}, excluded {excl}; D(O) {d_o.min():.2f}/{np.median(d_o):.2f}/{d_o.max():.2f} mm "
          f"(negative {int((d_o < 0).sum())}); drawn(O, mu 1.3) {dr.min():.1f}/{np.median(dr):.1f}/{dr.max():.1f}")
    out = {"tag": a.tag, "metric": METRIC, "o_kept": len(keep), "o_excluded": excl, "d_o_mm": d_o.tolist(), "rows": []}
    if a.noise_tag:
        nz = hand_error("O", a.noise_tag, keep)
        out["noise_floor"] = nz
        print(f"noise floor ({a.noise_tag}): E {100 * nz['E']:.2f}% (p75 {100 * nz['E_p75']:.2f}%), pairs {nz['pairs']}")
    for hand, (dc, ds) in DESIGN.items():
        r = hand_error(hand, a.tag, keep)
        if r is None:
            continue
        out["rows"].append({"hand": hand, "d_contact": dc, "d_skel": ds, **r})
        print(f"{hand:22s} Dc {dc:4.2f} Ds {ds:5.2f} | E {'-' if r['E'] is None else f'{100 * r['E']:5.1f}%'} | "
              f"E_sens {'-' if r['E_sens'] is None else f'{100 * r['E_sens']:5.1f}%'}"
              f" | pairs {r['pairs']}/{len(keep)} lost {r['lost']} solver {r['solver']}", flush=True)
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
    if a.out:
        a.out.write_text(json.dumps(out, indent=1, default=float))
    return 0


def cmd_check(a) -> int:
    for hand in a.hands:
        h = _load(hand, a.tag)
        rows = [h[k] for k in sorted(h)]
        fail = sum(any("solver_fail" in r[f"{mu:g}"] for mu in MU) for r in rows)
        ok = [r for r in rows if not any("solver_fail" in r[f"{mu:g}"] for mu in MU)]
        nh = sum(any(not r[f"{mu:g}"]["held"] for mu in MU) for r in ok)
        line = f"{hand} {a.tag}: {len(rows)} draws, solver {fail}, not held {nh}"
        if a.show_d:
            good = [r for r in ok if all(r[f"{mu:g}"]["held"] for mu in MU)]
            dd = np.array([r["2"][METRIC] - r["1.3"][METRIC] for r in good])
            dr = np.array([r["1.3"][METRIC] for r in good])
            line += (f"; drawn(1.3) {dr.min():.1f}/{np.median(dr):.1f}/{dr.max():.1f}; D {dd.min():.2f}/"
                     f"{np.median(dd):.2f}/{dd.max():.2f} mm (|D|<0.3: {int((abs(dd) < MIN_D_MM).sum())})")
        print(line)
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    pd_ = sub.add_parser("draws")
    pd_.add_argument("--n", type=int, default=64)
    pd_.add_argument("--seed", type=int, default=42)
    pd_.add_argument("--out", type=Path, required=True)
    pr = sub.add_parser("run")
    pr.add_argument("--hands", nargs="+", default=["O"])
    pr.add_argument("--draws", type=Path, required=True)
    pr.add_argument("--tag", required=True)
    pr.add_argument("--only", type=int, default=None)
    pr.add_argument("--delta_deg", type=float, default=0.0)
    pr.add_argument("--contact", choices=("point", "hydro"), default="point")
    pr.add_argument("--time_step", type=float, default=1.0e-3)
    pr.add_argument("--workers", type=int, default=6)
    pa = sub.add_parser("analyze")
    pa.add_argument("--tag", required=True)
    pa.add_argument("--noise_tag", default=None)
    pa.add_argument("--first", type=int, default=None, help="only draws 0..first-1 (the hydro subset)")
    pa.add_argument("--out", type=Path, default=None)
    pc = sub.add_parser("check")
    pc.add_argument("--tag", required=True)
    pc.add_argument("--hands", nargs="+", default=["O"])
    pc.add_argument("--show_d", action="store_true")
    a = p.parse_args()
    if a.cmd == "draws":
        draws = make_draws(a.n, a.seed)
        a.out.write_text(json.dumps({"seed": a.seed, "n": a.n, "rng": f"numpy.random.default_rng([{a.seed}, 42])",
                                     "ranges": RANDOM, "opts": OPTS, "draws": draws}, indent=1))
        print(f"{len(draws)} draws -> {a.out}; no placement {sum(d['x_c_mm'] is None for d in draws)}")
        return 0
    return {"run": cmd_run, "analyze": cmd_analyze, "check": cmd_check}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main())
