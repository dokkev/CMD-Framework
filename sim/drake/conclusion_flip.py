"""Does a redesign reverse an interface study's conclusion? (docs/experimental/conclusion_flip.md, pre-registered in
808365b.) Re-reads Study 3's existing runs (Drake logs and Isaac Lab logs; no new simulation) at the conclusion level:

- per pair (placement x direction, the pairs of each task's error E, minus pairs whose |D(O)| is below the setting's
  floor f = max(task minimum, P75 |D(O') - D(O)|), O' = O at its setpoint + 0.05 deg): sign flip D(H) D(O) < 0 and
  bias above effect |D(H) - D(O)| > |D(O)|;
- per hand (median D over the same pairs): reversed conclusion, and E(H) > 1;
- ranking: Kendall tau_b of H's interface x direction effects against O's, on the hand sets where several interface
  changes were measured on the same hands and placements;
- per-hand aggregates over the within-hand settings against D_contact and D_skel (Spearman, one-sided permutation p,
  20,000 permutations, seed 0).

    cd sim/drake && uv run python conclusion_flip.py

Writes report/study3/data/conclusion_flip.json.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from scipy.stats import kendalltau, rankdata

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "sim" / "isaaclab" / "scripts" / "study3_measurement"))
import analyze as A  # noqa: E402
import analyze_roll as AR  # noqa: E402
import box_pivot as BP  # noqa: E402
import hook_pull as HP  # noqa: E402
import pad_shape as PS  # noqa: E402
import dose_analysis as DA  # noqa: E402

OUT = ROOT / "report" / "study3" / "data" / "conclusion_flip.json"
MU = (1.3, 2.0)
DIRS = ("+x", "+x-z", "-z", "-x")
R_HANDS = ("h3to5_hr", "R", "h3to9_hr")
U_HANDS = ("h3to5_hu", "U", "h3to9_hu")
DESIGN = {h: v for h, v in A.DESIGN.items()}  # D_contact, D_skel (mm), the 18 hands
GRID = {h: (v[0], v[1]) for h, v in DA.EXPAND.items()}  # the 27 grid hands (seed 1, 7 mm)
MIN_PAIRS = 5  # a hand is judged in a setting on at least this many pairs
MIN_CELL = 3  # a ranking cell needs this many pairs
N_PERM = 20_000


def thickness(h: str) -> int:
    return 3 if h == "O" else int(DA.thickness(h))


# ------------------------------------------------------------------------------------------------ effect loaders
# Each returns {key: D}; key ends with the load direction where the task has one.

def _drake_pull(hand, contact, tag):
    """{mu: {key: F_limit}} over Gate 0 passes with a limit, both radii."""
    out = {}
    for m in MU:
        d = {}
        for r in A.RADII:
            envs = A.load_drake(hand, m, contact, tag, r)
            if envs is None:
                return None
            for e in envs:
                if e["gate0"]["pass"] and e["limit"]["f_limit_n"] is not None:
                    d[A.key(e)] = e["limit"]["f_limit_n"]
        out[m] = d
    return out


def _diff(hi: dict, lo: dict) -> dict:
    return {k: hi[k] - lo[k] for k in hi if k in lo}


def pull_fric(contact, tag):
    def f(hand):
        d = _drake_pull(hand, contact, tag)
        return None if d is None else _diff(d[2.0], d[1.3])
    return f


def pull_soft(mu, role):
    """Hydro pad 0.1 MPa over 3 mm (e21) -> over the pad's own thickness (e25); O's reference: O_h (e25-ref<h>)."""
    def f(hand, h=None):
        base = _drake_pull(hand, "hydro", "e21")
        soft = _drake_pull(hand, "hydro", "e25" if role == "H" else f"e25-ref{h}")
        if base is None or soft is None:
            return None
        return _diff(soft[mu], base[mu])
    return f


def pull_fricsoft(hand, tag="e25"):
    d = _drake_pull(hand, "hydro", tag)
    return None if d is None else _diff(d[2.0], d[1.3])


def shape(tag):
    def f(hand):
        runs = PS._runs(hand, tag)
        if runs is None:
            return None
        out = {}
        for k in sorted({k[:2] for k in runs}):
            rf, rc = runs.get(k + ("flat",)), runs.get(k + ("crowned",))
            if any(r is None or r.get("solver_fail") for r in (rf, rc)):
                continue
            if not (rf["gate0"]["pass"] and rc["gate0"]["pass"]):
                continue
            if max(rf["pre"]["pen_mm"], rc["pre"]["pen_mm"]) > PS.PEN_MM:
                continue
            for d in DIRS:
                ff, fc = PS._flim(rf, d), PS._flim(rc, d)
                if ff is not None and fc is not None:
                    out[k + (d,)] = fc - ff
        return out
    return f


def acquire(contact, tag, lost=None):
    """Experiment 23 (analyze_roll pull-in rules): Gate 0 at both mu and held to the end at both mu. lost: dict that
    collects the keys where the hand passed Gate 0 but lost the cylinder (not scored)."""
    def f(hand):
        AR.TASK = "pullin"
        _, _, variants, rk, metric, _, _ = AR.TASKS["pullin"]
        out = {}
        for s in variants:
            for r in AR.RADII:
                runs = {m: AR.load(hand, m, contact, tag, s, r) for m in MU}
                if any(v is None for v in runs.values()):
                    return None
                for k in runs[MU[0]]:
                    es = [runs[m].get(k) for m in MU]
                    if any(e is None or not e["gate0"]["pass"] or e.get(rk) is None for e in es):
                        continue
                    if not all(e[rk]["held"] for e in es):
                        if lost is not None:
                            lost.setdefault(hand, set()).add((s, r) + k)
                        continue
                    out[(s, r) + k] = es[1][rk][metric] - es[0][rk][metric]
        return out
    return f


def hook(tag):
    def f(hand, role="H"):
        if role == "O":
            keep, _ = HP.o_keep(tag)
            return dict(keep)
        h = HP._load(hand, tag)
        if h is None:
            return None
        out = {}
        for k, r in h.items():
            if r.get("solver_fail"):
                continue
            if not all(r["runs"][f"{mu:g}"]["gate0"]["pass"] for mu in MU):
                continue
            if max(r["runs"][f"{mu:g}"]["pre"]["pen_mm"] for mu in MU) > HP.PEN_MM:
                continue
            for d in HP.DIRS:
                fl = [HP._flim(r, mu, d) for mu in MU]
                if all(x is not None for x in fl):
                    out[k + (d,)] = fl[1] - fl[0]
        return out
    return f


def dial(contact, tag, random):
    def f(hand, role="H"):
        AR.TASK, AR.RANDOM = "tipdial_z", random
        rr = "any" if random else 15.0
        runs = {m: AR.tip_load(hand, m, contact, tag, rr) for m in MU}
        AR.RANDOM = False
        if any(v is None for v in runs.values()):
            return None
        out = {}
        for k in runs[MU[0]]:
            ds = [runs[m].get(k) for m in MU]
            if any(d is None or not d["touched"] for d in ds):
                continue
            if max(d["distal_force_max_n"] for d in ds) / AR.POINT_K > AR.TIP_PEN_MM:
                continue
            if role == "O" and any(d["proximal_force_max_n"] > AR.PROX_TOUCH_N for d in ds):
                continue
            out[k] = ds[1]["rotation_deg"] - ds[0]["rotation_deg"]
        return out
    return f


def pivot(tag):
    def f(hand, role="H"):
        if role == "O":
            keep, _ = BP.o_keep(tag)
            return {k: o[1]["tilt_deg"] - o[0]["tilt_deg"] for k, o in keep.items()}
        h = BP._load(hand, tag)
        if h is None:
            return None
        out = {}
        for k, rec in h.items():
            runs = [rec[f"{m:g}"] for m in BP.MU]
            if any("solver_fail" in x for x in runs) or any(BP.lost(x) for x in runs):
                continue
            if max(x["pen_mm"] for x in runs) > BP.PEN_MM:
                continue
            out[k] = runs[1]["tilt_deg"] - runs[0]["tilt_deg"]
        return out
    return f


def _isaac(hand, cond, mu, radii=(15.0, 20.0)):
    out, found = {}, False
    for r in radii:
        d = DA.latest(hand, mu, cond, r)
        if d is None:
            continue
        found = True
        for e in d["envs"]:
            if e["gate0"]["pass"] and e["limit"]["f_limit_n"] is not None:
                out[(r,) + DA.env_key(e)] = e["limit"]["f_limit_n"]
    return out if found else None


def isaac_fric(cond, radii=(15.0, 20.0)):
    def f(hand):
        lo, hi = _isaac(hand, cond, 1.3, radii), _isaac(hand, cond, 2.0, radii)
        return None if lo is None or hi is None else _diff(hi, lo)
    return f


def isaac_change(base, new, mu):
    """F(new) - F(base) at one mu; new may be a function of the hand's thickness (O's reference O_h)."""
    def f(hand, h=None):
        b = _isaac(hand, base, mu)
        n = _isaac(hand, new(h) if callable(new) else new, mu)
        return None if b is None or n is None else _diff(n, b)
    return f


def isaac_h4(base, hand_cond, ref):
    """H4 / Fig. 1: D(H) = F(H, soft pad) - F(O, stiff pad); D(O) = F(O_h) - F(O, stiff), both mu pooled."""
    def f(hand, role, h):
        out = {}
        for m in MU:
            b = _isaac("O", base, m)
            n = _isaac("O", ref(h), m) if role == "O" else _isaac(hand, hand_cond, m)
            if b is None or n is None:
                return None
            out.update({(m,) + k: v for k, v in _diff(n, b).items()})
        return out
    return f


# --------------------------------------------------------------------------------------------------- settings
# id: (engine/contact, task, interface, unit, task minimum, hands, O(h) -> {key: D}, H(hand) -> {key: D},
#      noise: (O dict, O' dict) maker or None, within-hand (pooled in the per-hand aggregates))

def S(**kw):
    kw.setdefault("noise", None)
    kw.setdefault("pooled", True)
    kw.setdefault("hands", list(DESIGN))
    return kw


def _plain(fn):
    """O and H from the same loader."""
    return {"o": lambda h, fn=fn: fn("O"), "h": fn}


def _role(fn):
    return {"o": lambda h, fn=fn: fn("O", "O"), "h": lambda hand, fn=fn: fn(hand, "H")}


ACQ_LOST = {"point": {}, "hydro": {}}
SETTINGS = {
    "pull_pt_fric": S(model="Drake point", task="pull-out", iface="friction 1.3 -> 2.0", unit="N", fmin=0.2,
                      **_plain(pull_fric("point", "e21")),
                      noise=lambda: (pull_fric("point", "e27")("O"), pull_fric("point", "e27nf")("O"))),
    "pull_hy_fric": S(model="Drake hydro", task="pull-out", iface="friction 1.3 -> 2.0", unit="N", fmin=0.2,
                      **_plain(pull_fric("hydro", "e21"))),
    "pull_hy_soft13": S(model="Drake hydro", task="pull-out", iface="pad 0.1 MPa/3 mm -> /h (mu 1.3)", unit="N",
                        fmin=0.2, o=lambda h: pull_soft(1.3, "O")("O", h), h=lambda hand: pull_soft(1.3, "H")(hand),
                        noise=lambda: (pull_soft(1.3, "O")("O", 7), _diff(
                            _drake_pull("O", "hydro", "e25n-ref7")[1.3], _drake_pull("O", "hydro", "e21")[1.3]))),
    "pull_hy_soft20": S(model="Drake hydro", task="pull-out", iface="pad 0.1 MPa/3 mm -> /h (mu 2.0)", unit="N",
                        fmin=0.2, o=lambda h: pull_soft(2.0, "O")("O", h), h=lambda hand: pull_soft(2.0, "H")(hand),
                        noise=lambda: (pull_soft(2.0, "O")("O", 7), _diff(
                            _drake_pull("O", "hydro", "e25n-ref7")[2.0], _drake_pull("O", "hydro", "e21")[2.0]))),
    "pull_hy_fricsoft": S(model="Drake hydro", task="pull-out", iface="friction on the own-thickness soft pad",
                          unit="N", fmin=0.2, o=lambda h: pull_fricsoft("O", f"e25-ref{h}"),
                          h=lambda hand: pull_fricsoft(hand),
                          noise=lambda: (pull_fricsoft("O", "e25-ref7"), pull_fricsoft("O", "e25n-ref7"))),
    "pull_pt_shape": S(model="Drake point", task="pull-out", iface="flat -> 2 mm crowned (mu 1.3)", unit="N",
                       fmin=0.6, **_plain(shape("e34")), noise=lambda: (shape("e34")("O"), shape("e34n")("O"))),
    "acq_pt": S(model="Drake point", task="acquisition", iface="friction 1.3 -> 2.0", unit="mm", fmin=0.3,
                o=lambda h: acquire("point", "e23")("O"), h=acquire("point", "e23", ACQ_LOST["point"]),
                noise=lambda: (acquire("point", "e23")("O"), acquire("point", "nf")("O"))),
    "acq_hy": S(model="Drake hydro", task="acquisition", iface="friction 1.3 -> 2.0", unit="mm", fmin=0.3,
                o=lambda h: acquire("hydro", "e23h")("O"), h=acquire("hydro", "e23h", ACQ_LOST["hydro"]),
                noise=lambda: (acquire("hydro", "e23h")("O"), acquire("hydro", "e23hn")("O"))),
    "hook_pt": S(model="Drake point", task="hook", iface="friction 1.3 -> 2.0", unit="N", fmin=0.2,
                 **_role(hook("e38")), noise=lambda: (hook("e38")("O", "O"), hook("e38n")("O", "H"))),
    "hook_hy": S(model="Drake hydro", task="hook", iface="friction 1.3 -> 2.0", unit="N", fmin=0.2,
                 **_role(hook("e38h")), noise=lambda: (hook("e38h")("O", "O"), hook("e38hn")("O", "H"))),
    "dial_pt": S(model="Drake point", task="fingertip disk (29z, 64)", iface="friction 1.3 -> 2.0", unit="deg",
                 fmin=0.3, **_role(dial("point", "e29z", True)),
                 noise=lambda: (dial("point", "e29z", True)("O", "O"), dial("point", "e29zn", True)("O", "H"))),
    "dial_hy": S(model="Drake hydro", task="fingertip disk (29zh, first 32)", iface="friction 1.3 -> 2.0",
                 unit="deg", fmin=0.3, **_role(dial("hydro", "e29zh", True)),
                 noise=lambda: (dial("hydro", "e29zh", True)("O", "O"), dial("hydro", "e29zhn", True)("O", "H"))),
    "pivot_pt": S(model="Drake point", task="box pivot (35, 64)", iface="friction 1.3 -> 2.0", unit="deg",
                  fmin=0.3, **_role(pivot("e35")), noise=lambda: (pivot("e35")("O", "O"), pivot("e35n")("O", "H"))),
    "pivot_hy": S(model="Drake hydro", task="box pivot (35h, first 32)", iface="friction 1.3 -> 2.0", unit="deg",
                  fmin=0.3, **_role(pivot("e35h")),
                  noise=lambda: (pivot("e35h")("O", "O"), pivot("e35hn")("O", "H"))),
    "is_fric": S(model="Isaac point", task="pull-out", iface="friction 1.3 -> 2.0", unit="N", fmin=0.2,
                 **_plain(isaac_fric("A")),
                 noise=lambda: (isaac_fric("A", (15.0,))("O"), isaac_fric("A-nf-ptd", (15.0,))("O"))),
    "is_stiff": S(model="Isaac point", task="pull-out", iface="ke 2.5e3 -> 1e3 N/m (mu 1.3)", unit="N", fmin=0.2,
                  **_plain(isaac_change("A", "A-ke1e3", 1.3))),
    "is_soft13": S(model="Isaac point", task="pull-out", iface="ke 2.5e3 -> 2.5e3 x 3/h (mu 1.3)", unit="N",
                   fmin=0.2, o=lambda h: isaac_change("A", lambda t: f"A-padk-ref{t}", 1.3)("O", h),
                   h=lambda hand: isaac_change("A", "A-padk", 1.3)(hand)),
    "is_soft20": S(model="Isaac point", task="pull-out", iface="ke 2.5e3 -> 2.5e3 x 3/h (mu 2.0)", unit="N",
                   fmin=0.2, o=lambda h: isaac_change("A", lambda t: f"A-padk-ref{t}", 2.0)("O", h),
                   h=lambda hand: isaac_change("A", "A-padk", 2.0)(hand)),
    "is_fricsoft": S(model="Isaac point", task="pull-out", iface="friction on the ke x 3/h pad", unit="N", fmin=0.2,
                     o=lambda h: isaac_fric(f"A-padk-ref{h}")("O"), h=isaac_fric("A-padk")),
    "is_soft_H4": S(model="Isaac point", task="pull-out", iface="soft pad, H4 / Fig. 1 (H soft - O stiff)",
                    unit="N", fmin=0.2, pooled=False,
                    o=lambda h: isaac_h4("A", "A-padk", lambda t: f"A-padk-ref{t}")("O", "O", h),
                    h=lambda hand: isaac_h4("A", "A-padk", lambda t: f"A-padk-ref{t}")(hand, "H", thickness(hand))),
    "grid_fric": S(model="Isaac point, grid", task="pull-out", iface="friction 1.3 -> 2.0", unit="N", fmin=0.2,
                   pooled=False, hands=list(DESIGN) + list(GRID), **_plain(isaac_fric("A-grid"))),
    "grid_soft13": S(model="Isaac point, grid", task="pull-out", iface="ke 2.5e3 -> 2.5e3 x 3/7 (mu 1.3)", unit="N",
                     fmin=0.2, pooled=False, hands=list(GRID),
                     o=lambda h: isaac_change("A-grid", "A-grid-padk-ref7", 1.3)("O"),
                     h=lambda hand: isaac_change("A-grid", "A-grid-padk", 1.3)(hand)),
    "grid_soft20": S(model="Isaac point, grid", task="pull-out", iface="ke 2.5e3 -> 2.5e3 x 3/7 (mu 2.0)", unit="N",
                     fmin=0.2, pooled=False, hands=list(GRID),
                     o=lambda h: isaac_change("A-grid", "A-grid-padk-ref7", 2.0)("O"),
                     h=lambda hand: isaac_change("A-grid", "A-grid-padk", 2.0)(hand)),
    "grid_fricsoft": S(model="Isaac point, grid", task="pull-out", iface="friction on the ke x 3/7 pad", unit="N",
                       fmin=0.2, pooled=False, hands=list(GRID), o=lambda h: isaac_fric("A-grid-padk-ref7")("O"),
                       h=isaac_fric("A-grid-padk")),
    "grid_soft_H4": S(model="Isaac point, grid", task="pull-out", iface="soft pad, H4 (exp. 19; H soft - O stiff)",
                      unit="N", fmin=0.2, pooled=False, hands=list(GRID),
                      o=lambda h: isaac_h4("A-grid", "A-grid-padk", lambda t: "A-grid-padk-ref7")("O", "O", 7),
                      h=lambda hand: isaac_h4("A-grid", "A-grid-padk", lambda t: "A-grid-padk-ref7")(hand, "H", 7)),
    "dial_pt28z": S(model="Drake point", task="fingertip disk (28z, 12; Fig. 4)", iface="friction 1.3 -> 2.0",
                    unit="deg", fmin=0.3, pooled=False, **_role(dial("point", "e28z", False)),
                    noise=lambda: (dial("point", "e28z", False)("O", "O"), dial("point", "e28zn", False)("O", "H"))),
}
RANKING = {
    "RS-Isaac": ["is_fric", "is_stiff", "is_soft13", "is_soft20", "is_fricsoft"],
    "RS-grid": ["grid_fric", "grid_soft13", "grid_soft20", "grid_fricsoft"],
    "RS-Drake-hydro": ["pull_hy_fric", "pull_hy_soft13", "pull_hy_soft20", "pull_hy_fricsoft"],
    "RS-Drake-point": ["pull_pt_fric", "pull_pt_shape"],
}


def design(h):
    return DESIGN.get(h) or GRID.get(h)


# --------------------------------------------------------------------------------------------------- statistics

def spearman(x, y) -> float:
    rx, ry = rankdata(x), rankdata(y)
    if np.std(rx) == 0 or np.std(ry) == 0:
        return float("nan")
    return float(np.corrcoef(rx, ry)[0, 1])


def spearman_perm(x, y, sign=+1) -> tuple[float, float]:
    """rho and the one-sided permutation p in the predicted direction (sign +1: rho > 0; -1: rho < 0)."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    ok = ~(np.isnan(x) | np.isnan(y))
    x, y = x[ok], y[ok]
    rho = spearman(x, y)
    if np.isnan(rho):
        return rho, float("nan")
    rng = np.random.default_rng(0)
    rx = rankdata(x)
    perms = np.array([np.corrcoef(rx, rankdata(rng.permutation(y)))[0, 1] for _ in range(N_PERM)])
    perms = np.nan_to_num(perms)
    p = np.mean(perms >= rho - 1e-12) if sign > 0 else np.mean(perms <= rho + 1e-12)
    return rho, float(p)


def partial_spearman(y, x, z) -> float:
    ry, rx, rz = (rankdata(np.asarray(v, float)) for v in (y, x, z))
    a_ = np.c_[rz, np.ones(len(rz))]
    res = lambda v: v - a_ @ np.linalg.lstsq(a_, v, rcond=None)[0]  # noqa: E731
    a, b = res(ry), res(rx)
    den = np.sqrt((a @ a) * (b @ b))
    return float(a @ b / den) if den > 0 else float("nan")


# --------------------------------------------------------------------------------------------------- analysis

def pairs(do: dict, dh: dict, floor: float):
    ks = [k for k in do if k in dh and abs(do[k]) >= floor - 1e-9]
    return ks, np.array([do[k] for k in ks]), np.array([dh[k] for k in ks])


def judge(d_o, d_h, floor):
    n = len(d_o)
    if n == 0:
        return None
    flip = d_h * d_o < 0
    e1 = np.abs(d_h - d_o) > np.abs(d_o)
    mo, mh = float(np.median(d_o)), float(np.median(d_h))
    rel = np.abs(d_h - d_o) / np.abs(d_o)
    judged = n >= MIN_PAIRS and abs(mo) >= floor - 1e-9
    return {"pairs": n, "flips": int(flip.sum()), "zeros": int((d_h == 0).sum()), "e_gt_1": int(e1.sum()),
            "flip_rate": float(flip.mean()), "e1_rate": float(e1.mean()), "median_D_O": mo, "median_D_H": mh,
            "E": float(np.median(rel)), "E_gt_1": bool(np.median(rel) > 1), "judged": judged,
            "reversed": bool(judged and mh * mo < 0)}


def group(rows: dict, hands) -> dict | None:
    rs = [rows[h] for h in hands if h in rows]
    if not rs:
        return None
    n = sum(r["pairs"] for r in rs)
    j = [r for r in rs if r["judged"]]
    return {"hands": len(rs), "pairs": n, "flip_rate": sum(r["flips"] for r in rs) / n,
            "e1_rate": sum(r["e_gt_1"] for r in rs) / n, "hands_judged": len(j),
            "hands_reversed": sum(r["reversed"] for r in j), "hands_E_gt_1": sum(r["E_gt_1"] for r in rs)}


def run_setting(sid, s, cache):
    o_cache = {}

    def o_for(h):
        t = thickness(h)
        if t not in o_cache:
            o_cache[t] = s["o"](t)
        return o_cache[t]

    floor, noise = s["fmin"], None
    if s["noise"]:
        on, onn = s["noise"]()
        ks = [k for k in on if k in onn]
        dev = np.abs(np.array([onn[k] - on[k] for k in ks]))
        p75 = float(np.percentile(dev, 75)) if len(ks) else 0.0
        floor = max(s["fmin"], p75)
        _, a, b = pairs(on, onn, floor)
        noise = {"pairs_all": len(ks), "p75_abs_dev": p75, **(judge(a, b, floor) or {"pairs": 0})}
    rows, raw = {}, {}
    for hand in s["hands"]:
        dh = s["h"](hand)
        do = o_for(hand)
        if not dh or not do:
            continue
        ks, a, b = pairs(do, dh, floor)
        j = judge(a, b, floor)
        if j is None:
            continue
        dc, ds = design(hand)
        rows[hand] = {"d_contact": dc, "d_skel": ds, "thickness": thickness(hand), **j}
        raw[hand] = (do, dh)
    cache[sid] = raw
    out = {k: v for k, v in s.items() if k in ("model", "task", "iface", "unit", "fmin", "pooled")}
    out |= {"floor": floor, "noise": noise, "hands": rows,
            "R": group(rows, R_HANDS), "U": group(rows, U_HANDS), "all": group(rows, list(rows))}
    if sid in ("acq_pt", "acq_hy"):
        lost = ACQ_LOST["point" if sid == "acq_pt" else "hydro"]
        o_keys = {k for k, v in o_for("R").items() if abs(v) >= floor - 1e-9}
        out["lost_pairs"] = {h: len(lost.get(h, set()) & o_keys) for h in rows}
    scored = [h for h, r in rows.items() if r["pairs"] >= MIN_PAIRS]
    if len(scored) >= 5:
        st = {}
        for key in ("flip_rate", "e1_rate"):
            v = [rows[h][key] for h in scored]
            for cand, i in (("d_contact", 0), ("d_skel", 1)):
                st[f"rho_{key}_{cand}"], st[f"p_{key}_{cand}"] = spearman_perm([design(h)[i] for h in scored], v)
        out["stats"] = st | {"hands": len(scored)}
    return out


def ranking(set_id, sids, cache, res):
    """Per hand: Kendall tau_b of interface x direction cell medians against O's (same pairs, no |D(O)| floor)."""
    hands = sorted(set.intersection(*[set(cache[s]) for s in sids]))  # measured on every interface of the set
    rows = {}
    for hand in hands:
        cells_o, cells_h, names = [], [], []
        lev_o, lev_h, lev_names = [], [], []
        for s in sids:
            if hand not in cache[s]:
                continue
            do, dh = cache[s][hand]
            ks = [k for k in do if k in dh]
            if len(ks) >= MIN_CELL:
                lev_o.append(float(np.median([do[k] for k in ks])))
                lev_h.append(float(np.median([dh[k] for k in ks])))
                lev_names.append(s)
            for d in DIRS:
                kd = [k for k in ks if k[-1] == d]
                if len(kd) >= MIN_CELL:
                    cells_o.append(float(np.median([do[k] for k in kd])))
                    cells_h.append(float(np.median([dh[k] for k in kd])))
                    names.append(f"{s}:{d}")
        if len(names) < 4:
            continue
        tau = float(kendalltau(cells_o, cells_h).statistic)
        r = {"items": len(names), "tau_cells": tau, "cells": dict(zip(names, zip(cells_o, cells_h))),
             "d_contact": design(hand)[0], "d_skel": design(hand)[1]}
        if len(lev_names) >= 2:
            r["tau_interfaces"] = float(kendalltau(lev_o, lev_h).statistic) if len(lev_names) >= 2 else None
            r["interfaces"] = dict(zip(lev_names, zip(lev_o, lev_h)))
            r["same_top"] = bool(np.argmax(np.abs(lev_o)) == np.argmax(np.abs(lev_h)))
            r["n_interfaces"] = len(lev_names)
        rows[hand] = r
    out = {"settings": sids, "hands": rows}
    hs = list(rows)
    if len(hs) >= 5:
        for key in ("tau_cells", "tau_interfaces"):
            v = [rows[h].get(key, np.nan) for h in hs]
            for cand, i in (("d_contact", 0), ("d_skel", 1)):
                out[f"rho_{key}_{cand}"], out[f"p_{key}_{cand}"] = spearman_perm(
                    [design(h)[i] for h in hs], v, sign=-1)
    for g, hh in (("R", R_HANDS), ("U", U_HANDS)):
        out[g] = {h: (rows[h]["tau_cells"], rows[h].get("tau_interfaces"), rows[h].get("same_top"))
                  for h in hh if h in rows}
    res["ranking"][set_id] = out


def aggregates(res):
    pooled = [s for s, v in res["settings"].items() if v["pooled"]]
    agg = {}
    for hand, (dc, ds) in DESIGN.items():
        fr, er, rv, eg = [], [], [], []
        for s in pooled:
            r = res["settings"][s]["hands"].get(hand)
            if r is None or r["pairs"] < MIN_PAIRS:
                continue
            fr.append(r["flip_rate"])
            er.append(r["e1_rate"])
            eg.append(float(r["E_gt_1"]))
            if r["judged"]:
                rv.append(float(r["reversed"]))
        taus = [res["ranking"][k]["hands"][hand]["tau_cells"] for k in ("RS-Isaac", "RS-Drake-hydro", "RS-Drake-point")
                if hand in res["ranking"][k]["hands"]]
        agg[hand] = {"d_contact": dc, "d_skel": ds, "settings": len(fr),
                     "flip_rate": float(np.mean(fr)) if fr else np.nan, "e1_rate": float(np.mean(er)) if er else np.nan,
                     "reversed_share": float(np.mean(rv)) if rv else np.nan, "settings_judged": len(rv),
                     "reversed_settings": int(sum(rv)),
                     "E_gt_1_share": float(np.mean(eg)) if eg else np.nan,
                     "tau_mean": float(np.mean(taus)) if taus else np.nan, "tau_sets": len(taus)}
    hs = list(agg)
    dc = np.array([agg[h]["d_contact"] for h in hs])
    ds = np.array([agg[h]["d_skel"] for h in hs])
    st = {}
    for key, sign in (("flip_rate", 1), ("e1_rate", 1), ("reversed_share", 1), ("E_gt_1_share", 1), ("tau_mean", -1)):
        v = np.array([agg[h][key] for h in hs])
        ok = ~np.isnan(v)
        rc, pc = spearman_perm(dc, v, sign)
        rs, ps = spearman_perm(ds, v, sign)
        st[key] = {"rho_d_contact": rc, "p_d_contact": pc, "rho_d_skel": rs, "p_d_skel": ps, "hands": int(ok.sum()),
                   "partial_d_contact_given_d_skel": partial_spearman(v[ok], dc[ok], ds[ok]),
                   "partial_d_skel_given_d_contact": partial_spearman(v[ok], ds[ok], dc[ok])}
    res["per_hand"] = agg
    res["per_hand_stats"] = st
    # post hoc (not pre-registered): only hands scored in at least 10 of the pooled settings (drops d4p5_s0 and
    # d5p5_s0, which fail the grasp tasks and are scored on the fingertip tasks only)
    keep = [i for i, h in enumerate(hs) if agg[h]["settings"] >= 10]
    ph = {}
    for key, sign in (("flip_rate", 1), ("e1_rate", 1), ("E_gt_1_share", 1)):
        v = np.array([agg[hs[i]][key] for i in keep])
        rc, pc = spearman_perm(dc[keep], v, sign)
        rs, ps = spearman_perm(ds[keep], v, sign)
        ph[key] = {"rho_d_contact": rc, "p_d_contact": pc, "rho_d_skel": rs, "p_d_skel": ps, "hands": len(keep)}
    res["per_hand_stats_posthoc_min10"] = ph
    # same, per contact model (descriptive)
    by_model = {}
    for label, pred in (("Drake point", lambda m: m == "Drake point"), ("Drake hydro", lambda m: m == "Drake hydro"),
                        ("Isaac point", lambda m: m == "Isaac point")):
        ss = [s for s in pooled if pred(res["settings"][s]["model"])]
        v = []
        for hand in hs:
            fr = [res["settings"][s]["hands"][hand]["flip_rate"] for s in ss
                  if hand in res["settings"][s]["hands"] and res["settings"][s]["hands"][hand]["pairs"] >= MIN_PAIRS]
            v.append(np.mean(fr) if fr else np.nan)
        rc, pc = spearman_perm(dc, v)
        rs, ps = spearman_perm(ds, v)
        by_model[label] = {"settings": ss, "rho_flip_d_contact": rc, "p": pc, "rho_flip_d_skel": rs, "p_skel": ps}
    res["per_hand_by_model"] = by_model


def examples(res):
    """U-type hand reversed at the hand level while the R-type hand of the same thickness is not, by |D~(O)| / f."""
    ex = []
    for sid, s in res["settings"].items():
        for r_h, u_h in zip(R_HANDS, U_HANDS):
            ru, rr = s["hands"].get(u_h), s["hands"].get(r_h)
            if ru and ru["reversed"] and rr and rr["judged"] and not rr["reversed"]:
                ex.append({"setting": sid, "iface": s["iface"], "model": s["model"], "task": s["task"],
                           "unit": s["unit"], "thickness": thickness(u_h), "floor": s["floor"],
                           "D_O": ru["median_D_O"], "D_U": ru["median_D_H"], "D_R": rr["median_D_H"],
                           "D_O_on_R_pairs": rr["median_D_O"], "U_flip_rate": ru["flip_rate"],
                           "R_flip_rate": rr["flip_rate"], "U_pairs": ru["pairs"], "R_pairs": rr["pairs"],
                           "strength": abs(ru["median_D_O"]) / s["floor"]})
    res["examples"] = sorted(ex, key=lambda e: -e["strength"])


def main():
    res = {"note": __doc__.splitlines()[0], "plan": "docs/experimental/conclusion_flip.md (808365b)",
           "settings": {}, "ranking": {}}
    cache = {}
    for sid, s in SETTINGS.items():
        res["settings"][sid] = run_setting(sid, s, cache)
        v = res["settings"][sid]
        fmt = lambda g: "-" if g is None else (f"{100 * g['flip_rate']:5.1f}/{100 * g['e1_rate']:5.1f}% "  # noqa
                                               f"rev {g['hands_reversed']}/{g['hands_judged']}")
        nz = v["noise"]
        print(f"{sid:17s} f {v['floor']:.3f} | noise "
              f"{'-' if not nz or not nz.get('pairs') else f'{100*nz['flip_rate']:.1f}/{100*nz['e1_rate']:.1f}% ({nz['pairs']})'}"
              f" | R {fmt(v['R'])} | U {fmt(v['U'])} | all {fmt(v['all'])} ({v['all']['pairs'] if v['all'] else 0} pairs)"
              + (f" | rho flip~Dc {v['stats']['rho_flip_rate_d_contact']:+.2f} p {v['stats']['p_flip_rate_d_contact']:.3f}"
                 f" ~Ds {v['stats']['rho_flip_rate_d_skel']:+.2f}" if "stats" in v else ""), flush=True)
    for set_id, sids in RANKING.items():
        ranking(set_id, sids, cache, res)
        rk = res["ranking"][set_id]
        print(set_id, {h: round(r["tau_cells"], 2) for h, r in rk["hands"].items()},
              {k: round(v, 3) for k, v in rk.items() if k.startswith(("rho_", "p_")) and v == v})
    aggregates(res)
    for k, v in res["per_hand_stats"].items():
        print(f"{k:15s} rho Dc {v['rho_d_contact']:+.2f} (p {v['p_d_contact']:.4f}) | Ds {v['rho_d_skel']:+.2f} "
              f"(p {v['p_d_skel']:.4f}) | partial Dc|Ds {v['partial_d_contact_given_d_skel']:+.2f} "
              f"Ds|Dc {v['partial_d_skel_given_d_contact']:+.2f} (n {v['hands']})")
    for k, v in res["per_hand_stats_posthoc_min10"].items():
        print(f"post hoc >= 10 settings: {k:13s} rho Dc {v['rho_d_contact']:+.2f} (p {v['p_d_contact']:.4f}) | "
              f"Ds {v['rho_d_skel']:+.2f} (p {v['p_d_skel']:.4f}) (n {v['hands']})")
    examples(res)
    for e in res["examples"][:8]:
        print(e)
    OUT.write_text(json.dumps(res, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o)))
    print(OUT)


if __name__ == "__main__":
    main()
