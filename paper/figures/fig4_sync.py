"""Fig. 4 data: for the five Drake tasks (power-grasp pull-out e21, grasp acquisition e23, hook-and-pull e38,
fingertip disk e28z, box pivot e35), the per-hand measured friction effect as a share of O's, D(H)/D(O) (100% = in sync
with O), over the same pairs and gates as the tasks' error E (sim/drake/analyze.py error, analyze_roll.py error and
tip_error, hook_pull.py hand_error, box_pivot.py hand_error; median |D(H)/D(O) - 1| is E exactly, asserted here against
report/study3/data), under point contact and, for the hydro labels, hydroelastic contact (experiment 21's rule; e21,
e23h, e38h, e29zh on the first 32 draws, e35h on the first 32 draws). Then O, R and U's object response over every
placement as deviation from O (median, quartiles): pull-out and hook held load from the --curve logs (e21c; the hook
logs carry their curves), acquisition drawn distance (e23c, e23 rerun with a trace every step, bit-identical results),
the disk's rotation (e28zc) and the block's tilt (e35). Writes report/study3/data/fig4_sync.json.

    cd sim/drake && uv run python ../../paper/figures/fig4_sync.py
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DRAKE = ROOT / "sim" / "drake"
sys.path.insert(0, str(DRAKE))
import analyze as A  # noqa: E402
import analyze_roll as AR  # noqa: E402
import box_pivot as BP  # noqa: E402
import hook_pull as HP  # noqa: E402

MU = (1.3, 2.0)
LOG = DRAKE / "logs"
S3 = ROOT / "report" / "study3" / "data"


def pull_pairs(hand, contact="point"):
    out = {}
    for r in A.RADII:
        by = {(h, m): {A.key(e): e for e in A.load_drake(h, m, contact, "e21", r)} for h in ("O", hand) for m in MU}
        for k in by[("O", MU[0])]:
            es = [by[hm].get(k) for hm in by]
            if any(e is None or not e["gate0"]["pass"] or e["limit"]["f_limit_n"] is None for e in es):
                continue
            o_lo, o_hi, h_lo, h_hi = (by[hm][k]["limit"]["f_limit_n"] for hm in
                                      (("O", 1.3), ("O", 2.0), (hand, 1.3), (hand, 2.0)))
            if abs(o_hi - o_lo) < 1e-9:
                continue
            out[k] = (h_hi - h_lo) / (o_hi - o_lo)
    return out


def acquire_pairs(hand, contact="point", tag="e23"):
    """Experiment 23 over analyze_roll.error's rules: O holds at both mu, |D(O)| >= 0.3 mm, H holds at both mu."""
    AR.TASK = "pullin"
    _, _, variants, rk, metric, min_effect, _ = AR.TASKS["pullin"]
    out = {}
    for s in variants:
        for r in AR.RADII:
            runs = {(h, m): AR.load(h, m, contact, tag, s, r) for h in ("O", hand) for m in MU}
            for k in runs[("O", MU[0])]:
                es = {hm: runs[hm].get(k) for hm in runs}
                if any(e is None or not e["gate0"]["pass"] or e.get(rk) is None for e in es.values()):
                    continue
                if not all(es[("O", m)][rk]["held"] for m in MU):
                    continue
                d_o = es[("O", MU[1])][rk][metric] - es[("O", MU[0])][rk][metric]
                if abs(d_o) < min_effect:
                    continue
                if not all(es[(hand, m)][rk]["held"] for m in MU):
                    continue
                out[(s, r) + k] = (es[(hand, MU[1])][rk][metric] - es[(hand, MU[0])][rk][metric]) / d_o
    return out


def hook_pairs(hand, tag="e38"):
    """Experiment 38 over hook_pull.hand_error: O's kept pairs; H's solver failures, Gate 0 failures, pad-force
    (penetration) gate and unlimited pairs dropped."""
    keep, _ = HP.o_keep(tag)
    h = HP._load(hand, tag)
    out = {}
    for k, d_o in keep.items():
        r = h.get(k[:2])
        if r is None or r.get("solver_fail"):
            continue
        if not all(r["runs"][f"{mu:g}"]["gate0"]["pass"] for mu in MU):
            continue
        if max(r["runs"][f"{mu:g}"]["pre"]["pen_mm"] for mu in MU) > HP.PEN_MM:
            continue
        f = [HP._flim(r, mu, k[2]) for mu in MU]
        if any(x is None for x in f):
            continue
        out[k] = (f[1] - f[0]) / d_o
    return out


def dial_pairs(hand, contact="point", tag="e28z", random=False):
    AR.TASK, AR.RANDOM = "tipdial_z", random
    out = {}
    for r in (("any",) if random else (15.0,)):
        o = {m: AR.tip_load("O", m, contact, tag, r) for m in MU}
        hh = {m: AR.tip_load(hand, m, contact, tag, r) for m in MU}
        for k in o[MU[0]]:
            os_ = [o[m][k] for m in MU]
            if not all(d["touched"] for d in os_):
                continue
            if any(d["distal_force_max_n"] / AR.POINT_K > AR.TIP_PEN_MM or d["proximal_force_max_n"] > AR.PROX_TOUCH_N
                   for d in os_):
                continue
            d_o = os_[1]["rotation_deg"] - os_[0]["rotation_deg"]
            if abs(d_o) < 0.3:
                continue
            hs = [hh[m].get(k) for m in MU]
            if any(d is None or not d["touched"] for d in hs):
                continue
            if max(d["distal_force_max_n"] for d in hs) / AR.POINT_K > AR.TIP_PEN_MM:
                continue
            out[k] = (hs[1]["rotation_deg"] - hs[0]["rotation_deg"]) / d_o
    AR.RANDOM = False
    return out


def pivot_pairs(hand, tag="e35"):
    """Experiment 35 over E's gates (box_pivot.hand_error): O's kept draws; the hand's solver failures, lost and
    over-pressed draws dropped; the end tilt."""
    keep, _ = BP.o_keep(tag)
    h = BP._load(hand, tag)
    out = {}
    for k, o in keep.items():
        runs = [h[k][f"{m:g}"] for m in BP.MU]
        if any("solver_fail" in x for x in runs) or any(BP.lost(x) for x in runs):
            continue
        if max(x["pen_mm"] for x in runs) > BP.PEN_MM:
            continue
        out[k] = (runs[1]["tilt_deg"] - runs[0]["tilt_deg"]) / (o[1]["tilt_deg"] - o[0]["tilt_deg"])
    return out


def _rows(path):
    d = json.load(open(path))
    rows = d if isinstance(d, list) else d["rows"]
    return {r["hand"]: r["E"] for r in rows}


# task: (point pairs, point E reference, hydro pairs, hydro E reference)
TASKS = {
    "pull": (pull_pairs, S3 / "drake_e21_point.json", lambda h: pull_pairs(h, "hydro"), S3 / "drake_e21_hydro.json"),
    "acquire": (acquire_pairs, S3 / "drake_e23_point.json", lambda h: acquire_pairs(h, "hydro", "e23h"),
                S3 / "drake_e23_hydro.json"),
    "hook": (hook_pairs, S3 / "drake_e38_point.json", lambda h: hook_pairs(h, "e38h"), S3 / "drake_e38_hydro.json"),
    "dial": (dial_pairs, S3 / "drake_e28z_point.json", lambda h: dial_pairs(h, "hydro", "e29zh", True),
             S3 / "drake_e29z_hydro.json"),
    "pivot": (pivot_pairs, S3 / "drake_e35_pivot.json", lambda h: pivot_pairs(h, "e35h"), S3 / "drake_e35_hydro.json"),
}


def _band(ys):
    y = np.asarray(ys)
    return {"median": np.nanmedian(y, 0).round(4).tolist(), "q25": np.nanpercentile(y, 25, 0).round(4).tolist(),
            "q75": np.nanpercentile(y, 75, 0).round(4).tolist(), "n": int(len(y))}


def _held(c, grid):
    """The load held up to each displacement (the limit once the grasp gives way), from [load, displacement] rows."""
    c = np.asarray(c)
    d = np.maximum.accumulate(np.maximum(c[:, 1], 0.0))
    return np.interp(grid, d, c[:, 0], right=c[-1, 0])


def _trajectories(series, keys_r, keys_u, n_cut=None):
    """series(h, mu, key) -> array. R and U minus O at each friction over their own E pairs; each hand's friction effect
    (2.0 minus 1.3) and absolute response over the pairs R and U share; R and U's effect minus O's as a share of O's
    median final effect (%)."""
    cut = (lambda y: y[:n_cut]) if n_cut else (lambda y: y)
    dv = {}
    for h, ks in (("R", keys_r), ("U", keys_u)):
        for m in MU:
            dv[f"{h}_{m:g}"] = _band([cut(series(h, m, k) - series("O", m, k)) for k in ks])
    pk = sorted(set(keys_r) & set(keys_u), key=str)
    eff = {h: {k: cut(series(h, 2.0, k) - series(h, 1.3, k)) for k in pk} for h in "ORU"}
    for h in "ORU":
        dv[f"{h}_effect"] = _band(list(eff[h].values()))
        for m in MU:
            dv[f"{h}_{m:g}_abs"] = _band([cut(series(h, m, k)) for k in pk])
    sc = dv["O_effect"]["median"][-1]
    for h in "RU":
        dv[f"{h}_bias"] = _band([100 * (eff[h][k] - eff["O"][k]) / sc for k in pk])
    dv["pairs_shared"] = len(pk)
    return dv


def deviation(pairs):
    dev = {}
    # pull-out: held load against cylinder displacement (e21c curves, Gate 0 passed), over the E pairs
    grid = np.linspace(0.0, 5.0, 101)
    cur = {}
    for h in "ORU":
        for m in MU:
            for r in A.RADII:
                for e in json.load(open(LOG / f"dk_{h}_mu{m:g}_point-e21c_r{r:g}.json"))["envs"]:
                    if e["gate0"]["pass"] and e.get("curve"):
                        cur[(h, m, A.key(e))] = _held([[x[0], x[2]] for x in e["curve"]], grid)
    ok = lambda h, ks: [k for k in ks if all((hh, m, k) in cur for hh in ("O", h) for m in MU)]
    dev["pull"] = _trajectories(lambda h, m, k: cur[(h, m, k)], ok("R", pairs["pull"]["R"]), ok("U", pairs["pull"]["U"]))
    dev["pull"]["x"] = grid.round(3).tolist()
    # acquisition: distance drawn toward the MCP against time (e23c)
    tr = {}
    for h in "ORU":
        for m in MU:
            for s in (3, 6):
                for r in AR.RADII:
                    p = LOG / f"dkpullin_{h}_mu{m:g}_point-e23c-a{s}_r{r:g}.json"
                    for e in json.load(open(p))["envs"]:
                        tr[(h, m, (s, r, e["q0_deg"], round(e["x_c_mm"], 3)))] = np.array(e["pullin"]["trace"])[:, 1]
    dev["acquire"] = _trajectories(lambda h, m, k: tr[(h, m, k)], list(pairs["acquire"]["R"]),
                                   list(pairs["acquire"]["U"]))
    dev["acquire"]["x"] = np.array(json.load(open(LOG / "dkpullin_O_mu1.3_point-e23c-a3_r15.json"))["envs"][0]
                                   ["pullin"]["trace"])[:, 0].round(4).tolist()
    # hook: held load against bar displacement (curves in the logs), over the E pairs
    hk = {h: HP._load(h, "e38") for h in "ORU"}
    dev["hook"] = _trajectories(lambda h, m, k: _held(hk[h][k[:2]]["runs"][f"{m:g}"]["dirs"][k[2]]["curve"], grid),
                                list(pairs["hook"]["R"]), list(pairs["hook"]["U"]))
    dev["hook"]["x"] = grid.round(3).tolist()
    # fingertip disk: rotation against time (e28zc, every step), over the E pairs
    dz = {}
    for h in "ORU":
        for m in MU:
            for e in json.load(open(LOG / f"dktipdial_{h}_mu{m:g}_point-e28zc_r15.json"))["envs"]:
                k = (e["rho_mm"], e["q0_deg"], round(e["x_c_mm"], 3), e["placement"].get("y_mm", 0.0))
                dz[(h, m, k)] = np.array(e["dial"]["trace"])[:, 1]
    dev["dial"] = _trajectories(lambda h, m, k: dz[(h, m, k)], list(pairs["dial"]["R"]), list(pairs["dial"]["U"]))
    n_d = len(next(iter(dz.values())))
    dev["dial"]["x"] = (np.arange(1, n_d + 1) / 30).round(4).tolist()
    # box pivot: tilt from the sweep's start against time (e35), over the E pairs
    bp = {h: BP._load(h, "e35") for h in "ORU"}
    n_p = min(len(bp[h][k][f"{m:g}"]["trace"]) for h in "ORU" for k in pairs["pivot"]["R"] for m in MU)
    dev["pivot"] = _trajectories(lambda h, m, k: np.array(bp[h][k][f"{m:g}"]["trace"])[:, 0],
                                 list(pairs["pivot"]["R"]), list(pairs["pivot"]["U"]), n_cut=n_p)
    dev["pivot"]["x"] = (np.arange(1, n_p + 1) / 30).round(4).tolist()
    return dev


def main():
    out = {"note": __doc__.splitlines()[0], "tasks": {}, "hydro": {}}
    pairs = {}
    for task, (fn, ref_p, fn_h, ref_h) in TASKS.items():
        hands = {}
        pairs[task] = {}
        for label, f, ref in (("point", fn, _rows(ref_p)), ("hydro", fn_h, _rows(ref_h))):
            for hand, (dc, ds) in A.DESIGN.items():
                try:
                    pr = f(hand)
                except (FileNotFoundError, TypeError, AttributeError):
                    continue
                if not pr:
                    continue
                v = np.array(list(pr.values()))
                e = float(np.median(np.abs(v - 1)))
                if hand in ref and ref[hand] is not None:
                    assert abs(e - ref[hand]) < 1e-9, (task, label, hand, e, ref[hand])
                row = {"d_contact": dc, "d_skel": ds, "E": e, "ratio_median": float(np.median(v)),
                       "bias_median": float(np.median(v - 1)),
                       "bias_q25": float(np.percentile(v - 1, 25)), "bias_q75": float(np.percentile(v - 1, 75)),
                       "pairs": len(v)}
                if label == "point":
                    hands[hand] = row
                    if hand in "RU":
                        pairs[task][hand] = pr
                else:
                    out["hydro"].setdefault(task, {})[hand] = row
        out["tasks"][task] = {"hands": hands}
        hy = out["hydro"].get(task, {})
        print(task, len(hands), "hands; point E R", round(hands["R"]["E"], 3), "U", round(hands["U"]["E"], 3),
              "| hydro R", hy.get("R", {}).get("bias_median"), "U", hy.get("U", {}).get("bias_median"))
    out["deviation"] = deviation(pairs)
    (S3 / "fig4_sync.json").write_text(json.dumps(out, indent=1))
    print(S3 / "fig4_sync.json")


if __name__ == "__main__":
    main()
