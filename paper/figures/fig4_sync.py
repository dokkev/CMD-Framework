"""Fig. 4 data: for the four Drake tasks (pull-out e21 point, fingertip dial e28z, coin drag e30b, box pivot e35), the per-hand
measured friction effect as a share of O's, D(H)/D(O) (100% = in sync with O), over the same pairs and gates as the
tasks' error E (sim/drake/analyze.py, analyze_roll.py tip_error, coin_drag.py hand_error, box_pivot.py hand_error;
median |D(H)/D(O) - 1| is
E exactly, checked here), and O, R, U's object response for one representative placement per task: the placement
whose R and U ratios are nearest their medians. Pull-out curves come from the --curve rerun (tag e21c, the same
simulation as e21), the dial's from e28zc (e28z with a trace every step). Writes report/study3/data/fig4_sync.json.

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
import coin_drag as CD  # noqa: E402
import box_pivot as BP  # noqa: E402

MU = (1.3, 2.0)
LOG = DRAKE / "logs"


def pull_pairs(hand):
    out = {}
    for r in A.RADII:
        by = {(h, m): {A.key(e): e for e in A.load_drake(h, m, "point", "e21", r)} for h in ("O", hand) for m in MU}
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


def dial_pairs(hand):
    AR.TASK = "tipdial_z"
    out = {}
    o = {m: AR.tip_load("O", m, "point", "e28z", 15.0) for m in MU}
    hh = {m: AR.tip_load(hand, m, "point", "e28z", 15.0) for m in MU}
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
    return out


def coin_pairs(hand):
    keep, _, _ = CD.o_pairs("e30b")
    hh = {m: CD.load(hand, m, "e30b") for m in CD.MU}
    out = {}
    for k, d_o in keep.items():
        hs = [hh[m].get(k) for m in CD.MU]
        if any(d is None or not d["touched"] for d in hs):
            continue
        if max(d["distal_force_max_n"] for d in hs) / CD.POINT_K > CD.PEN_MM:
            continue
        out[k] = (hs[1][CD.METRIC] - hs[0][CD.METRIC]) / d_o
    return out


def pivot_pairs(hand):
    """Experiment 35 over E's gates (box_pivot.hand_error): O's kept draws; the hand's solver failures, lost and
    over-pressed draws dropped; the end tilt."""
    keep, _ = BP.o_keep("e35")
    h = BP._load(hand, "e35")
    out = {}
    for k, o in keep.items():
        runs = [h[k][f"{m:g}"] for m in BP.MU]
        if any("solver_fail" in x for x in runs) or any(BP.lost(x) for x in runs):
            continue
        if max(x["pen_mm"] for x in runs) > BP.PEN_MM:
            continue
        out[k] = (runs[1]["tilt_deg"] - runs[0]["tilt_deg"]) / (o[1]["tilt_deg"] - o[0]["tilt_deg"])
    return out


def _band(ys):
    y = np.asarray(ys)
    return {"median": np.nanmedian(y, 0).round(4).tolist(), "q25": np.nanpercentile(y, 25, 0).round(4).tolist(),
            "q75": np.nanpercentile(y, 75, 0).round(4).tolist(), "n": int(len(y))}


def deviation():
    """R and U minus O over every placement both run: pull-out held load (max load sustained up to each cylinder
    displacement; the limit once the grasp gives way) on a displacement grid, dial rotation and coin travel in
    time; median and quartiles."""
    dgrid = np.linspace(0.0, 5.0, 101)
    dev = {"pull": {"x": dgrid.round(3).tolist()}, "dial": {}, "coin": {}, "pivot": {}}

    def held(c):
        d = np.maximum.accumulate(c[:, 2])
        return np.interp(dgrid, d, c[:, 0], right=c[-1, 0])

    for m in MU:
        def pe(h):
            o_ = {}
            for r in A.RADII:
                for e in json.load(open(LOG / f"dk_{h}_mu{m:g}_point-e21c_r{r:g}.json"))["envs"]:
                    if e["gate0"]["pass"] and e.get("curve"):
                        o_[A.key(e)] = np.array([x[:3] for x in e["curve"]])
            return o_
        po = pe("O")
        do = json.load(open(LOG / f"dktipdial_O_mu{m:g}_point-e28zc_r15.json"))["envs"]
        co = json.load(open(LOG / f"dkcoin_O_mu{m:g}_point-e30b.json"))["envs"]
        for h in ("R", "U"):
            ph = pe(h)
            dev["pull"][f"{h}_{m:g}"] = _band([held(ph[k]) - held(po[k]) for k in po if k in ph])
            dh = json.load(open(LOG / f"dktipdial_{h}_mu{m:g}_point-e28zc_r15.json"))["envs"]
            dev["dial"][f"{h}_{m:g}"] = _band([np.array(b["dial"]["trace"])[:, 1] - np.array(a["dial"]["trace"])[:, 1]
                                               for a, b in zip(do, dh)])
            ch = json.load(open(LOG / f"dkcoin_{h}_mu{m:g}_point-e30b.json"))["envs"]
            dev["coin"][f"{h}_{m:g}"] = _band([np.array(b["drag"]["trace"])[:, 2] - np.array(a["drag"]["trace"])[:, 2]
                                               for a, b in zip(co, ch)
                                               if len(a["drag"]["trace"]) == 30 == len(b["drag"]["trace"])])
    # pivot: the block's tilt from the sweep's start, every step over the sweep and hold, on E's draws
    for h in ("R", "U"):
        pr = pivot_pairs(h)
        o_, h_ = BP._load("O", "e35"), BP._load(h, "e35")
        for m in MU:
            ys = [np.array(h_[k][f"{m:g}"]["trace"])[:, 0] - np.array(o_[k][f"{m:g}"]["trace"])[:, 0] for k in pr]
            n_p = min(map(len, ys))
            dev["pivot"][f"{h}_{m:g}"] = _band([y[:n_p] for y in ys])
    dev["pivot"]["x"] = (np.arange(1, n_p + 1) / 30).round(4).tolist()
    # each hand's friction effect (below); O's is the reference
    def pe_o(m, h="O"):
        o_ = {}
        for r in A.RADII:
            for e in json.load(open(LOG / f"dk_{h}_mu{m:g}_point-e21c_r{r:g}.json"))["envs"]:
                if e["gate0"]["pass"] and e.get("curve"):
                    o_[A.key(e)] = np.array([x[:3] for x in e["curve"]])
        return o_
    # each hand's own friction effect, 2.0 minus 1.3 on that hand, over the same placements: O (the reference), R, U
    pk = sorted(set(pivot_pairs("R")) & set(pivot_pairs("U")), key=str)
    keys_p = None
    for h in ("O", "R", "U"):
        p1, p2 = pe_o(1.3, h), pe_o(2.0, h)
        keys_p = set(p1) & set(p2) if keys_p is None else keys_p & set(p1) & set(p2)
    for h in ("O", "R", "U"):
        p1, p2 = pe_o(1.3, h), pe_o(2.0, h)
        dev["pull"][f"{h}_effect"] = _band([held(p2[k]) - held(p1[k]) for k in sorted(keys_p)])
        for m_, pp in ((1.3, p1), (2.0, p2)):  # the absolute response, same placements
            dev["pull"][f"{h}_{m_:g}_abs"] = _band([held(pp[k]) for k in sorted(keys_p)])
        d1, d2 = (json.load(open(LOG / f"dktipdial_{h}_mu{m:g}_point-e28zc_r15.json"))["envs"] for m in MU)
        dev["dial"][f"{h}_effect"] = _band([np.array(b["dial"]["trace"])[:, 1] - np.array(a["dial"]["trace"])[:, 1]
                                            for a, b in zip(d1, d2)])
        for m_, dd in ((1.3, d1), (2.0, d2)):
            dev["dial"][f"{h}_{m_:g}_abs"] = _band([np.array(a["dial"]["trace"])[:, 1] for a in dd])
        c1, c2 = (json.load(open(LOG / f"dkcoin_{h}_mu{m:g}_point-e30b.json"))["envs"] for m in MU)
        dev["coin"][f"{h}_effect"] = _band([np.array(b["drag"]["trace"])[:, 2] - np.array(a["drag"]["trace"])[:, 2]
                                            for a, b in zip(c1, c2)
                                            if len(a["drag"]["trace"]) == 30 == len(b["drag"]["trace"])])
        for m_, cc in ((1.3, c1), (2.0, c2)):
            dev["coin"][f"{h}_{m_:g}_abs"] = _band([np.array(a["drag"]["trace"])[:, 2] for a in cc
                                                    if len(a["drag"]["trace"]) == 30])
        o_ = BP._load(h, "e35")
        ys = [np.array(o_[k]["2"]["trace"])[:, 0] - np.array(o_[k]["1.3"]["trace"])[:, 0] for k in pk]
        dev["pivot"][f"{h}_effect"] = _band([y[:n_p] for y in ys])
        for m_ in ("1.3", "2"):
            dev["pivot"][f"{h}_{m_}_abs"] = _band([np.array(o_[k][m_]["trace"])[:n_p, 0] for k in pk])
    # the bias in the friction effect: H's effect minus O's on the same placement, as a share of O's median final
    # effect (0: the friction effect measured as on O)
    p1o, p2o = pe_o(1.3, "O"), pe_o(2.0, "O")
    eff_o_p = {k: held(p2o[k]) - held(p1o[k]) for k in keys_p}
    d1o, d2o = (json.load(open(LOG / f"dktipdial_O_mu{m:g}_point-e28zc_r15.json"))["envs"] for m in MU)
    eff_o_d = [np.array(b["dial"]["trace"])[:, 1] - np.array(a["dial"]["trace"])[:, 1] for a, b in zip(d1o, d2o)]
    c1o, c2o = (json.load(open(LOG / f"dkcoin_O_mu{m:g}_point-e30b.json"))["envs"] for m in MU)
    oo = BP._load("O", "e35")
    eff_o_n = {k: (np.array(oo[k]["2"]["trace"])[:, 0] - np.array(oo[k]["1.3"]["trace"])[:, 0])[:n_p] for k in pk}
    for h in ("R", "U"):
        p1, p2 = pe_o(1.3, h), pe_o(2.0, h)
        sc = dev["pull"]["O_effect"]["median"][-1]
        dev["pull"][f"{h}_bias"] = _band([100 * (held(p2[k]) - held(p1[k]) - eff_o_p[k]) / sc for k in sorted(keys_p)])
        d1, d2 = (json.load(open(LOG / f"dktipdial_{h}_mu{m:g}_point-e28zc_r15.json"))["envs"] for m in MU)
        sc = dev["dial"]["O_effect"]["median"][-1]
        dev["dial"][f"{h}_bias"] = _band([100 * (np.array(b["dial"]["trace"])[:, 1] - np.array(a["dial"]["trace"])[:, 1]
                                                 - eo) / sc for a, b, eo in zip(d1, d2, eff_o_d)])
        c1, c2 = (json.load(open(LOG / f"dkcoin_{h}_mu{m:g}_point-e30b.json"))["envs"] for m in MU)
        sc = dev["coin"]["O_effect"]["median"][-1]
        ys = []
        for a, b, ao, bo in zip(c1, c2, c1o, c2o):
            tr = [np.array(z["drag"]["trace"]) for z in (a, b, ao, bo)]
            if all(len(t_) == 30 for t_ in tr):
                ys.append(100 * ((tr[1][:, 2] - tr[0][:, 2]) - (tr[3][:, 2] - tr[2][:, 2])) / sc)
        dev["coin"][f"{h}_bias"] = _band(ys)
        hh_ = BP._load(h, "e35")
        sc = dev["pivot"]["O_effect"]["median"][-1]
        dev["pivot"][f"{h}_bias"] = _band([100 * ((np.array(hh_[k]["2"]["trace"])[:, 0]
                                                   - np.array(hh_[k]["1.3"]["trace"])[:, 0])[:n_p] - eff_o_n[k]) / sc
                                           for k in pk])
    dev["dial"]["x"] = (np.arange(1, len(do[0]["dial"]["trace"]) + 1) / 30).round(4).tolist()
    dev["coin"]["x"] = [round(0.2 * (i + 1), 3) for i in range(30)]
    return dev


def main():
    S3 = ROOT / "report" / "study3" / "data"
    ref = {"pull": {r["hand"]: r["E"] for r in json.load(open(S3 / "drake_e21_point.json"))},
           "dial": {r["hand"]: r["E"] for r in json.load(open(S3 / "drake_e28z_point.json"))["rows"]},
           "coin": {r["hand"]: r["E"] for r in json.load(open(S3 / "drake_e30b_coin.json"))["rows"]},
           "pivot": {r["hand"]: r["E"] for r in json.load(open(S3 / "drake_e35_pivot.json"))["rows"]}}
    fns = {"pull": pull_pairs, "dial": dial_pairs, "coin": coin_pairs, "pivot": pivot_pairs}
    out = {"note": __doc__.splitlines()[0], "tasks": {}}
    for task, fn in fns.items():
        hands, per = {}, {}
        for hand, (dc, ds) in A.DESIGN.items():
            try:
                pr = fn(hand)
            except (FileNotFoundError, TypeError, AttributeError):
                continue
            if not pr:
                continue
            v = np.array(list(pr.values()))
            e = float(np.median(np.abs(v - 1)))
            if hand in ref[task] and ref[task][hand] is not None:
                assert abs(e - ref[task][hand]) < 1e-9, (task, hand, e, ref[task][hand])
            hands[hand] = {"d_contact": dc, "d_skel": ds, "ratio_median": float(np.median(v)),
                           "ratio_q25": float(np.percentile(v, 25)), "ratio_q75": float(np.percentile(v, 75)),
                           "E": e, "pairs": len(v)}
            per[hand] = pr
        # representative placement: shared by R and U, ratios nearest their medians (rank sum)
        ks = sorted(set(per["R"]) & set(per["U"]), key=str)
        score = {k: sum(abs(per[h][k] - hands[h]["ratio_median"]) / (abs(hands[h]["ratio_median"]) + 1e-9)
                        for h in ("R", "U")) for k in ks}
        rep = min(ks, key=lambda k: score[k])
        out["tasks"][task] = {"hands": hands, "representative": list(rep) if isinstance(rep, tuple) else rep,
                              "rep_ratio": {h: per[h][rep] for h in ("R", "U")}}
        print(task, len(hands), "hands; R", round(hands["R"]["ratio_median"], 3), "U", round(hands["U"]["ratio_median"], 3),
              "rep", rep, {h: round(per[h][rep], 3) for h in ("R", "U")})
    # the representative curves
    curves = {}
    t = out["tasks"]
    rp = tuple(t["pull"]["representative"])
    for h in "ORU":
        for m in MU:
            for e in json.load(open(LOG / f"dk_{h}_mu{m:g}_point-e21c_r{rp[0]:g}.json"))["envs"]:
                if A.key(e) == rp:
                    c = np.array([x[:3] for x in e["curve"]])
                    curves[f"pull_{h}_{m:g}"] = {"load_n": c[:, 0].round(3).tolist(),
                                                 "disp_mm": np.maximum.accumulate(c[:, 2]).round(4).tolist(),
                                                 "f_limit_n": e["limit"]["f_limit_n"]}
    rd = tuple(t["dial"]["representative"])
    for h in "ORU":
        for m in MU:
            for e in json.load(open(LOG / f"dktipdial_{h}_mu{m:g}_point-e28zc_r15.json"))["envs"]:
                if (e["rho_mm"], e["q0_deg"], round(e["x_c_mm"], 3), e["placement"].get("y_mm", 0.0)) == rd:
                    tr = np.array(e["dial"]["trace"])
                    curves[f"dial_{h}_{m:g}"] = {"t_s": (np.arange(1, len(tr) + 1) / 30).round(4).tolist(),
                                                 "rotation_deg": tr[:, 1].tolist()}
    rc = t["coin"]["representative"]
    for h in "ORU":
        for m in MU:
            d = CD.load(h, m, "e30b")[rc]
            tr = np.array(d["trace"])
            curves[f"coin_{h}_{m:g}"] = {"t_s": tr[:, 0].tolist(), "travel_mm": tr[:, 2].tolist()}
    rk = t["pivot"]["representative"]
    for h in "ORU":
        d = BP._load(h, "e35")[rk]
        for m in MU:
            tr = np.array(d[f"{m:g}"]["trace"])
            curves[f"pivot_{h}_{m:g}"] = {"t_s": (np.arange(1, len(tr) + 1) / 30).round(4).tolist(),
                                          "tilt_deg": tr[:, 0].tolist()}
    out["curves"] = curves
    out["deviation"] = deviation()
    (S3 / "fig4_sync.json").write_text(json.dumps(out, indent=1))
    print(S3 / "fig4_sync.json")


if __name__ == "__main__":
    main()
