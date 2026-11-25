"""Fig. 4 data: for the three Drake tasks (pull-out e21 point, fingertip dial e28z, coin drag e30b), the per-hand
measured friction effect as a share of O's, D(H)/D(O) (100% = in sync with O), over the same pairs and gates as the
tasks' error E (sim/drake/analyze.py, analyze_roll.py tip_error, coin_drag.py hand_error; median |D(H)/D(O) - 1| is
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


def _band(ys):
    y = np.asarray(ys)
    return {"median": np.nanmedian(y, 0).round(4).tolist(), "q25": np.nanpercentile(y, 25, 0).round(4).tolist(),
            "q75": np.nanpercentile(y, 75, 0).round(4).tolist(), "n": int(len(y))}


def deviation():
    """R and U minus O over every placement both run: pull-out held load (max load sustained up to each cylinder
    displacement; the limit once the grasp gives way) on a displacement grid, dial rotation and coin travel in
    time; median and quartiles."""
    dgrid = np.linspace(0.0, 5.0, 101)
    dev = {"pull": {"x": dgrid.round(3).tolist()}, "dial": {}, "coin": {}}

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
    dev["dial"]["x"] = (np.arange(1, len(do[0]["dial"]["trace"]) + 1) / 30).round(4).tolist()
    dev["coin"]["x"] = [round(0.2 * (i + 1), 3) for i in range(30)]
    return dev


def main():
    S3 = ROOT / "report" / "study3" / "data"
    ref = {"pull": {r["hand"]: r["E"] for r in json.load(open(S3 / "drake_e21_point.json"))},
           "dial": {r["hand"]: r["E"] for r in json.load(open(S3 / "drake_e28z_point.json"))["rows"]},
           "coin": {r["hand"]: r["E"] for r in json.load(open(S3 / "drake_e30b_coin.json"))["rows"]}}
    fns = {"pull": pull_pairs, "dial": dial_pairs, "coin": coin_pairs}
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
    out["curves"] = curves
    out["deviation"] = deviation()
    (S3 / "fig4_sync.json").write_text(json.dumps(out, indent=1))
    print(S3 / "fig4_sync.json")


if __name__ == "__main__":
    main()
