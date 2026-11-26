"""Attempt 8 (amended) judgement (``docs/experimental/rl_pip_only.md``): per task, observation level and training seed
the 5/7/9 mm E(R), E(U), G = E(U) - E(R), eval noise and transfer-ratio gap; per level the seed-mean G with noise
|G_20 - G_21| + mean eval noise; the monotone-trend verdict over levels per thickness and over the thickness mean.

    python3 scripts/finger_relay/attempt8_summary.py relay|walk
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

SIM = Path(__file__).resolve().parents[2]
METRIC = {"relay": "rotation_deg_mean", "walk": "walked_mm_mean"}
LEVELS = {"relay": ["", "ObsS", "ObsL", "Touch"], "walk": ["", "ObsS", "ObsL", "TouchDir"]}
NAMES = {"": "exact", "ObsS": "small noise", "ObsL": "large noise", "Touch": "touch only", "TouchDir": "touch+direction"}
PAIRS = {"5": ("R5", "U5"), "7": ("R", "U"), "9": ("R9", "U9")}
EVAL = (94000, 95000)


def one(run: Path, key: str, tag: str):
    def m(h, mu, s):
        p = run / f"transfer{tag}_{h}_mu{mu}_seed{s}.json"
        return json.loads(p.read_text())[key] if p.exists() else None

    def hand(h):
        v = {mu: [m(h, mu, s) for s in EVAL] for mu in ("1.3", "2.0")}
        if None in v["1.3"] + v["2.0"]:
            return None
        d = [v["2.0"][i] - v["1.3"][i] for i in range(2)]
        return {"m13": v["1.3"], "d": sum(d) / 2, "dd": abs(d[0] - d[1])}

    o = hand("O")
    if o is None:
        return None
    out = {}
    for t, (r_, u_) in PAIRS.items():
        hr, hu = hand(r_), hand(u_)
        if hr is None or hu is None:
            continue
        do = abs(o["d"])
        er, eu = abs(hr["d"] - o["d"]) / do, abs(hu["d"] - o["d"]) / do
        rr = sum(hr["m13"][i] / o["m13"][i] for i in range(2)) / 2
        ru = sum(hu["m13"][i] / o["m13"][i] for i in range(2)) / 2
        out[t] = {"E_R": er, "E_U": eu, "G": eu - er, "eval_noise": (hr["dd"] + hu["dd"]) / do, "ratio_gap": rr - ru,
                  # exploratory, post hoc (not the verdict): O's swap effect and the unnormalised distances
                  "delta_O": o["d"], "abs_R": abs(hr["d"] - o["d"]), "abs_U": abs(hu["d"] - o["d"]),
                  "ill_conditioned": abs(o["d"]) < 10.0}
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("task", choices=("relay", "walk"))
    a = ap.parse_args()
    run = SIM / "logs" / f"finger_{a.task}" / "run"
    res = {}
    for lv in LEVELS[a.task]:
        seeds = {s: one(run, METRIC[a.task], lv + ("s21" if s == 21 else "")) for s in (20, 21)}
        per = {}
        for t in PAIRS:
            vals = {s: v[t] for s, v in seeds.items() if v and t in v}
            if not vals:
                continue
            g = [v["G"] for v in vals.values()]
            noise = (abs(g[0] - g[1]) if len(g) == 2 else float("nan")) + sum(v["eval_noise"] for v in vals.values()) / len(vals)
            per[t] = {"seeds": vals, "G_mean": sum(g) / len(g), "noise": noise,
                      "ratio_gap_mean": sum(v["ratio_gap"] for v in vals.values()) / len(vals), "n_seeds": len(g)}
        if len(per) == 3:
            per["mean"] = {"G_mean": sum(per[t]["G_mean"] for t in PAIRS) / 3,
                           "noise": sum(per[t]["noise"] for t in PAIRS) / 3,
                           "ratio_gap_mean": sum(per[t]["ratio_gap_mean"] for t in PAIRS) / 3,
                           "n_seeds": min(per[t]["n_seeds"] for t in PAIRS)}
        res[lv] = per
    print(f"{a.task}: G = E(U) - E(R) (seed mean +- noise), ratio gap R - U")
    for t in (*PAIRS, "mean"):
        row = []
        for lv in LEVELS[a.task]:
            x = res.get(lv, {}).get(t)
            row.append(f"{NAMES[lv]}: " + (f"G {x['G_mean']:+.3f} +-{x['noise']:.3f} (n={x['n_seeds']}) ratio {x['ratio_gap_mean']:+.3f}"
                                            if x else "-"))
        print(f" {t:>4}: " + " | ".join(row))
        xs = [res.get(lv, {}).get(t) for lv in LEVELS[a.task]]
        if all(xs):
            drops = [(i, xs[i]["G_mean"] - xs[i + 1]["G_mean"], xs[i]["noise"] + xs[i + 1]["noise"]) for i in range(3)]
            bad = [d for d in drops if d[1] > d[2]]
            print(f"       monotone verdict: {'refuted (drop beyond noise at ' + str([NAMES[LEVELS[a.task][i]] + '->' + NAMES[LEVELS[a.task][i + 1]] for i, _, _ in bad]) + ')' if bad else 'not refuted'}"
                  f"; strictly increasing means: {all(xs[i]['G_mean'] < xs[i + 1]['G_mean'] for i in range(3))}")
    print("exploratory (post hoc, not the verdict): Delta(O) per seed; |D(H)-D(O)| R / U per thickness; * = |Delta(O)| < 10")
    for lv in LEVELS[a.task]:
        for t in PAIRS:
            x = res.get(lv, {}).get(t)
            if x:
                print(f"  {NAMES[lv]:>15} {t} mm: " + "  ".join(
                    f"s{k}: dO {v['delta_O']:+.1f}{'*' if v['ill_conditioned'] else ''} |D| {v['abs_R']:.1f}/{v['abs_U']:.1f}"
                    for k, v in x["seeds"].items()))
    (run / "attempt8_summary.json").write_text(json.dumps(res, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
