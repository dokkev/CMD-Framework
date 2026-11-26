"""Attempt 7a'/7b' judgement (``docs/experimental/rl_pip_only.md``): per hand the metric at pad mu 1.3 and 2.0 (two eval
seeds), the swap effect Delta = m(2.0) - m(1.3), E(H) = |Delta(H) - Delta(O)| / |Delta(O)|, the noise sum, the transfer
ratio m_H(1.3) / m_O(1.3) and the verdicts as registered.

    python3 scripts/finger_relay/pip_only_summary.py relay|walk [--variant Touch] [--pairs 7 5 9]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

SIM = Path(__file__).resolve().parents[2]
METRIC = {"relay": "rotation_deg_mean", "walk": "walked_mm_mean"}
PAIRS = {"7": ("R", "U"), "5": ("R5", "U5"), "9": ("R9", "U9")}
SEEDS = (94000, 95000)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("task", choices=("relay", "walk"))
    ap.add_argument("--variant", default="")
    ap.add_argument("--pairs", nargs="+", default=["7"])
    a = ap.parse_args()
    run = SIM / "logs" / f"finger_{a.task}" / "run"
    key = METRIC[a.task]
    extra = ("goals_per_episode",) + (("past_180_rate",) if a.task == "relay" else ("dropped_rate",))

    def m(h, mu, s, k=key):
        p = run / f"transfer{a.variant}_{h}_mu{mu}_seed{s}.json"
        return json.loads(p.read_text())[k] if p.exists() else None

    def hand(h):
        v = {mu: [m(h, mu, s) for s in SEEDS] for mu in ("1.3", "2.0")}
        if None in v["1.3"] + v["2.0"]:
            return None
        d = [v["2.0"][i] - v["1.3"][i] for i in range(2)]
        return {"m13": v["1.3"], "m20": v["2.0"], "d": d, "d_mean": sum(d) / 2, "d_seed_diff": abs(d[0] - d[1]),
                **{f"{k}_13": [m(h, "1.3", s, k) for s in SEEDS] for k in extra}}

    o = hand("O")
    if o is None:
        print("O incomplete")
        return 1
    out = {"O": o, "pairs": {}}
    print(f"O: {key} mu1.3 {o['m13']} mu2.0 {o['m20']} Delta {o['d_mean']:.2f} (seed diff {o['d_seed_diff']:.2f})")
    undecidable = abs(o["d_mean"]) < 2 * o["d_seed_diff"]
    for t in a.pairs:
        r_, u_ = PAIRS[t]
        hr, hu = hand(r_), hand(u_)
        if hr is None or hu is None:
            print(t, "mm: incomplete")
            continue
        do = abs(o["d_mean"])
        e = {h: abs(x["d_mean"] - o["d_mean"]) / do for h, x in ((r_, hr), (u_, hu))}
        noise_e = (hr["d_seed_diff"] + hu["d_seed_diff"]) / do
        ratio = {h: [x["m13"][i] / o["m13"][i] for i in range(2)] for h, x in ((r_, hr), (u_, hu))}
        rm = {h: sum(v) / 2 for h, v in ratio.items()}
        noise_r = sum(abs(v[0] - v[1]) for v in ratio.values())
        de, dr = e[u_] - e[r_], rm[r_] - rm[u_]
        verdict_e = ("undecidable (|Delta(O)| < 2x its seed diff)" if undecidable else
                     "E(R) < E(U) beyond noise: prediction holds" if de > noise_e else
                     "E(R) >= E(U) beyond noise: falsified" if -de > noise_e else "within noise")
        verdict_r = ("ratio_R > ratio_U beyond noise" if dr > noise_r else
                     "ratio_U > ratio_R beyond noise" if -dr > noise_r else "within noise")
        both_within = verdict_e.startswith("within") and verdict_r == "within noise"
        res = {"hands": {r_: hr, u_: hu}, "E": e, "noise_E": noise_e, "ratio": ratio, "ratio_mean": rm,
               "noise_ratio": noise_r, "verdict_E": verdict_e, "verdict_ratio": verdict_r,
               "stop_rule_both_within_noise": both_within}
        out["pairs"][t] = res
        print(f"{t} mm: E {r_} {e[r_]:.3f} {u_} {e[u_]:.3f} noise {noise_e:.3f} -> {verdict_e}")
        print(f"      ratio {r_} {rm[r_]:.3f} {u_} {rm[u_]:.3f} noise {noise_r:.3f} -> {verdict_r}; stop rule: {both_within}")
        for h, x in ((r_, hr), (u_, hu)):
            print(f"      {h}: {key} mu1.3 {[round(v, 1) for v in x['m13']]} mu2.0 {[round(v, 1) for v in x['m20']]} "
                  + " ".join(f"{k} {[round(v, 3) for v in x[k + '_13']]}" for k in extra))
    (run / f"summary{a.variant}.json").write_text(json.dumps(out, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
