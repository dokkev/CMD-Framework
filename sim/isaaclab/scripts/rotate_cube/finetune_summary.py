"""Summarise attempt 6 (``docs/experimental/rl_transfer.md``): recovery curves, recovery iterations, AUC, final ratios
and the mu 1.3 -> 2.0 rotation E of the fine-tuned policies.

    python3 scripts/rotate_cube/finetune_summary.py [--hands O R U] [--out logs/rotate_cube/finetune_run/summary.json]

Reads ``logs/rotate_cube/finetune_run/curve_<H>_it0.json`` (model_1497, iteration 0), each run's
``<run>_ft1|_ft2/eval/model_<k>_rotation_seed94000_*.json`` (256 envs; the runs split at iteration 100) and ``finetune_run/final_<H>_s<S>_mu<mu>_seed<E>.json``.
"""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

SIM = Path(__file__).resolve().parents[2]
RUN_DIR = SIM / "logs" / "rotate_cube" / "finetune_run"
EXP = SIM / "logs" / "rsl_rl" / "code_rotate_cube_omnireset_v2_finetune"
O_RELEASE = {94000: 933.2445068359375, 95000: 883.0}  # 1024-env held-out mean rotation of model_1497 on O
ITERS = list(range(0, 301, 25))
THRESHOLD = 0.9
KEYS = {"rot": "signed_rotation_deg_mean", "full30": "full_turn_within_40mm_and_upright30deg_rate",
        "fullclean": "full_turn_and_clean_timeout_rate", "goals": "goals_per_episode_mean"}


def load(path):
    return json.loads(Path(path).read_text())


def o_release():
    out = {}
    for seed in (94000, 95000):
        d = load(SIM / "logs" / "rotate_cube" / "transfer" / f"eval_O_mu1.3_seed{seed}.json")
        out[seed] = {k: d[v] for k, v in KEYS.items()}
    return out


def curve(hand, seed):
    it0 = RUN_DIR / f"curve_{hand}_it0.json"
    runs = sorted(EXP.glob(f"*_rotate_cube_{hand}_OmniResetV2_s{seed}_ft[123]"))  # the run's parts (split at 100/200)
    if not it0.exists() or not runs:
        return None
    pts = {0: load(it0)["signed_rotation_deg_mean"]}
    for k in ITERS[1:]:
        f = sorted(x for r in runs for x in glob.glob(str(r / "eval" / f"model_{k}_rotation_seed94000_*.json")))
        if f:
            pts[k] = load(f[-1])["signed_rotation_deg_mean"]
    return {k: v / O_RELEASE[94000] for k, v in pts.items()}


def recovery(c):
    for k in sorted(c):
        if c[k] >= THRESHOLD:
            return k
    return None


def auc(c):
    ks = sorted(c)
    if ks[-1] != 300 or ks != ITERS:
        return None
    return sum((c[a] + c[b]) / 2 * (b - a) for a, b in zip(ks, ks[1:])) / 300


def final(hand, seed, mu, eval_seed):
    p = RUN_DIR / f"final_{hand}_s{seed}_mu{mu}_seed{eval_seed}.json"
    if not p.exists():
        return None
    d = load(p)
    return {k: d[v] for k, v in KEYS.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hands", nargs="+", default=["O", "R", "U"])
    ap.add_argument("--out", type=Path)
    a = ap.parse_args()
    rel = o_release()
    res = {"o_release": rel, "curves": {}, "recovery": {}, "auc": {}, "best": {}, "final": {}, "E": {}}
    for h in a.hands:
        for s in (20, 21):
            c = curve(h, s)
            if c is None:
                continue
            key = f"{h}_s{s}"
            res["curves"][key] = c
            res["recovery"][key] = recovery(c)
            res["auc"][key] = auc(c)
            res["best"][key] = max(c.values())
            print(f"{key:7s} " + " ".join(f"{k}:{v:.3f}" for k, v in sorted(c.items())) +
                  f"  | recovery {res['recovery'][key]}  AUC {res['auc'][key]}  best {res['best'][key]:.3f}")
    for s in (20, 21):
        o_ft = {mu: [final("O", s, mu, e) for e in (94000, 95000)] for mu in ("1.3", "2.0")}
        for h in a.hands:
            f = {mu: [final(h, s, mu, e) for e in (94000, 95000)] for mu in ("1.3", "2.0")}
            if any(x is None for x in f["1.3"]):
                continue
            key = f"{h}_s{s}"
            res["final"][key] = {
                "abs_mu1.3": f["1.3"],
                "ratio_vs_release": {k: sum(f["1.3"][i][k] / rel[e][k] for i, e in enumerate((94000, 95000))) / 2
                                     for k in KEYS},
            }
            if all(x is not None for x in o_ft["1.3"]):
                res["final"][key]["ratio_vs_O_ft"] = {
                    k: sum(f["1.3"][i][k] / o_ft["1.3"][i][k] for i in range(2)) / 2 for k in KEYS}
            if all(x is not None for x in f["2.0"] + o_ft["2.0"] + o_ft["1.3"]):
                d = [f["2.0"][i]["rot"] - f["1.3"][i]["rot"] for i in range(2)]
                do = [o_ft["2.0"][i]["rot"] - o_ft["1.3"][i]["rot"] for i in range(2)]
                dm, dom = sum(d) / 2, sum(do) / 2
                res["E"][key] = {"d_rot": d, "d_rot_O_ft": do, "E": abs(dm - dom) / abs(dom),
                                 "seed_noise": abs(d[0] - d[1]) / abs(dom)}
    print(json.dumps({k: res[k] for k in ("final", "E")}, indent=1))
    if a.out:
        a.out.write_text(json.dumps(res, indent=1) + "\n")


if __name__ == "__main__":
    main()
