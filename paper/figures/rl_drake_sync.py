"""Drake sim-to-sim transfer of the Isaac Lab policies (docs/experimental/pr_mr.md), as Fig. 5 reads it: each 7 mm
redesign's mean score over O's, per task, training seed and contact model, from sim/drake/policy_transfer.py's runs
(sim/drake/logs/policy_transfer/<task>_<hand>_s<seed>_<tag>_<contact>.json, tag pt1; logs are not committed).

    python paper/figures/rl_drake_sync.py

Writes report/study4/data/rl_drake_transfer.json. A task with no redesign run (walk: its gate on O failed) is listed
with its O scores only."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOGS = ROOT / "sim" / "drake" / "logs" / "policy_transfer"
OUT = ROOT / "report" / "study4" / "data" / "rl_drake_transfer.json"
HANDS = ("MRM", "SK", "U")  # CoDe (MO-Pad), Skeleton, Coverage
SEEDS = (20, 21)


def main() -> int:
    out = {"source": "sim/drake/policy_transfer.py, tag pt1", "tasks": {}}
    for task in ("walk", "relay", "cube"):
        t = {}
        for contact in ("point", "hydro"):
            o = [LOGS / f"{task}_O_s{s}_pt1_{contact}.json" for s in SEEDS]
            if not all(p.exists() for p in o):
                continue
            o_score = [json.load(open(p))["score_mean"] for p in o]
            ratio = {}
            for h in HANDS:
                ps = [LOGS / f"{task}_{h}_s{s}_pt1_{contact}.json" for s in SEEDS]
                if all(p.exists() for p in ps):
                    ratio[h] = [json.load(open(p))["score_mean"] / so for p, so in zip(ps, o_score)]
            t[contact] = {"O_score": o_score, "ratio": ratio}
        out["tasks"][task] = t
    OUT.write_text(json.dumps(out, indent=1))
    print(OUT)
    for task, t in out["tasks"].items():
        for c, v in t.items():
            print(task, c, {h: [round(x, 2) for x in r] for h, r in v["ratio"].items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
