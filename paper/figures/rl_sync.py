"""Fig. 4 (learned policies) data: the share of the original hand's performance that each redesign keeps when a policy
trained on O runs on it zero-shot (pad friction 1.3, mean of eval seeds 94000 and 95000), per training seed.

- walk (7b'): a finger walks an object along its pads; walked mm per episode.
- relay (7a'): the fingertip's end turns a dial; rotation per episode, face anchor (the same face-frame coordinates on
  every hand), each hand over O's.
- cube: a whole-hand policy turns a cube about the vertical axis; signed rotation per episode (seed 20 from
  logs/pr_mr_transfer/cube, seed 21 from logs/pr_mr_transfer_cube_s21/cube).

Hands: O, R (CoDe in the paper), U (Coverage), SK (Skeleton, conv_e25), MRM (MO-Pad), MS (Palm-Tip).

    python3 paper/figures/rl_sync.py  ->  report/study4/data/rl_transfer.json
"""
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
L = ROOT / "sim" / "isaaclab" / "logs"
HANDS = ("R", "U", "SK", "MRM", "MS")
SEEDS = (94000, 95000)


def per_ckpt(path_fmt, key):
    """Mean over the eval seeds of `key`, one value per training checkpoint in the file (list) or one (dict)."""
    vals = []
    for s in SEEDS:
        d = json.loads(Path(path_fmt.format(s=s)).read_text())
        d = d if isinstance(d, list) else [d]
        vals.append([x[key] for x in d])
    return np.mean(vals, axis=0).tolist()


def main():
    out = {"note": __doc__.split("\n\n")[0], "tasks": {}}
    walk = {h: per_ckpt(str(L / "pr_mr_transfer" / "walk" / f"{h}_mu1.3_seed{{s}}.json"), "walked_mm_mean")
            for h in ("O",) + HANDS}
    relay = {h: per_ckpt(str(L / "pr_mr_transfer" / "relay" / f"face_{h}_mu1.3_seed{{s}}.json"), "rotation_deg_mean")
             for h in ("O",) + HANDS}
    cube = {}
    for h in ("O",) + HANDS:
        cube[h] = [per_ckpt(str(L / d / "cube" / f"eval_{h}_mu1.3_seed{{s}}.json"), "signed_rotation_deg_mean")[0]
                   for d in ("pr_mr_transfer", "pr_mr_transfer_cube_s21")]
    for name, raw in (("walk", walk), ("relay", relay), ("cube", cube)):
        out["tasks"][name] = {"O": raw["O"], "ratio": {h: [v / o for v, o in zip(raw[h], raw["O"])] for h in HANDS}}
    dest = ROOT / "report" / "study4" / "data" / "rl_transfer.json"
    dest.write_text(json.dumps(out, indent=1))
    for name, t in out["tasks"].items():
        print(name, {h: [round(v, 3) for v in r] for h, r in t["ratio"].items()})


if __name__ == "__main__":
    main()
