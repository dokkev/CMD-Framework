"""Quick local search for a CFL design whose 7 mm pads, lengthened to the collision limit, come closest at 150 deg:
random perturbations of the best candidates of fig2_pad_search.py, kept if the mechanism poses over 0..150 deg
without a collision and the proximal pad keeps at least 34 mm. Writes design/output/paper/fig2_pad_refine.json.

    conda run -n cmd python paper/figures/fig2_pad_refine.py [evaluations]
"""
import json
import sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "paper" / "figures"))
import fig2_pad_search as S  # noqa: E402  (runs nothing heavy at import? guarded below)
from core.optimization.mechanism import mechanism  # noqa: E402

N = int(sys.argv[1]) if len(sys.argv) > 1 else 600
rng = np.random.default_rng(0)
seeds = json.loads((ROOT / "design/output/paper/fig2_pad_search.json").read_text())[:5]
best = []
for sd in seeds:
    q_req, h = sd["cfg"]
    base = S.one_finger.study1_designs("cfl", S.spec, q_req, S.cfg)[h]
    names = list(mechanism(base.mechanism).variable_names)
    bounds = getattr(base.config, "bounds", None)
    cur, cur_gap = list(sd["x"]), sd["gap150_mm"]
    for it in range(N // len(seeds)):
        cand = list(cur)
        for i, nm in enumerate(names):
            if "mount_offset" in nm:
                continue
            step = 1.5 if nm.endswith("_mm") else 3.0
            cand[i] += rng.normal(0, step) * (rng.random() < 0.5)
            if bounds and nm in bounds:
                lo, hi = bounds[nm]
                cand[i] = float(np.clip(cand[i], lo, hi))
        r = S.evaluate(base.mechanism, base.config, cand)
        if r and r["len"][0] >= 34.0 and r["gap150_mm"] < cur_gap:
            cur, cur_gap = cand, r["gap150_mm"]
            print(f"{sd['source']} it {it}: gap {cur_gap:.2f} mm, len {[round(v, 1) for v in r['len']]}", flush=True)
    best.append({"source": sd["source"] + " refined", "cfg": sd["cfg"], "x": cur, "gap150_mm": cur_gap})
best.sort(key=lambda r: r["gap150_mm"])
(ROOT / "design/output/paper/fig2_pad_refine.json").write_text(json.dumps(best, indent=1))
print("best", round(best[0]["gap150_mm"], 2), best[0]["source"])
