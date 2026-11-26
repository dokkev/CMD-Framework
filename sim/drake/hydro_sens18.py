# usage: hydro_sens18.py <analyze --out json>
# E_sens over all 18 hands (hands with no scored pair: E_sens = inf), Spearman with average ranks for ties,
# one-sided permutation p (20000, seed 0)
import sys, json, numpy as np

from analyze import DESIGN
def rank(v):
    v = np.asarray(v, float); o = np.argsort(v, kind="stable"); r = np.empty(len(v)); i = 0
    while i < len(v):
        j = i
        while j + 1 < len(v) and v[o[j + 1]] == v[o[i]]: j += 1
        r[o[i:j + 1]] = (i + j) / 2; i = j + 1
    return r
def sp(x, y):
    rx, ry = rank(x), rank(y); rho = float(np.corrcoef(rx, ry)[0, 1])
    rng = np.random.default_rng(0)
    perms = [float(np.corrcoef(rx, rng.permutation(ry))[0, 1]) for _ in range(20000)]
    return rho, float(np.mean(np.array(perms) >= rho - 1e-12))
out = json.load(open(sys.argv[1]))
rows = {r["hand"]: r for r in out["rows"]}
hs = [h for h in DESIGN if h in rows]
es = [rows[h]["E_sens"] if rows[h]["E_sens"] is not None else float("inf") for h in hs]
rc, pc = sp([DESIGN[h][0] for h in hs], es); rs, ps = sp([DESIGN[h][1] for h in hs], es)
fmt = lambda e: "inf" if e == float("inf") else f"{100*e:.1f}%"
print(f"hands {len(hs)}; E_sens R {fmt(rows['R']['E_sens'])} U {fmt(rows['U']['E_sens'])}; inf hands {sum(e == float('inf') for e in es)}")
print(f"rho(E_sens, D_contact) {rc:.2f} (p {pc:.4f}); rho(E_sens, D_skel) {rs:.2f} (p {ps:.4f})")
