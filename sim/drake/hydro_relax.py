# Post hoc: run hook_pull / twist_grasp analyze with the pad-force gate at O's hydro force at 1.5 mm interpenetration
# (Drake contact surface, O's pad, rigid cylinder of the placement's radius) instead of 3.75 N.
# usage: hydro_relax.py <module> <tag> <noise_tag> <out>
import sys, importlib, numpy as np

R = np.array([5, 6, 7, 8, 12.5, 15, 17.5, 20.0])
F15 = np.array([4.36, 4.82, 5.24, 5.63, 7.12, 7.83, 8.47, 9.08])  # N, hydro_eq.py
mod = importlib.import_module(sys.argv[1])
orig = mod._load
def load(hand, tag):
    d = orig(hand, tag)
    if d is None: return None
    for r in d.values():
        g = float(np.interp(r["rho_mm"], R, F15))
        for run in r.get("runs", {}).values():
            if "pre" in run and "pen_mm" in run["pre"] and not run["pre"].get("_s"):
                run["pre"]["pen_mm"] = run["pre"]["pen_mm"] * 2.5 / g * 1.5; run["pre"]["_s"] = 1
    return d
mod._load = load
from pathlib import Path
class A: pass
a = A(); a.tag, a.noise_tag, a.out = sys.argv[2], sys.argv[3], Path(sys.argv[4])
if hasattr(mod, "D_MIN_N"): pass
a.d_min = None
sys.exit(mod.cmd_analyze(a))
