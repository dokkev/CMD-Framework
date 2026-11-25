"""Contact-surface paths of the simulated hands O, R and U for Fig. 1 (same construction as
sim/drake/render_hands.py: the distal face point c(q) in the proximal pad-face frame, q = 0..120 deg)
-> design/output/paper/fig1_paths_ORU.json.

    cd sim/drake && uv run python ../../paper/figures/fig1_paths.py
"""
import json, math, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim" / "drake"))
import two_pad_drake as T  # noqa: E402


def pose(g, qs, th, q_deg):
    q = math.radians(q_deg)
    theta = float(np.interp(q, qs, th))
    return g["ap"] + T.rot(theta) @ (g["b"] - g["ap"]), q


out = {}
for hand in "ORU":
    g = T.finger_geometry(hand)
    qs, th = T.servo_table(g)
    p = g["pads"]
    t = p["mount_offset_mm"] + p["thickness_mm"]
    cx = p["distal_end_mm"]
    c = []
    for d in np.linspace(0.0, 120.0, 121):
        B, q = pose(g, qs, th, d)
        c.append((B + T.rot(q) @ np.array([cx, -t]) - np.array([0.0, -t])).tolist())
    out[hand] = {"thickness_mm": p["thickness_mm"], "contact": c}
dest = ROOT / "design" / "output" / "paper" / "fig1_paths_ORU.json"
dest.parent.mkdir(parents=True, exist_ok=True)
json.dump(out, open(dest, "w"))
for h in "RU":
    d = np.max(np.linalg.norm(np.array(out[h]["contact"]) - np.array(out["O"]["contact"]), axis=1))
    print(h, "Delta_contact vs O = %.2f mm" % d)
print(dest)
