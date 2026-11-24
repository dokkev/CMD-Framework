"""Contact-face depth t and pole depth z (finite pole, 0 -> 120 deg) of Study 1's highest-coverage design of every
joint at q_req = 120 deg, for 3, 5, 7 and 9 mm pads -> design/output/paper/fig2_pole_depths.json (Fig. 2(b)).

    conda run -n cmd python paper/figures/fig2_poles.py
"""
import json
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "paper" / "figures"), str(ROOT / "design"), str(ROOT / "design" / "scripts" / "study"),
                str(ROOT / "design" / "scripts" / "study" / "study2_morphology_adaptation")]
import one_finger  # noqa: E402
from core.optimization.mechanism import read_config_file  # noqa: E402
from visualize.finger import view_of  # noqa: E402
from fig_mechanisms import finite_pole  # noqa: E402

cfg = read_config_file(one_finger.DEFAULT_CONFIG)
out = {}
for key in ("cfl", "2r_remote", "2r_dd", "rcj"):
    ds = one_finger.study1_designs(key, cfg["architectures"][key], 120.0, cfg)
    out[key] = []
    for h, d in sorted(ds.items()):
        v = view_of(d.mechanism, d.config, d.x)
        t = -float(v.proximal_pad()[:, 1].min())
        z = -float(finite_pole(v, 120.0)[1])
        out[key].append({"h": h, "t": t, "z": z})
        print(key, h, round(t, 2), round(z, 2))
dest = ROOT / "design" / "output" / "paper" / "fig2_pole_depths.json"
dest.write_text(json.dumps(out, indent=1))
print(dest)
