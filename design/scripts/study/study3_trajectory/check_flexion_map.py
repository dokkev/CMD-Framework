#!/usr/bin/env python3
"""Study 3: check the simulator's flexion-to-servo map (sim/isaaclab/src/code_isaaclab/joint_map.py) against the design kinematics.

    python scripts/study/study3_trajectory/check_flexion_map.py

For H0, H_R, H_U at 120 deg, 3 -> 7 mm: theta(q) and dtheta/dq from the exported four-bar table (index finger) against
the four-bar solved by the study code (e15), and the round trip q -> theta -> q. Reads saved designs and exports only.
"""
import sys, json, math, importlib.util, numpy as np, yaml
ROOT = __import__("pathlib").Path(__file__).resolve().parents[3]; sys.path.insert(0, str(ROOT / "scripts/study/exploratory")); sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location("joint_map", ROOT.parent / "sim/isaaclab/src/code_isaaclab/joint_map.py")
jm = importlib.util.module_from_spec(spec); sys.modules["joint_map"] = jm; spec.loader.exec_module(jm)
import e15_transmission_change as e15
from core.optimization.mechanism import read_config_file
config = read_config_file(e15.REPOSITORY_ROOT / "scripts/study/study2_morphology_adaptation/one_finger.yaml")
d = e15.study1.study1_designs("cfl", config["architectures"]["cfl"], 120.0, config)
for n, cfg in (("h0", d[3.0].config), ("hr", d[7.0].config), ("hu", d[7.0].config)):
    x = tuple(yaml.safe_load(open(ROOT / f"output/sim/study3/hands/cfl_q120_h3to7_{n}.yaml"))["source"]["design"])
    q, th, ratio = e15.transmission(x, cfg, math.radians(120.0))
    m = jm.flexion_maps(json.load(open(ROOT / f"output/sim/study3/hands/cfl_q120_h3to7_{n}.json")))["index"]
    sign = 1.0 if m.theta[-1] > m.theta[0] else -1.0
    th_sim = sign * (m.servo(q) - m.servo(0.0)); r_sim = sign * m.ratio(q)
    inner = (q > math.radians(2)) & (q < math.radians(118))
    print(f"{n}: |theta_sim - theta_design| max {math.degrees(np.max(np.abs(th_sim - th))):.4f} deg; "
          f"|ratio_sim - ratio_design| max {np.max(np.abs(r_sim - ratio)[inner]):.4f} (ratio {ratio.min():.3f}..{ratio.max():.3f}); "
          f"round trip max {math.degrees(np.max(np.abs(m.flexion(m.servo(q)) - q))):.2e} deg")
