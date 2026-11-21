#!/usr/bin/env python3
"""Study 3: the three CFL fingers as CoDe hand parameters, checked against the study's own kinematics.

    python scripts/study/study3_trajectory/make_hands.py [--flexion 90] [--to 5]

Study 3 (research-objective Section 8) needs H0 (Study 1, 3 mm), H_R (Study 2's most-coverage redesign for the
thicker pad) and H_U (Study 1's first independent coverage-only design for it) as hands the simulator can load.
A study design vector decodes to a four-bar in the proximal frame (A, A', B, B' at the extended pose, axes
collinear) and two pads that fill their segments' support; the CoDe hand model takes the same four-bar as
A, A', |A'B|, |AB'|, B' and C in the distal frame (origin B, +X along B->C), the PIP servo's tilt, and the pads
by their spans along each link line. Everything else (palm, MCP axes, thumb) is the prototype's.

Check: over q in [0, q_req], the fingertip C from the hand model's four-bar against the study's sweep, and the
relative contact trajectory rebuilt from the hand's pads against the study's. Writes
``output/sim/study3/hands/<name>.yaml`` (``base:`` the prototype parameters) only if both agree within 1e-6 mm.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT, ROOT / "scripts" / "study", ROOT / "scripts" / "study" / "exploratory"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import e15_transmission_change as e15  # noqa: E402
from core.morphology.code_hand.model import four_bar  # noqa: E402
from core.morphology.code_hand.parameters import PROTOTYPE, FingerParameters  # noqa: E402
from core.morphology.mechanism.linkage.design import decode  # noqa: E402
from core.optimization.analysis.contact_kinematics import contact_path, paths  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

OUT = ROOT / "output" / "sim" / "study3" / "hands"
PAD_WIDTH_MM = 19.0  # as the prototype variant (pivot8_balanced.yaml): the XM335 case width


def hand_parameters(x, config, thickness_mm: float) -> dict:
    finger = decode(x, config.link_length_resolution_mm)
    joint = finger.joint
    a, ap, b, bp = joint.a, joint.ap, joint.b, joint.bp
    tilt = math.degrees(finger.pip_servo.tilt_rad) if hasattr(finger, "pip_servo") else float(x[7])
    mounts = (float(x[12]), float(x[13])) if len(x) > 13 else (3.0, 3.0)
    if abs(mounts[0] - mounts[1]) > 1e-9:
        raise ValueError(f"the hand model takes one mount offset; the design has {mounts}")
    params = {
        "a_x_mm": a.x_mm, "a_z_mm": a.z_mm, "ap_x_mm": ap.x_mm, "ap_z_mm": ap.z_mm,
        "apb_mm": math.hypot(b.x_mm - ap.x_mm, b.z_mm - ap.z_mm), "abp_mm": math.hypot(bp.x_mm - a.x_mm, bp.z_mm - a.z_mm),
        "bp_x_mm": bp.x_mm - b.x_mm, "bp_z_mm": bp.z_mm - b.z_mm,
        "c_x_mm": finger.distal.end_mm, "c_z_mm": 0.0, "pip_servo_tilt_deg": tilt,
    }
    pads = {"proximal_start_mm": finger.proximal.start_mm, "proximal_end_mm": finger.proximal.end_mm,
            "distal_start_mm": finger.distal.start_mm, "distal_end_mm": finger.distal.end_mm,
            "mount_offset_mm": mounts[0], "thickness_mm": thickness_mm, "width_mm": PAD_WIDTH_MM}
    return {k: float(v) for k, v in params.items()}, {k: float(v) for k, v in pads.items()}


def check(base, x, config, finger_params: dict, pads: dict, q_req: float) -> tuple[float, float]:
    finger = FingerParameters(**{**FingerParameters().__dict__, **finger_params})
    study = paths(base, x, config)
    q = np.asarray(study["q"])
    tips = np.array([four_bar(finger, math.degrees(v)).c for v in q])
    depth = pads["mount_offset_mm"] + pads["thickness_mm"]
    contact = contact_path(tips, q, depth, depth)
    return (float(np.max(np.linalg.norm(tips - np.asarray(study["tip"]), axis=1))),
            float(np.max(np.linalg.norm(contact - np.asarray(study["contact"]), axis=1))))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--flexion", type=float, default=90.0)
    parser.add_argument("--to", type=float, default=5.0)
    args = parser.parse_args(argv)
    config = read_config_file(ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    spec = config["architectures"]["cfl"]
    designs = e15.study1.study1_designs("cfl", spec, args.flexion, config)
    h0, goal = designs[3.0], designs[args.to]
    base = mechanism(h0.mechanism)
    import csv
    with (e15.STUDY2 / "front.csv").open(encoding="utf-8", newline="") as stream:
        rows = [r for r in csv.DictReader(stream) if r["architecture"] == "cfl"
                and float(r["required_flexion_deg"]) == args.flexion and float(r["to_mm"]) == args.to]
    x_r = tuple(float(v) for v in max(rows, key=lambda r: float(r["c_pad_mean"]))["x"].split(";"))
    folder = e15.study1.condition_folder("cfl", spec, args.flexion, args.to, float(config["native_thickness_mm"]))
    x_u = e15.balanced_x(sorted(folder.glob("portfolio_*"))[0])
    OUT.mkdir(parents=True, exist_ok=True)
    tag = f"q{args.flexion:g}_h3to{args.to:g}"
    ok = True
    for name, x, cfg, h in (("h0", h0.x, h0.config, 3.0), ("hr", x_r, goal.config, args.to), ("hu", x_u, goal.config, args.to)):
        finger_params, pads = hand_parameters(x, cfg, h)
        tip_err, contact_err = check(base, x, cfg, finger_params, pads, math.radians(args.flexion))
        print(f"{name}: tip {tip_err:.2e} mm, relative contact trajectory {contact_err:.2e} mm from the study's")
        if max(tip_err, contact_err) > 1e-6:
            ok = False
            continue
        rel = Path("../../../../core/morphology/code_hand") / PROTOTYPE.name
        doc = {"base": str(rel), "finger": finger_params, "pads": pads,
               "source": {"study": "Study 3 (research-objective Section 8)", "design": [round(v, 6) for v in x],
                          "required_flexion_deg": args.flexion, "pad_thickness_mm": h}}
        path = OUT / f"cfl_{tag}_{name}.yaml"
        path.write_text(yaml.safe_dump(doc, sort_keys=False))
        print(f"  -> {path.relative_to(ROOT)}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
