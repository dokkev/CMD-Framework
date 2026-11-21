#!/usr/bin/env python3
"""How much a CFL redesign for a thicker interface changes the transmission (servo angle to flexion).

    python scripts/study/exploratory/e15_transmission_change.py [--flexion 90] [--to 5]

Study 3 (research-objective Section 8) compares three CFL fingers at one required flexion:
H0, Study 1's design for the 3 mm pad (Study 2's reference); H_R, Study 2's most-coverage redesign for the
thicker pad (the relative contact trajectory as a target); H_U, Study 1's independent coverage-only design
for the thicker pad (its first portfolio, the one Study 2 compared against). A frozen policy that commands
servo angles meets a changed transmission on H_R and H_U even where the relative contact trajectory is kept.
For each pair this reports, over q in [0, q_req]:

- D_c: the relative contact trajectory's largest change (mm), recomputed as a check against Study 2;
- the servo angle theta(q) and its largest difference from H0's at the same flexion (deg);
- the flexion error of a servo-angle command: H0's theta(q) sent to the other finger gives the flexion
  theta_other^-1(theta0(q)), and its largest difference from q (deg);
- dtheta/dq (the transmission ratio, which is the mechanical advantage tau_pip / tau_servo by virtual work)
  and its largest relative difference from H0's; a servo PD gain gives a flexion stiffness scaled by its
  square.

Reads saved designs only; writes nothing.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
for path in (REPOSITORY_ROOT, REPOSITORY_ROOT / "scripts" / "study",
             REPOSITORY_ROOT / "scripts" / "study" / "study2_morphology_adaptation"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import one_finger as study1  # noqa: E402
from core.morphology.mechanism.linkage.design import decode, read_design  # noqa: E402
from core.optimization.analysis.contact_kinematics import paths  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

STUDY2 = REPOSITORY_ROOT / "output" / "study" / "study2_redesign_2026-11-21"


def balanced_x(portfolio: Path) -> tuple[float, ...]:
    with (portfolio / "seeds.csv").open(encoding="utf-8", newline="") as stream:
        row = next(r for r in csv.DictReader(stream) if "balanced" in r.get("seed_role", ""))
    return tuple(read_design(row))


def transmission(x, config, q_req: float):
    q = np.linspace(0.0, q_req, 241)
    sweep = decode(x, config.link_length_resolution_mm).sweep_pip(q)
    n = len(sweep.q_pip_rad)
    if n < len(q):
        raise ValueError(f"the four-bar closes only up to {math.degrees(q[n - 1]):.1f} deg")
    qq, theta = np.asarray(sweep.q_pip_rad), np.asarray(sweep.servo_rad)
    if theta[-1] < theta[0]:
        theta = -theta  # sign convention: servo angle grows with flexion
    return qq, theta - theta[0], np.gradient(theta, qq)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--flexion", type=float, default=90.0)
    parser.add_argument("--to", type=float, default=5.0)
    args = parser.parse_args(argv)
    config = read_config_file(REPOSITORY_ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    spec = config["architectures"]["cfl"]
    designs = study1.study1_designs("cfl", spec, args.flexion, config)
    h0, goal = designs[3.0], designs[args.to]
    base = mechanism(h0.mechanism)
    q_req = math.radians(args.flexion)

    with (STUDY2 / "front.csv").open(encoding="utf-8", newline="") as stream:
        rows = [r for r in csv.DictReader(stream) if r["architecture"] == "cfl" and float(r["required_flexion_deg"]) == args.flexion
                and float(r["to_mm"]) == args.to]
    r_row = max(rows, key=lambda r: float(r["c_pad_mean"]))
    x_r = tuple(float(v) for v in r_row["x"].split(";"))
    native = float(config["native_thickness_mm"])
    folder = study1.condition_folder("cfl", spec, args.flexion, args.to, native)
    x_u = balanced_x(sorted(folder.glob("portfolio_*"))[0])

    reference = paths(base, h0.x, h0.config)
    scale = float(base.report(base.evaluate(h0.x, h0.config))["finger_length_mm"])
    q0, th0, ratio0 = transmission(h0.x, h0.config, q_req)
    print(f"CFL, q_req {args.flexion:g} deg, 3 -> {args.to:g} mm (finger length {scale:.1f} mm)")
    print(f"H0: theta(q_req) {math.degrees(th0[-1]):.1f} deg, dtheta/dq {ratio0.min():.2f}..{ratio0.max():.2f}")
    for name, x, cfg in (("H_R", x_r, goal.config), ("H_U", x_u, goal.config)):
        evaluation = base.evaluate(x, cfg)
        report = base.report(evaluation)
        own = paths(base, x, cfg, evaluation, len(reference["contact"]))
        d_c = float(np.max(np.linalg.norm(own["contact"] - reference["contact"], axis=1)))
        q, th, ratio = transmission(x, cfg, q_req)
        flex_err = np.interp(th0, th, q) - q0  # H0's servo command, sent to this finger
        print(f"{name}: C_pad {float(report.get('c_pad_mean', float('nan'))):.3f}, D_c {d_c:.2f} mm, "
              f"theta(q_req) {math.degrees(th[-1]):.1f} deg, "
              f"max |theta - theta0| {math.degrees(np.max(np.abs(th - th0))):.1f} deg, "
              f"flexion error of H0's command max {math.degrees(np.max(np.abs(flex_err))):.1f} deg "
              f"(at q {math.degrees(q0[np.argmax(np.abs(flex_err))]):.0f} deg), "
              f"dtheta/dq {ratio.min():.2f}..{ratio.max():.2f}, max rel. change {np.max(np.abs(ratio / ratio0 - 1)):.0%}, "
              f"MA_wc {float(report.get('ma_wc', float('nan'))):.3f}")
    print(f"H0: MA_wc {float(base.report(base.evaluate(h0.x, h0.config)).get('ma_wc', float('nan'))):.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
