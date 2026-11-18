# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""E1: does Newton (SolverMuJoCo / MuJoCo-Warp) close the CoDe four-bar, and does the PIP follow the model?

    uv run python scripts/code_hand/e1_newton_loop.py [--loop ball|revolute|none] [--gravity]

The hand is built with the ModelBuilder API (no USD). Each finger's PIP servo joint (A') is position driven
through a ramp; the other actuated joints hold 0. At every settled sample the script reports, per finger:
- the B' closure residual (distance between the driver link's and the distal's copies of B');
- the PIP angle measured from the bodies (distal relative to proximal) against the exported closure table
  evaluated at the measured servo joint angle;
- the passive joints (A, B) against the table.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from code_isaaclab.assets import DEFAULT_DESCRIPTION_PATH, Description, build_newton, quat_to_matrix, rotation_about
from code_isaaclab.loops import stiffen_mujoco_loops


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--description", type=Path, default=DEFAULT_DESCRIPTION_PATH)
    parser.add_argument("--loop", choices=("ball", "revolute", "none"), default="ball")
    parser.add_argument("--gravity", action="store_true")
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--settle", type=float, default=0.4, help="seconds per target")
    parser.add_argument("--eq-timeconst", type=float, help="equality solref time constant [s] (MuJoCo default 0.02)")
    parser.add_argument(
        "--eq-solimp",
        type=float,
        nargs=2,
        metavar=("DMIN", "DMAX"),
        help="equality solimp impedance (MuJoCo default 0.9 0.95)",
    )
    args = parser.parse_args(argv)

    import newton
    from newton.solvers import SolverMuJoCo

    desc = Description.load(args.description)
    fingers = desc.four_bar_digits()
    builder, index, _ = build_newton(desc, loop=args.loop)
    if not args.gravity:
        builder.gravity = 0.0
    model = builder.finalize()
    solver = SolverMuJoCo(model)
    mj = solver.mj_model
    if args.eq_timeconst is not None or args.eq_solimp is not None:
        default_tc = float(solver.mjw_model.eq_solref.numpy().reshape(-1, 2)[0, 0]) if mj.neq else 0.02
        default_imp = tuple(solver.mjw_model.eq_solimp.numpy().reshape(-1, 5)[0, :2]) if mj.neq else (0.9, 0.95)
        stiffen_mujoco_loops(
            solver,
            args.dt,
            timeconst=default_tc if args.eq_timeconst is None else args.eq_timeconst,
            solimp=default_imp if args.eq_solimp is None else tuple(args.eq_solimp),
        )
    if mj.neq:
        print(
            "eq_solref",
            solver.mjw_model.eq_solref.numpy().reshape(-1, 2)[0],
            "eq_solimp",
            solver.mjw_model.eq_solimp.numpy().reshape(-1, 5)[0],
        )
    eq_types = [int(t) for t in mj.eq_type]
    print(
        f"loop={args.loop}: Newton joints {model.joint_count}, dofs {model.joint_dof_count}; "
        f"MuJoCo nv {mj.nv}, neq {mj.neq} (types {eq_types}), nu {mj.nu}"
    )

    labels = list(model.joint_label)
    q_start = model.joint_q_start.numpy()

    def qi(name: str) -> int:
        return int(q_start[labels.index(name)])

    state0, state1, control = model.state(), model.state(), model.control()
    newton.eval_fk(model, model.joint_q, model.joint_qd, state0)
    servo_targets = np.linspace(0.0, 45.0, 10)  # PIP servo joint [deg] ~ PIP 0..~110
    worst = {"residual_mm": 0.0, "pip_err_deg": 0.0, "passive_err_deg": 0.0, "track_err_deg": 0.0}
    steps = int(args.settle / args.dt)
    rows = []
    cuts_by_name = {c["name"]: c for c in desc.loops}
    for target in servo_targets:
        tq = control.joint_target_q.numpy()
        for d in fingers:
            tq[qi(f"{d}_pip_servo")] = math.radians(target)
        control.joint_target_q.assign(tq)
        for _ in range(steps):
            solver.step(state0, state1, control, None, args.dt)
            state0, state1 = state1, state0
        jq = state0.joint_q.numpy()
        bq = state0.body_q.numpy()
        for d in fingers:
            servo = math.degrees(jq[qi(f"{d}_pip_servo")])
            prox, dist, drv = (bq[index[f"{d}_{n}"]] for n in ("proximal", "distal", "driver_link"))
            axis = np.asarray(desc.joint(f"{d}_pip_servo")["axis"], dtype=float)
            pip = rotation_about(quat_to_matrix(prox[3:]), quat_to_matrix(dist[3:]), axis)
            expected = desc.pip_from_servo_joint(d, servo)
            anchor = np.asarray(cuts_by_name[f"{d}_bp"]["anchor"], dtype=float)

            def world(pose: np.ndarray, body_name: str, anchor_pt: np.ndarray = anchor) -> np.ndarray:
                return pose[:3] + quat_to_matrix(pose[3:]) @ (anchor_pt - np.asarray(desc.body(body_name)["origin"]))

            residual = 1e3 * float(np.linalg.norm(world(drv, f"{d}_driver_link") - world(dist, f"{d}_distal")))
            closed = desc.closed_pose(d, expected)
            passive = max(abs(math.degrees(jq[qi(f"{d}_{k}")]) - closed[f"{d}_{k}"]) for k in ("a", "b"))
            worst["residual_mm"] = max(worst["residual_mm"], residual)
            worst["pip_err_deg"] = max(worst["pip_err_deg"], abs(pip - expected))
            worst["passive_err_deg"] = max(worst["passive_err_deg"], passive)
            worst["track_err_deg"] = max(worst["track_err_deg"], abs(servo - target))
            if d == fingers[0]:
                rows.append((target, servo, pip, expected, residual))
    print(f" target  servo    pip(sim) pip(model)  B' residual [mm]   ({fingers[0]})")
    for target, servo, pip, expected, residual in rows:
        print(f"{target:7.2f} {servo:7.2f} {pip:10.3f} {expected:10.3f} {residual:12.5f}")
    print("worst over fingers:", {k: round(v, 5) for k, v in worst.items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
