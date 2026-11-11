"""E4: the same CoDe hand USD (loop cut excluded from the articulation) on each Newton solver.

    uv run python scripts/code_hand/e4_backends.py --solver mujoco|kamino|xpbd|featherstone|semi_implicit|vbd
        [--loop ball|revolute] [--gravity] [--dt 0.002] [--substeps 1] [--mujoco-stiff]

Everything is measured from body poses, so maximal- and reduced-coordinate solvers are judged alike: the B' closure
residual, the PIP servo link's tracking of its target, the PIP angle against the exported closure table at the
measured servo angle, and the wall time per simulated second. A solver that fails to build, produces non-finite
states or ignores the loop is reported as such.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
import traceback
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from description import DEFAULT, Description, quat_to_matrix, rotation_about  # noqa: E402

FINGERS = ("index", "middle", "ring")
SOLVERS = ("mujoco", "kamino", "xpbd", "featherstone", "semi_implicit", "vbd")


def make_solver(name: str, model, args):
    from newton import solvers

    if name == "mujoco":
        solver = solvers.SolverMuJoCo(model)
        if args.mujoco_stiff:
            solref, solimp = solver.mjw_model.eq_solref.numpy(), solver.mjw_model.eq_solimp.numpy()
            solref[..., 0] = max(0.004, 2.0 * args.dt / args.substeps)
            solimp[..., 0], solimp[..., 1] = 0.99, 0.999
            solver.mjw_model.eq_solref.assign(solref)
            solver.mjw_model.eq_solimp.assign(solimp)
        return solver
    if name == "kamino":
        return solvers.SolverKamino(model)
    if name == "xpbd":
        return solvers.SolverXPBD(model, iterations=args.iterations)
    if name == "featherstone":
        return solvers.SolverFeatherstone(model)
    if name == "semi_implicit":
        return solvers.SolverSemiImplicit(model)
    if name == "vbd":
        return solvers.SolverVBD(model, iterations=args.iterations)
    raise ValueError(name)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--description", type=Path, default=DEFAULT)
    parser.add_argument("--solver", choices=SOLVERS, default="kamino")
    parser.add_argument("--loop", choices=("ball", "revolute"), default="ball")
    parser.add_argument("--gravity", action="store_true")
    parser.add_argument("--dt", type=float, default=0.002, help="control step [s]")
    parser.add_argument("--substeps", type=int, default=1)
    parser.add_argument("--iterations", type=int, default=10, help="XPBD / VBD iterations")
    parser.add_argument("--settle", type=float, default=0.4)
    parser.add_argument("--armature", type=float, default=1e-5)
    parser.add_argument("--mujoco-stiff", action="store_true", help="MuJoCo: stiff loop equality")
    args = parser.parse_args(argv)

    import newton
    import warp as wp

    desc = Description.load(args.description)
    builder = newton.ModelBuilder()
    if args.solver == "kamino":
        newton.solvers.SolverKamino.register_custom_attributes(builder)
    builder.add_usd(str(args.description.parent / f"code_hand_{args.loop}.usda"))
    if not args.gravity:
        builder.gravity = 0.0
    for k, a in enumerate(builder.joint_articulation):
        if a >= 0:
            for dof in range(builder.joint_qd_start[k], builder.joint_qd_start[k] + sum(builder.joint_dof_dim[k])):
                builder.joint_armature[dof] = args.armature
    if args.solver == "vbd":
        builder.color()
    model = builder.finalize()
    tag = f"{args.solver:13s} loop={args.loop:8s} gravity={'on ' if args.gravity else 'off'}"
    try:
        solver = make_solver(args.solver, model, args)
    except Exception as exc:  # noqa: BLE001
        print(f"{tag} | BUILD FAILED: {type(exc).__name__}: {str(exc).splitlines()[0][:160]}")
        return 1

    joint_names = [label.split("/")[-1] for label in model.joint_label]
    body_names = [label.split("/")[-1] for label in model.body_label]
    q_start = model.joint_q_start.numpy()
    bi = lambda name: body_names.index(name)  # noqa: E731
    articulation = model.joint_articulation.numpy()
    loops = [k for k, a in enumerate(articulation) if a < 0]
    X_p, X_c = model.joint_X_p.numpy(), model.joint_X_c.numpy()
    parents, children = model.joint_parent.numpy(), model.joint_child.numpy()
    axis = {d: np.array(desc.joint(f"{d}_pip_servo")["axis"]) for d in FINGERS}

    def anchor_world(pose, xform):
        return pose[:3] + quat_to_matrix(pose[3:]) @ xform[:3]

    state0, state1, control = model.state(), model.state(), model.control()
    newton.eval_fk(model, model.joint_q, model.joint_qd, state0)
    contacts = None
    worst = {"residual_mm": 0.0, "pip_err_deg": 0.0, "track_err_deg": 0.0}
    sub_dt = args.dt / args.substeps
    steps = int(round(args.settle / args.dt))
    wall = 0.0
    try:
        for target in np.linspace(0.0, 45.0, 10):
            tq = control.joint_target_q.numpy()
            for d in FINGERS:
                tq[int(q_start[joint_names.index(f"{d}_pip_servo")])] = math.radians(target)
            control.joint_target_q.assign(tq)
            wp.synchronize()
            start = time.perf_counter()
            for _ in range(steps * args.substeps):
                state0.clear_forces()
                solver.step(state0, state1, control, contacts, sub_dt)
                state0, state1 = state1, state0
            wp.synchronize()
            wall += time.perf_counter() - start
            bq = state0.body_q.numpy()
            if not np.isfinite(bq).all():
                print(f"{tag} | NON-FINITE state at servo target {target:.1f} deg")
                return 1
            for k in loops:
                r = np.linalg.norm(anchor_world(bq[parents[k]], X_p[k]) - anchor_world(bq[children[k]], X_c[k]))
                worst["residual_mm"] = max(worst["residual_mm"], 1e3 * float(r))
            for d in FINGERS:
                R = lambda name, d=d, bq=bq: quat_to_matrix(bq[bi(f"{d}_{name}")][3:])  # noqa: E731
                servo = rotation_about(R("proximal"), R("servo_link"), axis[d])
                pip = rotation_about(R("proximal"), R("distal"), axis[d])
                worst["pip_err_deg"] = max(worst["pip_err_deg"], abs(pip - desc.pip_from_servo_joint(d, servo)))
                worst["track_err_deg"] = max(worst["track_err_deg"], abs(servo - target))
    except Exception as exc:  # noqa: BLE001
        print(f"{tag} | STEP FAILED: {type(exc).__name__}: {str(exc).splitlines()[0][:160]}")
        traceback.print_exc(limit=2)
        return 1
    rate = 10 * args.settle / wall
    print(f"{tag} | residual {worst['residual_mm']:9.5f} mm  PIP err {worst['pip_err_deg']:8.4f} deg  "
          f"servo track {worst['track_err_deg']:8.4f} deg  final PIP {pip:7.2f}  {rate:6.2f}x realtime "
          f"(dt {args.dt}, substeps {args.substeps})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
