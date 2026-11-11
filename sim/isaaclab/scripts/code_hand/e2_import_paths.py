"""E2: the same loop-closure check as E1, with the hand loaded from each asset path Newton offers.

    uv run python scripts/code_hand/e2_import_paths.py --source builder|urdf|usd-ball|usd-revolute [--gravity]
        [--eq-timeconst 0.004 --eq-solimp 0.99 0.999]

- builder: ModelBuilder API (E1).
- urdf: ``add_urdf`` loads the tree (URDF has no loops); the loader closes each cut between the two marker frames
  it finds in the URDF (lumohand style), as a Newton joint outside the articulation.
- usd-*: ``add_usd`` with the cut authored as ``physics:excludeFromArticulation``; this is the path Isaac Lab's
  Newton backend takes.

Run ``make_assets.py`` first. The residual is measured from the loop joint's own anchors.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from description import DEFAULT, Description, build_newton, quat_to_matrix, rotation_about  # noqa: E402

FINGERS = ("index", "middle", "ring")


def build(source: str, desc: Description, out_dir: Path):
    import newton
    import warp as wp

    if source == "builder":
        builder, _, _ = build_newton(desc, loop="ball")
        return builder
    builder = newton.ModelBuilder()
    if source == "urdf":
        path = out_dir / "code_hand.urdf"
        builder.add_urdf(str(path), floating=False, collapse_fixed_joints=True)
        root = ET.parse(path).getroot()
        origins = {j.find("child").get("link"): (j.find("parent").get("link"),
                                                 [float(v) for v in j.find("origin").get("xyz").split()])
                   for j in root.findall("joint") if j.get("type") == "fixed"}
        labels = [label.split("/")[-1] for label in builder.body_label]
        for cut in desc.loops:
            (parent, p_xyz), (child, c_xyz) = (origins[f"{cut['name']}_loop_{s}"] for s in ("parent", "child"))
            j = builder.add_joint_ball(labels.index(parent), labels.index(child),
                                       parent_xform=wp.transform(wp.vec3(*p_xyz), wp.quat_identity()),
                                       child_xform=wp.transform(wp.vec3(*c_xyz), wp.quat_identity()),
                                       label=cut["name"])
            builder.joint_articulation[j] = -1
        # add_urdf reads no drives: give the actuated joints the same PD as the other paths.
        for k, label in enumerate(builder.joint_label):
            name = label.split("/")[-1]
            if name in desc.actuated():
                dof = builder.joint_qd_start[k]
                builder.joint_target_ke[dof], builder.joint_target_kd[dof] = 5.0, 0.05
        return builder
    loop = source.split("-")[1]
    builder.add_usd(str(out_dir / f"code_hand_{loop}.usda"))
    return builder


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--description", type=Path, default=DEFAULT)
    parser.add_argument("--source", choices=("builder", "urdf", "usd-ball", "usd-revolute"), default="usd-ball")
    parser.add_argument("--gravity", action="store_true")
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--settle", type=float, default=0.4)
    parser.add_argument("--eq-timeconst", type=float)
    parser.add_argument("--eq-solimp", type=float, nargs=2)
    parser.add_argument("--armature", type=float, help="set every tree joint's armature [kg m^2]")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)

    import newton
    from newton.solvers import SolverMuJoCo

    desc = Description.load(args.description)
    builder = build(args.source, desc, args.description.parent)
    if not args.gravity:
        builder.gravity = 0.0
    if args.armature is not None:
        for k, a in enumerate(builder.joint_articulation):
            if a >= 0:
                for dof in range(builder.joint_qd_start[k], builder.joint_qd_start[k] + builder.joint_dof_dim[k][1]):
                    builder.joint_armature[dof] = args.armature
    print("armature (first revolute dof):", builder.joint_armature[builder.joint_qd_start[1]])
    model = builder.finalize()
    solver = SolverMuJoCo(model)
    if args.eq_timeconst is not None:
        solref = solver.mjw_model.eq_solref.numpy()
        solref[..., 0] = args.eq_timeconst
        solver.mjw_model.eq_solref.assign(solref)
    if args.eq_solimp is not None:
        solimp = solver.mjw_model.eq_solimp.numpy()
        solimp[..., 0], solimp[..., 1] = args.eq_solimp
        solver.mjw_model.eq_solimp.assign(solimp)
    mj = solver.mj_model
    joint_names = [label.split("/")[-1] for label in model.joint_label]
    body_names = [label.split("/")[-1] for label in model.body_label]
    articulation = model.joint_articulation.numpy()
    loop_joints = [k for k, a in enumerate(articulation) if a < 0]
    ke = model.joint_target_ke.numpy()
    print(f"{args.source}: bodies {model.body_count}, joints {model.joint_count} "
          f"(loop: {[joint_names[k] for k in loop_joints]}), MuJoCo nv {mj.nv} neq {mj.neq} nu {mj.nu}, "
          f"driven dofs {int((ke > 0).sum())}")

    q_start = model.joint_q_start.numpy()
    qi = lambda name: int(q_start[joint_names.index(name)])  # noqa: E731
    bi = lambda name: body_names.index(name)  # noqa: E731
    X_p, X_c = model.joint_X_p.numpy(), model.joint_X_c.numpy()
    parents, children = model.joint_parent.numpy(), model.joint_child.numpy()

    def anchor_world(pose, xform):
        return pose[:3] + quat_to_matrix(pose[3:]) @ xform[:3]

    state0, state1, control = model.state(), model.state(), model.control()
    newton.eval_fk(model, model.joint_q, model.joint_qd, state0)
    worst = {"residual_mm": 0.0, "pip_err_deg": 0.0, "track_err_deg": 0.0}
    for target in np.linspace(0.0, 45.0, 10):
        tq = control.joint_target_q.numpy()
        for d in FINGERS:
            tq[qi(f"{d}_pip_servo")] = math.radians(target)
        control.joint_target_q.assign(tq)
        for _ in range(int(args.settle / args.dt)):
            solver.step(state0, state1, control, None, args.dt)
            state0, state1 = state1, state0
        jq, bq = state0.joint_q.numpy(), state0.body_q.numpy()
        sample = 0.0
        for k in loop_joints:
            r = np.linalg.norm(anchor_world(bq[parents[k]], X_p[k]) - anchor_world(bq[children[k]], X_c[k]))
            sample = max(sample, 1e3 * float(r))
        worst["residual_mm"] = max(worst["residual_mm"], sample)
        if args.verbose:
            print(f"  servo target {target:5.1f}: residual {sample:.5f} mm")
        for d in FINGERS:
            servo = math.degrees(jq[qi(f"{d}_pip_servo")])
            axis = np.array(desc.joint(f"{d}_pip_servo")["axis"])
            pip = rotation_about(quat_to_matrix(bq[bi(f"{d}_proximal")][3:]), quat_to_matrix(bq[bi(f"{d}_distal")][3:]),
                                 axis)
            worst["pip_err_deg"] = max(worst["pip_err_deg"], abs(pip - desc.pip_from_servo_joint(d, servo)))
            worst["track_err_deg"] = max(worst["track_err_deg"], abs(servo - target))
        if math.isclose(target, 45.0):
            print(f"  at servo 45 deg: {d} PIP {pip:.3f}, model {desc.pip_from_servo_joint(d, servo):.3f}")
    print("  worst:", {k: round(float(v), 5) for k, v in worst.items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
