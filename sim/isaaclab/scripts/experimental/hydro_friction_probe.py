"""Where does hydroelastic contact lose friction? A minimal probe outside the hand (Newton + MuJoCo-Warp only).

Experiment 16 (report/study3/experiments.md) stopped because, on the hydroelastic path, the slipping pad's
|f_t|/|f_n| read 0.77 at mu 1.3 and 1.80 at mu 2.0 (point contact: 1.30, 2.00). This probe asks whether the engine
itself applies the set friction, without the hand, the grasp or Isaac Lab's contact sensor.

Scene: a fixed "pad" shape (mu = MU, soft: kh = 3.33e8 N/m^3 as in experiment 16) and an object on it (mu 0.01,
rigid kh 1e10; MuJoCo combines friction by max, so the pair's mu is MU), under gravity. The object may only translate
in x and z (a D6 joint with no rotation), so it can neither tip nor roll. A horizontal force on it ramps up; it slips
when the force exceeds mu_eff * m g. mu_eff = F at slip / (m g), read from the applied force alone, so no contact
sensor is involved.

Variants: point contact (MuJoCo's own collision) against hydroelastic with and without contact reduction and
normal matching; a box on a box (flat patch) and a cylinder on a box (curved patch, as the grasped cylinder).

    uv run python scripts/experimental/hydro_friction_probe.py
"""
import argparse
import math

import numpy as np
import warp as wp

import newton
from newton.geometry import HydroelasticSDF

G = 9.81
MASS = 0.2  # kg
DT, SUBSTEPS = 1.0 / 240.0, 4
RAMP_S = 3.0  # the push grows from 0 to 3 x mu m g over this time
SLIP_SPEED = 0.02  # m/s: the object counts as slipping above this speed


def build(mu: float, shape: str, contact: str, impratio: float, reduce: bool, normal_matching: bool,
          gap: float = 0.002):
    builder = newton.ModelBuilder(gravity=-G)
    hydro = contact != "point"
    pad_cfg = newton.ModelBuilder.ShapeConfig(mu=mu, is_hydroelastic=hydro, kh=3.33e8, sdf_max_resolution=64,
                                              sdf_narrow_band_range=(-0.005, 0.005), gap=gap)
    obj_cfg = newton.ModelBuilder.ShapeConfig(mu=0.01, is_hydroelastic=hydro, kh=1.0e10, sdf_max_resolution=64,
                                              sdf_narrow_band_range=(-0.005, 0.005), gap=gap)
    builder.add_shape_box(body=-1, xform=wp.transform((0.0, 0.0, -0.01), wp.quat_identity()), hx=0.1, hy=0.1, hz=0.01,
                          cfg=pad_cfg)
    z0 = 0.015
    body = builder.add_link(xform=wp.transform((0.0, 0.0, z0), wp.quat_identity()), mass=MASS)
    if shape == "box":
        builder.add_shape_box(body=body, hx=0.015, hy=0.015, hz=0.015, cfg=obj_cfg)
    else:  # a cylinder lying on its side, axis along y, radius 15 mm as the grasped one
        q = wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), math.pi / 2)
        builder.add_shape_cylinder(body=body, xform=wp.transform(wp.vec3(0.0, 0.0, 0.0), q), radius=0.015,
                                   half_height=0.02, cfg=obj_cfg)
    dof = newton.ModelBuilder.JointDofConfig
    joint = builder.add_joint_d6(parent=-1, child=body, linear_axes=[dof(axis=newton.Axis.X), dof(axis=newton.Axis.Z)],
                                 angular_axes=[], parent_xform=wp.transform((0.0, 0.0, z0), wp.quat_identity()))
    builder.add_articulation([joint])
    model = builder.finalize()
    # the object's mass from its density would differ; set it exactly
    big = hydro and not reduce
    solver = newton.solvers.SolverMuJoCo(model, use_mujoco_contacts=not hydro, njmax=40000 if big else 1200,
                                         nconmax=8000 if big else 300,
                                         cone="elliptic", impratio=impratio, integrator="implicitfast")
    pipeline = None
    if hydro:
        cfg = HydroelasticSDF.Config(reduce_contacts=reduce, normal_matching=normal_matching, buffer_fraction=1.0)
        pipeline = newton.CollisionPipeline(model, rigid_contact_max=8000 if not reduce else 300, broad_phase="explicit",
                                            sdf_hydroelastic_config=cfg)
    return model, solver, pipeline, body


def run(mu, shape, contact, impratio, reduce=True, normal_matching=True, gap=0.002):
    model, solver, pipeline, body = build(mu, shape, contact, impratio, reduce, normal_matching, gap)
    s0, s1, control = model.state(), model.state(), model.control()
    newton.eval_fk(model, model.joint_q, model.joint_qd, s0)
    contacts = pipeline.contacts() if pipeline is not None else None
    mass = float(model.body_mass.numpy()[body])
    weight = mass * G
    settle, ramp = int(1.0 / DT), int(RAMP_S / DT)
    f_slip, x0 = None, None
    for k in range(settle + ramp):
        push = 0.0 if k < settle else 3.0 * mu * weight * (k - settle) / ramp
        for _ in range(SUBSTEPS):
            s0.clear_forces()
            f = np.zeros((model.body_count, 6), dtype=np.float32)
            f[body, 0] = push
            s0.body_f.assign(f)
            if pipeline is not None:
                pipeline.collide(s0, contacts)
            solver.step(s0, s1, control, contacts, DT / SUBSTEPS)
            s0, s1 = s1, s0
        q, qd = s0.joint_q.numpy(), s0.joint_qd.numpy()
        if k == settle:
            x0 = q.copy()
        if k > settle and f_slip is None and abs(qd[0]) > SLIP_SPEED:
            f_slip = push
            break
    sink = None if x0 is None else float(-x0[1]) * 1e3  # the joint's z: how far the object sank into the pad
    return (None if f_slip is None else f_slip / weight), mass, sink


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--impratio", type=float, default=1000.0)
    p.add_argument("--gap_sweep", action="store_true", help="cylinder only, reduced hydroelastic, over the contact gap")
    args = p.parse_args()
    wp.init()
    if args.gap_sweep:
        print("gap_mm  mu   mu_eff  sink_mm")
        for gap in (0.0, 0.0005, 0.002, 0.01):
            for mu in (1.3, 2.0):
                mu_eff, _, sink = run(mu, "cylinder", "hydro", args.impratio, True, True, gap)
                print(f"{gap * 1e3:5.1f}  {mu:.1f}  {'no slip' if mu_eff is None else f'{mu_eff:.3f}'}  {sink:.2f}",
                      flush=True)
        return
    rows = []
    for shape in ("box", "cylinder"):
        for mu in (1.3, 2.0):
            for contact, reduce, nm in (("point", True, True), ("hydro", True, True), ("hydro", True, False),
                                        ("hydro", False, False)):
                try:
                    mu_eff, mass, sink = run(mu, shape, contact, args.impratio, reduce, nm)
                except Exception as e:  # report and go on
                    rows.append((shape, mu, contact, reduce, nm, f"error: {e!r}"[:80], "", ""))
                    continue
                rows.append((shape, mu, contact, reduce, nm, "no slip" if mu_eff is None else f"{mu_eff:.3f}",
                             f"{mass:.3f}", "" if sink is None else f"{sink:.2f}"))
                print(rows[-1], flush=True)
    print("\nshape    mu   contact reduce normal_match  mu_eff   mass_kg  sink_mm")
    for r in rows:
        print(f"{r[0]:8s} {r[1]:.1f}  {r[2]:7s} {str(r[3]):6s} {str(r[4]):12s} {r[5]:8s} {r[6]:8s} {r[7]}")


if __name__ == "__main__":
    main()
