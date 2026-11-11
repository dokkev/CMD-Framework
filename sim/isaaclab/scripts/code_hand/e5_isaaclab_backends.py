"""E5: the CoDe hand in Isaac Lab on each Newton backend: closure, joint-state consistency, reset, throughput.

    uv run python scripts/code_hand/e5_isaaclab_backends.py --physics mjwarp|mjwarp_stiff|kamino|kamino_dvi|xpbd|vbd
        [--num-envs 256] [--gravity]

Per backend, with every PIP servo ramped from 0 to 45 deg:
- closure: worst B' residual and PIP error against the exported closure table (both from body poses);
- joint state: how far ``robot.data.joint_pos`` (what a policy observes) is from the body poses, for the servo
  joint and the passive joint B;
- reset: half the environments are written to PIP = 60 deg with the whole closed tree (A, A', B from the table),
  the other half with the servo joint only (A, B left at 0), as a task's reset might. Residual one step later and
  after settling;
- throughput: environment steps per second over the ramp.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from description import DEFAULT, Description, quat_to_matrix, rotation_about  # noqa: E402

FINGERS = ("index", "middle", "ring")
BACKENDS = ("mjwarp", "mjwarp_stiff", "kamino", "kamino_dvi", "xpbd", "vbd", "featherstone")


def physics_cfg(name: str, args):
    from isaaclab_newton.physics import (
        FeatherstoneSolverCfg,
        KaminoDVISolverCfg,
        KaminoPADMMSolverCfg,
        MJWarpSolverCfg,
        NewtonCfg,
        VBDSolverCfg,
        XPBDSolverCfg,
    )

    if name.startswith("mjwarp"):
        solver = MJWarpSolverCfg(integrator="implicitfast", njmax=40, nconmax=10, impratio=10.0, cone="elliptic")
    elif name == "kamino":
        solver = KaminoPADMMSolverCfg(use_fk_solver=True)
    elif name == "kamino_dvi":
        solver = KaminoDVISolverCfg(use_fk_solver=True)
    elif name == "xpbd":
        solver = XPBDSolverCfg(iterations=30)
    elif name == "vbd":
        solver = VBDSolverCfg(iterations=10, class_type="code_isaaclab.vbd_manager:NewtonVBDJointSyncManager")
    elif name == "featherstone":
        solver = FeatherstoneSolverCfg()
    else:
        raise ValueError(name)
    return NewtonCfg(solver_cfg=solver, num_substeps=args.substeps)


def stiffen_mujoco_loops(solver, dt: float) -> None:
    solref, solimp = solver.mjw_model.eq_solref.numpy(), solver.mjw_model.eq_solimp.numpy()
    solref[..., 0] = 2.0 * dt
    solimp[..., 0], solimp[..., 1] = 0.99, 0.999
    solver.mjw_model.eq_solref.assign(solref)
    solver.mjw_model.eq_solimp.assign(solimp)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--description", type=Path, default=DEFAULT)
    parser.add_argument("--usd", type=Path, default=DEFAULT.parent / "code_hand_ball.usda")
    parser.add_argument("--physics", choices=BACKENDS, default="mjwarp")
    parser.add_argument("--num-envs", type=int, default=256)
    parser.add_argument("--gravity", action="store_true")
    parser.add_argument("--dt", type=float, default=1 / 240)
    parser.add_argument("--substeps", type=int, default=2)
    parser.add_argument("--settle", type=float, default=0.4)
    parser.add_argument("--actuator", choices=("implicit", "explicit"), default="implicit",
                        help="servo PD in the solver (implicit) or computed by Isaac Lab as torque (explicit)")
    parser.add_argument("--servo-armature", type=float, default=1e-5,
                        help="reflected rotor inertia on the servo joints [kg m^2] (passive joints keep 1e-5)")
    parser.add_argument("--kd", type=float, default=0.05)
    args = parser.parse_args(argv)

    import torch

    import isaaclab.sim as sim_utils
    from isaaclab.actuators import IdealPDActuatorCfg, ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sim import SimulationCfg, build_simulation_context
    from isaaclab.utils import configclass

    desc = Description.load(args.description)
    actuated = desc.actuated()
    passive = [j["name"] for j in desc.joints if j["actuator"] is None]
    servo_cls = ImplicitActuatorCfg if args.actuator == "implicit" else IdealPDActuatorCfg

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim_utils.UsdFileCfg(usd_path=str(args.usd)),
            init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.3), joint_pos={".*": 0.0}),
            actuators={
                "servos": servo_cls(joint_names_expr=actuated, stiffness=5.0, damping=args.kd,
                                              effort_limit_sim=1.0, armature=args.servo_armature),
                "passive": ImplicitActuatorCfg(joint_names_expr=passive, stiffness=0.0, damping=0.0, armature=1e-5),
            },
        )

    gravity = (0.0, 0.0, -9.81) if args.gravity else (0.0, 0.0, 0.0)
    sim_cfg = SimulationCfg(dt=args.dt, device="cuda:0", gravity=gravity, physics=physics_cfg(args.physics, args))
    tag = (f"{args.physics:12s} {args.actuator:8s} arm={args.servo_armature:g} "
           f"gravity={'on ' if args.gravity else 'off'} envs={args.num_envs}")
    with build_simulation_context(sim_cfg=sim_cfg) as sim:
        scene = InteractiveScene(SceneCfg(num_envs=args.num_envs, env_spacing=0.5))
        sim.reset()
        robot = scene["robot"]
        if args.physics == "mjwarp_stiff":
            from isaaclab_newton.physics import NewtonManager
            stiffen_mujoco_loops(NewtonManager._solver, args.dt / args.substeps)

        names, bodies = robot.joint_names, robot.body_names
        jid = {n: names.index(n) for n in names}
        servo_ids = [jid[f"{d}_pip_servo"] for d in FINGERS]
        axis = {d: np.array(desc.joint(f"{d}_pip_servo")["axis"]) for d in FINGERS}
        offsets = {}
        for cut in desc.loops:
            anchor = np.array(cut["anchor"])
            offsets[cut["child"].split("_")[0]] = (anchor - np.array(desc.body(cut["parent"])["origin"]),
                                                   anchor - np.array(desc.body(cut["child"])["origin"]))

        def step(n: int) -> None:
            for _ in range(n):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(sim.cfg.dt)

        def measure(envs) -> dict[str, float]:
            pos = robot.data.body_pos_w.torch.cpu().numpy()
            rot = robot.data.body_quat_w.torch.cpu().numpy()
            jq = np.degrees(robot.data.joint_pos.torch.cpu().numpy())
            out = {"residual_mm": 0.0, "pip_err_deg": 0.0, "servo_deg": [], "jointpos_err_deg": 0.0,
                   "finite": bool(np.isfinite(pos).all() and np.isfinite(rot).all())}
            if not out["finite"]:
                return out
            for env in envs:
                R = lambda b, env=env: quat_to_matrix(rot[env, bodies.index(b)])  # noqa: E731
                for d in FINGERS:
                    p_off, c_off = offsets[d]
                    a = pos[env, bodies.index(f"{d}_driver_link")] + R(f"{d}_driver_link") @ p_off
                    b = pos[env, bodies.index(f"{d}_distal")] + R(f"{d}_distal") @ c_off
                    out["residual_mm"] = max(out["residual_mm"], 1e3 * float(np.linalg.norm(a - b)))
                    servo = rotation_about(R(f"{d}_proximal"), R(f"{d}_servo_link"), axis[d])
                    b_joint = rotation_about(R(f"{d}_servo_link"), R(f"{d}_distal"), axis[d])
                    pip = rotation_about(R(f"{d}_proximal"), R(f"{d}_distal"), axis[d])
                    out["pip_err_deg"] = max(out["pip_err_deg"], abs(pip - desc.pip_from_servo_joint(d, servo)))
                    out["servo_deg"].append(servo)
                    out["jointpos_err_deg"] = max(out["jointpos_err_deg"], abs(jq[env, jid[f"{d}_pip_servo"]] - servo),
                                                  abs(jq[env, jid[f"{d}_b"]] - b_joint))
            return out

        steps = int(round(args.settle / args.dt))
        n = args.num_envs
        sample = sorted({0, n // 2 - 1, n // 2, n - 1})
        worst = {"residual_mm": 0.0, "pip_err_deg": 0.0, "track_err_deg": 0.0, "jointpos_err_deg": 0.0}
        wall, count = 0.0, 0
        for target in np.linspace(0.0, 45.0, 10):
            goal = torch.zeros_like(robot.data.joint_pos.torch)
            goal[:, servo_ids] = math.radians(target)
            robot.set_joint_position_target(goal)
            torch.cuda.synchronize()
            start = time.perf_counter()
            step(steps)
            torch.cuda.synchronize()
            wall += time.perf_counter() - start
            count += steps
            m = measure(sample)
            if not m["finite"]:
                print(f"{tag} | NON-FINITE during the ramp at {target:.1f} deg")
                return 1
            worst["residual_mm"] = max(worst["residual_mm"], m["residual_mm"])
            worst["pip_err_deg"] = max(worst["pip_err_deg"], m["pip_err_deg"])
            worst["track_err_deg"] = max(worst["track_err_deg"], max(abs(s - target) for s in m["servo_deg"]))
            worst["jointpos_err_deg"] = max(worst["jointpos_err_deg"], m["jointpos_err_deg"])

        # Reset to PIP = 60 deg: closed tree in the first half, servo joint only in the second half.
        pose = {}
        for d in FINGERS:
            pose.update(desc.closed_pose(d, 60.0))
        full = torch.zeros((n, len(names)), device=robot.data.joint_pos.torch.device)
        for name, value in pose.items():
            full[: n // 2, jid[name]] = math.radians(value)
        for d in FINGERS:
            full[n // 2:, jid[f"{d}_pip_servo"]] = math.radians(pose[f"{d}_pip_servo"])
        robot.write_joint_position_to_sim_index(position=full)
        robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(full))
        goal = torch.zeros_like(full)
        goal[:, servo_ids] = full[0, servo_ids]
        robot.set_joint_position_target(goal)
        step(1)
        closed_1, servo_only_1 = measure([0, n // 2 - 1]), measure([n // 2, n - 1])
        step(steps)
        closed_s, servo_only_s = measure([0, n // 2 - 1]), measure([n // 2, n - 1])
        expected = desc.pip_from_servo_joint("index", pose["index_pip_servo"])

        def fmt(m):
            if not m["finite"]:
                return "non-finite"
            return f"{m['residual_mm']:.4f} mm / PIP err {m['pip_err_deg']:.3f} deg"

        print(f"{tag} | ramp: residual {worst['residual_mm']:.5f} mm, PIP err {worst['pip_err_deg']:.4f} deg, "
              f"servo track {worst['track_err_deg']:.4f} deg, joint_pos vs bodies {worst['jointpos_err_deg']:.4f} deg"
              f" | {n * count / wall:,.0f} env-steps/s")
        print(f"{'':12s} reset to PIP 60 (servo {pose['index_pip_servo']:.2f} deg, PIP {expected:.2f}): "
              f"closed tree -> 1 step {fmt(closed_1)}, settled {fmt(closed_s)}; servo only -> 1 step "
              f"{fmt(servo_only_1)}, settled {fmt(servo_only_s)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
