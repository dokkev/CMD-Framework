# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

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
import time
from pathlib import Path

import numpy as np

from code_isaaclab.assets import DEFAULT_DESCRIPTION_PATH, Description, quat_to_matrix, rotation_about
from code_isaaclab.loops import FourBarTables, loop_residual_mm, resolve_newton_solver, stiffen_mujoco_loops
from code_isaaclab.physics import SOLVER_BACKENDS, solver_physics_cfg


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--description", type=Path, default=DEFAULT_DESCRIPTION_PATH)
    parser.add_argument("--usd", type=Path, default=DEFAULT_DESCRIPTION_PATH.parent / "code_hand_ball.usda")
    parser.add_argument("--physics", choices=SOLVER_BACKENDS, default="mjwarp")
    parser.add_argument("--num-envs", type=int, default=256)
    parser.add_argument("--gravity", action="store_true")
    parser.add_argument("--dt", type=float, default=1 / 240)
    parser.add_argument("--substeps", type=int, default=2)
    parser.add_argument("--settle", type=float, default=0.4)
    parser.add_argument(
        "--actuator",
        choices=("implicit", "explicit"),
        default="implicit",
        help="servo PD in the solver (implicit) or computed by Isaac Lab as torque (explicit)",
    )
    parser.add_argument(
        "--servo-armature",
        type=float,
        default=1e-5,
        help="reflected rotor inertia on the servo joints [kg m^2] (passive joints keep 1e-5)",
    )
    parser.add_argument("--kd", type=float, default=0.05)
    args = parser.parse_args(argv)

    import torch

    from isaaclab.actuators import IdealPDActuatorCfg, ImplicitActuatorCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sim import SimulationCfg, build_simulation_context
    from isaaclab.utils import configclass

    from code_isaaclab.hands.code_hand import code_hand_articulation_cfg

    desc = Description.load(args.description)
    fingers = desc.four_bar_digits()
    actuated = desc.actuated()
    servo_cls = ImplicitActuatorCfg if args.actuator == "implicit" else IdealPDActuatorCfg

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        robot = code_hand_articulation_cfg(
            desc,
            args.usd,
            servo_actuator=servo_cls(
                joint_names_expr=actuated,
                stiffness=5.0,
                damping=args.kd,
                effort_limit_sim=1.0,
                armature=args.servo_armature,
            ),
        )

    gravity = (0.0, 0.0, -9.81) if args.gravity else (0.0, 0.0, 0.0)
    physics = solver_physics_cfg(args.physics, num_substeps=args.substeps, mjwarp_njmax=40, mjwarp_nconmax=10)
    sim_cfg = SimulationCfg(dt=args.dt, device="cuda:0", gravity=gravity, physics=physics)
    tag = (
        f"{args.physics:12s} {args.actuator:8s} arm={args.servo_armature:g} "
        f"gravity={'on ' if args.gravity else 'off'} envs={args.num_envs}"
    )
    with build_simulation_context(sim_cfg=sim_cfg) as sim:
        scene = InteractiveScene(SceneCfg(num_envs=args.num_envs, env_spacing=0.5))
        sim.reset()
        robot = scene["robot"]
        if args.physics == "mjwarp_stiff":
            stiffen_mujoco_loops(resolve_newton_solver(sim), args.dt / args.substeps, min_timeconst=0.0)

        names, bodies = robot.joint_names, robot.body_names
        jid = {n: names.index(n) for n in names}
        servo_ids = [jid[f"{d}_pip_servo"] for d in fingers]
        axis = {d: np.asarray(desc.joint(f"{d}_pip_servo")["axis"], dtype=float) for d in fingers}

        def step(n_steps: int) -> None:
            for _ in range(n_steps):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(sim.cfg.dt)

        def measure(envs: list[int]) -> dict[str, object]:
            pos = robot.data.body_pos_w.torch.cpu().numpy()
            rot = robot.data.body_quat_w.torch.cpu().numpy()
            jq = np.degrees(robot.data.joint_pos.torch.cpu().numpy())
            finite = bool(np.isfinite(pos).all() and np.isfinite(rot).all())
            out: dict[str, object] = {
                "residual_mm": 0.0,
                "pip_err_deg": 0.0,
                "servo_deg": [],
                "jointpos_err_deg": 0.0,
                "finite": finite,
            }
            if not finite:
                return out
            out["residual_mm"] = loop_residual_mm(desc, bodies, pos[envs], rot[envs])
            servos: list[float] = []
            pip_err = jointpos_err = 0.0
            for env in envs:
                for d in fingers:
                    r_prox = quat_to_matrix(rot[env, bodies.index(f"{d}_proximal")])
                    r_servo = quat_to_matrix(rot[env, bodies.index(f"{d}_servo_link")])
                    r_dist = quat_to_matrix(rot[env, bodies.index(f"{d}_distal")])
                    servo = rotation_about(r_prox, r_servo, axis[d])
                    b_joint = rotation_about(r_servo, r_dist, axis[d])
                    pip = rotation_about(r_prox, r_dist, axis[d])
                    pip_err = max(pip_err, abs(pip - desc.pip_from_servo_joint(d, servo)))
                    servos.append(servo)
                    jointpos_err = max(
                        jointpos_err,
                        abs(jq[env, jid[f"{d}_pip_servo"]] - servo),
                        abs(jq[env, jid[f"{d}_b"]] - b_joint),
                    )
            out["servo_deg"] = servos
            out["pip_err_deg"] = pip_err
            out["jointpos_err_deg"] = jointpos_err
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
            worst["residual_mm"] = max(worst["residual_mm"], float(m["residual_mm"]))
            worst["pip_err_deg"] = max(worst["pip_err_deg"], float(m["pip_err_deg"]))
            worst["track_err_deg"] = max(worst["track_err_deg"], max(abs(s - target) for s in m["servo_deg"]))
            worst["jointpos_err_deg"] = max(worst["jointpos_err_deg"], float(m["jointpos_err_deg"]))

        # Reset to PIP = 60 deg: closed tree in the first half, servo joint only in the second half.
        device = robot.data.joint_pos.torch.device
        tables = FourBarTables.from_description(desc, names, device=device)
        full = torch.zeros((n, len(names)), device=device)
        closed_half = tables.close_from_pip(full[: n // 2], math.radians(60.0))
        full[: n // 2] = closed_half
        full[n // 2 :, servo_ids] = closed_half[0, servo_ids]
        robot.write_joint_position_to_sim_index(position=full)
        robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(full))
        goal = torch.zeros_like(full)
        goal[:, servo_ids] = full[0, servo_ids]
        robot.set_joint_position_target(goal)
        step(1)
        closed_1, servo_only_1 = measure([0, n // 2 - 1]), measure([n // 2, n - 1])
        step(steps)
        closed_s, servo_only_s = measure([0, n // 2 - 1]), measure([n // 2, n - 1])
        first_digit = fingers[0]
        servo_60_deg = math.degrees(float(full[0, jid[f"{first_digit}_pip_servo"]]))
        expected = desc.pip_from_servo_joint(first_digit, servo_60_deg)

        def fmt(m: dict[str, object]) -> str:
            if not m["finite"]:
                return "non-finite"
            return f"{float(m['residual_mm']):.4f} mm / PIP err {float(m['pip_err_deg']):.3f} deg"

        print(
            f"{tag} | ramp: residual {worst['residual_mm']:.5f} mm, PIP err {worst['pip_err_deg']:.4f} deg, "
            f"servo track {worst['track_err_deg']:.4f} deg, joint_pos vs bodies {worst['jointpos_err_deg']:.4f} deg"
            f" | {n * count / wall:,.0f} env-steps/s"
        )
        print(
            f"{'':12s} reset to PIP 60 (servo {servo_60_deg:.2f} deg, PIP {expected:.2f}): "
            f"closed tree -> 1 step {fmt(closed_1)}, settled {fmt(closed_s)}; servo only -> 1 step "
            f"{fmt(servo_only_1)}, settled {fmt(servo_only_s)}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
