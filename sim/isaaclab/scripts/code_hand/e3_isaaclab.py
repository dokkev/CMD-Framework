# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""E3: the CoDe hand USD (loop cut excluded from the articulation) as an Isaac Lab Articulation on Newton.

    uv run python scripts/code_hand/e3_isaaclab.py [--usd .../code_hand_ball.usda] [--num-envs 64] [--gravity]
        [--eq-timeconst 0.004 --eq-solimp 0.99 0.999]

Reports what Isaac Lab sees (joints, bodies, which joints the loop cut became) and, over a ramp of the PIP servo
targets in every environment, the B' closure residual and the PIP angle against the exported closure table.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from code_isaaclab.assets import DEFAULT_DESCRIPTION_PATH, Description
from code_isaaclab.loops import loop_residual_mm, measure_finger_pip_deg, resolve_newton_solver, stiffen_mujoco_loops


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--description", type=Path, default=DEFAULT_DESCRIPTION_PATH)
    parser.add_argument("--usd", type=Path, default=DEFAULT_DESCRIPTION_PATH.parent / "code_hand_ball.usda")
    parser.add_argument("--num-envs", type=int, default=64)
    parser.add_argument("--gravity", action="store_true")
    parser.add_argument("--armature", type=float, default=1e-5)
    parser.add_argument("--eq-timeconst", type=float)
    parser.add_argument("--eq-solimp", type=float, nargs=2)
    parser.add_argument("--dt", type=float, default=1 / 240)
    parser.add_argument("--settle", type=float, default=0.4)
    args = parser.parse_args(argv)

    import torch
    import warp as wp

    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sim import SimulationCfg, build_simulation_context
    from isaaclab.utils import configclass

    from code_isaaclab.hands.code_hand import code_hand_articulation_cfg
    from code_isaaclab.physics import HandPhysicsCfg

    desc = Description.load(args.description)
    fingers = desc.four_bar_digits()
    actuated = desc.actuated()
    passive = desc.passive()

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        robot = code_hand_articulation_cfg(
            desc,
            args.usd,
            servo_actuator=ImplicitActuatorCfg(
                joint_names_expr=actuated,
                stiffness=5.0,
                damping=0.05,
                effort_limit_sim=1.0,
                armature=args.armature,
            ),
            passive_armature=args.armature,
        )

    physics = HandPhysicsCfg().newton_mjwarp
    physics.solver_cfg.njmax, physics.solver_cfg.nconmax = 200, 50
    gravity = (0.0, 0.0, -9.81) if args.gravity else (0.0, 0.0, 0.0)
    sim_cfg = SimulationCfg(dt=args.dt, device="cuda:0", gravity=gravity, physics=physics)

    def to_tensor(value):
        return value if isinstance(value, torch.Tensor) else wp.to_torch(value)

    with build_simulation_context(sim_cfg=sim_cfg) as sim:
        scene = InteractiveScene(SceneCfg(num_envs=args.num_envs, env_spacing=0.5))
        sim.reset()
        robot = scene["robot"]
        print(f"Isaac Lab: {robot.num_joints} joints, {robot.num_bodies} bodies, {args.num_envs} envs")
        print("  joints:", robot.joint_names)
        missing = [j for j in actuated + passive if j not in robot.joint_names]
        extra = [j for j in robot.joint_names if j not in actuated + passive]
        print("  missing tree joints:", missing, "| joints not in the tree (loop cuts leaking in?):", extra)

        solver = resolve_newton_solver(sim)
        mj = getattr(solver, "mj_model", None)
        if mj is not None:
            print(f"  MuJoCo: nv {mj.nv}, neq {mj.neq} (types {sorted({int(t) for t in mj.eq_type})}), nu {mj.nu}")
            if (args.eq_timeconst is not None or args.eq_solimp is not None) and mj.neq:
                default_tc = float(solver.mjw_model.eq_solref.numpy().reshape(-1, 2)[0, 0])
                default_imp = tuple(solver.mjw_model.eq_solimp.numpy().reshape(-1, 5)[0, :2])
                stiffen_mujoco_loops(
                    solver,
                    args.dt / physics.num_substeps,
                    timeconst=default_tc if args.eq_timeconst is None else args.eq_timeconst,
                    solimp=default_imp if args.eq_solimp is None else tuple(args.eq_solimp),
                )
                print(
                    "  eq_solref/solimp set:",
                    solver.mjw_model.eq_solref.numpy().reshape(-1, 2)[0],
                    solver.mjw_model.eq_solimp.numpy().reshape(-1, 5)[0][:2],
                )
        else:
            print("  (MuJoCo model not reachable from here; equality stiffness left at its default)")

        names = robot.joint_names
        bodies = robot.body_names
        servo_ids = [names.index(f"{d}_pip_servo") for d in fingers]
        quat = to_tensor(robot.data.body_quat_w)[0, bodies.index("palm")].cpu().numpy()
        w_last = abs(quat[3]) > abs(quat[0])
        print("  quaternion order:", "xyzw" if w_last else "wxyz", quat.round(4))

        worst = {"residual_mm": 0.0, "pip_err_deg": 0.0, "track_err_deg": 0.0, "env_spread_deg": 0.0}
        steps = int(args.settle / args.dt)
        sample_envs = sorted({0, args.num_envs - 1})
        pip = servo = 0.0
        for target in np.linspace(0.0, 45.0, 10):
            goal = torch.zeros_like(to_tensor(robot.data.joint_pos))
            goal[:, servo_ids] = math.radians(target)
            robot.set_joint_position_target(goal)
            for _ in range(steps):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(sim.cfg.dt)
            jq = to_tensor(robot.data.joint_pos).cpu().numpy()
            pos = to_tensor(robot.data.body_pos_w).cpu().numpy()
            rot = to_tensor(robot.data.body_quat_w).cpu().numpy()
            if not w_last:
                rot = np.concatenate((rot[..., 1:], rot[..., :1]), axis=-1)
            worst["residual_mm"] = max(
                worst["residual_mm"], loop_residual_mm(desc, bodies, pos[sample_envs], rot[sample_envs])
            )
            for env in sample_envs:
                for d in fingers:
                    servo = math.degrees(jq[env, names.index(f"{d}_pip_servo")])
                    pip = measure_finger_pip_deg(desc, d, bodies, rot[env])
                    worst["pip_err_deg"] = max(worst["pip_err_deg"], abs(pip - desc.pip_from_servo_joint(d, servo)))
                    worst["track_err_deg"] = max(worst["track_err_deg"], abs(servo - target))
            spread = np.degrees(jq[:, servo_ids].max(0) - jq[:, servo_ids].min(0)).max()
            worst["env_spread_deg"] = max(worst["env_spread_deg"], float(spread))
        if fingers:
            last = fingers[-1]
            expected_last = desc.pip_from_servo_joint(last, servo)
            print(f"  at servo 45 deg, env 0: {last} PIP {pip:.3f} vs model {expected_last:.3f}")
        print("  worst:", {k: round(float(v), 5) for k, v in worst.items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
