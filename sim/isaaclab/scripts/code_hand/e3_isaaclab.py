"""E3: the CoDe hand USD (loop cut excluded from the articulation) as an Isaac Lab Articulation on Newton.

    uv run python scripts/code_hand/e3_isaaclab.py [--usd .../code_hand_ball.usda] [--num-envs 64] [--gravity]
        [--eq-timeconst 0.004 --eq-solimp 0.99 0.999]

Reports what Isaac Lab sees (joints, bodies, which joints the loop cut became) and, over a ramp of the PIP servo
targets in every environment, the B' closure residual and the PIP angle against the exported closure table.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from description import DEFAULT, Description, quat_to_matrix, rotation_about  # noqa: E402

FINGERS = ("index", "middle", "ring")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--description", type=Path, default=DEFAULT)
    parser.add_argument("--usd", type=Path, default=DEFAULT.parent / "code_hand_ball.usda")
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

    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sim import SimulationCfg, build_simulation_context
    from isaaclab.utils import configclass

    from code_isaaclab.physics import HandPhysicsCfg

    desc = Description.load(args.description)
    actuated = desc.actuated()
    passive = [j["name"] for j in desc.joints if j["actuator"] is None]

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim_utils.UsdFileCfg(usd_path=str(args.usd)),
            init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.3), joint_pos={".*": 0.0}),
            actuators={
                "servos": ImplicitActuatorCfg(joint_names_expr=actuated, stiffness=5.0, damping=0.05,
                                              effort_limit_sim=1.0, armature=args.armature),
                "passive": ImplicitActuatorCfg(joint_names_expr=passive, stiffness=0.0, damping=0.0,
                                               armature=args.armature),
            },
        )

    physics = HandPhysicsCfg().newton_mjwarp
    physics.solver_cfg.njmax, physics.solver_cfg.nconmax = 200, 50
    gravity = (0.0, 0.0, -9.81) if args.gravity else (0.0, 0.0, 0.0)
    sim_cfg = SimulationCfg(dt=args.dt, device="cuda:0", gravity=gravity, physics=physics)
    to_t = lambda v: v if isinstance(v, torch.Tensor) else wp.to_torch(v)  # noqa: E731
    with build_simulation_context(sim_cfg=sim_cfg) as sim:
        scene = InteractiveScene(SceneCfg(num_envs=args.num_envs, env_spacing=0.5))
        sim.reset()
        robot = scene["robot"]
        print(f"Isaac Lab: {robot.num_joints} joints, {robot.num_bodies} bodies, {args.num_envs} envs")
        print("  joints:", robot.joint_names)
        missing = [j for j in actuated + passive if j not in robot.joint_names]
        extra = [j for j in robot.joint_names if j not in actuated + passive]
        print("  missing tree joints:", missing, "| joints not in the tree (loop cuts leaking in?):", extra)

        manager = sim.physics_manager if hasattr(sim, "physics_manager") else None
        solver = None
        for owner in (manager, getattr(manager, "_solver", None)):
            if owner is not None and hasattr(owner, "solver"):
                solver = owner.solver
        if solver is None:
            from isaaclab_newton.physics import NewtonManager
            solver = getattr(NewtonManager, "_solver", None)
        mj = getattr(solver, "mj_model", None)
        if mj is not None:
            print(f"  MuJoCo: nv {mj.nv}, neq {mj.neq} (types {sorted(set(int(t) for t in mj.eq_type))}), nu {mj.nu}")
            if args.eq_timeconst is not None or args.eq_solimp is not None:
                solref, solimp = solver.mjw_model.eq_solref.numpy(), solver.mjw_model.eq_solimp.numpy()
                if args.eq_timeconst is not None:
                    solref[..., 0] = args.eq_timeconst
                if args.eq_solimp is not None:
                    solimp[..., 0], solimp[..., 1] = args.eq_solimp
                solver.mjw_model.eq_solref.assign(solref)
                solver.mjw_model.eq_solimp.assign(solimp)
                print("  eq_solref/solimp set:", solref.reshape(-1, 2)[0], solimp.reshape(-1, 5)[0][:2])
        else:
            print("  (MuJoCo model not reachable from here; equality stiffness left at its default)")

        names = robot.joint_names
        bodies = robot.body_names
        servo_ids = [names.index(f"{d}_pip_servo") for d in FINGERS]
        quat = to_t(robot.data.body_quat_w)[0, bodies.index("palm")].cpu().numpy()
        w_last = abs(quat[3]) > abs(quat[0])
        print("  quaternion order:", "xyzw" if w_last else "wxyz", quat.round(4))
        xyzw = (lambda q: q) if w_last else (lambda q: np.r_[q[1:], q[0]])

        cut_offsets = {}
        for cut in desc.loops:
            anchor = np.array(cut["anchor"])
            cut_offsets[cut["name"]] = (anchor - np.array(desc.body(cut["parent"])["origin"]),
                                        anchor - np.array(desc.body(cut["child"])["origin"]))
        worst = {"residual_mm": 0.0, "pip_err_deg": 0.0, "track_err_deg": 0.0, "env_spread_deg": 0.0}
        steps = int(args.settle / args.dt)
        for target in np.linspace(0.0, 45.0, 10):
            goal = torch.zeros_like(to_t(robot.data.joint_pos))
            goal[:, servo_ids] = math.radians(target)
            robot.set_joint_position_target(goal)
            for _ in range(steps):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(sim.cfg.dt)
            jq = to_t(robot.data.joint_pos).cpu().numpy()
            pos = to_t(robot.data.body_pos_w).cpu().numpy()
            rot = to_t(robot.data.body_quat_w).cpu().numpy()
            for env in (0, args.num_envs - 1):
                R = lambda b, env=env, rot=rot: quat_to_matrix(xyzw(rot[env, bodies.index(b)]))  # noqa: E731
                for d in FINGERS:
                    cut = f"{d}_bp"
                    p_off, c_off = cut_offsets[cut]
                    drv, dist = f"{d}_driver_link", f"{d}_distal"
                    a = pos[env, bodies.index(drv)] + R(drv) @ p_off
                    b = pos[env, bodies.index(dist)] + R(dist) @ c_off
                    worst["residual_mm"] = max(worst["residual_mm"], 1e3 * float(np.linalg.norm(a - b)))
                    servo = math.degrees(jq[env, names.index(f"{d}_pip_servo")])
                    pip = rotation_about(R(f"{d}_proximal"), R(dist), np.array(desc.joint(f"{d}_pip_servo")["axis"]))
                    worst["pip_err_deg"] = max(worst["pip_err_deg"], abs(pip - desc.pip_from_servo_joint(d, servo)))
                    worst["track_err_deg"] = max(worst["track_err_deg"], abs(servo - target))
            spread = np.degrees(jq[:, servo_ids].max(0) - jq[:, servo_ids].min(0)).max()
            worst["env_spread_deg"] = max(worst["env_spread_deg"], float(spread))
        print(f"  at servo 45 deg, env 0: index PIP {pip:.3f} vs model {desc.pip_from_servo_joint(d, servo):.3f}")
        print("  worst:", {k: round(float(v), 5) for k, v in worst.items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
