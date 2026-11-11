"""E6: XM335 step responses on the CoDe hand in Isaac Lab, per servo model and physics backend.

    uv run python scripts/code_hand/e6_servo_steps.py --physics mjwarp_stiff|kamino --servo dc_motor|implicit
        [--gravity] [--armature 5e-3]

The servos are ``code_isaaclab.actuators.xm335_actuator_cfg`` built from the export's ``servo`` block. From rest
at zero, each test steps one joint's target and records the joint's motion:
- ``fe small``: index MCP-FE +2 deg (inside the linear range, below the 5 deg duty saturation);
- ``fe large``: index MCP-FE +60 deg (saturated: the speed should approach the no-load speed);
- ``pip large``: every PIP servo +40 deg (through the four-bar, PIP 0 to about 96 deg).
Reported per test: peak speed, time to 90 % of the step, overshoot and the final error. For the saturated steps the
dc_motor model's peak speed is bounded by the no-load speed, which the implicit model does not enforce.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from description import DEFAULT, Description  # noqa: E402
from e5_isaaclab_backends import physics_cfg, stiffen_mujoco_loops  # noqa: E402

FINGERS = ("index", "middle", "ring")
TESTS = (("fe small", ["index_fe"], 2.0, 0.5), ("fe large", ["index_fe"], 60.0, 0.8),
         ("pip large", [f"{d}_pip_servo" for d in FINGERS], 40.0, 0.8))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--description", type=Path, default=DEFAULT)
    parser.add_argument("--usd", type=Path, default=DEFAULT.parent / "code_hand_ball.usda")
    parser.add_argument("--physics", default="mjwarp_stiff")
    parser.add_argument("--servo", choices=("dc_motor", "implicit"), default="dc_motor")
    parser.add_argument("--armature", type=float, help="override the export's servo armature [kg m^2]")
    parser.add_argument("--gravity", action="store_true")
    parser.add_argument("--num-envs", type=int, default=4)
    parser.add_argument("--dt", type=float, default=1 / 240)
    parser.add_argument("--substeps", type=int, default=2)
    parser.add_argument("--json", type=Path, help="write the trajectories here")
    args = parser.parse_args(argv)

    import torch

    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sim import SimulationCfg, build_simulation_context
    from isaaclab.utils import configclass

    from code_isaaclab.actuators import xm335_actuator_cfg, xm335_gains

    desc = Description.load(args.description)
    servo = desc.data["servo"]
    gains = xm335_gains(servo)
    actuated = desc.actuated()
    passive = [j["name"] for j in desc.joints if j["actuator"] is None]

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim_utils.UsdFileCfg(usd_path=str(args.usd)),
            init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.3), joint_pos={".*": 0.0}),
            actuators={
                "servos": xm335_actuator_cfg(servo, actuated, mode=args.servo, armature=args.armature),
                "passive": ImplicitActuatorCfg(joint_names_expr=passive, stiffness=0.0, damping=0.0, armature=1e-5),
            },
        )

    args.settle = 0.4
    gravity = (0.0, 0.0, -9.81) if args.gravity else (0.0, 0.0, 0.0)
    sim_cfg = SimulationCfg(dt=args.dt, device="cuda:0", gravity=gravity, physics=physics_cfg(args.physics, args))
    print(f"{args.physics} / {args.servo}: kp {gains['kp']:.3f} N m/rad, kd {gains['kd']:.4f} N m s/rad, "
          f"stall {gains['stall']:.3f} N m, no-load {gains['no_load']:.3f} rad/s, "
          f"armature {args.armature if args.armature is not None else gains['armature']:g}")
    record = {}
    with build_simulation_context(sim_cfg=sim_cfg) as sim:
        scene = InteractiveScene(SceneCfg(num_envs=args.num_envs, env_spacing=0.5))
        sim.reset()
        robot = scene["robot"]
        if args.physics == "mjwarp_stiff":
            from isaaclab_newton.physics import NewtonManager
            stiffen_mujoco_loops(NewtonManager._solver, args.dt / args.substeps)
        names = robot.joint_names
        n = robot.num_joints

        def run(seconds: float, target: torch.Tensor, ids: list[int]):
            robot.set_joint_position_target(target)
            q, qd = [], []
            for _ in range(int(round(seconds / args.dt))):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(sim.cfg.dt)
                q.append(robot.data.joint_pos.torch[0, ids].cpu().numpy().copy())
                qd.append(robot.data.joint_vel.torch[0, ids].cpu().numpy().copy())
            return np.array(q), np.array(qd)

        for name, joints, step_deg, seconds in TESTS:
            ids = [names.index(j) for j in joints]
            zero = torch.zeros((args.num_envs, n), device=robot.data.joint_pos.torch.device)
            run(0.5, zero, ids)  # back to rest
            start = robot.data.joint_pos.torch[0, ids].cpu().numpy().copy()
            target = zero.clone()
            target[:, ids] = math.radians(step_deg)
            q, qd = run(seconds, target, ids)
            j = 0  # report the first joint of the group
            goal = math.radians(step_deg)
            rel = (q[:, j] - start[j]) / (goal - start[j])
            t90 = next((k * args.dt for k, r in enumerate(rel) if r >= 0.9), float("nan"))
            peak = float(np.abs(qd[:, j]).max())
            overshoot = max(0.0, float(rel.max() - 1.0)) * 100.0
            final = math.degrees(goal - q[-1, j])
            ok = np.isfinite(q).all()
            print(f"  {name:9s} step {step_deg:5.1f} deg | peak speed {peak:6.3f} rad/s "
                  f"({peak / gains['no_load'] * 100:5.1f} % of no-load) | t90 {t90 * 1e3:6.1f} ms | "
                  f"overshoot {overshoot:5.1f} % | final error {final:7.4f} deg{'' if ok else ' | NON-FINITE'}")
            record[name] = {"dt": args.dt, "q_rad": q[:, j].tolist(), "qd_rad_s": qd[:, j].tolist(),
                            "start_rad": float(start[j]), "goal_rad": goal}
    if args.json:
        args.json.write_text(json.dumps({"physics": args.physics, "servo": args.servo, "gains": gains,
                                         "gravity": args.gravity, "tests": record}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
