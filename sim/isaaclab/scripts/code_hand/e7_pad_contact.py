# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""E7: the finger pads as colliders: self-contact while curling, and pressing a fixed rod at the servo's stall torque.

    uv run python scripts/code_hand/e7_pad_contact.py --physics mjwarp_stiff|kamino
        [--description .../code_hand_pivot8.json] [--usd .../code_hand_pivot8_ball_colliders.usda]

The hand is the description with pads (``design/scripts/export/code_hand_sim.py``, ``--params`` the
``pivot8_balanced.yaml`` variant) built by ``make_assets.py --colliders``: capsules and pad boxes collide,
except the description's filtered pairs (within a digit, but not a finger's proximal and distal segments).
Servos are the XM335 ``dc_motor`` model, gravity is off, and contact forces come from Isaac Lab's ContactSensor
on every hand body.

Tests (all fingers at once):
- ``sweep``: no object; every PIP servo ramps the PIP from 0 to 119 deg (``--sweep-to``). Any contact force on a
  hand body is self-contact (the pads are designed to clear each other by 0.04 mm at the closest). The exported
  PIP limits end at 120 deg; a target at that end diverges on Kamino (loop and two joint limits at once).
- ``distal press``: a fixed rod (axis across the fingers) touches each distal pad's face centre at PIP 40 deg;
  the PIP servo is commanded 15 deg past that, so it stalls against the rod.
- ``proximal press``: a rod touches each proximal pad's face centre at FE 30 deg; the FE servo is commanded
  45 deg, the PIP servo holds 0.

Each press is checked by virtual work in the proximal (distal press) or palm (proximal press) frame, with the
measured contact force ``F`` (rod on finger) at the contact point ``P`` (the pad-face point nearest the rod axis)
and the driving servo's applied torque ``tau``:

    distal:    tau * d(theta_servo)/d(pip) + F . dP/d(pip) = 0     (dP/d(pip) from the exported four-bar table)
    proximal:  tau_fe + u_fe . ((P - O) x F) = 0
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from code_isaaclab.assets import (
    DEFAULT_DESCRIPTION_PATH,
    Description,
    quat_to_matrix,
    quat_wxyz_to_matrix,
)
from code_isaaclab.loops import (
    loop_residual_mm,
    measure_finger_pip_deg,
    resolve_newton_solver,
    stiffen_mujoco_loops,
)
from code_isaaclab.physics import SOLVER_BACKENDS, solver_physics_cfg

PIVOT8 = DEFAULT_DESCRIPTION_PATH.parent / "code_hand_pivot8.json"
ROD_RADIUS = 0.008
PIP_CONTACT_DEG, PIP_PAST_DEG = 40.0, 15.0
FE_CONTACT_DEG, FE_TARGET_DEG = 30.0, 45.0


def rotate(points: np.ndarray, anchor: np.ndarray, axis: np.ndarray, deg: float) -> np.ndarray:
    """Rotate points [m] by ``deg`` CCW about ``axis`` through ``anchor`` (Rodrigues)."""
    u = axis / np.linalg.norm(axis)
    t = math.radians(deg)
    k = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    r = np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k
    return (np.asarray(points) - anchor) @ r.T + anchor


class Finger:
    """One finger's geometry from the description, in the hand frame at the geometric zero."""

    def __init__(self, desc: Description, digit: str) -> None:
        self.desc, self.digit = desc, digit
        self.servo = desc.joint(f"{digit}_pip_servo")
        self.b = desc.joint(f"{digit}_b")
        self.fe = desc.joint(f"{digit}_fe")
        self.u = np.asarray(self.servo["axis"], dtype=float)
        self.pads = {}
        for seg in ("proximal", "distal"):
            body = desc.body(f"{digit}_{seg}")
            box = body["boxes"][0]
            self.pads[seg] = {
                "body": body,
                "centre": np.asarray(box["center"], dtype=float),
                "rot": quat_wxyz_to_matrix(box["quat_wxyz"]),
                "half": np.asarray(box["half_extents"], dtype=float),
            }

    def angles(self, pip_deg: float) -> tuple[float, float]:
        ap = self.desc.servo_joint_from_pip(self.digit, pip_deg)
        return ap, pip_deg - ap

    def distal_points(self, points: np.ndarray, pip_deg: float) -> np.ndarray:
        """Distal-body points (given at zero) at ``pip_deg``, in the proximal frame at zero (FE 0)."""
        ap, b = self.angles(pip_deg)
        moved = rotate(points, np.asarray(self.b["anchor"], dtype=float), self.u, b)
        return rotate(moved, np.asarray(self.servo["anchor"], dtype=float), self.u, ap)

    def face_centre(self, seg: str) -> tuple[np.ndarray, np.ndarray]:
        """The pad's contact face centre and outward (palmar) normal, at zero."""
        pad = self.pads[seg]
        n = pad["rot"][:, 1]
        return pad["centre"] + pad["half"][1] * n, n


def rod_pose(fingers: dict[str, Finger], test: str) -> tuple[np.ndarray, float]:
    """Rod axis point [m, hand frame] touching every finger's pad face centre in the test pose, and rod length."""
    points = []
    for f in fingers.values():
        if test == "distal":
            face, n = f.face_centre("distal")
            face = f.distal_points(face[None], PIP_CONTACT_DEG)[0]
            n = rotate(n[None], np.zeros(3), f.u, PIP_CONTACT_DEG)[0]
        else:
            face, n = f.face_centre("proximal")
            o = np.asarray(f.fe["anchor"], dtype=float)
            u = np.asarray(f.fe["axis"], dtype=float)
            face = rotate(face[None], o, u, FE_CONTACT_DEG)[0]
            n = rotate(n[None], np.zeros(3), u, FE_CONTACT_DEG)[0]
        points.append(face + ROD_RADIUS * n)
    arr = np.array(points)
    return arr.mean(axis=0), float(np.ptp(arr[:, 1]) + 0.03)


def run(test: str, args, desc: Description, fingers: dict[str, Finger]) -> None:
    import torch

    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab.sim import SimulationCfg, build_simulation_context
    from isaaclab.utils import configclass

    from code_isaaclab.actuators import xm335_gains
    from code_isaaclab.hands.code_hand import code_hand_articulation_cfg

    gains = xm335_gains(desc.servo)
    hand_pos = np.array([0.0, 0.0, 0.3])
    rod = None if test == "sweep" else rod_pose(fingers, test)

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        robot = code_hand_articulation_cfg(
            desc,
            args.usd,
            servo_mode="dc_motor",
            init_pos=tuple(hand_pos),
            activate_contact_sensors=True,
        )
        contacts = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*")

    if rod is not None:
        SceneCfg.rod = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Rod",
            spawn=sim_utils.CylinderCfg(
                radius=ROD_RADIUS,
                height=rod[1],
                axis="Y",
                collision_props=sim_utils.CollisionPropertiesCfg(),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(pos=tuple(hand_pos + rod[0])),
        )

    physics = solver_physics_cfg(args.physics, num_substeps=args.substeps, mjwarp_njmax=300, mjwarp_nconmax=100)
    sim_cfg = SimulationCfg(dt=args.dt, device="cuda:0", gravity=(0.0, 0.0, 0.0), physics=physics)
    with build_simulation_context(sim_cfg=sim_cfg) as sim:
        scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=0.5))
        sim.reset()
        robot, sensor = scene["robot"], scene["contacts"]
        if args.physics == "mjwarp_stiff":
            stiffen_mujoco_loops(resolve_newton_solver(sim), args.dt / args.substeps, min_timeconst=0.0)
        names, bodies = robot.joint_names, robot.body_names
        sensor_bodies = sensor.body_names
        jid = {n: names.index(n) for n in names}
        root = robot.data.root_pos_w.torch[0].cpu().numpy()

        def step(seconds: float, target: torch.Tensor, watch: set[str] | None = None) -> float:
            worst = 0.0
            for _ in range(int(round(seconds / args.dt))):
                robot.set_joint_position_target(target)
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(sim.cfg.dt)
                f_mag = np.linalg.norm(sensor.data.net_forces_w.torch[0].cpu().numpy(), axis=-1)
                for i, name in enumerate(sensor_bodies):
                    if watch is None or name not in watch:
                        worst = max(worst, float(f_mag[i]))
            return worst

        def state() -> tuple[np.ndarray, np.ndarray]:
            return (
                robot.data.body_pos_w.torch[0].cpu().numpy() - root,
                robot.data.body_quat_w.torch[0].cpu().numpy(),
            )

        target = torch.zeros((1, robot.num_joints), device=robot.data.joint_pos.torch.device)
        print(f"== {args.physics} / {test}")
        if test == "sweep":
            step(0.3, target)
            worst = 0.0
            for pip in np.linspace(0.0, args.sweep_to, 61):
                for d, f in fingers.items():
                    target[0, jid[f"{d}_pip_servo"]] = math.radians(f.angles(float(pip))[0])
                worst = max(worst, step(0.05, target))
            step(0.5, target)
            pos, quat = state()
            worst_hold = step(0.2, target)
            pips = [measure_finger_pip_deg(desc, d, bodies, quat) for d in fingers]
            print(
                f"  reached PIP {', '.join(f'{p:.1f}' for p in pips)} deg | largest hand contact force during the "
                f"sweep {worst:.4f} N, holding at {args.sweep_to:g} deg {worst_hold:.4f} N | loop residual "
                f"{loop_residual_mm(desc, bodies, pos, quat):.4f} mm"
            )
            return

        assert rod is not None
        seg = "distal" if test == "distal" else "proximal"
        step(0.2, target)
        for d, f in fingers.items():
            if test == "distal":
                target[0, jid[f"{d}_pip_servo"]] = math.radians(f.angles(PIP_CONTACT_DEG)[0] + PIP_PAST_DEG)
            else:
                target[0, jid[f"{d}_fe"]] = math.radians(FE_TARGET_DEG)
        pressed = {f"{d}_{seg}" for d in fingers}
        stray = step(1.5, target, watch=pressed)
        forces = sensor.data.net_forces_w.torch[0].cpu().numpy()
        torque = robot.data.applied_torque.torch[0].cpu().numpy()
        pos, quat = state()
        axis_point = rod[0]
        for d, f in fingers.items():
            body = f"{d}_{seg}"
            F = forces[sensor_bodies.index(body)]
            R = quat_to_matrix(quat[bodies.index(body)])
            origin = np.asarray(desc.body(body)["origin"], dtype=float)
            pad = f.pads[seg]
            q = R.T @ (axis_point - pos[bodies.index(body)]) + origin
            s = pad["rot"].T @ (q - pad["centre"])
            local = np.array([np.clip(s[0], -pad["half"][0], pad["half"][0]), pad["half"][1], 0.0])
            P0 = pad["centre"] + pad["rot"] @ local
            n_w = R @ pad["rot"][:, 1]
            normal = -float(F @ n_w)
            friction = float(np.linalg.norm(F + normal * n_w))
            if test == "distal":
                Rp = quat_to_matrix(quat[bodies.index(f"{d}_proximal")])
                Fp = Rp.T @ F
                pip = measure_finger_pip_deg(desc, d, bodies, quat)
                h = 0.25
                dP = (f.distal_points(P0[None], pip + h)[0] - f.distal_points(P0[None], pip - h)[0]) / math.radians(
                    2 * h
                )
                dtheta = (f.angles(pip + h)[0] - f.angles(pip - h)[0]) / (2 * h)
                tau = float(torque[jid[f"{d}_pip_servo"]])
                servo_term, contact_term = tau * dtheta, float(Fp @ dP)
                fe = math.degrees(float(robot.data.joint_pos.torch[0, jid[f"{d}_fe"]]))
                from_b = 1e3 * float((P0 - np.asarray(f.b["anchor"], dtype=float)) @ pad["rot"][:, 0])
                where = (
                    f"PIP {pip:6.2f} deg, FE {fe:6.2f} deg (tau {float(torque[jid[f'{d}_fe']]):6.3f}), "
                    f"contact {from_b:5.1f} mm from B"
                )
            else:
                o = np.asarray(f.fe["anchor"], dtype=float)
                P_w = pos[bodies.index(body)] + R @ (P0 - origin)
                tau = float(torque[jid[f"{d}_fe"]])
                servo_term = tau
                contact_term = float(np.asarray(f.fe["axis"], dtype=float) @ np.cross(P_w - o, F))
                fe = math.degrees(float(robot.data.joint_pos.torch[0, jid[f"{d}_fe"]]))
                where = f"FE {fe:6.2f} deg, contact {1e3 * float((P0 - o) @ pad['rot'][:, 0]):5.1f} mm from O"
            residual = (servo_term + contact_term) / abs(servo_term) if servo_term else float("nan")
            print(
                f"  {d:6s} {where} | tau {tau:6.3f} N m (stall {gains['stall']:.2f}) | normal {normal:6.3f} N, "
                f"friction {friction:5.3f} N | virtual-work residual {100 * residual:6.2f} %"
            )
        print(
            f"  loop residual {loop_residual_mm(desc, bodies, pos, quat):.4f} mm | "
            f"largest contact force on other hand bodies {stray:.4f} N"
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--description", type=Path, default=PIVOT8)
    parser.add_argument("--usd", type=Path, default=PIVOT8.parent / "code_hand_pivot8_ball_colliders.usda")
    parser.add_argument("--physics", choices=SOLVER_BACKENDS, default="mjwarp_stiff")
    parser.add_argument("--dt", type=float, default=1 / 240)
    parser.add_argument("--substeps", type=int, default=2)
    parser.add_argument("--tests", default="sweep,distal,proximal")
    parser.add_argument(
        "--sweep-to",
        type=float,
        default=119.0,
        help="PIP end of the sweep [deg]; the exported limits stop at 120, where Kamino diverges",
    )
    args = parser.parse_args(argv)

    desc = Description.load(args.description)
    fingers = {d: Finger(desc, d) for d in desc.four_bar_digits()}
    for test in args.tests.split(","):
        run(test, args, desc, fingers)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
