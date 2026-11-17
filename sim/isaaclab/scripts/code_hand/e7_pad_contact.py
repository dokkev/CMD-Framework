"""E7: the finger pads as colliders: self-contact while curling, and pressing a fixed rod at the servo's stall torque.

    uv run python scripts/code_hand/e7_pad_contact.py --physics mjwarp_stiff|kamino
        [--description .../code_hand_pivot8.json] [--usd .../code_hand_pivot8_ball_colliders.usda]

The hand is the description with pads (``design/scripts/export/code_hand_sim.py``, ``--params`` the
``pivot8_balanced.yaml`` variant) built by ``make_assets.py --colliders``: capsules and pad boxes collide,
except the description's filtered pairs (within a digit, but not a finger's proximal and distal segments).
Servos are the XM335 ``dc_motor`` model, gravity is off, and contact forces come from Isaac Lab's ContactSensor
on every hand body.

Tests (all three fingers at once):
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

reported as the residual relative to the servo term, with the normal force, the friction share, the loop
residual at B' and the largest contact force on any body other than the pressed one.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from description import DEFAULT, Description, quat_to_matrix  # noqa: E402
from e5_isaaclab_backends import stiffen_mujoco_loops  # noqa: E402

FINGERS = ("index", "middle", "ring")
PIVOT8 = DEFAULT.parent / "code_hand_pivot8.json"
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

    def __init__(self, desc: Description, digit: str):
        self.desc, self.digit = desc, digit
        self.servo = desc.joint(f"{digit}_pip_servo")
        self.b = desc.joint(f"{digit}_b")
        self.fe = desc.joint(f"{digit}_fe")
        self.u = np.array(self.servo["axis"])
        self.table = desc.data["four_bar"][digit]
        self.pads = {}
        for seg in ("proximal", "distal"):
            body = desc.body(f"{digit}_{seg}")
            box = body["boxes"][0]
            w, x, y, z = box["quat_wxyz"]
            self.pads[seg] = {"body": body, "centre": np.array(box["center"]),
                              "rot": quat_to_matrix(np.array([x, y, z, w])), "half": np.array(box["half_extents"])}

    def angles(self, pip_deg: float) -> tuple[float, float]:
        ap = float(np.interp(pip_deg, self.table["pip_deg"], self.table["ap_deg"]))
        return ap, pip_deg - ap

    def distal_points(self, points: np.ndarray, pip_deg: float) -> np.ndarray:
        """Distal-body points (given at zero) at ``pip_deg``, in the proximal frame at zero (FE 0)."""
        ap, b = self.angles(pip_deg)
        moved = rotate(points, np.array(self.b["anchor"]), self.u, b)
        return rotate(moved, np.array(self.servo["anchor"]), self.u, ap)

    def face_centre(self, seg: str) -> tuple[np.ndarray, np.ndarray]:
        """The pad's contact face centre and outward (palmar) normal, at zero."""
        pad = self.pads[seg]
        n = pad["rot"][:, 1]
        return pad["centre"] + pad["half"][1] * n, n


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--description", type=Path, default=PIVOT8)
    parser.add_argument("--usd", type=Path, default=PIVOT8.parent / "code_hand_pivot8_ball_colliders.usda")
    parser.add_argument("--physics", default="mjwarp_stiff")
    parser.add_argument("--dt", type=float, default=1 / 240)
    parser.add_argument("--substeps", type=int, default=2)
    parser.add_argument("--tests", default="sweep,distal,proximal")
    parser.add_argument("--sweep-to", type=float, default=119.0,
                        help="PIP end of the sweep [deg]; the exported limits stop at 120, where Kamino diverges")
    args = parser.parse_args(argv)

    desc = Description.load(args.description)
    fingers = {d: Finger(desc, d) for d in FINGERS}
    for test in args.tests.split(","):
        run(test, args, desc, fingers)
    return 0


def rod_pose(fingers: dict[str, Finger], test: str) -> tuple[np.ndarray, float]:
    """The rod's axis point [m, hand frame] touching every finger's pad face centre in the test's pose (the
    fingers' mean; they differ by tenths of a millimetre), and its length across them."""
    points = []
    for f in fingers.values():
        if test == "distal":
            face, n = f.face_centre("distal")
            face = f.distal_points(face[None], PIP_CONTACT_DEG)[0]
            n = rotate(n[None], np.zeros(3), f.u, PIP_CONTACT_DEG)[0]
        else:
            face, n = f.face_centre("proximal")
            o = np.array(f.fe["anchor"])
            u = np.array(f.fe["axis"])
            face = rotate(face[None], o, u, FE_CONTACT_DEG)[0]
            n = rotate(n[None], np.zeros(3), u, FE_CONTACT_DEG)[0]
        points.append(face + ROD_RADIUS * n)
    points = np.array(points)
    centre = points.mean(axis=0)
    return centre, float(np.ptp(points[:, 1]) + 0.03)


def run(test: str, args, desc: Description, fingers: dict[str, Finger]) -> None:
    import torch
    from e5_isaaclab_backends import physics_cfg

    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg, AssetBaseCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab.sim import SimulationCfg, build_simulation_context
    from isaaclab.utils import configclass

    from code_isaaclab.actuators import xm335_actuator_cfg, xm335_gains

    servo = desc.data["servo"]
    gains = xm335_gains(servo)
    actuated = desc.actuated()
    passive = [j["name"] for j in desc.joints if j["actuator"] is None]
    hand_pos = np.array([0.0, 0.0, 0.3])
    rod = None if test == "sweep" else rod_pose(fingers, test)

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim_utils.UsdFileCfg(usd_path=str(args.usd), activate_contact_sensors=True),
            init_state=ArticulationCfg.InitialStateCfg(pos=tuple(hand_pos), joint_pos={".*": 0.0}),
            actuators={
                "servos": xm335_actuator_cfg(servo, actuated, mode="dc_motor"),
                "passive": ImplicitActuatorCfg(joint_names_expr=passive, stiffness=0.0, damping=0.0, armature=1e-5),
            },
        )
        contacts = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*")

    if rod is not None:
        SceneCfg.rod = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Rod",
            spawn=sim_utils.CylinderCfg(radius=ROD_RADIUS, height=rod[1], axis="Y",
                                        collision_props=sim_utils.CollisionPropertiesCfg()),
            init_state=AssetBaseCfg.InitialStateCfg(pos=tuple(hand_pos + rod[0])))

    physics = physics_cfg(args.physics, args)
    if args.physics.startswith("mjwarp"):
        physics.solver_cfg.njmax, physics.solver_cfg.nconmax = 300, 100
    sim_cfg = SimulationCfg(dt=args.dt, device="cuda:0", gravity=(0.0, 0.0, 0.0), physics=physics)
    with build_simulation_context(sim_cfg=sim_cfg) as sim:
        scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=0.5))
        sim.reset()
        robot, sensor = scene["robot"], scene["contacts"]
        if args.physics == "mjwarp_stiff":
            from isaaclab_newton.physics import NewtonManager
            stiffen_mujoco_loops(NewtonManager._solver, args.dt / args.substeps)
        names, bodies = robot.joint_names, robot.body_names
        sensor_bodies = sensor.body_names
        jid = {n: names.index(n) for n in names}
        root = robot.data.root_pos_w.torch[0].cpu().numpy()

        def step(seconds: float, target: torch.Tensor, watch=None) -> float:
            """Step; return the largest contact force [N] seen on any hand body (``watch``: bodies to skip)."""
            worst = 0.0
            for _ in range(int(round(seconds / args.dt))):
                robot.set_joint_position_target(target)
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(sim.cfg.dt)
                f = np.linalg.norm(sensor.data.net_forces_w.torch[0].cpu().numpy(), axis=-1)
                for i, name in enumerate(sensor_bodies):
                    if watch is None or name not in watch:
                        worst = max(worst, float(f[i]))
            return worst

        target = torch.zeros((1, robot.num_joints), device=robot.data.joint_pos.torch.device)
        state = lambda: (robot.data.body_pos_w.torch[0].cpu().numpy() - root,  # noqa: E731
                         robot.data.body_quat_w.torch[0].cpu().numpy())

        def pip_of(d, pos, quat) -> float:
            from description import rotation_about
            R = lambda b: quat_to_matrix(quat[bodies.index(b)])  # noqa: E731
            return rotation_about(R(f"{d}_proximal"), R(f"{d}_distal"), fingers[d].u)

        def loop_residual_mm(pos, quat) -> float:
            worst = 0.0
            for cut in desc.loops:
                anchor = np.array(cut["anchor"])
                ends = []
                for side in ("parent", "child"):
                    b = cut[side]
                    ends.append(pos[bodies.index(b)] + quat_to_matrix(quat[bodies.index(b)])
                                @ (anchor - np.array(desc.body(b)["origin"])))
                worst = max(worst, 1e3 * float(np.linalg.norm(ends[0] - ends[1])))
            return worst

        print(f"== {args.physics} / {test}")
        if test == "sweep":
            step(0.3, target)
            worst = 0.0
            for pip in np.linspace(0.0, args.sweep_to, 61):
                for d, f in fingers.items():
                    target[0, jid[f"{d}_pip_servo"]] = math.radians(f.angles(pip)[0])
                worst = max(worst, step(0.05, target))
            step(0.5, target)
            pos, quat = state()
            worst_hold = step(0.2, target)
            pips = [pip_of(d, pos, quat) for d in FINGERS]
            print(f"  reached PIP {', '.join(f'{p:.1f}' for p in pips)} deg | largest hand contact force during the "
                  f"sweep {worst:.4f} N, holding at {args.sweep_to:g} deg {worst_hold:.4f} N | loop residual "
                  f"{loop_residual_mm(pos, quat):.4f} mm")
            return

        seg = "distal" if test == "distal" else "proximal"
        step(0.2, target)
        for d, f in fingers.items():
            if test == "distal":
                target[0, jid[f"{d}_pip_servo"]] = math.radians(f.angles(PIP_CONTACT_DEG)[0] + PIP_PAST_DEG)
            else:
                target[0, jid[f"{d}_fe"]] = math.radians(FE_TARGET_DEG)
        pressed = {f"{d}_{seg}" for d in FINGERS}
        stray = step(1.5, target, watch=pressed)
        forces = sensor.data.net_forces_w.torch[0].cpu().numpy()
        torque = robot.data.applied_torque.torch[0].cpu().numpy()
        pos, quat = state()
        axis_point = rod[0]
        for d, f in fingers.items():
            body = f"{d}_{seg}"
            F = forces[sensor_bodies.index(body)]
            R = quat_to_matrix(quat[bodies.index(body)])
            origin = np.array(desc.body(body)["origin"])
            pad = f.pads[seg]
            # The pad-face point nearest the rod axis, in the body's zero-pose (hand) coordinates.
            q = R.T @ (axis_point - pos[bodies.index(body)]) + origin
            s = pad["rot"].T @ (q - pad["centre"])
            local = np.array([np.clip(s[0], -pad["half"][0], pad["half"][0]), pad["half"][1], 0.0])
            P0 = pad["centre"] + pad["rot"] @ local
            n_w = R @ pad["rot"][:, 1]
            normal = -float(F @ n_w)
            friction = float(np.linalg.norm(F + normal * n_w))
            if test == "distal":
                # Work in the proximal body's frame: the finger's FE deflection drops out.
                Rp = quat_to_matrix(quat[bodies.index(f"{d}_proximal")])
                Fp = Rp.T @ F
                pip = pip_of(d, pos, quat)
                h = 0.25
                dP = ((f.distal_points(P0[None], pip + h)[0] - f.distal_points(P0[None], pip - h)[0])
                      / math.radians(2 * h))
                dtheta = (f.angles(pip + h)[0] - f.angles(pip - h)[0]) / (2 * h)
                tau = float(torque[jid[f"{d}_pip_servo"]])
                servo_term, contact_term = tau * dtheta, float(Fp @ dP)
                fe = math.degrees(float(robot.data.joint_pos.torch[0, jid[f"{d}_fe"]]))
                where = (f"PIP {pip:6.2f} deg, FE {fe:6.2f} deg (tau {float(torque[jid[f'{d}_fe']]):6.3f}), "
                         f"contact {1e3 * (P0 - np.array(f.b['anchor'])) @ pad['rot'][:, 0]:5.1f} mm from B")
            else:
                o = np.array(f.fe["anchor"])
                # The FE axis is fixed while AA holds 0: hand-frame axis and anchor as at zero.
                P_w = pos[bodies.index(body)] + R @ (P0 - origin)
                tau = float(torque[jid[f"{d}_fe"]])
                servo_term, contact_term = tau, float(np.array(f.fe["axis"]) @ np.cross(P_w - o, F))
                fe = math.degrees(float(robot.data.joint_pos.torch[0, jid[f"{d}_fe"]]))
                where = f"FE {fe:6.2f} deg, contact {1e3 * (P0 - o) @ pad['rot'][:, 0]:5.1f} mm from O"
            residual = (servo_term + contact_term) / abs(servo_term) if servo_term else float("nan")
            print(f"  {d:6s} {where} | tau {tau:6.3f} N m (stall {gains['stall']:.2f}) | normal {normal:6.3f} N, "
                  f"friction {friction:5.3f} N | virtual-work residual {100 * residual:6.2f} %")
        print(f"  loop residual {loop_residual_mm(pos, quat):.4f} mm | largest contact force on other hand bodies "
              f"{stray:.4f} N")


if __name__ == "__main__":
    raise SystemExit(main())
