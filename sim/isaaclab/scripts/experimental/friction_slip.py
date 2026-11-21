# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Experimental (playground, not the SSoT): is a pad-friction effect measured alike after each redesign?

    uv run python scripts/experimental/friction_slip.py --hand O|R|U --mu 0.4|0.8 --condition A|B [--radius_mm 15]

Hands O, R, U (Study 3, CFL, 120 deg, 3 -> 7 mm pads, flat). The pads carry a physics material of friction ``mu`` (the
interface property under test); the roller's own coefficient is 0.01, so the pads' decides (MuJoCo-Warp takes the
larger). A cylinder on an axle (``code_isaaclab.assets.roller``: free to spin about its axis, held otherwise) is
placed, per environment, where O's two pads would touch it at PIP flexion q0 in {95, 105, 115} deg, at the same height
above each hand's own proximal face (the MCP-level offset of the thicker pads is left out). PIP commanded in flexion
(flexion stiffness 5 N m/rad and damping 0.08 N m s/rad on every hand, servo effort unlimited), MCP and abduction 0.

- Condition A (fixed command): every hand flexes to q0 + ``--delta_deg`` (default 5 deg).
- Condition B (force-matched): starting from A's command, the PIP command is adjusted by force feedback until the
  roller's total contact force equals O's under condition A at the same mu and position (read from O's latest A run).

Then a torque about the axle rises at ``--rate`` N m/s, in each direction (one environment per q0 and direction);
T_slip is the torque at which the roller's speed first exceeds 0.5 rad/s (also recorded at 0.25 and 1.0). Before
slip the roller still turns, by the finger's compliance dragging the stuck pad (about 27 deg per N m on O, the same
at either mu); that creep is about 0.05 rad/s at the default rate, so a rotation threshold would catch it first.
Records, per environment: the pads' contact forces just before the torque (each pad body), T_slip, and the commanded
and measured flexion. Writes ``logs/experimental/friction/<run>.json``.
"""

from __future__ import annotations

import argparse
import contextlib
import glob
import json
import math
import sys
from pathlib import Path

import gymnasium as gym
import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "study3"))
sys.path.insert(0, str(HERE))
import common  # noqa: E402
from separability_quasistatic import FINGER_Y_M, GRASPS, HAND_Z, HANDS, MCP_X_M, proximal_face_z  # noqa: E402

OUT = common.SIM_ROOT / "logs" / "experimental" / "friction"
LENGTH_M, MASS_KG = 0.07, 0.05
K_Q, D_Q = 5.0, 0.08
SLIP_RAD_S = (0.25, 0.5, 1.0)  # roller speed [rad/s] taken as slip; stick-phase creep is about 0.05 rad/s
PRIMARY = 1  # index of the reported threshold (0.5 rad/s)
SIGNS = (1.0, -1.0)  # torque direction about the axle


def o_targets(mu: float, radius: int) -> list[float]:
    runs = sorted(glob.glob(str(OUT / f"fric_O_mu{mu:g}_A_r{radius}_*.json")))
    if not runs:
        raise FileNotFoundError(f"condition B needs O's condition A run at mu {mu:g}, radius {radius} mm")
    d = json.loads(Path(runs[-1]).read_text())
    return [e["force_total_n"] for e in d["envs"]]


def main(argv=None) -> int:
    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hand", required=True, choices=tuple(HANDS))
    parser.add_argument("--mu", type=float, required=True)
    parser.add_argument("--condition", default="A", choices=("A", "B"))
    parser.add_argument("--radius_mm", type=int, default=15, choices=tuple(GRASPS))
    parser.add_argument("--delta_deg", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=0.1, help="torque ramp [N m/s]")
    parser.add_argument("--max_torque", type=float, default=2.0, help="stop the ramp here [N m]")
    parser.add_argument("--full_trace", action="store_true", help="keep ramping after every roller has slipped")
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args

    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.app import launch_simulation
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sensors import ContactSensorCfg
    import isaaclab.sim as sim_utils
    from isaaclab.utils import configclass

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.assets.roller import write_roller_usd
    from code_isaaclab.flexion_control import FlexionEMAActionCfg
    from code_isaaclab.hands import get_hand
    from code_isaaclab.hands.code_variants import ASSET_DIR
    from code_isaaclab.hands.description_hands import cylinder_cfg, register_description_hand
    from code_isaaclab.tasks.reorient.code_env_cfg import CoDeReorientMainEnvCfg

    physics = next((t.split("=", 1)[1] for t in hydra_args if t.startswith("physics=")), "newton_mjwarp")
    path = HANDS[args.hand]
    desc = json.loads(path.read_text())
    rho = args.radius_mm * 1e-3
    face = proximal_face_z(desc)
    q0s = sorted(GRASPS[args.radius_mm])
    x_c = [GRASPS[args.radius_mm][q] * 1e-3 for q in q0s]
    targets = o_targets(args.mu, args.radius_mm) if args.condition == "B" else None
    roller_usd = write_roller_usd(ASSET_DIR / f"roller_r{args.radius_mm}.usda", rho, LENGTH_M, MASS_KG) \
        if not (ASSET_DIR / f"roller_r{args.radius_mm}.usda").exists() else ASSET_DIR / f"roller_r{args.radius_mm}.usda"
    name = register_description_hand(f"fs_{args.hand}_mu{args.mu:g}", path,
                                     cylinder_cfg(0.01, 0.02, 0.01, (0.3, 0.3, 0.1), kinematic=True),
                                     pad_friction=args.mu)

    @configclass
    class Cfg(CoDeReorientMainEnvCfg):
        hand: str = name

    env_cfg = Cfg()
    env_cfg.sim.physics = getattr(env_cfg.sim.physics, physics)
    env_cfg.scene.num_envs = len(q0s) * len(SIGNS)
    env_cfg.events.reset_object = None
    env_cfg.events.reset_hand.params["joint_position_noise"] = 0.0
    env_cfg.terminations.object_out_of_reach = None
    env_cfg.episode_length_s = 1.0e4
    old = env_cfg.actions.joint_pos
    env_cfg.actions.joint_pos = FlexionEMAActionCfg(asset_name="robot", joint_names=old.joint_names, alpha=1.0,
                                                    four_bar_tables=get_hand(name).four_bar_tables,
                                                    flexion_limits_deg=(0.0, 120.0), flexion_stiffness=K_Q,
                                                    flexion_damping=D_Q)
    env_cfg.scene.roller = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Roller",
        spawn=sim_utils.UsdFileCfg(usd_path=str(roller_usd)),
        init_state=ArticulationCfg.InitialStateCfg(pos=(MCP_X_M, -FINGER_Y_M, HAND_Z - (face - rho)),
                                                   joint_pos={"slide_x": 0.0, "spin": 0.0}),
        actuators={"slide": ImplicitActuatorCfg(joint_names_expr=["slide_x"], stiffness=1.0e5, damping=1.0e3,
                                                effort_limit_sim=1.0e4),
                   "spin": ImplicitActuatorCfg(joint_names_expr=["spin"], stiffness=0.0, damping=1.0e-4,
                                               effort_limit_sim=100.0)},
    )
    bodies = common.hand_bodies()
    env_cfg.scene.roller_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Roller/roller",
        filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{b}" for b in bodies])
    args.device = env_cfg.sim.device

    envs_out = []
    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make("CoDe-Reorient-Cube-CoDeFlat-Main", cfg=env_cfg)
        cleanup.callback(env.close)
        env.reset(seed=0)
        u = env.unwrapped
        n = u.num_envs
        roller, robot = u.scene["roller"], u.scene["robot"]
        sensor = u.scene.sensors["roller_contacts"]
        slide = roller.find_joints("slide_x")[0][0]
        spin = roller.find_joints("spin")[0][0]
        ids = torch.arange(n, device=u.device)
        env_q0 = [q for _ in SIGNS for q in q0s]
        env_x = [x for _ in SIGNS for x in x_c]
        env_sign = [s for s in SIGNS for _ in q0s]
        sign = torch.tensor(env_sign, device=u.device)
        xs = torch.tensor(env_x, device=u.device)
        pos = common.torch_of(roller.data.joint_pos).clone()
        pos[:, slide] = xs
        pos[:, spin] = 0.0
        roller.write_joint_position_to_sim_index(position=pos, env_ids=ids)
        roller.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(pos), env_ids=ids)
        target = torch.zeros_like(pos)
        target[:, slide] = xs
        roller.set_joint_position_target_index(target=target)
        term = u.action_manager.get_term("joint_pos")
        names = list(term._joint_names)
        lo, hi = term._cmd_limits[0, :, 0], term._cmd_limits[0, :, 1]
        pip = [k for k, nm in enumerate(names) if nm.endswith("_pip_servo") and not nm.startswith("thumb")]
        servo_ids = [robot.find_joints(names[k])[0][0] for k in pip]
        pad_cols = [bodies.index(b) for b in common.PAD_BODIES]

        def command(q_deg: torch.Tensor) -> torch.Tensor:
            cmd = torch.zeros(n, len(names), device=u.device)
            for k in pip:
                cmd[:, k] = torch.deg2rad(q_deg)
            return 2.0 * (cmd - lo) / (hi - lo) - 1.0

        def forces() -> torch.Tensor:  # (envs, pads) contact force magnitudes on the roller from each pad body
            matrix = common.torch_of(sensor.data.force_matrix_w)[:, 0]
            return matrix.norm(dim=-1)[:, pad_cols]

        def zero_torque():
            roller.set_joint_effort_target_index(target=torch.zeros(n, 1, device=u.device), joint_ids=[spin])

        q_cmd = torch.tensor([q + args.delta_deg for q in env_q0], device=u.device)
        for i in range(40):  # ramp the PIP to the command
            zero_torque()
            env.step(command(q_cmd * (i + 1) / 40))
        for _ in range(20):
            zero_torque()
            env.step(command(q_cmd))
        if targets is not None:
            goal = torch.tensor(targets, device=u.device)
            for _ in range(120):  # force feedback on the PIP command (about 2 N per deg near contact)
                zero_torque()
                env.step(command(q_cmd))
                q_cmd = torch.clamp(q_cmd + 0.1 * (goal - forces().sum(dim=1)), 60.0, 120.0)
            for _ in range(20):
                zero_torque()
                env.step(command(q_cmd))
        model = u.sim.physics_manager.get_model()
        labels = list(model.shape_label)
        mus = model.shape_material_mu.numpy()
        shape_mu = {lab: float(mus[k]) for k, lab in enumerate(labels)
                    if "env_0/" in lab and ("_pad" in lab or "Roller" in lab)}
        print("shape mu:", shape_mu)
        f_pre = forces().cpu().numpy()
        theta = common.torch_of(robot.data.joint_pos)[:, servo_ids]
        q_meas = torch.stack([term._tables.flexion(names[k], theta[:, i]) for i, k in enumerate(pip)], dim=1)
        angle0 = common.torch_of(roller.data.joint_pos)[:, spin].clone()
        slip = torch.full((len(SLIP_RAD_S), n), float("nan"), device=u.device)
        f_slip = torch.full((n,), float("nan"), device=u.device)
        dt = u.step_dt
        trace = []
        for i in range(int(args.max_torque / (args.rate * dt)) + 1):
            tau = args.rate * dt * (i + 1)
            roller.set_joint_effort_target_index(target=(tau * sign).unsqueeze(1), joint_ids=[spin])
            env.step(command(q_cmd))
            turned = (common.torch_of(roller.data.joint_pos)[:, spin] - angle0) * sign
            speed = common.torch_of(roller.data.joint_vel)[:, spin] * sign
            total = forces().sum(dim=1)
            trace.append([tau] + np.degrees(turned.cpu().numpy()).tolist() + speed.cpu().tolist() + total.cpu().tolist())
            for k, w in enumerate(SLIP_RAD_S):
                hit = torch.isnan(slip[k]) & (speed > w)
                if k == PRIMARY:
                    f_slip = torch.where(hit, total, f_slip)
                slip[k] = torch.where(hit, torch.full_like(slip[k], tau), slip[k])
            if not bool(torch.isnan(slip).any()) and not args.full_trace:
                break

        def value(x) -> float | None:
            return None if math.isnan(float(x)) else float(x)

        for e in range(n):
            envs_out.append({"q0_deg": env_q0[e], "torque_sign": env_sign[e], "x_c_m": env_x[e],
                             "q_cmd_deg": float(q_cmd[e]), "q_meas_deg": np.degrees(q_meas[e].cpu().numpy()).tolist(),
                             "pad_force_n": dict(zip(common.PAD_BODIES, f_pre[e].tolist())),
                             "force_total_n": float(f_pre[e].sum()), "force_distal_n": float(f_pre[e][1::2].sum()),
                             "force_proximal_n": float(f_pre[e][0::2].sum()),
                             "t_slip_nm": value(slip[PRIMARY, e]),
                             "t_slip_by_threshold": {f"{w:g}": value(slip[k, e]) for k, w in enumerate(SLIP_RAD_S)},
                             "force_at_slip_n": value(f_slip[e]),
                             "target_force_n": None if targets is None else targets[e]})

    result = {"hand": args.hand, "mu": args.mu, "condition": args.condition, "radius_mm": args.radius_mm,
              "delta_deg": args.delta_deg, "rate_nm_s": args.rate, "flexion_stiffness": K_Q, "flexion_damping": D_Q,
              "physics": physics, "shape_mu_env0": shape_mu, "envs": envs_out,
              "slip_speed_rad_s": SLIP_RAD_S[PRIMARY],
              "trace_columns": ["tau_nm"] + [f"{c}_env{e}" for c in ("turned_deg", "speed_rad_s", "force_n")
                                              for e in range(len(envs_out))],
              "trace": trace}
    run = common.run_id(f"fric_{args.hand}_mu{args.mu:g}_{args.condition}_r{args.radius_mm}")
    out = common.write_json(OUT / f"{run}.json", {**result, "manifest": common.manifest(command=sys.argv)})
    for e in envs_out:
        print(f"q0 {e['q0_deg']} sign {e['torque_sign']:+.0f}: q_cmd {e['q_cmd_deg']:.1f} N_total {e['force_total_n']:.2f} (distal {e['force_distal_n']:.2f})"
              f" T_slip {e['t_slip_nm']} N_slip {e['force_at_slip_n']}")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
