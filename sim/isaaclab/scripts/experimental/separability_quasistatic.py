# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Experimental (not in the SSoT): can an interface effect be measured apart from the morphology change a thicker
interface forces? A quasi-static flexion sweep against a held cylinder, no learning.

    uv run python scripts/experimental/separability_quasistatic.py --hand O|R|U --shape flat|crowned --radius_mm 15

Hands (Study 3's, CFL, 120 deg): O = H0 (3 mm pads), R = H_R (7 mm, redesigned keeping the relative contact
trajectory), U = H_U (7 mm, redesigned for coverage alone). Shape: the pads' contact face flat or crowned (2 mm
sagitta), the same on every hand: the intended interface change.

A kinematic cylinder (axis across the fingers, 70 mm long) is held on each hand's proximal pads, at the place where
O's two pads would both touch it at flexion q0 in {95, 105, 115} deg (one environment each; positions from the design
kinematics), at the same height above that hand's own proximal face: the MCP-level offset of the thicker pads (a
stated limitation) is left out, so only the finger-internal relative contact trajectory differs between hands.
The PIPs are commanded in flexion (``FlexionEMAAction``, alpha 1, flexion stiffness and damping matched across hands,
servo effort unlimited) from 60 to 120 deg in 0.5 deg steps, three env steps each; MCP and abduction at 0.

Writes ``logs/experimental/separability/<run>.json``: per environment and step, commanded and measured flexion and
each pad's contact force.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import sys
from pathlib import Path

import gymnasium as gym
import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "study3"))
import common  # noqa: E402

HANDS = {k: common.REPO_ROOT / "design" / "output" / "sim" / "study3" / "hands" / f"cfl_q120_h3to7_{v}.json"
         for k, v in (("O", "h0"), ("R", "hr"), ("U", "hu"))}
OUT = common.SIM_ROOT / "logs" / "experimental" / "separability"
# x_c [mm from the MCP axis along the proximal face] where O's two pads touch a cylinder of radius rho at q0
# (design kinematics, e16's construction; 2026-11-21)
GRASPS = {12: {95: 30.9, 105: 28.5, 115: 25.7}, 15: {95: 27.6, 105: 24.6, 115: 21.0}, 18: {95: 24.4, 105: 20.7, 115: 16.3}}
MCP_X_M = 0.0375  # the middle finger's MCP flexion axis, hand x
FINGER_Y_M = -0.025  # middle finger, hand y
HAND_Z = 0.5
LENGTH_M = 0.07
K_Q, D_Q = 5.0, 0.08  # flexion stiffness [N m/rad] and damping [N m s/rad], every hand
SWEEP = np.arange(60.0, 120.0 + 1e-9, 0.5)
HOLD = 3


def proximal_face_z(desc: dict) -> float:
    box = next(b for body in desc["bodies"] for b in (body.get("boxes") or []) if b["name"] == "middle_proximal_pad")
    return float(box["center"][2]) - float(box["half_extents"][1])


def main(argv=None) -> int:
    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hand", required=True, choices=tuple(HANDS))
    parser.add_argument("--shape", default="flat", choices=("flat", "crowned"))
    parser.add_argument("--radius_mm", type=int, default=15, choices=tuple(GRASPS))
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args

    from isaaclab.app import launch_simulation
    from isaaclab.utils import configclass

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.contact_interface import CROWNED, FLAT
    from code_isaaclab.flexion_control import FlexionEMAActionCfg
    from code_isaaclab.hands import get_hand
    from code_isaaclab.hands.description_hands import cylinder_cfg, register_description_hand
    from code_isaaclab.tasks.reorient.code_env_cfg import CoDeReorientMainEnvCfg

    physics = next((t.split("=", 1)[1] for t in hydra_args if t.startswith("physics=")), "newton_mjwarp")
    path = HANDS[args.hand]
    desc = json.loads(path.read_text())
    rho = args.radius_mm * 1e-3
    face = proximal_face_z(desc)
    q0s = sorted(GRASPS[args.radius_mm])
    xs = [MCP_X_M + GRASPS[args.radius_mm][q] * 1e-3 for q in q0s]
    z_w = HAND_Z - (face - rho)
    name = register_description_hand(f"sx_{args.hand}_{args.shape}_{args.radius_mm}", path,
                                     cylinder_cfg(rho, LENGTH_M, 0.05, (xs[0], -FINGER_Y_M, z_w), kinematic=True),
                                     profile=CROWNED if args.shape == "crowned" else FLAT)

    @configclass
    class Cfg(CoDeReorientMainEnvCfg):
        hand: str = name

    env_cfg = Cfg()
    env_cfg.sim.physics = getattr(env_cfg.sim.physics, physics)
    env_cfg.scene.num_envs = len(q0s)
    env_cfg.events.reset_object = None
    env_cfg.events.reset_hand.params["joint_position_noise"] = 0.0
    env_cfg.terminations.object_out_of_reach = None
    env_cfg.episode_length_s = 1.0e4
    old = env_cfg.actions.joint_pos
    env_cfg.actions.joint_pos = FlexionEMAActionCfg(asset_name="robot", joint_names=old.joint_names, alpha=1.0,
                                                    four_bar_tables=get_hand(name).four_bar_tables,
                                                    flexion_limits_deg=(0.0, 120.0), flexion_stiffness=K_Q,
                                                    flexion_damping=D_Q)
    bodies = common.add_sensors(env_cfg, physics)
    args.device = env_cfg.sim.device

    rows = []
    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make("CoDe-Reorient-Cube-CoDeFlat-Main", cfg=env_cfg)
        cleanup.callback(env.close)
        env.reset(seed=0)
        u = env.unwrapped
        obj = u.scene["object"]
        n = u.num_envs
        pose = torch.zeros(n, 7, device=u.device)
        pose[:, 0] = torch.tensor(xs, device=u.device)
        pose[:, 1] = -FINGER_Y_M
        pose[:, 2] = z_w
        pose[:, :3] += u.scene.env_origins
        pose[:, 6] = 1.0  # quaternion xyzw identity
        obj.write_root_pose_to_sim_index(root_pose=pose, env_ids=torch.arange(n, device=u.device))
        probe = common.Probe(env, bodies, description=path)
        term = u.action_manager.get_term("joint_pos")
        names = list(term._joint_names)
        lo, hi = term._cmd_limits[0, :, 0], term._cmd_limits[0, :, 1]
        pip = [k for k, nm in enumerate(names) if nm.endswith("_pip_servo") and not nm.startswith("thumb")]
        servo_ids = [u.scene["robot"].find_joints(names[k])[0][0] for k in pip]

        def command(q_deg: float) -> torch.Tensor:
            cmd = torch.zeros(n, len(names), device=u.device)
            cmd[:, pip] = math.radians(q_deg)
            return 2.0 * (cmd - lo) / (hi - lo) - 1.0

        schedule = list(np.linspace(0.0, SWEEP[0], 30)) + [q for q in SWEEP for _ in range(HOLD)]
        for step, q in enumerate(schedule):
            env.step(command(float(q)))
            obj.write_root_pose_to_sim_index(root_pose=pose, env_ids=torch.arange(n, device=u.device))
            if step < 30:
                continue
            m = probe.step_metrics()
            theta = u.scene["robot"].data.joint_pos.torch[:, servo_ids]
            q_meas = torch.stack([term._tables.flexion(names[k], theta[:, i]) for i, k in enumerate(pip)], dim=1)
            rows.append({"q_cmd_deg": float(q), "q_meas_deg": np.degrees(q_meas.cpu().numpy()).tolist(),
                         "pad_force_n": m["pad_force_n"].cpu().numpy().tolist(),
                         "loop_mm": m["loop_residual_mm"].cpu().numpy().tolist()})

    result = {"hand": args.hand, "shape": args.shape, "radius_mm": args.radius_mm, "q0_deg": q0s, "x_mcp_m": xs,
              "proximal_face_z_m": face, "flexion_stiffness": K_Q, "flexion_damping": D_Q, "physics": physics,
              "pads": common.PAD_BODIES, "description": str(path)}
    run = common.run_id(f"sep_{args.hand}_{args.shape}_r{args.radius_mm}")
    out = common.write_json(OUT / f"{run}.json", {**result, "rows": rows, "manifest": common.manifest(command=sys.argv)})
    last = rows[-1]
    print(json.dumps({**{k: v for k, v in result.items() if k not in ("description",)},
                      "final_pad_force_n": last["pad_force_n"], "loop_mm_max": max(max(r["loop_mm"]) for r in rows)}))
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
