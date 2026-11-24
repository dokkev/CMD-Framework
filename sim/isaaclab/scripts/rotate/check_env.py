# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Self-review of ``CoDe-Rotate-Cylinder-O`` before training: is the hand O, does the cylinder rest in the cradle,
does the rotation reward have the intended sign?

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run python scripts/rotate/check_env.py [--num_envs 8] physics=newton_mjwarp

1. Hand O: the description the hand was built from and its parameter file; in the simulation, the pose-invariant
   distances between the finger's joint axes (body origins sit on them) against the yaml's finger parameters
   (a_x, |A'|, apb, abp), and the proximal pad's contact face depth below the MCP axis against mount offset +
   thickness (from the cylinder's contact point on it).
2. Zero action (hold the cradle) for ``--hold_s`` seconds: the cylinder's displacement from its resting place, its
   speed and spin, its axis tilt, the pads touching it, drops.
3. Reward sign: the cylinder given a spin of -/+ ``--spin`` rad/s about world y while held; the rotation reward and the
   velocity of its top point (negative x: toward the palm).
4. Scripted curl: every PIP closed from the cradle by the full action range and back; the spin and the rotation
   reward it produces (which way a curling finger turns the cylinder).
"""

from __future__ import annotations

import argparse
import contextlib
import math
import sys
from pathlib import Path

import gymnasium as gym
import torch
import yaml

TASK = "CoDe-Rotate-Cylinder-O"
PAD_BODIES = [f"{f}_{s}" for f in ("index", "middle", "ring") for s in ("proximal", "distal")]


def torch_of(value):
    return value if isinstance(value, torch.Tensor) else value.torch


def main(argv=None) -> int:
    from isaaclab.app import add_launcher_args

    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--task", default=TASK, help="CoDe-Rotate-Cylinder-O, or -O/-R/-U-Smooth")
    parser.add_argument("--num_envs", type=int, default=8)
    parser.add_argument("--hold_s", type=float, default=5.0)
    parser.add_argument("--spin", type=float, default=2.0)
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args

    from isaaclab.app import launch_simulation
    from isaaclab.sensors import ContactSensorCfg

    from isaaclab_tasks.utils import resolve_task_config

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.assets.description import Description
    from code_isaaclab.hands import get_hand
    from code_isaaclab.tasks.rotate import rotate_mdp
    from code_isaaclab.tasks.rotate.env_cfg import CYLINDER_RADIUS_M, ROTATION_DIRECTION, SPIN_CLIP


    env_cfg, _ = resolve_task_config(args.task, "rsl_rl_cfg_entry_point")
    env_cfg.scene.num_envs = args.num_envs
    desc_path = Path(env_cfg.description)
    desc = Description.load(desc_path)
    bodies = [b["name"] for b in desc.bodies]
    env_cfg.scene.cyl_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/object", track_contact_points=True,
        filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{b}" for b in bodies])
    args.device = env_cfg.sim.device

    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make(args.task, cfg=env_cfg)
        cleanup.callback(env.close)
        env.reset(seed=0)
        u = env.unwrapped
        n, dev = u.num_envs, u.device
        robot, cyl = u.scene["robot"], u.scene["object"]
        sensor = u.scene.sensors["cyl_contacts"]
        origins = u.scene.env_origins
        term = u.action_manager.get_term("joint_pos")
        zero = torch.zeros(n, term.action_dim, device=dev)

        # --- 1. hand O --------------------------------------------------------------------------------------------
        source = desc.source.get("parameters")
        params = yaml.safe_load((Path(desc_path).parents[5] / source).read_text()) if source else None
        spec = get_hand(env_cfg.hand_name)
        print("\n=== 1. hand ===")
        print(f"description: {desc_path}\n  source: {desc.source}\n  usd: {spec.robot_cfg.spawn.usd_path}")
        fin, pads = params["finger"], params["pads"]
        pos = torch_of(robot.data.body_link_pos_w)[0]
        quat = torch_of(robot.data.body_link_quat_w)[0]
        bid = {b: robot.find_bodies(b)[0][0] for b in bodies}
        import isaaclab.utils.math as math_utils

        print(f"{'finger':8s} {'quantity':28s} {'sim [mm]':>9s} {'yaml [mm]':>9s}")
        for f in ("index", "middle", "ring"):
            p = {s: pos[bid[f"{f}_{s}"]] for s in ("proximal", "servo_link", "driver_link", "distal")}
            bp = desc.body(f"{f}_distal")["origin"]
            loop = next(lp for lp in desc.loops if lp["name"] == f"{f}_bp")["anchor"]
            local = torch.tensor([loop[k] - bp[k] for k in range(3)], device=dev)
            b_prime = p["distal"] + math_utils.quat_apply(quat[bid[f"{f}_distal"]][None], local[None])[0]
            rows = [
                ("MCP axis -> A' (|ap|)", p["proximal"], p["servo_link"], math.hypot(fin["ap_x_mm"], fin["ap_z_mm"])),
                ("MCP axis -> A (a_x)", p["proximal"], p["driver_link"], math.hypot(fin["a_x_mm"], fin["a_z_mm"])),
                ("A' -> B (apb)", p["servo_link"], p["distal"], fin["apb_mm"]),
                ("A -> B' (abp)", p["driver_link"], b_prime, fin["abp_mm"]),
            ]
            for label, a, b, ref in rows:
                print(f"{f:8s} {label:28s} {1e3 * float((a - b).norm()):9.2f} {ref:9.2f}")
        print(f"yaml pads: thickness {pads['thickness_mm']} mm, mount offset {pads['mount_offset_mm']} mm, "
              f"proximal {pads['proximal_start_mm']}..{pads['proximal_end_mm']} mm, "
              f"distal {pads['distal_start_mm']:.1f}..{pads['distal_end_mm']} mm")
        servo_ids = [robot.find_joints(f"{f}_pip_servo")[0][0] for f in ("index", "middle", "ring")]

        def flexion_deg() -> list[float]:
            theta = torch_of(robot.data.joint_pos)[0, servo_ids]
            return [round(math.degrees(float(term._tables.flexion(f"{f}_pip_servo", theta[i:i + 1])[0])), 1)
                    for i, f in enumerate(("index", "middle", "ring"))]

        # --- 2. zero action ---------------------------------------------------------------------------------------
        print("\n=== 2. zero action (hold the cradle) ===")
        pad_cols = [bodies.index(b) for b in PAD_BODIES]
        other_cols = [k for k in range(len(bodies)) if k not in pad_cols]
        steps = int(args.hold_s / u.step_dt)
        drops = torch.zeros(n, dtype=torch.bool, device=dev)
        for k in range(steps):
            _, _, terminated, _, _ = env.step(zero)
            drops |= terminated.bool()
            if (k + 1) % int(1.0 / u.step_dt) == 0 or k == 0:
                d = rotate_mdp.object_pos_from_nominal(u)
                spin = rotate_mdp.axis_spin(u)[:, 0]
                v = torch_of(cyl.data.root_lin_vel_w).norm(dim=1)
                tilt = torch.rad2deg(torch.acos(rotate_mdp.cylinder_axis_w(u)[:, 1].clamp(-1, 1)))
                normal = torch_of(sensor.data.force_matrix_w)[:, 0].norm(dim=-1)  # (n, bodies)
                touching = (normal[:, pad_cols] > 1e-3).sum(dim=1).float()
                print(f"t {(k + 1) * u.step_dt:4.1f} s | |d| mean {1e3 * float(d.norm(dim=1).mean()):6.2f} max "
                      f"{1e3 * float(d.norm(dim=1).max()):6.2f} mm | d_x {1e3 * float(d[:, 0].mean()):6.2f} d_z "
                      f"{1e3 * float(d[:, 2].mean()):6.2f} mm | speed max {1e3 * float(v.max()):6.2f} mm/s | spin max "
                      f"{float(spin.abs().max()):.3f} rad/s | tilt max {float(tilt.max()):.2f} deg | pads touching "
                      f"{float(touching.mean()):.1f} | pad N {float(normal[:, pad_cols].sum(1).mean()):.3f} other N "
                      f"{float(normal[:, other_cols].sum(1).mean()):.3f} | drops {int(drops.sum())}")
        normal = torch_of(sensor.data.force_matrix_w)[0, 0].norm(dim=-1)
        print("env 0 pad normal forces [N]: " + ", ".join(f"{b} {float(normal[bodies.index(b)]):.3f}" for b in PAD_BODIES))
        print(f"env 0 PIP flexion [deg] (index, middle, ring): {flexion_deg()}  (cradle 50)")
        # pad face depth: the contact point on a proximal pad in its body frame (body axes = hand axes at zero pose)
        point = torch_of(sensor.data.contact_pos_w)[0, 0]  # (bodies, 3)
        for f in ("index", "middle", "ring"):
            c = bodies.index(f"{f}_proximal")
            if float(normal[c]) > 1e-3 and bool(torch.isfinite(point[c]).all()):
                b = bid[f"{f}_proximal"]
                local = math_utils.quat_apply_inverse(quat[b][None], (point[c] - pos[b])[None])[0]
                print(f"{f} proximal contact point in the proximal frame: x {1e3 * float(local[0]):.2f} mm along, "
                      f"face depth {-1e3 * float(local[2]):.2f} mm below the MCP axis (yaml: mount offset + thickness "
                      f"= {pads['mount_offset_mm'] + pads['thickness_mm']:.1f} mm)")
        if getattr(env_cfg, "rest_face", None) is not None:
            face = rotate_mdp.object_pos_face(u)
            rest = torch.tensor(env_cfg.rest_face, device=dev)
            print(f"face frame: cylinder centre (env 0) {[round(1e3 * float(x), 2) for x in face[0]]} mm, rest place "
                  f"{[round(1e3 * float(x), 2) for x in rest]} mm (two-pad rule), distance from rest mean "
                  f"{1e3 * float(rotate_mdp.object_rest_distance(u).mean()):.2f} max "
                  f"{1e3 * float(rotate_mdp.object_rest_distance(u).max()):.2f} mm")
            pts = rotate_mdp.distal_pad_points_face(u, u.observation_manager._group_obs_term_cfgs["policy"][
                u.observation_manager._group_obs_term_names["policy"].index("distal_pad_points")].params["asset_cfg"])
            print(f"distal pad-face points (env 0, face frame) [mm]: {[round(1e3 * float(x), 1) for x in pts[0]]}")
        print(f"cylinder centre (env 0, env frame): {[round(1e3 * float(x), 2) for x in torch_of(cyl.data.root_pos_w)[0] - origins[0]]} mm,"
              f" nominal {[round(1e3 * float(x), 2) for x in rotate_mdp.nominal_pos_w(u)[0] - origins[0]]} mm")

        # --- 3. reward sign ---------------------------------------------------------------------------------------
        print("\n=== 3. reward sign ===")
        omega = torch.zeros(n, 6, device=dev)
        omega[:, 4] = torch.where(torch.arange(n, device=dev) % 2 == 0, -args.spin, args.spin)
        cyl.write_root_velocity_to_sim_index(root_velocity=omega, env_ids=torch.arange(n, device=dev))
        r = rotate_mdp.rotation_reward(u, ROTATION_DIRECTION, *SPIN_CLIP)
        w = torch_of(cyl.data.root_ang_vel_w)
        top = torch.cross(w, torch.tensor([0.0, 0.0, CYLINDER_RADIUS_M], device=dev).expand(n, 3), dim=1)
        for e in range(min(n, 2)):
            print(f"env {e}: omega_y {float(w[e, 1]):+.2f} rad/s, spin about axis {float(rotate_mdp.axis_spin(u)[e, 0]):+.2f}"
                  f", top point v_x {1e3 * float(top[e, 0]):+.1f} mm/s ({'toward' if top[e, 0] < 0 else 'away from'} "
                  f"the palm) -> rotation reward {float(r[e]):+.2f}")
        _, rew, _, _, _ = env.step(zero)
        print(f"after one step: rotation reward (env 0, 1) {[round(float(x), 3) for x in rotate_mdp.rotation_reward(u, ROTATION_DIRECTION, *SPIN_CLIP)[:2]]}"
              f", total step reward {[round(float(x), 4) for x in rew[:2]]}")

        # --- 4. scripted curl -------------------------------------------------------------------------------------
        print("\n=== 4. scripted curl of every PIP (from a fresh reset) ===")
        env.reset(seed=1)
        for _ in range(int(1.0 / u.step_dt)):
            env.step(zero)
        pip_cols = [k for k, nm in enumerate(term._joint_names) if nm.endswith("_pip_servo")]
        angle = torch.zeros(n, device=dev)
        rot_sum = torch.zeros(n, device=dev)
        drops = torch.zeros(n, dtype=torch.bool, device=dev)
        for phase, (a0, a1) in (("close", (0.0, 1.0)), ("open", (1.0, -1.0)), ("close again", (-1.0, 0.0))):
            seg = int(1.5 / u.step_dt)
            angle.zero_()
            rot_sum.zero_()
            for i in range(seg):
                act = zero.clone()
                act[:, pip_cols] = a0 + (a1 - a0) * (i + 1) / seg
                _, _, terminated, _, _ = env.step(act)
                drops |= terminated.bool()
                angle += rotate_mdp.axis_spin(u)[:, 0] * u.step_dt
                rot_sum += rotate_mdp.rotation_reward(u, ROTATION_DIRECTION, *SPIN_CLIP) * u.step_dt
            d = rotate_mdp.object_pos_from_nominal(u)
            print(f"{phase:12s} action {a0:+.0f} -> {a1:+.0f}: PIP flexion {flexion_deg()} deg | spin integral mean "
                  f"{math.degrees(float(angle.mean())):+7.1f} deg (min {math.degrees(float(angle.min())):+.1f}, max "
                  f"{math.degrees(float(angle.max())):+.1f}) | rotation reward integral {float(rot_sum.mean()):+.3f} | "
                  f"|d| {1e3 * float(d.norm(dim=1).mean()):.1f} mm | drops so far {int(drops.sum())}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
