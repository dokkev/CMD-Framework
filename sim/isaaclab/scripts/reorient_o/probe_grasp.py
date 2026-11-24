# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Scripted probes of ``CoDe-Reorient-Cube-<hand>`` before training: which axes and how far can this hand, with its
thumb pad, turn the object from the grasp? (as ``report/study3/study3_pilot.md`` did for the pilot's cube)

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run python scripts/reorient_o/probe_grasp.py [--task CoDe-Reorient-Cube-O] \
        [--repeats 2] physics=newton_mjwarp

One environment per (probe, repeat). Each probe moves some actuated joints from the grasp by a fixed offset (PIP in
flexion), ramped over ``--ramp_s``, held ``--hold_s``, ramped back and held again; actions are applied without EMA
smoothing. The drop termination is off. First every environment holds the grasp for 1 s (settling). Per probe: the
largest object rotation about each world axis relative to its orientation after settling (rotation vector of
``q q_settled^-1``; signed component of the largest magnitude), the rotation left at the end, the largest
displacement, the largest force on the object from non-pad bodies, the pads in contact on average, and whether it fell
(more than 30 mm from where it settled). Writes ``logs/reorient_o/probes/<run>.json`` and prints a table.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import sys
import time
from pathlib import Path

import gymnasium as gym
import torch

PROBES: dict[str, dict[str, float]] = {
    "hold": {},
    "pip+10": {"pip": 10}, "pip+20": {"pip": 20}, "pip+30": {"pip": 30}, "pip+60": {"pip": 60}, "pip-20": {"pip": -20},
    "mcp+15": {"fe": 15}, "mcp-15": {"fe": -15},
    "aa+15": {"aa": 15}, "aa-15": {"aa": -15},
    "index_aa+15,ring_aa-15": {"index_aa": 15, "ring_aa": -15},
    "index_pip+60": {"index_pip": 60}, "ring_pip+60": {"ring_pip": 60},
    "index_pip+60,ring_pip-20": {"index_pip": 60, "ring_pip": -20},
    "thumb_ip+15": {"thumb_ip": 15}, "thumb_ip-15": {"thumb_ip": -15},
    "thumb_mcp+15": {"thumb_mcp": 15}, "thumb_mcp-15": {"thumb_mcp": -15},
    "thumb_raa+15": {"thumb_raa": 15}, "thumb_raa-15": {"thumb_raa": -15},
    "thumb_ro+15": {"thumb_ro": 15}, "thumb_ro-15": {"thumb_ro": -15},
    "pip+20,thumb_ip-10": {"pip": 20, "thumb_ip": -10}, "pip+20,thumb_mcp-15": {"pip": 20, "thumb_mcp": -15},
    "pip+20,thumb_mcp+15": {"pip": 20, "thumb_mcp": 15}, "pip+30,mcp-15": {"pip": 30, "fe": -15},
    "pip+60,thumb_ip-10": {"pip": 60, "thumb_ip": -10},
    "pip+60,thumb_mcp-15": {"pip": 60, "thumb_mcp": -15},
    "mcp+15,pip+40": {"fe": 15, "pip": 40},
    "mcp+15,thumb_mcp-15": {"fe": 15, "thumb_mcp": -15},
}
"""Probe name -> joint offsets [deg]: ``pip``/``fe``/``aa`` move that joint of all three fingers, ``<finger>_<j>`` one
finger's, ``thumb_<j>`` a thumb joint."""
FINGERS = ("index", "middle", "ring")
PADS = [f"{f}_{s}" for f in FINGERS for s in ("proximal", "distal")] + ["thumb_distal"]


def torch_of(value):
    return value if isinstance(value, torch.Tensor) else value.torch


def main(argv=None) -> int:
    from isaaclab.app import add_launcher_args

    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--task", default="CoDe-Reorient-Cube-O")
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--ramp_s", type=float, default=1.5)
    parser.add_argument("--hold_s", type=float, default=1.0)
    parser.add_argument("--only", nargs="*", default=None, help="probe names to run (default: all)")
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args

    from isaaclab.app import launch_simulation
    from isaaclab.sensors import ContactSensorCfg

    import isaaclab.utils.math as math_utils
    from isaaclab_tasks.utils import resolve_task_config

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.assets.description import Description
    from code_isaaclab.tasks.rotate.env_cfg import HANDS

    probes = {k: v for k, v in PROBES.items() if args.only is None or k in args.only}
    envs = [(name, r) for name in probes for r in range(args.repeats)]
    env_cfg, _ = resolve_task_config(args.task, "rsl_rl_cfg_entry_point")
    env_cfg.scene.num_envs = len(envs)
    env_cfg.terminations.object_out_of_reach = None
    env_cfg.episode_length_s = 1.0e4
    env_cfg.actions.joint_pos.alpha = 1.0
    bodies = [b["name"] for b in Description.load(HANDS[env_cfg.hand_key]).bodies]
    env_cfg.scene.obj_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/object", filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{b}" for b in bodies])
    args.device = env_cfg.sim.device

    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make(args.task, cfg=env_cfg)
        cleanup.callback(env.close)
        env.reset(seed=0)
        u = env.unwrapped
        n, dev, dt = u.num_envs, u.device, u.step_dt
        obj, robot = u.scene["object"], u.scene["robot"]
        sensor = u.scene.sensors["obj_contacts"]
        term = u.action_manager.get_term("joint_pos")
        names = list(term._joint_names)
        lo, hi = term._cmd_limits[0, :, 0], term._cmd_limits[0, :, 1]
        grasp = term._to_flexion(torch_of(robot.data.default_joint_pos)[:, term._joint_ids])[0]
        pad_cols = [bodies.index(b) for b in PADS]
        other_cols = [k for k in range(len(bodies)) if k not in pad_cols]

        offsets = torch.zeros(n, len(names), device=dev)
        for e, (name, _) in enumerate(envs):
            for key, deg in probes[name].items():
                for k, jn in enumerate(names):
                    finger, _, joint = jn.partition("_")
                    joint = joint.replace("_servo", "")
                    if (key == joint and finger in FINGERS) or key == jn.replace("_servo", ""):
                        offsets[e, k] = math.radians(deg)

        def act(scale: float) -> torch.Tensor:
            cmd = torch.clamp(grasp + scale * offsets, lo, hi)
            return 2.0 * (cmd - lo) / (hi - lo) - 1.0

        for _ in range(int(1.0 / dt)):  # settle in the grasp
            env.step(act(0.0))
        settled = torch_of(sensor.data.force_matrix_w)[:, 0].norm(dim=-1).mean(dim=0)  # (bodies,), mean over envs
        print("contact force on the object after settling, mean over environments [N]: " + ", ".join(
            f"{b} {float(f):.2f}" for b, f in zip(bodies, settled) if float(f) > 0.01))
        q0 = torch_of(obj.data.root_quat_w).clone()
        p0 = torch_of(obj.data.root_pos_w).clone()
        rot_max = torch.zeros(n, 3, device=dev)
        disp_max = torch.zeros(n, device=dev)
        other_max = torch.zeros(n, device=dev)
        pads_sum = torch.zeros(n, device=dev)
        steps = 0
        ramp, hold = int(args.ramp_s / dt), int(args.hold_s / dt)
        schedule = ([(i + 1) / ramp for i in range(ramp)] + [1.0] * hold + [1.0 - (i + 1) / ramp for i in range(ramp)]
                    + [0.0] * hold)
        started = time.time()
        rot_at_peak = torch.zeros(n, 3, device=dev)
        for k, s in enumerate(schedule):
            env.step(act(s))
            q = torch_of(obj.data.root_quat_w)
            rv = math_utils.axis_angle_from_quat(math_utils.quat_mul(q, math_utils.quat_conjugate(q0)))
            rot_max = torch.where(rv.abs() > rot_max.abs(), rv, rot_max)
            if k == ramp + hold - 1:
                rot_at_peak = rv.clone()
            disp_max = torch.maximum(disp_max, (torch_of(obj.data.root_pos_w) - p0).norm(dim=1))
            force = torch_of(sensor.data.force_matrix_w)[:, 0].norm(dim=-1)  # (n, bodies)
            other_max = torch.maximum(other_max, force[:, other_cols].sum(dim=1))
            pads_sum += (force[:, pad_cols] > 1e-3).float().sum(dim=1)
            steps += 1
        q = torch_of(obj.data.root_quat_w)
        rot_end = math_utils.axis_angle_from_quat(math_utils.quat_mul(q, math_utils.quat_conjugate(q0)))
        fell = disp_max > 0.03

    rows = []
    for e, (name, r) in enumerate(envs):
        rows.append({"probe": name, "repeat": r, "offsets_deg": probes[name],
                     "rot_max_deg_xyz": [round(math.degrees(float(v)), 1) for v in rot_max[e]],
                     "rot_at_peak_deg_xyz": [round(math.degrees(float(v)), 1) for v in rot_at_peak[e]],
                     "rot_end_deg_xyz": [round(math.degrees(float(v)), 1) for v in rot_end[e]],
                     "disp_max_mm": round(1e3 * float(disp_max[e]), 1), "nonpad_force_max_n": round(float(other_max[e]), 2),
                     "pads_in_contact_mean": round(float(pads_sum[e]) / steps, 2), "fell": bool(fell[e])})
    out_dir = Path(__file__).resolve().parents[2] / "logs" / "reorient_o" / "probes"
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"probe_{args.task}_{time.strftime('%Y%m%d-%H%M%S')}.json"
    out.write_text(json.dumps({"task": args.task, "grasp": env_cfg.grasp, "ramp_s": args.ramp_s, "hold_s": args.hold_s,
                               "wall_s": time.time() - started, "rows": rows}, indent=1) + "\n")
    print(f"{'probe':28s} {'rot max x,y,z [deg]':>22s} {'at peak x,y,z':>20s} {'end x,y,z':>20s} {'disp mm':>8s} "
          f"{'nonpad N':>8s} {'pads':>5s} fell")
    for row in rows:
        print(f"{row['probe']:28s} {str(row['rot_max_deg_xyz']):>22s} {str(row['rot_at_peak_deg_xyz']):>20s} "
              f"{str(row['rot_end_deg_xyz']):>20s} {row['disp_max_mm']:8.1f} {row['nonpad_force_max_n']:8.2f} "
              f"{row['pads_in_contact_mean']:5.2f} {row['fell']}")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
