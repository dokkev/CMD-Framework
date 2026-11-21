# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3 gate: does flexion-space control work in the simulator? (MuJoCo-Warp by default)

    uv run python scripts/study3/flexion_gate.py [--mode levels|step] [--stiffness K] [--damping D] physics=newton_mjwarp

The task's PIP servo actions are replaced by :class:`~code_isaaclab.flexion_control.FlexionEMAAction` (alpha 1, no
smoothing); the cube is moved out of the hand and the drop termination switched off, so only the fingers move.

- ``levels``: PIP flexion targets 10, 30, 60, 90, 110 deg, each held; per level the measured flexion of the index
  finger, from the servo joint through theta_H^-1 and, independently of the tables, from the proximal and distal body
  orientations (:func:`code_isaaclab.loops.measure_finger_pip_deg`), against the command.
- ``step``: 30 -> 60 deg; the time to 63% of the step, for comparing gain schedules (a scheduled stiffness K and K/4
  should differ if the runtime gain write reaches the solver).

Writes ``logs/study3_pilot/flexion_gate/<run>.json``.
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

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402


def main(argv=None) -> int:
    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--protocol", default="main", choices=tuple(common.PROTOCOLS))
    parser.add_argument("--variant", default="flat", choices=tuple(common.TASKS))
    parser.add_argument("--mode", default="levels", choices=("levels", "step"))
    parser.add_argument("--stiffness", type=float, default=None, help="K_q* [N m/rad] (None: servo gains)")
    parser.add_argument("--damping", type=float, default=None, help="D_q* [N m s/rad] (None: servo damping)")
    parser.add_argument("--hold_steps", type=int, default=90)
    parser.add_argument("--tag", default="")
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args

    from isaaclab.app import launch_simulation

    from isaaclab_tasks.utils import resolve_task_config

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.assets.description import Description
    from code_isaaclab.flexion_control import FlexionEMAActionCfg
    from code_isaaclab.hands import get_hand
    from code_isaaclab.hands.code_variants import STUDY3_DESCRIPTION
    from code_isaaclab.loops import measure_finger_pip_deg

    task = common.PROTOCOLS[args.protocol][args.variant]
    physics = next((t.split("=", 1)[1] for t in hydra_args if t.startswith("physics=")), "newton_mjwarp")
    env_cfg, _ = resolve_task_config(task, "")
    env_cfg.scene.num_envs = 1
    spec = get_hand(env_cfg.hand)
    old = env_cfg.actions.joint_pos
    env_cfg.actions.joint_pos = FlexionEMAActionCfg(
        asset_name="robot", joint_names=old.joint_names, alpha=1.0, four_bar_tables=spec.four_bar_tables,
        flexion_limits_deg=(0.0, 120.0), flexion_stiffness=args.stiffness, flexion_damping=args.damping)
    env_cfg.scene.object.init_state.pos = (0.3, 0.3, 0.1)  # out of the hand
    env_cfg.terminations.object_out_of_reach = None
    env_cfg.episode_length_s = 1.0e4
    args.device = env_cfg.sim.device
    desc = Description.load(STUDY3_DESCRIPTION)

    if args.mode == "levels":
        schedule = [q for q in (10.0, 30.0, 60.0, 90.0, 110.0) for _ in range(args.hold_steps)]
    else:
        schedule = [30.0] * args.hold_steps + [60.0] * args.hold_steps
    rows = []
    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make(task, cfg=env_cfg)
        cleanup.callback(env.close)
        env.reset(seed=0)
        unwrapped = env.unwrapped
        robot = unwrapped.scene["robot"]
        term = unwrapped.action_manager.get_term("joint_pos")
        names = list(term._joint_names)
        lo, hi = term._cmd_limits[0, :, 0], term._cmd_limits[0, :, 1]
        servo_id = robot.find_joints("index_pip_servo")[0][0]
        dt = unwrapped.step_dt
        for step, q_deg in enumerate(schedule):
            command = torch.zeros(1, len(names), device=unwrapped.device)
            for k, n in enumerate(names):
                if n.endswith("_pip_servo"):
                    command[0, k] = math.radians(q_deg)
            action = 2.0 * (command - lo) / (hi - lo) - 1.0
            env.step(action)
            theta = robot.data.joint_pos.torch[0, servo_id]
            q_tables = float(term._tables.flexion("index_pip_servo", theta.reshape(1))[0])
            quat = robot.data.body_quat_w.torch[0].detach().cpu().numpy()
            q_bodies = measure_finger_pip_deg(desc, "index", list(robot.body_names), quat)
            stiff = robot.data.joint_stiffness.torch[0, servo_id].item() if hasattr(robot.data, "joint_stiffness") else None
            rows.append({"t": step * dt, "command_deg": q_deg, "q_tables_deg": math.degrees(q_tables),
                         "q_bodies_deg": float(q_bodies), "servo_deg": math.degrees(float(theta)), "servo_stiffness": stiff})

    summary = {"task": task, "physics": physics, "mode": args.mode, "stiffness": args.stiffness, "damping": args.damping}
    if args.mode == "levels":
        levels = []
        for q_deg in sorted({r["command_deg"] for r in rows}):
            tail = [r for r in rows if r["command_deg"] == q_deg][-20:]
            levels.append({"command_deg": q_deg, "q_tables_deg": float(np.mean([r["q_tables_deg"] for r in tail])),
                           "q_bodies_deg": float(np.mean([r["q_bodies_deg"] for r in tail])),
                           "tables_minus_bodies_deg": float(np.mean([r["q_tables_deg"] - r["q_bodies_deg"] for r in tail])),
                           "servo_stiffness": tail[-1]["servo_stiffness"]})
        summary["levels"] = levels
    else:
        after = [r for r in rows if r["command_deg"] == 60.0]
        start = [r for r in rows if r["command_deg"] == 30.0][-1]["q_bodies_deg"]
        final = float(np.mean([r["q_bodies_deg"] for r in after[-20:]]))
        target = start + 0.632 * (final - start)
        t0 = after[0]["t"]
        rise = next((r["t"] - t0 for r in after if (r["q_bodies_deg"] - target) * np.sign(final - start) >= 0), None)
        summary.update({"start_deg": start, "final_deg": final, "rise_63_s": rise})
    run = common.run_id(f"flexion_gate_{args.mode}{('_' + args.tag) if args.tag else ''}")
    path = common.write_json(common.LOG_ROOT / "flexion_gate" / f"{run}.json", {**summary, "rows": rows,
                                                                                    "manifest": common.manifest(command=sys.argv)})
    print(json.dumps({k: v for k, v in summary.items() if k != "rows"}, indent=1))
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
