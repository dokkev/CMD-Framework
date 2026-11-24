# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Physics of the reorient task's hand built with and without visual meshes, as the simulator sees them.

    for v in 0 1; do CODE_REORIENT_O_VISUALS=$v PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run python \
        scripts/reorient_o/physics_parity.py --task CoDe-Reorient-Cube-O-Reach physics=newton_mjwarp; done

Writes, per run, ``logs/reorient_o/parity/<task>_vis<0|1>.json``: the asset, the number of collision shapes per body (Newton
model, environment 0), body masses, centres of mass and inertias, joint names, and the cube's pose after 2 s holding the
grasp (the action that commands the default joint pose; the same 4 environments and seed). Compare the two files.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import sys
from pathlib import Path

import gymnasium as gym
import torch

OUT = Path(__file__).resolve().parents[2] / "logs" / "reorient_o" / "parity"


def main(argv=None) -> int:
    from isaaclab.app import add_launcher_args

    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--task", default="CoDe-Reorient-Cube-O-Reach")
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args

    from isaaclab.app import launch_simulation

    from isaaclab_tasks.utils import resolve_task_config

    import code_isaaclab.tasks  # noqa: F401

    env_cfg, _ = resolve_task_config(args.task, "rsl_rl_cfg_entry_point")
    env_cfg.scene.num_envs = 4
    env_cfg.seed = 0
    args.device = env_cfg.sim.device
    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make(args.task, cfg=env_cfg)
        cleanup.callback(env.close)
        env.reset(seed=0)
        u = env.unwrapped
        import newton

        model = u.sim.physics_manager.get_model()
        labels = list(model.shape_label)
        body_of = model.shape_body.numpy()
        flags = model.shape_flags.numpy()
        collide = int(newton.ShapeFlags.COLLIDE_SHAPES)
        body_labels = list(model.body_label)
        env0 = [k for k, b in enumerate(body_labels) if "env_0/" in b]
        shapes = {}
        for s, b in enumerate(body_of):
            if b in env0 and flags[s] & collide:
                shapes[body_labels[b].split("/")[-1]] = shapes.get(body_labels[b].split("/")[-1], 0) + 1
        mass = model.body_mass.numpy()
        com = model.body_com.numpy()
        inertia = model.body_inertia.numpy()
        bodies = {body_labels[b].split("/")[-1]: {"mass": float(mass[b]), "com": [round(float(x), 9) for x in com[b]],
                                                  "inertia_diag": [float(inertia[b][i][i]) for i in range(3)]}
                  for b in env0}
        term = u.action_manager.get_term("joint_pos")
        lo, hi = term._cmd_limits[0, :, 0], term._cmd_limits[0, :, 1]
        robot = u.scene["robot"]
        hold = term._to_flexion(robot.data.default_joint_pos.torch[:, term._joint_ids])
        act = 2.0 * (hold.clamp(lo, hi) - lo) / (hi - lo) - 1.0
        for _ in range(int(2.0 / u.step_dt)):
            env.step(act)
        obj = u.scene["object"]
        pose = torch.cat((obj.data.root_pos_w.torch - u.scene.env_origins, obj.data.root_quat_w.torch), dim=1)
        record = {"task": args.task, "visuals": env_cfg.visuals, "usd": env_cfg.scene.robot.spawn.usd_path,
                  "collision_shapes_per_body": dict(sorted(shapes.items())), "collision_shapes_total": sum(shapes.values()),
                  "bodies": dict(sorted(bodies.items())), "joints": list(robot.joint_names),
                  "cube_pose_after_2s": [[round(float(x), 7) for x in row] for row in pose]}
    OUT.mkdir(parents=True, exist_ok=True)
    out = OUT / f"{args.task}_vis{int(env_cfg.visuals)}.json"
    out.write_text(json.dumps(record, indent=1) + "\n")
    print(out, "collision shapes:", record["collision_shapes_total"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
