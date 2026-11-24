# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Are the ``CoDe-Rotate-Cube`` "drops" falls, or cubes that slid while still held?

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run --extra rsl-rl python scripts/rotate_cube/drop_diagnosis.py \
        --checkpoint <model.pt> [--num_envs 512] [--steps 300] physics=newton_mjwarp

Plays the checkpoint (deterministic) and, at every termination, records the cube's displacement from its grasp position
split into horizontal (world xy) and vertical (z, signed), its speed, how many pad-carrying bodies still touch it
(> 0.05 N), and which test fired (off-grasp, tilt). Prints the distributions and writes ``<run>/eval/drop_diagnosis.json``.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import sys
from pathlib import Path

import gymnasium as gym
import torch

FINGERS = ("index", "middle", "ring")
PAD_BODIES = [f"{f}_{s}" for f in FINGERS for s in ("proximal", "distal")] + ["thumb_distal"]


def main(argv=None) -> int:
    from isaaclab.app import add_launcher_args

    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--task", default="CoDe-Rotate-Cube-O")
    parser.add_argument("--num_envs", type=int, default=512)
    parser.add_argument("--steps", type=int, default=300)
    parser.add_argument("--max_distance", type=float, default=None, help="off-grasp termination [m] (the run's)")
    parser.add_argument("--tilt_min_dot", type=float, default=None, help="tilt termination threshold (the run's)")
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args

    from importlib.metadata import version as _version

    from isaaclab.app import launch_simulation
    from isaaclab.sensors import ContactSensorCfg

    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg

    from isaaclab_tasks.utils import resolve_task_config

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.assets.description import Description
    from code_isaaclab.tasks.rotate_cube.grasp import HANDS
    from code_isaaclab.tasks.rotate_cube.rotate_cube_mdp import grasp_term

    env_cfg, agent_cfg = resolve_task_config(args.task, "rsl_rl_cfg_entry_point")
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.seed = agent_cfg.seed = 1000
    if args.max_distance is not None:
        env_cfg.terminations.object_off_grasp.params["max_distance"] = args.max_distance
    if args.tilt_min_dot is not None:
        env_cfg.terminations.object_tilted.params["min_dot"] = args.tilt_min_dot
    bodies = [b["name"] for b in Description.load(HANDS[env_cfg.hand_key]).bodies]
    env_cfg.scene.obj_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/object", filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{b}" for b in bodies])
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, _version("rsl-rl-lib"))
    args.device = env_cfg.sim.device
    rows = []
    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make(args.task, cfg=env_cfg)
        cleanup.callback(env.close)
        env.reset(seed=1000)
        wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        runner = create_rsl_rl_runner(wrapped, agent_cfg)
        runner.load(str(args.checkpoint))
        policy = runner.get_inference_policy(device=wrapped.unwrapped.device)
        u = env.unwrapped
        obj, sensor, g = u.scene["object"], u.scene.sensors["obj_contacts"], grasp_term(u)
        pad_cols = [bodies.index(b) for b in PAD_BODIES]
        tq = lambda v: v if isinstance(v, torch.Tensor) else v.torch  # noqa: E731
        tm = u.termination_manager
        obs = wrapped.get_observations()
        prev = None
        for step in range(args.steps):
            # the state before the step: a terminating env is reset inside step(), so keep the last live state
            pos = tq(obj.data.root_pos_w) - u.scene.env_origins
            disp = pos - g.grasp_pos
            vel = tq(obj.data.root_lin_vel_w).norm(dim=1)
            pads = (tq(sensor.data.force_matrix_w)[:, 0].norm(dim=-1)[:, pad_cols] > 0.05).sum(dim=1)
            tilt_term = tm.get_term_cfg("object_tilted").func
            import isaaclab.utils.math as math_utils

            up_z = math_utils.quat_apply(tq(obj.data.root_quat_w), tilt_term.up_body)[:, 2]
            prev = (disp.clone(), vel.clone(), pads.clone(), up_z.clone())
            with torch.inference_mode():
                obs, _, dones, _ = wrapped.step(policy(obs))
            off, tilt = tm.get_term("object_off_grasp").bool(), tm.get_term("object_tilted").bool()
            for e in (off | tilt).nonzero().flatten().tolist():
                d, v, p, uz = prev[0][e], prev[1][e], prev[2][e], prev[3][e]
                rows.append({"step": step, "off_grasp": bool(off[e]), "tilt": bool(tilt[e]),
                             "horizontal_mm": 1e3 * float(d[:2].norm()), "vertical_mm": 1e3 * float(d[2]),
                             "speed_mm_s": 1e3 * float(v), "pads_touching": int(p),
                             "up_z_last_live": float(uz)})

    def q(key, cond=lambda r: True):
        x = torch.tensor([r[key] for r in rows if cond(r)], dtype=torch.float)
        return {"n": int(x.numel()), "p10": round(float(x.quantile(0.1)), 2), "median": round(float(x.median()), 2),
                "p90": round(float(x.quantile(0.9)), 2)} if x.numel() else None

    summary = {
        "checkpoint": str(args.checkpoint), "terminations": len(rows),
        "off_grasp": sum(r["off_grasp"] for r in rows), "tilt": sum(r["tilt"] for r in rows),
        "last_live_step_before_termination": {
            "horizontal_mm": q("horizontal_mm"), "vertical_mm": q("vertical_mm"), "speed_mm_s": q("speed_mm_s"),
            "pads_touching": q("pads_touching"), "up_z_last_live": q("up_z_last_live")},
        "tilt_only": {"pads_touching": q("pads_touching", lambda r: r["tilt"] and not r["off_grasp"]),
                      "vertical_mm": q("vertical_mm", lambda r: r["tilt"] and not r["off_grasp"]),
                      "speed_mm_s": q("speed_mm_s", lambda r: r["tilt"] and not r["off_grasp"]),
                      "fraction_2plus_pads": sum(r["pads_touching"] >= 2 for r in rows if r["tilt"] and not r["off_grasp"])
                      / max(sum(r["tilt"] and not r["off_grasp"] for r in rows), 1)},
        "fraction_with_2plus_pads_touching": sum(r["pads_touching"] >= 2 for r in rows) / max(len(rows), 1),
        "fraction_vertical_below_minus_10mm": sum(r["vertical_mm"] < -10 for r in rows) / max(len(rows), 1),
        "fraction_horizontal_dominant": sum(r["horizontal_mm"] > abs(r["vertical_mm"]) for r in rows) / max(len(rows), 1),
        "rows": rows[:2000],
    }
    out = args.checkpoint.parent / "eval" / f"drop_diagnosis_{args.checkpoint.stem}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "rows"}, indent=1))
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
