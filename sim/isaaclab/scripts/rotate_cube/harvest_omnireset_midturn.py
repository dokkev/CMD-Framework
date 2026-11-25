# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Harvest mid-turn states from policy roll-outs, the source of v2's ``ObjectMidTurnHandWound`` resets.

    env -u DISPLAY PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CODE_OMNIRESET_ABNORMAL_FACTOR=10 uv run --extra rsl-rl python \\
        scripts/rotate_cube/harvest_omnireset_midturn.py --checkpoint <model.pt> --out <dir>/harvest_midturn.pt \\
        [--num_envs 1024] [--turn_count 6000] [--limit_count 4000] physics=newton_mjwarp

Khandate et al. (RSS 2023, section V-B): from a fixed initialisation a policy learns only in-grasp rotation; finger
gaiting needed reset states from inside the gait. Here the deterministic policy rolls out on the v1 task's training
resets, and a state is kept (at most once per ``--every`` steps per environment) when the cube is held (the thumb and
at least one finger link touch it, within 30 mm of its rest position, tilted < 20 deg) and either

- ``turn``: the cube is 30-80 deg into the episode's turn (integrated world-z rotation since the reset), or
- ``limit``: some servo is within 0.15 rad of a joint limit (PIP in flexion, 0-120 deg).

Kept: every joint's position, velocity and target, the cube's env-frame pose and velocity, the turn so far
(``progress``, rad) and the kind. ``gen_omnireset_resets.py --type ObjectMidTurnHandWound --source_rows <out>`` turns
them into settled reset states (jittered, half with one digit lifted, the goal 10-60 deg ahead).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from importlib.metadata import version
from pathlib import Path

FINGER_BODIES = [f"{f}_{s}" for f in ("index", "middle", "ring") for s in ("proximal", "distal")]


def main() -> int:
    from isaaclab.app import add_launcher_args, launch_simulation

    from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli

    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--task", default="CoDe-Rotate-Cube-O-OmniReset")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--num_envs", type=int, default=1024)
    p.add_argument("--turn_count", type=int, default=6000)
    p.add_argument("--limit_count", type=int, default=4000)
    p.add_argument("--every", type=int, default=15, help="control steps between two states of one environment")
    p.add_argument("--max_steps", type=int, default=6000)
    p.add_argument("--seed", type=int, default=11)
    add_launcher_args(p)
    p.set_defaults(device=None, visualizer=[])
    args, hydra = setup_preset_cli(p, None)
    sys.argv = [sys.argv[0]] + hydra
    if args.out.exists():
        p.error(f"destination already exists: {args.out}")

    import gymnasium as gym
    import torch

    from isaaclab.sensors import ContactSensorCfg

    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.tasks.rotate_cube_omnireset.mdp import goal_context, object_tilt, reset_term

    cfg, agent = resolve_task_config(args.task, "rsl_rl_cfg_entry_point")
    cfg.scene.num_envs = args.num_envs
    cfg.seed = agent.seed = args.seed
    cfg.scene.obj_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/object",
        filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{b}" for b in FINGER_BODIES + ["thumb_distal"]],
    )
    agent = handle_deprecated_rsl_rl_cfg(agent, version("rsl-rl-lib"))
    args.device = cfg.sim.device
    rest = torch.tensor(cfg.grasp["rest_env"])
    with launch_simulation(cfg, args), torch.inference_mode():
        env = gym.make(args.task, cfg=cfg)
        u = env.unwrapped
        wrapped = RslRlVecEnvWrapper(env, clip_actions=agent.clip_actions)
        runner = create_rsl_rl_runner(wrapped, agent)
        runner.load(args.checkpoint, load_cfg={"actor": True}, map_location=wrapped.device)
        policy = runner.get_inference_policy(device=wrapped.device)
        robot, obj, sensor = u.scene["robot"], u.scene["object"], u.scene.sensors["obj_contacts"]
        term = reset_term(u)
        lo, hi = term.flexion_limits()
        rest_d = rest.to(u.device)
        last = torch.full((u.num_envs,), -10**9, dtype=torch.long, device=u.device)
        kept = {k: [] for k in ("q", "qd", "target", "pos", "quat", "vel", "progress", "kind")}
        counts = {"turn": 0, "limit": 0}
        obs = wrapped.get_observations()
        step = 0
        for step in range(args.max_steps):
            obs, _, dones, _ = wrapped.step(policy(obs))
            ctx = goal_context(u)
            pos = robot.data.joint_pos.torch
            flex = term._to_flexion(pos[:, term._act])
            near_limit = ((flex - lo).abs() < 0.15) | ((hi - flex).abs() < 0.15)
            near_limit = near_limit.any(dim=1)
            cube_pos = obj.data.root_pos_w.torch - u.scene.env_origins
            quat = obj.data.root_quat_w.torch
            touch = sensor.data.force_matrix_w.torch[:, 0].norm(dim=-1) > 0.05
            held = touch[:, -1] & touch[:, :-1].any(dim=1) & ((cube_pos - rest_d).norm(dim=1) < 0.03) & (
                object_tilt(u, quat) < math.radians(20)) & ~dones.bool() & (
                u.episode_length_buf >= args.every)  # fmt: skip
            turn = (ctx.progress >= math.radians(30)) & (ctx.progress <= math.radians(80))
            spaced = step - last >= args.every
            want_turn = held & turn & spaced & (counts["turn"] < args.turn_count)
            want_limit = held & near_limit & ~turn & spaced & (counts["limit"] < args.limit_count)
            take = want_turn | want_limit
            if take.any():
                for key, val in (("q", pos), ("qd", robot.data.joint_vel.torch),
                                 ("target", robot.data.joint_pos_target.torch), ("pos", cube_pos), ("quat", quat),
                                 ("vel", torch.cat((obj.data.root_lin_vel_w.torch, obj.data.root_ang_vel_w.torch), 1)),
                                 ("progress", ctx.progress), ("kind", want_turn.long())):  # fmt: skip
                    kept[key].append(val[take].clone().cpu())
                counts["turn"] += int(want_turn.sum())
                counts["limit"] += int(want_limit.sum())
                last[take] = step
            if step % 300 == 0:
                print(f"step {step}: turn {counts['turn']}, limit {counts['limit']}", flush=True)
            if counts["turn"] >= args.turn_count and counts["limit"] >= args.limit_count:
                break
        data = {k: torch.cat(v) for k, v in kept.items()}
        data["goal_offset"] = torch.zeros(len(data["q"]))
        meta = {"checkpoint": args.checkpoint, "task": args.task, "num_envs": args.num_envs, "steps": step + 1,
                "seed": args.seed, "counts": counts, "every": args.every,
                "progress_deg_quantiles_10_50_90": torch.rad2deg(data["progress"]).quantile(
                    torch.tensor([0.1, 0.5, 0.9])).tolist()}  # fmt: skip
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("xb") as stream:
            torch.save({**data, "joint_names": list(robot.joint_names), "hand": cfg.hand_key, "meta": meta}, stream)
        args.out.with_suffix(".json").write_text(json.dumps(meta, indent=1) + "\n")
        print(json.dumps(meta, indent=1))
        env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
