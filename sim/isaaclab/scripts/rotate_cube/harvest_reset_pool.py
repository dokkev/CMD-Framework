# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Harvest a reset pool of mid-rotation states from a policy roll-out (OmniReset's diverse resets, Yin et al. 2026).

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run --extra rsl-rl python scripts/rotate_cube/harvest_reset_pool.py \\
        --checkpoint <model.pt> --out_dir logs/rotate_cube/<name> [--num_envs 1024] [--seed 61000] \\
        [--samples_per_env 4] [--t_min 2] [--t_max 20] physics=newton_mjwarp

Rolls the policy out deterministically on the task's grasp cache (training settings: observation noise and
terminations as configured) for each environment's first episode. Each environment draws ``--samples_per_env`` times
uniformly in [``--t_min``, ``--t_max``) s; at each, the state is kept if the cube is held: at least 2 pad-carrying
bodies touch it (contact force > 0.05 N, the grasp cache's gate), it lies within 40 mm of its grasp position and its
reset up axis keeps z >= 0.5 (60 deg tilt). Late samples include the states where rotation has stopped at the joint
limits. Kept per state: every joint's position, velocity and PD target, the relative action's targets (actuated, PIP in
flexion), the cube's env-frame position, quaternion and world velocity, and the episode step (the gait clock's phase).
Writes ``<out_dir>/<hand>.pt`` (refusing to overwrite) and ``<hand>.json``. Train with
``CODE_ROTATE_CUBE_RESET_POOL=<out_dir>``.
"""

from __future__ import annotations

import argparse
import json
import math
from contextlib import ExitStack
from importlib.metadata import version
from pathlib import Path

import gymnasium as gym
import torch

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import math as math_utils

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg

from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli

import code_isaaclab.tasks  # noqa: F401
from code_isaaclab.tasks.rotate_cube.rotate_cube_mdp import _t, actuated_targets_normalized, grasp_term

PAD_BODIES = [f"{f}_{s}" for f in ("index", "middle", "ring") for s in ("proximal", "distal")] + ["thumb_distal"]
MAX_DISPLACEMENT_M = 0.04
MIN_UP_DOT = 0.5


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--task", default="CoDe-Rotate-Cube-O")
    parser.add_argument("--out_dir", type=Path, required=True)
    parser.add_argument("--num_envs", type=int, default=1024)
    parser.add_argument("--seed", type=int, default=61000)
    parser.add_argument("--samples_per_env", type=int, default=4)
    parser.add_argument("--t_min", type=float, default=2.0)
    parser.add_argument("--t_max", type=float, default=20.0)
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, overrides = setup_preset_cli(parser, argv)
    if not 0.0 <= args.t_min < args.t_max:
        parser.error("need 0 <= --t_min < --t_max")
    checkpoint = args.checkpoint.resolve(strict=True)
    env_cfg, agent_cfg = resolve_task_config(args.task, "rsl_rl_cfg_entry_point", overrides=overrides)
    out = args.out_dir / f"{env_cfg.hand_key}.pt"
    if out.exists():
        parser.error(f"destination already exists: {out}")
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.seed = agent_cfg.seed = args.seed
    env_cfg.scene.object_pad_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/object", filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{b}" for b in PAD_BODIES]
    )
    cache = Path(env_cfg.events.reset_grasp.params["cache"]).resolve(strict=True)
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, version("rsl-rl-lib"))
    args.device = env_cfg.sim.device

    with launch_simulation(env_cfg, args), ExitStack() as cleanup, torch.inference_mode():
        env = gym.make(args.task, cfg=env_cfg)
        cleanup.callback(env.close)
        wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        runner = create_rsl_rl_runner(wrapped, agent_cfg)
        runner.load(str(checkpoint), load_cfg={"actor": True}, map_location=wrapped.device)
        policy = runner.get_inference_policy(device=wrapped.device)
        u = env.unwrapped
        robot, obj, contacts = u.scene["robot"], u.scene["object"], u.scene.sensors["object_pad_contacts"]
        tilted = u.termination_manager.get_term_cfg("object_tilted").func
        action = u.action_manager.get_term("joint_pos")
        n, dev, dt = u.num_envs, u.device, u.step_dt
        lo, hi = math.ceil(args.t_min / dt), math.floor(args.t_max / dt)
        hi = min(hi, int(u.max_episode_length) - 1)
        sample_steps = torch.randint(lo, hi, (n, args.samples_per_env), device=dev)
        env.reset(seed=args.seed)
        obs = wrapped.get_observations()
        active = torch.ones(n, dtype=torch.bool, device=dev)
        yaw = torch.zeros(n, device=dev)
        keys = ("q", "qd", "target", "phase_step", "target_flex", "pos", "quat", "vel", "t_s", "yaw_rad",
                "limit_fraction")
        rows = {k: [] for k in keys}
        drawn = 0
        for step in range(1, hi + 1):
            prev_quat = _t(obj.data.root_quat_w).clone()
            obs, _, _, _ = wrapped.step(policy(obs))
            active &= ~u.reset_buf
            quat = _t(obj.data.root_quat_w)
            dq = math_utils.axis_angle_from_quat(math_utils.quat_mul(quat, math_utils.quat_conjugate(prev_quat)))
            yaw += torch.where(active, dq[:, 2], 0.0)
            due = active & (sample_steps == step).any(dim=1)
            drawn += int(((sample_steps == step).sum(dim=1) * active).sum())
            if not due.any():
                continue
            pos = _t(obj.data.root_pos_w) - u.scene.env_origins
            touching = (_t(contacts.data.force_matrix_w)[:, 0].norm(dim=-1) > 0.05).sum(dim=1)
            held = (
                due & (touching >= 2) & ((pos - grasp_term(u).grasp_pos).norm(dim=1) <= MAX_DISPLACEMENT_M)
                & (tilted.up_dot() >= MIN_UP_DOT)
            )  # fmt: skip
            if not held.any():
                continue
            rows["q"].append(_t(robot.data.joint_pos)[held].cpu())
            rows["qd"].append(_t(robot.data.joint_vel)[held].cpu())
            rows["target"].append(_t(robot.data.joint_pos_target)[held].cpu())
            rows["phase_step"].append(u.episode_length_buf[held].float().cpu())
            rows["target_flex"].append(action.targets[held].cpu())
            rows["pos"].append(pos[held].cpu())
            rows["quat"].append(quat[held].cpu())
            rows["vel"].append(_t(obj.data.root_vel_w)[held].cpu())
            rows["t_s"].append(torch.full((int(held.sum()),), step * dt))
            rows["yaw_rad"].append(yaw[held].cpu())
            saturated = actuated_targets_normalized(u)[held].abs() >= 0.95
            rows["limit_fraction"].append(saturated.float().mean(dim=1).cpu())
        if not rows["q"]:
            raise RuntimeError("No held states to harvest")
        data = {k: torch.cat(v) for k, v in rows.items()}
        from code_isaaclab.tasks.rotate_cube.grasp import CUBE_SIZE_M, GRASP_MCP_DEG, GRASP_PIP_DEG

        data.update({
            "joint_names": list(robot.joint_names), "actuated": list(env_cfg.events.reset_grasp.params["actuated"]),
            "hand": env_cfg.hand_key, "cube_size_m": CUBE_SIZE_M, "grasp_mcp_deg": GRASP_MCP_DEG,
            "grasp_pip_deg": GRASP_PIP_DEG, "pad_crown_mm": env_cfg.pad_crown_mm,
            "pad_crown_layout": env_cfg.pad_crown_layout,
        })  # fmt: skip
        summary = {
            "checkpoint": str(checkpoint), "grasp_cache": str(cache), "seed": args.seed, "num_envs": n,
            "overrides": overrides, "samples_per_env": args.samples_per_env, "t_range_s": [args.t_min, args.t_max],
            "max_displacement_m": MAX_DISPLACEMENT_M, "min_up_dot": MIN_UP_DOT, "drawn_while_active": drawn,
            "kept": len(data["q"]),
            "t_s_quartiles": torch.quantile(data["t_s"], torch.tensor([0.25, 0.5, 0.75])).tolist(),
            "yaw_deg_p10_p50_p90": torch.rad2deg(
                torch.quantile(data["yaw_rad"], torch.tensor([0.1, 0.5, 0.9]))
            ).tolist(),
            "heading_deg_octant_counts": torch.histc(
                torch.rad2deg(math_utils.euler_xyz_from_quat(data["quat"])[2]) % 360.0, bins=8, min=0.0, max=360.0
            ).int().tolist(),
            "mean_saturated_target_fraction": float(data["limit_fraction"].mean()),
            "states_with_quarter_targets_saturated": float((data["limit_fraction"] >= 0.25).float().mean()),
        }  # fmt: skip
        args.out_dir.mkdir(parents=True, exist_ok=True)
        with out.open("xb") as stream:
            torch.save(data, stream)
        (args.out_dir / f"{env_cfg.hand_key}.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary, indent=2))
        print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
