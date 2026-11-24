# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Revalidate cached grasps through the training reset path, then probe single-finger release and return."""

from __future__ import annotations

import argparse
import contextlib
import json
from pathlib import Path

import gymnasium as gym
import torch

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import math as math_utils

from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli

import code_isaaclab.tasks  # noqa: F401
from code_isaaclab.tasks.rotate_cube.rotate_cube_mdp import _t, grasp_term

FINGERS = ("index", "middle", "ring")
PAD_LINKS = tuple(f"{finger}_{link}" for finger in FINGERS for link in ("proximal", "distal")) + ("thumb_distal",)


def _contact_state(sensor) -> tuple[torch.Tensor, torch.Tensor]:
    forces = _t(sensor.data.force_matrix_w)[:, 0].norm(dim=-1)
    touch = forces > 0.05
    digit_contact = torch.stack([touch[:, 2 * k] | touch[:, 2 * k + 1] for k in range(len(FINGERS))], dim=1)
    thumb = touch[:, -1]
    return digit_contact, thumb


def _pose_state(env, initial_pos: torch.Tensor, initial_up: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    obj = env.scene["object"]
    pos = _t(obj.data.root_pos_w) - env.scene.env_origins
    quat = _t(obj.data.root_quat_w)
    world_up = torch.zeros(env.num_envs, 3, device=env.device)
    world_up[:, 2] = 1.0
    up_z = (math_utils.quat_apply(quat, initial_up) * world_up).sum(dim=1)
    return (pos - initial_pos).norm(dim=1), up_z


def _phase_summary(env, sensor, initial_pos: torch.Tensor, initial_up: torch.Tensor, env_ids=None) -> dict:
    displacement, up_z = _pose_state(env, initial_pos, initial_up)
    digit, thumb = _contact_state(sensor)
    if env_ids is not None:
        displacement, up_z, digit, thumb = (displacement[env_ids], up_z[env_ids], digit[env_ids], thumb[env_ids])
    return {
        "cube_displacement_mm_median": float((1e3 * displacement).median()),
        "cube_displacement_mm_p95": float(torch.quantile(1e3 * displacement, 0.95)),
        "cube_up_dot_min": float(up_z.min()),
        "thumb_contact_rate": float(thumb.float().mean()),
        "distinct_finger_contact_count_mean": float(digit.sum(dim=1).float().mean()),
        "all_three_digits_contact_rate": float((digit.sum(dim=1) == 3).float().mean()),
    }


def _headroom_summary(action) -> dict[str, float]:
    lower = action._cmd_limits[..., 0]
    upper = action._cmd_limits[..., 1]
    target = action.targets
    headroom = torch.minimum(target - lower, upper - target).clamp_min(0.0)
    return {name: float(torch.rad2deg(headroom[:, k]).quantile(0.05)) for k, name in enumerate(action._joint_names)}


def _run_phase(env, command: torch.Tensor, steps: int, initial_pos: torch.Tensor, initial_up: torch.Tensor):
    obj = env.scene["object"]
    max_displacement = torch.zeros(env.num_envs, device=env.device)
    min_up_dot = torch.ones(env.num_envs, device=env.device)
    yaw = torch.zeros(env.num_envs, device=env.device)
    for _ in range(steps):
        previous_quat = _t(obj.data.root_quat_w).clone()
        env.step(command)
        displacement, up_z = _pose_state(env, initial_pos, initial_up)
        max_displacement = torch.maximum(max_displacement, displacement)
        min_up_dot = torch.minimum(min_up_dot, up_z)
        dq = math_utils.axis_angle_from_quat(
            math_utils.quat_mul(_t(obj.data.root_quat_w), math_utils.quat_conjugate(previous_quat))
        )
        yaw += dq[:, 2]
    return max_displacement, min_up_dot, yaw


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path, required=True, help="grasp cache loaded by the environment reset")
    parser.add_argument("--num_envs", type=int, default=96)
    parser.add_argument("--seed", type=int, default=42000)
    parser.add_argument("--zero_steps", type=int, default=90, help="zero-action hold before the finger probe")
    parser.add_argument("--release_steps", type=int, default=12, help="steps to extend one PIP joint")
    parser.add_argument("--hold_steps", type=int, default=12, help="steps to hold the finger released")
    parser.add_argument("--output", type=Path, help="optional JSON report path")
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, overrides = setup_preset_cli(parser, argv)
    if args.num_envs < 3 or args.zero_steps < 1 or args.release_steps < 1 or args.hold_steps < 0:
        parser.error("num_envs must be >= 3; zero_steps and release_steps must be positive; hold_steps nonnegative")
    cache = args.cache.resolve(strict=True)
    env_cfg, _ = resolve_task_config("CoDe-Rotate-Cube-O", "rsl_rl_cfg_entry_point", overrides=overrides)
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.events.reset_grasp.params["cache"] = str(cache)
    env_cfg.seed = args.seed
    env_cfg.terminations.object_off_grasp = None
    env_cfg.terminations.object_tilted = None
    env_cfg.scene.object_pad_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/object",
        filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{name}" for name in PAD_LINKS],
    )
    args.device = env_cfg.sim.device

    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make("CoDe-Rotate-Cube-O", cfg=env_cfg)
        cleanup.callback(env.close)
        u = env.unwrapped
        robot, obj = u.scene["robot"], u.scene["object"]
        sensor = u.scene.sensors["object_pad_contacts"]
        reset = grasp_term(u)
        action = u.action_manager.get_term("joint_pos")
        env.reset(seed=args.seed)
        zero = torch.zeros(u.num_envs, u.action_manager.total_action_dim, device=u.device)
        initial_pos = _t(obj.data.root_pos_w).clone() - u.scene.env_origins
        initial_quat = _t(obj.data.root_quat_w).clone()
        world_up = torch.zeros(u.num_envs, 3, device=u.device)
        world_up[:, 2] = 1.0
        initial_up = math_utils.quat_apply_inverse(initial_quat, world_up)
        target_error_after_reset = float((action.targets - reset.grasp_target).abs().max())

        max_displacement = torch.zeros(u.num_envs, device=u.device)
        min_up_dot = torch.ones(u.num_envs, device=u.device)
        yaw = torch.zeros(u.num_envs, device=u.device)
        target_error_after_first_step = None
        for step in range(args.zero_steps):
            prev = _t(obj.data.root_quat_w).clone()
            env.step(zero)
            if step == 0:
                target_error_after_first_step = float(
                    (
                        _t(robot.data.joint_pos_target)[:, action._joint_ids]
                        - reset.cache["target"][reset.grasp_cache_index][:, action._joint_ids]
                    )
                    .abs()
                    .max()
                )
            displacement, up_z = _pose_state(u, initial_pos, initial_up)
            max_displacement = torch.maximum(max_displacement, displacement)
            min_up_dot = torch.minimum(min_up_dot, up_z)
            dq = math_utils.axis_angle_from_quat(
                math_utils.quat_mul(_t(obj.data.root_quat_w), math_utils.quat_conjugate(prev))
            )
            yaw += dq[:, 2]

        zero_hold = {
            "steps": args.zero_steps,
            "seconds": args.zero_steps * u.step_dt,
            "max_cube_displacement_mm_median": float((1e3 * max_displacement).median()),
            "max_cube_displacement_mm_p95": float(torch.quantile(1e3 * max_displacement, 0.95)),
            "minimum_up_dot_p05": float(torch.quantile(min_up_dot, 0.05)),
            "absolute_yaw_deg_p95": float(torch.quantile(torch.rad2deg(yaw).abs(), 0.95)),
            "target_error_after_reset_rad": target_error_after_reset,
            "target_error_after_first_zero_action_rad": target_error_after_first_step,
            "joint_target_headroom_deg_p05": _headroom_summary(action),
            **_phase_summary(u, sensor, initial_pos, initial_up),
        }
        default_safe = (max_displacement <= 0.025) & (min_up_dot >= 0.5)
        training_safe = (max_displacement <= 0.055) & (min_up_dot >= 0.35)
        zero_hold["survived_default_termination_envelope_rate"] = float(default_safe.float().mean())
        zero_hold["survived_training_termination_envelope_rate"] = float(training_safe.float().mean())
        cache_rows = [
            {
                "cache_index": int(reset.grasp_cache_index[k]),
                "max_displacement_mm": float(1e3 * max_displacement[k]),
                "min_up_dot": float(min_up_dot[k]),
                "safe_under_default_termination": bool(default_safe[k]),
                "safe_under_training_termination": bool(training_safe[k]),
            }
            for k in range(u.num_envs)
        ]

        rows = []
        pip_names = [f"{finger}_pip_servo" for finger in FINGERS]
        missing = [name for name in pip_names if name not in action._joint_names]
        if missing:
            raise RuntimeError(f"Expected PIP action joints not found: {missing}; got {action._joint_names}")
        ids_by_digit = {finger: action._joint_names.index(f"{finger}_pip_servo") for finger in FINGERS}
        for digit_index, finger in enumerate(FINGERS):
            env_ids = torch.arange(digit_index, u.num_envs, len(FINGERS), device=u.device)
            before_target = action.targets[env_ids, ids_by_digit[finger]].clone()
            open_action = torch.zeros_like(zero)
            open_action[env_ids, ids_by_digit[finger]] = -1.0
            release_max_disp, release_min_up, _ = _run_phase(
                u, open_action, args.release_steps, initial_pos, initial_up
            )
            release_summary = _phase_summary(u, sensor, initial_pos, initial_up, env_ids)
            released_target = action.targets[env_ids, ids_by_digit[finger]]
            if args.hold_steps:
                hold_max_disp, hold_min_up, _ = _run_phase(u, zero, args.hold_steps, initial_pos, initial_up)
            else:
                hold_max_disp, hold_min_up = release_max_disp, release_min_up
            hold_summary = _phase_summary(u, sensor, initial_pos, initial_up, env_ids)
            close_action = torch.zeros_like(zero)
            close_action[env_ids, ids_by_digit[finger]] = 1.0
            return_max_disp, return_min_up, _ = _run_phase(u, close_action, args.release_steps, initial_pos, initial_up)
            return_summary = _phase_summary(u, sensor, initial_pos, initial_up, env_ids)
            digit_contact, _ = _contact_state(sensor)
            release_summary["max_displacement_mm_p95"] = float(torch.quantile(1e3 * release_max_disp[env_ids], 0.95))
            release_summary["min_up_dot_p05"] = float(torch.quantile(release_min_up[env_ids], 0.05))
            hold_summary["max_displacement_mm_p95"] = float(torch.quantile(1e3 * hold_max_disp[env_ids], 0.95))
            hold_summary["min_up_dot_p05"] = float(torch.quantile(hold_min_up[env_ids], 0.05))
            return_summary["max_displacement_mm_p95"] = float(torch.quantile(1e3 * return_max_disp[env_ids], 0.95))
            return_summary["min_up_dot_p05"] = float(torch.quantile(return_min_up[env_ids], 0.05))
            rows.append(
                {
                    "released_finger": finger,
                    "env_count": len(env_ids),
                    "target_opening_deg_median": float(torch.rad2deg((before_target - released_target).abs()).median()),
                    "during_release": release_summary,
                    "after_hold": hold_summary,
                    "after_return": return_summary,
                    "released_digit_recontact_rate": float(digit_contact[env_ids, digit_index].float().mean()),
                }
            )

        report = {
            "task": "CoDe-Rotate-Cube-O",
            "cache": str(cache),
            "seed": args.seed,
            "num_envs": u.num_envs,
            "zero_action_hold": zero_hold,
            "zero_action_rows": cache_rows,
            "single_finger_release_return": rows,
        }
        report_json = json.dumps(report, indent=2, allow_nan=False)
        print(report_json)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(report_json + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
