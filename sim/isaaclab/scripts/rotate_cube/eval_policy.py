# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Evaluate deterministic cube rotation on the first episode of each seeded grasp.

Run from sim/isaaclab with `uv run --extra rsl-rl python scripts/rotate_cube/eval_policy.py
--checkpoint <model.pt>`. Pass Isaac Lab overrides for departures from the standard O actor
or evaluation environment. Use the same overrides and seed when comparing checkpoints.
"""

from __future__ import annotations

import argparse
import json
import math
from contextlib import ExitStack
from hashlib import sha256
from importlib.metadata import version
from pathlib import Path

import gymnasium as gym
import torch

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.managers import RecorderTerm, RecorderTermCfg
from isaaclab.managers.recorder_manager import DatasetExportMode, RecorderManagerBaseCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass
from isaaclab.utils import math as math_utils

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg

from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli

import code_isaaclab.tasks  # noqa: F401
from code_isaaclab.tasks.rotate_cube.rotate_cube_mdp import _t, actuated_targets_normalized, grasp_term


class CubeRotationMetrics(RecorderTerm):
    """Accumulate actual pose increments before Isaac Lab resets completed environments."""

    def record_pre_step(self):
        self.previous_quat = _t(self._env.scene["object"].data.root_quat_w).clone()
        return None, None

    def record_post_step(self):
        env = self._env
        obj = env.scene["object"]
        quat = _t(obj.data.root_quat_w)
        dq = math_utils.axis_angle_from_quat(math_utils.quat_mul(quat, math_utils.quat_conjugate(self.previous_quat)))
        displacement = _t(obj.data.root_pos_w) - env.scene.env_origins - grasp_term(env).grasp_pos
        self.yaw += torch.where(self.active, dq[:, 2], 0.0)
        second_half = self.active & (self.length >= env.max_episode_length // 2)
        self.late_yaw += torch.where(second_half, dq[:, 2], 0.0)
        if self.contacts is not None:
            forces = _t(self.contacts.data.force_matrix_w)[:, 0].norm(dim=-1)
            touch = forces > 0.05
            self.thumb_finger_steps += self.active & touch[:, self.thumb_column] & (touch.sum(dim=1) >= 2)
            self.no_contact_steps += self.active & ~touch.any(dim=1)
            digit_touch = torch.stack([touch[:, cols].any(dim=1) for cols in self.digit_columns], dim=1)
            self.digit_contact_steps += self.active[:, None] & digit_touch
            if self.prev_digit_touch is None:
                self.prev_digit_touch = digit_touch.clone()
            else:
                self.finger_release_events += self.active[:, None] & self.prev_digit_touch & ~digit_touch
                self.finger_recontact_events += self.active[:, None] & ~self.prev_digit_touch & digit_touch
                self.prev_digit_touch = torch.where(self.active[:, None], digit_touch, self.prev_digit_touch)
        self.off_axis += torch.where(self.active, dq[:, :2].norm(dim=1), 0.0)
        self.max_displacement = torch.where(
            self.active, torch.maximum(self.max_displacement, displacement.norm(dim=1)), self.max_displacement
        )
        up_z = math_utils.quat_apply(quat, self.initial_up_body)[:, 2]
        self.min_up_z = torch.where(self.active, torch.minimum(self.min_up_z, up_z), self.min_up_z)
        old_heading = math_utils.quat_apply(self.previous_quat, self.initial_x_body)
        new_heading = math_utils.quat_apply(quat, self.initial_x_body)
        cross_z = old_heading[:, 0] * new_heading[:, 1] - old_heading[:, 1] * new_heading[:, 0]
        dot_xy = (old_heading[:, :2] * new_heading[:, :2]).sum(dim=1)
        self.heading += torch.where(self.active, torch.atan2(cross_z, dot_xy), 0.0)
        self.length += self.active
        tm = env.termination_manager
        self.off_grasp |= self.active & tm.get_term("object_off_grasp")
        self.tilted |= self.active & tm.get_term("object_tilted")
        self.timeout |= self.active & tm.time_outs
        if self.trace is not None:
            targets = actuated_targets_normalized(env)
            values = torch.stack((
                self.yaw, self.heading, displacement.norm(dim=1), up_z,
                (targets.abs() >= 0.99).float().mean(dim=1),
                env.action_manager.action.clamp(-1.0, 1.0).square().mean(dim=1).sqrt(),
            ), dim=-1)
            self.trace[self.trace_step] = torch.where(self.active[:, None], values, torch.nan)
            self.target_trace[self.trace_step] = torch.where(self.active[:, None], targets, torch.nan)
            self.trace_step += 1
        if self.capture_grasps:
            low_speed = _t(obj.data.root_lin_vel_w).norm(dim=1) < 0.005
            keep = (
                self.active & tm.time_outs & ~tm.terminated & low_speed
                & (self.max_displacement <= 0.04) & (self.min_up_z >= 0.5)
                & (torch.minimum(self.yaw, self.heading) >= math.pi / 4)
                & touch[:, self.thumb_column] & (touch.sum(dim=1) >= 2)
            )
            robot = env.scene["robot"]
            self.grasp_q[keep] = _t(robot.data.joint_pos)[keep]
            self.grasp_target[keep] = _t(robot.data.joint_pos_target)[keep]
            self.grasp_pos[keep] = (_t(obj.data.root_pos_w) - env.scene.env_origins)[keep]
            self.grasp_quat[keep] = quat[keep]
            self.grasp_valid |= keep
        self.active &= ~env.reset_buf
        return None, None

    def start(self, capture_grasps=False, capture_trace=False):
        n, device = self._env.num_envs, self._env.device
        self.capture_grasps = capture_grasps
        if capture_grasps:
            robot = self._env.scene["robot"]
            self.grasp_q = torch.zeros_like(_t(robot.data.joint_pos))
            self.grasp_target = torch.zeros_like(self.grasp_q)
            self.grasp_pos = torch.zeros(n, 3, device=device)
            self.grasp_quat = torch.zeros(n, 4, device=device)
            self.grasp_valid = torch.zeros(n, dtype=torch.bool, device=device)
        self.active = torch.ones(n, dtype=torch.bool, device=device)
        self.trace = None
        if capture_trace:
            self.trace_step = 0
            self.trace = torch.empty(self._env.max_episode_length, n, 6, device=device)
            self.target_names = list(self._env.action_manager.get_term("joint_pos")._joint_names)
            self.target_trace = torch.empty(self._env.max_episode_length, n, len(self.target_names), device=device)
        self.yaw, self.off_axis, self.max_displacement, self.length = torch.zeros(4, n, device=device)
        self.heading = torch.zeros_like(self.yaw)
        self.late_yaw = torch.zeros_like(self.yaw)
        self.contacts = self._env.scene.sensors.get("object_pad_contacts")
        if self.contacts is not None:
            columns = {name: k for k, name in enumerate(self.contacts.filter_object_names)}
            self.digit_columns = [
                [columns[f"{finger}_proximal"], columns[f"{finger}_distal"]]
                for finger in ("index", "middle", "ring")
            ]
            self.thumb_column = columns["thumb_distal"]
            self.thumb_finger_steps = torch.zeros_like(self.yaw)
            self.no_contact_steps = torch.zeros_like(self.yaw)
            self.digit_contact_steps = torch.zeros(n, 3, device=device)
            self.finger_release_events = torch.zeros(n, 3, dtype=torch.long, device=device)
            self.finger_recontact_events = torch.zeros(n, 3, dtype=torch.long, device=device)
            self.prev_digit_touch = None
        self.off_grasp, self.tilted, self.timeout = torch.zeros(3, n, dtype=torch.bool, device=device)
        up = self.yaw.new_tensor((0., 0., 1.)).expand(n, 3)
        quat = _t(self._env.scene["object"].data.root_quat_w)
        self.initial_up_body = math_utils.quat_apply_inverse(quat, up)
        x = self.yaw.new_tensor((1., 0., 0.)).expand(n, 3)
        self.initial_x_body = math_utils.quat_apply_inverse(quat, x)
        self.min_up_z = torch.ones_like(self.yaw)

    def summarize(self):
        seconds = self.length * self._env.step_dt
        clean_timeout = self.timeout & ~(self.off_grasp | self.tilted)
        yaw_deg = torch.rad2deg(self.yaw)
        peak_tilt_deg = torch.rad2deg(self.min_up_z.clamp(-1.0, 1.0).acos())
        retained = clean_timeout & (self.max_displacement <= 0.04) & (peak_tilt_deg <= 60.0)
        turn = torch.minimum(self.yaw, self.heading)
        summary = {
            "clean_timeout_rate": float(clean_timeout.float().mean()),
            "off_grasp_rate": float(self.off_grasp.float().mean()),
            "tilted_rate": float(self.tilted.float().mean()),
            "duration_s_mean": float(seconds.mean()),
            "signed_rotation_deg_mean": float(yaw_deg.mean()),
            "projected_heading_change_deg_mean": float(torch.rad2deg(self.heading).mean()),
            "signed_rotation_deg_p10_p50_p90": yaw_deg.quantile(yaw_deg.new_tensor([0.1, 0.5, 0.9])).tolist(),
            "clean_timeout_rotation_deg_mean": float(yaw_deg[clean_timeout].mean()) if clean_timeout.any() else None,
            "clean_timeout_second_half_rotation_deg_mean": (
                float(torch.rad2deg(self.late_yaw[clean_timeout]).mean()) if clean_timeout.any() else None
            ),
            "signed_rate_rad_s_mean": float((self.yaw / seconds).mean()),
            "off_axis_rate_rad_s_mean": float((self.off_axis / seconds).mean()),
            "max_displacement_mm_mean": float(1e3 * self.max_displacement.mean()),
            "peak_tilt_deg_median": float(peak_tilt_deg.quantile(0.5)),
            "quarter_turn_within_40mm_and_60deg_rate": float(((turn >= math.pi / 2) & retained).float().mean()),
            "half_turn_within_40mm_and_60deg_rate": float(((turn >= math.pi) & retained).float().mean()),
            "three_quarter_turn_within_40mm_and_60deg_rate": (
                float(((turn >= 3 * math.pi / 2) & retained).float().mean())
            ),
            "full_turn_and_clean_timeout_rate": float(((turn >= math.tau) & clean_timeout).float().mean()),
            "full_turn_within_40mm_and_60deg_rate": float(((turn >= math.tau) & retained).float().mean()),
            "max_retained_rotation_deg": float(torch.rad2deg(turn[retained].max())) if retained.any() else None,
        }
        if self.contacts is not None:
            summary["thumb_and_finger_link_contact_fraction"] = float((self.thumb_finger_steps / self.length).mean())
            summary["no_pad_link_contact_fraction"] = float((self.no_contact_steps / self.length).mean())
            summary["distinct_finger_contact_count_mean"] = float(
                (self.digit_contact_steps.sum(dim=1) / self.length.clamp_min(1)).mean()
            )
            summary["per_finger_contact_fraction"] = {
                finger: float((self.digit_contact_steps[:, k] / self.length.clamp_min(1)).mean())
                for k, finger in enumerate(("index", "middle", "ring"))
            }
            summary["finger_contact_release_events_mean"] = {
                finger: float(self.finger_release_events[:, k].float().mean())
                for k, finger in enumerate(("index", "middle", "ring"))
            }
            summary["finger_contact_recontact_events_mean"] = {
                finger: float(self.finger_recontact_events[:, k].float().mean())
                for k, finger in enumerate(("index", "middle", "ring"))
            }
            summary["episodes_with_finger_release_rate"] = {
                finger: float((self.finger_release_events[:, k] > 0).float().mean())
                for k, finger in enumerate(("index", "middle", "ring"))
            }
            summary["episodes_with_finger_recontact_rate"] = {
                finger: float((self.finger_recontact_events[:, k] > 0).float().mean())
                for k, finger in enumerate(("index", "middle", "ring"))
            }
        if self.capture_grasps:
            summary["export_grasp_count"] = int(self.grasp_valid.sum())
        return summary

    def save_grasps(self, path: Path, checkpoint: Path, cache: Path, seed: int):
        from code_isaaclab.tasks.reorient.o_env_cfg import CUBE_SIZE_M, GRASP_MCP_DEG, GRASP_PIP_DEG

        keep = self.grasp_valid
        if not keep.any():
            raise RuntimeError("No settled, retained rotation end states available to export")
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as output:
            torch.save({
                "q": self.grasp_q[keep].cpu(),
                "target": self.grasp_target[keep].cpu(),
                "pos": self.grasp_pos[keep].cpu(),
                "quat": self.grasp_quat[keep].cpu(),
                "joint_names": list(self._env.scene["robot"].joint_names),
                "hand": self._env.cfg.hand_key,
                "cube_size_m": CUBE_SIZE_M,
                "grasp_mcp_deg": GRASP_MCP_DEG,
                "grasp_pip_deg": GRASP_PIP_DEG,
                "pad_crown_mm": self._env.cfg.pad_crown_mm,
                "pad_crown_layout": self._env.cfg.pad_crown_layout,
                "source_checkpoint": str(checkpoint),
                "source_grasp_cache": str(cache),
                "source_seed": seed,
            }, output)


@configclass
class EvaluationRecorderCfg(RecorderManagerBaseCfg):
    dataset_export_mode = DatasetExportMode.EXPORT_NONE
    rotation = RecorderTermCfg(class_type=CubeRotationMetrics)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--checkpoint", type=Path, action="append", required=True, help="repeat to compare checkpoints")
    parser.add_argument("--task", default="CoDe-Rotate-Cube-O")
    parser.add_argument("--num_envs", type=int, default=64)
    parser.add_argument("--eval_seed", type=int, default=1000)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--trace", type=Path, help="save first-episode trajectories to a compressed NumPy file")
    parser.add_argument("--contact_metrics", action="store_true", help="measure contact with pad-carrying links")
    parser.add_argument("--export_grasps", type=Path, help="save settled 45-degree end states as curriculum starts")
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, overrides = setup_preset_cli(parser, argv)
    if args.num_envs < 1:
        parser.error("--num_envs must be positive")
    if (args.output or args.export_grasps or args.trace) and len(args.checkpoint) != 1:
        parser.error("--output, --export_grasps and --trace require exactly one checkpoint")
    for path in (args.export_grasps, args.trace):
        if path and path.exists():
            parser.error(f"destination already exists: {path}")
    contact_metrics = args.contact_metrics or args.export_grasps is not None
    checkpoints = [path.resolve(strict=True) for path in args.checkpoint]
    env_cfg, agent_cfg = resolve_task_config(args.task, "rsl_rl_cfg_entry_point", overrides=overrides)
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.seed = agent_cfg.seed = args.eval_seed
    env_cfg.recorders = EvaluationRecorderCfg()
    if contact_metrics:
        pad_links = [f"{finger}_{link}" for finger in ("index", "middle", "ring") for link in ("proximal", "distal")]
        pad_links.append("thumb_distal")
        env_cfg.scene.object_pad_contacts = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/object",
            filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{b}" for b in pad_links],
        )
    cache = Path(env_cfg.events.reset_grasp.params["cache"]).resolve(strict=True)
    settings = {
        "task": args.task,
        "eval_seed": args.eval_seed,
        "num_envs": args.num_envs,
        "overrides": overrides,
        "episode_s_max": env_cfg.episode_length_s,
        "policy_observation_noise": env_cfg.observations.policy.enable_corruption,
        "grasp_cache": str(cache),
        "grasp_cache_sha256": sha256(cache.read_bytes()).hexdigest(),
        "pad_crown_mm": env_cfg.pad_crown_mm,
        "pad_crown_layout": env_cfg.pad_crown_layout,
        "phase_period_s": env_cfg.phase_period_s,
        "observe_rotation_goal": env_cfg.observe_rotation_goal,
        "contact_metrics": contact_metrics,
        "export_grasps": args.export_grasps is not None,
        "off_grasp_distance_m": env_cfg.terminations.object_off_grasp.params["max_distance"],
        "tilt_min_dot": env_cfg.terminations.object_tilted.params["min_dot"],
    }
    settings_id = sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()[:10]
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, version("rsl-rl-lib"))
    args.device = env_cfg.sim.device

    with launch_simulation(env_cfg, args), ExitStack() as cleanup, torch.inference_mode():
        env = gym.make(args.task, cfg=env_cfg)
        cleanup.callback(env.close)
        wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        runner = create_rsl_rl_runner(wrapped, agent_cfg)
        # Isaac Lab's recorder manager exposes term names but no public instance accessor.
        metrics = env.unwrapped.recorder_manager._terms["rotation"]
        for checkpoint in checkpoints:
            runner.load(str(checkpoint), load_cfg={"actor": True}, map_location=wrapped.device)
            policy = runner.get_inference_policy(device=wrapped.device)
            env.reset(seed=args.eval_seed)
            metrics.start(capture_grasps=args.export_grasps is not None, capture_trace=args.trace is not None)
            obs = wrapped.get_observations()
            for _ in range(wrapped.max_episode_length):
                obs, _, _, _ = wrapped.step(policy(obs))
                if not metrics.active.any():
                    break
            if metrics.active.any():
                raise RuntimeError("Evaluation ended before every first episode terminated")
            result = {
                "checkpoint": str(checkpoint),
                **settings,
                **metrics.summarize(),
            }
            name = f"{checkpoint.stem}_rotation_seed{args.eval_seed}_{settings_id}.json"
            out = args.output or checkpoint.parent / "eval" / name
            out.parent.mkdir(parents=True, exist_ok=True)
            report = json.dumps(result, indent=2, allow_nan=False)
            out.write_text(report + "\n")
            print(report)
            print(out)
            if args.export_grasps:
                metrics.save_grasps(args.export_grasps, checkpoint, cache, args.eval_seed)
            if args.trace:
                import numpy as np

                args.trace.parent.mkdir(parents=True, exist_ok=True)
                with args.trace.open("xb") as stream:
                    np.savez_compressed(
                        stream, steps=metrics.trace[: metrics.trace_step].cpu().numpy(),
                        columns=("net_yaw_rad", "heading_rad", "displacement_m", "up_dot",
                                 "target_limit_fraction", "action_rms"),
                        normalized_targets=metrics.target_trace[: metrics.trace_step].cpu().numpy(),
                        target_names=metrics.target_names,
                        step_dt=env.unwrapped.step_dt,
                        clean_timeout=(metrics.timeout & ~(metrics.off_grasp | metrics.tilted)).cpu().numpy(),
                    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
