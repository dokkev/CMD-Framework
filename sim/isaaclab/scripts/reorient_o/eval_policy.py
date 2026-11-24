# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Evaluate a ``CoDe-Reorient-Cube-<hand>`` policy: successes per episode, drops, and how the cube and the joints move.

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run --extra rsl-rl python scripts/reorient_o/eval_policy.py \
        --run <training log dir> [--task CoDe-Reorient-Cube-O|R|U] [--episodes 256] [--eval_seed 1000] physics=newton_mjwarp

The policy's deterministic (mean) action; every environment plays exactly one episode from the shared first reset,
with a fixed goal sequence per environment (``sequence_seed``), the same for every task and hand. Per episode: goals
reached, drop or time out, length; the cube's total turning about the goal axis (sum of |change| of its angle about
the axis) and the largest angle reached from rest; the goal steps; the actuated joints' motion (rms and peak to peak,
PIP in flexion) after the first second. Writes ``<run>/eval/<checkpoint>_<task>_<physics>_seed<seed>.json``.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import sys
from pathlib import Path

import gymnasium as gym
import torch

SKIP_STEPS = 30


def main(argv=None) -> int:
    from isaaclab.app import add_launcher_args

    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--checkpoint", default="last")
    parser.add_argument("--task", default="CoDe-Reorient-Cube-O")
    parser.add_argument("--episodes", type=int, default=256)
    parser.add_argument("--eval_seed", type=int, default=1000)
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args

    from importlib.metadata import version as _version

    import isaaclab.utils.math as math_utils
    from isaaclab.app import launch_simulation

    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg

    from isaaclab_tasks.utils import resolve_task_config

    import code_isaaclab.tasks  # noqa: F401

    if args.checkpoint == "last":
        checkpoint = sorted(args.run.glob("model_*.pt"), key=lambda p: int(p.stem.split("_")[1]))[-1]
    else:
        checkpoint = args.run / args.checkpoint
    physics = next((t.split("=", 1)[1] for t in hydra_args if t.startswith("physics=")), "newton_mjwarp")
    env_cfg, agent_cfg = resolve_task_config(args.task, "rsl_rl_cfg_entry_point")
    env_cfg.scene.num_envs = args.episodes
    env_cfg.seed = agent_cfg.seed = args.eval_seed
    env_cfg.commands.object_pose.sequence_seed = args.eval_seed
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, _version("rsl-rl-lib"))
    args.device = env_cfg.sim.device
    n = args.episodes

    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make(args.task, cfg=env_cfg)
        cleanup.callback(env.close)
        torch.manual_seed(args.eval_seed)
        env.reset(seed=args.eval_seed)
        wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        runner = create_rsl_rl_runner(wrapped, agent_cfg)
        runner.load(str(checkpoint))
        policy = runner.get_inference_policy(device=wrapped.unwrapped.device)
        u = env.unwrapped
        dev = u.device
        cmd = u.command_manager.get_term("object_pose")
        axis = torch.tensor(env_cfg.commands.object_pose.axis, device=dev)
        base = torch.tensor(env_cfg.commands.object_pose.base_quat, device=dev).expand(n, 4)
        obj, robot = u.scene["object"], u.scene["robot"]
        term = u.action_manager.get_term("joint_pos")
        names = list(term._joint_names)
        tq = lambda v: v if isinstance(v, torch.Tensor) else v.torch  # noqa: E731

        quat_prev = [tq(obj.data.root_quat_w).clone()]
        unwrapped = [torch.zeros(n, device=dev)]

        def angle_about_axis() -> torch.Tensor:
            """The cube's rotation from its reset orientation about the goal axis [rad], unwrapped (accumulated from
            each step's small rotation, so continuous turning keeps counting past half a turn)."""
            q_now = tq(obj.data.root_quat_w)
            step = math_utils.axis_angle_from_quat(math_utils.quat_mul(q_now, math_utils.quat_conjugate(quat_prev[0])))
            unwrapped[0] = unwrapped[0] + (step * axis).sum(dim=1)
            quat_prev[0] = q_now.clone()
            return unwrapped[0].clone()

        from isaaclab.envs import mdp as base_mdp
        from isaaclab.managers import SceneEntityCfg

        actuated = SceneEntityCfg("robot", joint_names=names, preserve_order=True)
        actuated.resolve(u.scene)
        from code_isaaclab.tasks.rotate.env_cfg import HANDS

        no_load = json.loads(HANDS[env_cfg.hand_key].read_text())["servo"]["no_load_speed_rad_s"]  # XM335, 11.1 V
        smooth = {"joint_vel_l2": 0.0, "joint_acc_l2": 0.0, "action_rate_l2": 0.0}
        smooth_steps = 0
        speed_max = []  # per step, per env: the largest actuated servo joint speed [rad/s]
        yaw_rate = []

        obs = wrapped.get_observations()
        active = torch.ones(n, dtype=torch.bool, device=dev)
        length = torch.zeros(n, device=dev)
        goals = torch.zeros(n, device=dev)
        dropped = torch.zeros(n, dtype=torch.bool, device=dev)
        turned = torch.zeros(n, device=dev)
        peak = torch.zeros(n, device=dev)
        steps_sum = torch.zeros(n, device=dev)
        prev = angle_about_axis()
        start = prev.clone()
        net = torch.zeros(n, device=dev)
        lo_ang, hi_ang = prev.clone(), prev.clone()
        last_success_angle = prev.clone()
        per_success = []  # (env, |angle change since the previous success or the start| [rad], goal angle, cube angle)
        metric_last = torch.zeros(n, device=dev)  # upstream's consecutive_success at the last live step
        metric_after_reset = []  # its value right after each episode reset (must be 0)
        joints = []
        for _ in range(int(u.max_episode_length) + 2):
            goal_before = cmd.goal_angle.clone()
            with torch.inference_mode():
                obs, _, dones, _ = wrapped.step(policy(obs))
            done = dones.bool()
            live = active & ~done
            reached = live & (cmd.goal_angle != goal_before)
            goals += reached.float()
            steps_sum += torch.where(reached, (cmd.goal_angle - goal_before).abs(), torch.zeros_like(steps_sum))
            ang = angle_about_axis()
            if live.any():
                smooth["joint_vel_l2"] += float(base_mdp.joint_vel_l2(u, actuated)[live].mean())
                smooth["joint_acc_l2"] += float(base_mdp.joint_acc_l2(u, actuated)[live].mean())
                smooth["action_rate_l2"] += float(base_mdp.action_rate_l2(u)[live].mean())
                smooth_steps += 1
            speed_max.append(torch.where(live, tq(robot.data.joint_vel)[:, actuated.joint_ids].abs().amax(dim=1),
                                         torch.nan))
            yaw_rate.append(torch.where(live, (ang - prev) / u.step_dt, torch.nan))
            for e in reached.nonzero().flatten().tolist():
                per_success.append((e, float((ang[e] - last_success_angle[e]).abs()), float(goal_before[e]), float(ang[e])))
            last_success_angle = torch.where(reached, ang, last_success_angle)
            lo_ang = torch.where(live, torch.minimum(lo_ang, ang), lo_ang)
            hi_ang = torch.where(live, torch.maximum(hi_ang, ang), hi_ang)
            consec = cmd.metrics["consecutive_success"]
            metric_last = torch.where(live, consec, metric_last)
            metric_after_reset += consec[active & done].tolist()
            net = torch.where(live, ang - start, net)
            turned += torch.where(live, (ang - prev).abs(), torch.zeros_like(turned))
            peak = torch.where(live, torch.maximum(peak, ang.abs()), peak)
            prev = ang
            q = term._to_flexion(tq(robot.data.joint_pos)[:, term._joint_ids])
            joints.append(torch.where(live[:, None], q, torch.nan))
            dropped |= active & done & u.termination_manager.get_term("object_out_of_reach").bool()
            length += active.float()
            active &= ~done
            if not bool(active.any()):
                break

    sp = torch.stack(speed_max, dim=1)
    rate = torch.stack(yaw_rate, dim=1)[:, SKIP_STEPS:]
    full = ~torch.isnan(rate[:, : int(u.max_episode_length) - SKIP_STEPS - 2]).any(dim=1)
    freq_hz = amp = None
    if bool(full.any()):
        x = rate[full][:, : int(u.max_episode_length) - SKIP_STEPS - 2]
        x = x - x.mean(dim=1, keepdim=True)
        spec = torch.fft.rfft(x, dim=1).abs() * 2.0 / x.shape[1]
        spec[:, 0] = 0.0
        f = torch.fft.rfftfreq(x.shape[1], u.step_dt).to(x.device)
        k = spec.argmax(dim=1)
        freq_hz, amp = float(f[k].median()), float(torch.rad2deg(spec.gather(1, k[:, None])).mean())
    step_deg = torch.rad2deg(torch.tensor([r[1] for r in per_success])) if per_success else torch.zeros(1)
    goal_deg = torch.rad2deg(torch.tensor([r[2] for r in per_success])) if per_success else torch.zeros(1)
    hist_edges = [0, 1, 2, 3, 5, 10, 15, 20, 30, 45, 60, 1e9]
    hist = torch.histogram(step_deg.float(), bins=torch.tensor(hist_edges, dtype=torch.float)).hist.tolist()
    q = torch.stack(joints, dim=1)[:, SKIP_STEPS:]
    rms = torch.rad2deg(torch.nanmean((q - torch.nanmean(q, dim=1, keepdim=True)) ** 2, dim=1).sqrt())
    p2p = torch.rad2deg(torch.nan_to_num(q, nan=-1e9).amax(dim=1) - torch.nan_to_num(q, nan=1e9).amin(dim=1))
    dt = u.step_dt
    summary = {
        "task": args.task, "run": str(args.run), "checkpoint": checkpoint.name, "physics": physics, "episodes": n,
        "eval_seed": args.eval_seed, "goal_axis": list(env_cfg.commands.object_pose.axis),
        "goal_max_deg": math.degrees(env_cfg.commands.object_pose.max_angle),
        "goal_min_step_deg": math.degrees(env_cfg.commands.object_pose.min_step),
        "success_threshold_deg": math.degrees(env_cfg.commands.object_pose.orientation_success_threshold),
        "successes_per_episode_mean": float(goals.mean()), "successes_per_episode_median": float(goals.median()),
        "successes_per_episode_min": float(goals.min()), "successes_per_episode_max": float(goals.max()),
        "drop_rate": float(dropped.float().mean()), "episode_s_mean": float((length * dt).mean()),
        "seconds_per_goal": float((length * dt).sum() / goals.sum().clamp_min(1)),
        "goal_step_deg_mean": float(torch.rad2deg(steps_sum.sum() / goals.sum().clamp_min(1))),
        "cube_turning_deg_per_episode_mean": float(torch.rad2deg(turned).mean()),
        "cube_turning_deg_per_goal": float(torch.rad2deg(turned.sum() / goals.sum().clamp_min(1))),
        "cube_peak_angle_from_rest_deg_mean": float(torch.rad2deg(peak).mean()),
        "cube_net_rotation_deg_mean": float(torch.rad2deg(net).mean()),
        "cube_net_rotation_turns_mean": float(torch.rad2deg(net).mean() / 360.0),
        "cube_total_rotation_turns_mean": float(torch.rad2deg(turned).mean() / 360.0),
        "cube_net_rotation_speed_deg_s_mean": float(torch.rad2deg(net / (length * u.step_dt).clamp_min(1e-6)).mean()),
        "yaw_rate_dominant_freq_hz_median": freq_hz, "yaw_rate_dominant_amplitude_deg_s_mean": amp,
        "smoothness_terms_per_step_mean": {k: v / max(smooth_steps, 1) for k, v in smooth.items()},
        "servo_no_load_speed_rad_s": no_load,
        "max_joint_speed_rad_s_p50": float(torch.nanquantile(sp.flatten()[~torch.isnan(sp.flatten())], 0.5)),
        "max_joint_speed_rad_s_p95": float(torch.nanquantile(sp.flatten()[~torch.isnan(sp.flatten())], 0.95)),
        "steps_with_a_joint_over_no_load_speed_fraction": float((sp > no_load).float().nansum() / (~torch.isnan(sp)).sum()),
        "cube_net_rotation_abs_deg_mean": float(torch.rad2deg(net).abs().mean()),
        "cube_yaw_range_deg_mean": float(torch.rad2deg(hi_ang - lo_ang).mean()),
        "cube_yaw_range_deg_median": float(torch.rad2deg(hi_ang - lo_ang).median()),
        "rotation_between_successes_deg": {
            "count": len(per_success), "median": float(step_deg.median()), "min": float(step_deg.min()),
            "mean": float(step_deg.mean()), "p10": float(step_deg.quantile(0.1)), "p90": float(step_deg.quantile(0.9)),
            "histogram_edges_deg": hist_edges[:-1] + ["inf"], "histogram_counts": [int(c) for c in hist]},
        "goal_angle_at_success_abs_deg_median": float(goal_deg.abs().median()),
        "goals_with_abs_angle_over_20deg_fraction": float((goal_deg.abs() > 20).float().mean()),
        "upstream_consecutive_success_at_end_mean": float(metric_last.mean()),
        "upstream_consecutive_success_minus_goals_mean": float((metric_last - goals).mean()),
        "upstream_consecutive_success_after_reset_max": max(metric_after_reset) if metric_after_reset else None,
        "joint_rms_deg": {nm: round(float(rms[:, k].nanmean()), 2) for k, nm in enumerate(names)},
        "joint_peak_to_peak_deg": {nm: round(float(p2p[:, k].mean()), 2) for k, nm in enumerate(names)},
    }
    out = args.run / "eval" / f"{checkpoint.stem}_{args.task}_{physics}_seed{args.eval_seed}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps(summary, indent=1))
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
