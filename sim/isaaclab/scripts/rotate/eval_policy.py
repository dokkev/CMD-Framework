# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Measure how fast a trained ``CoDe-Rotate-Cylinder-O`` policy turns the cylinder.

    uv run --extra rsl-rl python scripts/rotate/eval_policy.py --run <training log dir> [--checkpoint model_999.pt] \
        [--episodes 256] [--eval_seed 1000] physics=newton_mjwarp

The policy's deterministic (mean) action; each environment plays exactly one episode from the shared first reset.
Per episode: length, drop or time out, mean cylinder speed and off-axis angular speed, net rotation about the cylinder's axis in the rewarded direction (the integral
of its spin, so back-and-forth motion cancels), mean spin rate over the episode, the fraction of steps spinning the
rewarded way, the largest distance from the resting place, and the rotation reward collected. Also: the mean distance from the resting place, the joint-velocity L2, the spin ripple's dominant frequency and
amplitude, and each actuated joint's motion (rms and peak to peak, PIP in flexion) after the first second. Writes
``<run>/eval/<checkpoint>_<task>_<physics>_seed<seed>.json`` and prints a summary.
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

TASK = "CoDe-Rotate-Cylinder-O"


SKIP_STEPS = 30
"""Steps (1 s) left out of the ripple and joint-motion statistics (the start of the gait)."""


def spin_ripple(spin: torch.Tensor, dt: float) -> dict:
    """Ripple of the spin about its mean, per environment over its live steps after :data:`SKIP_STEPS`: the dominant
    frequency of its spectrum (environments that ran at least 4 s), that component's amplitude, and the rms ripple."""
    s = spin[:, SKIP_STEPS:]
    live = (~torch.isnan(s)).float().mean(dim=0) >= 0.5  # columns most environments are still live in
    s = s[:, : int(live.long().cumprod(dim=0).sum())]  # (the time-out step itself is not live)
    full = ~torch.isnan(s).any(dim=1)
    rms = torch.nanmean((s - torch.nanmean(s, dim=1, keepdim=True)) ** 2, dim=1).sqrt()
    out = {"spin_ripple_rms_rad_s_mean": float(rms.nanmean())}
    if int(full.sum()) and s.shape[1] * dt >= 4.0:
        x = s[full] - s[full].mean(dim=1, keepdim=True)
        spec = torch.fft.rfft(x, dim=1).abs() * 2.0 / x.shape[1]
        spec[:, 0] = 0.0
        freq = torch.fft.rfftfreq(x.shape[1], dt).to(x.device)
        k = spec.argmax(dim=1)
        out.update({"spin_ripple_freq_hz_median": float(freq[k].median()),
                    "spin_ripple_peak_amplitude_rad_s_mean": float(spec.gather(1, k[:, None]).mean()),
                    "spin_ripple_spectrum_mean_top3_hz": [round(float(f), 2) for f in
                                                          freq[spec.mean(dim=0).topk(3).indices]]})
    return out


def main(argv=None) -> int:
    from isaaclab.app import add_launcher_args

    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--checkpoint", default="last")
    parser.add_argument("--episodes", type=int, default=256)
    parser.add_argument("--eval_seed", type=int, default=1000)
    parser.add_argument("--task", default=None, help="task id (default: the run's run.json, else CoDe-Rotate-Cylinder-O)")
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args

    from importlib.metadata import version as _version

    from isaaclab.app import launch_simulation

    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg

    from isaaclab_tasks.utils import resolve_task_config

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.tasks.rotate import rotate_mdp
    from code_isaaclab.tasks.rotate.env_cfg import ROTATION_DIRECTION, SPIN_CLIP

    if args.checkpoint == "last":
        checkpoint = sorted(args.run.glob("model_*.pt"), key=lambda p: int(p.stem.split("_")[1]))[-1]
    else:
        checkpoint = args.run / args.checkpoint
    run_info = args.run / "run.json"
    task = args.task or (json.loads(run_info.read_text()).get("task") if run_info.exists() else None) or TASK
    physics = next((t.split("=", 1)[1] for t in hydra_args if t.startswith("physics=")), "newton_mjwarp")
    env_cfg, agent_cfg = resolve_task_config(task, "rsl_rl_cfg_entry_point")
    env_cfg.scene.num_envs = args.episodes
    env_cfg.seed = agent_cfg.seed = args.eval_seed
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, _version("rsl-rl-lib"))
    args.device = env_cfg.sim.device
    n = args.episodes

    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make(task, cfg=env_cfg)
        cleanup.callback(env.close)
        torch.manual_seed(args.eval_seed)
        env.reset(seed=args.eval_seed)
        wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        runner = create_rsl_rl_runner(wrapped, agent_cfg)
        runner.load(str(checkpoint))
        policy = runner.get_inference_policy(device=wrapped.unwrapped.device)
        u = env.unwrapped
        dev, dt = u.device, u.step_dt
        dropped_term = lambda: u.termination_manager.get_term("object_dropped").bool()  # noqa: E731

        obs = wrapped.get_observations()
        active = torch.ones(n, dtype=torch.bool, device=dev)
        length = torch.zeros(n, device=dev)
        angle = torch.zeros(n, device=dev)
        reward = torch.zeros(n, device=dev)
        forward = torch.zeros(n, device=dev)
        max_dev = torch.zeros(n, device=dev)
        speed = torch.zeros(n, device=dev)
        wobble = torch.zeros(n, device=dev)
        dist = torch.zeros(n, device=dev)
        joint_vel_l2 = torch.zeros(n, device=dev)
        cyl, robot = u.scene["object"], u.scene["robot"]
        term = u.action_manager.get_term("joint_pos")
        joint_names = list(term._joint_names)
        spins, joints = [], []  # every environment's spin and actuated joint positions (PIP in flexion), per step
        dropped = torch.zeros(n, dtype=torch.bool, device=dev)
        trace = []
        for _ in range(int(u.max_episode_length) + 2):
            with torch.inference_mode():
                obs, _, dones, _ = wrapped.step(policy(obs))
            done = dones.bool()
            live = active & ~done  # the state read below is still this episode's (not a reset)
            spin = ROTATION_DIRECTION * rotate_mdp.axis_spin(u)[:, 0]
            angle += torch.where(live, spin * dt, torch.zeros_like(angle))
            reward += torch.where(live, spin.clamp(*SPIN_CLIP) * dt, torch.zeros_like(reward))
            forward += (live & (spin > 0)).float()
            max_dev = torch.where(live, torch.maximum(max_dev, rotate_mdp.object_pos_deviation(u)), max_dev)
            v = rotate_mdp._t(cyl.data.root_lin_vel_w)
            w = rotate_mdp._t(cyl.data.root_ang_vel_w)
            w_perp = w - (w * rotate_mdp.cylinder_axis_w(u)).sum(1, keepdim=True) * rotate_mdp.cylinder_axis_w(u)
            speed += torch.where(live, v.norm(dim=1), torch.zeros_like(speed))
            wobble += torch.where(live, w_perp.norm(dim=1), torch.zeros_like(wobble))
            dist += torch.where(live, rotate_mdp.object_pos_deviation(u), torch.zeros_like(dist))
            qd = rotate_mdp._t(robot.data.joint_vel)[:, term._joint_ids]
            joint_vel_l2 += torch.where(live, (qd ** 2).sum(dim=1), torch.zeros_like(joint_vel_l2))
            q = term._to_flexion(rotate_mdp._t(robot.data.joint_pos)[:, term._joint_ids])
            spins.append(torch.where(live, spin, torch.nan))
            joints.append(torch.where(live[:, None], q, torch.nan))
            length += active.float()
            dropped |= active & done & dropped_term()
            if len(trace) < int(u.max_episode_length):
                trace.append([round(float(x), 4) for x in torch.where(live[:8], spin[:8], torch.nan)])
            active &= ~done
            if not bool(active.any()):
                break

    secs = length * dt
    ripple = spin_ripple(torch.stack(spins, dim=1), dt)
    q = torch.stack(joints, dim=1)[:, SKIP_STEPS:]  # (envs, steps, joints)
    amp = torch.rad2deg(torch.nanmean((q - torch.nanmean(q, dim=1, keepdim=True)) ** 2, dim=1).sqrt())  # rms, per env
    peak = torch.rad2deg(torch.nan_to_num(q, nan=-1e9).amax(dim=1) - torch.nan_to_num(q, nan=1e9).amin(dim=1))
    rate = angle / secs.clamp_min(dt)
    full = ~dropped
    summary = {
        "task": task, "run": str(args.run), "checkpoint": checkpoint.name, "physics": physics, "episodes": n,
        "eval_seed": args.eval_seed, "episode_s_max": float(u.max_episode_length * dt),
        "drop_rate": float(dropped.float().mean()),
        "episode_s_mean": float(secs.mean()),
        "net_rotation_rad_mean": float(angle.mean()),
        "net_rotation_rad_median": float(angle.median()),
        "net_rotation_rad_mean_full_episodes": float(angle[full].mean()) if bool(full.any()) else None,
        "spin_rate_rad_s_mean": float(rate.mean()),
        "spin_rate_rad_s_median": float(rate.median()),
        "spin_rate_rad_s_p90": float(rate.quantile(0.9)),
        "forward_fraction_mean": float((forward / length.clamp_min(1)).mean()),
        "rotation_reward_integral_mean": float(reward.mean()),
        "max_displacement_mm_mean": float(1e3 * max_dev.mean()),
        "object_speed_mm_s_mean": float(1e3 * (speed / length.clamp_min(1)).mean()),
        "off_axis_ang_vel_rad_s_mean": float((wobble / length.clamp_min(1)).mean()),
        "distance_from_rest_mm_mean": float(1e3 * (dist / length.clamp_min(1)).mean()),
        "joint_vel_l2_mean": float((joint_vel_l2 / length.clamp_min(1)).mean()),
        **ripple,
        "joint_motion_after_s": SKIP_STEPS * dt,
        "joint_rms_deg": {nm: round(float(amp[:, k].nanmean()), 2) for k, nm in enumerate(joint_names)},
        "joint_peak_to_peak_deg": {nm: round(float(peak[:, k].mean()), 2) for k, nm in enumerate(joint_names)},
        "spin_trace_first8_rad_s": trace,
    }
    out = args.run / "eval" / f"{checkpoint.stem}_{task}_{physics}_seed{args.eval_seed}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "spin_trace_first8_rad_s"}, indent=1))
    print(f"net rotation per episode: {math.degrees(summary['net_rotation_rad_mean']):.1f} deg mean; written {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
