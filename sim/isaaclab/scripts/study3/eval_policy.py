# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3: evaluate one frozen checkpoint on one contact interface and one physics backend.

    uv run --extra rsl-rl python scripts/study3/eval_policy.py --run <training log dir> --test_variant crowned \
        --episodes 256 --eval_seed 1000 physics=newton_vbd

The policy is the checkpoint's deterministic (mean) action, never updated. Each environment plays exactly one
episode, from the reset all environments share at the start: with the same ``--eval_seed`` and ``--episodes``
every condition (training variant, test variant, backend) starts from the same cube poses, hand joint noise and
first goals; the script stores them so this can be checked. Per episode it records:

- whether the first goal was reached, when, and how many goals were reached in all (later goals are drawn as
  the episode goes, so only the first is matched across conditions);
- how the episode ended (time out, or the cube out of reach: a drop), and its length;
- the least orientation error to the first goal, and the error at the end;
- the mean absolute action and action change, the joint tracking error;
- the largest loop-cut residual and passive-joint closure error (an episode above ``LOOP_VALID_MM`` is flagged
  numerically invalid);
- on backends that report contacts: the pads touching the cube on average, the mean pad force, the force the
  cube receives from non-pad bodies, the fraction of steps with pad contact;
- the first 16 episodes' cube yaw and goal angle per step (for trajectory plots).

Writes ``logs/study3_pilot/eval/<run id>/`` (``episodes.csv``, ``summary.json``, ``trajectories.npz``).
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import json
import sys
from pathlib import Path

import warp as wp

wp.config.enable_backward = False

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

LOOP_VALID_MM = 1.0
TRACE_ENVS = 16


def _parse(argv):
    from isaaclab.app import add_launcher_args

    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", type=Path, required=True, help="training log directory (holds model_*.pt)")
    parser.add_argument("--checkpoint", default="last", help="model file name, or 'last'")
    parser.add_argument("--test_variant", choices=tuple(common.TASKS), required=True)
    parser.add_argument("--episodes", type=int, default=256)
    parser.add_argument("--eval_seed", type=int, default=1000)
    parser.add_argument("--out_root", type=Path, default=common.LOG_ROOT / "eval")
    parser.add_argument("--trace_all", action="store_true", help="store every episode's cube yaw (not only 16)")
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args
    return args, hydra_args


def main(argv=None) -> int:
    args, hydra_args = _parse(argv)
    from isaaclab.app import launch_simulation

    import isaaclab.utils.math as math_utils
    from importlib.metadata import version as _version

    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg

    from isaaclab_tasks.utils import resolve_task_config

    import code_isaaclab.tasks  # noqa: F401

    training = json.loads((args.run / "study3_manifest.json").read_text())
    if args.checkpoint == "last":
        checkpoint = sorted(args.run.glob("model_*.pt"), key=lambda p: int(p.stem.split("_")[1]))[-1]
    else:
        checkpoint = args.run / args.checkpoint
    physics = next((t.split("=", 1)[1] for t in hydra_args if t.startswith("physics=")), "newton_mjwarp")
    protocol = training.get("protocol", "pilot")
    task = common.PROTOCOLS[protocol][args.test_variant]
    env_cfg, agent_cfg = resolve_task_config(task, "rsl_rl_cfg_entry_point")
    if protocol == "main":  # every policy and interface scored on the same goals, one for one
        env_cfg.commands.object_pose.sequence_seed = args.eval_seed
    env_cfg.scene.num_envs = args.episodes
    env_cfg.seed = args.eval_seed
    agent_cfg.seed = args.eval_seed
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, _version("rsl-rl-lib"))
    bodies = common.add_sensors(env_cfg, physics)
    args.device = env_cfg.sim.device
    run = common.run_id(f"{training['variant']}_s{training['seed']}_on_{args.test_variant}_{physics}")
    out_dir = args.out_root / run
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
        unwrapped = env.unwrapped
        probe = common.Probe(env, bodies)
        command = unwrapped.command_manager.get_term("object_pose")
        term = unwrapped.termination_manager
        device = unwrapped.device

        obs = wrapped.get_observations()
        init_cube = (common.torch_of(unwrapped.scene["object"].data.root_pos_w) - unwrapped.scene.env_origins).clone()
        init_goal = command.goal_angle.clone()
        init_joint = common.torch_of(unwrapped.scene["robot"].data.joint_pos)[:, probe.actuated].clone()

        active = torch.ones(n, dtype=torch.bool, device=device)
        length = torch.zeros(n, device=device)
        goals = torch.zeros(n, device=device)
        first_goal_step = torch.full((n,), -1.0, device=device)
        min_err_first = torch.full((n,), float("inf"), device=device)
        final_err = torch.zeros(n, device=device)
        dropped = torch.zeros(n, dtype=torch.bool, device=device)
        timed_out = torch.zeros(n, dtype=torch.bool, device=device)
        act_abs = torch.zeros(n, device=device)
        act_rate = torch.zeros(n, device=device)
        tracking = torch.zeros(n, device=device)
        loop_max = torch.zeros(n, device=device)
        sync_max = torch.zeros(n, device=device)
        pads = torch.zeros(n, device=device)
        pad_force = torch.zeros(n, device=device)
        nonpad = torch.zeros(n, device=device)
        contact_steps = torch.zeros(n, device=device)
        yaw_trace, goal_trace = [], []
        previous_action = None
        max_steps = int(unwrapped.max_episode_length) + 2
        started = common.timer()
        for step in range(max_steps):
            goal_before = command.goal_angle.clone()
            with torch.inference_mode():
                action = policy(obs)
                obs, _, dones, _ = wrapped.step(action)
            done = dones.bool() if dones.dtype != torch.bool else dones
            m = probe.step_metrics()
            err = m["orientation_error"]
            live = active & ~done  # still in its first episode after this step (no reset happened)
            reached = live & (command.goal_angle != goal_before)
            first = reached & (goals == 0)
            first_goal_step = torch.where(first, torch.full_like(first_goal_step, step + 1), first_goal_step)
            goals += reached.float()
            in_first = active & (goals == 0)
            min_err_first = torch.where(in_first & ~done, torch.minimum(min_err_first, err), min_err_first)
            length += active.float()
            a = action.float()
            act_abs += torch.where(active, a.abs().mean(dim=1), torch.zeros_like(act_abs))
            if previous_action is not None:
                act_rate += torch.where(active, (a - previous_action).abs().mean(dim=1), torch.zeros_like(act_rate))
            previous_action = a
            tracking += torch.where(active, m["tracking_rad"], torch.zeros_like(tracking))
            loop_max = torch.where(active & ~done, torch.maximum(loop_max, m["loop_residual_mm"]), loop_max)
            sync_max = torch.where(active & ~done, torch.maximum(sync_max, m["passive_sync_rad"]), sync_max)
            if "pad_force_n" in m:
                touching = (m["pad_force_n"] > 1e-3).float().sum(dim=1)
                live_f = (active & ~done).float()
                pads += touching * live_f
                pad_force += m["pad_force_n"].sum(dim=1) * live_f
                nonpad += m["nonpad_force_on_cube_n"] * live_f
                contact_steps += (touching > 0).float() * live_f
            final_err = torch.where(active & ~done, err, final_err)
            ending = active & done
            dropped |= ending & term.get_term("object_out_of_reach").bool()
            timed_out |= ending & term.get_term("time_out").bool()
            active &= ~done
            traced = n if args.trace_all else TRACE_ENVS
            rot = math_utils.axis_angle_from_quat(m["cube_quat"][:traced])
            yaw_trace.append(torch.where(active[:traced], rot[:, 2], torch.full_like(rot[:, 2], float("nan"))).cpu())
            goal_trace.append(command.goal_angle[:traced].clone().cpu())
            if not bool(active.any()):
                break
        elapsed = common.timer() - started
        steps_live = length.clamp(min=1.0)

    to = lambda t: t.detach().cpu().numpy()  # noqa: E731
    rows = []
    for i in range(n):
        live_steps = float(steps_live[i])
        rows.append(
            {
                "episode": i,
                "train_variant": training["variant"],
                "train_seed": training["seed"],
                "test_variant": args.test_variant,
                "physics": physics,
                "init_cube_x": float(init_cube[i, 0]),
                "init_cube_y": float(init_cube[i, 1]),
                "first_goal_rad": float(init_goal[i]),
                "init_joint_l1": float(init_joint[i].abs().sum()),
                "success_first_goal": int(first_goal_step[i] >= 0),
                "steps_to_first_goal": float(first_goal_step[i]) if first_goal_step[i] >= 0 else "",
                "goals_reached": int(goals[i]),
                "dropped": int(dropped[i]),
                "timed_out": int(timed_out[i]),
                "unfinished": int(active[i]),
                "length": int(length[i]),
                "min_error_first_goal_rad": float(min_err_first[i]) if torch.isfinite(min_err_first[i]) else "",
                "final_error_rad": float(final_err[i]),
                "action_abs_mean": float(act_abs[i]) / live_steps,
                "action_rate_mean": float(act_rate[i]) / live_steps,
                "tracking_rad_mean": float(tracking[i]) / live_steps,
                "loop_residual_mm_max": float(loop_max[i]),
                "passive_sync_rad_max": float(sync_max[i]),
                "valid": int(float(loop_max[i]) <= LOOP_VALID_MM),
                "pads_in_contact_mean": float(pads[i]) / live_steps if "pad_force_n" in m else "",
                "pad_force_n_mean": float(pad_force[i]) / live_steps if "pad_force_n" in m else "",
                "nonpad_force_n_mean": float(nonpad[i]) / live_steps if "pad_force_n" in m else "",
                "contact_fraction": float(contact_steps[i]) / live_steps if "pad_force_n" in m else "",
            }
        )
    out_dir.mkdir(parents=True, exist_ok=False)
    with (out_dir / "episodes.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    np.savez(out_dir / "trajectories.npz", yaw=torch.stack(yaw_trace).numpy(), goal=torch.stack(goal_trace).numpy(),
             dt=float(unwrapped.step_dt))
    initial = np.concatenate([to(init_cube), to(init_goal)[:, None]], axis=1)
    valid = [r for r in rows if r["valid"]]
    summary = {
        "run": run,
        "train_run": str(args.run),
        "train_variant": training["variant"],
        "train_seed": training["seed"],
        "checkpoint": checkpoint.name,
        "checkpoint_sha256": common.file_sha256(checkpoint),
        "test_variant": args.test_variant,
        "task": task,
        "protocol": protocol,
        "goal_sequence_seed": env_cfg.commands.object_pose.sequence_seed,
        "physics": physics,
        "solver": {"type": type(env_cfg.sim.physics.solver_cfg).__name__,
                   "num_substeps": getattr(env_cfg.sim.physics, "num_substeps", None),
                   **{k: getattr(env_cfg.sim.physics.solver_cfg, k) for k in ("iterations", "njmax", "nconmax")
                      if hasattr(env_cfg.sim.physics.solver_cfg, k)}},
        "eval_seed": args.eval_seed,
        "episodes": n,
        "initial_state_sha256": __import__("hashlib").sha256(np.round(initial, 6).tobytes()).hexdigest(),
        "success_first_goal": float(np.mean([r["success_first_goal"] for r in rows])),
        "goals_reached_mean": float(np.mean([r["goals_reached"] for r in rows])),
        "drop_rate": float(np.mean([r["dropped"] for r in rows])),
        "valid_fraction": len(valid) / n,
        "success_first_goal_valid_only": float(np.mean([r["success_first_goal"] for r in valid])) if valid else None,
        "length_mean": float(np.mean([r["length"] for r in rows])),
        "wall_s": elapsed,
        "env_steps_per_s": float(length.sum()) / elapsed,
        "manifest": common.manifest(command=sys.argv, overrides=hydra_args),
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=str) + "\n")
    print(json.dumps({k: summary[k] for k in ("run", "success_first_goal", "goals_reached_mean", "drop_rate",
                                               "valid_fraction", "initial_state_sha256")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
