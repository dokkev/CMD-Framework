# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3, Stage B: validate a CoDe contact-interface task before any training.

    uv run python scripts/study3/validate_env.py --variant flat --policy hold --num_envs 16 --steps 300 \
        physics=newton_mjwarp

Policies (the action is the upstream absolute joint target, scaled to the joint limits):
- ``hold``: the targets of the default (extended) pose;
- ``zero``: action 0, i.e. every joint to the middle of its range (upstream's "zero action");
- ``random``: uniform in [-1, 1], seeded;
- ``flex``: hold, then every finger's FE and PIP servo flexed slowly to ``--amplitude`` of their range and back;
- ``abduct``: hold, then the three fingers' AA swept together to ``--amplitude`` of their range and back.

Per step and environment it records: joint tracking error, loop-cut residual from body poses, passive-joint
closure against the export's table, the cube's pose (rotation vector from its reset orientation), orientation
error to the goal, contact forces between each pad and the cube, contact on non-pad hand bodies, any hand
contact not explained by the cube (self-contact), finiteness, resets and their reasons, and throughput.
Writes ``logs/study3_pilot/validate/<run id>/summary.json`` with the run manifest.
"""

from __future__ import annotations

import argparse
import contextlib
import math
import sys
from pathlib import Path

import warp as wp

wp.config.enable_backward = False

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402


def _parse(argv):
    from isaaclab.app import add_launcher_args

    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--variant", choices=tuple(common.TASKS), required=True)
    parser.add_argument("--protocol", choices=tuple(common.PROTOCOLS), default="pilot")
    parser.add_argument("--policy", choices=("hold", "zero", "random", "flex", "abduct"), default="hold")
    parser.add_argument("--num_envs", type=int, default=16)
    parser.add_argument("--steps", type=int, default=300)
    parser.add_argument("--amplitude", type=float, default=0.3)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--tag", default="")
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args
    return args, hydra_args


def rotation_vector(quat_xyzw: torch.Tensor) -> torch.Tensor:
    """Axis times angle of each quaternion (world frame, from the identity)."""
    import isaaclab.utils.math as math_utils

    return math_utils.axis_angle_from_quat(quat_xyzw)


def main(argv=None) -> int:
    args, hydra_args = _parse(argv)
    from isaaclab.app import launch_simulation
    from isaaclab.utils import validate

    from isaaclab_tasks.utils import resolve_task_config

    import code_isaaclab.tasks  # noqa: F401

    task = common.PROTOCOLS[args.protocol][args.variant]
    physics = next((t.split("=", 1)[1] for t in hydra_args if t.startswith("physics=")), "newton_mjwarp")
    torch.manual_seed(args.seed)
    env_cfg, _ = resolve_task_config(task, "")
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.seed = args.seed
    bodies = common.add_sensors(env_cfg, physics)
    args.device = env_cfg.sim.device
    validate(env_cfg)
    solver_cfg = env_cfg.sim.physics.solver_cfg
    solver = {"type": type(solver_cfg).__name__, "num_substeps": getattr(env_cfg.sim.physics, "num_substeps", None)}
    for key in ("iterations", "integrator", "njmax", "nconmax", "cone", "impratio"):
        if hasattr(solver_cfg, key):
            solver[key] = getattr(solver_cfg, key)
    run = common.run_id(f"{args.variant}_{physics}_{args.policy}{('_' + args.tag) if args.tag else ''}")
    out_dir = common.LOG_ROOT / "validate" / run

    rows, finite_ok = [], True
    reasons = {"time_out": 0, "object_out_of_reach": 0}
    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make(task, cfg=env_cfg)
        cleanup.callback(env.close)
        obs, _ = env.reset(seed=args.seed)
        unwrapped = env.unwrapped
        probe = common.Probe(env, bodies)
        hold = probe.hold_action()
        joint_names = [unwrapped.scene["robot"].joint_names[i] for i in probe.actuated]
        shapes = {
            "action": list(unwrapped.action_manager.action.shape),
            "observation": {k: list(v.shape) for k, v in obs.items()},
            "actuated_joints": joint_names,
            "robot_joints": list(unwrapped.scene["robot"].joint_names),
        }
        generator = torch.Generator(device=unwrapped.device).manual_seed(args.seed)
        fe = [i for i, n in enumerate(joint_names) if n.endswith(("_fe", "_pip_servo")) and not n.startswith("thumb")]
        aa = [i for i, n in enumerate(joint_names) if n.endswith("_aa")]
        start_rot = rotation_vector(common.torch_of(unwrapped.scene["object"].data.root_quat_w)).clone()
        started = common.timer()
        for step in range(args.steps):
            phase = math.sin(math.pi * step / max(args.steps - 1, 1)) if step > 30 else 0.0
            if args.policy == "hold":
                action = hold.clone()
            elif args.policy == "zero":
                action = torch.zeros_like(hold)
            elif args.policy == "random":
                action = 2.0 * torch.rand(hold.shape, generator=generator, device=hold.device) - 1.0
            else:
                action = hold.clone()
                for i in fe if args.policy == "flex" else aa:
                    action[:, i] = hold[:, i] + 2.0 * args.amplitude * phase
                action = action.clamp(-1.0, 1.0)
            with torch.inference_mode():
                obs, reward, terminated, truncated, extras = env.step(action)
            metrics = probe.step_metrics()
            term = unwrapped.termination_manager
            for name in reasons:
                reasons[name] += int(term.get_term(name).sum())
            rot = rotation_vector(metrics["cube_quat"])
            finite_obs = all(bool(torch.isfinite(v).all()) for v in obs.values()) and bool(torch.isfinite(reward).all())
            finite_ok &= finite_obs and bool(metrics["finite"].all())
            row = {
                "step": step,
                "tracking_rad": float(metrics["tracking_rad"].mean()),
                "loop_residual_mm_max": float(metrics["loop_residual_mm"].max()),
                "passive_sync_rad_max": float(metrics["passive_sync_rad"].max()),
                "cube_height_m": float(metrics["cube_pos"][:, 2].mean()),
                "cube_rot_x": float(rot[:, 0].mean()),
                "cube_rot_y": float(rot[:, 1].mean()),
                "cube_rot_z": float(rot[:, 2].mean()),
                "cube_rot_abs_x": float(rot[:, 0].abs().mean()),
                "cube_rot_abs_y": float(rot[:, 1].abs().mean()),
                "cube_rot_abs_z": float(rot[:, 2].abs().mean()),
                "orientation_error": float(metrics["orientation_error"].mean()),
                "reward": float(reward.mean()),
                "resets": int((terminated | truncated).sum()),
            }
            if "pad_force_n" in metrics:
                touching = metrics["pad_force_n"] > 1e-3
                row.update(
                    pads_in_contact=float(touching.float().sum(dim=1).mean()),
                    pad_force_n=float(metrics["pad_force_n"].sum(dim=1).mean()),
                    nonpad_force_on_cube_n=float(metrics["nonpad_force_on_cube_n"].mean()),
                    stray_contact_n_max=float(metrics.get("stray_contact_n", torch.zeros(1)).max()),
                )
            rows.append(row)
        elapsed = common.timer() - started
        del start_rot

    keys = [k for k in rows[0] if k not in ("step",)]
    summary = {
        "run": run,
        "task": task,
        "protocol": args.protocol,
        "variant": args.variant,
        "physics": physics,
        "solver": solver,
        "policy": args.policy,
        "amplitude": args.amplitude if args.policy in ("flex", "abduct") else None,
        "seed": args.seed,
        "num_envs": args.num_envs,
        "steps": args.steps,
        "all_finite": finite_ok,
        "terminations": reasons,
        "env_steps_per_s": args.num_envs * args.steps / elapsed,
        "wall_s": elapsed,
        "shapes": shapes,
        "mean": {k: sum(r[k] for r in rows) / len(rows) for k in keys},
        "max": {k: max(r[k] for r in rows) for k in keys},
        "final": rows[-1],
        "rows": rows,
        "manifest": common.manifest(command=sys.argv, overrides=hydra_args),
    }
    path = common.write_json(out_dir / "summary.json", summary)
    brief = {k: summary[k] for k in ("run", "all_finite", "terminations", "env_steps_per_s")}
    print(brief)
    for k in ("tracking_rad", "loop_residual_mm_max", "passive_sync_rad_max", "cube_height_m", "orientation_error",
              "pads_in_contact", "pad_force_n", "nonpad_force_on_cube_n", "stray_contact_n_max",
              "cube_rot_abs_x", "cube_rot_abs_y", "cube_rot_abs_z"):
        if k in summary["mean"]:
            print(f"  {k:24s} mean {summary['mean'][k]:9.4f}  max {summary['max'][k]:9.4f}  final {summary['final'][k]:9.4f}")
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
