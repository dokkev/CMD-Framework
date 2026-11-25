# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Drop reset states that diverge or slam the cube when started under the training physics.

    env -u DISPLAY PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run python scripts/rotate_cube/filter_omnireset_resets.py \\
        --in_dir <datasets> --out_dir <filtered> [--num_envs 4096] [--steps 10] [--max_force_n 200] \\
        physics=newton_mjwarp

The OmniReset datasets were settled at friction 0.3 (OmniReset's collection setting), and some states diverge on the
first step under the training contact model (``code_isaaclab.contact_model``; NaN onsets at episode step 1 in v2d).
Every row of every ``resets_<type>.pt`` in ``--in_dir`` is written into an environment of the v2 training task (its
contact model and domain randomisation), the relative action is held at zero for ``--steps`` control steps, and the
row is kept if the robot and cube stay finite and the net contact force on the cube stays at most ``--max_force_n``.
Writes the kept rows to ``--out_dir`` (same file names and keys) and ``filter_report.json`` with the counts per type.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main() -> int:
    from isaaclab.app import add_launcher_args, launch_simulation

    from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli

    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--in_dir", type=Path, required=True)
    p.add_argument("--out_dir", type=Path, required=True)
    p.add_argument("--task", default="CoDe-Rotate-Cube-O-OmniResetV2")
    p.add_argument("--num_envs", type=int, default=4096)
    p.add_argument("--steps", type=int, default=10)
    p.add_argument("--max_force_n", type=float, default=200.0)
    add_launcher_args(p)
    p.set_defaults(device=None, visualizer=[])
    args, hydra = setup_preset_cli(p, None)
    sys.argv = [sys.argv[0]] + hydra
    if args.out_dir.exists() and any(args.out_dir.glob("resets_*.pt")):
        p.error(f"{args.out_dir} already holds datasets")

    import os

    import gymnasium as gym
    import torch

    from isaaclab.sensors import ContactSensorCfg

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.tasks.rotate_cube_omnireset.mdp import DATASET_KEYS, recover_worlds, reset_term

    os.environ["CODE_OMNIRESET_DATASETS"] = str(args.in_dir)
    cfg, _ = resolve_task_config(args.task, "rsl_rl_cfg_entry_point")
    cfg.scene.num_envs = args.num_envs
    cfg.episode_length_s = 1.0e4
    for name in ("abnormal_robot", "non_finite_state", "cube_out_of_reach"):
        setattr(cfg.terminations, name, None)
    cfg.rewards.abnormal_robot = None
    cfg.scene.cube_contacts = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/object")
    args.device = cfg.sim.device
    report = {"steps": args.steps, "max_force_n": args.max_force_n, "task": args.task, "types": {}}
    with launch_simulation(cfg, args), torch.inference_mode():
        env = gym.make(args.task, cfg=cfg)
        u = env.unwrapped
        n, dev = u.num_envs, u.device
        env.reset()
        robot, obj, sensor = u.scene["robot"], u.scene["object"], u.scene.sensors["cube_contacts"]
        term = reset_term(u)
        zero = torch.zeros(n, u.action_manager.total_action_dim, device=dev)
        all_ids = torch.arange(n, device=dev)
        args.out_dir.mkdir(parents=True, exist_ok=True)
        for path in sorted(args.in_dir.glob("resets_*.pt")):
            raw = torch.load(path, map_location="cpu", weights_only=True)
            rows = len(raw["q"])
            keep = torch.zeros(rows, dtype=torch.bool)
            why = {"nonfinite_step1": 0, "nonfinite_later": 0, "force": 0}
            peak_all = torch.zeros(rows)
            for start in range(0, rows, n):
                k = min(n, rows - start)
                ids = all_ids[:k]
                recover_worlds(u, all_ids)  # every world clean before the next batch
                batch = {key: raw[key][start:start + k].to(dev, torch.float32) for key in DATASET_KEYS}
                term.write(ids, batch)
                u.action_manager.reset(ids)
                finite = torch.ones(k, dtype=torch.bool, device=dev)
                first = torch.ones(k, dtype=torch.bool, device=dev)
                peak = torch.zeros(k, device=dev)
                for s in range(args.steps):
                    env.step(zero)
                    ok = torch.isfinite(robot.data.joint_pos.torch[:k]).all(1) & torch.isfinite(
                        robot.data.joint_vel.torch[:k]).all(1) & torch.isfinite(obj.data.root_pose_w.torch[:k]).all(1)
                    if s == 0:
                        first = ok.clone()
                    finite &= ok
                    f = sensor.data.net_forces_w.torch[:k, 0].norm(dim=-1)
                    peak = torch.maximum(peak, torch.nan_to_num(f, nan=float("inf")))
                good = finite & (peak <= args.max_force_n)
                why["nonfinite_step1"] += int((~first).sum())
                why["nonfinite_later"] += int((first & ~finite).sum())
                why["force"] += int((finite & (peak > args.max_force_n)).sum())
                keep[start:start + k] = good.cpu()
                peak_all[start:start + k] = peak.cpu()
            out = {key: (v[keep] if hasattr(v, "shape") and len(v) == rows else v) for key, v in raw.items()}
            torch.save(out, args.out_dir / path.name)
            finite_peak = peak_all[torch.isfinite(peak_all)]
            report["types"][path.stem.removeprefix("resets_")] = {
                "rows": rows, "kept": int(keep.sum()), "dropped": rows - int(keep.sum()), **why,
                "contact_force_n_p50_p99_max": [float(finite_peak.quantile(0.5)), float(finite_peak.quantile(0.99)),
                                                float(finite_peak.max())] if len(finite_peak) else None,
            }  # fmt: skip
            print(path.name, report["types"][path.stem.removeprefix("resets_")], flush=True)
        (args.out_dir / "filter_report.json").write_text(json.dumps(report, indent=1) + "\n")
        env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
