# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Generate the grasp cache of ``CoDe-Rotate-Cube-<hand>`` (Hora's / LEAP Hand's procedure).

    env -u DISPLAY -u WAYLAND_DISPLAY PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run python scripts/rotate_cube/gen_grasps.py \
        --hand O --out_dir logs/rotate_cube/<name> [--target 4096] [--num_envs 2048] [--noise 0.25] [--hold_s 3] \
        [--seed_start 0] [--render] physics=newton_mjwarp

Each round resets every environment to the cradle grasp with U(-noise, noise) rad on every actuated joint (PIP in
flexion, the four-bar closed) and the cube 3 mm above its rest, turned about world z by a random yaw; the relative
action is held at zero for ``--hold_s``. A grasp is kept if the cube touches at least 2 pad-carrying bodies (a finger's
proximal or distal segment, or the thumb's distal; contact force > 0.05 N), lies within 15 mm of its rest, keeps its up
axis (cos >= 0.95) and has settled (speed < 5 mm/s). Kept: every joint's measured position and commanded target, plus
the cube's env-frame position and quaternion. Rounds repeat until ``--target`` grasps; writes ``<out_dir>/<hand>.pt``
(refusing to overwrite) and ``<hand>.json``, and prints the acceptance rate (and, with ``--render``, a PNG of four
accepted grasps). The pads are the task's (``CODE_ROTATE_CUBE_CROWN_MM``, default 1 mm); a held-out evaluation cache
uses a seed range disjoint from the training cache's (s15's: ``heldout_crown1_final4``, ``--target 1024
--seed_start 76000``). Train or evaluate on it with ``CODE_ROTATE_CUBE_GRASPS=<out_dir>``.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import sys
import time
from pathlib import Path

os.environ["CODE_ROTATE_CUBE_GEN_GRASPS"] = "1"

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

FINGERS = ("index", "middle", "ring")
PAD_BODIES = [f"{f}_{s}" for f in FINGERS for s in ("proximal", "distal")] + ["thumb_distal"]


def main(argv=None) -> int:
    from isaaclab.app import add_launcher_args

    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hand", default="O", choices=("O", "R", "U", "R5", "U5", "R9", "U9", "RL", "UL120", "UL"))
    parser.add_argument("--out_dir", type=Path, required=True, help="cache directory (gets <hand>.pt)")
    parser.add_argument("--target", type=int, default=4096)
    parser.add_argument("--num_envs", type=int, default=2048)
    parser.add_argument("--noise", type=float, default=0.25)
    parser.add_argument("--hold_s", type=float, default=3.0)
    parser.add_argument("--max_rounds", type=int, default=20)
    parser.add_argument("--seed_start", type=int, default=0, help="first reset seed in the grasp-cache sequence")
    parser.add_argument("--render", action="store_true")
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args

    import isaaclab.utils.math as math_utils
    from isaaclab.app import launch_simulation
    from isaaclab.sensors import ContactSensorCfg

    from isaaclab_tasks.utils import resolve_task_config

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.assets.description import Description
    from code_isaaclab.tasks.rotate_cube.grasp import CUBE_SIZE_M, GRASP_MCP_DEG, GRASP_PIP_DEG, HANDS

    out = args.out_dir / f"{args.hand}.pt"
    if out.exists():
        parser.error(f"destination already exists: {out}")

    task = f"CoDe-Rotate-Cube-{args.hand}"
    env_cfg, _ = resolve_task_config(task, "rsl_rl_cfg_entry_point")
    assert env_cfg.gen_grasps
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.episode_length_s = 1.0e4
    env_cfg.events.reset_grasp.params["joint_noise"] = args.noise
    bodies = [b["name"] for b in Description.load(HANDS[args.hand]).bodies]
    env_cfg.scene.obj_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/object", filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{b}" for b in bodies])
    args.device = env_cfg.sim.device
    rest = torch.tensor(env_cfg.grasp["rest_env"])

    kept = {"q": [], "pos": [], "quat": [], "target": []}
    tried = 0
    started = time.time()
    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make(task, cfg=env_cfg)
        cleanup.callback(env.close)
        u = env.unwrapped
        n, dev = u.num_envs, u.device
        robot, obj = u.scene["robot"], u.scene["object"]
        sensor = u.scene.sensors["obj_contacts"]
        pad_cols = [bodies.index(b) for b in PAD_BODIES]
        zero = torch.zeros(n, u.action_manager.total_action_dim, device=dev)
        rest_d = rest.to(dev)
        tq = lambda v: v if isinstance(v, torch.Tensor) else v.torch  # noqa: E731
        for rnd in range(args.max_rounds):
            env.reset(seed=args.seed_start + rnd)
            q0 = tq(obj.data.root_quat_w).clone()
            up_body = math_utils.quat_apply_inverse(q0, torch.tensor([0.0, 0.0, 1.0], device=dev).expand(n, 3))
            for _ in range(int(args.hold_s / u.step_dt)):
                env.step(zero)
            force = tq(sensor.data.force_matrix_w)[:, 0].norm(dim=-1)[:, pad_cols]
            touching = (force > 0.05).sum(dim=1)
            pos = tq(obj.data.root_pos_w) - u.scene.env_origins
            quat = tq(obj.data.root_quat_w)
            up = math_utils.quat_apply(quat, up_body)[:, 2]
            speed = tq(obj.data.root_lin_vel_w).norm(dim=1)
            ok = (touching >= 2) & ((pos - rest_d).norm(dim=1) < 0.015) & (up >= 0.95) & (speed < 0.005)
            tried += n
            for key, val in (
                ("q", tq(robot.data.joint_pos)),
                ("pos", pos),
                ("quat", quat),
                ("target", tq(robot.data.joint_pos_target)),
            ):
                kept[key].append(val[ok].cpu())
            total = sum(len(x) for x in kept["q"])
            print(f"round {rnd}: kept {int(ok.sum())}/{n} (pads touching: mean {float(touching.float().mean()):.2f}; "
                  f"dropped {int(((pos - rest_d).norm(dim=1) >= 0.015).sum())}, tilted {int((up < 0.95).sum())}, "
                  f"moving {int((speed >= 0.005).sum())}), total {total}", flush=True)
            if total >= args.target:
                break
        if args.render:
            render(u, ok, args.out_dir / f"{args.hand}_samples.png")
    data = {k: torch.cat(v)[: max(args.target, 0)] for k, v in kept.items()}
    accepted = sum(len(x) for x in kept["q"])
    data.update({"joint_names": list(robot.joint_names), "hand": args.hand, "noise_rad": args.noise,
                 "cube_size_m": CUBE_SIZE_M, "grasp_mcp_deg": GRASP_MCP_DEG, "grasp_pip_deg": GRASP_PIP_DEG,
                 "pad_crown_mm": env_cfg.pad_crown_mm,
                 "pad_crown_layout": env_cfg.pad_crown_layout,
                 "contact_model": os.environ.get("CODE_ROTATE_CUBE_CONTACT", "default"),
                 "servo_colliders": os.environ.get("CODE_ROTATE_CUBE_SERVO_COLLIDERS", "1"),
                 "hold_s": args.hold_s, "seed_start": args.seed_start, "tried": tried, "accepted": accepted,
                 "acceptance": accepted / max(tried, 1)})
    args.out_dir.mkdir(parents=True, exist_ok=True)
    with out.open("xb") as stream:
        torch.save(data, stream)
    (args.out_dir / f"{args.hand}.json").write_text(json.dumps(
        {k: v for k, v in data.items() if not isinstance(v, torch.Tensor)} | {"stored": int(data["q"].shape[0]),
                                                                               "wall_s": time.time() - started},
        indent=1) + "\n")
    print(f"{out}: {data['q'].shape[0]} grasps stored; acceptance {accepted}/{tried} = {accepted / max(tried, 1):.3f}")
    return 0


def render(u, ok, path) -> None:
    """Four accepted grasps of the last round, drawn with Newton's GL viewer offscreen."""
    import pyglet

    if not os.environ.get("DISPLAY"):
        pyglet.options["headless"] = True
    import newton
    import warp as wp
    from PIL import Image

    from code_isaaclab.tasks.rotate_cube.camera import CAMERA_EYE_ENV, CAMERA_LOOKAT_ENV

    model = u.sim.physics_manager.get_model()
    viewer = newton.viewer.ViewerGL(width=960, height=540, headless=True)
    viewer.set_model(model)
    viewer.set_world_offsets((0.0, 0.0, 0.0))
    frames = []
    for k in ok.nonzero().flatten()[:4].tolist():
        viewer.set_visible_worlds([k])
        o = u.scene.env_origins[k].tolist()
        eye = [o[i] + CAMERA_EYE_ENV[i] for i in range(3)]
        d = [CAMERA_LOOKAT_ENV[i] - CAMERA_EYE_ENV[i] for i in range(3)]
        viewer.set_camera(pos=wp.vec3(*eye), pitch=math.degrees(math.atan2(d[2], math.hypot(d[0], d[1]))),
                          yaw=math.degrees(math.atan2(d[1], d[0])))
        for _ in range(2):
            viewer.begin_frame(0.0)
            viewer.log_state(u.sim.physics_manager.get_state_0())
            viewer.end_frame()
        frames.append(viewer.get_frame().numpy())
    viewer.close()
    if frames:
        import numpy as np

        grid = np.concatenate([np.concatenate(frames[i:i + 2], axis=1) for i in range(0, len(frames) - 1, 2)], axis=0)
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(grid).save(path)
        print(path)


if __name__ == "__main__":
    raise SystemExit(main())
