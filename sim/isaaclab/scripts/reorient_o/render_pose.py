# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Render hand O (or R, U) with its visual meshes in Newton GL at a few poses, for comparison with Drake's VTK render
(``sim/drake/load_hand.py --render``): palm up, MCP 0, abduction and thumb at zero, every PIP at each flexion.

    env -u DISPLAY -u WAYLAND_DISPLAY PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run python scripts/reorient_o/render_pose.py \
        [--task CoDe-Reorient-Cube-O] [--pip 0 60 120] physics=newton_mjwarp

Builds the task with ``CODE_REORIENT_O_VISUALS=1`` (drawing-only meshes), 4 environments, gravity and the drop
termination off, the cube moved out of view; drives every joint to the pose (PIP in flexion, no smoothing) for 1 s, then
draws environment 0 with Newton's GL viewer offscreen from the video camera (``tasks/reorient/camera.py``), 960 x 540.
Writes ``logs/reorient_o/render/newton_<hand>_pip<q>.png``.
"""

from __future__ import annotations

import argparse
import contextlib
import math
import os
import sys
from pathlib import Path

os.environ["CODE_REORIENT_O_VISUALS"] = "1"

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

OUT = Path(__file__).resolve().parents[2] / "logs" / "reorient_o" / "render"


def main(argv=None) -> int:
    from isaaclab.app import add_launcher_args

    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--task", default="CoDe-Reorient-Cube-O")
    parser.add_argument("--pip", type=float, nargs="+", default=[0.0, 60.0, 120.0])
    parser.add_argument("--fov_deg", type=float, default=45.0)
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args

    import pyglet

    if not os.environ.get("DISPLAY"):
        pyglet.options["headless"] = True
    from isaaclab.app import launch_simulation

    from isaaclab_tasks.utils import resolve_task_config

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.tasks.reorient.camera import env0_camera

    env_cfg, _ = resolve_task_config(args.task, "rsl_rl_cfg_entry_point")
    assert env_cfg.visuals, "the visual asset was not selected"
    env_cfg.scene.num_envs = 4
    env_cfg.sim.gravity = (0.0, 0.0, 0.0)
    env_cfg.terminations.object_out_of_reach = None
    env_cfg.episode_length_s = 1.0e4
    env_cfg.actions.joint_pos.alpha = 1.0
    env_cfg.events.reset_object = None
    env_cfg.scene.object.init_state.pos = (0.0, 0.0, -1.0)  # out of view
    args.device = env_cfg.sim.device
    hand = args.task.rsplit("-", 1)[-1]

    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make(args.task, cfg=env_cfg)
        cleanup.callback(env.close)
        env.reset(seed=0)
        u = env.unwrapped
        n, dev = u.num_envs, u.device
        term = u.action_manager.get_term("joint_pos")
        names = list(term._joint_names)
        lo, hi = term._cmd_limits[0, :, 0], term._cmd_limits[0, :, 1]
        robot = u.scene["robot"]

        import newton
        import warp as wp
        from PIL import Image

        manager = u.sim.physics_manager
        model = manager.get_model()
        viewer = newton.viewer.ViewerGL(width=960, height=540, headless=True)
        viewer.set_model(model)
        viewer.set_world_offsets((0.0, 0.0, 0.0))
        viewer.set_visible_worlds([0])
        viewer.show_collision = False
        eye, lookat = env0_camera(n)
        eye_v, look_v = torch.tensor(eye), torch.tensor(lookat)
        d = look_v - eye_v
        yaw = math.degrees(math.atan2(float(d[1]), float(d[0])))
        pitch = math.degrees(math.atan2(float(d[2]), float(torch.linalg.norm(d[:2]))))
        if hasattr(viewer, "camera") and hasattr(viewer.camera, "fov"):
            viewer.camera.fov = args.fov_deg
        OUT.mkdir(parents=True, exist_ok=True)
        for q in args.pip:
            cmd = torch.zeros(n, len(names), device=dev)
            for k, nm in enumerate(names):
                cmd[:, k] = math.radians(q) if nm.endswith("_pip_servo") else 0.0
            act = 2.0 * (cmd.clamp(lo, hi) - lo) / (hi - lo) - 1.0
            for _ in range(int(1.0 / u.step_dt)):
                env.step(act)
            viewer.set_camera(pos=wp.vec3(*eye), pitch=pitch, yaw=yaw)
            for _ in range(2):
                viewer.begin_frame(0.0)
                viewer.log_state(manager.get_state_0())
                viewer.end_frame()
            path = OUT / f"newton_{hand}_pip{q:g}.png"
            Image.fromarray(viewer.get_frame().numpy()).save(path)
            theta = robot.data.joint_pos.torch[0, term._joint_ids]
            flex = [round(math.degrees(float(term._to_flexion(theta[None])[0, k])), 1) for k, nm in enumerate(names)
                    if nm.endswith("_pip_servo")]
            print(path, "PIP flexion reached [deg]:", flex)
        viewer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
