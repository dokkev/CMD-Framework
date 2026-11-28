# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Render the original hand's visual USD in a learned-policy task scene (Fig. 5 of the paper), with Newton's GL viewer,
offscreen: the finger walk (a cylinder lying across the proximal pads, a distal pad on its top) or the finger relay
(the dial over the index and middle fingertips). The objects sit where the task's environment puts them
(``tasks/finger_walk/env_cfg.py``, ``tasks/finger_relay/env_cfg.py``: the proximal pad-face frame); the fingers are set
to a pose from the task by hand. No simulation.

    env -u DISPLAY -u WAYLAND_DISPLAY uv run python scripts/code_hand/render_task.py --task walk|relay [--hand O]

Writes logs/visuals/task_<task>_<hand>.png.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
USD = {"O": "cfl_q120_h3to7_h0", "R": "cfl_q120_h3to7_hr", "U": "cfl_q120_h3to7_hu"}
PIP = {"walk": {"index": 62.0, "middle": 38.0, "ring": 12.0}, "relay": {"index": 30.0, "middle": 16.0, "ring": 0.0}}
CAM = {"walk": (0.10, -0.12, 0.12), "relay": (0.14, 0.13, 0.012)}
"""Camera offset from the object, in the proximal pad-face frame (x toward the tips, z out of the pads) [m]."""
OBJECT_RGB = (0.86, 0.55, 0.30)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--task", choices=("walk", "relay"), required=True)
    p.add_argument("--hand", default="O", choices=tuple(USD))
    p.add_argument("--pip", default="{}", help="JSON overrides of the PIP flexions [deg]")
    p.add_argument("--cam", type=float, nargs=3, default=None, help="camera offset from the object, face frame [m]")
    p.add_argument("--size", type=int, nargs=2, default=(1400, 1100))
    args = p.parse_args(argv)
    pip = {**PIP[args.task], **json.loads(args.pip)}
    import pyglet
    if not os.environ.get("DISPLAY"):
        pyglet.options["headless"] = True
    import newton
    import warp as wp
    from PIL import Image
    from scipy.spatial.transform import Rotation

    from code_isaaclab.assets import Description
    from code_isaaclab.hands.code_variants import HAND_ROOT_POS
    from code_isaaclab.tasks.finger_relay import env_cfg as relay
    from code_isaaclab.tasks.finger_walk import env_cfg as walk
    from code_isaaclab.tasks.rotate_cube.grasp import HANDS, face_frame

    desc = Description.load(HANDS[args.hand])
    o, r = face_frame(desc, 0.0)
    o = np.asarray(o) - np.asarray(HAND_ROOT_POS)  # the render puts the hand's root at the origin
    r = np.asarray(r).reshape(3, 3)

    def at(x):  # face frame -> render frame
        return o + r @ np.asarray(x, float)

    def quat(m):
        return tuple(Rotation.from_matrix(m).as_quat())

    shapes = []  # (kind, centre, quat, dims)
    if args.task == "walk":
        x = sum(walk.START_X_M) / 2
        # the cylinder's axis across the fingers (face y); Newton's cylinder axis is its local z
        m = r @ Rotation.from_euler("x", 90, degrees=True).as_matrix()
        shapes.append(("cylinder", at([x, 0.0, walk.RADIUS_M + 0.0015]), quat(m), (walk.RADIUS_M, walk.LENGTH_M / 2)))
    else:
        place = relay.dial_place_face()
        shapes.append(("cylinder", at(place + [0, 0, relay.THICK_M / 2]), quat(r), (relay.RADIUS_M, relay.THICK_M / 2)))
        shapes.append(("cylinder", at(place + [0, 0, relay.THICK_M + 0.012]), quat(r), (0.003, 0.012)))  # axle
        shapes.append(("box", at(place + [relay.RADIUS_M * 0.72, 0, relay.THICK_M + 0.001]), quat(r),
                       (0.010, 0.0025, 0.001)))  # a mark on the dial's top

    def build(with_objects):
        b = newton.ModelBuilder()
        b.add_usd(str(ROOT / "logs" / "urdf" / f"{USD[args.hand]}_thumbpad_vis_ball_colliders.usda"))
        if with_objects:
            for kind, c, q, d in shapes:
                xf = wp.transform(wp.vec3(*c), wp.quat(*q))
                if kind == "cylinder":
                    b.add_shape_cylinder(body=-1, xform=xf, radius=d[0], half_height=d[1])
                else:
                    b.add_shape_box(body=-1, xform=xf, hx=d[0], hy=d[1], hz=d[2])
        return b.finalize()

    model = build(True)
    names = [l.rsplit("/", 1)[-1] for l in model.joint_label]
    starts = model.joint_q_start.numpy()
    q = model.joint_q.numpy().copy()
    x_p = model.joint_X_p.numpy()
    x_p[names.index("palm_fixed")] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]  # palm up: half a turn about x
    model.joint_X_p.assign(x_p)
    for digit in desc.four_bar_digits():
        for name, deg in desc.closed_pose(digit, pip.get(digit, 0.0)).items():
            q[starts[names.index(name)]] = math.radians(deg)
    for name, deg in relay.THUMB_DEG.items():
        q[starts[names.index(name)]] = math.radians(deg)
    state = model.state()
    newton.eval_fk(model, wp.array(q, dtype=wp.float32, device=model.device), model.joint_qd, state)
    colors = model.shape_color.numpy()
    colors[-len(shapes):] = OBJECT_RGB
    if args.task == "relay":
        colors[-2] = (0.55, 0.55, 0.58)
        colors[-1] = (0.20, 0.20, 0.22)
    model.shape_color.assign(colors)

    viewer = newton.viewer.ViewerGL(width=args.size[0], height=args.size[1], headless=True)
    viewer.set_model(model)
    viewer.show_collision = False
    centre = shapes[0][1]
    eye = centre + r @ np.asarray(args.cam or CAM[args.task])
    d = centre - eye
    viewer.camera.fov = 32.0
    viewer.set_camera(pos=wp.vec3(*eye), pitch=math.degrees(math.atan2(d[2], np.linalg.norm(d[:2]))),
                      yaw=math.degrees(math.atan2(d[1], d[0])))
    for _ in range(2):
        viewer.begin_frame(0.0)
        viewer.log_state(state)
        viewer.end_frame()
    out = ROOT / "logs" / "visuals" / f"task_{args.task}_{args.hand}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(viewer.get_frame().numpy()).save(out)
    viewer.close()
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
