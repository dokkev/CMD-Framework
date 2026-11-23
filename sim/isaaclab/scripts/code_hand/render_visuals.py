# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Render a hand's visual USD (``make_assets.py ... --visuals``) with Newton's GL viewer, offscreen, at PIP flexions.

    env -u DISPLAY -u WAYLAND_DISPLAY uv run python scripts/code_hand/render_visuals.py \
        --usd logs/urdf/cfl_q120_h3to7_h0_thumbpad_vis_ball_colliders.usda --pip 0 60 120 \
        [--view side|side_driver|overview]

No simulation: the USD is imported with ``ModelBuilder.add_usd`` (the importer Isaac Lab's Newton backend uses), the
four-bars are set closed from the description's ``four_bar`` tables (the joint coordinates), and the bodies are posed
by forward kinematics. ``side`` is ``sim/drake/load_hand.py``'s side camera (``render3d``'s, on the ring finger, the
hand in its own frame: dorsal up), ``side_driver`` mirrored at the index finger from
the driver's side; ``overview`` a three-quarter view of the whole hand. Also prints the colours the
importer resolved for the visual meshes. Writes ``logs/visuals/newton_<stem>_pip<q>[_side].png``.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SIDE_CAMERA = {"target_mm": (40.0, -12.0), "dist": 0.24, "az_deg": -20.0, "el_deg": 14.0, "fov_deg": 30.0}
"""As ``sim/drake/load_hand.py`` (``render_hands.py``'s camera for ``hands_ORU_3d.png``)."""


def side_camera(desc, digit: str = "ring", side: float = -1.0):
    """The camera on ``digit`` from its ``-y`` side (``side=+1``: from ``+y``, mirrored): eye and target."""
    o = np.asarray(desc.joint(f"{digit}_fe")["anchor"], float)
    x = np.asarray(desc.joint(f"{digit}_a")["anchor"], float) - o
    x /= np.linalg.norm(x)
    y = -side * np.asarray(desc.joint(f"{digit}_fe")["axis"], float)
    z = np.cross(x, -side * y)
    c = SIDE_CAMERA
    target = o + 1e-3 * (c["target_mm"][0] * x + c["target_mm"][1] * z)
    az, el = math.radians(c["az_deg"]), math.radians(c["el_deg"])
    eye = target + c["dist"] * (math.sin(az) * math.cos(el) * x - math.cos(az) * math.cos(el) * y + math.sin(el) * z)
    return eye, target


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--description", type=Path, default=None,
                        help="defaults to design/output/sim/study3/hands/<hand>.json from the USD's name")
    parser.add_argument("--pip", type=float, nargs="+", default=[0.0, 60.0, 120.0])
    parser.add_argument("--view", choices=("side", "side_driver", "overview"), default="side")
    parser.add_argument("--size", type=int, nargs=2, default=(960, 800))
    args = parser.parse_args(argv)

    import pyglet

    if not os.environ.get("DISPLAY"):
        pyglet.options["headless"] = True
    import newton
    import warp as wp
    from PIL import Image

    from code_isaaclab.assets import Description

    stem = args.usd.name.split("_thumbpad")[0].split("_ball")[0]
    hands = ROOT.parents[1] / "design" / "output" / "sim" / "study3" / "hands"
    description = args.description or hands / f"{stem}.json"
    desc = Description.load(description)

    builder = newton.ModelBuilder()
    builder.add_usd(str(args.usd))
    model = builder.finalize()
    state = model.state()
    names = [label.rsplit("/", 1)[-1] for label in model.joint_label]
    starts = model.joint_q_start.numpy()

    colours = {}
    for k, label in enumerate(model.shape_label):
        if "/visual_" in label:
            colours[label.rsplit("/", 1)[-1]] = tuple(round(float(v), 3) for v in model.shape_color.numpy()[k])
    print("visual colours (display):", dict(sorted(colours.items())))

    viewer = newton.viewer.ViewerGL(width=args.size[0], height=args.size[1], headless=True)
    viewer.set_model(model)
    viewer.show_collision = False
    if args.view in ("side", "side_driver"):
        eye, target = side_camera(desc) if args.view == "side" else side_camera(desc, "index", 1.0)
        viewer.camera.fov = SIDE_CAMERA["fov_deg"]
    else:
        eye, target = np.array([0.17, -0.21, 0.12]), np.array([0.04, -0.02, 0.0])
        viewer.camera.fov = 45.0
    d = target - eye
    yaw = math.degrees(math.atan2(d[1], d[0]))
    pitch = math.degrees(math.atan2(d[2], np.linalg.norm(d[:2])))
    out_dir = ROOT / "logs" / "visuals"
    out_dir.mkdir(parents=True, exist_ok=True)
    for q in args.pip:
        joint_q = model.joint_q.numpy().copy()
        for digit in desc.four_bar_digits():
            for name, deg in desc.closed_pose(digit, q).items():
                joint_q[starts[names.index(name)]] = math.radians(deg)
        q_wp = wp.array(joint_q, dtype=wp.float32, device=model.device)
        newton.eval_fk(model, q_wp, model.joint_qd, state)
        viewer.set_camera(pos=wp.vec3(*eye), pitch=pitch, yaw=yaw)
        for _ in range(2):
            viewer.begin_frame(0.0)
            viewer.log_state(state)
            viewer.end_frame()
        path = out_dir / f"newton_{stem}_pip{q:g}{'' if args.view == 'overview' else '_' + args.view}.png"
        Image.fromarray(viewer.get_frame().numpy()).save(path)
        print(path)
    viewer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
