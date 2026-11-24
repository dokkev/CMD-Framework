# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Render a hand's visual USD holding an object (for Fig. 1 of the paper), with Newton's GL viewer, offscreen.

    env -u DISPLAY -u WAYLAND_DISPLAY uv run python scripts/code_hand/render_grasp.py \
        --usd logs/urdf/cfl_q120_h3to7_h0_thumbpad_vis_ball_colliders.usda [--object sphere|box] [--out name]

No simulation: the fingers are set to a grasp pose (four-bars closed from the description, as render_visuals.py),
the bodies posed by forward kinematics, and the object placed by hand among the pads (static, drawing only).
Writes logs/visuals/grasp_<out>.png.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
POSE = {"fe": {"index": 38.0, "middle": 44.0, "ring": 50.0}, "pip": {"index": 70.0, "middle": 78.0, "ring": 86.0},
        "aa": {"index": -6.0, "middle": 0.0, "ring": 6.0},
        "thumb_ro": 75.0, "thumb_raa": 35.0, "thumb_mcp": 25.0, "thumb_ip": 30.0}


def hand_points(model, state, newton, step: float = 0.0008) -> np.ndarray:
    """Points on every visual surface of the posed hand, in the world [m]: mesh triangles sampled about every
    ``step``, boxes and capsules on their surfaces. Used to keep the object clear of the hand."""
    from scipy.spatial.transform import Rotation
    types, scales = model.shape_type.numpy(), model.shape_scale.numpy()
    bodies, tfs, body_q = model.shape_body.numpy(), model.shape_transform.numpy(), state.body_q.numpy()
    out = []
    for k, label in enumerate(model.shape_label):
        if "/visual_" not in label and not label.endswith("_pad"):
            continue
        g = newton.GeoType(int(types[k])).name
        sc = scales[k]
        if g in ("MESH", "CONVEX_MESH"):
            src = model.shape_source[k]
            v, f = np.asarray(src.vertices) * sc, np.asarray(src.indices).reshape(-1, 3)
            pts = [v]
            a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
            size = np.maximum(np.linalg.norm(b - a, axis=1), np.linalg.norm(c - a, axis=1))
            for n in sorted(set(np.clip((size / step).astype(int), 1, 40))):
                sel = (np.clip((size / step).astype(int), 1, 40) == n)
                for i in range(n + 1):
                    for j in range(n + 1 - i):
                        u, w = i / n, j / n
                        pts.append(a[sel] + u * (b[sel] - a[sel]) + w * (c[sel] - a[sel]))
            local = np.concatenate(pts)
        elif g == "BOX":
            h = sc[:3]
            n = np.maximum((2 * h / step).astype(int), 2)
            grids = [np.linspace(-h[i], h[i], n[i]) for i in range(3)]
            faces = []
            for ax in range(3):
                o = [x for x in range(3) if x != ax]
                U, W = np.meshgrid(grids[o[0]], grids[o[1]])
                for sgn in (-1, 1):
                    P = np.zeros((U.size, 3)); P[:, o[0]] = U.ravel(); P[:, o[1]] = W.ravel(); P[:, ax] = sgn * h[ax]
                    faces.append(P)
            local = np.concatenate(faces)
        elif g == "CAPSULE":
            r, hh = sc[0], sc[1]
            th, z = np.meshgrid(np.linspace(0, 2 * np.pi, 40), np.linspace(-hh - r, hh + r, 40))
            rad = np.where(np.abs(z) > hh, np.sqrt(np.maximum(r ** 2 - (np.abs(z) - hh) ** 2, 0)), r)
            local = np.stack([rad.ravel() * np.cos(th.ravel()), rad.ravel() * np.sin(th.ravel()), z.ravel()], 1)
        else:
            continue
        tf = tfs[k]
        p = Rotation.from_quat(tf[3:]).apply(local) + tf[:3]
        if bodies[k] >= 0:
            bq = body_q[bodies[k]]
            p = Rotation.from_quat(bq[3:]).apply(p) + bq[:3]
        out.append(p)
    return np.concatenate(out)


def fit_box(points, centre, quat, margin=0.0006, search=0.012, n=4000, seed=0):
    """The largest cube (half side s) of orientation ``quat`` near ``centre`` with no hand point inside it (plus a
    margin): a random search over centres within ``search`` [m]. Returns (centre, s)."""
    from scipy.spatial.transform import Rotation
    R = Rotation.from_quat(quat)
    rng = np.random.default_rng(seed)
    best = (np.asarray(centre), 0.0)
    near = points[np.linalg.norm(points - centre, axis=1) < 0.06]
    for c in np.concatenate([[centre], centre + rng.uniform(-search, search, size=(n, 3))]):
        local = R.inv().apply(near - c)
        s = np.min(np.max(np.abs(local), axis=1)) - margin
        if s > best[1]:
            best = (c, s)
    return best


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--usd", type=Path, required=True)
    p.add_argument("--object", choices=("sphere", "box", "cylinder"), default="sphere")
    p.add_argument("--size", type=int, nargs=2, default=(1400, 1100))
    p.add_argument("--eye", type=float, nargs=3, default=None)
    p.add_argument("--out", default="hand")
    p.add_argument("--scale", type=float, default=0.95, help="object size relative to the pad cage")
    p.add_argument("--offset", type=float, nargs=3, default=(0.0, 0.0, 0.0), help="object centre offset [m]")
    p.add_argument("--pose", default="{}", help="JSON overrides of POSE")
    p.add_argument("--radius", type=float, default=None, help="object radius [m]; default from the pad cage")
    p.add_argument("--tilt", type=float, nargs=3, default=(0.0, 0.0, 0.0), help="object rotation, xyz Euler [deg]")
    p.add_argument("--cam", type=float, nargs=3, default=(0.20, -0.22, 0.16), help="camera offset from the object [m]")
    args = p.parse_args(argv)
    import json
    for k, v in json.loads(args.pose).items():
        if isinstance(v, dict):
            POSE[k].update(v)
        else:
            POSE[k] = v
    import pyglet
    if not os.environ.get("DISPLAY"):
        pyglet.options["headless"] = True
    import newton
    import warp as wp
    from PIL import Image
    from code_isaaclab.assets import Description

    stem = args.usd.name.split("_thumbpad")[0].split("_ball")[0]
    desc = Description.load(ROOT.parents[1] / "design" / "output" / "sim" / "study3" / "hands" / f"{stem}.json")

    def build(obj=None):
        b = newton.ModelBuilder()
        b.add_usd(str(args.usd))
        if obj is not None:
            kind, centre, r, q = obj
            xf = wp.transform(wp.vec3(*centre), wp.quat(*q))
            if kind == "sphere":
                b.add_shape_sphere(body=-1, xform=xf, radius=r)
            elif kind == "box":
                b.add_shape_box(body=-1, xform=xf, hx=r, hy=r, hz=r)
            else:
                b.add_shape_capsule(body=-1, xform=xf, radius=r, half_height=1.6 * r)
        return b.finalize()

    def pose(model):
        names = [l.rsplit("/", 1)[-1] for l in model.joint_label]
        starts = model.joint_q_start.numpy()
        q = model.joint_q.numpy().copy()
        k = names.index("palm_fixed")  # palm up: half a turn about x
        x_p = model.joint_X_p.numpy()
        x_p[k] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        model.joint_X_p.assign(x_p)
        for digit in desc.four_bar_digits():
            for name, deg in desc.closed_pose(digit, POSE["pip"][digit]).items():
                q[starts[names.index(name)]] = math.radians(deg)
            q[starts[names.index(f"{digit}_fe")]] = math.radians(POSE["fe"][digit])
            q[starts[names.index(f"{digit}_aa")]] = math.radians(POSE["aa"][digit])
        for name in ("thumb_ro", "thumb_raa", "thumb_mcp", "thumb_ip"):
            q[starts[names.index(name)]] = math.radians(POSE[name])
        state = model.state()
        newton.eval_fk(model, wp.array(q, dtype=wp.float32, device=model.device), model.joint_qd, state)
        return state

    model = build()
    state = pose(model)
    body_q = state.body_q.numpy()
    labels = [l.rsplit("/", 1)[-1] for l in model.shape_label]
    pads = []
    for k, lab in enumerate(labels):
        if lab.endswith("_pad") and lab != "visual_pad":
            body = model.shape_body.numpy()[k]
            tf = model.shape_transform.numpy()[k]
            bq = body_q[body]
            t = wp.transform_point(wp.transform(wp.vec3(*bq[:3]), wp.quat(*bq[3:])), wp.vec3(*tf[:3]))
            pads.append((lab, np.array(t)))
    pts = np.array([p for lab, p in pads if "distal" in lab or "proximal" in lab])
    centre = pts.mean(axis=0) + np.asarray(args.offset)
    r = args.radius or args.scale * min(np.linalg.norm(p - centre) for p in pts)
    print("object centre", centre.round(4), "radius", round(r, 4))
    from scipy.spatial.transform import Rotation
    quat = Rotation.from_euler("xyz", args.tilt, degrees=True).as_quat()
    if args.object == "box":  # no penetration: the largest cube clear of every visual surface
        pts_hand = hand_points(model, state, newton)
        centre, r = fit_box(pts_hand, centre, quat)
        if args.radius:
            r = min(r, args.radius)
        print("clear cube: centre", np.round(centre, 4), "half side", round(r, 4), "points", len(pts_hand))
    model = build((args.object, centre, r, tuple(quat)))
    state = pose(model)
    colors = model.shape_color.numpy()
    colors[-1] = (0.86, 0.55, 0.30)
    model.shape_color.assign(colors)
    viewer = newton.viewer.ViewerGL(width=args.size[0], height=args.size[1], headless=True)
    viewer.set_model(model)
    viewer.show_collision = False
    eye = np.array(args.eye) if args.eye else centre + np.asarray(args.cam)
    d = centre - eye
    viewer.camera.fov = 32.0
    viewer.set_camera(pos=wp.vec3(*eye), pitch=math.degrees(math.atan2(d[2], np.linalg.norm(d[:2]))),
                      yaw=math.degrees(math.atan2(d[1], d[0])))
    for _ in range(2):
        viewer.begin_frame(0.0)
        viewer.log_state(state)
        viewer.end_frame()
    out = ROOT / "logs" / "visuals" / f"grasp_{args.out}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(viewer.get_frame().numpy()).save(out)
    viewer.close()
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
