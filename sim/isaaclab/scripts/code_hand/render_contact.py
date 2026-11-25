# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Render one finger of a hand's visual USD from the side, closed on a cylinder that touches both its pads
(Fig. 1 of the paper), with Newton's GL viewer, offscreen.

    env -u DISPLAY -u WAYLAND_DISPLAY uv run python scripts/code_hand/render_contact.py \
        --usd logs/urdf/cfl_q120_h3to7_hr_thumbpad_vis_ball_colliders.usda --pip 90 --out R

No simulation: the ring finger is posed (four-bar closed from the description, as render_visuals.py), the other
digits curled away; the cylinder's axis is the finger's joint axis and its centre is where it is largest while
clear of the finger's visual surfaces (so it touches the pads). Writes logs/visuals/contact_<out>.png.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--usd", type=Path, required=True)
    p.add_argument("--pip", type=float, default=90.0)
    p.add_argument("--fe", type=float, default=0.0)
    p.add_argument("--radius", type=float, default=None, help="cylinder radius [m]; default: the largest clear")
    p.add_argument("--size", type=int, nargs=2, default=(1400, 1100))
    p.add_argument("--out", default="hand")
    p.add_argument("--dial", action="store_true", help="draw the object as a dial on an axle, contacts marked")
    args = p.parse_args(argv)
    import pyglet
    if not os.environ.get("DISPLAY"):
        pyglet.options["headless"] = True
    import newton
    import warp as wp
    from PIL import Image
    from scipy.spatial.transform import Rotation
    from code_isaaclab.assets import Description
    import render_grasp as G
    import render_visuals as V

    stem = args.usd.name.split("_thumbpad")[0].split("_ball")[0]
    desc = Description.load(ROOT.parents[1] / "design" / "output" / "sim" / "study3" / "hands" / f"{stem}.json")
    digit = "ring"

    extras = []  # (kind, xform args, colour)

    def build(cyl=None):
        b = newton.ModelBuilder()
        b.add_usd(str(args.usd))
        if cyl is not None:
            c, q, r, hh = cyl
            b.add_shape_cylinder(body=-1, xform=wp.transform(wp.vec3(*c), wp.quat(*q)), radius=r, half_height=hh)
            for kind, a1, a2_ in extras:
                if kind == "axle":
                    b.add_shape_cylinder(body=-1, xform=wp.transform(wp.vec3(*a1), wp.quat(*q)), radius=a2_[0],
                                         half_height=a2_[1])
                elif kind == "box":
                    b.add_shape_box(body=-1, xform=wp.transform(wp.vec3(*a1), wp.quat(*q)), hx=a2_[0], hy=a2_[1],
                                    hz=a2_[2])
                else:
                    b.add_shape_sphere(body=-1, xform=wp.transform(wp.vec3(*a1), wp.quat(0, 0, 0, 1)), radius=a2_)
        return b.finalize()

    def pose(model):
        names = [l.rsplit("/", 1)[-1] for l in model.joint_label]
        starts = model.joint_q_start.numpy()
        q = model.joint_q.numpy().copy()
        for name, deg in desc.closed_pose(digit, args.pip).items():
            q[starts[names.index(name)]] = math.radians(deg)
        q[starts[names.index(f"{digit}_fe")]] = math.radians(args.fe)
        for other in ("index", "middle"):  # out of the way of the side view
            q[starts[names.index(f"{other}_fe")]] = math.radians(-20.0)
        state = model.state()
        newton.eval_fk(model, wp.array(q, dtype=wp.float32, device=model.device), model.joint_qd, state)
        return state

    model = build()
    state = pose(model)
    # the ring finger's surfaces, in its plane
    keep = [i for i, lab in enumerate(model.shape_label) if f"{digit}_" in lab]
    pts = G.hand_points(model, state, newton)
    labels = model.shape_label
    mask_model = type("M", (), {})()
    o = np.asarray(desc.joint(f"{digit}_fe")["anchor"], float)
    ax_y = np.asarray(desc.joint(f"{digit}_fe")["axis"], float)
    ax_y /= np.linalg.norm(ax_y)
    xd = np.asarray(desc.joint(f"{digit}_a")["anchor"], float) - o
    xd -= (xd @ ax_y) * ax_y
    xd /= np.linalg.norm(xd)
    zd = np.cross(xd, ax_y)
    # keep points within the finger's slab
    rel = pts - o
    slab = np.abs(rel @ ax_y) < 0.010
    p2 = np.c_[rel[slab] @ xd, rel[slab] @ zd]
    # a cylinder of the given radius touching both pads: of the centres just clear of the finger by r, the one
    # nearest the PIP joint (so it sits in the crook between the phalanges)
    r = args.radius or 0.015
    a_ = np.asarray(desc.joint(f"{digit}_a")["anchor"], float) - o
    a2 = np.array([a_ @ xd, a_ @ zd])
    from scipy.spatial import cKDTree

    def slab2(points):
        rr = points - o
        keep_ = np.abs(rr @ ax_y) < 0.010
        return np.c_[rr[keep_] @ xd, rr[keep_] @ zd]
    pp2 = slab2(G.hand_points(model, state, newton, only=f"{digit}_proximal_pad"))
    dp2 = slab2(G.hand_points(model, state, newton, only=f"{digit}_distal_pad"))
    tree, tp, td = cKDTree(p2), cKDTree(pp2), cKDTree(dp2)
    gx, gz = np.meshgrid(np.linspace(-0.02, 0.12, 561), np.linspace(-0.10, 0.03, 521))
    cand = np.c_[gx.ravel(), gz.ravel()]
    da_, _ = tree.query(cand)
    dpp, _ = tp.query(cand)
    ddp, _ = td.query(cand)
    err = np.maximum(np.abs(dpp - r), np.abs(ddp - r))
    err[da_ < r - 0.0003] = np.inf  # nothing of the finger inside the object
    k = int(np.argmin(err))
    best = (err[k], cand[k, 0], cand[k, 1])
    print("touch error", round(err[k] * 1000, 2), "mm")
    contact_pts = []
    for tr in (tp, td):
        _, ii = tr.query(cand[k])
        contact_pts.append(tr.data[ii])
    _, cx, cz = best
    r = r - 0.0
    centre = o + cx * xd + cz * zd
    rotm = np.c_[xd, np.cross(ax_y, xd), ax_y]  # cylinder axis (local z) along the joint axis
    quat = Rotation.from_matrix(rotm).as_quat()
    print("cylinder centre", np.round(centre, 4), "radius", round(r, 4))
    colours = [(0.92, 0.80, 0.62)]
    if args.dial:
        hh = 0.006
        extras.append(("axle", centre, (0.0025, 0.016)))
        colours.append((0.45, 0.45, 0.48))
        for sgn in (-1, 1):
            spoke = centre + (r * 0.5) * zd + sgn * (hh + 0.0004) * ax_y
            extras.append(("box", spoke, (0.0011, r * 0.45, 0.0005)))
            colours.append((0.35, 0.30, 0.25))
        for c2 in contact_pts:  # the two touch points, marked on both end faces
            for sgn in (-1, 1):
                extras.append(("sphere", o + c2[0] * xd + c2[1] * zd + sgn * (hh + 0.0012) * ax_y, 0.0019))
                colours.append((0.89, 0.18, 0.16))
            print("contact at", np.round(c2, 4))
    else:
        hh = 0.014
    model = build((centre, tuple(quat), r, hh))
    state = pose(model)
    cols = model.shape_color.numpy()
    for k_, c_ in enumerate(colours):
        cols[len(cols) - len(colours) + k_] = c_
    model.shape_color.assign(cols)
    viewer = newton.viewer.ViewerGL(width=args.size[0], height=args.size[1], headless=True)
    viewer.set_model(model)
    viewer.show_collision = False
    eye, target = V.side_camera(desc)
    d = target - eye
    viewer.camera.fov = V.SIDE_CAMERA["fov_deg"]
    viewer.set_camera(pos=wp.vec3(*eye), pitch=math.degrees(math.atan2(d[2], np.linalg.norm(d[:2]))),
                      yaw=math.degrees(math.atan2(d[1], d[0])))
    for _ in range(2):
        viewer.begin_frame(0.0)
        viewer.log_state(state)
        viewer.end_frame()
    out = ROOT / "logs" / "visuals" / f"contact_{args.out}.png"
    Image.fromarray(viewer.get_frame().numpy()).save(out)
    viewer.close()
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
