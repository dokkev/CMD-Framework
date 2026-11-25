"""Experiment 33b's three-finger pinch on hand O's URDF (pinch_rotate.py frames --draws logs/pinch3_e33b.json ->
logs/pinch_frames_O_d<draw>.json): the thumb at its pose (pinch3_pose.POSE), index and middle at the draw's MCP
flexion and at the frame's PIP flexion, ring curled, the two-tone sphere (render_pinch.sphere_tris) in the
index-middle mid-plane at the simulated pose, and a red arc arrow once it turns. Three frames: the sweep's start,
middle and the end of the hold.

    uv run python render_pinch3.py --draw 60  ->  logs/render_pinch3_{0,1,2}.png, logs/render_pinch3_strip.png
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import load_hand as LH
import pinch3_pose as PP
import render3d as R
import render_pinch as RP
import two_pad_drake as T

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
CAMERA = {"dir": (0.75, 1.0, -0.25), "dist": 0.40, "fov_deg": 30.0, "size": (1200, 1000)}
"""From the index finger's outer side (+y), in front of the fingertips and a little below the palm: the thumb pad, the
sphere and the index pad in profile, the middle finger behind."""


def scene(fr, pl, folder, start, y_mid):
    """The sphere and the arrow in the planar frame (mm; x along the proximal phalanx, y lateral, z dorsal), the sphere
    at the mid-plane y_mid."""
    sc = R.Scene(folder)
    sx, sz, rot = fr["sphere_mm_rad"]
    ta, tb = RP.sphere_tris((sx, sz), pl["r_mm"], -rot)
    off = np.array([0.0, y_mid, 0.0])
    for tris, rgba in ((ta, RP.SPHERE_A), (tb, RP.SPHERE_B)):
        fixed = []
        for t in tris:
            t = t + off
            nrm = np.cross(t[1] - t[0], t[2] - t[0])
            fixed.append(t if nrm @ (sum(t) / 3 - np.array([sx, y_mid, sz])) > 0 else t[[0, 2, 1]])
        sc.mesh(fixed, rgba)
    turned = rot - start["sphere_mm_rad"][2]
    if abs(turned) > math.radians(0.5):
        sgn = math.copysign(1.0, turned)
        n = np.array(pl["n"])  # from the sphere toward the thumb
        a_mid = math.atan2(-n[0], n[1])  # beside the sphere, between the pads' contacts (in (x, z) angles)
        R.arc_arrow(sc, np.array([sx, sz]), pl["r_mm"] + 3.5, a_mid - sgn * math.radians(40),
                    a_mid + sgn * math.radians(40), 0.0, R.LOAD)
        # the arrow drawn at y 0 is moved to the mid-plane's near side below
    return sc


def render(frames, pl, draw, out_dir=LOGS):
    from PIL import Image
    from pydrake.geometry import (ClippingRange, ColorRenderCamera, GeometryInstance, Mesh, PerceptionProperties,
                                  RenderCameraCore, RenderLabel, Rgba)
    from pydrake.math import RigidTransform, RotationMatrix
    from pydrake.systems.sensors import CameraInfo

    urdf, description = LH.paths("O")
    paths = []
    sweep = [f for f in frames if f["phase"] == "sweep"]
    picks = [sweep[0], sweep[len(sweep) // 2 - 4], sweep[-1]]
    for i, fr in enumerate(picks):
        diagram, plant, sg, model, desc, _ = LH.load_hand(urdf, description, renderer=True, gravity=False,
                                                         pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        ctx = diagram.CreateDefaultContext()
        pctx, sctx = plant.GetMyMutableContextFromRoot(ctx), sg.GetMyMutableContextFromRoot(ctx)
        J = lambda n: plant.GetJointByName(n, model)
        LH.set_pose(plant, pctx, model, desc, 0.0)
        for k, v in LH.table(desc, "ring", 100).items():
            J(k).set_angle(pctx, v)
        J("ring_fe").set_angle(pctx, math.radians(95))
        for k, v in PP.POSE.items():
            J(k).set_angle(pctx, math.radians(v))
        for d in ("index", "middle"):
            for k, v in LH.table(desc, d, fr["q_deg"]).items():
                J(k).set_angle(pctx, v)
            J(f"{d}_fe").set_angle(pctx, math.radians(pl["fe_deg"]))
        # the planar frame (index FE anchor, index_proximal's axes) in the world
        fe_anchor = np.asarray(next(j for j in desc["joints"] if j["name"] == "index_fe")["anchor"], float)
        org = np.asarray(next(b for b in desc["bodies"] if b["name"] == "index_proximal")["origin"], float)
        Xp = plant.EvalBodyPoseInWorld(pctx, plant.GetBodyByName("index_proximal", model))
        Xplane = RigidTransform(Xp.rotation(), Xp.multiply(fe_anchor - org))
        y_mid = -12.5
        sc = scene(fr, pl, LOGS / "render3d" / f"pinch3_{i}", picks[0], y_mid)
        # the arrow (drawn at y 0..) goes to the near side of the sphere: shift the arrow meshes in y
        face_o = 6.0  # the frames are in world (link-line) z already; sphere poses come from the plant
        for k, (path, rgba) in enumerate(sc.items):
            shift = RigidTransform([0.0, 1e-3 * (y_mid + pl["r_mm"] + 2.5), 0.0]) if rgba == R.LOAD else RigidTransform()
            gi = GeometryInstance(Xplane.multiply(shift), Mesh(str(path), 1.0), f"s{k}")
            pp = PerceptionProperties()
            pp.AddProperty("phong", "diffuse", Rgba(*rgba))
            pp.AddProperty("label", "id", RenderLabel(3000 + k))
            gi.set_perception_properties(pp)
            sg.RegisterGeometry(sctx, plant.get_source_id(), sg.world_frame_id(), gi)
        c0 = picks[0]["sphere_mm_rad"]
        target = Xplane.multiply(1e-3 * np.array([c0[0], y_mid, c0[1]]))
        dvec = np.asarray(CAMERA["dir"], float)
        eye = target + CAMERA["dist"] * dvec / np.linalg.norm(dvec)
        fwd = (target - eye) / np.linalg.norm(target - eye)
        right = np.cross(fwd, [0.0, 0.0, 1.0]); right /= np.linalg.norm(right)
        down = np.cross(fwd, right)
        core = RenderCameraCore("vtk", CameraInfo(*CAMERA["size"], math.radians(CAMERA["fov_deg"])),
                                ClippingRange(0.01, 3.0), RigidTransform())
        img = sg.get_query_output_port().Eval(sctx).RenderColorImage(
            ColorRenderCamera(core, False), sg.world_frame_id(),
            RigidTransform(RotationMatrix(np.column_stack([right, down, fwd])), eye))
        out = out_dir / f"render_pinch3_{i}.png"
        Image.fromarray(np.array(img.data)[:, :, :3]).save(out)
        paths.append(out)
        print(f"{out}: q {fr['q_deg']:.1f} deg, sphere turned {math.degrees(fr['sphere_mm_rad'][2] - c0[2]):+.2f} deg")
    R.strip(paths, out_dir / "render_pinch3_strip.png")
    return paths


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--draw", type=int, default=60)
    p.add_argument("--dir", type=float, nargs=3, default=None)
    a = p.parse_args()
    if a.dir:
        CAMERA["dir"] = tuple(a.dir)
    d = json.loads((LOGS / f"pinch_frames_O_d{a.draw}.json").read_text())
    render(d["frames"], d["placement"], a.draw)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
