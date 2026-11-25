"""Experiment 34's crowned-pad grasp on hand O (pad_shape.py): one placement re-simulated with crowned pads, three
frames (settled grasp, mid load ramp, at the limit) drawn as render3d's power-grasp frames (index, middle and ring on
the cylinder, palm up, HOLD_ISO_CAMERA), with the task fingers' pad visuals replaced by crowned prisms (the collision
pad boxes' envelopes, the contact face a 2 mm arc along the pad). render3d, load_hand and pad_shape are imported
read-only.

    uv run python render_shape.py [--rho 15 --q0 95 --direction +x]  ->  logs/render_shape{0,1,2}.png, _strip.png
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import load_hand as LH
import pad_shape as PS
import render3d as R
import two_pad_drake as T

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"


def frames(rho, q0, direction):
    pl = next(p for p in json.loads(PS.PLACES.read_text())["places"] if p["rho_mm"] == rho and p["q0_deg"] == q0)
    s, m_, gate, pre, (kt, dt, th) = PS.settle("O", "crowned", rho, pl["crowned"]["c_mm"], q0 + 5.0)
    assert gate["pass"], gate
    spin = s.plant.GetJointByName("spin")
    d = np.array(T.DIRECTIONS[direction])
    snap = lambda load: {"distal": [0.0, 0.0, float(s.measure()["q"])],
                         "centre": (1e3 * s.measure()["c"][[0, 2]]).tolist(), "spin": float(spin.get_angle(s.pctx)),
                         "load_n": (load * d).tolist()}
    out = [snap(0.0)]
    lim, _ = PS.ramp(s, s.measure(), direction)  # runs to the limit on this simulator
    f_lim = lim["f_limit_n"]
    # re-run to stop at half the limit for the middle frame
    s2, *_ = PS.settle("O", "crowned", rho, pl["crowned"]["c_mm"], q0 + 5.0)
    steps = int(0.5 * f_lim / T.FINGERS / (T.RATE_N_S * T.DT_ENV))
    for i in range(steps):
        s2.ctrl.load = T.RATE_N_S * T.DT_ENV * (i + 1) * d
        s2.step()
    sp2 = s2.plant.GetJointByName("spin")
    mid = {"distal": [0.0, 0.0, float(s2.measure()["q"])], "centre": (1e3 * s2.measure()["c"][[0, 2]]).tolist(),
           "spin": float(sp2.get_angle(s2.pctx)), "load_n": (0.5 * f_lim * d).tolist()}
    out.append(mid)
    out.append(snap(f_lim))
    return out, lim


def crowned_tris(Xb_box, size_m):
    """Triangles (mm, body frame) of the crowned prism filling the pad box: thickness along its smallest axis, the
    contact face (the one away from the link) an arc along its longest axis."""
    Rm, c = Xb_box.rotation().matrix(), 1e3 * Xb_box.translation()
    size = 1e3 * np.asarray(size_m)
    k_t = int(np.argmin(size))
    others = [k for k in range(3) if k != k_t]
    k_l = max(others, key=lambda k: size[k])
    k_w = [k for k in others if k != k_l][0]
    e_t = Rm[:, k_t]
    if (c + e_t * size[k_t] / 2) @ e_t < (c - e_t * size[k_t] / 2) @ (-e_t):
        e_t = -e_t
    # outward: the face whose centre is further from the body origin along its normal
    f_plus, f_minus = c + e_t * size[k_t] / 2, c - e_t * size[k_t] / 2
    if np.linalg.norm(f_minus) > np.linalg.norm(f_plus):
        e_t = -e_t
    e_l, e_w = Rm[:, k_l], Rm[:, k_w]
    L, W, H = size[k_l], size[k_w], size[k_t]
    crown = PS.CROWN_MM
    Rr = (L / 2) ** 2 / (2 * crown) + crown / 2
    us = np.linspace(-L / 2, L / 2, PS.SEGMENTS + 1)
    top = [c + u * e_l + (H / 2 - (Rr - math.sqrt(Rr ** 2 - u ** 2))) * e_t for u in us]
    bot = [c + u * e_l - (H / 2) * e_t for u in us]
    tris = []
    for side in (-1.0, 1.0):
        pass
    w2 = W / 2 * e_w
    ring = top + bot[::-1]  # the profile loop
    n = len(ring)
    for i in range(n):  # side walls across the width
        a, b = ring[i], ring[(i + 1) % n]
        tris += [np.array([a - w2, b - w2, b + w2]), np.array([a - w2, b + w2, a + w2])]
    cen = sum(ring) / n
    for sgn in (-1.0, 1.0):  # the two profile caps (fan from the centroid)
        for i in range(n):
            a, b = ring[i], ring[(i + 1) % n]
            tris.append(np.array([cen + sgn * w2, a + sgn * w2, b + sgn * w2]))
    centre = cen
    out = []
    for t in tris:
        nrm = np.cross(t[1] - t[0], t[2] - t[0])
        out.append(t if nrm @ (t.mean(0) - centre) >= 0 else t[[0, 2, 1]])
    return out


def render_frames(frs, out_dir=LOGS, rho=15.0):
    from PIL import Image
    from pydrake.geometry import (ClippingRange, ColorRenderCamera, GeometryInstance, Mesh, PerceptionProperties,
                                  RenderCameraCore, RenderLabel, Rgba, Role)
    from pydrake.math import RigidTransform, RotationMatrix
    from pydrake.systems.sensors import CameraInfo

    paths = []
    snapd = {"hold": {"rho_mm": rho}, "rho_mm": rho}
    for i, st in enumerate(frs):
        ghost = frs[1]["centre"] if i == 2 else None
        sc = R.build("hold", st, snapd, LOGS / "render3d" / f"shape{i}", ghost=ghost, finger=False,
                     span=R.task_span())
        urdf, description, desc, origin, _ = R.hand_frame("O", R.TASK_REF)
        diagram, plant, sg, model, desc, _ = LH.load_hand(urdf, description, renderer=True, gravity=False,
                                                         pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        ctx = diagram.CreateDefaultContext()
        pctx, sctx = plant.GetMyMutableContextFromRoot(ctx), sg.GetMyMutableContextFromRoot(ctx)
        LH.set_pose(plant, pctx, model, desc, 0.0, 0.0, R.THUMB_NEUTRAL)
        q = st["distal"][2]
        for digit in R.TASK_FINGERS:
            for name, value in LH.table(desc, digit, math.degrees(q)).items():
                plant.GetJointByName(name, model).set_angle(pctx, value)
        # swap the task fingers' pad visuals for crowned prisms on the same bodies
        insp = sg.model_inspector()
        src = plant.get_source_id()
        boxes = {}
        for gid in insp.GetAllGeometryIds(Role.kProximity):
            body = plant.GetBodyFromFrameId(insp.GetFrameId(gid))
            shp = insp.GetShape(gid)
            if body is not None and type(shp).__name__ == "Box" and any(body.name() == f"{d}_{p}" for d in
                                                                         R.TASK_FINGERS for p in ("proximal", "distal")):
                boxes[body.name()] = (insp.GetPoseInFrame(gid), (shp.width(), shp.depth(), shp.height()),
                                      insp.GetFrameId(gid))
        for gid in list(insp.GetAllGeometryIds(Role.kPerception)):
            nm = insp.GetName(gid)
            if any(nm.endswith(f"{d}_{p}_pad") for d in R.TASK_FINGERS for p in ("proximal", "distal")):
                sg.RemoveGeometry(sctx, src, gid)
        for bname, (X, size, fid) in boxes.items():
            pad_sc = R.Scene(LOGS / "render3d" / f"shape{i}_{bname}")
            pad_sc.mesh(crowned_tris(X, size), R.PAD)
            path, rgba = pad_sc.items[0]
            gi = GeometryInstance(RigidTransform(), Mesh(str(path), 1.0), f"crown_{bname}")
            pp = PerceptionProperties()
            pp.AddProperty("phong", "diffuse", Rgba(*rgba))
            pp.AddProperty("label", "id", RenderLabel(5000 + len(paths) * 10 + list(boxes).index(bname)))
            gi.set_perception_properties(pp)
            sg.RegisterGeometry(sctx, src, fid, gi)
        Xo = RigidTransform(1e-3 * origin)
        for k, (path, rgba) in enumerate(sc.items):
            gi = GeometryInstance(Xo, Mesh(str(path), 1.0), f"task_{k}")
            pp = PerceptionProperties()
            pp.AddProperty("phong", "diffuse", Rgba(*rgba))
            pp.AddProperty("label", "id", RenderLabel(1000 + k))
            gi.set_perception_properties(pp)
            sg.RegisterGeometry(sctx, src, sg.world_frame_id(), gi)
        c = R.HOLD_ISO_CAMERA
        target = 1e-3 * (origin + np.asarray(c["target_mm"], float))
        az, el = math.radians(c["az_deg"]), math.radians(c["el_deg"])
        eye = target + c["dist"] * np.array([math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el), math.sin(el)])
        fwd = (target - eye) / np.linalg.norm(target - eye)
        right = np.cross(fwd, c.get("up", [0.0, 0.0, 1.0])); right /= np.linalg.norm(right)
        down = np.cross(fwd, right)
        core = RenderCameraCore("vtk", CameraInfo(R.W, R.H, math.radians(c["fov_deg"])), ClippingRange(0.01, 3.0),
                                RigidTransform())
        img = sg.get_query_output_port().Eval(sctx).RenderColorImage(
            ColorRenderCamera(core, False), sg.world_frame_id(),
            RigidTransform(RotationMatrix(np.column_stack([right, down, fwd])), eye))
        out = out_dir / f"render_shape{i}.png"
        Image.fromarray(np.array(img.data)[:, :, :3]).save(out)
        paths.append(out)
        print(out)
    R.strip(paths, out_dir / "render_shape_strip.png")
    return paths


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--rho", type=float, default=15.0)
    p.add_argument("--q0", type=float, default=95.0)
    p.add_argument("--direction", default="+x")
    a = p.parse_args()
    frs, lim = frames(a.rho, a.q0, a.direction)
    (LOGS / "shape_render_frames.json").write_text(json.dumps({"rho": a.rho, "q0": a.q0, "direction": a.direction,
                                                               "limit": lim, "frames": frs}, indent=1))
    print(f"limit {lim}")
    render_frames(frs, rho=a.rho)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
