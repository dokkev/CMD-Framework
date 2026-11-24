"""3D renders of the three Study 3 tasks on hand O at the states Drake simulated (logs/task_snapshots.json, written by
render_tasks.py), with Drake's VTK renderer.

The hand is the USD's geometry: the plain URDF with the description's drawing-only visuals (XM335 servos as the
design's D-profile, links laid out by the packaging layers, the palm housing; sim/isaaclab make_assets.py --visuals),
loaded by load_hand.py. The planar frame (x along the finger from the MCP-FE axis, z dorsal) is the middle finger's;
index, middle and ring are closed at the frame's flexion through the description's four_bar table (the simulation
divides the load over three fingers, as Isaac Lab's hand does), MCP FE and AA at 0, the thumb at its zeros. The
cylinder, or the dial on its axle, spans the three fingers and is posed from the simulation; a red arrow gives the
load. Its geometry is written as OBJ files in logs/render3d/ and anchored in the hand frame.

add_finger() still draws one finger from its parameter file alone (pivots, pads, the X330 mesh, bowed link plates),
for render_hands.py.

    uv run python render3d.py   ->  logs/render_<task><frame>.png
"""
import json
import math
import struct
from pathlib import Path

import numpy as np
import yaml

import two_pad_drake as T

HERE = Path(__file__).resolve().parent
W, H = 1200, 1000
LIGHT, MID, DARK = [0.84, 0.85, 0.87, 1.0], [0.66, 0.68, 0.71, 1.0], [0.22, 0.23, 0.25, 1.0]
STEEL = [0.90, 0.91, 0.93, 1.0]
PAD = [0.27, 0.50, 0.93, 1.0]
LOAD = [0.82, 0.18, 0.14, 1.0]
OBJECT = [0.90, 0.68, 0.42, 1.0]
KNOB = [0.30, 0.33, 0.38, 1.0]
WIDTH = 19.0  # mm


def rot(q, p):
    c, s = math.cos(q), math.sin(q)
    return np.array([c * p[0] + s * p[1], -s * p[0] + c * p[1]])


def hull(points):
    pts = sorted(map(tuple, points))

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(p)
        return out
    lo, hi = half(pts), half(pts[::-1])
    return np.array(lo[:-1] + hi[:-1])


def circles_hull(circles, n=48):
    pts = [np.asarray(c, float) + r * np.array([math.cos(t), math.sin(t)]) for c, r in circles
           for t in np.linspace(0, 2 * math.pi, n, endpoint=False)]
    return hull(np.array(pts))


def rounded_rect(x0, x1, z0, z1, r, n=10):
    r = min(r, (x1 - x0) / 2, (z1 - z0) / 2)
    return circles_hull([(np.array([x, z]), r) for x in (x0 + r, x1 - r) for z in (z0 + r, z1 - r)], n * 4)


def clip_above(poly, z_min):
    """The part of a convex polygon (x, z) with z >= z_min."""
    out = []
    n = len(poly)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        if p[1] >= z_min:
            out.append(p)
        if (p[1] >= z_min) != (q[1] >= z_min):
            t = (z_min - p[1]) / (q[1] - p[1])
            out.append(p + t * (q - p))
    return np.array(out)


def ear_clip(poly):
    idx = list(range(len(poly)))
    out = []

    def area2(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    guard = 0
    while len(idx) > 3 and guard < 20000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if area2(a, b, c) <= 1e-12:
                continue
            if any(area2(a, b, poly[m]) > 0 and area2(b, c, poly[m]) > 0 and area2(c, a, poly[m]) > 0
                   for m in idx if m not in (i0, i1, i2)):
                continue
            out.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            break
    if len(idx) == 3:
        out.append(tuple(idx))
    return out


def fix_winding(verts, normals, faces):
    out = []
    for a, b, c in faces:
        nrm = np.cross(verts[b] - verts[a], verts[c] - verts[a])
        out.append((a, c, b) if np.dot(nrm, normals[a] + normals[b] + normals[c]) < 0 else (a, b, c))
    return out


class Scene:
    def __init__(self, folder: Path):
        self.folder = folder
        folder.mkdir(parents=True, exist_ok=True)
        for old in folder.glob("*.obj"):
            old.unlink()
        self.items = []  # (obj path, rgba)

    def _write(self, verts, normals, faces, rgba):
        path = self.folder / f"g{len(self.items):03d}.obj"
        with open(path, "w") as f:
            for v in verts:
                f.write(f"v {v[0] * 1e-3:.6f} {v[1] * 1e-3:.6f} {v[2] * 1e-3:.6f}\n")
            for n in normals:
                f.write(f"vn {n[0]:.5f} {n[1]:.5f} {n[2]:.5f}\n")
            for a, b, c in faces:
                f.write(f"f {a + 1}//{a + 1} {b + 1}//{b + 1} {c + 1}//{c + 1}\n")
        self.items.append((path, rgba))

    def extrude(self, poly, y0, y1, rgba, smooth_deg=35.0, axis="y"):
        """A 2D profile in (x, z dorsal) mm extruded across y0..y1 (mm); ``axis="z"``: a profile in (x, y) extruded
        along z0..z1."""
        poly = np.asarray(poly, float)
        if np.sum(poly[:, 0] * np.roll(poly[:, 1], -1) - np.roll(poly[:, 0], -1) * poly[:, 1]) < 0:
            poly = poly[::-1]
        n = len(poly)
        verts, normals, faces = [], [], []
        edge_n = []
        for i in range(n):
            d = poly[(i + 1) % n] - poly[i]
            edge_n.append(np.array([d[1], 0.0, -d[0]]) / (np.linalg.norm(d) + 1e-12))  # outward for CCW
        cos_s = math.cos(math.radians(smooth_deg))
        for i in range(n):
            j = (i + 1) % n
            ni = edge_n[i]
            na = edge_n[i - 1] if np.dot(edge_n[i - 1], ni) > cos_s else ni
            nb = edge_n[j] if np.dot(edge_n[j], ni) > cos_s else ni
            va, vb = (na + ni) / np.linalg.norm(na + ni), (nb + ni) / np.linalg.norm(nb + ni)
            base = len(verts)
            for p, nn in ((poly[i], va), (poly[j], vb)):
                for y in (y0, y1):
                    verts.append([p[0], y, p[1]])
                    normals.append(nn)
            faces += [(base, base + 2, base + 3), (base, base + 3, base + 1)]
        for y, sgn in ((y0, -1.0), (y1, 1.0)):
            base = len(verts)
            for p in poly:
                verts.append([p[0], y, p[1]])
                normals.append([0.0, sgn, 0.0])
            faces += [(base + a, base + b, base + c) for a, b, c in ear_clip(poly)]
        verts, normals = np.array(verts, float), np.array(normals, float)
        if axis == "z":
            verts, normals = verts[:, [0, 2, 1]], normals[:, [0, 2, 1]]
        self._write(verts, normals, fix_winding(verts, normals, faces), rgba)

    def cylinder(self, centre, y0, y1, r, rgba, n=72, axis="y"):
        poly = [np.asarray(centre, float) + r * np.array([math.cos(t), math.sin(t)])
                for t in np.linspace(0, 2 * math.pi, n, endpoint=False)]
        self.extrude(poly, y0, y1, rgba, smooth_deg=20.0, axis=axis)

    def bar(self, p, q, half_width, y0, y1, rgba, axis="y"):
        self.extrude(circles_hull([(p, half_width), (q, half_width)], 24), y0, y1, rgba, axis=axis)

    def arc_bar(self, p, q, half_width, sag, y0, y1, rgba, n=32):
        """A link from p to q bowed by sag (mm) to its left (+90 deg from p->q in (x, z)), with round ends; the pins
        stay where the kinematics put them, the bow only clears the object."""
        p, q = np.asarray(p, float), np.asarray(q, float)
        d = q - p
        left = np.array([-d[1], d[0]]) / np.linalg.norm(d)
        ctrl = (p + q) / 2 + 2.0 * sag * left
        ts = np.linspace(0.0, 1.0, n)
        pts = np.array([(1 - t) ** 2 * p + 2 * (1 - t) * t * ctrl + t ** 2 * q for t in ts])
        tan = np.array([2 * (1 - t) * (ctrl - p) + 2 * t * (q - ctrl) for t in ts])
        tan /= np.linalg.norm(tan, axis=1)[:, None]
        nor = np.stack([-tan[:, 1], tan[:, 0]], axis=1)
        a = pts + half_width * nor
        b = pts - half_width * nor
        def cap(c, t0, sign):
            ang0 = math.atan2(t0[1], t0[0])
            return [c + half_width * np.array([math.cos(ang0 + sign * u), math.sin(ang0 + sign * u)])
                    for u in np.linspace(math.pi / 2, -math.pi / 2, 12)[1:-1]] if sign > 0 else \
                   [c + half_width * np.array([math.cos(ang0 + u), math.sin(ang0 + u)])
                    for u in np.linspace(math.pi / 2, 3 * math.pi / 2, 12)[1:-1]]
        poly = list(a) + cap(pts[-1], tan[-1], 1) + list(b[::-1]) + cap(pts[0], tan[0], -1)
        self.extrude(np.array(poly), y0, y1, rgba)

    def mesh(self, tris, rgba):
        verts, normals, faces = [], [], []
        for t in tris:
            nrm = np.cross(t[1] - t[0], t[2] - t[0])
            nrm = nrm / (np.linalg.norm(nrm) + 1e-12)
            base = len(verts)
            for v in t:
                verts.append(v)
                normals.append(nrm)
            faces.append((base, base + 1, base + 2))
        self._write(np.array(verts), np.array(normals), faces, rgba)


def load_stl(path: Path) -> np.ndarray:
    d = path.read_bytes()
    n = struct.unpack("<I", d[80:84])[0]
    rec = np.frombuffer(d[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].astype(float)


def servo_tris(centre_xz, tilt_deg, servo):
    """The X330 at a servo centre in the linkage plane. Its part frame is the centre frame moved 5.5 mm along +Z (the
    output axis, the finger's lateral +y); the body runs along the frame's -Y, tilted dorsally by tilt from -x."""
    t = math.radians(tilt_deg)
    ey = np.array([math.cos(t), 0.0, -math.sin(t)])
    ez = np.array([0.0, 1.0, 0.0])
    ex = np.cross(ey, ez)
    R = np.column_stack([ex, ey, ez])
    o = np.array([centre_xz[0], 0.0, centre_xz[1]]) + 5.5 * ez
    return np.einsum("ij,tkj->tki", R, servo) + o


GHOST = [0.50, 0.52, 0.56, 1.0]
GONE_MM = 40.0
"""A cylinder farther than this from its ghost has left the grasp: it is not drawn, and its path arrow stops here (a
pulled-out cylinder flies metres away, out of view, and would stretch the shadow map)."""


def dashed_ring(sc, centre, r, y, rgba, n=30):
    for k in range(n):
        t0, t1 = 2 * math.pi * k / n, 2 * math.pi * (k + 0.55) / n
        sc.bar(centre + r * np.array([math.cos(t0), math.sin(t0)]),
               centre + r * np.array([math.cos(t1), math.sin(t1)]), 0.45, y - 0.5, y + 0.5, rgba)


def arrow(sc, tail, head, y, rgba, half_width=0.8, head_len=4.5, head_half=2.6):
    tail, head = np.asarray(tail, float), np.asarray(head, float)
    d = (head - tail) / np.linalg.norm(head - tail)
    nrm = np.array([-d[1], d[0]])
    base = head - head_len * d
    sc.bar(tail, base, half_width, y - 0.8, y + 0.8, rgba)
    sc.extrude([base + head_half * nrm, base - head_half * nrm, head], y - 0.8, y + 0.8, rgba, smooth_deg=0.0)


def arc_arrow(sc, centre, r, a0, a1, y, rgba, n=24):
    """An arc of radius r from angle a0 to a1 (rad, in (x, z)) with a head at a1."""
    ts = np.linspace(a0, a1, n)
    pts = [centre + r * np.array([math.cos(t), math.sin(t)]) for t in ts]
    for p, q in zip(pts[:-4], pts[1:-3]):
        sc.bar(p, q, 0.8, y - 0.8, y + 0.8, rgba)
    arrow(sc, pts[-5], pts[-1], y, rgba)


def add_finger(sc: "Scene", hand: str, B, q: float) -> None:
    """One finger of `hand` (its own pivots, pads and PIP servo tilt) with the distal at pose (B, q), in mm."""
    g = T.finger_geometry(hand)
    pads = g["pads"]
    tilt = yaml.safe_load(T.hand_file(hand).read_text())["finger"]["pip_servo_tilt_deg"]
    a, ap, bpl = g["a"], g["ap"], g["bp_local"]
    B = np.asarray(B, float)
    Bp = B + rot(q, bpl)
    m, th = pads["mount_offset_mm"], pads["thickness_mm"]
    hw = WIDTH / 2
    plate = 2.0
    servo = load_stl(HERE / "assets" / "x330.stl")

    # proximal phalanx: a palmar plate behind the pad, side plates hulled around O, A, A' and the pad ends
    sc.extrude(rounded_rect(-6.0, pads["proximal_end_mm"] + 2.0, -m, 1.0, 1.0), -hw, hw, LIGHT)
    side = circles_hull([((0.0, 0.0), 8.0), (a, 5.0), (ap, 6.5),
                         ((pads["proximal_end_mm"] + 1.0, -m + 1.0), 1.0), ((-6.0, -m + 1.0), 1.0)])
    side = clip_above(side, -m)  # the plates stop at the pad mount, so the pad shows below them
    for y in (-hw - plate, hw):
        sc.extrude(side, y, y + plate, LIGHT)
    sc.extrude(rounded_rect(pads["proximal_start_mm"], pads["proximal_end_mm"], -m - th, -m, 0.8), -hw, hw, PAD)
    sc.mesh(servo_tris(ap, tilt, servo), DARK)
    sc.mesh(servo_tris((0.0, 0.0), 0.0, servo), DARK)  # MCP-FE servo, behind the joint

    # distal phalanx, in its frame (origin B, +x toward the tip), placed by the simulated pose
    def to_world(poly):
        return np.array([B + rot(q, p) for p in poly])
    ds, de = pads["distal_start_mm"], pads["distal_end_mm"]
    sc.extrude(to_world(rounded_rect(ds - 1.0, de + 1.5, -m, 1.0, 1.0)), -hw, hw, LIGHT)
    dside = circles_hull([((0.0, 0.0), 5.5), (bpl, 5.0), ((de + 0.5, -m + 1.5), 1.5),
                          ((de - 3.0, 2.0), 2.0), ((ds, -m + 1.0), 1.0)])
    dside = clip_above(dside, -m)
    for y in (-hw - plate, hw):
        sc.extrude(to_world(dside), y, y + plate, LIGHT)
    sc.extrude(to_world(rounded_rect(ds, de, -m - th, -m, 0.8)), -hw, hw, PAD)

    # crossed four-bar: the driver A'-B outside, the coupler A-B' inside, each a pair of plates; steel pins
    for s in (-1, 1):
        yc = s * (hw + plate + 0.6)
        sc.arc_bar(a, Bp, 2.8, 5.0, min(yc, yc + s * 1.8), max(yc, yc + s * 1.8), MID)
        yd = s * (hw + plate + 3.0)
        sc.arc_bar(ap, B, 3.2, 8.0, min(yd, yd + s * 2.0), max(yd, yd + s * 2.0), MID)
    for p, r in ((a, 1.6), (ap, 2.2), (B, 1.6), (Bp, 1.6)):
        sc.cylinder(p, -hw - plate - 5.5, hw + plate + 5.5, r, STEEL)
    sc.cylinder((0.0, 0.0), -hw - plate - 1.0, hw + plate + 1.0, 4.0, STEEL)



def build(task: str, st: dict, snap: dict, folder: Path, ghost=None, turn_sign=1.0, finger: bool = True,
          span=(-14.0, 14.0)) -> Scene:
    """The task frame's scene in the finger's linkage plane (mm): the object (the cylinder or the dial on its axle),
    the load arrow and the ghost outline, and with ``finger`` the finger drawn by ``add_finger``. ``span``: the
    object's extent across the finger (y, mm; the camera looks from -y)."""
    sc = Scene(folder)
    if finger:
        add_finger(sc, "O", st["distal"][:2], st["distal"][2])

    # the object
    c = np.array(st["centre"])
    rho = snap[task].get("rho_mm", snap["rho_mm"])
    y_lo, y_hi = span
    if task == "dial":
        sc.cylinder(c, y_lo, y_hi, rho, KNOB, n=96)
        for k in range(36):  # knurl
            t = 2 * math.pi * k / 36 + st["spin"]
            sc.cylinder(c + (rho - 0.2) * np.array([math.cos(t), math.sin(t)]), y_lo + 2, y_hi - 2, 0.9, KNOB, n=12)
        axis = np.array(snap["dial"]["axis"])
        sc.cylinder(axis, y_lo - 3.0, y_hi + 8.0, 2.5, STEEL)
        sc.extrude(rounded_rect(axis[0] - 7, axis[0] + 7, axis[1] - 30.0, axis[1] + 4.0, 3.0), y_hi + 6.0,
                   y_hi + 11.0, MID)
        if abs(st["spin"]) > 0.03:  # the pointer's start, faint
            sc.bar(c + 0.25 * rot(0.0, np.array([0.0, -(rho - 3.0)])), c + np.array([0.0, -(rho - 3.0)]), 1.0,
                   y_lo - 0.4, y_lo + 1.0, [0.62, 0.64, 0.68, 1.0])
        mark = c + rot(-st["spin"], np.array([0.0, -(rho - 3.0)]))
        sc.bar(c + 0.25 * (mark - c), mark, 1.3, y_lo - 0.6, y_lo + 1.0, [0.95, 0.80, 0.25, 1.0])
        for k in range(-6, 7):  # a fixed scale every 10 deg around the pointer's start (-z)
            t = -math.pi / 2 + math.radians(10 * k)
            u = np.array([math.cos(t), math.sin(t)])
            sc.bar(axis + (rho + 1.8) * u, axis + (rho + (5.0 if k % 3 == 0 else 3.6)) * u, 0.45, y_lo - 0.5,
                   y_lo + 0.5, GHOST)
        if abs(st["spin"]) > 0.01:  # the turn: an arc arrow in the dial's direction
            arc_arrow(sc, axis, rho + 8.0, -math.pi / 2, -math.pi / 2 + turn_sign * math.radians(55), y_lo - 1.0,
                      LOAD)
    else:
        gone = False
        if ghost is not None:  # where the cylinder was: a dashed outline and its path
            g0 = np.asarray(ghost, float)
            dashed_ring(sc, g0, rho, y_lo - 0.6, GHOST)
            moved = np.linalg.norm(c - g0)
            gone = moved > GONE_MM  # out of the hand and out of view: only its path's start, clipped
            if moved > 3.0:
                end = g0 + (c - g0) * min(1.0, GONE_MM / moved)
                arrow(sc, g0, end, y_lo - 1.2, GHOST, half_width=0.55, head_len=2.8, head_half=1.6)
        if gone:
            return sc
        sc.cylinder(c, y_lo, y_hi, rho, OBJECT, n=96)
        mark = c + rot(-st["spin"], np.array([0.0, -(rho - 2.0)]))
        sc.bar(c, mark, 0.9, y_lo - 0.6, y_lo + 1.0, [0.45, 0.30, 0.16, 1.0])
        load = np.array(st["load_n"])
        if np.linalg.norm(load) > 0:
            dirv = load / np.linalg.norm(load)
            y = y_lo - 6.0
            tail, head = c + dirv * 4.0, c + dirv * (rho + 14.0)
            sc.bar(tail, head, 1.0, y - 1.0, y + 1.0, LOAD)
            nrm = np.array([-dirv[1], dirv[0]])
            sc.extrude([head + 3.2 * nrm, head - 3.2 * nrm, head + dirv * 6.0], y - 1.0, y + 1.0, LOAD,
                       smooth_deg=0.0)
    return sc


def render(sc: Scene, out: Path, target_xz, dist=0.21, az_deg=-22.0, el_deg=14.0, fov_deg=24.0):
    from pydrake.geometry import (ClippingRange, ColorRenderCamera, GeometryInstance, LightParameter,
                                  MakeRenderEngineVtk, Mesh, PerceptionProperties, RenderCameraCore,
                                  RenderEngineVtkParams, RenderLabel, Rgba, SceneGraph)
    from pydrake.math import RigidTransform, RotationMatrix
    from pydrake.systems.sensors import CameraInfo
    from PIL import Image

    sg = SceneGraph()
    params = RenderEngineVtkParams()
    params.default_clear_color = np.array([1.0, 1.0, 1.0])
    params.lights = [
        LightParameter(type="directional", direction=[0.35, 0.55, -0.75], intensity=0.80, frame="world"),
        LightParameter(type="directional", direction=[-0.6, 0.3, -0.2], intensity=0.30, frame="world"),
        LightParameter(type="directional", direction=[0.1, 0.2, 0.9], intensity=0.20, frame="world"),
        LightParameter(type="directional", direction=[0.0, 0.0, 1.0], intensity=0.30, frame="camera"),
    ]
    params.cast_shadows = True
    params.shadow_map_size = 4096
    sg.AddRenderer("vtk", MakeRenderEngineVtk(params))
    src = sg.RegisterSource("scene")
    for k, (path, rgba) in enumerate(sc.items):
        gi = GeometryInstance(RigidTransform(), Mesh(str(path), 1.0), f"g{k}")
        pp = PerceptionProperties()
        pp.AddProperty("phong", "diffuse", Rgba(*rgba))
        pp.AddProperty("label", "id", RenderLabel(k + 1))
        gi.set_perception_properties(pp)
        sg.RegisterAnchoredGeometry(src, gi)

    target = np.array([target_xz[0] * 1e-3, 0.0, target_xz[1] * 1e-3])
    az, el = math.radians(az_deg), math.radians(el_deg)
    eye = target + dist * np.array([math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el), math.sin(el)])
    fwd = (target - eye) / np.linalg.norm(target - eye)
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    R = RotationMatrix(np.column_stack([right, down, fwd]))
    core = RenderCameraCore("vtk", CameraInfo(W, H, math.radians(fov_deg)), ClippingRange(0.01, 2.0),
                            RigidTransform())
    ctx = sg.CreateDefaultContext()
    img = sg.get_query_output_port().Eval(ctx).RenderColorImage(ColorRenderCamera(core, False), sg.world_frame_id(),
                                                                 RigidTransform(R, eye))
    Image.fromarray(np.array(img.data)[:, :, :3]).save(out)
    print(out)


TASK_FINGERS = ("index", "middle", "ring")
"""The fingers that carry the task: the planar simulation divides the load over three (two_pad_drake.FINGERS), as
Isaac Lab's hand does with index, middle and ring on one cylinder."""
TASK_REF = "middle"
"""The finger whose linkage plane the planar frame maps to (the middle one, so the object spans the three evenly)."""
TASK_CAMERA = {"target_mm": (28.0, -10.0, -10.0), "dist": 0.30, "az_deg": -34.0, "el_deg": -10.0, "fov_deg": 33.0}
"""Task frames on the hand: the point looked at in the reference finger's planar frame (x along, y lateral, z dorsal,
mm), the camera's distance [m], azimuth from -y toward +x and elevation [deg] and field of view (render()'s
convention): from the ring finger's side and a little below, so the three fingers' pads and the object show."""
THUMB_NEUTRAL = {"thumb_ro": 0.0, "thumb_raa": 0.0, "thumb_mcp": 0.0, "thumb_ip": 0.0}
"""The thumb out of the way at its joints' zeros [deg]: it lies beside the index finger, its pad toward +y."""


def hand_frame(hand: str = "O", ref: str = TASK_REF):
    """The hand's URDF and description, and the planar frame's origin in the hand frame (mm): the reference finger's
    MCP-FE axis, x along the proximal at MCP 0, z dorsal (the hand frame's axes), with each task finger's lateral
    offset from it (mm)."""
    import load_hand as LH

    urdf, description = LH.paths(hand)
    desc = json.loads(description.read_text())
    joints = {j["name"]: j for j in desc["joints"]}
    origin = 1e3 * np.asarray(joints[f"{ref}_fe"]["anchor"], float)
    offsets = {f: 1e3 * joints[f"{f}_fe"]["anchor"][1] - origin[1] for f in TASK_FINGERS}
    return urdf, description, desc, origin, offsets


def task_span(hand: str = "O", margin_mm: float = 4.5) -> tuple[float, float]:
    """The object's extent across the task fingers (y in the reference finger's frame, mm): their pads and a margin."""
    _, _, desc, _, offsets = hand_frame(hand)
    hw = desc["pads"]["width_mm"] / 2 + margin_mm
    return min(offsets.values()) - hw, max(offsets.values()) + hw


def render_on_hand(sc: Scene, out: Path, q: float, hand: str = "O", camera: dict | None = None,
                   size=(W, H), ref: str = TASK_REF, fingers=TASK_FINGERS, curled=()) -> Path:
    """The scene (in the reference finger's planar frame) with the hand's URDF geometry (the USD's visuals, through
    load_hand.py), palm down in the hand frame: MCP FE and AA at 0, the task fingers' four-bars closed at flexion q
    [rad] from the description's ``four_bar`` table (``fingers``), the ``curled`` ones at ``CURL`` (MCP FE, PIP deg), the
    others at 0, the thumb at ``THUMB_NEUTRAL``. ``camera["up"]``, if given, is the image's up direction in the hand
    frame (for a view along z)."""
    from PIL import Image
    from pydrake.geometry import (ClippingRange, ColorRenderCamera, GeometryInstance, Mesh, PerceptionProperties,
                                  RenderCameraCore, RenderLabel, Rgba)
    from pydrake.math import RigidTransform, RotationMatrix
    from pydrake.systems.sensors import CameraInfo

    import load_hand as LH

    urdf, description, desc, origin, _ = hand_frame(hand, ref)
    diagram, plant, sg, model, desc, _ = LH.load_hand(urdf, description, renderer=True, gravity=False,
                                                     pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    ctx = diagram.CreateDefaultContext()
    pctx, sctx = plant.GetMyMutableContextFromRoot(ctx), sg.GetMyMutableContextFromRoot(ctx)
    LH.set_pose(plant, pctx, model, desc, 0.0, 0.0, THUMB_NEUTRAL)
    for digit in curled:
        for name, value in LH.table(desc, digit, CURL[1]).items():
            plant.GetJointByName(name, model).set_angle(pctx, value)
        plant.GetJointByName(f"{digit}_fe", model).set_angle(pctx, math.radians(CURL[0]))
    for digit in fingers:
        for name, value in LH.table(desc, digit, math.degrees(q)).items():
            plant.GetJointByName(name, model).set_angle(pctx, value)
    X = RigidTransform(1e-3 * origin)
    for k, (path, rgba) in enumerate(sc.items):
        gi = GeometryInstance(X, Mesh(str(path), 1.0), f"task_{k}")
        pp = PerceptionProperties()
        pp.AddProperty("phong", "diffuse", Rgba(*rgba))
        pp.AddProperty("label", "id", RenderLabel(1000 + k))
        gi.set_perception_properties(pp)
        sg.RegisterGeometry(sctx, plant.get_source_id(), sg.world_frame_id(), gi)

    c = {**TASK_CAMERA, **(camera or {})}
    target = 1e-3 * (origin + np.asarray(c["target_mm"], float))
    az, el = math.radians(c["az_deg"]), math.radians(c["el_deg"])
    eye = target + c["dist"] * np.array([math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el), math.sin(el)])
    fwd = (target - eye) / np.linalg.norm(target - eye)
    right = np.cross(fwd, c.get("up", [0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    core = RenderCameraCore("vtk", CameraInfo(size[0], size[1], math.radians(c["fov_deg"])), ClippingRange(0.01, 3.0),
                            RigidTransform())
    img = sg.get_query_output_port().Eval(sctx).RenderColorImage(
        ColorRenderCamera(core, False), sg.world_frame_id(), RigidTransform(RotationMatrix(np.column_stack([right, down, fwd])), eye))
    Image.fromarray(np.array(img.data)[:, :, :3]).save(out)
    print(out)
    return out


TIPDIAL_REF = "index"
"""Experiment 28z's disk sits under the index finger, its axle on the outer side (+y, away from the middle finger).
The simulation is one finger on one disk (radius 15 mm, 12 mm to the side), so one disk is drawn; under the middle
or ring finger it would reach under a neighbour's pad, and one disk per finger would overlap."""
TIPDIAL_CAMERA = {"target_mm": (52.0, -16.0, -4.0), "dist": 0.27, "az_deg": 110.0, "el_deg": 50.0, "fov_deg": 33.0}
"""From above the finger (dorsal), 40 deg off the disk's axis, in front of the fingertip and a little to the index
finger's outer side: the curled middle and ring fingers face the camera, the fingertip rests on the disk's top face,
and the pointer and scale on that face show. (Azimuth 110 rather than 100 deg uncovers a little more of the disk beside
the distal phalanx early in the sweep.)"""
TIPDIAL_FINGERS, TIPDIAL_CURLED = ("index",), ("middle", "ring")
"""The index finger turns the disk at the frame's flexion; middle and ring are curled away (``CURL``)."""
CURL = (90.0, 120.0)
"""MCP FE and PIP flexion [deg] of a curled finger, near the limits (FE 100, PIP 120 in the description)."""
DISK = [0.93, 0.78, 0.45, 1.0]
CONTACT = [0.86, 0.10, 0.10, 1.0]


def build_tipdial(st: dict, tip: dict, folder: Path, contact_marker: bool = True) -> Scene:
    """Experiment 28z's frame in the index finger's planar frame (mm): the disk (axis z, at the simulated centre and
    spin) with a pointer and a scale every 10 deg on its top face (the pointer's start faint), a cap on the axle, the
    index pad's contact outlined on the top face (``contact_marker``), the axle and its hub below, and an arc arrow
    above the top face in the direction it turns once it has turned."""
    sc = Scene(folder)
    rho, thick = tip["rho_mm"], tip["thick_mm"]
    c = np.asarray(st["centre"], float)
    cxy, z_top, z_bot = c[:2], c[2] + thick / 2, c[2] - thick / 2
    sc.cylinder(cxy, z_bot, z_top, rho, DISK, n=120, axis="z")
    for k in range(48):  # knurl on the rim, turning with the disk
        t = 2 * math.pi * k / 48 + st["spin"]
        sc.cylinder(cxy + (rho - 0.2) * np.array([math.cos(t), math.sin(t)]), z_bot + 0.4, z_top - 0.4, 0.7, DISK,
                    n=10, axis="z")
    phi0 = math.pi / 2  # the pointer starts outward (+y), away from the finger
    u = lambda a: np.array([math.cos(a), math.sin(a)])
    if abs(st["spin"]) > 0.03:
        sc.bar(cxy + 3.0 * u(phi0), cxy + (rho - 2.0) * u(phi0), 1.2, z_top - 0.2, z_top + 0.3,
               [0.62, 0.64, 0.68, 1.0], axis="z")
    sc.bar(cxy + 3.0 * u(phi0 + st["spin"]), cxy + (rho - 2.0) * u(phi0 + st["spin"]), 1.8, z_top - 0.2,
           z_top + 0.6, KNOB, axis="z")
    sc.cylinder(cxy, z_top - 0.2, z_top + 1.2, 3.0, STEEL, axis="z")  # the axle's cap
    if contact_marker and st.get("contact") is not None and st.get("distal_force_n", 0.0) > 0.05:
        p = np.asarray(st["contact"], float)[:2]  # the index pad's contact on the top face
        for k in range(16):
            a0, a1 = 2 * math.pi * k / 16, 2 * math.pi * (k + 0.6) / 16
            sc.bar(p + 3.0 * u(a0), p + 3.0 * u(a1), 0.5, z_top - 0.2, z_top + 0.6, CONTACT, axis="z")
    for k in range(-9, 10):  # the fixed scale, on the sprung sleeve (it follows the disk down, not round)
        a = phi0 + math.radians(10 * k)
        sc.bar(cxy + (rho + 1.8) * u(a), cxy + (rho + (5.0 if k % 3 == 0 else 3.6)) * u(a), 0.45,
               z_top - 0.6, z_top + 0.6, GHOST, axis="z")
    base = tip["base_z"]
    sc.cylinder(cxy, base, z_bot, 2.0, STEEL, axis="z")  # the axle, down to its hub (small: it shadows the disk)
    sc.cylinder(cxy, base - 4.0, base, 4.0, MID, axis="z")
    if abs(st["spin"]) > 0.01:
        sgn = math.copysign(1.0, st["spin"])
        r, z = rho + 7.0, z_top + 1.5
        pts = [cxy + r * u(phi0 + sgn * math.radians(a)) for a in np.linspace(-20, 45, 24)]
        for a_, b_ in zip(pts[:-4], pts[1:-3]):
            sc.bar(a_, b_, 0.8, z - 0.8, z + 0.8, LOAD, axis="z")
        d = (pts[-1] - pts[-5]) / np.linalg.norm(pts[-1] - pts[-5])
        nrm = np.array([-d[1], d[0]])
        base = pts[-1] - 4.5 * d
        sc.extrude([base + 2.6 * nrm, base - 2.6 * nrm, pts[-1]], z - 0.8, z + 0.8, LOAD, smooth_deg=0.0, axis="z")
    return sc


def tipdial_scene(st: dict, tip: dict, folder: Path, contact_marker: bool = True) -> Scene:
    """build_tipdial with the hub's depth: the axle's hub 10 mm below the lowest the disk goes in the frames."""
    lowest = min(f["centre"][2] for f in tip["frames"]) - tip["thick_mm"] / 2
    info = {**tip, "base_z": lowest - 10.0}
    return build_tipdial(st, info, folder, contact_marker)


HOLD_ISO_CAMERA = {"target_mm": (22.0, -4.0, -12.0), "dist": 0.33, "az_deg": 22.0, "el_deg": -36.0, "fov_deg": 33.0,
                   "up": [0.0, 0.0, -1.0]}
"""The pull-out frames as a power grasp, the hand palm up: from above the palm (36 deg), in front of it and on the ring
finger's side, so the three fingers wrapping the cylinder, the pads, the palm housing, the cylinder's end with its
stripe, the load arrow and the ghost show. Chosen over views from further in front (azimuth 45 and 70 deg), where the
closed fingers' backs hide the cylinder and its end face turns edge-on."""


def strip(paths, out: Path, pad: int = 12) -> Path:
    """The images side by side, cropped to their common ink box."""
    from PIL import Image
    ims = [np.asarray(Image.open(p).convert("RGB")) for p in paths]
    ink = np.any(np.stack([im.min(axis=2) < 245 for im in ims]), axis=0)
    r, c = np.where(ink)
    box = (max(r.min() - pad, 0), r.max() + pad, max(c.min() - pad, 0), c.max() + pad)
    Image.fromarray(np.concatenate([im[box[0]:box[1], box[2]:box[3]] for im in ims], axis=1)).save(out)
    print(out)
    return out


def render_hold_iso(snap: dict) -> list[Path]:
    """render_hold_iso{0,1,2}.png and render_hold_iso_strip.png: the pull-out frames with index, middle and ring
    closed on the cylinder at the frame's flexion, from HOLD_ISO_CAMERA."""
    frames, out = snap["hold"]["frames"], []
    for i, st in enumerate(frames):
        ghost = frames[1]["centre"] if i == 2 else None
        sc = build("hold", st, snap, T.OUT / "render3d" / f"hold_iso{i}", ghost=ghost, finger=False,
                   span=task_span())
        out.append(render_on_hand(sc, T.OUT / f"render_hold_iso{i}.png", st["distal"][2], camera=HOLD_ISO_CAMERA))
    strip(out, T.OUT / "render_hold_iso_strip.png")
    return out


def main():
    snap = json.loads((T.OUT / "task_snapshots.json").read_text())
    for task in ("hold", "acquire", "dial"):
        for i, st in enumerate(snap[task]["frames"]):
            frames = snap[task]["frames"]
            ghost = {("acquire", 2): frames[0]["centre"], ("hold", 2): frames[1]["centre"]}.get((task, i))
            sc = build(task, st, snap, T.OUT / "render3d" / f"{task}{i}", ghost=ghost,
                       turn_sign=math.copysign(1.0, frames[-1]["spin"]), finger=False, span=task_span())
            render_on_hand(sc, T.OUT / f"render_{task}{i}.png", st["distal"][2])
    render_hold_iso(snap)
    if "tipdial" in snap:
        tip = snap["tipdial"]
        for i, st in enumerate(tip["frames"]):
            sc = tipdial_scene(st, tip, T.OUT / "render3d" / f"tipdial{i}")
            render_on_hand(sc, T.OUT / f"render_tipdial{i}.png", st["distal"][2], camera=TIPDIAL_CAMERA,
                           ref=TIPDIAL_REF, fingers=TIPDIAL_FINGERS, curled=TIPDIAL_CURLED)


if __name__ == "__main__":
    main()
