# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Drawing-only geometry of the CoDe hand, generated from a ``code_hand_sim/1`` description.

:func:`hand_visuals` is the one place the visual look is made; :func:`~code_isaaclab.assets.builders.write_usd`
(``visuals=True``) and :func:`~code_isaaclab.assets.builders.write_urdf` (``visuals=True``, OBJ meshes) attach its
pieces to their bodies, with no collider and no mass. Every piece is an :class:`Extrusion`: a planar profile in
millimetres extruded along an axis, placed in the hand frame at the geometric zero (where every body takes the
hand frame's orientation, so a piece moves with its body by the body's origin alone).

The finger follows ``sim/drake/render3d.py::add_finger`` (the look of ``sim/drake/logs/hands_ORU_3d.png``), in the
finger's linkage plane: ``x`` from the MCP-FE axis ``O`` toward the PIP pivot ``A``, ``z`` dorsal (away from the
pads), ``y`` lateral along the FE axis. Its inputs are all description fields: the pivots ``O``, ``A``, ``A'``,
``B`` (``joints[].anchor`` of ``<d>_fe``, ``<d>_a``, ``<d>_pip_servo``, ``<d>_b``) and ``B'`` (``loops[].anchor``),
the pad boxes (``bodies[].boxes``) and every servo's frame and profile (``visuals``, ``servo.shape``):

- proximal and distal side plates, in pairs, inside the servo depth: the rounded hull around the link's pivots
  and pad ends, clipped at the pad mount so the pad shows below, and cut around the PIP servo and the FE servo's
  rounded cap;
- a palmar plate behind each pad, cut back where a servo body sits (the PIP servo over the proximal pad, the
  FE servo's cap around ``O``);
- the crossed four-bar in the layers of the design's packaging model
  (``design/core/morphology/mechanism/linkage/packaging.py``, ``LAYERS = ("central", "driver", "coupler")``): the
  pads, servos and pin housings in the central layer, as deep as a servo; the driver ``A'-B`` (the ``servo_link``
  body) one plate just outside it on the servo horn's side (+y), the coupler ``A-B'`` (the ``driver_link`` body)
  one plate just outside it on the other side; each bowed dorsally (sag 8 and 5 mm), the pins where the
  kinematics put them, running from the link's plate through the central layer;
- steel pins and servo hubs, blue pads, dark servo bodies (the packaging D profile).

Lateral layout: render3d's side plates (y +-9.5..11.5 mm) run through the PIP servo (+-11 mm) and its four-bar
plates stand 3.5..8.5 mm outside the servo, into the neighbouring finger (25 mm pitch: 3 mm between servos). Here
each 3 mm gap holds one finger's driver plate and its neighbour's coupler plate (:data:`FACE_GAP_MM`,
:data:`LINK_MM`). :func:`servo_clearances` checks the result over the four-bar's range.

The thumb is plainer: its servo bodies, plates along its links outside the servo depth, and a fingertip case
beyond the IP servo on which the thumb pad sits. The palm is a rounded plate under the abduction servos with an arm
to the thumb's rotation servo, kept outside the thumb's sweep about that axis.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .description import Description, quat_wxyz_to_matrix

MM = 1e-3

PALETTE: dict[str, tuple[float, float, float]] = {
    "plate": (0.84, 0.85, 0.87),
    "shell": (0.86, 0.87, 0.88),
    "accent": (0.33, 0.34, 0.37),
    "link": (0.66, 0.68, 0.71),
    "servo": (0.22, 0.23, 0.25),
    "steel": (0.90, 0.91, 0.93),
    "pad": (0.27, 0.50, 0.93),
}
"""Display colours (sRGB, as render3d's LIGHT, MID, DARK, STEEL and PAD; the palm housing's shell and accents).
USD stores colours linear (:func:`srgb_to_linear`); URDF ``rgba`` and Drake take them as given."""

GAP_MM = 0.5
"""Clearance kept between a plate and a servo body in the plane where a plate is cut back around it."""
FACE_GAP_MM, LINK_MM, HUB_MM = 0.2, 1.2, 0.9
"""The link plates just outside the central layer (the servo depth): their gap to a servo face and thickness; a
servo hub's length out of its face. With the 25 mm finger pitch and 22 mm servo depth, each 3 mm gap between
neighbouring servos holds one finger's driver plate and the neighbour's coupler plate: 0.2 + 1.2 + 0.2 + 1.2 + 0.2."""
DRIVER_SIDE = 1
"""The driver's side along the FE axis (``+1``: the servo horn's, ``+y``); the coupler is on the other."""
SIDE_MM = 2.0
"""Phalanx side plate thickness (inside the servo depth, 1 mm in from its faces)."""
DRIVER_HALF_WIDTH_MM, COUPLER_HALF_WIDTH_MM = 3.2, 2.8
"""The link plates' half widths (render3d's): each ends in a semicircle of this radius about its pin."""
CAP_SEGMENTS = 32


def srgb_to_linear(c: tuple[float, float, float]) -> tuple[float, float, float]:
    """A display (sRGB) colour as the linear value USD ``displayColor`` and ``diffuseColor`` hold."""
    return tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in c)


# --- planar profiles (mm) -------------------------------------------------------------------------------------


def hull(points) -> np.ndarray:
    """Convex hull (counter-clockwise) of 2D points."""
    pts = sorted(map(tuple, np.asarray(points, float)))

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


def circle(c, r: float, n: int = 48) -> np.ndarray:
    return np.array([np.asarray(c, float) + r * np.array([math.cos(t), math.sin(t)])
                     for t in np.linspace(0.0, 2 * math.pi, n, endpoint=False)])


def circles_hull(circles, n: int = 48) -> np.ndarray:
    return hull(np.concatenate([circle(c, r, n) for c, r in circles]))


def rounded_rect(x0, x1, z0, z1, r, n: int = 10) -> np.ndarray:
    r = max(1e-3, min(r, (x1 - x0) / 2, (z1 - z0) / 2))
    return circles_hull([(np.array([x, z]), r) for x in (x0 + r, x1 - r) for z in (z0 + r, z1 - r)], n * 4)


def clip_above(poly, z_min: float) -> np.ndarray:
    """The part of a convex polygon ``(x, z)`` with ``z >= z_min``."""
    out = []
    n = len(poly)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        if p[1] >= z_min:
            out.append(p)
        if (p[1] >= z_min) != (q[1] >= z_min):
            out.append(p + (z_min - p[1]) / (q[1] - p[1]) * (q - p))
    return np.array(out)


def arc_bar(p, q, half_width: float, sag: float, n: int = 32) -> np.ndarray:
    """A link from ``p`` to ``q`` bowed by ``sag`` to its left (+90 deg from ``p->q``), with round ends; the pins
    stay at ``p`` and ``q`` (render3d's ``arc_bar``)."""
    p, q = np.asarray(p, float), np.asarray(q, float)
    d = q - p
    left = np.array([-d[1], d[0]]) / np.linalg.norm(d)
    ctrl = (p + q) / 2 + 2.0 * sag * left
    ts = np.linspace(0.0, 1.0, n)
    pts = np.array([(1 - t) ** 2 * p + 2 * (1 - t) * t * ctrl + t**2 * q for t in ts])
    tan = np.array([2 * (1 - t) * (ctrl - p) + 2 * t * (q - ctrl) for t in ts])
    tan /= np.linalg.norm(tan, axis=1)[:, None]
    nor = np.stack([-tan[:, 1], tan[:, 0]], axis=1)

    def cap(c, t0, start):  # a semicircle of the half width about the pin, from one side of the bar to the other
        a0 = math.atan2(t0[1], t0[0])
        us = (np.linspace(-math.pi / 2, -3 * math.pi / 2, 18) if start  # right side, behind the pin, left side
              else np.linspace(math.pi / 2, -math.pi / 2, 18))[1:-1]  # left side, ahead of the pin, right side
        return [c + half_width * np.array([math.cos(a0 + u), math.sin(a0 + u)]) for u in us]

    return np.array(list(pts + half_width * nor) + cap(pts[-1], tan[-1], False)
                    + list((pts - half_width * nor)[::-1]) + cap(pts[0], tan[0], True))


def _segment_z_range(poly: np.ndarray, x: float) -> tuple[float, float] | None:
    """The z interval a convex polygon covers on the vertical line at ``x`` (None: missed)."""
    zs = []
    n = len(poly)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        if (p[0] - x) * (q[0] - x) <= 0 and abs(q[0] - p[0]) > 1e-12:
            zs.append(p[1] + (x - p[0]) / (q[0] - p[0]) * (q[1] - p[1]))
        elif abs(p[0] - x) < 1e-12:
            zs.append(p[1])
    return (min(zs), max(zs)) if zs else None


def plate_under(x0: float, x1: float, z0: float, z1: float, obstacles, step: float = 0.25,
                min_thickness: float = 0.4) -> np.ndarray | None:
    """A plate on ``z0`` from ``x0`` to ``x1``, up to ``z1`` but below every convex polygon of ``obstacles``
    (dilated beforehand by the clearance); where an obstacle reaches below ``z0 + min_thickness`` the plate ends.
    The longest such run, as a polygon (None if none)."""
    xs = np.arange(x0, x1 + 1e-9, step)
    top = np.full(len(xs), z1)
    for poly in obstacles:
        for k, x in enumerate(xs):
            span = _segment_z_range(poly, x)
            if span is not None and span[1] > z0:
                top[k] = min(top[k], span[0])
    ok = top >= z0 + min_thickness
    best, start = (0, -1), None
    for k, flag in enumerate(list(ok) + [False]):
        if flag and start is None:
            start = k
        elif not flag and start is not None:
            if k - start > best[1] - best[0] + 1:
                best = (start, k - 1)
            start = None
    i, j = best
    if j - i < 2:
        return None
    upper = [(xs[k], top[k]) for k in range(j, i - 1, -1)]
    poly = [(xs[i], z0), (xs[j], z0)] + upper
    keep = [poly[0]]  # drop collinear points along the top
    for k in range(1, len(poly)):
        nxt = poly[(k + 1) % len(poly)]
        a, b = np.subtract(poly[k], keep[-1]), np.subtract(nxt, poly[k])
        if abs(a[0] * b[1] - a[1] * b[0]) > 1e-9:
            keep.append(poly[k])
    return np.array(keep)


def _shapes(geom) -> list[np.ndarray]:
    """The polygons (exteriors, counter-clockwise, without the last repeated point) of a shapely geometry, larger
    than 0.5 mm^2."""
    from shapely.geometry import Polygon
    from shapely.geometry.polygon import orient

    parts = [geom] if isinstance(geom, Polygon) else list(getattr(geom, "geoms", []))
    out = []
    for part in parts:
        if isinstance(part, Polygon) and part.area > 0.5:
            out.append(np.asarray(orient(part.simplify(0.02), 1.0).exterior.coords)[:-1])
    return out


def difference(poly, cuts) -> list[np.ndarray]:
    """``poly`` minus every polygon of ``cuts`` (2D, mm)."""
    from shapely.geometry import Polygon

    geom = Polygon(poly).buffer(0)
    for cut in cuts:
        geom = geom.difference(Polygon(cut).buffer(0))
    return _shapes(geom)


def intersection(poly, other) -> list[np.ndarray]:
    from shapely.geometry import Polygon

    return _shapes(Polygon(poly).buffer(0).intersection(Polygon(other).buffer(0)))


def ear_clip(poly) -> list[tuple[int, int, int]]:
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


def d_profile(shape: dict) -> np.ndarray:
    """A servo's packaging profile (``servo.shape``) in its actuator frame ``(x, y)`` [mm], counter-clockwise."""
    if shape["type"] == "d_extrusion":
        r, rear = shape["radius"] / MM, shape["rear_extent"] / MM
        cap = [(r * math.cos(t), r * math.sin(t)) for t in np.linspace(0.0, math.pi, CAP_SEGMENTS + 1)]
        return np.array([(-r, -rear), (r, -rear)] + cap)
    if shape["type"] == "box_extrusion":
        hw, front = shape["width"] / 2 / MM, shape["shaft_from_front"] / MM
        rear = shape["length"] / MM - front
        return np.array([(-hw, -rear), (hw, -rear), (hw, front), (-hw, front)])
    raise ValueError(f"unknown servo shape {shape['type']!r}")


# --- pieces ---------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Extrusion:
    """A profile in ``(u, v)`` [mm] extruded along ``w`` from ``w0`` to ``w1`` [mm], at ``origin`` [mm] in the hand
    frame at the geometric zero, riding on ``body``. ``servo`` names the servo a piece *is* (its body) and
    ``hub_of`` the servo whose output it is (a hub touching it); both are left out of that servo's clearance."""

    name: str
    body: str
    material: str
    profile: np.ndarray
    origin: np.ndarray
    u: np.ndarray
    v: np.ndarray
    w: np.ndarray
    w0: float
    w1: float
    smooth_deg: float = 35.0
    servo: str | None = None
    hub_of: str | None = None

    def point(self, p2, t: float) -> np.ndarray:
        return self.origin + p2[0] * self.u + p2[1] * self.v + t * self.w

    def mesh(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Vertices [m] (hand frame at the geometric zero), per-vertex normals and triangles; side normals are
        smoothed across corners under ``smooth_deg`` (round ends look round)."""
        poly = np.asarray(self.profile, float)
        if np.sum(poly[:, 0] * np.roll(poly[:, 1], -1) - np.roll(poly[:, 0], -1) * poly[:, 1]) < 0:
            poly = poly[::-1]
        n = len(poly)
        edge_n = []
        for i in range(n):
            d = poly[(i + 1) % n] - poly[i]
            e = d[1] * self.u - d[0] * self.v  # outward for a counter-clockwise profile
            edge_n.append(e / (np.linalg.norm(e) + 1e-12))
        cos_s = math.cos(math.radians(self.smooth_deg))
        verts, normals, faces = [], [], []
        for i in range(n):
            j = (i + 1) % n
            ni = edge_n[i]
            na = edge_n[i - 1] if np.dot(edge_n[i - 1], ni) > cos_s else ni
            nb = edge_n[j] if np.dot(edge_n[j], ni) > cos_s else ni
            va, vb = (na + ni) / np.linalg.norm(na + ni), (nb + ni) / np.linalg.norm(nb + ni)
            base = len(verts)
            for p, nn in ((poly[i], va), (poly[j], vb)):
                for t in (self.w0, self.w1):
                    verts.append(self.point(p, t))
                    normals.append(nn)
            faces += [(base, base + 2, base + 3), (base, base + 3, base + 1)]
        tris = ear_clip(poly)
        for t, sgn in ((self.w0, -1.0), (self.w1, 1.0)):
            base = len(verts)
            for p in poly:
                verts.append(self.point(p, t))
                normals.append(sgn * self.w)
            faces += [(base + a, base + b, base + c) for a, b, c in tris]
        verts, normals = np.array(verts) * MM, np.array(normals)
        fixed = []
        for a, b, c in faces:  # wind every triangle with its normals
            nrm = np.cross(verts[b] - verts[a], verts[c] - verts[a])
            fixed.append((a, c, b) if np.dot(nrm, normals[a] + normals[b] + normals[c]) < 0 else (a, b, c))
        return verts, normals, np.array(fixed, dtype=np.int32)


def rounded_box_mesh(center, half, fillet: float, rot=None, segments: int = 6
                     ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """A rounded box (a box of half extents ``half - fillet`` swept by a sphere of radius ``fillet``), as the convex
    hull of its corner spheres sampled with ``segments`` steps per quarter circle: vertices, per-vertex normals (the
    sphere's) and outward triangles, in the units given. ``segments=2`` gives 56 vertices, a collider hull."""
    from scipy.spatial import ConvexHull

    half, c = np.asarray(half, float), np.asarray(center, float)
    r = min(float(fillet), float(half.min()))
    inner = half - r
    rot = np.eye(3) if rot is None else np.asarray(rot, float)
    pts, nrm = [], []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                for i in range(segments + 1):
                    phi = 0.5 * math.pi * i / segments
                    for j in range(segments + 1 if i else 1):
                        th = 0.5 * math.pi * j / segments
                        n = np.array([sx * math.sin(phi) * math.cos(th), sy * math.sin(phi) * math.sin(th),
                                      sz * math.cos(phi)])
                        pts.append(np.array([sx, sy, sz]) * inner + r * n)
                        nrm.append(n)
    pts, nrm = np.array(pts), np.array(nrm)
    pts, idx = np.unique(np.round(pts, 9), axis=0, return_index=True)
    nrm = nrm[idx]
    hull = ConvexHull(pts)
    faces = []
    for tri, eq in zip(hull.simplices, hull.equations, strict=True):
        a, b, cc = tri
        n = np.cross(pts[b] - pts[a], pts[cc] - pts[a])
        faces.append((a, b, cc) if n @ eq[:3] > 0 else (a, cc, b))
    used = np.unique(np.array(faces))
    remap = -np.ones(len(pts), dtype=np.int64)
    remap[used] = np.arange(len(used))
    faces = remap[np.array(faces)]
    return pts[used] @ rot.T + c, nrm[used] @ rot.T, faces.astype(np.int32)


@dataclass(frozen=True)
class RoundedBox:
    """A rounded box piece [mm]: ``center`` in the hand frame at the geometric zero, ``rot`` its axes (columns)."""

    name: str
    body: str
    material: str
    center: np.ndarray
    half: np.ndarray
    fillet: float
    rot: np.ndarray

    def mesh(self, segments: int = 6) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        verts, normals, faces = rounded_box_mesh(self.center, self.half, self.fillet, self.rot, segments)
        return verts * MM, normals, faces

    def distance(self, points: np.ndarray) -> np.ndarray:
        """Signed distance [mm] of hand-frame points [mm] (negative inside)."""
        r = min(self.fillet, float(np.min(self.half)))
        q = np.abs((points - self.center) @ self.rot) - (self.half - r)
        return np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0) - r


def housing_boxes(desc: Description) -> list[RoundedBox]:
    """The palm housing (``bodies[palm].housing`` of the description) as :class:`RoundedBox` pieces (empty if the
    description has none)."""
    out = []
    for body in desc.bodies:
        for box in body.get("housing", []):
            out.append(RoundedBox(box["name"], body["name"], "shell", _mm(box["center"]), _mm(box["half_extents"]),
                                  float(box["fillet"]) / MM, quat_wxyz_to_matrix(box["quat_wxyz"])))
    return out


def housing_pieces(desc: Description) -> list:
    """The housing's shell (its colliders' shape, drawn finer) and dark accents: a pad on the flat palmar face of the
    palm block, a band along the hood's front edge."""
    shell = housing_boxes(desc)
    out: list = list(shell)
    by_name = {b.name: b for b in shell}
    if "palm_block" in by_name:
        b = by_name["palm_block"]
        lo, hi = b.center - b.half, b.center + b.half
        inset = b.fillet + 1.0
        out.append(RoundedBox("palm_face_pad", b.body, "accent",
                              np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2] + 0.25]),
                              np.array([(hi[0] - lo[0]) / 2 - inset, (hi[1] - lo[1]) / 2 - inset, 0.55]), 0.5,
                              np.eye(3)))
    if "palm_hood" in by_name:
        b = by_name["palm_hood"]
        lo, hi = b.center - b.half, b.center + b.half
        out.append(RoundedBox("palm_front_band", b.body, "accent",
                              np.array([hi[0] - 0.9, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2]),
                              np.array([1.2, (hi[1] - lo[1]) / 2 - b.fillet - 1.0, 2.0]), 0.8, np.eye(3)))
    return out


def _unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _mm(p) -> np.ndarray:
    return np.asarray(p, float) / MM


def _pad_boxes(desc: Description, body: str) -> list[dict]:
    return [box for box in desc.body(body).get("boxes", []) if box["name"].endswith("_pad")]


def _box_corners(box: dict) -> np.ndarray:
    r = quat_wxyz_to_matrix(box["quat_wxyz"])
    h = _mm(box["half_extents"])
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    return _mm(box["center"]) + (signs * h) @ r.T


def _servos(desc: Description) -> dict[str, dict]:
    return {item["servo"]: item for item in desc.data.get("visuals") or [] if item.get("shape") == "servo"}


def servo_pieces(desc: Description) -> list[Extrusion]:
    """Every servo body of the description's ``visuals``: the ``servo.shape`` profile at its actuator frame."""
    profile = d_profile(desc.servo["shape"])
    half = desc.servo["shape"]["depth"] / 2 / MM
    out = []
    for key, item in _servos(desc).items():
        r = quat_wxyz_to_matrix(item["quat_wxyz"])
        out.append(Extrusion(item["name"], item["body"], "servo", profile, _mm(item["position"]), r[:, 0], r[:, 1],
                             r[:, 2], -half, half, servo=key))
    return out


def pad_pieces(desc: Description) -> list[Extrusion]:
    """Every pad box (``bodies[].boxes`` named ``*_pad``) as a slightly rounded blue block."""
    out = []
    for body in desc.bodies:
        for box in _pad_boxes(desc, body["name"]):
            r = quat_wxyz_to_matrix(box["quat_wxyz"])
            hx, hy, hz = _mm(box["half_extents"])
            out.append(Extrusion(f"{box['name']}_visual", body["name"], "pad", rounded_rect(-hx, hx, -hy, hy, 0.8),
                                 _mm(box["center"]), r[:, 0], r[:, 1], r[:, 2], -hz, hz))
    return out


@dataclass(frozen=True)
class FingerPlane:
    """A finger's linkage plane at the geometric zero: origin ``O`` (the FE axis), ``u`` toward ``A``, ``v`` dorsal,
    ``w`` along the FE axis [mm]."""

    origin: np.ndarray
    u: np.ndarray
    v: np.ndarray
    w: np.ndarray

    def xz(self, p) -> np.ndarray:
        d = np.asarray(p, float) - self.origin
        return np.array([d @ self.u, d @ self.v])

    def y(self, p) -> float:
        return float((np.asarray(p, float) - self.origin) @ self.w)

    def piece(self, name, body, material, profile, w0, w1, **kw) -> Extrusion:
        return Extrusion(name, body, material, np.asarray(profile, float), self.origin, self.u, self.v, self.w,
                         min(w0, w1), max(w0, w1), **kw)


def finger_plane(desc: Description, digit: str) -> FingerPlane:
    o = _mm(desc.joint(f"{digit}_fe")["anchor"])
    w = _unit(desc.joint(f"{digit}_fe")["axis"])
    a = _mm(desc.joint(f"{digit}_a")["anchor"])
    u = _unit((a - o) - ((a - o) @ w) * w)
    v = np.cross(u, w)
    pad = _box_corners(_pad_boxes(desc, f"{digit}_proximal")[0]).mean(axis=0)
    if (pad - o) @ v > 0:  # v must point away from the pads
        v = -v
    return FingerPlane(o, u, v, w)


def _servo_polygon(desc: Description, key: str, plane: FingerPlane, dilate: float = 0.0) -> np.ndarray:
    """A servo body's profile seen in the finger plane (its shaft along the plane normal), dilated by ``dilate``."""
    item = _servos(desc)[key]
    r = quat_wxyz_to_matrix(item["quat_wxyz"])
    pts = [plane.xz(_mm(item["position"]) + r[:, :2] @ p) for p in d_profile(desc.servo["shape"])]
    return circles_hull([(p, dilate) for p in pts], 8) if dilate > 0 else hull(pts)


def finger_pieces(desc: Description, digit: str) -> list[Extrusion]:
    """One finger's plates, links, pins and the MCP bracket (module docstring)."""
    pl = finger_plane(desc, digit)
    d = digit
    O, A = np.zeros(2), pl.xz(_mm(desc.joint(f"{d}_a")["anchor"]))  # noqa: E741 (the pivots keep the linkage names)
    Ap, B = pl.xz(_mm(desc.joint(f"{d}_pip_servo")["anchor"])), pl.xz(_mm(desc.joint(f"{d}_b")["anchor"]))
    loop = next(c for c in desc.loops if c["child"] == f"{d}_distal")
    Bp = pl.xz(_mm(loop["anchor"]))
    prox, dist = (np.array([pl.xz(p) for p in _box_corners(_pad_boxes(desc, f"{d}_{s}")[0])])
                  for s in ("proximal", "distal"))
    px0, px1, pm = prox[:, 0].min(), prox[:, 0].max(), prox[:, 1].max()
    dx0, dx1, dm = dist[:, 0].min(), dist[:, 0].max(), dist[:, 1].max()
    half_pad = max(abs(pl.y(p)) for p in _box_corners(_pad_boxes(desc, f"{d}_proximal")[0]))
    S = desc.servo["shape"]["depth"] / 2 / MM
    cap_r = desc.servo["shape"].get("radius", 0.0) / MM
    outer = (S + FACE_GAP_MM, S + FACE_GAP_MM + LINK_MM)  # the link layers, just outside the central layer
    side = (S - 1.0 - SIDE_MM, S - 1.0)  # the phalanx plates, inside the servo depth
    out: list[Extrusion] = []

    def pair(name, body, material, profiles, layer, **kw):
        for sgn, tag in ((-1, "r"), (1, "l")):
            for k, prof in enumerate(profiles):
                out.append(pl.piece(f"{name}_{tag}{k or ''}", body, material, prof, sgn * layer[0], sgn * layer[1],
                                    **kw))

    pip = _servo_polygon(desc, f"{d}/pip", pl, GAP_MM)
    fe = _servo_polygon(desc, f"{d}/fe", pl, GAP_MM)
    fe_cap = circle(O, cap_r + GAP_MM, 64)

    # proximal phalanx: side plates in the central layer, cut around the PIP servo and the FE servo's cap; the
    # palmar plate behind the pad
    body = f"{d}_proximal"
    hull_p = clip_above(circles_hull([(O, 8.0), (A, 5.0), (Ap, 6.5), ((px1 + 1.0, pm + 1.0), 1.0),
                                      ((px0 - 6.0, pm + 1.0), 1.0)]), pm)
    pair(f"{d}_proximal_side", body, "plate", difference(hull_p, [pip, fe, fe_cap]), side)
    palmar = plate_under(px0 - 6.0, px1 + 2.0, pm, 1.0, [pip, fe, fe_cap])
    if palmar is not None:
        out.append(pl.piece(f"{d}_proximal_palmar", body, "plate", palmar, -half_pad, half_pad))
    pair(f"{d}_fe_hub", body, "steel", [circle(O, 4.0, 64)], (S, S + FACE_GAP_MM + HUB_MM), hub_of=f"{d}/fe")

    # distal phalanx (render3d's hull) in the central layer
    body = f"{d}_distal"
    hull_d = circles_hull([(B, 5.5), (Bp, 5.0), ((dx1 + 0.5, dm + 1.5), 1.5), ((dx1 - 3.0, B[1] + 2.0), 2.0),
                           ((dx0, dm + 1.0), 1.0)])
    pair(f"{d}_distal_side", body, "plate", [clip_above(hull_d, dm)], side)
    out.append(pl.piece(f"{d}_distal_palmar", body, "plate", rounded_rect(dx0 - 1.0, dx1 + 1.5, dm, B[1] + 1.0, 1.0),
                        -half_pad, half_pad))

    # crossed four-bar (``packaging.LAYERS``): the driver A'-B on the servo horn's side (+y), the coupler A-B' on
    # the other; pins from each link's plate through the central layer to the far phalanx plate
    sgn = 1.0 if DRIVER_SIDE > 0 else -1.0
    drv = (sgn * outer[0], sgn * outer[1])
    cpl = (-sgn * outer[0], -sgn * outer[1])
    out.append(pl.piece(f"{d}_driver", f"{d}_servo_link", "link", arc_bar(Ap, B, DRIVER_HALF_WIDTH_MM, 8.0), *drv))
    out.append(pl.piece(f"{d}_coupler", f"{d}_driver_link", "link", arc_bar(A, Bp, COUPLER_HALF_WIDTH_MM, 5.0), *cpl))
    out.append(pl.piece(f"{d}_pip_hub", f"{d}_servo_link", "steel", circle(Ap, 2.6, 64), sgn * S, drv[0],
                        hub_of=f"{d}/pip"))
    out.append(pl.piece(f"{d}_pin_b", f"{d}_distal", "steel", circle(B, 1.6, 32), -sgn * side[1], drv[1]))
    for name, c, body in ((f"{d}_pin_a", A, f"{d}_proximal"), (f"{d}_pin_bp", Bp, f"{d}_distal")):
        out.append(pl.piece(name, body, "steel", circle(c, 1.6, 32), cpl[1], sgn * side[1]))

    # MCP: a bracket under the abduction servo out to the FE servo's back, a hub on the AA axis
    servos = _servos(desc)
    aa, fe_item = servos[f"{d}/aa"], servos[f"{d}/fe"]
    r_aa = quat_wxyz_to_matrix(aa["quat_wxyz"])
    c, shaft = _mm(aa["position"]), r_aa[:, 2]
    toward = _mm(fe_item["position"]) - c
    sgn = 1.0 if toward @ shaft > 0 else -1.0
    u = _unit(toward - (toward @ shaft) * shaft)
    v = np.cross(shaft, u)
    r_fe = quat_wxyz_to_matrix(fe_item["quat_wxyz"])
    fe_back = min((_mm(fe_item["position"]) + r_fe[:, :2] @ p - c) @ u for p in d_profile(desc.servo["shape"]))
    x_end = fe_back - GAP_MM
    profile = circles_hull([((0.0, 0.0), cap_r), ((x_end - 2.0, -(S - 2.0)), 2.0), ((x_end - 2.0, S - 2.0), 2.0)])
    w_range = (sgn * (S + GAP_MM), sgn * (S + GAP_MM + 2.5))
    out.append(Extrusion(f"{d}_mcp_bracket", f"{d}_mcp", "plate", profile, c, u, v, shaft, min(w_range),
                         max(w_range)))
    out.append(Extrusion(f"{d}_aa_hub", f"{d}_mcp", "steel", circle((0.0, 0.0), 4.0, 64), c, u, v, shaft,
                         min(sgn * S, w_range[0]), max(sgn * S, w_range[0]), hub_of=f"{d}/aa"))
    return out


def _aabb(piece: Extrusion) -> tuple[np.ndarray, np.ndarray]:
    pts = np.array([piece.point(p, t) for p in piece.profile for t in (piece.w0, piece.w1)])
    return pts.min(axis=0), pts.max(axis=0)


def thumb_and_palm_pieces(desc: Description, fingers: list[Extrusion]) -> list[Extrusion]:
    """The thumb (servo bodies, plates along its links, the fingertip case under its pad) and the palm plate."""
    servos = _servos(desc)
    S = desc.servo["shape"]["depth"] / 2 / MM
    r_cap = desc.servo["shape"].get("radius", 0.0) / MM
    profile = d_profile(desc.servo["shape"])
    out: list[Extrusion] = []

    def frame(key):
        item = servos[key]
        r = quat_wxyz_to_matrix(item["quat_wxyz"])
        return _mm(item["position"]), r[:, 0], r[:, 1], r[:, 2]

    def box(key):
        c, u, v, w = frame(key)
        pts = np.array([c + p[0] * u + p[1] * v + t * w for p in profile for t in (-S, S)])
        return pts.min(axis=0), pts.max(axis=0)

    side = (S + GAP_MM, S + GAP_MM + SIDE_MM)
    outer = (side[1] + 0.6, side[1] + 0.6 + SIDE_MM)  # the metacarpal's plates, outside the proximal's

    # CMC: a plate from the RO servo's output face to the RAA servo's back, across the RAA servo's section
    c, u, v, w = frame("thumb/ro")
    lo, hi = box("thumb/raa")
    w0 = S + GAP_MM
    w1 = min((p - c) @ w for p in (lo, hi, np.array([lo[0], hi[1], lo[2]]), np.array([hi[0], lo[1], hi[2]]))) - GAP_MM
    corners = [np.array([x, y, z]) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    uv = np.array([[(p - c) @ u, (p - c) @ v] for p in corners])
    out.append(Extrusion("thumb_cmc_plate", "thumb_cmc", "plate",
                         rounded_rect(uv[:, 0].min(), uv[:, 0].max(), uv[:, 1].min(), uv[:, 1].max(), 4.0),
                         c, u, v, w, w0, w1))
    out.append(Extrusion("thumb_ro_hub", "thumb_cmc", "steel", circle((0, 0), 5.0, 64), c, u, v, w, S, w0,
                         hub_of="thumb/ro"))

    # metacarpal: plates (outer layer about the MCP shaft) from the RAA output face to the MCP axis
    cm, um, vm, wm = frame("thumb/mcp")
    craa, _, _, wraa = frame("thumb/raa")
    start = craa + (S + GAP_MM) * wraa
    s2 = np.array([(start - cm) @ um, (start - cm) @ vm])
    along = s2 / np.linalg.norm(s2)
    meta =circles_hull([((0.0, 0.0), 8.0), (s2 - along * 7.0, 7.0)])  # round at the MCP pin and at the RAA hub
    for s, tag in ((-1, "r"), (1, "l")):
        out.append(Extrusion(f"thumb_metacarpal_plate_{tag}", "thumb_metacarpal", "plate", meta, cm, um, vm, wm,
                             min(s * outer[0], s * outer[1]), max(s * outer[0], s * outer[1])))
    _, uraa, vraa, _ = frame("thumb/raa")
    out.append(Extrusion("thumb_raa_hub", "thumb_metacarpal", "steel", circle((0, 0), 5.0, 64), craa, uraa, vraa, wraa,
                         S, S + GAP_MM, hub_of="thumb/raa"))
    for s, tag in ((-1, "r"), (1, "l")):
        out.append(Extrusion(f"thumb_mcp_hub_{tag}", "thumb_metacarpal", "steel", circle((0, 0), 4.0, 64), cm, um, vm,
                             wm, min(s * S, s * (outer[1] + 0.8)), max(s * S, s * (outer[1] + 0.8)),
                             hub_of="thumb/mcp"))

    # proximal: plates (inner layer) along the MCP servo's body to the IP axis, the IP hub
    ci = frame("thumb/ip")[0]
    ip2 = np.array([(ci - cm) @ um, (ci - cm) @ vm])
    along = ip2 / np.linalg.norm(ip2)
    mid = along * (r_cap + 4.0)
    prox = circles_hull([(ip2, 8.0), (mid + along * 3.0, 7.0)])  # round at the IP pin and over the MCP servo
    for s, tag in ((-1, "r"), (1, "l")):
        out.append(Extrusion(f"thumb_proximal_plate_{tag}", "thumb_proximal", "plate", prox, cm, um, vm, wm,
                             min(s * side[0], s * side[1]), max(s * side[0], s * side[1])))
        out.append(Extrusion(f"thumb_ip_hub_{tag}", "thumb_proximal", "steel", circle(ip2, 4.0, 64), cm, um, vm, wm,
                             min(s * S, s * (side[1] + 0.8)), max(s * S, s * (side[1] + 0.8)), hub_of="thumb/ip"))

    # distal: the fingertip case beyond the IP servo's body, out to the tip, under the pad
    ci, ui, vi, wi = frame("thumb/ip")
    rear = -profile[:, 1].min()
    tip = _mm(desc.data["tips"]["thumb"])
    tip_v = (tip - ci) @ vi
    pads = _pad_boxes(desc, "thumb_distal") if any(b["name"] == "thumb_distal" for b in desc.bodies) else []
    face, half_w = r_cap, S - 1.0
    if pads:
        corners = _box_corners(pads[0])
        us = np.array([(p - ci) @ ui for p in corners])
        face = us.min() if us.mean() > 0 else -us.max()
        half_w = max(abs((p - ci) @ wi) for p in corners) + 1.0
        sign = 1.0 if us.mean() > 0 else -1.0
    else:
        sign = 1.0
    v_lo, v_hi = (tip_v - 2.0, -rear - GAP_MM) if tip_v < 0 else (rear + GAP_MM, tip_v + 2.0)
    u_lo, u_hi = (-r_cap, face) if sign > 0 else (-face, r_cap)
    out.append(Extrusion("thumb_tip_case", "thumb_distal", "plate", rounded_rect(u_lo, u_hi, v_lo, v_hi, 5.0),
                         ci, ui, vi, wi, -half_w, half_w))

    # palm: a rounded plate under the abduction brackets, an arm round the thumb's sweep to the RO servo
    brackets = [p for p in fingers if p.name.endswith("_mcp_bracket")]
    z_axis = brackets[0].w if brackets else np.array([0.0, 0.0, 1.0])
    lows = [_aabb(p)[0] for p in brackets]
    highs = [_aabb(p)[1] for p in brackets]
    sgn = -1.0 if brackets and np.mean([b.w0 for b in brackets]) < 0 else 1.0
    top = (min(lo[2] for lo in lows) if sgn < 0 else max(hi[2] for hi in highs)) + sgn * GAP_MM
    zr = (top - 3.0, top) if sgn < 0 else (top, top + 3.0)
    aa_boxes = [box(k) for k in servos if k.endswith("/aa")]
    raa_lo, raa_hi = box("thumb/raa")
    ro_lo, ro_hi = box("thumb/ro")
    x0 = max(min(b[0][0] for b in aa_boxes) - 3.0, raa_hi[0] + GAP_MM)
    x1 = max(_mm(servos[k]["position"])[0] for k in servos if k.endswith("/aa")) + 2.0
    y0, y1 = min(b[0][1] for b in aa_boxes) - 1.5, max(b[1][1] for b in aa_boxes) + 1.5
    origin = np.zeros(3)
    ex, ey = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
    out.append(Extrusion("palm_plate", "palm", "plate", rounded_rect(x0, x1, y0, y1, 6.0), origin, ex, ey, z_axis,
                         *zr))
    # the thumb's sweep about the RO axis over the RO joint's range: the arm passes beyond it
    c_ro = frame("thumb/ro")[0]
    ro = next(j for j in desc.joints if j["child"] == "thumb_cmc")
    boxes = [box(k) for k in servos if k.startswith("thumb/") and k != "thumb/ro"]
    boxes += [_aabb(p) for p in out if p.body.startswith("thumb_")]
    corners = np.array([[b[i][0], b[j][1], b[k][2]] for b in boxes for i in (0, 1) for j in (0, 1) for k in (0, 1)])
    swept = np.concatenate([(corners - c_ro) @ _rotation(ro["axis"], a).T + c_ro
                            for a in np.linspace(*ro["limits_deg"], 37)])
    toward_fingers = y0 < c_ro[1]
    y_arm = swept[:, 1].min() - GAP_MM if toward_fingers else swept[:, 1].max() + GAP_MM
    arm = (rounded_rect(ro_lo[0], x0 + 8.0, y_arm - 7.0, y_arm, 3.0) if toward_fingers
           else rounded_rect(ro_lo[0], x0 + 8.0, y_arm, y_arm + 7.0, 3.0))
    out.append(Extrusion("palm_thumb_arm", "palm", "plate", arm, origin, ex, ey, z_axis, *zr))
    # under the RO servo's x range (clear of the CMC plate): a plate from the arm to the servo's side
    side_y = ro_lo[1] - GAP_MM
    out.append(Extrusion("palm_ro_floor", "palm", "plate",
                         rounded_rect(ro_lo[0], ro_hi[0] - GAP_MM, y_arm - 7.0, side_y, 3.0), origin, ex, ey, z_axis,
                         *zr))
    out.append(Extrusion("palm_ro_bracket", "palm", "plate",
                         rounded_rect(ro_lo[0], ro_hi[0] - GAP_MM, zr[0], ro_hi[2], 3.0), origin, ex,
                         np.array([0.0, 0.0, 1.0]), np.array([0.0, -1.0, 0.0]), -side_y, -side_y + 2.0))
    return out


def hand_visuals(desc: Description) -> list[Extrusion]:
    """Every visual piece of the hand (module docstring), each on the body it moves with."""
    fingers = [p for d in desc.four_bar_digits() for p in finger_pieces(desc, d)]
    out = servo_pieces(desc) + pad_pieces(desc) + fingers
    housing = housing_pieces(desc)
    if "thumb/ro" in _servos(desc):
        thumb = thumb_and_palm_pieces(desc, fingers)
        out += [p for p in thumb if not (housing and p.body == "palm")]  # the housing replaces the palm plates
    return out + housing


def body_meshes(desc: Description, pieces: list[Extrusion] | None = None
                ) -> dict[tuple[str, str], tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """The pieces merged per ``(body, material)``: vertices [m] in the body frame (its origin at its joint
    anchor, the hand frame's orientation), per-vertex normals and triangles."""
    origin = {b["name"]: np.asarray(b["origin"], float) for b in desc.bodies}
    merged: dict[tuple[str, str], list] = {}
    for piece in hand_visuals(desc) if pieces is None else pieces:
        verts, normals, faces = piece.mesh()
        merged.setdefault((piece.body, piece.material), []).append((verts - origin[piece.body], normals, faces))
    out = {}
    for key, parts in merged.items():
        offs = np.cumsum([0] + [len(p[0]) for p in parts[:-1]])
        out[key] = (np.concatenate([p[0] for p in parts]), np.concatenate([p[1] for p in parts]),
                    np.concatenate([p[2] + o for p, o in zip(parts, offs, strict=True)]))
    return out


# --- posing and the servo clearance check ---------------------------------------------------------------------


def _rotation(axis, deg: float) -> np.ndarray:
    a = _unit(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    t = math.radians(deg)
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


def body_poses(desc: Description, angles_deg: dict[str, float]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Each body's motion from the geometric zero, ``x -> R x + t`` (``t`` in mm), with the tree joints at
    ``angles_deg`` (missing joints at zero)."""
    poses = {b["name"]: (np.eye(3), np.zeros(3)) for b in desc.bodies}
    for joint in desc.joints:  # tree order: a parent's pose is set before its children
        rp, tp = poses[joint["parent"]]
        rj = _rotation(joint["axis"], angles_deg.get(joint["name"], 0.0))
        c = _mm(joint["anchor"])
        poses[joint["child"]] = (rp @ rj, rp @ (c - rj @ c) + tp)
    return poses


def posed(piece: Extrusion, pose: tuple[np.ndarray, np.ndarray]) -> Extrusion:
    r, t = pose
    return Extrusion(piece.name, piece.body, piece.material, piece.profile, r @ piece.origin + t, r @ piece.u,
                     r @ piece.v, r @ piece.w, piece.w0, piece.w1, piece.smooth_deg, piece.servo, piece.hub_of)


def _inside(points: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Even-odd point-in-polygon for ``(n, 2)`` points."""
    x, y = points[:, 0:1], points[:, 1:2]
    a, b = poly, np.roll(poly, -1, axis=0)
    crosses = ((a[:, 1] > y) != (b[:, 1] > y)) & (
        x < (b[:, 0] - a[:, 0]) * (y - a[:, 1]) / (b[:, 1] - a[:, 1] + 1e-300) + a[:, 0])
    return crosses.sum(axis=1) % 2 == 1


def _point_segments(points: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Distance of each point to the polygon's boundary."""
    a, b = poly, np.roll(poly, -1, axis=0)
    d = b - a
    t = np.clip(((points[:, None, :] - a) * d).sum(-1) / np.maximum((d * d).sum(-1), 1e-300), 0.0, 1.0)
    return np.linalg.norm(points[:, None, :] - (a + t[..., None] * d), axis=-1).min(axis=1)


def _segments_cross(p: np.ndarray, q: np.ndarray) -> bool:
    a, b = p, np.roll(p, -1, axis=0)
    c, d = q, np.roll(q, -1, axis=0)

    def orient(o, s, r):
        return (s[..., 0] - o[..., 0]) * (r[..., 1] - o[..., 1]) - (s[..., 1] - o[..., 1]) * (r[..., 0] - o[..., 0])

    A, B, C, D = a[:, None], b[:, None], c[None], d[None]
    return bool(np.any((orient(A, B, C) * orient(A, B, D) < 0) & (orient(C, D, A) * orient(C, D, B) < 0)))


def polygon_distance(p: np.ndarray, q: np.ndarray) -> float:
    """Distance between two simple polygons, 0 when they overlap."""
    if _inside(p[:1], q)[0] or _inside(q[:1], p)[0] or _segments_cross(p, q):
        return 0.0
    return float(min(_point_segments(p, q).min(), _point_segments(q, p).min()))


def _surface_samples(piece: Extrusion, step: float) -> np.ndarray:
    poly = np.asarray(piece.profile, float)
    ring = []
    for a, b in zip(poly, np.roll(poly, -1, axis=0), strict=True):
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        ring += [a + (b - a) * k / n for k in range(n)]
    ring = np.array(ring)
    lo, hi = poly.min(axis=0), poly.max(axis=0)
    gx, gy = np.meshgrid(np.arange(lo[0], hi[0] + step, step), np.arange(lo[1], hi[1] + step, step))
    grid = np.stack([gx.ravel(), gy.ravel()], axis=1)
    grid = grid[_inside(grid, poly)]
    ts = np.linspace(piece.w0, piece.w1, max(2, int(math.ceil((piece.w1 - piece.w0) / step)) + 1))
    sides = [ring @ np.stack([piece.u, piece.v]) + t * piece.w for t in ts]
    caps = [grid @ np.stack([piece.u, piece.v]) + t * piece.w for t in (piece.w0, piece.w1)]
    return np.concatenate(sides + caps) + piece.origin


def _distance_to_extrusion(points: np.ndarray, piece: Extrusion) -> np.ndarray:
    """Distance of 3D points to an extrusion (negative inside: minus the depth to its nearest face)."""
    d = points - piece.origin
    uv = np.stack([d @ piece.u, d @ piece.v], axis=1)
    t = d @ piece.w
    planar = _point_segments(uv, np.asarray(piece.profile, float))
    inside_p = _inside(uv, np.asarray(piece.profile, float))
    axial = np.maximum(piece.w0 - t, t - piece.w1)
    out = np.where(inside_p, np.maximum(axial, 0.0), np.hypot(planar, np.maximum(axial, 0.0)))
    inside = inside_p & (axial < 0)
    return np.where(inside, -np.minimum(planar, -axial), out)


def clearance(piece: Extrusion, servo: Extrusion, step: float = 0.4) -> float:
    """The distance between a piece and a servo body [mm]; negative: they intersect. Exact when both extrude along
    the same axis (the finger's, and the thumb's along its flexion axes); otherwise from surface samples ``step``
    apart, both ways."""
    if abs(abs(piece.w @ servo.w) - 1.0) < 1e-9:
        to2 = np.stack([servo.u, servo.v])
        p2 = (np.array([piece.point(p, 0.0) for p in piece.profile]) - servo.origin) @ to2.T
        q2 = np.asarray(servo.profile, float)
        off = (piece.origin - servo.origin) @ servo.w
        sgn = 1.0 if piece.w @ servo.w > 0 else -1.0
        lo, hi = sorted((off + sgn * piece.w0, off + sgn * piece.w1))
        gap = max(servo.w0 - hi, lo - servo.w1)
        planar = polygon_distance(p2, q2)
        if planar == 0.0 and gap < 0.0:
            return float(gap)  # minus the lateral overlap
        return float(math.hypot(planar, max(gap, 0.0)))
    best = math.inf
    for src, dst in ((piece, servo), (servo, piece)):
        pts = _surface_samples(src, step)
        lo, hi = _aabb(dst)
        for margin in (2.0, 8.0, math.inf):  # only the samples near the other body count
            near = pts[np.all((pts > lo - margin) & (pts < hi + margin), axis=1)]
            found = float(_distance_to_extrusion(near, dst).min()) if len(near) else math.inf
            if found <= margin:  # a sample outside the box is farther than the margin
                best = min(best, found)
                break
    return best


def _aabb_gap(a: Extrusion, b: Extrusion) -> float:
    (alo, ahi), (blo, bhi) = _aabb(a), _aabb(b)
    return float(np.linalg.norm(np.maximum(0.0, np.maximum(alo - bhi, blo - ahi))))


def servo_clearances(desc: Description, poses: list[dict[str, float]], near_mm: float = 6.0
                     ) -> dict[tuple[str, str], float]:
    """The smallest distance [mm] between every visual piece (but the servo bodies themselves and each servo's own
    output hub) and every servo body over the joint ``poses`` (dicts of tree joint angles [deg]), per ``(piece,
    servo)`` pair that comes within ``near_mm``. Negative: they intersect."""
    pieces = [p for p in hand_visuals(desc) if isinstance(p, Extrusion)]
    servos = [p for p in pieces if p.servo is not None]
    others = [p for p in pieces if p.servo is None]
    out: dict[tuple[str, str], float] = {}
    seen: dict[tuple[str, str], set] = {}
    for angles in poses:
        bp = body_poses(desc, angles)
        ps = {p.name: posed(p, bp[p.body]) for p in pieces}
        for s in servos:
            rs, ts = bp[s.body]
            for p in others:
                if p.hub_of == s.servo:
                    continue
                rp, tp = bp[p.body]  # the pair's relative motion: skip a pose already checked
                rel = tuple(np.round(np.concatenate([(rs.T @ rp).ravel(), rs.T @ (tp - ts)]), 6))
                key = (p.name, s.servo)
                if rel in seen.setdefault(key, set()):
                    continue
                seen[key].add(rel)
                a, b = ps[p.name], ps[s.name]
                if _aabb_gap(a, b) > near_mm:
                    continue
                out[key] = min(out.get(key, math.inf), clearance(a, b))
    return out


def _capsule_samples(cap: dict, step: float) -> np.ndarray:
    """Points [mm] on a capsule's surface (``bodies[].capsule``, hand frame at the geometric zero)."""
    a, b, r = _mm(cap["start"]), _mm(cap["end"]), float(cap["radius"]) / MM
    axis = b - a
    length = float(np.linalg.norm(axis))
    u = axis / length if length > 1e-9 else np.array([0.0, 0.0, 1.0])
    e1 = np.cross(u, [1.0, 0.0, 0.0] if abs(u[0]) < 0.9 else [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(u, e1)
    n = max(8, int(2 * math.pi * r / step))
    ring = [math.cos(t) * e1 + math.sin(t) * e2 for t in np.linspace(0, 2 * math.pi, n, endpoint=False)]
    pts = [a + t * axis + r * d for t in np.linspace(0, 1, max(2, int(length / step) + 1)) for d in ring]
    for c, sgn in ((a, -1.0), (b, 1.0)):  # hemispheres
        for phi in np.linspace(0, math.pi / 2, max(3, int(r * math.pi / 2 / step))):
            pts += [c + r * (math.cos(phi) * d + sgn * math.sin(phi) * u) for d in ring]
    return np.array(pts)


def _capsule_distance(points: np.ndarray, cap: dict, pose) -> np.ndarray:
    r, t = pose
    a, b, rad = r @ _mm(cap["start"]) + t, r @ _mm(cap["end"]) + t, float(cap["radius"]) / MM
    ab = b - a
    s = np.clip(((points - a) @ ab) / max(ab @ ab, 1e-12), 0.0, 1.0)
    return np.linalg.norm(points - (a + s[:, None] * ab), axis=1) - rad


def _box_samples(box: dict, step: float) -> np.ndarray:
    r = quat_wxyz_to_matrix(box["quat_wxyz"])
    h = _mm(box["half_extents"])
    axes = [np.arange(-hi, hi + 1e-9, step) if hi > step else np.array([-hi, hi]) for hi in h]
    g = np.array(np.meshgrid(*axes, indexing="ij")).reshape(3, -1).T
    surface = g[np.any(np.isclose(np.abs(g), h, atol=step / 2), axis=1)]
    return surface @ r.T + _mm(box["center"])


def housing_clearances(desc: Description, poses: list[dict[str, float]], step: float = 0.5
                       ) -> dict[tuple[str, str], tuple[float, int]]:
    """The smallest distance [mm] between the palm housing (its rounded boxes and their accents, which the palm's
    convex colliders lie inside) and every moving body, per ``(body, "visual" | "collider")``, over the joint
    ``poses``, with the index of the pose where it is reached. Negative: they intersect. From surface samples
    ``step`` apart on the moving parts (against the boxes' exact distance) and the housing's corner vertices (against
    the moving parts' exact distance)."""
    housing = [p for p in hand_visuals(desc) if isinstance(p, RoundedBox) and p.body == "palm"]
    if not housing:
        return {}
    pieces = [p for p in hand_visuals(desc) if isinstance(p, Extrusion) and p.body != "palm"]
    lo = np.min([b.center - b.half for b in housing], axis=0) - 8.0
    hi = np.max([b.center + b.half for b in housing], axis=0) + 8.0
    corners = np.concatenate([b.mesh(4)[0] / MM for b in housing])
    samples: dict[tuple[str, str], list] = {}
    for piece in pieces:
        samples.setdefault((piece.body, "visual"), []).append(_surface_samples(piece, step))
    colliders: dict[str, list] = {}
    for body in desc.bodies:
        if body["name"] == "palm":
            continue
        parts = [_capsule_samples(body["capsule"], step)] + [_box_samples(b, step) for b in body.get("boxes", [])]
        samples[(body["name"], "collider")] = parts
        colliders[body["name"]] = body
    samples = {k: np.concatenate(v) for k, v in samples.items()}
    out: dict[tuple[str, str], tuple[float, int]] = {}
    for k, angles in enumerate(poses):
        bp = body_poses(desc, angles)
        for (body, kind), pts in samples.items():
            r, t = bp[body]
            moved = pts @ r.T + t
            near = moved[np.all((moved > lo) & (moved < hi), axis=1)]
            d = min((float(b.distance(near).min()) for b in housing), default=math.inf) if len(near) else math.inf
            if kind == "visual":  # the housing's corners into the moving plates
                for piece in pieces:
                    if piece.body == body:
                        d = min(d, float(_distance_to_extrusion(corners, posed(piece, bp[body])).min()))
            else:
                cap = colliders[body]["capsule"]
                d = min(d, float(_capsule_distance(corners, cap, bp[body]).min()))
            key = (body, kind)
            if d < out.get(key, (math.inf, -1))[0]:
                out[key] = (d, k)
    return out


def finger_poses(desc: Description, q_degs) -> list[dict[str, float]]:
    """Every finger closed at each PIP flexion of ``q_degs`` (the ``four_bar`` tables), the rest at zero."""
    return [{k: v for d in desc.four_bar_digits() for k, v in desc.closed_pose(d, q).items()} for q in q_degs]


__all__ = ["Extrusion", "PALETTE", "RoundedBox", "body_meshes", "body_poses", "clearance", "finger_plane",
           "finger_poses", "hand_visuals", "housing_boxes", "housing_clearances", "posed", "rounded_box_mesh",
           "servo_clearances", "srgb_to_linear"]
