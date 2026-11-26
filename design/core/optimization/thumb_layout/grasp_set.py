"""Grasp-set benchmark: task-agnostic, kinematics-only grasp analysis of a hand over a fixed object set.

The pre-registration and results: ``docs/experimental/grasp_set.md``. Everything is in the hand frame H [mm]; joint
angles in radians inside the search. The hand model (kinematics, joint limits, capsules, palm boxes) is the thumb-layout
evaluator's (:mod:`.evaluator`), imported unchanged.

For a hand, an object (:class:`Shape`, free in pose) and a grasp type (:data:`GRASPS`: which pads touch), a multi-start
Levenberg–Marquardt search over the joints, the object pose and the contact places on the pads looks for a grasp that
is (1) in contact: each contact point within 1 mm of the surface, (2) aligned: the pad normal within the friction cone
of the surface's inward normal, (3) within the joint limits (thumb 5 deg inside), (4) collision-free (signed distance
>= -1 mm) apart from the contacting pads, and (5) force closure: Nguyen's antipodal test for two contacts (no torsional
friction: closure up to rotation about the contact line), Ferrari–Canny epsilon (L1, 8-edge cones) > 0 for more.
Quality: epsilon (for two contacts in the 5-D wrench subspace they span).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import ConvexHull, QhullError, cKDTree

from core.morphology.code_hand import FINGERS, HandParameters
from core.optimization.thumb_layout.design import PAD_FACE_MM, ThumbDesign
from core.optimization.thumb_layout.evaluator import (
    _SAMPLES,
    COLLISION_MM,
    FUNCTIONAL_MARGIN_DEG,
    PAD_EDGE_INSET_MM,
    REACH_MM,
    Evaluator,
    _apply,
    capsule_box_distance,
    segment_distance,
)

MUS = (0.5, 1.0)
CONE_EDGES = 8
THUMB_PAD_WIDTH_MM = 18.0
"""``sim/isaaclab`` ``THUMB_PAD_MM`` width."""
NORMAL_WEIGHT_MM = 20.0
FC_WEIGHT_MM = (10.0, 2.0)
"""The force-closure surrogate's weight in the search's two phases (steer, then polish feasibility)."""
ITERATIONS = (50, 30)
SEEDS = 48
PALM_STEP_MM = 5.0
FD_STEP = 1e-4

# variable layout: thumb (4) | index, middle, ring (3 each) | object centre (3) | rotation vector (3) | pads (2 each)
Q, C, W, PADS = slice(0, 13), slice(13, 16), slice(16, 19), 19
FINGER_Q = {d: slice(4 + 3 * i, 7 + 3 * i) for i, d in enumerate(FINGERS)}


# ---------------------------------------------------------------- objects


@dataclass(frozen=True)
class Shape:
    """A convex object: ``sphere`` (diameter ``size_mm``), ``cylinder`` (diameter ``size_mm``, ``length_mm``, axis
    local z, flat caps) or ``box`` (a cube of side ``size_mm``). Centred at its local origin."""

    kind: str
    size_mm: float
    length_mm: float = 0.0
    name: str = ""
    core: bool = True

    @property
    def label(self) -> str:
        return self.name or f"{self.kind}{self.size_mm:g}"

    @property
    def rho(self) -> float:
        """The largest centroid-to-surface distance (the torque scale)."""
        r = self.size_mm / 2.0
        if self.kind == "sphere":
            return r
        if self.kind == "cylinder":
            return math.hypot(r, self.length_mm / 2.0)
        return r * math.sqrt(3.0)

    def closest(self, x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Local points ``x`` (..., 3): signed distance, the nearest surface point and its outward normal."""
        r = self.size_mm / 2.0
        if self.kind == "sphere":
            d = np.linalg.norm(x, axis=-1)
            n = np.where(d[..., None] > 1e-9, x / np.maximum(d, 1e-9)[..., None], np.array([0.0, 0.0, 1.0]))
            return d - r, r * n, n
        if self.kind == "box":
            return _box_closest(x, np.full(3, r))
        rho = np.linalg.norm(x[..., :2], axis=-1)
        e = np.where(rho[..., None] > 1e-9, x[..., :2] / np.maximum(rho, 1e-9)[..., None], np.array([1.0, 0.0]))
        d2, s2, n2 = _box_closest(np.stack((rho, x[..., 2]), axis=-1), np.array([r, self.length_mm / 2.0]),
                                  nonneg_first=True)
        sp = np.concatenate((s2[..., :1] * e, s2[..., 1:]), axis=-1)
        n = np.concatenate((n2[..., :1] * e, n2[..., 1:]), axis=-1)
        return d2, sp, n

    def sdf(self, x: np.ndarray) -> np.ndarray:
        return self.closest(x)[0]

    def edge_cloud(self, step: float = PALM_STEP_MM) -> np.ndarray:
        """Points on the object's edges (box edges, cylinder rims) for the object-vs-palm-box test."""
        r = self.size_mm / 2.0
        if self.kind == "box":
            t = np.linspace(-r, r, max(int(math.ceil(2 * r / step)) + 1, 2))
            pts = []
            for axis in range(3):
                for a in (-r, r):
                    for b in (-r, r):
                        p = np.zeros((len(t), 3))
                        p[:, axis] = t
                        p[:, (axis + 1) % 3], p[:, (axis + 2) % 3] = a, b
                        pts.append(p)
            return np.concatenate(pts)
        if self.kind == "cylinder":
            n = max(int(math.ceil(2 * math.pi * r / step)), 8)
            ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
            ring = np.stack((r * np.cos(ang), r * np.sin(ang)), axis=1)
            h = self.length_mm / 2.0
            return np.concatenate([np.hstack((ring, np.full((n, 1), z))) for z in (-h, h)])
        return np.zeros((0, 3))

    def seed_modes(self) -> list[tuple[float, str, float]]:
        """(the pad-to-centre distance, the mode, the share of the seeds)."""
        r = self.size_mm / 2.0
        if self.kind == "cylinder":
            return [(r, "side", 2 / 3), (self.length_mm / 2.0, "cap", 1 / 3)]
        return [(r, "side", 1.0)]


def _box_closest(x: np.ndarray, half: np.ndarray, nonneg_first: bool = False):
    """An axis-aligned box's signed distance, nearest surface point and outward normal. ``nonneg_first``: the first
    coordinate is a radius (>= 0; the cylinder's 2-D section), so its "negative face" does not exist."""
    q = np.abs(x) - half
    if nonneg_first:
        q = q.copy()
        q[..., 0] = x[..., 0] - half[0]
    outside = np.maximum(q, 0.0)
    d_out = np.linalg.norm(outside, axis=-1)
    sp_out = np.clip(x, -half, half)
    if nonneg_first:
        sp_out[..., 0] = np.minimum(x[..., 0], half[0])
    n_out = (x - sp_out) / np.maximum(d_out, 1e-12)[..., None]
    k = np.argmax(q, axis=-1)
    sign = np.where(np.take_along_axis(x, k[..., None], -1) >= 0.0, 1.0, -1.0)
    onehot = np.eye(x.shape[-1])[k]
    n_in = onehot * sign
    sp_in = x * (1.0 - onehot) + onehot * sign * half
    use_out = (d_out > 1e-9)[..., None]
    sdf = d_out + np.minimum(q.max(axis=-1), 0.0)
    return sdf, np.where(use_out, sp_out, sp_in), np.where(use_out, n_out, n_in)


def object_set(extras: bool = True) -> list[Shape]:
    """The registered set: sphere, cylinder (length 100) and cube at 10, 20, 35, 50, 70 mm; plus a 5 mm rod and a
    50 x 5 mm disc."""
    out = [Shape(kind, s, 100.0 if kind == "cylinder" else 0.0) for kind in ("sphere", "cylinder", "box")
           for s in (10.0, 20.0, 35.0, 50.0, 70.0)]
    if extras:
        out += [Shape("cylinder", 5.0, 100.0, "rod5", False), Shape("cylinder", 50.0, 5.0, "disc50", False)]
    return out


GRASPS: dict[str, tuple[tuple[str, tuple[str, ...]], ...]] = {
    "pinch_index": (("index", ("D",)),),
    "pinch_middle": (("middle", ("D",)),),
    "pinch_ring": (("ring", ("D",)),),
    "tripod": (("index", ("D",)), ("middle", ("D",))),
    "four": tuple((d, ("P", "D")) for d in FINGERS),
}
"""Grasp type -> the finger contacts (digit, allowed pads) besides the thumb pad."""


# ---------------------------------------------------------------- rotations


def rotvec_matrix(w: np.ndarray) -> np.ndarray:
    th = np.linalg.norm(w, axis=-1)
    small = th < 1e-8
    a = np.where(small, 1.0, np.sin(th) / np.where(small, 1.0, th))
    b = np.where(small, 0.5, (1.0 - np.cos(th)) / np.where(small, 1.0, th) ** 2)
    k = np.zeros((*w.shape[:-1], 3, 3))
    k[..., 0, 1], k[..., 0, 2], k[..., 1, 2] = -w[..., 2], w[..., 1], -w[..., 0]
    k[..., 1, 0], k[..., 2, 0], k[..., 2, 1] = w[..., 2], -w[..., 1], w[..., 0]
    return np.eye(3) + a[..., None, None] * k + b[..., None, None] * (k @ k)


def _frame_with_z(z: np.ndarray, hint: np.ndarray) -> np.ndarray:
    """Rotation matrices (N, 3, 3) whose z column is ``z`` and x column is ``hint`` made orthogonal to it."""
    z = z / np.linalg.norm(z, axis=-1, keepdims=True)
    x = hint - np.einsum("ni,ni->n", hint, z)[:, None] * z
    bad = np.linalg.norm(x, axis=-1) < 1e-6
    alt = np.where(np.abs(z[:, :1]) < 0.9, np.array([1.0, 0, 0]), np.array([0, 1.0, 0]))
    x = np.where(bad[:, None], alt - np.einsum("ni,ni->n", alt, z)[:, None] * z, x)
    x /= np.linalg.norm(x, axis=-1, keepdims=True)
    return np.stack((x, np.cross(z, x), z), axis=-1)


# ---------------------------------------------------------------- grasp quality


def _tangents(n: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    a = np.where(np.abs(n[:, :1]) < 0.9, np.array([1.0, 0, 0]), np.array([0, 1.0, 0]))
    t1 = np.cross(n, a)
    t1 /= np.linalg.norm(t1, axis=-1, keepdims=True)
    return t1, np.cross(n, t1)


def primitive_wrenches(points, normals_in, centre, rho: float, mu: float, edges: int = CONE_EDGES) -> np.ndarray:
    """(k * edges, 6): each contact's friction-cone edges (unit normal force) and their torques about ``centre`` / rho."""
    points, n = np.asarray(points, float), np.asarray(normals_in, float)
    n = n / np.linalg.norm(n, axis=-1, keepdims=True)
    t1, t2 = _tangents(n)
    ang = 2 * math.pi * np.arange(edges) / edges
    f = n[:, None] + mu * (np.cos(ang)[None, :, None] * t1[:, None] + np.sin(ang)[None, :, None] * t2[:, None])
    tau = np.cross((points - np.asarray(centre, float))[:, None], f) / rho
    return np.concatenate((f, tau), axis=-1).reshape(-1, 6)


def epsilon(points, normals_in, centre, rho: float, mu: float, edges: int = CONE_EDGES) -> float:
    """Ferrari–Canny epsilon (L1): the distance from the origin to the boundary of the convex hull of the primitive
    wrenches, 0 if the origin is not strictly inside. Two contacts: in the (5-D) subspace their wrenches span (the torque
    about the contact line, which point contacts cannot resist, is left out). More contacts: the full 6-D space."""
    w = primitive_wrenches(points, normals_in, centre, rho, mu, edges)
    k = len(points)
    need = 5 if k == 2 else 6
    _, s, vt = np.linalg.svd(w, full_matrices=False)
    rank = int((s > 1e-9 * s[0]).sum())
    if rank < need:
        return 0.0
    w = w @ vt[:need].T
    try:
        hull = ConvexHull(w)
    except (QhullError, ValueError):
        return 0.0
    off = hull.equations[:, -1]
    return float(-off.max()) if (off < -1e-12).all() else 0.0


def antipodal(points, normals_in, mu: float) -> bool:
    """Nguyen: two contacts are force closure (up to rotation about their line) iff the segment between them lies
    inside both friction cones."""
    p, n = np.asarray(points, float), np.asarray(normals_in, float)
    d = p[1] - p[0]
    length = np.linalg.norm(d)
    if length < 1e-9:
        return False
    d /= length
    cos_lim = math.cos(math.atan(mu))
    n = n / np.linalg.norm(n, axis=-1, keepdims=True)
    return bool(d @ n[0] >= cos_lim and -d @ n[1] >= cos_lim)


# ---------------------------------------------------------------- the hand


def _box_surface(centre: np.ndarray, half: np.ndarray, step: float) -> np.ndarray:
    axes = [np.linspace(-h, h, max(int(math.ceil(2 * h / step)) + 1, 2)) for h in half]
    pts = []
    for k in range(3):
        a, b = [axes[i] for i in range(3) if i != k]
        g = np.stack(np.meshgrid(a, b, indexing="ij"), -1).reshape(-1, 2)
        for s in (-half[k], half[k]):
            p = np.zeros((len(g), 3))
            p[:, k] = s
            p[:, [i for i in range(3) if i != k]] = g
            pts.append(p)
    return np.concatenate(pts) + centre


class GraspHand:
    """A hand (finger morphology ``parameters`` and thumb ``design``) prepared for the grasp search."""

    def __init__(self, parameters: HandParameters, design: ThumbDesign, evaluator: Evaluator | None = None) -> None:
        self.ev = ev = evaluator if evaluator is not None else Evaluator(parameters)
        s = ev._setup(design)
        self.design = design
        self.thumb, self.boxes, self.fingers = s["thumb"], s["boxes"], ev.fingers
        margin = math.radians(FUNCTIONAL_MARGIN_DEG)
        self.q_lo = np.concatenate([ev.thumb_lo + margin] + [ev.finger_lo] * 3)
        self.q_hi = np.concatenate([ev.thumb_hi - margin] + [ev.finger_hi] * 3)
        ok = s["functional_ok"] & s["palm_ok"][False]
        self.thumb_grid = ev.thumb_grid[ok]
        self.thumb_grid_frames = self.thumb.frames(self.thumb_grid)
        self.thumb_reach = self.thumb.pad_reach
        self.thumb_lateral = THUMB_PAD_WIDTH_MM / 2.0 - PAD_EDGE_INSET_MM
        pads = parameters.pads
        self.finger_lateral = pads.width_mm / 2.0 - PAD_EDGE_INSET_MM
        self.box_c = np.array([b["c"] for b in self.boxes])
        self.box_h = np.array([b["h"] for b in self.boxes])
        self.box_names = [b["name"] for b in self.boxes]
        self.box_points = [_box_surface(b["c"], b["h"], PALM_STEP_MM) for b in self.boxes]
        # capsules: thumb (5), index, middle, ring (4 each)
        names = ["thumb_raa_servo", "thumb_metacarpal", "thumb_proximal", "thumb_distal", "thumb_pad"]
        for d in FINGERS:
            names += [f"{d}_fe_servo", f"{d}_pip_servo", f"{d}_proximal", f"{d}_distal"]
        self.capsule_names = names
        pairs = [(i, j) for i in range(5) for j in range(5, 17)]
        for a in range(3):
            for b in range(a + 1, 3):
                ia, ib = 5 + 4 * a, 5 + 4 * b
                pairs += [(ia + i, ib + j) for i in (2, 3) for j in range(4)]
                pairs += [(ib + i, ia + j) for i in (2, 3) for j in (0, 1)]
        self.pairs = np.array(pairs)
        box_pairs = []
        for k, name in enumerate(self.box_names):
            parts = (0, 1, 2, 3, 4) if name in ("palm_hood", "palm_block") else (2, 3, 4)
            box_pairs += [(i, k) for i in parts]
        self.box_pairs = np.array(box_pairs)
        self.labels = ([f"{names[i]}|{names[j]}" for i, j in self.pairs]
                       + [f"{names[i]}|{self.box_names[k]}" for i, k in self.box_pairs]
                       + [f"object|{n}" for n in names] + [f"object|{n}" for n in self.box_names])

    # -- kinematics

    def capsules(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray, dict, dict]:
        tf = self.thumb.frames(q[:, :4])
        ends = [self.thumb.capsules(tf)]
        radii = [self.thumb.radii]
        ff = {}
        for d in FINGERS:
            fk = self.fingers[d]
            ff[d] = fk.frames(q[:, FINGER_Q[d]])
            e, r, _ = fk.capsules(ff[d])
            ends.append(e)
            radii.append(r)
        return np.concatenate(ends, axis=1), np.concatenate(radii), tf, ff

    def thumb_contact(self, tf: dict, along: np.ndarray, lateral: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        e = tf["E"]
        local = np.stack((np.full_like(along, PAD_FACE_MM), -(self.design.pad_along_mm + along), lateral), axis=-1)
        return (e[:, :3, :3] @ local[..., None])[..., 0] + e[:, :3, 3], e[:, :3, 0]

    def finger_contact(self, frames: dict, digit: str, pad: np.ndarray, along: np.ndarray, lateral: np.ndarray):
        """``pad``: 0 proximal, 1 distal (per row)."""
        fk = self.fingers[digit]
        out_p, out_n = [], []
        for key in ("P", "D"):
            f = frames[key]
            local = np.stack((along, np.full_like(along, fk.spans[key][2]), lateral), axis=-1)
            out_p.append((f[:, :3, :3] @ local[..., None])[..., 0] + f[:, :3, 3])
            out_n.append(f[:, :3, 1])
        use_d = (pad == 1)[:, None]
        return np.where(use_d, out_p[1], out_p[0]), np.where(use_d, out_n[1], out_n[0])

    def pad_bounds(self, digit: str, pad: int) -> tuple[tuple[float, float], tuple[float, float]]:
        start, end, _ = self.fingers[digit].spans["PD"[pad]]
        lat = self.finger_lateral
        return (start + PAD_EDGE_INSET_MM, end - PAD_EDGE_INSET_MM), (-lat, lat)

    # -- proximity

    def hand_clearance(self, ends: np.ndarray, radii: np.ndarray) -> np.ndarray:
        """(B, pairs + box pairs) signed distances between hand primitives (the module docstring's pairs)."""
        i, j = self.pairs[:, 0], self.pairs[:, 1]
        sd = segment_distance(ends[:, i, 0], ends[:, i, 1], ends[:, j, 0], ends[:, j, 1]) - radii[i] - radii[j]
        bi, bk = self.box_pairs[:, 0], self.box_pairs[:, 1]
        sb = capsule_box_distance(ends[:, bi, 0], ends[:, bi, 1], radii[bi], self.box_c[bk][:, None], self.box_h[bk][:, None])
        return np.concatenate((sd, sb), axis=1)


# ---------------------------------------------------------------- one problem


@dataclass
class PairResult:
    hand: str
    object: str
    grasp: str
    found: dict
    eps: dict
    best: dict
    seeds: int
    failures: dict
    facing_min_deg: float | None = None

    def as_dict(self) -> dict:
        return dict(self.__dict__)


class GraspProblem:
    """One (hand, object, grasp type) search."""

    def __init__(self, hand: GraspHand, shape: Shape, grasp: str) -> None:
        self.hand, self.shape, self.grasp = hand, shape, grasp
        self.contacts = GRASPS[grasp]
        self.k = 1 + len(self.contacts)
        self.m = PADS + 2 * self.k
        self.edges = shape.edge_cloud()

    # -- geometry of a batch

    def geometry(self, x: np.ndarray, pads: np.ndarray, r0: np.ndarray, palm: bool = True) -> dict:
        h, shape = self.hand, self.shape
        q = x[:, Q]
        ends, radii, tf, ff = h.capsules(q)
        c = x[:, C]
        rot = r0 @ rotvec_matrix(x[:, W])
        pts, nrm = [], []
        p, n = h.thumb_contact(tf, x[:, PADS], x[:, PADS + 1])
        pts.append(p)
        nrm.append(n)
        for i, (digit, _) in enumerate(self.contacts):
            col = PADS + 2 * (i + 1)
            p, n = h.finger_contact(ff[digit], digit, pads[:, i], x[:, col], x[:, col + 1])
            pts.append(p)
            nrm.append(n)
        pts, nrm = np.stack(pts, 1), np.stack(nrm, 1)  # (B, k, 3)
        local = np.einsum("bji,bkj->bki", rot, pts - c[:, None])
        sdf, sp, sn = shape.closest(local)
        surf = np.einsum("bij,bkj->bki", rot, sp) + c[:, None]
        snorm = np.einsum("bij,bkj->bki", rot, sn)
        # object vs hand capsules: 9 points along each
        samples = ends[:, :, None, 0] + (ends[:, :, None, 1] - ends[:, :, None, 0]) * _SAMPLES[:, None]
        loc = np.einsum("bji,bnsj->bnsi", rot, samples - c[:, None, None])
        obj_caps = shape.sdf(loc).min(axis=-1) - radii
        out = {"pts": pts, "pad_n": nrm, "sdf": sdf, "surf": surf, "snorm": snorm, "c": c, "rot": rot,
               "hand_sd": h.hand_clearance(ends, radii), "obj_caps": obj_caps}
        if palm:
            out["obj_palm"] = self.palm_clearance(c, rot)
        return out

    def palm_clearance(self, c: np.ndarray, rot: np.ndarray) -> np.ndarray:
        h, shape = self.hand, self.shape
        if shape.kind == "sphere":
            q = np.abs(c[:, None] - h.box_c) - h.box_h
            d = np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)
            return d - shape.size_mm / 2.0
        cols = []
        edges_w = np.einsum("bij,kj->bki", rot, self.edges) + c[:, None]
        for k, pts in enumerate(h.box_points):
            loc = np.einsum("bji,bkj->bki", rot, pts[None] - c[:, None])
            a = shape.sdf(loc).min(axis=-1)
            q = np.abs(edges_w - h.box_c[k]) - h.box_h[k]
            b = (np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)).min(axis=-1)
            cols.append(np.minimum(a, b))
        return np.stack(cols, axis=1)

    def residual(self, g: dict, fc_weight: float) -> np.ndarray:
        n_in = -g["snorm"]
        rho = self.shape.rho
        force = n_in.sum(axis=1)
        torque = np.cross(g["surf"] - g["c"][:, None], n_in).sum(axis=1) / rho
        parts = [g["sdf"], NORMAL_WEIGHT_MM * (g["pad_n"] + g["snorm"]).reshape(len(n_in), -1),
                 fc_weight * force, fc_weight * torque,
                 np.maximum(-g["hand_sd"], 0.0), np.maximum(-g["obj_caps"], 0.0)]
        if "obj_palm" in g:
            parts.append(np.maximum(-g["obj_palm"], 0.0))
        return np.concatenate(parts, axis=1)

    # -- search

    def seeds(self, n_seeds: int, rng: np.random.Generator):
        """Initial points, per-seed pad choices and seed orientations (pre-registration: geometric matching of the
        object centre each pad pose proposes, ``p + r n``)."""
        h = self.hand
        tf = h.thumb_grid_frames
        n_grid = len(h.thumb_grid)
        alongs = np.array([-1.0, 0.0, 1.0]) * h.thumb_reach
        tp, tn = zip(*[h.thumb_contact(tf, np.full(n_grid, a), np.zeros(n_grid)) for a in alongs])
        tp, tn = np.concatenate(tp), np.concatenate(tn)
        t_idx = np.tile(np.arange(n_grid), 3)
        t_along = np.repeat(alongs, n_grid)
        xs, pad_list, rots = [], [], []
        for r, mode, share in self.shape.seed_modes():
            count = max(int(round(n_seeds * share)), 1)
            vt = tp + r * tn
            total = np.zeros(len(vt))
            nsum = tn.copy()
            match = []
            for digit, allowed in self.contacts:
                grid, frames = h.ev.finger_grid[digit]
                fk = h.fingers[digit]
                props_v, props = [], []
                for pad in allowed:
                    start, end, face = fk.spans[pad]
                    for u in (start + PAD_EDGE_INSET_MM, (start + end) / 2.0, end - PAD_EDGE_INSET_MM):
                        p = _apply(frames[pad], np.array([u, face, 0.0]))
                        n = frames[pad][:, :3, 1]
                        props_v.append(p + r * n)
                        props.append((np.arange(len(grid)), np.full(len(grid), "PD".index(pad)),
                                      np.full(len(grid), u), n, p + r * n))
                v = np.concatenate(props_v)
                gi = np.concatenate([a[0] for a in props])
                padc = np.concatenate([a[1] for a in props])
                ua = np.concatenate([a[2] for a in props])
                nn = np.concatenate([a[3] for a in props])
                dist, j = cKDTree(v).query(vt)
                total += dist
                nsum = nsum + nn[j]
                match.append((grid[gi[j]], padc[j], ua[j], v[j], nn[j]))
            cost = total + 10.0 * np.linalg.norm(nsum, axis=-1) + rng.exponential(2.0, len(total))
            picked, seen = [], set()
            for i in np.argsort(cost):
                if t_idx[i] in seen:
                    continue
                seen.add(t_idx[i])
                picked.append(i)
                if len(picked) == count:
                    break
            picked = np.array(picked)
            x = np.zeros((len(picked), self.m))
            x[:, :4] = h.thumb_grid[t_idx[picked]]
            centres = [vt[picked]]
            x[:, PADS] = t_along[picked]
            pads = np.zeros((len(picked), len(self.contacts)), int)
            for i, (digit, _) in enumerate(self.contacts):
                fq, padc, ua, v, _ = (a[picked] for a in match[i])
                x[:, FINGER_Q[digit]] = fq
                pads[:, i] = padc
                x[:, PADS + 2 * (i + 1)] = ua
                centres.append(v)
            x[:, C] = np.mean(centres, axis=0)
            n_t = tn[picked]
            digit0 = self.contacts[0][0]
            f0 = h.fingers[digit0].frames(x[:, FINGER_Q[digit0]])["D"]
            lateral, along = f0[:, :3, 2], f0[:, :3, 0]
            if self.shape.kind == "sphere":
                rot = np.broadcast_to(np.eye(3), (len(picked), 3, 3)).copy()
            elif self.shape.kind == "box":
                rot = _frame_with_z(n_t, lateral)
            elif mode == "cap":
                rot = _frame_with_z(n_t, lateral)
            else:  # cylinder axis across the contact line: along the finger's width or its length, at random
                axis = np.where(rng.random(len(picked))[:, None] < 0.5, lateral, along)
                axis = axis - np.einsum("ni,ni->n", axis, n_t)[:, None] * n_t
                rot = _frame_with_z(axis, n_t)
            xs.append(x)
            pad_list.append(pads)
            rots.append(rot)
        return np.concatenate(xs), np.concatenate(pad_list), np.concatenate(rots)

    def bounds(self, pads: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        h = self.hand
        n = len(pads)
        lo, hi = np.zeros((n, self.m)), np.zeros((n, self.m))
        lo[:, Q], hi[:, Q] = h.q_lo, h.q_hi
        lo[:, C], hi[:, C] = -400.0, 400.0
        lo[:, W], hi[:, W] = -math.pi, math.pi
        lo[:, PADS], hi[:, PADS] = -h.thumb_reach, h.thumb_reach
        lo[:, PADS + 1], hi[:, PADS + 1] = -h.thumb_lateral, h.thumb_lateral
        for i, (digit, _) in enumerate(self.contacts):
            col = PADS + 2 * (i + 1)
            for pad in (0, 1):
                (a, b), (la, lb) = h.pad_bounds(digit, pad)
                sel = pads[:, i] == pad
                lo[sel, col], hi[sel, col], lo[sel, col + 1], hi[sel, col + 1] = a, b, la, lb
        return lo, hi

    def _residual_and_jacobian(self, x, pads, r0, fc_weight):
        n, m = x.shape
        batch = np.concatenate([x] + [x + FD_STEP * np.eye(m)[i] for i in range(m)])
        g = self.geometry(batch, np.tile(pads, (m + 1, 1)), np.tile(r0, (m + 1, 1, 1)), palm=False)
        r = self.residual(g, fc_weight).reshape(m + 1, n, -1)
        jac = (r[1:] - r[0]) / FD_STEP  # (m, n, M)
        # the palm term depends on the object pose only
        pose = list(range(C.start, W.stop))
        pb = np.concatenate([x] + [x + FD_STEP * np.eye(m)[i] for i in pose])
        rot = np.tile(r0, (len(pose) + 1, 1, 1)) @ rotvec_matrix(pb[:, W])
        rp = np.maximum(-self.palm_clearance(pb[:, C], rot), 0.0).reshape(len(pose) + 1, n, -1)
        jp = np.zeros((m, n, rp.shape[-1]))
        jp[pose] = (rp[1:] - rp[0]) / FD_STEP
        return (np.concatenate((r[0], rp[0]), axis=1), np.concatenate((jac, jp), axis=2).transpose(1, 2, 0))

    def solve(self, x, pads, r0, iterations: int, fc_weight: float) -> np.ndarray:
        lo, hi = self.bounds(pads)
        x = np.clip(x, lo, hi)
        lam = np.full(len(x), 1e-2)
        r, jac = self._residual_and_jacobian(x, pads, r0, fc_weight)
        cost = (r * r).sum(axis=1)
        for _ in range(iterations):
            jt = jac.transpose(0, 2, 1)
            hess = jt @ jac
            grad = (jt @ r[..., None])[..., 0]
            diag = np.einsum("nii->ni", hess) + 1e-6
            a = hess + lam[:, None, None] * (diag[:, :, None] * np.eye(self.m))
            dx = -np.linalg.solve(a, grad[..., None])[..., 0]
            trial = np.clip(x + dx, lo, hi)
            g = self.geometry(trial, pads, r0)
            rt = self.residual(g, fc_weight)
            ct = (rt * rt).sum(axis=1)
            better = ct < cost
            if better.any():
                x = np.where(better[:, None], trial, x)
                cost = np.where(better, ct, cost)
                r_new, j_new = self._residual_and_jacobian(x[better], pads[better], r0[better], fc_weight)
                r[better], jac[better] = r_new, j_new
            lam = np.clip(np.where(better, lam * 0.3, lam * 5.0), 1e-7, 1e7)
        return x

    # -- exact check

    def check(self, x, pads, r0) -> list[dict]:
        g = self.geometry(x, pads, r0)
        sd = np.concatenate((g["hand_sd"], g["obj_caps"], g["obj_palm"]), axis=1)
        sd_min, which = sd.min(axis=1), sd.argmin(axis=1)
        angle = np.degrees(np.arccos(np.clip(np.einsum("bki,bki->bk", g["pad_n"], -g["snorm"]), -1, 1)))
        out = []
        for b in range(len(x)):
            contact = bool(np.all(np.abs(g["sdf"][b]) <= REACH_MM))
            clear = bool(sd_min[b] >= COLLISION_MM)
            row = {"contact": contact, "clear": clear, "max_sdf_mm": float(np.abs(g["sdf"][b]).max()),
                   "max_angle_deg": float(angle[b].max()), "clearance_mm": float(sd_min[b]),
                   "blocker": self.hand.labels[which[b]] if not clear else "", "mu": {}}
            if self.k == 2:
                n_t, n_f = g["pad_n"][b]
                row["facing_deg"] = float(np.degrees(np.arccos(np.clip(-n_t @ n_f, -1, 1))))
            for mu in MUS:
                aligned = bool(angle[b].max() <= math.degrees(math.atan(mu)))
                eps, fc = 0.0, False
                if contact and clear and aligned:
                    pts, n_in = g["surf"][b], -g["snorm"][b]
                    eps = epsilon(pts, n_in, g["c"][b], self.shape.rho, mu)
                    fc = antipodal(pts, n_in, mu) if self.k == 2 else eps > 0.0
                row["mu"][mu] = {"aligned": aligned, "fc": fc, "feasible": contact and clear and aligned and fc,
                                 "eps": eps}
            out.append(row)
        return out

    def run(self, n_seeds: int = SEEDS, rng_seed: int = 0, hand_tag: str = "") -> PairResult:
        rng = np.random.default_rng(rng_seed)
        x, pads, r0 = self.seeds(n_seeds, rng)
        finals = []
        for iters, w in zip(ITERATIONS, FC_WEIGHT_MM):
            x = self.solve(x, pads, r0, iters, w)
            finals.append(x.copy())
        rows, xs = [], []
        for xf in finals:
            rows += self.check(xf, pads, r0)
            xs.append(xf)
        xs = np.concatenate(xs)
        pads2, r02 = np.tile(pads, (len(finals), 1)), np.tile(r0, (len(finals), 1, 1))
        found, eps, best = {}, {}, {}
        for mu in MUS:
            feas = [i for i, r in enumerate(rows) if r["mu"][mu]["feasible"]]
            found[str(mu)] = bool(feas)
            eps[str(mu)] = max((rows[i]["mu"][mu]["eps"] for i in feas), default=0.0)
            if feas:
                i = max(feas, key=lambda i: rows[i]["mu"][mu]["eps"])
                best[str(mu)] = self._describe(xs[i], pads2[i], r02[i], rows[i])
        failures = {"contact": 0, "collision": 0, "alignment": 0, "force_closure": 0}
        for r in rows:  # at mu 0.5: the first failing criterion of each end point
            m = r["mu"][MUS[0]]
            if not r["contact"]:
                failures["contact"] += 1
            elif not r["clear"]:
                failures["collision"] += 1
            elif not m["aligned"]:
                failures["alignment"] += 1
            elif not m["fc"]:
                failures["force_closure"] += 1
        facing = None
        if self.k == 2:
            ok = [r["facing_deg"] for r in rows if r["contact"] and r["clear"]]
            facing = float(min(ok)) if ok else None
        if not found[str(MUS[0])]:
            near = min(rows, key=lambda r: (not r["contact"], not r["clear"], r["max_angle_deg"]))
            failures["closest"] = {k: near[k] for k in ("contact", "clear", "max_sdf_mm", "max_angle_deg",
                                                         "clearance_mm", "blocker")}
        return PairResult(hand_tag, self.shape.label, self.grasp, found, eps, best, len(x), failures, facing)

    def _describe(self, x, pads, r0, row) -> dict:
        rot = r0 @ rotvec_matrix(x[None, W])[0]
        return {"thumb_deg": np.degrees(x[:4]).round(1).tolist(),
                "fingers_deg": {d: np.degrees(x[FINGER_Q[d]]).round(1).tolist() for d in FINGERS},
                "pads": {d: "PD"[int(p)] for (d, _), p in zip(self.contacts, pads)},
                "object_centre_mm": x[C].round(2).tolist(), "object_rotation": rot.round(4).tolist(),
                "pad_places_mm": x[PADS:].round(2).tolist(), "max_sdf_mm": round(row["max_sdf_mm"], 3),
                "max_angle_deg": round(row["max_angle_deg"], 2), "clearance_mm": round(row["clearance_mm"], 3),
                **({"facing_deg": round(row["facing_deg"], 2)} if "facing_deg" in row else {})}


def run_pair(hand: GraspHand, shape: Shape, grasp: str, n_seeds: int = SEEDS, rng_seed: int = 0,
             hand_tag: str = "") -> PairResult:
    return GraspProblem(hand, shape, grasp).run(n_seeds, rng_seed, hand_tag)
