"""Robotic Kapandji test and thumb–finger pinch opposition for a thumb design on a fixed finger morphology.

Everything is in the hand frame H [mm], joint angles in the model's coordinates (``core.morphology.code_hand``),
radians inside the solver. Numpy only: batched forward kinematics, a seeded task-priority damped-least-squares IK and
capsule / box proximity.

**Targets** (thumb pad centre on the target surface point, pad normals opposed):
- per finger (index, middle, ring): ``tip`` (the distal pad's face, 5 mm short of its distal end), ``distal`` (the
  distal pad's face centre), ``proximal`` (the proximal pad's face centre); outward normal: the pad box's palmar +Y;
- ``palm``: the palm block's palmar face (``scripts/export/code_hand_sim.palm_housing``) under the ring finger's base,
  6 mm behind its front edge; normal -Z.
A target is reached when some configuration within the joint limits (thumb, and the target finger's AA, FE and PIP;
the other fingers extended at zero) puts the thumb pad's face centre within 1 mm of the point (touch, as in the
clinical test), with the pad normal within 60 deg of facing the target's (no side or back contact), and nothing
collides (below). Opposition quality is left to the pinch objective.

**Pinch**: thumb pad centre on the finger's distal-pad normal line, 6-20 mm off the face (an object between), thumb
and that finger free; the objective is the smallest angle from facing (``180 - opposition``) over collision-free
solutions with the pad centre within 2 mm of the line.

**Collision model** (signed distance below -1 mm is a collision; the primitives are coarse, see the doc):
- thumb: the RAA servo (capsule along its body, r 9.5), the metacarpal (RAA to MCP centres, r 6), the proximal
  segment (MCP to IP centres, r 9.5: the MCP servo body and the IP servo, whichever way it is turned), the distal
  segment (IP axis to 9.5 mm short of the end effector, r 9.5: servo and fingertip case) and the pad (its face's
  centre line, 1.5 mm in from the face, r 1.5);
- each finger: the MCP-FE servo (capsule along its body, r 9.5), the PIP servo (same), the proximal and distal
  segments as capsules of the pad width (r 9.5) whose palmar side is the pad face, over each pad's length;
- palm: the housing's four boxes (hood, block, bridge, heel), unrounded.
Pairs: every thumb primitive against the other fingers; the thumb (not its pad) against the target finger; the
thumb's proximal, distal segments and pad against all palm boxes (the pad not for the palm target), its RAA servo and
metacarpal against the hood and block (the heel and bridge are built round the thumb's base); the target finger's
segments against the other fingers.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from core.hardware.xm335 import XM335Actuator
from core.morphology.code_hand import FINGERS, CoDeHand, HandParameters, four_bar
from core.morphology.code_hand.model import THUMB_MCP_MOUNT
from core.morphology.code_hand.parameters import rx, ry, rz, trans
from core.optimization.thumb_layout.design import (
    PAD_FACE_MM,
    PAD_THICKNESS_MM,
    SERVO_R_MM,
    SERVO_REAR_MM,
    ThumbDesign,
    hand_parameters,
)

_EXPORT = Path(__file__).resolve().parents[3] / "scripts" / "export" / "code_hand_sim.py"
_spec = importlib.util.spec_from_file_location("_code_hand_sim_for_thumb_layout", _EXPORT)
code_hand_sim = sys.modules.setdefault(_spec.name, importlib.util.module_from_spec(_spec))
if not hasattr(code_hand_sim, "LIMITS"):
    _spec.loader.exec_module(code_hand_sim)

LIMITS_DEG = {**code_hand_sim.LIMITS, "pip": (0.0, 120.0)}
"""The exporter's joint limits (``scripts/export/code_hand_sim.py`` ``LIMITS``, which the URDF/USD carry); PIP flexion
0-120 deg (the studies' required flexion; the four-bar table runs from -10)."""
THUMB_JOINTS = ("ro", "raa", "mcp", "ip")
FINGER_JOINTS = ("aa", "fe", "pip")
TARGET_KINDS = ("tip", "distal", "proximal")
TARGETS = tuple(f"{d}/{k}" for d in FINGERS for k in TARGET_KINDS) + ("palm",)
PINCH_FINGERS = ("index", "middle")

REACH_MM, FACING_DEG, COLLISION_MM = 1.0, 60.0, -1.0
"""Kapandji reach: touch (pad centre within 1 mm), collision-free, within the limits; the pad normal only within 60 deg of
facing (no side or back contact). Opposition quality is the pinch objective's. (Until 2026-11-25 the facing tolerance
was 25 deg; docs/experimental/thumb_layout_opt.md, decision log.)"""
PINCH_GAP_MM, PINCH_LINE_MM = (6.0, 20.0), 2.0
TIP_INSET_MM = 5.0
PALM_INSET_MM = 6.0
FINGER_R_MM = 9.5
"""Half the pad width (19 mm): the finger segments' capsule radius."""
METACARPAL_R_MM = 6.0
PAD_R_MM = PAD_THICKNESS_MM / 2.0
NORMAL_WEIGHT_MM = 30.0
"""Seed matching for the pinch: a unit of normal mismatch counts as this many millimetres."""
TOUCH_WEIGHT_MM = 10.0
"""The same for the Kapandji touch targets (facing matters less there)."""

GRID_STEP_DEG = 15.0
SEEDS = 16
ITERATIONS = 40


# ---------------------------------------------------------------- batched transforms


def _rz_batch(theta: np.ndarray) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    m = np.zeros((*theta.shape, 4, 4))
    m[..., 0, 0], m[..., 0, 1], m[..., 1, 0], m[..., 1, 1] = c, -s, s, c
    m[..., 2, 2] = m[..., 3, 3] = 1.0
    return m


def _mm(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return a @ b


def _apply(frame: np.ndarray, point) -> np.ndarray:
    point = np.asarray(point, float)
    return frame[..., :3, :3] @ point + frame[..., :3, 3]


# ---------------------------------------------------------------- proximity


def segment_distance(p0, p1, q0, q1) -> np.ndarray:
    """Closest distance between segments ``p0-p1`` and ``q0-q1`` (broadcast over leading axes)."""
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a = np.einsum("...i,...i", d1, d1)
    e = np.einsum("...i,...i", d2, d2)
    f = np.einsum("...i,...i", d2, r)
    c = np.einsum("...i,...i", d1, r)
    b = np.einsum("...i,...i", d1, d2)
    eps = 1e-9
    denom = a * e - b * b
    s = np.where(denom > eps, np.clip((b * f - c * e) / np.maximum(denom, eps), 0.0, 1.0), 0.0)
    t = np.where(e > eps, (b * s + f) / np.maximum(e, eps), 0.0)
    s = np.where(t < 0.0, np.where(a > eps, np.clip(-c / np.maximum(a, eps), 0.0, 1.0), 0.0), s)
    s = np.where(t > 1.0, np.where(a > eps, np.clip((b - c) / np.maximum(a, eps), 0.0, 1.0), 0.0), s)
    t = np.clip(t, 0.0, 1.0)
    diff = (p0 + d1 * s[..., None]) - (q0 + d2 * t[..., None])
    return np.linalg.norm(diff, axis=-1)


_SAMPLES = np.linspace(0.0, 1.0, 9)


def capsule_box_distance(p0, p1, radius, centre, half) -> np.ndarray:
    """Signed distance from a capsule to an axis-aligned box (H), from 9 points along the capsule's segment."""
    pts = p0[..., None, :] + (p1 - p0)[..., None, :] * _SAMPLES[:, None]
    q = np.abs(pts - centre) - half
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=-1)
    inside = np.minimum(q.max(axis=-1), 0.0)
    return (outside + inside).min(axis=-1) - radius


# ---------------------------------------------------------------- kinematics


class FingerKinematics:
    """One finger's chain (``model.CoDeHand.finger_frames``), batched, the PIP four-bar from a 0.25 deg table."""

    def __init__(self, parameters: HandParameters, digit: str) -> None:
        p, pad = parameters.finger, parameters.pads
        if pad is None:
            raise ValueError("the finger morphology needs pads")
        self.digit = digit
        self.base = parameters.layout.base(digit).matrix() @ rz(90.0)
        self.fe_offset = trans(p.fe_axis_x_mm, 0.0, p.fe_axis_z_mm) @ rx(-90.0)
        self.pip_servo = trans(p.ap_x_mm, -p.ap_z_mm, 0.0) @ rz(p.pip_servo_tilt_deg - 90.0)
        self.table_deg = np.arange(LIMITS_DEG["pip"][0] - 1.0, LIMITS_DEG["pip"][1] + 1.0 + 1e-9, 0.25)
        b = np.array([four_bar(p, float(q)).b for q in self.table_deg])
        self.bx, self.bz = b[:, 0], b[:, 1]
        mount_p, mount_d = pad.mount_offsets_mm
        face_p, face_d = mount_p + pad.thickness_mm, mount_d + pad.thickness_mm
        self.face = (face_p, face_d)
        self.targets_local = {  # (frame, point in it); the normal is the frame's +Y
            "proximal": ("P", np.array([(pad.proximal_start_mm + pad.proximal_end_mm) / 2.0, face_p, 0.0])),
            "distal": ("D", np.array([(pad.distal_start_mm + pad.distal_end_mm) / 2.0, face_d, 0.0])),
            "tip": ("D", np.array([pad.distal_end_mm - TIP_INSET_MM, face_d, 0.0])),
        }
        inset = FINGER_R_MM / 2.0
        self.segments_local = {
            "proximal": ("P", np.array([pad.proximal_start_mm + inset, face_p - FINGER_R_MM, 0.0]),
                         np.array([pad.proximal_end_mm - inset, face_p - FINGER_R_MM, 0.0])),
            "distal": ("D", np.array([pad.distal_start_mm + inset, face_d - FINGER_R_MM, 0.0]),
                       np.array([pad.distal_end_mm - inset, face_d - FINGER_R_MM, 0.0])),
        }

    def frames(self, q: np.ndarray) -> dict[str, np.ndarray]:
        """``q`` (N, 3) [rad]: AA, FE, PIP. Returns the FE servo, proximal (P), PIP servo and distal (D) frames."""
        aa, fe, pip = q[:, 0], q[:, 1], q[:, 2]
        fe_servo = self.base @ _rz_batch(aa) @ self.fe_offset
        prox = fe_servo @ _rz_batch(fe)
        deg = np.degrees(pip)
        bx, bz = np.interp(deg, self.table_deg, self.bx), np.interp(deg, self.table_deg, self.bz)
        t = np.zeros((len(q), 4, 4))
        t[:, :3, :3] = np.eye(3)
        t[:, 3, 3] = 1.0
        t[:, 0, 3], t[:, 1, 3] = bx, -bz
        dist = prox @ t @ _rz_batch(pip)
        return {"fe": fe_servo, "P": prox, "pip": prox @ self.pip_servo, "D": dist}

    def target(self, frames: dict, kind: str) -> tuple[np.ndarray, np.ndarray]:
        name, point = self.targets_local[kind]
        frame = frames[name]
        return _apply(frame, point), frame[:, :3, 1]

    def capsules(self, frames: dict) -> tuple[np.ndarray, np.ndarray, list[str]]:
        """(N, 4, 2, 3) segment ends, radii (4,), names: FE servo, PIP servo, proximal, distal."""
        ends, radii, names = [], [], []
        for key in ("fe", "pip"):
            f = frames[key]
            ends.append(np.stack((f[:, :3, 3], _apply(f, (0.0, -(SERVO_REAR_MM - SERVO_R_MM), 0.0))), axis=1))
            radii.append(SERVO_R_MM)
            names.append(f"{self.digit}_{key}_servo")
        for key, (frame, a, b) in self.segments_local.items():
            f = frames[frame]
            ends.append(np.stack((_apply(f, a), _apply(f, b)), axis=1))
            radii.append(FINGER_R_MM)
            names.append(f"{self.digit}_{key}")
        return np.stack(ends, axis=1), np.array(radii), names


class ThumbKinematics:
    """The thumb chain (``model.CoDeHand.thumb_frames``), batched, with the design's pad and primitives."""

    NAMES = ("thumb_raa_servo", "thumb_metacarpal", "thumb_proximal", "thumb_distal", "thumb_pad")

    def __init__(self, parameters: HandParameters, design: ThumbDesign) -> None:
        t = parameters.thumb
        self.design = design
        self.base = parameters.layout.thumb.matrix()
        self.a1 = trans(z=t.ro_to_raa_mm) @ ry(-90.0)
        self.a2 = rz(t.raa_to_mcp_clocking_deg) @ trans(z=t.raa_to_mcp_mm) @ ry(-90.0) @ THUMB_MCP_MOUNT
        self.a3 = trans(y=-t.mcp_to_ip_mm)
        ee = design.ee_mm
        self.pad_point = np.array([PAD_FACE_MM, -design.pad_along_mm, 0.0])
        half = design.pad_length_mm / 2.0
        self.pad_line = (np.array([PAD_FACE_MM - PAD_R_MM, -(design.pad_along_mm - half + PAD_R_MM), 0.0]),
                         np.array([PAD_FACE_MM - PAD_R_MM, -(design.pad_along_mm + half - PAD_R_MM), 0.0]))
        self.distal_end = np.array([0.0, -max(ee - SERVO_R_MM, 0.0), 0.0])
        self.radii = np.array([SERVO_R_MM, METACARPAL_R_MM, SERVO_R_MM, SERVO_R_MM, PAD_R_MM])

    def frames(self, q: np.ndarray) -> dict[str, np.ndarray]:
        ro, raa, mcp, ip = (q[:, i] for i in range(4))
        raa_f = self.base @ _rz_batch(ro) @ self.a1
        mcp_f = raa_f @ _rz_batch(raa) @ self.a2 @ _rz_batch(mcp)
        ip_f = mcp_f @ self.a3
        return {"raa": raa_f, "mcp": mcp_f, "ip": ip_f, "E": ip_f @ _rz_batch(ip)}

    def pad(self, frames: dict) -> tuple[np.ndarray, np.ndarray]:
        e = frames["E"]
        return _apply(e, self.pad_point), e[:, :3, 0]

    def capsules(self, frames: dict) -> np.ndarray:
        raa, mcp, ip, e = frames["raa"], frames["mcp"], frames["ip"], frames["E"]
        o = lambda f: f[:, :3, 3]
        ends = [
            (o(raa), _apply(raa, (0.0, -(SERVO_REAR_MM - SERVO_R_MM), 0.0))),
            (o(raa), o(mcp)),
            (o(mcp), o(ip)),
            (o(e), _apply(e, self.distal_end)),
            (_apply(e, self.pad_line[0]), _apply(e, self.pad_line[1])),
        ]
        return np.stack([np.stack(pair, axis=1) for pair in ends], axis=1)


def _grid(limits: list[tuple[float, float]], step: float) -> np.ndarray:
    axes = [np.radians(np.arange(lo, hi + 1e-9, step)) for lo, hi in limits]
    return np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, len(limits))


def _limits_rad(names) -> tuple[np.ndarray, np.ndarray]:
    lo = np.radians([LIMITS_DEG[n][0] for n in names])
    hi = np.radians([LIMITS_DEG[n][1] for n in names])
    return lo, hi


# ---------------------------------------------------------------- results


@dataclass
class TaskResult:
    name: str
    reached: bool
    position_mm: float
    angle_deg: float
    clearance_mm: float
    blocker: str
    thumb_deg: tuple[float, ...]
    finger_deg: tuple[float, ...] = ()
    gap_mm: float | None = None
    residual: float = 0.0

    def as_dict(self) -> dict:
        return {k: (round(v, 3) if isinstance(v, float) else v) for k, v in self.__dict__.items()}


@dataclass
class Evaluation:
    design: ThumbDesign
    kapandji: dict[str, TaskResult]
    pinch: dict[str, TaskResult]
    envelope: dict[str, float] = field(default_factory=dict)

    @property
    def reached(self) -> int:
        return sum(r.reached for r in self.kapandji.values())

    @property
    def kapandji_score(self) -> float:
        unreached = [r for r in self.kapandji.values() if not r.reached]
        if not unreached:
            return float(len(self.kapandji))
        margin = float(np.mean([1.0 / (1.0 + r.residual) for r in unreached]))
        return self.reached + 0.999 * margin

    @property
    def pinch_deg(self) -> float:
        """The worse of the thumb–index and thumb–middle best facing angles [deg from antiparallel]."""
        return max(r.residual for r in self.pinch.values())

    def summary(self) -> dict:
        return {"design": self.design.__dict__, "kapandji_score": round(self.kapandji_score, 4),
                "reached": self.reached, "pinch_deg": round(self.pinch_deg, 2),
                "kapandji": {k: v.as_dict() for k, v in self.kapandji.items()},
                "pinch": {k: v.as_dict() for k, v in self.pinch.items()}, "envelope": self.envelope}


# ---------------------------------------------------------------- evaluator


class Evaluator:
    """Fixed finger morphology ``parameters``; :meth:`evaluate` scores a :class:`ThumbDesign` on it."""

    def __init__(self, parameters: HandParameters, seeds: int = SEEDS, iterations: int = ITERATIONS,
                 floor_mm: float = 20.0) -> None:
        self.parameters = parameters
        self.floor_mm = floor_mm
        """The palm envelope's floor: this far below the palm's flat palmar face (:meth:`envelope`)."""
        self.seeds, self.iterations = seeds, iterations
        self.fingers = {d: FingerKinematics(parameters, d) for d in FINGERS}
        self._pairs: dict = {}
        self.finger_lo, self.finger_hi = _limits_rad(FINGER_JOINTS)
        self.thumb_lo, self.thumb_hi = _limits_rad(THUMB_JOINTS)
        self.thumb_grid = _grid([LIMITS_DEG[n] for n in THUMB_JOINTS], GRID_STEP_DEG)
        finger_grid = _grid([LIMITS_DEG[n] for n in FINGER_JOINTS], GRID_STEP_DEG)
        zero = np.zeros((1, 3))
        self.rest = {}  # each finger extended: capsules (4, 2, 3), radii, names
        for d, fk in self.fingers.items():
            ends, radii, names = fk.capsules(fk.frames(zero))
            self.rest[d] = (ends[0], radii, names)
        self.finger_grid = {}
        for d, fk in self.fingers.items():
            frames = fk.frames(finger_grid)
            ends, radii, _ = fk.capsules(frames)
            sd = self._finger_vs_rest(d, ends, radii)
            ok = sd >= COLLISION_MM
            targets = {k: fk.target(frames, k) for k in TARGET_KINDS}
            self.finger_grid[d] = (finger_grid[ok], {k: (p[ok], n[ok]) for k, (p, n) in targets.items()})

    # -- proximity helpers

    def _finger_vs_rest(self, digit: str, ends: np.ndarray, radii: np.ndarray) -> np.ndarray:
        """Min signed distance of the moving finger's segments (proximal, distal) to the other fingers at rest."""
        out = np.full(len(ends), np.inf)
        for other in FINGERS:
            if other == digit:
                continue
            o_ends, o_radii, _ = self.rest[other]
            for i in (2, 3):
                d = segment_distance(ends[:, i, None, 0], ends[:, i, None, 1], o_ends[None, :, 0], o_ends[None, :, 1])
                out = np.minimum(out, (d - radii[i] - o_radii[None]).min(axis=1))
        return out

    def _thumb_clearance(self, caps: np.ndarray, radii: np.ndarray, boxes: list[dict], others: list[str],
                         palm_target: bool, finger: tuple[np.ndarray, np.ndarray, list[str]] | None = None,
                         finger_digit: str | None = None) -> tuple[np.ndarray, np.ndarray, list[str]]:
        """The least signed clearance of each pose of a batch (thumb ``caps``, with the target ``finger``'s capsules
        if given), the index of the pair that sets it, and the pair names (module docstring for the pairs)."""
        n_t = caps.shape[1]
        n_f = 0 if finger is None else finger[0].shape[1]
        key = (tuple(others), palm_target, finger_digit, tuple(b["name"] for b in boxes), n_t, n_f)
        if key not in self._pairs:
            names = list(ThumbKinematics.NAMES) + ([] if finger is None else list(finger[2]))
            static, static_r = [], []
            for other in others:
                o_ends, o_radii, o_names = self.rest[other]
                static.append(o_ends)
                static_r.append(o_radii)
                names += o_names
            s0 = n_t + n_f
            n_s = len(names) - s0
            pairs, labels = [], []
            for i in range(n_t):
                for j in range(s0, s0 + n_s):
                    pairs.append((i, j))
                    labels.append(f"{names[i]}|{names[j]}")
            for i in range(min(n_t, 4)):
                for j in range(n_t, s0):
                    pairs.append((i, j))
                    labels.append(f"{names[i]}|{names[j]}")
            for i in range(n_t + 2, s0):  # the target finger's segments against the other fingers
                for j in range(s0, s0 + n_s):
                    pairs.append((i, j))
                    labels.append(f"{names[i]}|{names[j]}")
            box_pairs = []
            for k, box in enumerate(boxes):
                parts = (2, 3) if palm_target else (2, 3, 4)
                if box["name"] in ("palm_hood", "palm_block"):
                    parts = (0, 1, *parts)
                for i in parts:
                    box_pairs.append((i, k))
                    labels.append(f"{names[i]}|{box['name']}")
            static = np.concatenate(static) if static else np.zeros((0, 2, 3))
            static_r = np.concatenate(static_r) if static_r else np.zeros(0)
            pairs = np.array(pairs, int).reshape(-1, 2)
            box_pairs = np.array(box_pairs, int).reshape(-1, 2)
            centres = np.array([boxes[k]["c"] for k in box_pairs[:, 1]]).reshape(-1, 1, 3)
            halves = np.array([boxes[k]["h"] for k in box_pairs[:, 1]]).reshape(-1, 1, 3)
            self._pairs[key] = (static, static_r, pairs, box_pairs, centres, halves, labels)
        static, static_r, pairs, box_pairs, centres, halves, labels = self._pairs[key]
        n = len(caps)
        parts = [caps] + ([] if finger is None else [finger[0]]) + [np.broadcast_to(static, (n, *static.shape))]
        every = np.concatenate(parts, axis=1)
        r = np.concatenate([radii] + ([] if finger is None else [finger[1]]) + [static_r])
        cols = []
        if len(pairs):
            i, j = pairs[:, 0], pairs[:, 1]
            cols.append(segment_distance(every[:, i, 0], every[:, i, 1], every[:, j, 0], every[:, j, 1]) - r[i] - r[j])
        if len(box_pairs):
            i = box_pairs[:, 0]
            cols.append(capsule_box_distance(every[:, i, 0], every[:, i, 1], r[i], centres, halves))
        sd = np.concatenate(cols, axis=1)
        return sd.min(axis=1), sd.argmin(axis=1), labels

    # -- evaluation

    def palm_boxes(self, hand: HandParameters) -> list[dict]:
        housing = code_hand_sim.palm_housing(CoDeHand(hand).frames(), XM335Actuator().geometry)
        return [{"name": b["name"], "c": np.array(b["center"]) * 1e3, "h": np.array(b["half_extents"]) * 1e3}
                for b in housing]

    def evaluate(self, design: ThumbDesign) -> Evaluation:
        hand = hand_parameters(self.parameters, design)
        thumb = ThumbKinematics(hand, design)
        boxes = self.palm_boxes(hand)
        block = next(b for b in boxes if b["name"] == "palm_block")
        ring_y = hand.layout.ring.y_mm
        palm_point = np.array([block["c"][0] + block["h"][0] - PALM_INSET_MM, ring_y, block["c"][2] - block["h"][2]])
        palm_normal = np.array([0.0, 0.0, -1.0])

        # thumb grid, its pads and its clearance to the palm and to each finger at rest
        frames = thumb.frames(self.thumb_grid)
        t_p, t_n = thumb.pad(frames)
        caps = thumb.capsules(frames)
        palm_ok = {}
        for palm_target in (False, True):
            sd, _, _ = self._thumb_clearance(caps, thumb.radii, boxes, [], palm_target)
            palm_ok[palm_target] = sd >= COLLISION_MM
        rest_ok = {d: self._thumb_clearance(caps, thumb.radii, [], [d], False)[0] >= COLLISION_MM for d in FINGERS}

        kapandji, pinch = {}, {}
        for digit in FINGERS:
            others = [d for d in FINGERS if d != digit]
            ok = palm_ok[False] & np.logical_and.reduce([rest_ok[d] for d in others])
            for kind in TARGET_KINDS:
                kapandji[f"{digit}/{kind}"] = self._finger_task(thumb, boxes, digit, kind, ok, t_p, t_n, gaps=None)
            if digit in PINCH_FINGERS:
                pinch[digit] = self._finger_task(thumb, boxes, digit, "distal", ok, t_p, t_n,
                                                 gaps=(7.0, 13.0, 19.0))
        ok = palm_ok[True] & np.logical_and.reduce([rest_ok[d] for d in FINGERS])
        kapandji["palm"] = self._palm_task(thumb, boxes, palm_point, palm_normal, ok, t_p, t_n)
        return Evaluation(design, kapandji, pinch, self.envelope(hand))

    def _match(self, ok: np.ndarray, t_p, t_n, goal_p, goal_n, k: int = 3, weight: float = NORMAL_WEIGHT_MM):
        """Seed pairs: the thumb grid poses (``ok``) whose pad best matches each goal (point, desired thumb normal)."""
        idx = np.flatnonzero(ok)
        if not len(idx) or not len(goal_p):
            return np.zeros(0, int), np.zeros(0, int)
        tree = cKDTree(np.hstack((t_p[idx], weight * t_n[idx])))
        dist, nn = tree.query(np.hstack((goal_p, weight * goal_n)), k=k)
        order = np.argsort(dist, axis=None)
        goal_i, which = np.unravel_index(order, dist.shape)
        thumb_i = idx[nn[goal_i, which]]
        picked, seen_t, seen_g = [], set(), set()
        for g, t in zip(goal_i, thumb_i):
            if t in seen_t or g in seen_g:
                continue
            seen_t.add(t)
            seen_g.add(g)
            picked.append((g, t))
            if len(picked) == self.seeds:
                break
        picked = np.array(picked, int).reshape(-1, 2)
        return picked[:, 0], picked[:, 1]

    def _finger_task(self, thumb: ThumbKinematics, boxes, digit, kind, ok, t_p, t_n, gaps) -> TaskResult:
        fk = self.fingers[digit]
        grid, targets = self.finger_grid[digit]
        p_f, n_f = targets[kind]
        name = f"{digit}/{kind}" if gaps is None else f"pinch/{digit}"
        if gaps is None:
            g_i, t_i = self._match(ok, t_p, t_n, p_f, -n_f, weight=TOUCH_WEIGHT_MM)
            g0 = np.zeros(len(g_i))
        else:
            gp = np.concatenate([p_f + g * n_f for g in gaps])
            gn = np.concatenate([-n_f] * len(gaps))
            g_i, t_i = self._match(ok, t_p, t_n, gp, gn)
            g0 = np.repeat(np.array(gaps), len(p_f))[g_i]
            g_i = g_i % len(p_f)
        if not len(g_i):
            return TaskResult(name, False, math.inf, 180.0, -math.inf, "no seed", (), residual=1e3)
        x0 = np.hstack((self.thumb_grid[t_i], grid[g_i], (g0 / 10.0)[:, None]))
        lo = np.concatenate((self.thumb_lo, self.finger_lo, [PINCH_GAP_MM[0] / 10.0]))
        hi = np.concatenate((self.thumb_hi, self.finger_hi, [PINCH_GAP_MM[1] / 10.0]))
        n_var = 7 if gaps is None else 8

        def residual(x):
            tf = thumb.frames(x[:, :4])
            p_t, n_t = thumb.pad(tf)
            ff = fk.frames(x[:, 4:7])
            p, n = fk.target(ff, kind)
            goal = p + (x[:, 7:8] * 10.0) * n if gaps is not None else p
            return p_t - goal, n_t + n

        others = [d for d in FINGERS if d != digit]

        def clearance(x, detail=False):
            out = self._thumb_clearance(thumb.capsules(thumb.frames(x[:, :4])), thumb.radii, boxes, others, False,
                                        finger=fk.capsules(fk.frames(x[:, 4:7])), finger_digit=digit)
            return out if detail else out[0]

        x = _solve(residual, x0[:, :n_var], lo[:n_var], hi[:n_var], self.iterations, clearance)
        e_p, _ = residual(x)
        tf = thumb.frames(x[:, :4])
        ff = fk.frames(x[:, 4:7])
        sd, which, names = clearance(x, detail=True)
        pos = np.linalg.norm(e_p, axis=1)
        _, n = fk.target(ff, kind)
        _, n_t = thumb.pad(tf)
        angle = np.degrees(np.arccos(np.clip(-np.einsum("ij,ij->i", n, n_t), -1.0, 1.0)))
        if gaps is None:
            ok_pos = pos <= REACH_MM
            reached = ok_pos & (angle <= FACING_DEG) & (sd >= COLLISION_MM)
            res = (np.maximum(pos - REACH_MM, 0.0) / 5.0 + np.maximum(angle - FACING_DEG, 0.0) / FACING_DEG
                   + np.maximum(COLLISION_MM - sd, 0.0) / 5.0)
        else:
            ok_pos = pos <= PINCH_LINE_MM
            reached = ok_pos & (sd >= COLLISION_MM)
            res = angle + 10.0 * np.maximum(pos - PINCH_LINE_MM, 0.0) + 10.0 * np.maximum(COLLISION_MM - sd, 0.0)
        best = int(np.argmin(np.where(reached, -1e9 if gaps is None else res, res)))
        return TaskResult(name, bool(reached[best]), float(pos[best]), float(angle[best]), float(sd[best]),
                          names[which[best]] if sd[best] < COLLISION_MM else "",
                          tuple(np.degrees(x[best, :4]).round(1)), tuple(np.degrees(x[best, 4:7]).round(1)),
                          float(x[best, 7] * 10.0) if gaps is not None else None,
                          0.0 if (gaps is None and reached[best]) else float(res[best]))

    def _palm_task(self, thumb: ThumbKinematics, boxes, point, normal, ok, t_p, t_n) -> TaskResult:
        idx = np.flatnonzero(ok)
        if not len(idx):
            return TaskResult("palm", False, math.inf, 180.0, -math.inf, "no seed", (), residual=1e3)
        tree = cKDTree(np.hstack((t_p[idx], TOUCH_WEIGHT_MM * t_n[idx])))
        _, nn = tree.query(np.hstack((point, -TOUCH_WEIGHT_MM * normal)), k=min(self.seeds, len(idx)))
        x0 = self.thumb_grid[idx[np.atleast_1d(nn)]]

        def residual(x):
            p_t, n_t = thumb.pad(thumb.frames(x))
            return p_t - point, n_t + normal

        def clearance(x, detail=False):
            out = self._thumb_clearance(thumb.capsules(thumb.frames(x)), thumb.radii, boxes, list(FINGERS), True)
            return out if detail else out[0]

        x = _solve(residual, x0, self.thumb_lo, self.thumb_hi, self.iterations, clearance)
        e_p, _ = residual(x)
        tf = thumb.frames(x)
        sd, which, names = clearance(x, detail=True)
        pos = np.linalg.norm(e_p, axis=1)
        _, n_t = thumb.pad(tf)
        angle = np.degrees(np.arccos(np.clip(-(n_t @ normal), -1.0, 1.0)))
        reached = (pos <= REACH_MM) & (angle <= FACING_DEG) & (sd >= COLLISION_MM)
        res = (np.maximum(pos - REACH_MM, 0.0) / 5.0 + np.maximum(angle - FACING_DEG, 0.0) / FACING_DEG
               + np.maximum(COLLISION_MM - sd, 0.0) / 5.0)
        best = int(np.argmin(np.where(reached, -1e9, res)))
        return TaskResult("palm", bool(reached[best]), float(pos[best]), float(angle[best]), float(sd[best]),
                          names[which[best]] if sd[best] < COLLISION_MM else "",
                          tuple(np.degrees(x[best]).round(1)), residual=0.0 if reached[best] else float(res[best]))

    def envelope(self, hand: HandParameters) -> dict[str, float]:
        """The thumb base's palm-envelope margins [mm] (>= 0 inside): the RAA servo, swept about the RO axis over the
        RO range, clear of the finger abduction servos by 2 mm; the RO servo between the hood's top and ``floor_mm``
        below the palm's flat palmar face."""
        geometry = XM335Actuator().geometry
        frames = CoDeHand(hand).frames()
        aa = [code_hand_sim._servo_box(frames.servos[f"{d}/aa"], geometry) for d in FINGERS]
        lo = np.min([a.min(axis=0) for a in aa], axis=0)
        hi = np.max([a.max(axis=0) for a in aa], axis=0)
        raa = _servo_cloud(frames.servos["thumb/raa"], geometry)
        ro_f = frames.servos["thumb/ro"]
        worst = math.inf
        for deg in np.linspace(*LIMITS_DEG["ro"], 37):
            pts = code_hand_sim._rotate(raa, ro_f[:3, 3], ro_f[:3, 2], deg)
            for a in aa:
                a_lo, a_hi = a.min(axis=0), a.max(axis=0)
                c, h = (a_lo + a_hi) / 2, (a_hi - a_lo) / 2
                q = np.abs(pts - c) - h
                worst = min(worst, float((np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)).min()))
        ro = code_hand_sim._servo_box(ro_f, geometry)
        z_top = hi[2] + code_hand_sim.PALM_HOUSING.wall_mm
        z_palm = lo[2] - code_hand_sim.PALM_HOUSING.palmar_depth_mm
        return {"raa_sweep_clearance_mm": round(worst - 2.0, 3), "ro_below_top_mm": round(float(z_top - ro[:, 2].max()), 3),
                "ro_above_floor_mm": round(float(ro[:, 2].min() - (z_palm - self.floor_mm)), 3)}


def _servo_cloud(frame: np.ndarray, geometry, n: int = 6) -> np.ndarray:
    """Points filling a servo's packaging profile (D: the box and its cap) in H."""
    r, rear, hz = geometry.radius_mm, geometry.rear_extent_mm, geometry.depth_mm / 2
    pts = [(x, y, z) for x in np.linspace(-r, r, n) for y in np.linspace(-rear, r, 2 * n) for z in np.linspace(-hz, hz, n)
           if y <= 0 or x * x + y * y <= r * r]
    pts = np.array([(*p, 1.0) for p in pts])
    return (frame @ pts.T).T[:, :3]


def _priority_step(jp, jn, ep, en, lam_p, lam_n) -> np.ndarray:
    m = jp.shape[2]
    a = jp @ jp.transpose(0, 2, 1) + lam_p**2 * np.eye(3)
    jp_pinv = jp.transpose(0, 2, 1) @ np.linalg.inv(a)
    dx1 = -(jp_pinv @ ep[..., None])[..., 0]
    jn_hat = jn @ (np.eye(m) - jp_pinv @ jp)
    b = jn_hat @ jn_hat.transpose(0, 2, 1) + lam_n**2 * np.eye(3)
    rhs = -en - (jn @ dx1[..., None])[..., 0]
    return dx1 + (jn_hat.transpose(0, 2, 1) @ np.linalg.solve(b, rhs[..., None]))[..., 0]


def _solve(residual, x0: np.ndarray, lo: np.ndarray, hi: np.ndarray, iterations: int, clearance=None) -> np.ndarray:
    """Task-priority damped least squares, batched over seeds: the position residual first, the normal one in its
    null space; finite-difference Jacobians; each step clamped to 0.2 (rad) and to the limits. ``clearance(x)``: a
    step (or its half) is taken only if it keeps a clear seed clear (or does not deepen a collision)."""
    x = np.clip(x0.astype(float), lo, hi)
    sd = clearance(x) if clearance is not None else None
    n, m = x.shape
    h = 1e-5
    eye = np.eye(m)
    lam_p, lam_n = 0.5, 0.05
    for _ in range(iterations):
        batch = np.concatenate([x] + [x + h * eye[i] for i in range(m)])
        e_p, e_n = residual(batch)
        e_p, e_n = e_p.reshape(m + 1, n, 3), e_n.reshape(m + 1, n, 3)
        jp = ((e_p[1:] - e_p[0]) / h).transpose(1, 2, 0)  # (n, 3, m)
        jn = ((e_n[1:] - e_n[0]) / h).transpose(1, 2, 0)
        ep, en = e_p[0], e_n[0]
        free = np.ones((n, m))
        for _ in range(3):  # active set: a joint at a limit that the step pushes further out is frozen
            dx = _priority_step(jp * free[:, None], jn * free[:, None], ep, en, lam_p, lam_n)
            stuck = ((x <= lo + 1e-9) & (dx < 0.0)) | ((x >= hi - 1e-9) & (dx > 0.0))
            if not (stuck & (free > 0)).any():
                break
            free = np.where(stuck, 0.0, free)
        norm = np.linalg.norm(dx, axis=1, keepdims=True)
        dx *= np.minimum(1.0, 0.2 / np.maximum(norm, 1e-12))
        if clearance is None:
            x = np.clip(x + dx, lo, hi)
            continue
        for scale in (1.0, 0.5, 0.25):
            trial = np.clip(x + scale * dx, lo, hi)
            sd_t = clearance(trial)
            ok = sd_t >= np.minimum(sd, COLLISION_MM)
            x = np.where(ok[:, None], trial, x)
            sd = np.where(ok, sd_t, sd)
            dx = np.where(ok[:, None], 0.0, dx)
            if ok.all():
                break
    return x
