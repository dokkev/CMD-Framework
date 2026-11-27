"""Robotic Kapandji test (All Hands Up's method) and thumb–finger pinch for a thumb design on a fixed finger morphology.

Everything is in the hand frame H [mm], joint angles in the model's coordinates (``core.morphology.code_hand``),
radians inside the solver. Numpy only: batched forward kinematics, a seeded task-priority damped-least-squares IK and
capsule / box proximity.

**Kapandji levels** (``LANDMARKS``; the clinical K0-K10 adapted to three two-phalanx fingers, the mapping and its
reasons in ``docs/experimental/thumb_layout_opt.md``). As in All Hands Up ("a level counts as reached when the thumb
tip makes contact"): a level is reached when some configuration within the joint limits (thumb, and the landmark
finger's AA, FE and PIP; the other fingers extended at zero) puts a point of the **thumb tip** within 1 mm of the
landmark, with nothing colliding (below). No facing condition. The thumb tip is the fingertip case's rounded end, a
quarter circle of radius 13 mm (the pad face's offset) in the IP servo frame's X-Y plane from the pad face (13 mm off
the centre line, 13 mm short of the end effector) round to the end effector on the centre line; the contact point's
place on it is an IK variable. The score is the clinical one: the highest level whose lower levels are all reached
(-1: none), plus a tie-break margin to the next level.

**Pinch** (reported, no longer optimised): thumb pad centre on the finger's distal-pad normal line, 6-20 mm off the
face, thumb and that finger free; the smallest angle from facing over collision-free solutions with the pad centre
within 2 mm of the line. :meth:`Evaluator.min_grasp_diameter` uses the same solver for the smallest graspable
cylinder.

**Collision model** (signed distance below -1 mm is a collision; the primitives are coarse, see the doc):
- thumb: the RAA servo (capsule along its body, r 9.5), the metacarpal (RAA to MCP centres, r 6), the proximal
  segment (MCP to IP centres, r 9.5: the MCP servo body and the IP servo, whichever way it is turned), the distal
  segment (IP axis to 9.5 mm short of the end effector, r 9.5: servo and fingertip case) and the pad (its face's
  centre line, 1.5 mm in from the face, r 1.5);
- each finger: the MCP-FE servo (capsule along its body, r 9.5), the PIP servo (same), the proximal and distal
  segments as capsules of the pad width (r 9.5) whose palmar side is the pad face, over each pad's length;
- palm: the housing's four boxes (hood, block, bridge, heel), unrounded.
Pairs: every thumb primitive against the other fingers; the thumb (not its pad) against the landmark's finger; the
thumb's proximal, distal segments and pad against all palm boxes (the pad not for a palm landmark), its RAA servo and
metacarpal against the hood and block (the heel and bridge are built round the thumb's base); the moving finger's
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

from core.hardware.xm335 import XM335Actuator, XM335DriveModel
from core.morphology.code_hand import FINGERS, CoDeHand, HandParameters, four_bar, pip_servo_deg
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
PINCH_FINGERS = ("index", "middle")

REACH_MM, COLLISION_MM = 1.0, -1.0
PINCH_GAP_MM, PINCH_LINE_MM = (6.0, 20.0), 2.0
TIP_INSET_MM = 5.0
TIP_RADIUS_MM = PAD_FACE_MM
"""The thumb tip's rounding: a quarter circle from the pad face to the end effector."""
FINGER_R_MM = 9.5
"""Half the pad width (19 mm): the finger segments' capsule radius (also the radial-side landmarks' offset)."""
METACARPAL_R_MM = 6.0
PAD_R_MM = PAD_THICKNESS_MM / 2.0
NORMAL_WEIGHT_MM = 30.0
"""Seed matching for the pinch: a unit of normal mismatch counts as this many millimetres."""
TOUCH_WEIGHT_MM = 10.0
"""The same for the Kapandji contacts (no facing condition; the normal only steers the seeds and the IK's null space)."""

FUNCTIONAL_FACING_DEG, FUNCTIONAL_FORCE_N, FUNCTIONAL_MARGIN_DEG = 45.0, 2.0, 5.0
"""Functional Kapandji: pad facing within 45 deg, 2 N along the contact normal within the servos' stall torques, every
thumb joint 5 deg inside its limits (user, 2026-11-25)."""
PAD_EDGE_INSET_MM = 2.0
"""The functional contact is on the pad face's centre line, this far in from the pad's ends (the face, not its edge)."""
SUPPLY_V = 11.1

GRID_STEP_DEG = 10.0
SEEDS = 16
ITERATIONS = 40


@dataclass(frozen=True)
class Landmark:
    """A Kapandji level's landmark: on finger ``digit``'s proximal (``P``) or distal (``D``) frame, at ``where`` along
    its pad (``"start"``, ``"end"``, ``"centre"`` or a fraction of the pad length from its start; an ``(x, offset)``
    pair of the pad length is relative) on the ``side`` (``"palmar"``: the pad face, normal +Y; ``"radial"``: the finger
    capsule's thumb-side surface, normal +Z, the FE axis, which points to the thumb); or on the palm (``digit`` None)
    at the palm block's palmar face, ``back_mm`` behind its front edge, across the given fingers' mean y."""

    level: int
    name: str
    digit: str | None
    frame: str = "P"
    where: float = 0.5
    side: str = "palmar"
    end_offset_mm: float = 0.0
    back_mm: float = 0.0
    across: tuple[str, ...] = ()


LANDMARKS = (
    Landmark(0, "K0 index proximal phalanx, radial, near MCP", "index", "P", 0.25, "radial"),
    Landmark(1, "K1 index proximal phalanx, radial, near PIP", "index", "P", 0.85, "radial"),
    Landmark(2, "K2 index distal phalanx, radial", "index", "D", 0.5, "radial"),
    Landmark(3, "K3 index tip", "index", "D", 1.0, "palmar", -TIP_INSET_MM),
    Landmark(4, "K4 middle tip", "middle", "D", 1.0, "palmar", -TIP_INSET_MM),
    Landmark(5, "K5 ring tip", "ring", "D", 1.0, "palmar", -TIP_INSET_MM),
    Landmark(6, "K6 ring distal pad", "ring", "D", 0.5, "palmar"),
    Landmark(7, "K7 ring PIP crease", "ring", "P", 1.0, "palmar", -TIP_INSET_MM),
    Landmark(8, "K8 ring proximal phalanx base (MCP crease)", "ring", "P", 0.25, "palmar"),
    Landmark(9, "K9 palm at the ring MCP base", None, back_mm=6.0, across=("ring",)),
    Landmark(10, "K10 distal palmar crease (under index-middle)", None, back_mm=12.0, across=("index", "middle")),
)


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
        servo = np.unwrap(np.radians([pip_servo_deg(p, float(q)) for q in self.table_deg]))
        self.servo_ratio = np.gradient(servo, np.radians(self.table_deg))
        """d theta / d q of the PIP servo against the PIP angle (the four-bar's transmission)."""
        self.bx, self.bz = b[:, 0], b[:, 1]
        mount_p, mount_d = pad.mount_offsets_mm
        face_p, face_d = mount_p + pad.thickness_mm, mount_d + pad.thickness_mm
        self.face = (face_p, face_d)
        self.spans = {"P": (pad.proximal_start_mm, pad.proximal_end_mm, face_p),
                      "D": (pad.distal_start_mm, pad.distal_end_mm, face_d)}
        self.targets_local = {  # (frame, point in it); the normal is the frame's +Y
            "distal": ("D", np.array([(pad.distal_start_mm + pad.distal_end_mm) / 2.0, face_d, 0.0])),
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

    def landmark_local(self, lm) -> tuple[str, np.ndarray, np.ndarray]:
        """A :class:`Landmark` on this finger: its frame key, point and outward normal in that frame."""
        start, end, face = self.spans[lm.frame]
        u = start + lm.where * (end - start) + lm.end_offset_mm
        if lm.side == "radial":
            return lm.frame, np.array([u, face - FINGER_R_MM, FINGER_R_MM]), np.array([0.0, 0.0, 1.0])
        return lm.frame, np.array([u, face, 0.0]), np.array([0.0, 1.0, 0.0])

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
        self.tip_centre = np.array([0.0, -(ee - TIP_RADIUS_MM), 0.0])
        self.pad_reach = max(design.pad_length_mm / 2.0 - PAD_EDGE_INSET_MM, 0.0)
        """The functional contact's travel either side of the pad centre along the segment [mm]."""
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

    def tip(self, frames: dict, phi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """The thumb tip's point at ``phi`` [rad] (0: on the pad face; pi/2: the end effector) and its outward normal."""
        e = frames["E"]
        local_n = np.stack((np.cos(phi), -np.sin(phi), np.zeros_like(phi)), axis=-1)
        local_p = self.tip_centre + TIP_RADIUS_MM * local_n
        return (e[:, :3, :3] @ local_p[..., None])[..., 0] + e[:, :3, 3], (e[:, :3, :3] @ local_n[..., None])[..., 0]

    def pad_face(self, frames: dict, along: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """A point of the pad face's centre line, ``along`` [mm] from its centre toward the tip, and the pad normal."""
        e = frames["E"]
        local = np.stack((np.full_like(along, PAD_FACE_MM), -(self.design.pad_along_mm + along), np.zeros_like(along)),
                         axis=-1)
        return (e[:, :3, :3] @ local[..., None])[..., 0] + e[:, :3, 3], e[:, :3, 0]

    def contact(self, frames: dict, value: np.ndarray, mode: str) -> tuple[np.ndarray, np.ndarray]:
        """``"touch"``: :meth:`tip` at ``value`` [rad]; ``"functional"``: :meth:`pad_face` at ``value`` [mm]."""
        return self.tip(frames, value) if mode == "touch" else self.pad_face(frames, value)

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


def _limits_rad(names, limits=None) -> tuple[np.ndarray, np.ndarray]:
    limits = LIMITS_DEG if limits is None else limits
    lo = np.radians([limits[n][0] for n in names])
    hi = np.radians([limits[n][1] for n in names])
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
    tip_deg: float | None = None
    """Where the contact is: on the tip [deg] (0: pad face, 90: the end), or on the pad [mm from its centre, + tipward]."""
    force_n: float | None = None
    """Functional levels: the largest force along the contact normal within every servo's stall torque."""

    def as_dict(self) -> dict:
        out = {}
        for k, v in self.__dict__.items():
            if isinstance(v, tuple):
                v = tuple(float(x) for x in v)
            out[k] = round(float(v), 3) if isinstance(v, float | np.floating) else v
        return out


def chain_level(levels: dict[int, TaskResult]) -> int:
    """The clinical score: the highest level whose lower levels are all reached (-1: not even K0)."""
    level = -1
    while level + 1 in levels and levels[level + 1].reached:
        level += 1
    return level


def chain_score(levels: dict[int, TaskResult]) -> float:
    """The level plus a tie-break in [0, 1): the margin to the next level, 1 / (1 + its residual)."""
    level = chain_level(levels)
    if level + 1 not in levels:
        return float(level)
    return level + 0.999 / (1.0 + levels[level + 1].residual)


@dataclass
class Evaluation:
    design: ThumbDesign
    levels: dict[int, TaskResult]
    """All Hands Up's touch Kapandji (reported)."""
    pinch: dict[str, TaskResult]
    envelope: dict[str, float] = field(default_factory=dict)
    functional: dict[int, TaskResult] = field(default_factory=dict)
    """The functional Kapandji (optimised)."""

    @property
    def reached(self) -> int:
        """How many of the 11 touch levels are reached (in any order)."""
        return sum(r.reached for r in self.levels.values())

    @property
    def kapandji_level(self) -> int:
        return chain_level(self.levels)

    @property
    def kapandji_score(self) -> float:
        return chain_score(self.levels)

    @property
    def functional_reached(self) -> int:
        return sum(r.reached for r in self.functional.values())

    @property
    def functional_level(self) -> int:
        return chain_level(self.functional)

    @property
    def functional_score(self) -> float:
        return chain_score(self.functional)

    @property
    def pinch_deg(self) -> float:
        """The worse of the thumb–index and thumb–middle best facing angles [deg from antiparallel]."""
        return max(r.residual for r in self.pinch.values()) if self.pinch else math.nan

    def summary(self) -> dict:
        out = {"design": self.design.__dict__}
        if self.functional:
            out |= {"functional_level": self.functional_level, "functional_score": round(self.functional_score, 4),
                    "functional_reached": self.functional_reached}
        if self.levels:
            out |= {"kapandji_level": self.kapandji_level, "kapandji_score": round(self.kapandji_score, 4),
                    "reached": self.reached}
        out |= {"pinch_deg": round(self.pinch_deg, 2),
                "functional": {LANDMARKS[k].name: v.as_dict() for k, v in self.functional.items()},
                "levels": {LANDMARKS[k].name: v.as_dict() for k, v in self.levels.items()},
                "pinch": {k: v.as_dict() for k, v in self.pinch.items()}, "envelope": self.envelope}
        return out


# ---------------------------------------------------------------- evaluator


class Evaluator:
    """Fixed finger morphology ``parameters``; :meth:`evaluate` scores a :class:`ThumbDesign` on it."""

    def __init__(self, parameters: HandParameters, seeds: int = SEEDS, iterations: int = ITERATIONS,
                 floor_mm: float = 20.0, double_support: bool = False,
                 thumb_limits_deg: dict[str, tuple[float, float]] | None = None) -> None:
        self.parameters = parameters
        self.floor_mm = floor_mm
        self.double_support = double_support
        """Include the RAA servo's double-support plates (:data:`RAA_PLATES_MM`) in the RAA sweep (:meth:`envelope`)."""
        """The palm envelope's floor: this far below the palm's flat palmar face (:meth:`envelope`)."""
        self.seeds, self.iterations = seeds, iterations
        self.stall_torque_nm = XM335DriveModel(supply_voltage_v=SUPPLY_V).stall_torque_nm
        self.fingers = {d: FingerKinematics(parameters, d) for d in FINGERS}
        self._pairs: dict = {}
        self.finger_lo, self.finger_hi = _limits_rad(FINGER_JOINTS)
        self.thumb_limits = {**LIMITS_DEG, **(thumb_limits_deg or {})}
        """Thumb joint limits [deg]: the exporter's, with ``thumb_limits_deg`` overriding (e.g. IP hyperextension,
        ``{"ip": (-30.0, 100.0)}``; the sim assets keep the exporter's)."""
        self.thumb_lo, self.thumb_hi = _limits_rad(THUMB_JOINTS, self.thumb_limits)
        self.thumb_grid = _grid([self.thumb_limits[n] for n in THUMB_JOINTS], GRID_STEP_DEG)
        finger_grid = _grid([LIMITS_DEG[n] for n in FINGER_JOINTS], GRID_STEP_DEG)
        zero = np.zeros((1, 3))
        self.rest = {}  # each finger extended: capsules (4, 2, 3), radii, names
        for d, fk in self.fingers.items():
            ends, radii, names = fk.capsules(fk.frames(zero))
            self.rest[d] = (ends[0], radii, names)
        self.finger_grid = {}  # per finger: the collision-free grid poses and their frames
        for d, fk in self.fingers.items():
            frames = fk.frames(finger_grid)
            ends, radii, _ = fk.capsules(frames)
            ok = self._finger_vs_rest(d, ends, radii) >= COLLISION_MM
            self.finger_grid[d] = (finger_grid[ok], {k: v[ok] for k, v in frames.items()})
        # the palm landmarks: fixed by the fingers' layout (the exporter's palm block front and palmar face)
        geometry = XM335Actuator().geometry
        frames = CoDeHand(parameters).frames()
        aa = np.concatenate([code_hand_sim._servo_box(frames.servos[f"{d}/aa"], geometry) for d in FINGERS])
        aa_axes = np.array([frames.servos[f"{d}/aa"][:3, 3] for d in FINGERS])
        housing = code_hand_sim.PALM_HOUSING
        self.palm_front_x = aa_axes[:, 0].min() - geometry.radius_mm - housing.gap_mm
        self.palm_z = aa.min(axis=0)[2] - housing.palmar_depth_mm

    def palm_landmark(self, lm: Landmark) -> tuple[np.ndarray, np.ndarray]:
        y = float(np.mean([self.parameters.layout.base(d).y_mm for d in lm.across]))
        return np.array([self.palm_front_x - lm.back_mm, y, self.palm_z]), np.array([0.0, 0.0, -1.0])

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
        """The least signed clearance of each pose of a batch (thumb ``caps``, with the moving ``finger``'s capsules
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
            for i in range(n_t + 2, s0):  # the moving finger's segments against the other fingers
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
            self._pairs[key] = (static, static_r, pairs, box_pairs, labels)
        static, static_r, pairs, box_pairs, labels = self._pairs[key]
        # the box geometry changes with the design: never cached
        centres = np.array([boxes[k]["c"] for k in box_pairs[:, 1]]).reshape(-1, 1, 3)
        halves = np.array([boxes[k]["h"] for k in box_pairs[:, 1]]).reshape(-1, 1, 3)
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

    def _setup(self, design: ThumbDesign) -> dict:
        """The design's thumb, palm and its grid's clearances (shared by every task of one evaluation)."""
        hand = hand_parameters(self.parameters, design)
        thumb = ThumbKinematics(hand, design)
        boxes = self.palm_boxes(hand)
        frames = thumb.frames(self.thumb_grid)
        caps = thumb.capsules(frames)
        palm_ok = {t: self._thumb_clearance(caps, thumb.radii, boxes, [], t)[0] >= COLLISION_MM for t in (False, True)}
        rest_ok = {d: self._thumb_clearance(caps, thumb.radii, [], [d], False)[0] >= COLLISION_MM for d in FINGERS}
        n = len(self.thumb_grid)
        # the contact sets the seeds are matched on: the tip (touch) at three places, the pad face (functional) at three
        contact = {}
        for mode, values in (("touch", np.radians([0.0, 45.0, 90.0])), ("functional", np.array([-1.0, 0.0, 1.0]))):
            if mode == "functional":
                values = values * thumb.pad_reach
            pts = [thumb.contact(frames, np.full(n, v), mode) for v in values]
            contact[mode] = (np.concatenate([q[0] for q in pts]), np.concatenate([q[1] for q in pts]), values)
        margin = np.radians(FUNCTIONAL_MARGIN_DEG)
        functional_ok = np.all((self.thumb_grid >= self.thumb_lo + margin - 1e-9)
                               & (self.thumb_grid <= self.thumb_hi - margin + 1e-9), axis=1)
        return {"hand": hand, "thumb": thumb, "boxes": boxes, "palm_ok": palm_ok, "rest_ok": rest_ok,
                "pad": thumb.pad(frames), "contact": contact, "functional_ok": functional_ok}

    def evaluate(self, design: ThumbDesign, touch: bool = True, functional: bool = True,
                 pinch: bool = True) -> Evaluation:
        s = self._setup(design)
        levels = {lm.level: self._level(s, lm, "touch") for lm in LANDMARKS} if touch else {}
        functional_levels = {lm.level: self._level(s, lm, "functional") for lm in LANDMARKS} if functional else {}
        pinches = {}
        if pinch:
            for digit in PINCH_FINGERS:
                pinches[digit] = self._pinch(s, digit, PINCH_GAP_MM, (7.0, 13.0, 19.0))
        return Evaluation(design, levels, pinches, self.envelope(s["hand"]), functional_levels)

    def _ok(self, s: dict, others, palm_target: bool) -> np.ndarray:
        return s["palm_ok"][palm_target] & np.logical_and.reduce([s["rest_ok"][d] for d in others])

    def _match(self, ok: np.ndarray, t_p, t_n, goal_p, goal_n, k: int = 3, weight: float = NORMAL_WEIGHT_MM):
        """Seed pairs: the thumb set's entries (``ok``) that best match each goal (point, desired thumb normal), one
        per goal and per thumb entry, best first."""
        idx = np.flatnonzero(ok)
        if not len(idx) or not len(goal_p):
            return np.zeros(0, int), np.zeros(0, int)
        tree = cKDTree(np.hstack((t_p[idx], weight * t_n[idx])))
        dist, nn = tree.query(np.hstack((goal_p, weight * goal_n)), k=min(k, len(idx)))
        dist, nn = dist.reshape(len(goal_p), -1), nn.reshape(len(goal_p), -1)
        order = np.argsort(dist, axis=None)
        goal_i, which = np.unravel_index(order, dist.shape)
        thumb_i = idx[nn[goal_i, which]]
        picked, seen_t, seen_g = [], set(), set()
        for g, t in zip(goal_i, thumb_i):
            if t in seen_t or (g in seen_g and len(goal_p) > 1):
                continue
            seen_t.add(t)
            seen_g.add(g)
            picked.append((g, t))
            if len(picked) == self.seeds:
                break
        picked = np.array(picked, int).reshape(-1, 2)
        return picked[:, 0], picked[:, 1]

    def _level(self, s: dict, lm: Landmark, mode: str = "touch") -> TaskResult:
        """One Kapandji level.

        ``"touch"`` (All Hands Up): the thumb tip (``ThumbKinematics.tip``, the place on it free) within 1 mm of the
        landmark, collision-free, within the limits. ``"functional"``: the thumb pad face (its centre line, 2 mm in
        from its ends; the place free) within 1 mm, its normal within 45 deg of facing the landmark's, every thumb
        joint 5 deg inside its limits, collision-free, and at least 2 N along the contact normal within every servo's
        stall torque, for the thumb and for the landmark's finger (:meth:`_force_capacity`)."""
        thumb, boxes = s["thumb"], s["boxes"]
        functional = mode == "functional"
        n_grid = len(self.thumb_grid)
        c_p, c_n, c_values = s["contact"][mode]
        margin = np.radians(FUNCTIONAL_MARGIN_DEG) if functional else 0.0
        t_lo, t_hi = self.thumb_lo + margin, self.thumb_hi - margin
        c_lo, c_hi = (-thumb.pad_reach, thumb.pad_reach) if functional else (0.0, math.pi / 2)
        weight = NORMAL_WEIGHT_MM if functional else TOUCH_WEIGHT_MM
        name = lm.name if not functional else lm.name + " (functional)"
        unreached = lambda why: TaskResult(name, False, math.inf, math.nan, math.nan, why, (), residual=10.0)
        grid_ok = s["functional_ok"] if functional else np.ones(n_grid, bool)
        if lm.digit is None:
            point, normal = self.palm_landmark(lm)
            on_palm = min(abs(float(capsule_box_distance(point, point, 0.0, b["c"], b["h"]))) for b in boxes)
            if on_palm > REACH_MM:
                return unreached("no palm surface at the landmark")
            ok = np.tile(self._ok(s, FINGERS, True) & grid_ok, 3)
            g_i, t_i = self._match(ok, c_p, c_n, point[None], -normal[None], k=self.seeds, weight=weight)
            if not len(t_i):
                return unreached("no seed")
            x0 = np.hstack((self.thumb_grid[t_i % n_grid], c_values[t_i // n_grid][:, None]))
            lo, hi = np.concatenate((t_lo, [c_lo])), np.concatenate((t_hi, [c_hi]))

            def residual(x):
                p_t, n_t = thumb.contact(thumb.frames(x[:, :4]), x[:, 4], mode)
                return p_t - point, n_t + normal

            def clearance(x, detail=False):
                out = self._thumb_clearance(thumb.capsules(thumb.frames(x[:, :4])), thumb.radii, boxes, list(FINGERS),
                                            True)
                return out if detail else out[0]

            x = _solve(residual, x0, lo, hi, self.iterations, clearance)
            fk, finger_x, place = None, np.zeros((len(x), 0)), x[:, 4]
        else:
            fk = self.fingers[lm.digit]
            grid, gframes = self.finger_grid[lm.digit]
            key, p_loc, n_loc = fk.landmark_local(lm)
            p_f = _apply(gframes[key], p_loc)
            n_f = gframes[key][:, :3, :3] @ n_loc
            others = [d for d in FINGERS if d != lm.digit]
            ok = np.tile(self._ok(s, others, False) & grid_ok, 3)
            g_i, t_i = self._match(ok, c_p, c_n, p_f, -n_f, weight=weight)
            if not len(t_i):
                return unreached("no seed")
            x0 = np.hstack((self.thumb_grid[t_i % n_grid], grid[g_i], c_values[t_i // n_grid][:, None]))
            lo = np.concatenate((t_lo, self.finger_lo, [c_lo]))
            hi = np.concatenate((t_hi, self.finger_hi, [c_hi]))

            def residual(x):
                p_t, n_t = thumb.contact(thumb.frames(x[:, :4]), x[:, 7], mode)
                f = fk.frames(x[:, 4:7])[key]
                return p_t - _apply(f, p_loc), n_t + f[:, :3, :3] @ n_loc

            def clearance(x, detail=False):
                out = self._thumb_clearance(thumb.capsules(thumb.frames(x[:, :4])), thumb.radii, boxes, others, False,
                                            finger=fk.capsules(fk.frames(x[:, 4:7])), finger_digit=lm.digit)
                return out if detail else out[0]

            x = _solve(residual, x0, lo, hi, self.iterations, clearance)
            finger_x, place = x[:, 4:7], x[:, 7]
        e_p, e_n = residual(x)
        sd, which, names = clearance(x, detail=True)
        pos = np.linalg.norm(e_p, axis=1)
        # the angle from facing between the contact's normal and the landmark's
        angle = np.degrees(np.arccos(np.clip(1.0 - 0.5 * np.einsum("ij,ij->i", e_n, e_n), -1.0, 1.0)))
        reached = (pos <= REACH_MM) & (sd >= COLLISION_MM)
        res = np.maximum(pos - REACH_MM, 0.0) / 5.0 + np.maximum(COLLISION_MM - sd, 0.0) / 5.0
        force = np.full(len(x), np.nan)
        if functional:
            force = self._force_capacity(residual, x, e_n, fk, lm)
            reached &= (angle <= FUNCTIONAL_FACING_DEG) & (force >= FUNCTIONAL_FORCE_N)
            res = (res + np.maximum(angle - FUNCTIONAL_FACING_DEG, 0.0) / FUNCTIONAL_FACING_DEG
                   + np.maximum(FUNCTIONAL_FORCE_N - force, 0.0) / FUNCTIONAL_FORCE_N)
        best = int(np.argmin(np.where(reached, -1e9 + pos, res)))
        why = names[which[best]] if sd[best] < COLLISION_MM else ""
        if functional and not reached[best]:
            parts = [why] if why else []
            if pos[best] > REACH_MM:
                parts.append(f"pad {pos[best]:.1f} mm off")
            if angle[best] > FUNCTIONAL_FACING_DEG:
                parts.append(f"facing {angle[best]:.0f} deg")
            if force[best] < FUNCTIONAL_FORCE_N:
                parts.append(f"force {force[best]:.1f} N")
            why = "; ".join(parts)
        return TaskResult(name, bool(reached[best]), float(pos[best]), float(angle[best]), float(sd[best]), why,
                          tuple(np.degrees(x[best, :4]).round(1)), tuple(np.degrees(finger_x[best]).round(1)),
                          residual=0.0 if reached[best] else float(res[best]),
                          tip_deg=float(place[best] if functional else np.degrees(place[best])),
                          force_n=float(force[best]) if functional else None)

    def _force_capacity(self, residual, x: np.ndarray, e_n: np.ndarray, fk, lm: Landmark) -> np.ndarray:
        """The largest force [N] along the contact normal that every servo holds within its stall torque (XM335 at
        the supply voltage, ``core.hardware.xm335``): ``tau = J^T f`` for the thumb's four direct-drive joints and,
        on a finger landmark, the finger's AA and FE (direct drive) and its PIP servo (the joint torque over the
        four-bar's ``d theta / d q``). ``J`` is the contact point's finite-difference Jacobian (mm/rad)."""
        n, m = x.shape
        h = 1e-5
        joints = 4 if fk is None else 7
        batch = np.concatenate([x] + [x + h * np.eye(m)[i] for i in range(joints)])
        e_p, _ = residual(batch)
        e_p = e_p.reshape(joints + 1, n, 3)
        jac = (e_p[1:] - e_p[0]) / h  # (joints, n, 3): thumb columns +dp_t, finger columns -dp_f
        # the contact normal: the landmark's outward normal, the one the thumb pushes against
        normal = -self._landmark_normal(x, fk, lm)
        per_newton = np.abs(np.einsum("jni,ni->jn", jac, normal)) * 1e-3  # N m per N, each joint
        if fk is not None:
            ratio = np.abs(np.interp(np.degrees(x[:, 6]), fk.table_deg, fk.servo_ratio))
            per_newton[6] = per_newton[6] / np.maximum(ratio, 1e-6)
        return self.stall_torque_nm / np.maximum(per_newton.max(axis=0), 1e-9)

    def _landmark_normal(self, x: np.ndarray, fk, lm: Landmark) -> np.ndarray:
        if fk is None:
            return np.broadcast_to(self.palm_landmark(lm)[1], (len(x), 3))
        key, _, n_loc = fk.landmark_local(lm)
        return fk.frames(x[:, 4:7])[key][:, :3, :3] @ n_loc

    def _pinch(self, s: dict, digit: str, gap_bounds: tuple[float, float], gaps) -> TaskResult:
        """The thumb pad centre on ``digit``'s distal-pad normal line at a gap in ``gap_bounds`` [mm]: the smallest
        angle from facing (``residual``; off-line or colliding candidates penalised by 10 deg per mm)."""
        thumb, boxes = s["thumb"], s["boxes"]
        fk = self.fingers[digit]
        grid, gframes = self.finger_grid[digit]
        p_f, n_f = fk.target(gframes, "distal")
        others = [d for d in FINGERS if d != digit]
        ok = self._ok(s, others, False)
        t_p, t_n = s["pad"]
        gp = np.concatenate([p_f + g * n_f for g in gaps])
        gn = np.concatenate([-n_f] * len(gaps))
        g_i, t_i = self._match(ok, t_p, t_n, gp, gn)
        name = f"pinch/{digit}"
        if not len(g_i):
            return TaskResult(name, False, math.inf, 180.0, -math.inf, "no seed", (), residual=1e3)
        g0 = np.repeat(np.array(gaps, float), len(p_f))[g_i]
        x0 = np.hstack((self.thumb_grid[t_i], grid[g_i % len(p_f)], (np.clip(g0, *gap_bounds) / 10.0)[:, None]))
        lo = np.concatenate((self.thumb_lo, self.finger_lo, [gap_bounds[0] / 10.0]))
        hi = np.concatenate((self.thumb_hi, self.finger_hi, [gap_bounds[1] / 10.0]))

        def residual(x):
            p_t, n_t = thumb.pad(thumb.frames(x[:, :4]))
            p, n = fk.target(fk.frames(x[:, 4:7]), "distal")
            return p_t - (p + (x[:, 7:8] * 10.0) * n), n_t + n

        def clearance(x, detail=False):
            out = self._thumb_clearance(thumb.capsules(thumb.frames(x[:, :4])), thumb.radii, boxes, others, False,
                                        finger=fk.capsules(fk.frames(x[:, 4:7])), finger_digit=digit)
            return out if detail else out[0]

        x = _solve(residual, x0, lo, hi, self.iterations, clearance)
        e_p, e_n = residual(x)
        sd, which, names = clearance(x, detail=True)
        pos = np.linalg.norm(e_p, axis=1)
        angle = np.degrees(np.arccos(np.clip(1.0 - 0.5 * np.einsum("ij,ij->i", e_n, e_n), -1.0, 1.0)))
        reached = (pos <= PINCH_LINE_MM) & (sd >= COLLISION_MM)
        res = angle + 10.0 * np.maximum(pos - PINCH_LINE_MM, 0.0) + 10.0 * np.maximum(COLLISION_MM - sd, 0.0)
        best = int(np.argmin(res))
        return TaskResult(name, bool(reached[best]), float(pos[best]), float(angle[best]), float(sd[best]),
                          names[which[best]] if sd[best] < COLLISION_MM else "",
                          tuple(np.degrees(x[best, :4]).round(1)), tuple(np.degrees(x[best, 4:7]).round(1)),
                          float(x[best, 7] * 10.0), float(res[best]))

    def min_grasp_diameter(self, design: ThumbDesign, mu: float = 0.5,
                           diameters=(2, 3, 4, 6, 8, 10, 12, 15, 20, 25, 30, 40, 50, 60)) -> dict:
        """The smallest cylinder [mm] held between the thumb pad and the index or middle distal pad with force closure
        (geometric): the pads ``D`` apart along the finger pad's normal through its centre (within 2 mm), collision-free,
        and the pads within ``2 atan(mu)`` of parallel (two flat contacts on a cylinder: the chord between them lies in
        both friction cones). Per finger, the smallest ``D`` of ``diameters`` that works and the angle there."""
        s = self._setup(design)
        limit = 2.0 * math.degrees(math.atan(mu))
        out = {"mu": mu, "max_angle_deg": round(limit, 1)}
        for digit in PINCH_FINGERS:
            out[digit] = None
            for d in diameters:
                r = self._pinch(s, digit, (float(d), float(d)), (float(d),))
                if r.reached and r.angle_deg <= limit:
                    out[digit] = {"diameter_mm": d, "angle_deg": round(r.angle_deg, 1)}
                    break
        found = [out[d]["diameter_mm"] for d in PINCH_FINGERS if out[d]]
        out["min_diameter_mm"] = min(found) if found else None
        return out

    def envelope(self, hand: HandParameters) -> dict[str, float]:
        """The thumb base's palm-envelope margins [mm] (>= 0 inside): the RAA servo, swept about the RO axis over the
        RO range, clear of the finger abduction servos by 2 mm; the RO servo body clear of them and of the fingers at
        rest by 2 mm; the RO servo between the hood's top and ``floor_mm`` below the palm's flat palmar face."""
        geometry = XM335Actuator().geometry
        frames = CoDeHand(hand).frames()
        aa = [code_hand_sim._servo_box(frames.servos[f"{d}/aa"], geometry) for d in FINGERS]
        lo = np.min([a.min(axis=0) for a in aa], axis=0)
        hi = np.max([a.max(axis=0) for a in aa], axis=0)
        raa = _servo_cloud(frames.servos["thumb/raa"], geometry)
        if self.double_support:
            raa = np.concatenate([raa] + [_box_cloud(frames.servos["thumb/raa"], *b) for b in RAA_PLATES_MM])
        ro_f = frames.servos["thumb/ro"]

        def to_boxes(pts):
            worst = math.inf
            for a in aa:
                a_lo, a_hi = a.min(axis=0), a.max(axis=0)
                c, h = (a_lo + a_hi) / 2, (a_hi - a_lo) / 2
                q = np.abs(pts - c) - h
                worst = min(worst, float((np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)).min()))
            return worst

        sweep = min(to_boxes(code_hand_sim._rotate(raa, ro_f[:3, 3], ro_f[:3, 2], deg))
                    for deg in np.linspace(*self.thumb_limits["ro"], 37))
        ro_pts = _servo_cloud(ro_f, geometry)
        ro_clear = to_boxes(ro_pts)
        for d in FINGERS:
            ends, radii, _ = self.rest[d]
            dist = segment_distance(ro_pts[:, None], ro_pts[:, None], ends[None, :, 0], ends[None, :, 1]) - radii[None]
            ro_clear = min(ro_clear, float(dist.min()))
        ro = code_hand_sim._servo_box(ro_f, geometry)
        z_top = hi[2] + code_hand_sim.PALM_HOUSING.wall_mm
        z_palm = lo[2] - code_hand_sim.PALM_HOUSING.palmar_depth_mm
        return {"raa_sweep_clearance_mm": round(sweep - 2.0, 3), "ro_servo_clearance_mm": round(ro_clear - 2.0, 3),
                "ro_below_top_mm": round(float(z_top - ro[:, 2].max()), 3),
                "ro_above_floor_mm": round(float(ro[:, 2].min() - (z_palm - self.floor_mm)), 3)}


RAA_PLATES_MM = (((-13.0, -25.5, -9.5), (-9.5, 9.5, 9.5)), ((9.5, -25.5, -9.5), (13.0, 9.5, 9.5)))
"""The RAA servo's double-support plates as boxes (low, high corners) in its servo frame [mm]: Onshape `oblique-cmc`
branch, `thumb-example` assembly, `cmc-aa-bracket` (RO horn side) and `cmc-aa-idler-bracket` (far side). Their
STL extents mapped into the evaluator's RAA frame (the CAD X330 part frame shifted 5.5 mm along the output axis;
the CAD RO -> RAA transform matches the evaluator's to 0.000). The plates lie on the servo's side faces, along RO's
axis, 3.5 mm thick; boxes enclose any cut-outs (conservative)."""


def _box_cloud(frame: np.ndarray, low, high, n: int = 6) -> np.ndarray:
    """Points filling a box given in ``frame``'s coordinates, in H."""
    g = [np.linspace(a, b, n) for a, b in zip(low, high)]
    pts = np.array([(x, y, z, 1.0) for x in g[0] for y in g[1] for z in g[2]])
    return (frame @ pts.T).T[:, :3]


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
    null space; finite-difference Jacobians; each step clamped to 0.2 (rad) and to the limits, with an active set at
    the limits. ``clearance(x)``: a step (or its half, quarter) is taken only if it keeps a clear seed clear (or does
    not deepen a collision)."""
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
