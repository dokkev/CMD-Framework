# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""The hands and the pinch grasp of cube rotation (no Isaac Lab imports; numpy kinematics of the description export).

- the hands: the exports of the reference O and the 7 mm redesigns R (keeps the contact-surface motion) and U (coverage
  alone), as in ``scripts/study3_measurement/common.py``;
- the grasp: every finger's MCP at :data:`GRASP_MCP_DEG` and PIP at :data:`GRASP_PIP_DEG` (the same in q on every hand);
  a 40 mm cube lies with a face on the three tilted proximal pads, its far face :data:`WALL_GAP_M` short of the distal
  pads standing at 90 deg, and the thumb pad presses its top face (:func:`pinch_grasp`);
- the proximal pad-face frame (:func:`face_frame`), in which the policy observes the cube's position.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np

from code_isaaclab.assets.description import Description
from code_isaaclab.hands.code_variants import HAND_ROOT_POS, REPO_ROOT
from code_isaaclab.hands.description_hands import thumb_pad_box

HAND_DIR = Path(os.environ.get("CODE_ROTATE_HAND_DIR", REPO_ROOT / "design" / "output" / "sim" / "study3" / "hands"))
HANDS = {k: HAND_DIR / f"cfl_q120_h3to7_{v}.json" for k, v in (("O", "h0"), ("R", "hr"), ("U", "hu"))}
"""Exports of O, R and U (``design/scripts/export/code_hand_sim.py --params <hand>.yaml``)."""
_TRANSFER_DEFAULT = Path(__file__).resolve().parents[4] / "logs" / "rotate_cube" / "transfer" / "hands"
TRANSFER_HAND_DIR = Path(os.environ.get("CODE_ROTATE_TRANSFER_HAND_DIR", _TRANSFER_DEFAULT))
HANDS.update({
    f"{k}{t}": TRANSFER_HAND_DIR / f"cfl_q120_h3to{t}_{v}.json" for t in (5, 9) for k, v in (("R", "hr"), ("U", "hu"))
})
"""R and U of the 5 and 9 mm pairs (R5, U5, R9, U9), re-exported with the current exporter (servo shapes and visuals,
which the committed ``design/output`` exports of these pairs predate; kinematics identical) by
``scripts/rotate_cube/export_transfer_hands.sh``."""

FINGERS = ("index", "middle", "ring")
THUMB = ("thumb_ro", "thumb_raa", "thumb_mcp", "thumb_ip")
CUBE_SIZE_M = 0.040
CUBE_MASS_KG = 0.05 * (CUBE_SIZE_M / 0.05) ** 3
"""The density of the Study 3 task's 50 mm / 50 g cube (25.6 g). A 50 mm cube does not fit between the MCP knuckles and
the distal pads standing at 90 deg (the proximal pad is 44 mm long)."""
GRASP_MCP_DEG = 35.0
GRASP_PIP_DEG = 90.0
"""The distal pads stand at right angles to the proximal pads, a wall behind the cube's far face, so that PIP flexion
pushes the cube."""
WALL_GAP_M = 0.002
"""The cube's far face this far short of the distal pads' wall (:func:`distal_wall_along`) at rest."""
GRASP_THUMB_DEG = {"thumb_ro": -90.0, "thumb_raa": 20.0}
"""The thumb's fixed joints in the grasp; its MCP and IP are solved per hand by :func:`pinch_grasp` (ro -90, raa 20
reach farthest over the cube)."""
PINCH_SQUEEZE_M = 0.0005
"""The pinch is closed this much below the cube's width (a light preload through the thumb's drives)."""
FLEXION_STIFFNESS, FLEXION_DAMPING = 5.0, 0.08
"""PIP impedance in flexion space, K_q / (dtheta/dq)^2 (:class:`~code_isaaclab.flexion_control.FlexionEMAActionCfg`)."""


def face_frame(desc: Description, mcp_deg: float = GRASP_MCP_DEG) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """The proximal pad-face frame, fixed to the palm: the middle finger's proximal pad face with the MCP at
    ``mcp_deg``; origin at the foot of the MCP axis on that face, x along the face toward the tip, z its outward normal
    (toward the object), y = z x x. Returns ``(origin, rotation)`` in the environment frame, the rotation as 9 numbers,
    row-major, whose columns are the frame's axes."""
    fe = np.array(desc.joint("middle_fe")["anchor"], dtype=float)
    pad = next(b for body in desc.bodies for b in (body.get("boxes") or []) if b["name"] == "middle_proximal_pad")
    face_z = float(pad["center"][2]) - float(pad["half_extents"][1])  # palmar face (hand -z side) at q = 0
    t = math.radians(mcp_deg)  # MCP flexion: a rotation about hand +y (positive toward the palmar side, -z)
    rot_y = np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])
    root = np.diag([1.0, -1.0, -1.0])  # palm up: half a turn about x
    foot = fe + rot_y @ (np.array([fe[0], fe[1], face_z]) - fe)
    origin = np.array(HAND_ROOT_POS) + root @ foot
    ex = root @ rot_y @ np.array([1.0, 0.0, 0.0])
    ez = root @ rot_y @ np.array([0.0, 0.0, -1.0])
    rot = np.stack((ex, np.cross(ez, ex), ez), axis=1)
    return tuple(float(v) for v in origin), tuple(float(v) for v in rot.reshape(-1))


# --- kinematics of the export (hand frame at the geometric zero; joints rotate about their anchors) -------------------


def _rot(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def _joint_tf(desc: Description, name: str, deg: float) -> np.ndarray:
    j = desc.joint(name)
    r, p = _rot(j["axis"], math.radians(deg)), np.asarray(j["anchor"], float)
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r, p - r @ p
    return m


def _quat_wxyz_to_matrix(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])  # fmt: skip


def _matrix_to_quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    w = 0.5 * math.sqrt(max(1e-12, 1.0 + np.trace(r)))
    return tuple(
        float(v)
        for v in ((r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w)
    )


def thumb_pad_pose(desc: Description, deg: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
    """The thumb pad's face centre and outward normal in the hand frame at thumb joint angles ``deg``."""
    m = np.eye(4)
    for name in THUMB:
        m = m @ _joint_tf(desc, name, deg[name])
    box = thumb_pad_box(desc)
    rot = np.array(_quat_wxyz_to_matrix(box["quat_wxyz"]))
    face = np.asarray(box["center"]) + rot[:, 1] * box["half_extents"][1]
    return (m @ np.append(face, 1.0))[:3], m[:3, :3] @ rot[:, 1]


def distal_wall_along(desc: Description, mcp_deg: float = GRASP_MCP_DEG, pip_deg: float = GRASP_PIP_DEG) -> float:
    """Where the middle finger's distal pad face meets the proximal pad face plane at the grasp, along the face from the
    MCP axis's foot [m] (O: 44 mm, the distal pads standing at 90 deg at the end of the proximal pad)."""
    origin, rot9 = face_frame(desc, mcp_deg)
    origin, rot = np.asarray(origin), np.asarray(rot9).reshape(3, 3)
    pose = desc.closed_pose("middle", pip_deg)
    m = (
        _joint_tf(desc, "middle_fe", mcp_deg)
        @ _joint_tf(desc, "middle_pip_servo", pose["middle_pip_servo"])
        @ _joint_tf(desc, "middle_b", pose["middle_b"])
    )
    box = next(b for b in desc.body("middle_distal")["boxes"] if b["name"] == "middle_distal_pad")
    c, h = np.asarray(box["center"]), box["half_extents"]
    flip = np.diag([1.0, -1.0, -1.0])
    face_ends_hand = [(m @ np.array([c[0] + sx * h[0], c[1], c[2] - h[1], 1.0]))[:3] for sx in (-1.0, 1.0)]
    ends = [rot.T @ (np.asarray(HAND_ROOT_POS) + flip @ p - origin) for p in face_ends_hand]
    (x0, _, z0), (x1, _, z1) = ends
    return float(x0 + (x1 - x0) * (0.0 - z0) / (z1 - z0))  # the face line at height 0


def pinch_grasp(desc: Description, mcp_deg: float = GRASP_MCP_DEG) -> dict:
    """The grasp: the face frame (env frame); the cube's rest centre (face frame ``(wall - WALL_GAP_M - size/2, 0,
    size/2)``: a face on the proximal pads, its far face short of the distal pads' wall) and its rest pose in the env
    frame; the thumb's MCP and IP angles (0.25 deg grid) that put the thumb pad's face centre on the cube's top face,
    ``CUBE_SIZE_M - PINCH_SQUEEZE_M`` above the proximal pad face, as close above the cube's centre as the thumb
    reaches, its normal within 20 deg of straight down."""
    origin, rot9 = face_frame(desc, mcp_deg)
    origin, rot = np.asarray(origin), np.asarray(rot9).reshape(3, 3)  # columns: face axes in the env frame
    root = np.diag([1.0, -1.0, -1.0])  # palm up (hand -> env frame)
    height = CUBE_SIZE_M - PINCH_SQUEEZE_M
    rest_along = distal_wall_along(desc, mcp_deg) - WALL_GAP_M - CUBE_SIZE_M / 2
    best = None
    for tm in np.arange(0.0, 100.0, 0.25):
        for ti in np.arange(-10.0, 100.0, 0.25):
            c, n = thumb_pad_pose(desc, {**GRASP_THUMB_DEG, "thumb_mcp": tm, "thumb_ip": ti})
            cf = rot.T @ (np.asarray(HAND_ROOT_POS) + root @ c - origin)
            nf = rot.T @ (root @ n)
            if abs(cf[2] - height) > 2.5e-4 or nf[2] > -math.cos(math.radians(20.0)) or abs(cf[1]) > 0.008:
                continue
            score = abs(cf[0] - rest_along)
            if best is None or score < best[0]:
                best = (score, float(tm), float(ti), cf, nf)
    if best is None or best[0] > CUBE_SIZE_M / 2 - 0.004:
        raise ValueError("the thumb pad cannot press the cube's top face in this grasp")
    _, tm, ti, cf, nf = best
    rest_face = (rest_along, 0.0, CUBE_SIZE_M / 2)
    rest_env = origin + rot @ np.asarray(rest_face)
    return {
        "thumb_mcp_deg": tm,
        "thumb_ip_deg": ti,
        "thumb_pad_face": [float(v) for v in cf],
        "thumb_normal_face": [float(v) for v in nf],
        "face_origin": tuple(float(v) for v in origin),
        "face_rot": tuple(float(v) for v in rot9),
        "rest_face": rest_face,
        "rest_env": tuple(float(v) for v in rest_env),
        "rest_quat_xyzw": _matrix_to_quat_xyzw(rot),
    }


def grasp_joint_pos(desc: Description, thumb_mcp_deg: float, thumb_ip_deg: float) -> dict[str, float]:
    """Every joint at the grasp [rad]: MCP, PIP (four-bar closed from the export's tables), abduction 0, the thumb."""
    pose = {j["name"]: 0.0 for j in desc.joints}  # explicit names: overlapping regex keys are an error
    for f in FINGERS:
        pose[f"{f}_fe"] = math.radians(GRASP_MCP_DEG)
        for joint, deg in desc.closed_pose(f, GRASP_PIP_DEG).items():
            pose[joint] = math.radians(deg)
    for name, deg in {**GRASP_THUMB_DEG, "thumb_mcp": thumb_mcp_deg, "thumb_ip": thumb_ip_deg}.items():
        pose[name] = math.radians(deg)
    return pose


__all__ = [
    "CUBE_MASS_KG",
    "CUBE_SIZE_M",
    "FLEXION_DAMPING",
    "FLEXION_STIFFNESS",
    "GRASP_MCP_DEG",
    "GRASP_PIP_DEG",
    "HANDS",
    "face_frame",
    "grasp_joint_pos",
    "pinch_grasp",
    "thumb_pad_pose",
]
