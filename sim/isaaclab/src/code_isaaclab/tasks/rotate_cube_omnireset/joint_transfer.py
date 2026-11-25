# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Contact-referenced joint maps for running an O-trained policy on another hand H (``docs/experimental/
rl_transfer.md``, condition "mapped"): per actuated joint a monotone 1-D map ``q_H = f_H(q_O)``, so that an O-joint
command puts H's contact surfaces where O's would be. Joint space only; no online inverse kinematics.

- PIP (in flexion): experiment 27's contact setpoint, ``argmin_q |c_H(q) - c_O(q_O)|``, ``c`` the point of the distal
  contact face below the fingertip (the distal pad's distal end, on its face) in the proximal face's frame (origin at
  the foot of the MCP axis on the proximal pad face; ``sim/drake/two_pad_drake.reference_point``), from the
  description's four-bar tables (``Description.closed_pose``);
- MCP flexion: least squares on the proximal pad face's pose in the palm frame (three points along the face's centre
  line: its proximal end, middle and distal end);
- abduction and the thumb: identity (their axes, anchors and the thumb pad are the same on every hand; checked).

Each map is tabulated on a fine grid over O's command range and is linear in between; its inverse (for the
observations) interpolates the same table the other way (the maps are checked to be strictly increasing). Pure numpy.
"""

from __future__ import annotations

import math
from functools import lru_cache

import numpy as np

from code_isaaclab.assets.description import Description

FINGERS = ("index", "middle", "ring")
GRID_DEG = 0.25


def _rot(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def _joint(desc: Description, name: str, deg: float) -> tuple[np.ndarray, np.ndarray]:
    j = desc.joint(name)
    r, p = _rot(j["axis"], math.radians(deg)), np.asarray(j["anchor"], float)
    return r, p - r @ p


def _box(desc: Description, body: str, name: str) -> dict:
    return next(b for b in desc.body(body)["boxes"] if b["name"] == name)


def _box_axes(box: dict) -> np.ndarray:
    w, x, y, z = box["quat_wxyz"]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])  # fmt: skip


def _prox_face_origin(desc: Description, finger: str) -> np.ndarray:
    """The foot of the MCP axis on the proximal pad face, hand frame at the geometric zero (``grasp.face_frame``)."""
    fe = np.asarray(desc.joint(f"{finger}_fe")["anchor"], float)
    pad = _box(desc, f"{finger}_proximal", f"{finger}_proximal_pad")
    face_z = float(pad["center"][2]) - float(pad["half_extents"][1])
    return np.array([fe[0], fe[1], face_z])


def pip_reference_point(desc: Description, finger: str, q_deg: np.ndarray) -> np.ndarray:
    """The distal face point below the fingertip at flexion ``q_deg`` (MCP at 0), in the proximal face frame [m],
    ``(len(q), 3)``."""
    pad = _box(desc, f"{finger}_distal", f"{finger}_distal_pad")
    axes, c, h = _box_axes(pad), np.asarray(pad["center"], float), pad["half_extents"]
    tip = c + axes[:, 0] * h[0] + axes[:, 1] * h[1]  # distal end, on the contact face (+y)
    origin = _prox_face_origin(desc, finger)
    out = []
    for q in np.atleast_1d(q_deg):
        pose = desc.closed_pose(finger, float(q))
        r1, t1 = _joint(desc, f"{finger}_pip_servo", pose[f"{finger}_pip_servo"])
        r2, t2 = _joint(desc, f"{finger}_b", pose[f"{finger}_b"])
        p = r1 @ (r2 @ tip + t2) + t1
        out.append(p - origin)
    return np.array(out)


def proximal_face_points(desc: Description, finger: str, fe_deg: np.ndarray) -> np.ndarray:
    """Three points on the proximal pad face's centre line (proximal end, middle, distal end) at MCP flexion
    ``fe_deg``, hand frame [m], ``(len(fe), 3, 3)``."""
    pad = _box(desc, f"{finger}_proximal", f"{finger}_proximal_pad")
    axes, c, h = _box_axes(pad), np.asarray(pad["center"], float), pad["half_extents"]
    face = c + axes[:, 1] * h[1]
    pts = np.stack([face - axes[:, 0] * h[0], face, face + axes[:, 0] * h[0]])
    out = []
    for q in np.atleast_1d(fe_deg):
        r, t = _joint(desc, f"{finger}_fe", float(q))
        out.append(pts @ r.T + t)
    return np.array(out)


def _argmin_map(target: np.ndarray, candidates: np.ndarray, grid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """For each row of ``target`` the grid value whose candidate is nearest (refined by a parabola), and the
    residual."""
    flat_t = target.reshape(len(target), -1)
    flat_c = candidates.reshape(len(candidates), -1)
    d = np.linalg.norm(flat_t[:, None, :] - flat_c[None, :, :], axis=-1)  # (targets, grid)
    k = d.argmin(axis=1)
    q = grid[k].astype(float).copy()
    inner = (k > 0) & (k < len(grid) - 1)
    i = np.nonzero(inner)[0]
    y0, y1, y2 = d[i, k[i] - 1], d[i, k[i]], d[i, k[i] + 1]
    denom = y0 - 2 * y1 + y2
    shift = np.where(np.abs(denom) > 1e-15, 0.5 * (y0 - y2) / np.where(denom == 0, 1, denom), 0.0)
    q[i] = grid[k[i]] + np.clip(shift, -1, 1) * (grid[1] - grid[0])
    return q, d[np.arange(len(target)), k]


@lru_cache(maxsize=16)
def joint_maps(o_path: str, h_path: str, pip_max_deg: float = 120.0) -> dict:
    """``{joint: {"q_o_deg", "q_h_deg", "residual_mm", "max_dev_deg"}}`` for the actuated joints that are not identity
    (the PIP servos, in flexion, and the MCP flexion joints), plus ``"identity"``: the joints left as they are."""
    o, h = Description.load(o_path), Description.load(h_path)
    out: dict = {}
    pip_grid = np.arange(0.0, pip_max_deg + 1e-9, GRID_DEG)
    for f in FINGERS:
        target = pip_reference_point(o, f, pip_grid)
        cand = pip_reference_point(h, f, pip_grid)
        q, res = _argmin_map(target, cand, pip_grid)
        out[f"{f}_pip_servo"] = {"q_o_deg": pip_grid, "q_h_deg": q, "residual_mm": 1e3 * res}
        lo, hi = (float(v) for v in o.joint(f"{f}_fe")["limits_deg"])
        fe_grid = np.arange(lo, hi + 1e-9, GRID_DEG)
        q, res = _argmin_map(proximal_face_points(o, f, fe_grid), proximal_face_points(h, f, fe_grid), fe_grid)
        out[f"{f}_fe"] = {"q_o_deg": fe_grid, "q_h_deg": q, "residual_mm": 1e3 * res / math.sqrt(3)}
    for name, m in out.items():
        if not np.all(np.diff(m["q_h_deg"]) >= 0):
            raise ValueError(f"{name}: the map from O to {h_path} is not monotone")
        m["max_dev_deg"] = float(np.max(np.abs(m["q_h_deg"] - m["q_o_deg"])))
    same = []
    for j in o.joints:
        name = j["name"]
        if name.endswith(("_aa",)) or name.startswith("thumb_"):
            jh = h.joint(name)
            if not (np.allclose(j["anchor"], jh["anchor"]) and np.allclose(j["axis"], jh["axis"])):
                raise ValueError(f"{name}: anchor or axis differs between O and {h_path}; identity is not justified")
            same.append(name)
    if json_key(o, "thumb_distal") != json_key(h, "thumb_distal"):
        raise ValueError("the thumb's distal segment differs between O and H")
    out["identity"] = same
    return out


def json_key(desc: Description, body: str) -> str:
    import json

    return json.dumps(desc.body(body), sort_keys=True)


__all__ = ["joint_maps", "pip_reference_point", "proximal_face_points"]
