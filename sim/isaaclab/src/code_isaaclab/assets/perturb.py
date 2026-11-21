# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Contact-face perturbations of a ``code_hand_sim/1`` description, for separating where a contact offset sits.

Two perturbations of equal size ``delta`` [m], applied to the ordinary fingers (Study 3,
``report/study3/contact_importance.md``):

- :func:`offset_fingers` (P): every body the MCP flexion carries (proximal segment, both four-bar links, distal
  segment, their pads, joint anchors, loop anchors, fingertip) moves ``delta`` palmar (hand ``-z`` at the zero
  pose), the MCP flexion axis staying where it is. Both contact faces move together relative to the MCP: the
  finger-internal relative contact trajectory is unchanged exactly, the faces' path relative to the palm changes.
- :func:`offset_distal_faces` (D): each distal pad thickens by ``delta`` on its contact side (mounting face fixed),
  so the distal contact face moves ``delta`` along its own normal and the proximal side does not change: a pure
  change of the relative contact trajectory.

Both return a new description (deep copy) with a ``perturbation`` record. Angles (the four-bar tables) and
collision filters are untouched: neither perturbation changes the linkage.
"""

from __future__ import annotations

import copy
from collections.abc import Sequence

import numpy as np

FINGERS = ("index", "middle", "ring")
PALMAR = np.array([0.0, 0.0, -1.0])
"""The ordinary fingers' palmar direction in the hand frame at the zero pose (pads below the link lines)."""


def _shift(point, offset) -> list[float]:
    return [float(v) for v in np.asarray(point, dtype=float) + offset]


def _rotate(quat_wxyz, vector) -> np.ndarray:
    w, x, y, z = (float(v) for v in quat_wxyz)
    q = np.array([x, y, z])
    v = np.asarray(vector, dtype=float)
    t = 2.0 * np.cross(q, v)
    return v + w * t + np.cross(q, t)


def offset_fingers(desc: dict, delta: float, digits: Sequence[str] = FINGERS) -> dict:
    """P: move everything the MCP flexion carries ``delta`` palmar, the MCP axis fixed."""
    out = copy.deepcopy(desc)
    offset = delta * PALMAR
    moved = {f"{d}_{part}" for d in digits for part in ("proximal", "servo_link", "driver_link", "distal")}
    for body in out["bodies"]:
        if body["name"] not in moved:
            continue
        for key in ("origin", "com"):
            body[key] = _shift(body[key], offset)
        body["capsule"]["start"] = _shift(body["capsule"]["start"], offset)
        body["capsule"]["end"] = _shift(body["capsule"]["end"], offset)
        for box in body.get("boxes") or []:
            box["center"] = _shift(box["center"], offset)
    for joint in out["joints"]:
        if joint["child"] in moved and joint["parent"] in moved:  # pip_servo, a, b (not the MCP flexion)
            joint["anchor"] = _shift(joint["anchor"], offset)
    for loop in out.get("loops", []):
        if loop["child"] in moved:
            loop["anchor"] = _shift(loop["anchor"], offset)
    for digit in digits:
        if digit in out.get("tips", {}):
            out["tips"][digit] = _shift(out["tips"][digit], offset)
    out["perturbation"] = {"kind": "P", "delta_m": delta, "digits": list(digits),
                           "what": "bodies carried by the MCP flexion moved palmar; MCP axis fixed"}
    return out


def offset_distal_faces(desc: dict, delta: float, digits: Sequence[str] = FINGERS) -> dict:
    """D: thicken each distal pad by ``delta`` on its contact side (its mounting face fixed)."""
    out = copy.deepcopy(desc)
    names = {f"{d}_distal_pad" for d in digits}
    found = 0
    for body in out["bodies"]:
        for box in body.get("boxes") or []:
            if box["name"] not in names:
                continue
            box["half_extents"][1] = float(box["half_extents"][1]) + delta / 2.0  # local +y is the contact face
            box["center"] = _shift(box["center"], _rotate(box["quat_wxyz"], [0.0, delta / 2.0, 0.0]))
            found += 1
    if found != len(names):
        raise ValueError(f"expected {len(names)} distal pads, found {found}")
    out["perturbation"] = {"kind": "D", "delta_m": delta, "digits": list(digits),
                           "what": "distal pads thickened on the contact side; mounting faces fixed"}
    return out


__all__ = ["FINGERS", "offset_distal_faces", "offset_fingers"]
