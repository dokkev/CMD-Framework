# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Reader and geometry helpers for the ``code_hand_sim/1`` JSON export.

``design/scripts/export/code_hand_sim.py`` writes a simulator-neutral hand description so ``code_isaaclab``
never imports ``design/core``. Nothing in this module imports Isaac Lab, so standalone Newton scripts and unit
tests can load descriptions directly.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

FORMAT = "code_hand_sim/1"
"""Supported export schema identifier."""

DEFAULT_DESCRIPTION_PATH = Path(__file__).resolve().parents[5] / "design" / "output" / "sim" / "code_hand.json"
"""Default output path written by ``design/scripts/export/code_hand_sim.py``."""


@dataclass(frozen=True)
class Description:
    """Parsed ``code_hand_sim/1`` hand description."""

    data: dict[str, Any]

    def __post_init__(self) -> None:
        fmt = self.data.get("format")
        if fmt != FORMAT:
            raise ValueError(f"expected export format {FORMAT!r}, got {fmt!r}")

    @classmethod
    def load(cls, path: str | Path = DEFAULT_DESCRIPTION_PATH) -> Description:
        """Read and validate a ``code_hand_sim/1`` JSON file."""
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    @property
    def source(self) -> dict[str, Any]:
        return self.data["source"]

    @property
    def bodies(self) -> list[dict[str, Any]]:
        return self.data["bodies"]

    @property
    def joints(self) -> list[dict[str, Any]]:
        return self.data["joints"]

    @property
    def loops(self) -> list[dict[str, Any]]:
        return self.data.get("loops", [])

    @property
    def servo(self) -> dict[str, Any]:
        return self.data["servo"]

    @property
    def four_bar(self) -> dict[str, dict[str, list[float]]]:
        return self.data.get("four_bar", {})

    @property
    def filtered_pairs(self) -> list[list[str]]:
        return self.data.get("collision", {}).get("filtered_pairs", [])

    def body(self, name: str) -> dict[str, Any]:
        for body in self.bodies:
            if body["name"] == name:
                return body
        raise KeyError(f"unknown body {name!r}")

    def joint(self, name: str) -> dict[str, Any]:
        for joint in self.joints:
            if joint["name"] == name:
                return joint
        raise KeyError(f"unknown joint {name!r}")

    def actuated(self) -> list[str]:
        """Names of tree joints driven by an actuator, in tree order."""
        return [j["name"] for j in self.joints if j.get("actuator") is not None]

    def passive(self) -> list[str]:
        """Names of passive tree joints (e.g. four-bar pivots ``a`` and ``b``), in tree order."""
        return [j["name"] for j in self.joints if j.get("actuator") is None]

    def four_bar_digits(self) -> tuple[str, ...]:
        """Digits that carry an exported four-bar closure table (empty for open-chain hands)."""
        return tuple(self.four_bar.keys())

    def fingertip_bodies(self) -> list[str]:
        """Distal segment body names for every digit listed in ``tips`` (or ``*_distal`` fallback)."""
        tips = self.data.get("tips")
        if tips:
            return [f"{digit}_distal" for digit in tips]
        return [b["name"] for b in self.bodies if b["name"].endswith("_distal")]

    def pip_from_servo_joint(self, digit: str, ap_deg: float) -> float:
        """Interpolate the distal PIP angle [deg] from the PIP servo tree-joint angle ``ap_deg`` [deg]."""
        table = self.four_bar[digit]
        xp = np.asarray(table["ap_deg"], dtype=float)
        fp = np.asarray(table["pip_deg"], dtype=float)
        if xp.size > 1 and xp[0] > xp[-1]:
            xp, fp = xp[::-1], fp[::-1]
        return float(np.interp(ap_deg, xp, fp))

    def servo_joint_from_pip(self, digit: str, pip_deg: float) -> float:
        """Interpolate the PIP servo tree-joint angle ``ap_deg`` [deg] at ``pip_deg`` [deg]."""
        table = self.four_bar[digit]
        return float(np.interp(pip_deg, table["pip_deg"], table["ap_deg"]))

    def closed_pose(self, digit: str, pip_deg: float) -> dict[str, float]:
        """Tree joint angles [deg] of ``digit``'s four-bar at ``pip_deg``."""
        table = self.four_bar[digit]
        return {
            f"{digit}_a": float(np.interp(pip_deg, table["pip_deg"], table["a_deg"])),
            f"{digit}_pip_servo": float(np.interp(pip_deg, table["pip_deg"], table["ap_deg"])),
            f"{digit}_b": float(np.interp(pip_deg, table["pip_deg"], table["b_deg"])),
        }

    def closed_pose_from_servo(self, digit: str, ap_deg: float) -> dict[str, float]:
        """Tree joint angles [deg] of ``digit``'s four-bar at servo tree-joint angle ``ap_deg`` [deg]."""
        pip_deg = self.pip_from_servo_joint(digit, ap_deg)
        pose = self.closed_pose(digit, pip_deg)
        pose[f"{digit}_pip_servo"] = float(ap_deg)
        return pose


def rod_inertia(body: dict[str, Any]) -> np.ndarray:
    """Placeholder diagonal inertia: slender rod along the body's capsule, isotropic floor ``1e-8 kg m^2``."""
    start, end = np.asarray(body["capsule"]["start"], dtype=float), np.asarray(body["capsule"]["end"], dtype=float)
    length = float(np.linalg.norm(end - start))
    value = max(float(body["mass"]) * length**2 / 12.0, 1e-8)
    return np.eye(3) * value


def quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    """Convert a Warp / Newton quaternion ``(x, y, z, w)`` to a 3x3 rotation matrix."""
    x, y, z, w = (float(v) for v in q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_wxyz_to_matrix(q: np.ndarray) -> np.ndarray:
    """Convert a ``(w, x, y, z)`` quaternion (as stored in export ``boxes[].quat_wxyz``) to a 3x3 matrix."""
    w, x, y, z = (float(v) for v in q)
    return quat_xyzw_to_matrix(np.array([x, y, z, w]))


quat_to_matrix = quat_xyzw_to_matrix
"""Alias for :func:`quat_xyzw_to_matrix` (Warp / Isaac Lab Newton ``body_quat_w`` convention)."""


def rotation_about(r_parent: np.ndarray, r_child: np.ndarray, axis: np.ndarray) -> float:
    """Signed rotation [deg] of ``r_child`` relative to ``r_parent`` about ``axis`` (in parent frame)."""
    rel = r_parent.T @ r_child
    unit = np.asarray(axis, dtype=float)
    unit = unit / np.linalg.norm(unit)
    ref = np.cross(unit, [1.0, 0.0, 0.0])
    if np.linalg.norm(ref) < 1e-6:
        ref = np.cross(unit, [0.0, 1.0, 0.0])
    ref = ref / np.linalg.norm(ref)
    moved = rel @ ref
    return math.degrees(math.atan2(float(unit @ np.cross(ref, moved)), float(ref @ moved)))


__all__ = [
    "DEFAULT_DESCRIPTION_PATH",
    "FORMAT",
    "Description",
    "quat_to_matrix",
    "quat_wxyz_to_matrix",
    "quat_xyzw_to_matrix",
    "rod_inertia",
    "rotation_about",
]
