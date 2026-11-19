"""Cylinder-based contact pad coverage (problem formulation A.3, A.5.1).

Extend the two pads' contact faces to lines; they cross at ``X``. A cylinder
inscribed in the V they open touches both faces at the same distance ``t``
from ``X``. Pad ``i`` covers the distances ``T_i`` its face (less what is
hidden) spans along its open ray, the ray from ``X`` toward its outer end.
Both pads hold a cylinder together at ``T = T_1 & T_2``, and each pad's
available region is ``c_1 = c_2 = |T|``.

A face is a segment ``(2, 2)``, its outer end first (the MCP end of the
proximal pad, the fingertip end of the distal one), in one frame.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from core.geometry.intervals import IntervalSet

_PARALLEL = 1e-9


def common_length_mm(proximal_face: np.ndarray, distal_face: np.ndarray,
                     hidden: Sequence[IntervalSet] = (IntervalSet(), IntervalSet())) -> float:
    """``|T_1 & T_2|`` (mm) for two posed faces.

    ``hidden[i]`` is the part of face ``i`` an occluder hides, as distances
    from its outer end (``core.geometry.intervals.face_occluded_set``). Faces
    in parallel (no crossing) give 0.
    """

    sets = []
    faces = (np.asarray(proximal_face, dtype=float), np.asarray(distal_face, dtype=float))
    rays = []
    for face in faces:
        along = face[0] - face[1]
        length = float(np.linalg.norm(along))
        if length <= 0.0:
            return 0.0
        rays.append(along / length)
    cross = rays[0][0] * rays[1][1] - rays[0][1] * rays[1][0]
    if abs(cross) < _PARALLEL:
        return 0.0
    # X = inner_0 + s u_0 = inner_1 + r u_1
    gap = faces[1][1] - faces[0][1]
    s = (gap[0] * rays[1][1] - gap[1] * rays[1][0]) / cross
    x = faces[0][1] + s * rays[0]
    for face, ray, cut in zip(faces, rays, hidden):
        t_outer, t_inner = float((face[0] - x) @ ray), float((face[1] - x) @ ray)
        span = IntervalSet.closed(max(min(t_outer, t_inner), 0.0), max(t_outer, t_inner))
        cut_t = IntervalSet.from_intervals((t_outer - b, t_outer - a) for a, b in cut.intervals)
        sets.append(span.difference(cut_t))
    return sets[0].intersection(sets[1]).measure


def posed_finger_length_mm(l1_mm: float, l2_mm: float, proximal_end: Sequence[float],
                           distal_start: Sequence[float]) -> float:
    """``L_f(q) = L_1 + ||r_2,start(q) - r_1,end|| + L_2``, both ends in the proximal frame."""

    gap = math.hypot(float(distal_start[0]) - float(proximal_end[0]),
                     float(distal_start[1]) - float(proximal_end[1]))
    return float(l1_mm) + gap + float(l2_mm)


def pad_coverage(common_mm: float, posed_length_mm: float) -> float:
    """``C_pad(M, q) = (c_1 + c_2) / L_f(q) = 2 |T| / L_f(q)``."""

    return 2.0 * float(common_mm) / float(posed_length_mm)


def evaluation_poses_rad(q_req_rad: float) -> tuple[float, ...]:
    """``Q_eval = {90 deg, (90 deg + q_req) / 2, q_req}`` (distinct, increasing; needs ``q_req >= 90 deg``)."""

    low = math.pi / 2.0
    if q_req_rad < low - 1e-12:
        raise ValueError(f"Q_eval needs q_req >= 90 deg, got {math.degrees(q_req_rad):.3f} deg")
    poses = sorted({round(value, 12) for value in (low, 0.5 * (low + q_req_rad), q_req_rad)})
    return tuple(poses)


@dataclass(frozen=True)
class PoseCoverage:
    """Contact pad coverage at one pose of ``Q_eval``."""

    q_rad: float
    common_mm: float
    """``c_1 = c_2 = |T|``."""
    finger_length_mm: float
    """``L_f(q)`` (0 where the pose does not exist)."""

    @property
    def c_pad(self) -> float:
        return pad_coverage(self.common_mm, self.finger_length_mm) if self.finger_length_mm > 0.0 else 0.0


def mean_coverage(poses: Sequence[PoseCoverage]) -> float:
    """``C_pad`` averaged over the poses (0 with none): the objective ``c_pad_mean``."""

    return sum(pose.c_pad for pose in poses) / len(poses) if poses else 0.0


__all__ = ["PoseCoverage", "common_length_mm", "mean_coverage", "evaluation_poses_rad", "pad_coverage", "posed_finger_length_mm"]
