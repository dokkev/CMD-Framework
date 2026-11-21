"""The finite rotation pole of a planar motion, and the clearance of two
rectangular pads turned about it (problem formulation A.6.0, Study 0).

Flexion ``q`` turns the distal phalanx toward the palm (clockwise in the
``x``-``z`` plane, ``z`` dorsal): ``y(q) = R_q y0 + b(q)``. For ``q`` in
``(0, 2 pi)`` that displacement is the rotation by ``q`` about one point, the
pole ``P(q)``. The pole is a property of the finite displacement from the
extended pose; it is not the instantaneous centre of the motion, which also
depends on how the pole moves (``y' = R_q' (y0 - P) + (I - R_q) P'``).

- :func:`rotation`, :func:`pole`, :func:`turned`: the motion and its pole;
- :func:`rectangular_clearance`: two rectangles, one turned about a pole,
  exactly (separating axes): apart, touching or overlapping at one pose;
- :func:`symmetric_gap_law`: the closed form for the joint gap, valid only
  for pads of equal depth placed symmetrically about the pole at every pose.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

POLE_MIN_RAD = 1e-6
"""Below this flexion the pole is not computed: ``I - R_q`` loses its inverse as ``q -> 0``."""


def rotation(q_rad: float) -> np.ndarray:
    """``R_q``: the turn by ``q`` toward the palm (clockwise)."""

    c, s = math.cos(q_rad), math.sin(q_rad)
    return np.array([[c, s], [-s, c]])


def pole(q_rad: float, moved: Sequence[float], start: Sequence[float]) -> np.ndarray:
    """The pole of the displacement that turns the body by ``q`` and takes the
    point ``start`` to ``moved``: ``P = (I - R_q)^-1 (moved - R_q start)``,
    in closed form ``(b - cot(q/2) J b) / 2``. NaN for ``q`` below
    :data:`POLE_MIN_RAD` (the extended pose has no pole)."""

    if abs(q_rad) < POLE_MIN_RAD:
        return np.array((math.nan, math.nan))
    b = np.asarray(moved, dtype=float) - rotation(q_rad) @ np.asarray(start, dtype=float)
    cot = 1.0 / math.tan(q_rad / 2.0)
    return 0.5 * np.array((b[0] + cot * b[1], b[1] - cot * b[0]))


def turned(points, about: Sequence[float], q_rad: float) -> np.ndarray:
    """``points`` (``(n, 2)`` or ``(2,)``) turned by ``q`` about the pole ``about``."""

    about = np.asarray(about, dtype=float)
    return (np.asarray(points, dtype=float) - about) @ rotation(q_rad).T + about


def rectangular_clearance(center_a: Sequence[float], half_a: Sequence[float], center_b0: Sequence[float],
                          half_b: Sequence[float], about: Sequence[float], q_rad: float) -> float:
    """The separation of two rectangles at one pose, exactly (separating axes).

    ``A`` is fixed, axis-aligned, centred at ``center_a`` with half lengths
    ``half_a = (a_x, a_z)``. ``B`` is axis-aligned at the extended pose,
    centred at ``center_b0`` with half lengths ``half_b``, then turned by
    ``q`` about ``about``. With ``U = {e_x, e_z, R_q e_x, R_q e_z}`` and
    ``m_u = |u . (c_B(q) - c_A)| - r_A(u) - r_B(u)``, the value is
    ``max_u m_u``: positive when the rectangles are apart (by that much along
    the best axis), 0 when they touch, negative when they overlap with
    positive area (by that depth). The sign is exact; the positive value is a
    separation along an axis, not the Euclidean distance.
    """

    turn = rotation(q_rad)
    a = np.asarray(half_a, dtype=float)
    b = np.asarray(half_b, dtype=float)
    between = turned(center_b0, about, q_rad) - np.asarray(center_a, dtype=float)
    own = (np.array((1.0, 0.0)), np.array((0.0, 1.0)))
    other = (turn @ own[0], turn @ own[1])
    best = -math.inf
    for u in (*own, *other):
        reach_a = a[0] * abs(u @ own[0]) + a[1] * abs(u @ own[1])
        reach_b = b[0] * abs(u @ other[0]) + b[1] * abs(u @ other[1])
        best = max(best, abs(u @ between) - reach_a - reach_b)
    return float(best)


def symmetric_gap_law(face_depth_mm: float, pole_depth_mm: Sequence[float], q_rad: Sequence[float],
                      required_rad: float) -> tuple[float, float]:
    """The symmetric clearance law: ``G = max_q 2 [t - z(q)]_+ tan(q/2)`` over
    ``0 < q <= q_req``, and the flexion that sets it (rad; 0 when the gap is 0).

    Valid when both pads' contact faces are ``t`` below the link line and, at
    every pose, their joint-side lower corners are equally far from the pole
    along the line (so the pole's ``x`` stays midway between them). It is
    then the least gap with which the pads do not meet anywhere on the way
    from the extended pose to ``q_req``. Otherwise it is an estimate.
    """

    best, at = 0.0, 0.0
    for z, q in zip(pole_depth_mm, q_rad):
        if q <= POLE_MIN_RAD or q > required_rad + 1e-9 or not math.isfinite(z):
            continue
        value = 2.0 * max(face_depth_mm - z, 0.0) * math.tan(q / 2.0)
        if value > best:
            best, at = value, q
    return best, at


__all__ = ["POLE_MIN_RAD", "pole", "rectangular_clearance", "rotation", "symmetric_gap_law", "turned"]
