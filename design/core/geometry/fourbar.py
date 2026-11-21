"""A four-bar's motion from its dimensions, in closed form (report/study0/feasible_space.md).

The CFL: pivots ``A`` and ``A'`` on the proximal phalanx, ``B`` and ``B'`` on
the distal one, links ``A'-B`` (length ``r1``) and ``A-B'`` (length ``r2``).
With ``u = A - A'`` (length ``e``, angle ``alpha``), ``w = B' - B`` (length
``c``, angle ``beta``) and ``delta = alpha - beta``, at flexion ``q`` (the
distal phalanx turned by ``q`` toward the palm) ``B`` is where the circle
about ``A'`` of radius ``r1`` meets the circle about ``A - R_q w`` of radius
``r2``, and the centres are ``D(q)`` apart:

    D(q)^2 = e^2 + c^2 - 2 e c cos(delta + q).

The four-bar closes at ``q`` iff ``|r1 - r2| <= D(q) <= r1 + r2``; the two
assemblies meet only at equality, so it flexes to ``q_req`` on the assembly it
is extended in iff the inequalities are strict on ``[0, q_req]``.

- :func:`shape`: ``(e, c, delta, r1, r2)`` of four pivots;
- :func:`centre_distance_range`: ``min`` and ``max`` of ``D`` over ``[0, q_req]``;
- :func:`reach`: the largest flexion on the extended assembly;
- :func:`closes`: the strict condition on ``[0, q_req]``, with a margin;
- :func:`from_closure_coordinates` / :func:`closure_coordinates`: coordinates
  in which that condition is the box (``A`` and ``B`` on the link line).
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np


def shape(a: Sequence[float], ap: Sequence[float], b: Sequence[float], bp: Sequence[float]):
    """``(e, c, delta, r1, r2)`` of the pivots in the extended pose."""

    a, ap, b, bp = (np.asarray(p, dtype=float) for p in (a, ap, b, bp))
    u, w = a - ap, bp - b
    return (float(np.hypot(*u)), float(np.hypot(*w)), math.atan2(u[1], u[0]) - math.atan2(w[1], w[0]),
            float(np.hypot(*(b - ap))), float(np.hypot(*(bp - a))))


def centre_distance(e: float, c: float, delta: float, q: float) -> float:
    return math.sqrt(max(e * e + c * c - 2.0 * e * c * math.cos(delta + q), 0.0))


def centre_distance_range(e: float, c: float, delta: float, required: float) -> tuple[float, float]:
    """``(D_min, D_max)`` over ``[0, required]``: the ends, and ``|e - c|`` or
    ``e + c`` where ``delta + q`` passes a multiple of ``2 pi`` or an odd
    multiple of ``pi`` inside the range."""

    values = [centre_distance(e, c, delta, 0.0), centre_distance(e, c, delta, required)]
    first = math.ceil(delta / math.pi)
    last = math.floor((delta + required) / math.pi)
    for k in range(first, last + 1):
        values.append(abs(e - c) if k % 2 == 0 else e + c)
    return min(values), max(values)


def closes(e: float, c: float, delta: float, r1: float, r2: float, required: float, margin: float = 0.0) -> bool:
    """The four-bar flexes to ``required`` on the assembly it is extended in,
    the centre distance staying ``margin`` inside its limits."""

    low, high = centre_distance_range(e, c, delta, required)
    return abs(r1 - r2) + margin < low and r1 + r2 - margin > high


def reach(e: float, c: float, delta: float, r1: float, r2: float) -> float:
    """The largest flexion on the extended assembly (rad, at most ``pi``): the
    first ``q > 0`` at which ``D(q)`` meets ``|r1 - r2|`` or ``r1 + r2``."""

    if e <= 0.0 or c <= 0.0:
        return 0.0
    best = math.pi
    for limit in (r1 - r2, r1 + r2):
        k = (e * e + c * c - limit * limit) / (2.0 * e * c)
        if abs(k) <= 1.0:
            angle = math.acos(k)
            for root in (angle, -angle):
                q = (root - delta) % (2.0 * math.pi)
                if q > 1e-9:
                    best = min(best, q)
    return best


def branch_of(a: Sequence[float], ap: Sequence[float], b: Sequence[float], bp: Sequence[float]) -> int:
    """Which extended assembly the pivots are in (+1 or -1): the side of the
    line from ``A'`` to ``A - w`` that ``B`` is on."""

    a, ap, b, bp = (np.asarray(p, dtype=float) for p in (a, ap, b, bp))
    line, to_b = a - (bp - b) - ap, b - ap
    return 1 if line[0] * to_b[1] - line[1] * to_b[0] >= 0.0 else -1


def joint_distance(e: float, c: float, delta: float, r1: float, r2: float, q, branch: int = 1) -> np.ndarray:
    """``|A - B(q)|`` at each flexion of ``q`` (rad) on the assembly ``branch``
    (``nan`` where the four-bar does not close)."""

    q = np.asarray(q, dtype=float)
    first = np.array((-e, 0.0))
    second = -c * np.stack((np.cos(-delta - q), np.sin(-delta - q)), axis=-1)
    between = second - first
    distance = np.hypot(between[..., 0], between[..., 1])
    with np.errstate(invalid="ignore", divide="ignore"):
        along = (distance ** 2 + r1 * r1 - r2 * r2) / (2.0 * distance)
        across = np.sqrt(r1 * r1 - along ** 2)
        unit = between / distance[..., None]
    normal = np.stack((-unit[..., 1], unit[..., 0]), axis=-1)
    b = first + along[..., None] * unit + branch * across[..., None] * normal
    return np.hypot(b[..., 0], b[..., 1])


def transmission_angle(e: float, c: float, delta: float, r1: float, r2: float, q, branch: int = 1) -> np.ndarray:
    """The angle at ``B'`` between ``B'A`` and ``B'B`` at each flexion (rad)."""

    span = joint_distance(e, c, delta, r1, r2, q, branch)
    return np.arccos(np.clip((r2 * r2 + c * c - span ** 2) / (2.0 * r2 * c), -1.0, 1.0))


def from_closure_coordinates(a_x: float, e: float, c: float, delta: float, total: float, difference: float,
                             branch: int = 1):
    """The pivots ``(A, A', B, B')`` of the four-bar with ``A`` at ``(a_x, 0)``,
    ``B`` on the link line beyond it, the given ``e``, ``c``, ``delta`` and
    link lengths ``r1 = (total + difference) / 2``, ``r2 = (total - difference) / 2``;
    ``branch`` (+1 or -1) picks one of the two extended assemblies. ``None``
    where the extended pose does not close.

    About ``A``: ``A' = -e (cos alpha, sin alpha)`` and ``A - w = -c (cos beta,
    sin beta)`` with ``beta = alpha - delta``; ``B`` is on both circles. Turning
    ``alpha`` turns the whole figure about ``A``, so ``B`` on the link line
    fixes ``alpha``."""

    r1, r2 = 0.5 * (total + difference), 0.5 * (total - difference)
    if r1 <= 0.0 or r2 <= 0.0 or e <= 0.0 or c <= 0.0:
        return None
    first = np.array((-e, 0.0))                                    # A' about A, alpha = 0
    second = -c * np.array((math.cos(-delta), math.sin(-delta)))   # A - w about A, alpha = 0
    between = second - first
    distance = float(np.hypot(*between))
    if distance <= 0.0:
        return None
    along = (distance * distance + r1 * r1 - r2 * r2) / (2.0 * distance)
    across = r1 * r1 - along * along
    if across < 0.0:
        return None
    unit = between / distance
    b = first + along * unit + branch * math.sqrt(across) * np.array((-unit[1], unit[0]))
    span = float(np.hypot(*b))
    if span <= 0.0:
        return None
    alpha = -math.atan2(b[1], b[0])                                # turn B onto the +x axis
    cos, sin = math.cos(alpha), math.sin(alpha)
    turn = np.array([[cos, -sin], [sin, cos]])
    origin = np.array((a_x, 0.0))
    ap, hat, b = origin + turn @ first, turn @ second, origin + np.array((span, 0.0))
    return origin, ap, b, b - hat                                  # B' = B + w, w = -(A - w about A)


def closure_coordinates(a: Sequence[float], ap: Sequence[float], b: Sequence[float], bp: Sequence[float]):
    """``(a_x, e, c, delta, total, difference)`` of pivots whose ``A`` and
    ``B`` are on the link line (the inverse of :func:`from_closure_coordinates`
    up to the branch)."""

    e, c, delta, r1, r2 = shape(a, ap, b, bp)
    return float(np.asarray(a, dtype=float)[0]), e, c, delta, r1 + r2, r1 - r2


__all__ = ["branch_of", "centre_distance", "centre_distance_range", "closes", "closure_coordinates",
           "from_closure_coordinates", "joint_distance", "reach", "shape", "transmission_angle"]
