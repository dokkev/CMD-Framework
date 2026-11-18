"""Exact 1D interval-set algebra and analytical face-occlusion projections.

Leaf geometry module (depends only on ``packaging.faces``) used by both
``core.morphology.coverage`` and individual mechanism evaluators.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from typing import Literal

import numpy as np

from core.morphology.packaging.faces import crossing_free_lengths, fold_free_lengths

_INTERVAL_EPS = 1e-12
_NORMAL_EPS = 1e-9


@dataclass(frozen=True)
class IntervalSet:
    """Finite union of disjoint non-degenerate closed intervals ``[a_k, b_k]`` on ``R``."""

    intervals: tuple[tuple[float, float], ...] = ()

    def __post_init__(self) -> None:
        cleaned: list[tuple[float, float]] = []
        raw = sorted(
            (float(a), float(b))
            for a, b in self.intervals
            if math.isfinite(a) and math.isfinite(b) and float(b) - float(a) > _INTERVAL_EPS
        )
        for a, b in raw:
            if cleaned and a <= cleaned[-1][1] + _INTERVAL_EPS:
                cleaned[-1] = (cleaned[-1][0], max(cleaned[-1][1], b))
            else:
                cleaned.append((a, b))
        object.__setattr__(self, "intervals", tuple(cleaned))

    @classmethod
    def empty(cls) -> IntervalSet:
        return cls(())

    @classmethod
    def closed(cls, low: float, high: float) -> IntervalSet:
        if not (math.isfinite(low) and math.isfinite(high)) or high - low <= _INTERVAL_EPS:
            return cls(())
        return cls(((float(low), float(high)),))

    @classmethod
    def from_intervals(cls, intervals: Iterable[tuple[float, float]]) -> IntervalSet:
        return cls(tuple(intervals))

    @property
    def measure(self) -> float:
        """One-dimensional Lebesgue measure ``mu(U) = sum_k (b_k - a_k)``."""

        return sum(b - a for a, b in self.intervals)

    @property
    def is_empty(self) -> bool:
        return len(self.intervals) == 0

    @property
    def is_connected(self) -> bool:
        return len(self.intervals) <= 1

    @property
    def component_count(self) -> int:
        return len(self.intervals)

    def contains(self, xi: float, *, tol: float = 1e-9) -> bool:
        x = float(xi)
        return any(a - tol <= x <= b + tol for a, b in self.intervals)

    def union(self, other: IntervalSet) -> IntervalSet:
        return IntervalSet((*self.intervals, *other.intervals))

    def intersection(self, other: IntervalSet) -> IntervalSet:
        if self.is_empty or other.is_empty:
            return IntervalSet.empty()
        out: list[tuple[float, float]] = []
        i = j = 0
        a_list, b_list = self.intervals, other.intervals
        while i < len(a_list) and j < len(b_list):
            lo = max(a_list[i][0], b_list[j][0])
            hi = min(a_list[i][1], b_list[j][1])
            if hi - lo > _INTERVAL_EPS:
                out.append((lo, hi))
            if a_list[i][1] < b_list[j][1]:
                i += 1
            else:
                j += 1
        return IntervalSet(tuple(out))

    def difference(self, other: IntervalSet) -> IntervalSet:
        """Exact set difference ``self \\ other`` (interior occlusion splits intervals)."""

        if self.is_empty or other.is_empty:
            return self
        remaining = list(self.intervals)
        for cut_lo, cut_hi in other.intervals:
            next_remaining: list[tuple[float, float]] = []
            for lo, hi in remaining:
                if cut_hi <= lo + _INTERVAL_EPS or cut_lo >= hi - _INTERVAL_EPS:
                    next_remaining.append((lo, hi))
                    continue
                if cut_lo - lo > _INTERVAL_EPS:
                    next_remaining.append((lo, cut_lo))
                if hi - cut_hi > _INTERVAL_EPS:
                    next_remaining.append((cut_hi, hi))
            remaining = next_remaining
            if not remaining:
                break
        return IntervalSet(tuple(remaining))


def segment_occluded_interval(
    face_outer: np.ndarray,
    face_inner: np.ndarray,
    outward_normal: np.ndarray,
    seg_start: np.ndarray,
    seg_end: np.ndarray,
) -> IntervalSet:
    """Exact interval of ``xi in [0, l]`` occluded along ``+outward_normal`` by segment ``seg_start--seg_end``."""

    f0 = np.asarray(face_outer, dtype=float)
    f1 = np.asarray(face_inner, dtype=float)
    diff = f1 - f0
    length = float(np.linalg.norm(diff))
    if length <= _INTERVAL_EPS:
        return IntervalSet.empty()
    t_hat = diff / length
    n_hat = np.asarray(outward_normal, dtype=float)

    p0 = np.asarray(seg_start, dtype=float) - f0
    p1 = np.asarray(seg_end, dtype=float) - f0
    xi0, nu0 = float(np.dot(p0, t_hat)), float(np.dot(p0, n_hat))
    xi1, nu1 = float(np.dot(p1, t_hat)), float(np.dot(p1, n_hat))

    if nu0 <= _NORMAL_EPS and nu1 <= _NORMAL_EPS:
        return IntervalSet.empty()
    if nu0 > 0.0 and nu1 > 0.0:
        lo, hi = min(xi0, xi1), max(xi0, xi1)
    elif nu0 > 0.0:
        s0 = nu0 / (nu0 - nu1)
        xi_cross = xi0 + s0 * (xi1 - xi0)
        lo, hi = min(xi0, xi_cross), max(xi0, xi_cross)
    else:
        s0 = -nu0 / (nu1 - nu0)
        xi_cross = xi0 + s0 * (xi1 - xi0)
        lo, hi = min(xi_cross, xi1), max(xi_cross, xi1)

    return IntervalSet.closed(max(0.0, lo), min(length, hi))


def disk_occluded_interval(
    face_outer: np.ndarray,
    face_inner: np.ndarray,
    outward_normal: np.ndarray,
    center: np.ndarray,
    radius_mm: float,
) -> IntervalSet:
    """Exact interval of ``xi in [0, l]`` occluded along ``+outward_normal`` by disk ``D(center, r)``."""

    r = float(radius_mm)
    if r <= _INTERVAL_EPS:
        return IntervalSet.empty()
    f0 = np.asarray(face_outer, dtype=float)
    f1 = np.asarray(face_inner, dtype=float)
    diff = f1 - f0
    length = float(np.linalg.norm(diff))
    if length <= _INTERVAL_EPS:
        return IntervalSet.empty()
    t_hat = diff / length
    n_hat = np.asarray(outward_normal, dtype=float)

    rel = np.asarray(center, dtype=float) - f0
    xi_c = float(np.dot(rel, t_hat))
    nu_c = float(np.dot(rel, n_hat))

    if nu_c <= -r + _NORMAL_EPS:
        return IntervalSet.empty()
    half_width = r if nu_c > 0.0 else math.sqrt(max(r * r - nu_c * nu_c, 0.0))
    return IntervalSet.closed(max(0.0, xi_c - half_width), min(length, xi_c + half_width))


def face_occluded_set(
    face: np.ndarray,
    outward_normal: np.ndarray,
    *,
    segments: Sequence[tuple[np.ndarray, np.ndarray]] = (),
    disks: Sequence[tuple[np.ndarray, float]] = (),
) -> IntervalSet:
    """Exact union ``O_i^link <= [0, l_i]`` of all segment and disk occluders in front of ``face``."""

    f = np.asarray(face, dtype=float)
    f0, f1 = f[0], f[1]
    occluded = IntervalSet.empty()
    for s0, s1 in segments:
        occluded = occluded.union(segment_occluded_interval(f0, f1, outward_normal, s0, s1))
    for center, radius in disks:
        occluded = occluded.union(disk_occluded_interval(f0, f1, outward_normal, center, radius))
    return occluded


def _segment_fold_interval(
    face: np.ndarray,
    other_face: np.ndarray,
    other_normal: np.ndarray,
    length: float,
    expected_free: float,
) -> IntervalSet:
    if expected_free >= length - _INTERVAL_EPS:
        return IntervalSet.closed(0.0, length)
    outer, inner = face[0], face[1]
    t = (inner - outer) / length
    g0, g1 = other_face[0], other_face[1]
    other_len = float(np.linalg.norm(g1 - g0))
    if other_len <= _INTERVAL_EPS:
        return IntervalSet.closed(0.0, length)
    tg = (g1 - g0) / other_len
    n = np.asarray(other_normal, dtype=float)
    rel = outer - g0
    a, b = float(np.dot(rel, n)), float(np.dot(t, n))
    if abs(b) < 1e-12:
        behind_lo, behind_hi = (-math.inf, math.inf) if a < 0.0 else (math.inf, -math.inf)
    elif b < 0.0:
        behind_lo, behind_hi = -a / b, math.inf
    else:
        behind_lo, behind_hi = -math.inf, -a / b
    c, d = float(np.dot(rel, tg)), float(np.dot(t, tg))
    if abs(d) < 1e-12:
        inside = 0.0 <= c <= other_len
        facing_lo, facing_hi = (-math.inf, math.inf) if inside else (math.inf, -math.inf)
    else:
        r1, r2 = -c / d, (other_len - c) / d
        facing_lo, facing_hi = min(r1, r2), max(r1, r2)
    lo = max(0.0, behind_lo, facing_lo)
    hi = min(length, behind_hi, facing_hi)
    return IntervalSet.closed(0.0, length).difference(IntervalSet.closed(lo, hi))


def base_accessible_sets(
    proximal_face: np.ndarray,
    proximal_normal: np.ndarray,
    distal_face: np.ndarray,
    distal_normal: np.ndarray,
    *,
    fold_mode: Literal["crossing", "segment", "none"] = "crossing",
) -> dict[str, IntervalSet]:
    """Base accessible intervals ``U_i^base(M, phi) <= [0, l_i]`` before linkage occlusion."""

    p_f = np.asarray(proximal_face, dtype=float)
    d_f = np.asarray(distal_face, dtype=float)
    l_p = float(np.linalg.norm(p_f[1] - p_f[0]))
    l_d = float(np.linalg.norm(d_f[1] - d_f[0]))
    if fold_mode == "none":
        return {
            "proximal_pad": IntervalSet.closed(0.0, l_p),
            "distal_pad": IntervalSet.closed(0.0, l_d),
        }
    if fold_mode == "crossing":
        free = crossing_free_lengths(p_f, d_f)
        return {
            "proximal_pad": IntervalSet.closed(0.0, min(l_p, float(free["proximal_pad"][0]))),
            "distal_pad": IntervalSet.closed(0.0, min(l_d, float(free["distal_pad"][0]))),
        }
    if fold_mode == "segment":
        free = fold_free_lengths(p_f, proximal_normal, d_f, distal_normal)
        return {
            "proximal_pad": _segment_fold_interval(p_f, d_f, distal_normal, l_p, float(free["proximal_pad"][0])),
            "distal_pad": _segment_fold_interval(d_f, p_f, proximal_normal, l_d, float(free["distal_pad"][0])),
        }
    raise ValueError(f"fold_mode must be 'crossing', 'segment', or 'none'; got {fold_mode!r}")


__all__ = [
    "IntervalSet",
    "base_accessible_sets",
    "disk_occluded_interval",
    "face_occluded_set",
    "segment_occluded_interval",
]
