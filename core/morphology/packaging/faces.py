"""What of a contact face is hidden, by the other face in the fold
(:func:`fold_free_lengths`) or by any solid within a probe's depth in front
of it (:func:`probe_blocked`), whatever the mechanism."""

from __future__ import annotations

import math

import numpy as np

from core.morphology.packaging import _kernels
from core.morphology.packaging.shapes import as_polygons

def fold_free_lengths(
    proximal_face: np.ndarray, proximal_normal: np.ndarray, distal_face: np.ndarray, distal_normal: np.ndarray,
) -> dict[str, np.ndarray]:
    """Per-sample length of each contact face not hidden in the fold.

    Faces are segments ``(n, 2, 2)`` (or ``(2, 2)``), their outer end first
    (the MCP end of the proximal face, the fingertip end of the distal one),
    with their outward (palmar) unit normals ``(n, 2)`` (or ``(2,)``), all in
    the proximal frame. A point of one face is hidden when it lies behind the
    other face (on its pad's side of its line) and opposite it (its
    projection on that line falls within the other face): there the other
    face, not an object, is in front of it. Each face loses the length so
    hidden. The other face's segment, not its endless line, decides: barely
    flexed, two faces at different depths cross far from either pad, and
    hide nothing (the lines alone would hide a whole face there). Where the
    other face covers the crossing, this is the crossing rule
    (:func:`face_crossing_lengths`); the length lost grows continuously with
    the flexion.
    """

    def faces(face, normal):
        face = np.asarray(face, dtype=float)
        face = face[None] if face.ndim == 2 else face
        normal = np.broadcast_to(np.asarray(normal, dtype=float), (len(face), 2))
        return face, normal

    proximal_face, proximal_normal = faces(proximal_face, proximal_normal)
    distal_face, distal_normal = faces(distal_face, distal_normal)
    count = max(len(proximal_face), len(distal_face))
    proximal_face = np.broadcast_to(proximal_face, (count, 2, 2))
    distal_face = np.broadcast_to(distal_face, (count, 2, 2))
    proximal_normal = np.broadcast_to(proximal_normal, (count, 2))
    distal_normal = np.broadcast_to(distal_normal, (count, 2))

    def hidden(face: np.ndarray, other: np.ndarray, other_normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """The face's length and how much of it ``other`` hides."""

        outer, inner = face[:, 0], face[:, 1]
        length = np.linalg.norm(inner - outer, axis=1)
        t = (inner - outer) / np.where(length > 0.0, length, 1.0)[:, None]
        g0, g1 = other[:, 0], other[:, 1]
        other_length = np.linalg.norm(g1 - g0, axis=1)
        tg = (g1 - g0) / np.where(other_length > 0.0, other_length, 1.0)[:, None]
        rel = outer - g0
        # Behind the other face: (p - g0) . n < 0, linear in s along the face.
        a, b = np.sum(rel * other_normal, axis=1), np.sum(t * other_normal, axis=1)
        flat = np.abs(b) < 1e-12
        root = -a / np.where(flat, 1.0, b)
        behind_low = np.where(flat, np.where(a < 0.0, -np.inf, np.inf), np.where(b < 0.0, root, -np.inf))
        behind_high = np.where(flat, np.where(a < 0.0, np.inf, -np.inf), np.where(b > 0.0, root, np.inf))
        # Opposite it: 0 <= (p - g0) . tg <= its length.
        c, d = np.sum(rel * tg, axis=1), np.sum(t * tg, axis=1)
        level = np.abs(d) < 1e-12
        first = -c / np.where(level, 1.0, d)
        second = (other_length - c) / np.where(level, 1.0, d)
        inside = (c >= 0.0) & (c <= other_length)
        facing_low = np.where(level, np.where(inside, -np.inf, np.inf), np.minimum(first, second))
        facing_high = np.where(level, np.where(inside, np.inf, -np.inf), np.maximum(first, second))
        low = np.maximum.reduce([np.zeros(count), behind_low, facing_low])
        high = np.minimum.reduce([length, behind_high, facing_high])
        return length, np.maximum(high - low, 0.0)

    proximal_length, proximal_hidden = hidden(proximal_face, distal_face, distal_normal)
    distal_length, distal_hidden = hidden(distal_face, proximal_face, proximal_normal)
    return {"proximal_pad": proximal_length - proximal_hidden, "distal_pad": distal_length - distal_hidden}


def fold_hidden_points(face: np.ndarray, other_face: np.ndarray, other_normal: np.ndarray,
                       points: int) -> np.ndarray:
    """Which of ``points`` evenly spread points of a face (at the middles of
    equal pieces, as :func:`probe_blocked` probes) the other face hides in
    the fold, as :func:`fold_free_lengths` measures it: behind the other face
    and opposite it. Faces ``(n, 2, 2)``, the other's normal ``(n, 2)``;
    returns ``(n, points)``."""

    face = np.asarray(face, dtype=float)
    count = len(face)
    other_face = np.broadcast_to(np.asarray(other_face, dtype=float), (count, 2, 2))
    other_normal = np.broadcast_to(np.asarray(other_normal, dtype=float), (count, 2))
    share = (np.arange(points) + 0.5) / points
    at = face[:, :1] + share[None, :, None] * (face[:, 1:] - face[:, :1])           # (n, k, 2)
    g0, g1 = other_face[:, 0], other_face[:, 1]
    other_length = np.linalg.norm(g1 - g0, axis=1)
    tg = (g1 - g0) / np.where(other_length > 0.0, other_length, 1.0)[:, None]
    rel = at - g0[:, None]
    behind = np.sum(rel * other_normal[:, None], axis=-1) < 0.0
    along = np.sum(rel * tg[:, None], axis=-1)
    return behind & (along >= 0.0) & (along <= other_length[:, None])


PROBE_STEP_MM = 0.5
"""Spacing of the face points :func:`probe_free_lengths` probes."""


def probe_free_lengths(face: np.ndarray, normal: np.ndarray, depth_mm: float, polygons=(), segments=(),
                       disks=(), step_mm: float = PROBE_STEP_MM) -> np.ndarray:
    """The face's length times the share of its points :func:`probe_blocked` leaves free."""

    length, blocked = probe_blocked(face, normal, depth_mm, polygons, segments, disks, step_mm)
    return length * (1.0 - blocked.mean(axis=1))


def probe_blocked(face: np.ndarray, normal: np.ndarray, depth_mm: float, polygons=(), segments=(),
                  disks=(), step_mm: float = PROBE_STEP_MM) -> tuple[np.ndarray, np.ndarray]:
    """Per-sample length of a contact face an object can reach.

    A point of the face is reachable when nothing lies within ``depth_mm`` in
    front of it: the probe from it along the outward normal, ``depth_mm``
    long, meets no solid. Faces ``(n, 2, 2)`` (outer end first) with unit
    normals ``(n, 2)``; solids, every other part of the finger at each
    sample, as convex ``polygons`` ``(n, m, 2)``, ``segments`` ``(n, 2, 2)``
    (bars seen edge on) and ``disks`` ``((n, 2) centres, radius)``. The face
    is probed every ``step_mm`` (at the middles of equal pieces). Returns
    each face's length ``(n,)`` and which points are blocked ``(n, k)``.
    """

    face = np.asarray(face, dtype=float)
    count = len(face)
    normal = np.broadcast_to(np.asarray(normal, dtype=float), (count, 2))
    outer, inner = face[:, 0], face[:, 1]
    length = np.linalg.norm(inner - outer, axis=1)
    points = max(int(math.ceil(float(length.max()) / step_mm)), 1) if count else 1
    share = (np.arange(points) + 0.5) / points
    start = outer[:, None] + share[None, :, None] * (inner - outer)[:, None]          # (n, k, 2)
    end = start + depth_mm * normal[:, None]
    start = start + 1e-6 * normal[:, None]  # off the face itself
    blocked = np.zeros((count, points), dtype=bool)
    start, end = np.ascontiguousarray(start), np.ascontiguousarray(end)
    # A bar is a polygon of two corners (seen edge on): the probe crosses it or not.
    for polygon in (*polygons, *segments):
        polygon = np.broadcast_to(np.asarray(polygon, dtype=float), (count, *np.shape(polygon)[-2:]))
        blocked |= _kernels.probe_polygon(start, end, as_polygons(polygon))
    for centers, radius in disks:
        centers = np.broadcast_to(np.asarray(centers, dtype=float), (count, 2))[:, None]
        d = end - start
        t = np.clip(np.sum((centers - start) * d, axis=-1) / np.maximum(np.sum(d * d, axis=-1), 1e-12), 0.0, 1.0)
        blocked |= np.linalg.norm(start + t[..., None] * d - centers, axis=-1) <= radius
    return length, blocked


__all__ = [
    "PROBE_STEP_MM",
    "fold_free_lengths",
    "fold_hidden_points",
    "probe_blocked",
    "probe_free_lengths",
]
