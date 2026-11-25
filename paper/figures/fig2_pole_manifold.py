"""Where the crossed four-bar can put the finite pole, by two-position (Burmester) synthesis, for q swept over
[Q0, Q1] (Fig. 2(b)). No random sampling: a point P is reachable at flexion q if some design in the study's CFL
space rotates the distal phalanx by q about P.

Design space (as fig_mechanisms.four_bar_pole_sets): A = (a, 0), B = (b, 0) on the link line, 0.1 <= a <= b <= 90;
A' = A + (-20..30, 0..25); B' = B + (-50..20, -60..80). Links A'-B (A' fixed, B moving) and A-B' (A fixed, B'
moving). With R the rotation by q about P:
  link A'-B : A' lies on the perpendicular bisector of B and R B;
  link A-B' : B' lies on the perpendicular bisector of A and R^-1 A (|A - R B'| = |R^-1 A - B'|).
A candidate (a, b) whose bisectors both cross their boxes gives A' and B' (the bisector points nearest the boxes'
centres); it counts only if its links A'-B and A-B' cross at q = 0 and the linkage closes through [0, q] on one
assembly branch (stepped in 1 deg, as fig_mechanisms.four_bar_pole_sets) and ends with the distal phalanx at
R(B), R(B').

    conda run -n cmd python paper/figures/fig2_pole_manifold.py
Writes design/output/paper/fig2_pole_manifold.npz (grid x, z and the reachable mask over the q sweep).
"""
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "design" / "output" / "paper" / "fig2_pole_manifold.npz"
Q0, Q1, DQ = 10.0, 120.0, 10.0
XS = np.arange(-35.0, 60.01, 1.0)
ZS = np.arange(-25.0, 35.01, 1.0)
AS = np.arange(0.1, 90.01, 2.0)


def rot(pts, q, c):
    """Palmar flexion by q (clockwise in x distal, z dorsal) about c; pts (..., 2), c (..., 2)."""
    cq, sq = np.cos(q), np.sin(q)
    d = pts - c
    return np.stack([cq * d[..., 0] + sq * d[..., 1], -sq * d[..., 0] + cq * d[..., 1]], -1) + c


def crosses_box(m, n, lo, hi):
    """Does the line {X : n.(X - m) = 0} meet the axis-aligned box [lo, hi]? m, n, lo, hi broadcast (..., 2)."""
    f = []
    for cx in (lo[..., 0], hi[..., 0]):
        for cz in (lo[..., 1], hi[..., 1]):
            f.append(n[..., 0] * (cx - m[..., 0]) + n[..., 1] * (cz - m[..., 1]))
    f = np.stack(f, -1)
    return (f.min(-1) <= 0) & (f.max(-1) >= 0)


def nearest_on_line(m, n, lo, hi):
    """The point of the line n.(X - m) = 0 nearest the box centre, clipped into the box."""
    c = (lo + hi) / 2
    t = np.array([-n[1], n[0]]) / np.linalg.norm(n)
    x = m + ((c - m) @ t) * t
    return np.clip(x, lo, hi)


def segments_cross(p1, p2, p3, p4) -> bool:
    side = lambda a, b, c: (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])  # noqa: E731
    return side(p1, p2, p3) * side(p1, p2, p4) < 0 and side(p3, p4, p1) * side(p3, p4, p2) < 0


def rot2(t):
    return np.array([[np.cos(t), np.sin(t)], [-np.sin(t), np.cos(t)]])


def closes(A, Ap, B, Bp, q, P) -> bool:
    """The linkage moves from 0 to q on one branch and ends with B at R_P(B)."""
    r1, r2, w = np.linalg.norm(B - Ap), np.linalg.norm(Bp - A), Bp - B
    prev = B.copy()
    for t in np.radians(np.arange(0.0, np.degrees(q) + 1e-9, 1.0)):
        ah = A - rot2(t) @ w
        d = np.linalg.norm(ah - Ap)
        if not abs(r1 - r2) < d < r1 + r2:
            return False
        along = (r1 ** 2 - r2 ** 2 + d ** 2) / (2 * d)
        u = (ah - Ap) / d
        mid, off = Ap + along * u, np.sqrt(max(r1 ** 2 - along ** 2, 0.0)) * np.array([-u[1], u[0]])
        c1, c2 = mid + off, mid - off
        prev = c1 if np.linalg.norm(c1 - prev) < np.linalg.norm(c2 - prev) else c2
    target = rot(B[None], q, P[None])[0]
    return np.linalg.norm(prev - target) < 0.5


def reachable(P, q):
    """Mask over the points P (N, 2) reachable at flexion q (rad)."""
    N = len(P)
    ok = np.zeros(N, bool)
    Pn = P[:, None, :]
    for a in AS:
        A = np.array([a, 0.0])
        Ainv = rot(A[None, None, :], -q, Pn)
        m2, n2 = (A + Ainv) / 2, Ainv - A
        bs = np.arange(a, 90.01, 2.0)
        B = np.stack([bs, np.zeros_like(bs)], -1)[None]
        lo2, hi2 = B + np.array([-50.0, -60.0]), B + np.array([20.0, 80.0])
        c2 = crosses_box(m2, n2, lo2, hi2)
        RB = rot(B, q, Pn)
        m1, n1 = (B + RB) / 2, RB - B
        lo1, hi1 = A + np.array([-20.0, 0.0]), A + np.array([30.0, 25.0])
        c1 = crosses_box(m1, n1, lo1[None, None], hi1[None, None])
        for i, j in zip(*np.nonzero(c1 & c2 & ~ok[:, None])):
            if ok[i]:
                continue
            Bj = B[0, j]
            Ap = nearest_on_line(m1[i, j], n1[i, j], lo1, hi1)
            Bp = nearest_on_line(m2[i, 0], n2[i, 0], lo2[0, j], hi2[0, j])
            if segments_cross(Ap, Bj, A, Bp) and closes(A, Ap, Bj, Bp, q, P[i]):
                ok[i] = True
    return ok


def main() -> int:
    gx, gz = np.meshgrid(XS, ZS)
    P = np.c_[gx.ravel(), gz.ravel()]
    qs = np.arange(Q0, Q1 + 1e-9, DQ)
    masks = []
    t0 = time.time()
    for q in qs:
        masks.append(reachable(P, np.radians(q)).reshape(gx.shape))
        print(f"q {q:5.1f}: {masks[-1].mean() * 100:5.1f}% of the grid  ({time.time() - t0:5.0f} s)", flush=True)
    masks = np.array(masks)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(OUT, x=XS, z=ZS, q=qs, masks=masks)
    print(OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
