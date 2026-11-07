#!/usr/bin/env python3
"""E08: the range of object radii that both pad faces can touch at once.

At each distal flexion phi (30-150 deg, 5 deg steps), circles tangent to both
contact-face lines on their outward side are scanned over radius r = 1-80 mm
(0.25 mm). A radius is admissible at phi when:
- both tangency points lie on the faces (a 0.5 mm margin from the ends);
- the circle clears every non-pad solid (links, pins, servos, pulleys,
  rollers, carrier, the MCP servo) and the far region of each pad.
The admissible set over all poses, and its extent, are reported per
architecture and module. This is geometry only: no force closure,
friction or stability is implied.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from matplotlib.path import Path as MplPath

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
from core.optimization.analysis.transmission import Transmission  # noqa: E402

PHIS = tuple(float(p) for p in range(30, 151, 5))
RADII = np.arange(1.0, 80.0001, 0.25)
MODULES = ("baseline", "thick", "combined_L30_T3", "combined_L30_T8")
SOLIDS = ("carrier", "link", "pin", "servo", "mcp_servo", "pulley", "roller")
MARGIN = 0.5


def _seg_dist(p, a, b):
    d = b - a
    t = np.clip(np.dot(p - a, d) / max(np.dot(d, d), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(p - a - t * d))


def clears(center, r, scene):
    solids = scene.solids["proximal_pad"]
    for name in SOLIDS:
        shapes = solids[name]
        for poly in shapes.polygons:
            poly = np.asarray(poly, dtype=float)
            poly = poly[0] if poly.ndim == 3 else poly
            if MplPath(poly).contains_point(center):
                return False
            if min(_seg_dist(center, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))) < r:
                return False
        for centers, radius in shapes.disks:
            c = np.asarray(centers, dtype=float).reshape(-1, 2)[0]
            if np.linalg.norm(center - c) < r + radius:
                return False
        for seg in shapes.segments:
            a, b = np.asarray(seg, dtype=float).reshape(-1, 2, 2)[0]
            if _seg_dist(center, a, b) < r:
                return False
    return True


def admissible(scene):
    (pf, pn), (df, dn) = scene.faces["proximal_pad"], scene.faces["distal_pad"]
    p0, p1, n_p = pf[0, 0], pf[0, 1], pn[0]
    d0, d1, n_d = df[0, 0], df[0, 1], dn[0]
    A = np.array([n_p, n_d])
    if abs(np.linalg.det(A)) < 1e-9:
        return np.zeros(len(RADII), dtype=bool)
    ok = np.zeros(len(RADII), dtype=bool)
    for i, r in enumerate(RADII):
        c = np.linalg.solve(A, np.array([n_p @ p0 + r, n_d @ d0 + r]))
        good = True
        for (a, b, n) in ((p0, p1, n_p), (d0, d1, n_d)):
            t = c - r * n  # tangency point on the face line
            u = b - a
            s = np.dot(t - a, u) / np.dot(u, u)
            length = np.linalg.norm(u)
            if not (MARGIN / length <= s <= 1.0 - MARGIN / length):
                good = False
                break
        if good and clears(c, r, scene):
            ok[i] = True
    return ok


def main():
    refs, cat = common.references(), common.catalog()
    out = common.output("e08_enclosing")
    rows, summary = [], []
    for module in MODULES:
        for rid, ref in refs.items():
            x, config = common.carried(ref, cat[module].modules)
            if x is None:
                continue
            t = Transmission(ref.mechanism, x, config)
            union = np.zeros(len(RADII), dtype=bool)
            for phi in PHIS:
                try:
                    scene = t.scene(np.radians(phi))
                except ValueError:
                    continue
                ok = admissible(scene)
                union |= ok
                rows.append({"reference_id": rid, "module": module, "phi_deg": phi,
                             "r_min_mm": float(RADII[ok].min()) if ok.any() else None,
                             "r_max_mm": float(RADII[ok].max()) if ok.any() else None,
                             "measure_mm": float(ok.sum() * 0.25)})
            summary.append({"reference_id": rid, "module": module, "upper_bound": ref.upper_bound,
                            "r_min_mm": float(RADII[union].min()) if union.any() else None,
                            "r_max_mm": float(RADII[union].max()) if union.any() else None,
                            "measure_mm": float(union.sum() * 0.25)})
    common.shared.write_csv(out / "summary.csv", summary)
    common.finish(out, rows, {"phis_deg": PHIS, "radii_mm": [float(RADII[0]), float(RADII[-1]), 0.25],
                              "modules": MODULES, "margin_mm": MARGIN, "solids": SOLIDS})
    for s in summary:
        print(s)


if __name__ == "__main__":
    main()
