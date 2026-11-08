#!/usr/bin/env python3
"""E01: actuator-inclusive packaging share at matched morphology (extended pose).

For each matched reference carrying a module, rasterize the existing scene's
solids (side view, 0.1 mm grid). Then read off:
- the hardware's palmar intrusion below the link line along the finger,
  h(x), and the length over which hardware stays within d of the line
  (the "module-free length");
- the dorsal-palmar thickness;
- the pads' share of the occupied side-view area.
Bars are capsules of the goal's link radius (0 when unset).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from matplotlib.path import Path as MplPath

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
from core.optimization.analysis.transmission import Transmission  # noqa: E402

HARDWARE = ("link", "pin", "servo", "pulley", "roller", "carrier")
STEP = 0.1
DEPTHS = (0.0, 3.0, 5.0, 8.0, 12.0)
MODULES = ("baseline", "thick")


def _poly(poly):
    poly = np.asarray(poly, dtype=float)
    return poly[0] if poly.ndim == 3 else poly


def rasterize(shapes_list, X, Z, link_radius):
    mask = np.zeros(X.shape, dtype=bool)
    points = np.column_stack((X.ravel(), Z.ravel()))
    for shapes in shapes_list:
        for poly in shapes.polygons:
            mask |= MplPath(_poly(poly)).contains_points(points, radius=1e-9).reshape(X.shape)
        for centers, radius in shapes.disks:
            c = np.asarray(centers, dtype=float).reshape(-1, 2)[0]
            mask |= (X - c[0]) ** 2 + (Z - c[1]) ** 2 <= radius ** 2
        for seg in shapes.segments:
            a, b = np.asarray(seg, dtype=float).reshape(-1, 2, 2)[0]
            d = b - a
            t = np.clip(((X - a[0]) * d[0] + (Z - a[1]) * d[1]) / max(d @ d, 1e-12), 0, 1)
            dist2 = (X - a[0] - t * d[0]) ** 2 + (Z - a[1] - t * d[1]) ** 2
            mask |= dist2 <= max(link_radius, STEP / 2) ** 2
    return mask


def analyse(reference, modules):
    x, config = common.carried(reference, modules)
    if x is None:
        return None
    scene = Transmission(reference.mechanism, x, config).scene(0.0)
    solids = scene.solids["proximal_pad"]
    pads = [solids["other_pad"], scene.solids["distal_pad"]["other_pad"]]
    link_radius = getattr(common.goal_of(config), "link_radius_mm", None) or 0.0
    xs = np.arange(-15.0, 105.0, STEP)
    zs = np.arange(-40.0, 40.0, STEP)
    X, Z = np.meshgrid(xs, zs)
    hardware = rasterize([solids[k] for k in HARDWARE], X, Z, link_radius)
    pad = rasterize(pads, X, Z, 0.0)
    length = common.shared.partitions.partition(reference.mechanism).external(x, config)["finger_length_mm"]
    inside = (xs >= 0.0) & (xs <= length)
    below = np.where(hardware & (Z < 0.0), -Z, 0.0).max(axis=0)  # palmar intrusion h(x), mm
    row = {"reference_id": reference.reference_id, "module": None, "finger_length_mm": length}
    for d in DEPTHS:
        row[f"free_length_d{d:g}_mm"] = float(np.sum(below[inside] <= d + 1e-9) * STEP)
    row["free_share_d0"] = row["free_length_d0_mm"] / length
    row["max_intrusion_mm"] = float(below[inside].max())
    occupied = hardware | pad
    zocc = Z[occupied]
    row["dorsal_mm"], row["palmar_mm"] = float(zocc.max()), float(zocc.min())
    row["thickness_mm"] = row["dorsal_mm"] - row["palmar_mm"]
    row["hardware_area_mm2"] = float((hardware & ~pad).sum() * STEP ** 2)
    row["pad_area_mm2"] = float(pad.sum() * STEP ** 2)
    row["pad_share_of_occupied"] = row["pad_area_mm2"] / float(occupied.sum() * STEP ** 2)
    profile = {"x_mm": xs[inside], "h_mm": below[inside]}
    return row, profile


def main():
    refs, cat = common.references(), common.catalog()
    out = common.output("e01_packaging")
    rows, profiles = [], {}
    for name in MODULES:
        for rid, ref in refs.items():
            found = analyse(ref, cat[name].modules)
            if found is None:
                rows.append({"reference_id": rid, "module": name, "mounted": False})
                continue
            row, profile = found
            row.update(module=name, mounted=True, upper_bound=ref.upper_bound)
            rows.append(row)
            profiles[f"{rid}__{name}"] = profile
    np.savez(out / "profiles.npz", **{f"{k}__{v}": profile[v] for k, profile in profiles.items() for v in profile})
    common.finish(out, rows, {"modules": MODULES, "depths_mm": DEPTHS, "grid_mm": STEP, "pose_deg": 0.0,
                              "hardware_categories": HARDWARE},
                  note="mcp_servo (palm) excluded; bars as capsules of goal link radius or half the grid")
    for r in rows:
        print({k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()})


if __name__ == "__main__":
    main()
