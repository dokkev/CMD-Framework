#!/usr/bin/env python3
"""E07: hardware occlusion of the contact faces across the posture range.

For each matched reference carrying a module, the existing scene is built at
distal flexion phi = 0..150 deg (5 deg steps) and each face is probed along its
outward normal (``packaging.probe_blocked``) to depths 5, 10 and 20 mm. A face
point is "occluded" when a solid other than its own pad lies within the
probe depth. The share is reported for the finger's hardware (links, pins,
servos, pulleys, rollers, carrier; the MCP servo excluded) and, separately,
for the other pad (the fold). This is 2D, normal approach only; Y layers are
not resolved, so a bar in another layer still counts.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
from core.morphology.packaging.faces import probe_blocked  # noqa: E402
from core.optimization.analysis.transmission import Transmission  # noqa: E402

PHIS = tuple(float(p) for p in range(0, 151, 5))
DEPTHS = (5.0, 10.0, 20.0)
MODULES = ("baseline", "thick", "combined_L30_T3", "combined_L30_T8")
HARDWARE = ("carrier", "link", "pin", "servo", "pulley", "roller")


def masks(scene, side, depth, categories):
    face, normal = scene.faces[side]
    union = None
    for name in categories:
        shapes = scene.solids[side][name]
        _, blocked = probe_blocked(face, normal, depth, shapes.polygons, shapes.segments, shapes.disks)
        union = blocked if union is None else union | blocked
    return union


def main():
    refs, cat = common.references(), common.catalog()
    out = common.output("e07_occlusion")
    rows = []
    for module in MODULES:
        for rid, ref in refs.items():
            x, config = common.carried(ref, cat[module].modules)
            if x is None:
                continue
            t = Transmission(ref.mechanism, x, config)
            for phi in PHIS:
                try:
                    scene = t.scene(np.radians(phi))
                except ValueError:  # the four-bar does not close at this flexion
                    rows.append({"reference_id": rid, "module": module, "phi_deg": phi, "closes": False})
                    continue
                row = {"reference_id": rid, "module": module, "phi_deg": phi, "closes": True}
                for side in ("proximal_pad", "distal_pad"):
                    for depth in DEPTHS:
                        hw = masks(scene, side, depth, HARDWARE)
                        fold = masks(scene, side, depth, ("other_pad",))
                        key = f"{side.split('_')[0]}_d{depth:g}"
                        row[f"{key}_hardware"] = float(hw.mean())
                        row[f"{key}_fold"] = float(fold.mean())
                        row[f"{key}_any"] = float((hw | fold).mean())
                    for name in HARDWARE:
                        row[f"{side.split('_')[0]}_d10_{name}"] = float(masks(scene, side, 10.0, (name,)).mean())
                rows.append(row)
    summary = []
    for module in MODULES:
        for rid in refs:
            sel = [r for r in rows if r["reference_id"] == rid and r["module"] == module and r.get("closes")]
            if not sel:
                continue
            entry = {"reference_id": rid, "module": module, "poses": len(sel)}
            for side in ("proximal", "distal"):
                for depth in DEPTHS:
                    for kind in ("hardware", "fold"):
                        values = np.array([r[f"{side}_d{depth:g}_{kind}"] for r in sel])
                        entry[f"{side}_d{depth:g}_{kind}_mean"] = float(values.mean())
                        entry[f"{side}_d{depth:g}_{kind}_max"] = float(values.max())
                lo = np.array([r[f"{side}_d10_hardware"] for r in sel if r["phi_deg"] <= 60])
                hi = np.array([r[f"{side}_d10_hardware"] for r in sel if r["phi_deg"] >= 90])
                entry[f"{side}_d10_hardware_0to60"] = float(lo.mean()) if len(lo) else None
                entry[f"{side}_d10_hardware_90to150"] = float(hi.mean()) if len(hi) else None
            summary.append(entry)
    common.shared.write_csv(out / "summary.csv", summary)
    common.finish(out, rows, {"phis_deg": PHIS, "depths_mm": DEPTHS, "modules": MODULES, "hardware": HARDWARE})
    for s in summary:
        print(s["reference_id"], s["module"], s["poses"],
              {k: round(v, 3) for k, v in s.items() if k.endswith(("d10_hardware_mean", "d10_hardware_max",
                                                                   "0to60", "90to150")) and v is not None})


if __name__ == "__main__":
    main()
