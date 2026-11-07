#!/usr/bin/env python3
"""Packaging trade-off: finger thickness against contact accommodation.

Measures the extended-pose side-view packaging of each matched reference,
and of each servo tilt of the E04 sweep, from the existing scene solids
(`core.morphology.contact.loss`). The baseline 20 x 3 mm module pair is
used, at policy depths. The palm-mounted MCP servo is excluded. The
measures are:

- ``dorsal_top_mm``: the highest point of the finger hardware (links, pins,
  servos, pulleys, rollers, carrier);
- ``pad_bottom_mm``: the lowest point of the two pads, i.e. the palmar
  contact surface;
- ``hardware_bottom_mm``: the lowest point of the finger hardware;
- ``h_finger_mm = dorsal_top - pad_bottom``, the primary metric;
- ``h_bbox_mm = max(top) - min(bottom)`` over hardware and pads, the full
  envelope. It differs from ``h_finger`` when hardware lies below the pads.

Accommodation comes from saved results: Study 1 (``assembly_results.csv``)
for the references, and E04 (``summary.csv``) for the tilt sweep. Nothing is
re-optimized. Bars are drawn as capsules of the goal's link radius (0 in the
study configs).

    python scripts/study/exploratory/packaging_tradeoff.py
"""
from __future__ import annotations

import csv
import dataclasses
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
from core.optimization.analysis.transmission import Transmission  # noqa: E402

HARDWARE = ("link", "pin", "servo", "pulley", "roller", "carrier")
STUDY1 = common.ROOT / "output/study/study1_fixed_morphology/assembly_results.csv"
E04 = common.OUTPUT / "e04_actuator_orientation/summary.csv"
TILT_BASES = {"code_matched": "code_matched", "2r_dd_tilt0_matched": "2r_dd", "belt_2r_matched": "belt_2r_matched",
              "rcj_tilt0_matched": "rcj"}


def extremes(shapes_list, link_radius=0.0):
    """Highest and lowest z (mm) over the solids, at the scene's first pose."""
    zs = []
    for shapes in shapes_list:
        for polygon in shapes.polygons:
            polygon = np.asarray(polygon, dtype=float)
            zs.append((polygon[0] if polygon.ndim == 3 else polygon)[:, 1])
        for segment in shapes.segments:
            segment = np.asarray(segment, dtype=float)
            z = (segment[0] if segment.ndim == 3 else segment)[:, 1]
            zs.append(np.concatenate((z - link_radius, z + link_radius)))
        for centers, radius in shapes.disks:
            centers = np.asarray(centers, dtype=float)
            z = (centers[0] if centers.ndim == 2 else centers)[1]
            zs.append(np.array((z - radius, z + radius)))
    every = np.concatenate(zs) if zs else np.array([math.nan])
    return float(every.max()), float(every.min())


def scene_of(reference, modules):
    x, config = common.carried(reference, modules)
    if x is None:
        return None, None, None
    return Transmission(reference.mechanism, x, config).scene(0.0), x, config


def measure(scene, config):
    solids = scene.solids["proximal_pad"]
    link_radius = getattr(common.goal_of(config), "link_radius_mm", None) or 0.0
    top, hw_bottom = extremes([solids[k] for k in HARDWARE], link_radius)
    pads = [solids["other_pad"], scene.solids["distal_pad"]["other_pad"]]
    pad_top, pad_bottom = extremes(pads)
    return {"dorsal_top_mm": top, "hardware_bottom_mm": hw_bottom, "pad_bottom_mm": pad_bottom,
            "h_finger_mm": top - pad_bottom,
            "h_bbox_mm": max(top, pad_top) - min(hw_bottom, pad_bottom),
            "hardware_below_pads": bool(hw_bottom < pad_bottom - 1e-9)}


def with_tilt(reference, tilt):
    """The E04 construction: the tilt in the vector and as the goal's actuation input."""
    names = common.mechanism(reference.mechanism).variable_names
    x = list(reference.x)
    x[names.index("servo_tilt_deg")] = tilt
    goal = common.goal_of(reference.config)
    config = common.with_goal(reference.config, dataclasses.replace(
        goal, pip_servo_tilt_rad=math.radians(tilt), bounds={**goal.bounds, "servo_tilt_deg": (tilt, tilt)}))
    return dataclasses.replace(reference, x=tuple(x), config=config)


def study1_counts():
    counts = {}
    with STUDY1.open() as stream:
        for r in csv.DictReader(stream):
            c = counts.setdefault(r["reference_id"], {"mounted": 0, "feasible_90": 0, "feasible_120": 0,
                                                       "feasible_150": 0})
            flexion = int(float(r["required_flexion_deg"]))
            if flexion == 120 and r["mounted"] == "True":
                c["mounted"] += 1
            if r["feasible"] == "True":
                c[f"feasible_{flexion}"] += 1
    return counts


def main():
    refs, cat = common.references(), common.catalog()
    baseline = cat["baseline"].modules
    out = common.output("packaging_tradeoff")
    counts = study1_counts()
    reference_rows = []
    for rid, ref in refs.items():
        scene, x, config = scene_of(ref, baseline)
        names = common.mechanism(ref.mechanism).variable_names
        tilt = float(ref.x[names.index("servo_tilt_deg")]) if "servo_tilt_deg" in names else None
        reference_rows.append({"reference_id": rid, "label": common.LABEL[rid], "mechanism": ref.mechanism,
                               "upper_bound": ref.upper_bound, "servo_tilt_deg": tilt,
                               "module": "baseline 20x3 / 20x3", **measure(scene, config), **counts[rid],
                               "x": ";".join(repr(float(v)) for v in x)})
    sweep = {(r["reference_id"], float(r["tilt_deg"])): r for r in csv.DictReader(E04.open())}
    tilt_rows = []
    for (base, tilt), saved in sorted(sweep.items()):
        ref = with_tilt(refs[base], tilt)
        scene, x, config = scene_of(ref, baseline)
        row = {"base_reference": base, "architecture": TILT_BASES[base], "tilt_deg": tilt,
               "carried_120": int(saved["carried"]), "mounted": int(saved["mounted"]),
               "reference_feasible": saved["baseline_feasible"] == "True",
               "reference_failure": saved["baseline_failure"]}
        if scene is None:
            tilt_rows.append({**row, "baseline_mounts": False})
            continue
        tilt_rows.append({**row, "baseline_mounts": True, **measure(scene, config)})
    common.shared.write_csv(out / "references.csv", reference_rows)
    common.shared.write_csv(out / "tilt_sweep.csv", tilt_rows)
    common.finish(out, [], {"module": "baseline", "pose_deg": 0.0, "hardware": HARDWARE,
                            "excluded": "mcp_servo (palm)", "flexion_for_tilt_counts_deg": 120.0},
                  inputs={"study1": str(STUDY1.relative_to(common.ROOT)), "e04": str(E04.relative_to(common.ROOT))},
                  metrics={"h_finger_mm": "dorsal hardware top - lowest pad surface (extended, MCP servo excluded)",
                           "h_bbox_mm": "full extended assembly height, hardware and pads, MCP servo excluded"})
    for r in reference_rows:
        print({k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items() if k != "x"})


if __name__ == "__main__":
    main()
