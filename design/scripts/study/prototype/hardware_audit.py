#!/usr/bin/env python3
"""Hardware audit of the CoDe prototype (report/prototype.md) on the existing planar model.

Re-evaluates the prototype design vector (run_004 #2113, references.yaml) with
its own optimization config (proto_tilt10_align, linkage-v12) and with the
Study 1/2 config (code_tilt10.yaml). It reports, from the existing FK and
assembly model:
- the raw side-view gaps (no clearance subtracted) of every hardware pair
  that E06 flagged or that the planar model exempts by a Y-layer rule, over
  the distal motion and the MCP angles;
- each pair's model status (checked, or exempt and why);
- the pin housing sizes the gaps depend on.
It adds no geometry and no kinematics. The Y-direction (3D) check it cannot
do is listed in the hardware report as UNVERIFIED.

    python scripts/study/prototype/hardware_audit.py
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
for p in (str(ROOT), str(HERE.parent)):
    if p not in sys.path:
        sys.path.insert(0, p)

import shared  # noqa: E402
from core.morphology.mechanism.linkage.design import decode  # noqa: E402
from core.morphology.mechanism.linkage.model import four_bar  # noqa: E402
from core.morphology.mechanism.linkage.packaging import (  # noqa: E402
    assembly, driver_link_servo_gaps, layout, link_capsule_margins, own_pin_gaps, pip_footprint,
    pip_servo_pad_gap_mm, pivot_pin_gap_mm, poses, reference_poses)
from core.morphology.packaging.assembly import Pair, margins  # noqa: E402
from core.optimization.mechanism import read_config_file  # noqa: E402
from core.optimization.study import load_run  # noqa: E402

RUN = ROOT / "output/servo_close/linkage/proto_tilt10_align/run_004"
OUT = ROOT / "output/study/prototype/hardware_audit"
PHI_STEP_DEG = 1.0


def prototype_x():
    values = read_config_file(ROOT / shared.REFERENCES)["morphology"]
    return tuple(float(v) for v in values["x"])


def raw_gaps(name, x, config):
    found = shared.mechanism("linkage")
    evaluation = found.evaluate(x, config)
    report = found.report(evaluation)
    packaging = config.packaging
    finger = decode(x, config.link_length_resolution_mm)
    phi_max = float(report["phi_max_deg"])
    hyper = -math.degrees(getattr(config, "hyperextension_rad", 0.0) or 0.0)  # the required hyperextension, included
    sweep = finger.sweep_pip(np.radians(np.arange(hyper, phi_max + 1e-9, PHI_STEP_DEG)))
    built = assembly(finger, packaging)
    moving = poses(finger, sweep.b, sweep.tip, sweep.bp)
    rest = reference_poses(finger)
    lay = layout(packaging)

    def pair_gap(first, second, posed):
        key = tuple(sorted((first, second)))
        return float(np.min(margins(built, posed, packaging, [Pair(*key, "touch")], snap=False)[key]))

    keep_outs = {tuple(sorted((k.first, k.second))) for k in lay.keep_outs} if hasattr(lay, "keep_outs") else set()

    def status(first, second):
        key = tuple(sorted((first, second)))
        if key in keep_outs:
            return "checked (keep-out)"
        if key in {p.key for p in lay.pairs}:
            return "checked"
        if key in lay.unchecked:
            return "exempt (switch or pin rule)"
        parts = {p.name: p for p in lay.parts}
        if not (parts[first].layers & parts[second].layers):
            return "exempt (different Y layers)"
        return "not a pair (same body or joint)"

    footprint = pip_footprint(packaging, finger)
    mcp = tuple(config.mcp_servo_angles_rad)
    capsules = link_capsule_margins(finger, packaging, sweep.b, sweep.tip, sweep.bp, 0.0)
    clearance = packaging.body_clearance_mm
    proximal_own, distal_own = own_pin_gaps(finger, packaging)
    rows = [
        ("A pin housing - PIP servo body", pair_gap("a_pin", "pip_servo", rest), status("a_pin", "pip_servo"),
         f"a_pin_servo_check={packaging.a_pin_servo_check}"),
        ("A pin housing - PIP shaft keep-out", pivot_pin_gap_mm(packaging, finger) + 0.0, "checked" if not
         packaging.allow_collision_pip_servo_to_joint_a else "exempt (allow_collision_pip_servo_to_joint_a)",
         f"keep-out diameter {packaging.ap_clearance_diameter_mm} mm"),
        ("PIP servo - MCP servo (MCP angles)", float(packaging.mcp_servo_gap_mm(footprint, mcp)), "checked",
         f"{len(mcp)} MCP angles {math.degrees(min(mcp)):.0f}..{math.degrees(max(mcp)):.0f} deg"),
        ("B' pin housing - PIP servo body (motion)", pair_gap("bp_pin", "pip_servo", moving),
         status("bp_pin", "pip_servo"), "allow_collision_pip_servo_to_joint_bp="
         f"{packaging.allow_collision_pip_servo_to_joint_bp}"),
        ("B pin housing - PIP servo body (motion)", pair_gap("b_pin", "pip_servo", moving),
         status("b_pin", "pip_servo"), ""),
        ("driver link A-B' - PIP servo (motion, side view)",
         float(np.min(driver_link_servo_gaps(finger, footprint, sweep.bp))), status("driver_link", "pip_servo"),
         "link centre line; plate width not modelled"),
        ("coupler A'-B - PIP servo (motion, side view)", pair_gap("coupler", "pip_servo", moving),
         status("coupler", "pip_servo"), "the coupler is driven by the servo at A'"),
        ("proximal pad - PIP servo body", pip_servo_pad_gap_mm(finger, packaging), status("pip_servo", "proximal_pad"),
         f"pip_servo_pad_check={packaging.pip_servo_pad_check}"),
        ("proximal pad - own pins A, A'", proximal_own, "checked" if packaging.pads_off_pins else "exempt",
         f"pads_off_pins={packaging.pads_off_pins}"),
        ("distal pad - own pins B, B'", distal_own, "checked" if packaging.pads_off_pins else "exempt", ""),
        ("proximal pad - distal pad (motion)", pair_gap("distal_pad", "proximal_pad", moving),
         status("distal_pad", "proximal_pad"), ""),
        ("pads - crossing links (motion, centre lines)",
         float(min(np.min(v) for k, v in capsules.items() if "a_bp" in k[1] or "ap_b" in k[1])) + clearance,
         "exempt (different Y layers)", "links are centre lines; plate width not modelled"),
        ("MCP servo - proximal pad", pair_gap("mcp_servo", "proximal_pad", rest), status("mcp_servo", "proximal_pad"),
         "fixed by proximal_pad_start"),
    ]
    housings = {pin: packaging.pivot_radius_mm(pin) * 2 for pin in ("a", "ap", "b", "bp")}
    summary = {"config": name, "feasible_at_req": bool(evaluation.feasible), "phi_max_deg": phi_max,
               "stop_reason": report.get("stop_reason"), "body_clearance_mm": clearance,
               "pin_housing_diameters_mm": housings, "bearing_note": "PIVOT_CLEARANCE_DIAMETER_MM docstring: "
               "McMaster 57155K424, 3 mm shaft, 6 mm OD, 2 mm wide, 7.2 x 0.6 mm flange, in 1 mm walls",
               "servo": {"radius_mm": packaging.pip_servo.radius_mm,
                         "rear_extent_mm": packaging.pip_servo.rear_extent_mm,
                         "depth_y_mm": getattr(packaging.pip_servo, "depth_mm", None)},
               "joints_mm": {"A": (finger.joint.a.x_mm, finger.joint.a.z_mm),
                             "A'": (finger.joint.ap.x_mm, finger.joint.ap.z_mm),
                             "B": (finger.joint.b.x_mm, finger.joint.b.z_mm)},
               "gaps": [{"pair": p, "raw_gap_mm": round(g, 3), "model_status": s, "note": n} for p, g, s, n in rows]}
    return summary


def mcp_sweep(x, config, degrees=(-30, -20, -10, -5, 0, 10, 30, 60, 90)):
    """PIP servo to MCP servo gap (raw) over MCP angles; the configs sample MCP 0 only."""
    finger = decode(x, config.link_length_resolution_mm)
    footprint = pip_footprint(config.packaging, finger)
    return [{"mcp_deg": d, "gap_mm": round(float(config.packaging.mcp_servo_gap_mm(footprint, (math.radians(d),))), 3)}
            for d in degrees]


def thickness_envelope(x, config):
    """Feasibility of the prototype at its own mounting depth for module thickness
    (pad block height palmar of the mounting surface) and module length, both pads alike."""
    import dataclasses

    found = shared.mechanism("linkage")
    rows = []
    for length in (None, 25.0, 20.0):
        for height in (3.0, 4.0, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 9.0, 10.0, 12.0):
            pad = lambda q: dataclasses.replace(q, height_mm=height, length_mm=length)  # noqa: E731
            p = config.packaging
            c = dataclasses.replace(config, packaging=dataclasses.replace(
                p, proximal_pad=pad(p.proximal_pad), distal_pad=pad(p.distal_pad)))
            if length is not None:  # a shorter module lowers the minimum-support requirement to its own length
                c = dataclasses.replace(c, minimum_proximal_support_mm=min(length, c.minimum_proximal_support_mm),
                                        minimum_distal_support_mm=min(length, c.minimum_distal_support_mm))
            row = {"module_length_mm": length or "full span", "module_thickness_mm": height}
            for flexion in (120, 150):
                e = found.evaluate(x, dataclasses.replace(c, required_rom_rad=math.radians(flexion)))
                row[f"feasible_{flexion}"] = bool(e.feasible)
                row[f"failing_{flexion}"] = ";".join(sorted(k for k, v in e.constraints.items() if v > 0))
            row["phi_max_deg"] = round(float(found.report(found.evaluate(x, c))["phi_max_deg"]), 1)
            rows.append(row)
    return rows


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    x = prototype_x()
    run = load_run(RUN)
    _, study_config = shared.load_config("scripts/study/code_tilt10.yaml")
    results = [raw_gaps("prototype run config (proto_tilt10_align run_004, linkage-v12 goal)", x, run.config),
               raw_gaps("study config (code_tilt10.yaml)", x, study_config)]
    envelope = thickness_envelope(x, run.config)
    shared.write_csv(OUT / "thickness_envelope.csv", envelope)
    record = {"design_x": x, "source": shared.git_commit(OUT), "results": results,
              "mcp_sweep_prototype_config": mcp_sweep(x, run.config),
              "thickness_envelope": "thickness_envelope.csv (prototype config, mounting depth 3.0 mm both pads)",
              "kinematics": "existing FK (decode + sweep_pip); no geometry added"}
    (OUT / "audit.json").write_text(json.dumps(record, indent=2, default=str) + "\n")
    for r in results:
        print("==", r["config"], "| feasible", r["feasible_at_req"], "| phi_max", round(r["phi_max_deg"], 1),
              "| clearance", r["body_clearance_mm"], "| housings", r["pin_housing_diameters_mm"])
        for g in r["gaps"]:
            print(f"  {g['raw_gap_mm']:8.3f}  {g['pair']:48s} {g['model_status']:34s} {g['note']}")
    print("MCP sweep:", record["mcp_sweep_prototype_config"])
    for r in envelope:
        print(r["module_length_mm"], r["module_thickness_mm"], r["feasible_120"], r["feasible_150"], r["phi_max_deg"])


if __name__ == "__main__":
    main()
