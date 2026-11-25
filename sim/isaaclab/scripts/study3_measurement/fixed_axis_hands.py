# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3 measurement (problem_formulation A.6.4): fixed-axis contrast hands for Study 3 (report/study3/study3.md, 14th
experiment).

    python3 scripts/study3_measurement/fixed_axis_hands.py

Builds ``code_hand_sim/1`` descriptions of a fixed-axis finger family from the 3 mm CFL reference O: each finger's PIP
is one revolute joint on the link line at the centre of O's joint gap (x_a), driven directly (theta = q). The family
keeps the skeleton exactly (same links, axis and fingertip path; Delta_skel = 0) and changes only the pad thickness
h in {3, 5, 7, 9} mm at O's mount depth (3 mm). Each pad's joint-side end sits at the clearance law's gap
G = 2 (d + h) tan(q_req / 2) (Eq. gsym, z = 0, q_req 120 deg) split evenly about the axis; the outer ends stay at
the MCP and the fingertip. In the proximal-phalanx frame the axis lies d + h above the face, so the contact-surface
motion changes by Delta_contact = 2 sin(q / 2) dh (max sqrt(3) dh at 120 deg), the fixed-axis bound with equality.

The four-bar's tree is kept so that the flexion action and its tables work unchanged: ``<d>_pip_servo`` now joins
the proximal phalanx and the servo link at the axis, and the servo link carries the distal phalanx's link, mass and
pad (two_pad_wrench.py reads it as the distal pad for ``fa_*`` hands). ``<d>_distal`` and ``<d>_driver_link`` are
unloaded dummies on ``<d>_b`` and ``<d>_a``; there is no loop, and the table is the identity (ap = pip, a = b = 0).
The tree, and so the order of the contact sensor's body columns, is the four-bar's.

Also writes the placements ``fa_grasps.json``: the cylinder where the 3 mm finger touches it with both pads at q0,
x_c = x_a - (d + h + rho) tan(q0 / 2) from the MCP along the face, kept where both tangent points lie on its pads.
"""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
HAND_DIR = ROOT / "design" / "output" / "sim" / "study3" / "hands"
GRASPS = ROOT / "design" / "output" / "sim" / "study3" / "fa_grasps.json"
FINGERS = ("index", "middle", "ring")
MOUNT_M, Q_REQ = 0.003, math.radians(120.0)
THICKNESS_MM = (3, 5, 7, 9)
DUMMY_R = 0.0005


def body(desc: dict, name: str) -> dict:
    return next(b for b in desc["bodies"] if b["name"] == name)


def joint(desc: dict, name: str) -> dict:
    return next(j for j in desc["joints"] if j["name"] == name)


def axis_x(o: dict, f: str) -> float:
    """Centre of O's joint gap along x (proximal pad end, distal pad start)."""
    pp, dp = body(o, f"{f}_proximal")["boxes"][0], body(o, f"{f}_distal")["boxes"][0]
    return 0.5 * ((pp["center"][0] + pp["half_extents"][0]) + (dp["center"][0] - dp["half_extents"][0]))


def build(o: dict, h_mm: float) -> dict:
    d = copy.deepcopy(o)
    h = h_mm * 1e-3
    gap = 2.0 * (MOUNT_M + h) * math.tan(Q_REQ / 2.0)
    d["source"] = {**d["source"], "derived": f"fixed_axis_hands.py from cfl_q120_h3to7_h0.json, h {h_mm:g} mm"}
    d["loops"] = []
    for f in FINGERS:
        prox, dist = body(d, f"{f}_proximal"), body(d, f"{f}_distal")
        servo, driver = body(d, f"{f}_servo_link"), body(d, f"{f}_driver_link")
        servo_mass = servo["mass"]
        x0, y, z = prox["origin"]
        xa = axis_x(o, f)
        xt = o["tips"][f][0]
        zc = z - MOUNT_M - h / 2.0
        # proximal phalanx: link to the axis, pad from the MCP to the gap
        prox["capsule"] = {"start": [x0, y, z], "end": [xa, y, z], "radius": prox["capsule"]["radius"]}
        prox["com"] = [0.5 * (x0 + xa), y, z]
        p1 = xa - gap / 2.0
        pb = prox["boxes"][0]
        pb["center"], pb["half_extents"] = [0.5 * (x0 + p1), y, zc], [0.5 * (p1 - x0), h / 2.0, pb["half_extents"][2]]
        # the four-bar's tree is kept, because the contact sensor's body columns follow it. The servo link turns about
        # the axis and carries the distal phalanx's link, mass and pad (the analysis reads it as the distal pad);
        # <d>_distal is an unloaded dummy on <d>_b, and the driver link a dummy inside the proximal phalanx
        zs = z + 0.002  # dorsal of the link line, out of the object's reach
        d1 = xa + gap / 2.0
        db = dist["boxes"][0]
        db["center"], db["half_extents"] = [0.5 * (d1 + xt), y, zc], [0.5 * (xt - d1), h / 2.0, db["half_extents"][2]]
        servo.update(origin=[xa, y, z], mass=dist["mass"], com=[0.5 * (xa + xt), y, z], boxes=[db],
                     capsule={"start": [xa, y, z], "end": [xt, y, z], "radius": dist["capsule"]["radius"]})
        dist.update(origin=[xa, y, z], mass=servo_mass, com=[xa + 0.005, y, zs],
                    capsule={"start": [xa + 0.004, y, zs], "end": [xa + 0.006, y, zs], "radius": DUMMY_R})
        dist.pop("boxes", None)
        d["tips"][f] = [xt, y, z]
        driver["origin"] = [x0 + 0.010, y, zs]
        driver["capsule"] = {"start": [x0 + 0.010, y, zs], "end": [x0 + 0.012, y, zs], "radius": DUMMY_R}
        driver["com"] = [x0 + 0.011, y, zs]
        js, jb, ja = joint(d, f"{f}_pip_servo"), joint(d, f"{f}_b"), joint(d, f"{f}_a")
        js.update(parent=f"{f}_proximal", child=f"{f}_servo_link", anchor=[xa, y, z], limits_deg=[-10.0, 125.0])
        js["actuator"] = {**js["actuator"], "zero_deg": 0.0}
        jb.update(parent=f"{f}_servo_link", child=f"{f}_distal", anchor=[xa, y, z], limits_deg=[-0.05, 0.05])
        ja.update(parent=f"{f}_proximal", child=f"{f}_driver_link", anchor=list(driver["origin"]), limits_deg=[-0.05, 0.05])
        t = d["four_bar"][f]
        t["ap_deg"] = list(t["pip_deg"])
        t["a_deg"] = [0.0] * len(t["pip_deg"])
        t["b_deg"] = [0.0] * len(t["pip_deg"])
        pairs = d["collision"]["filtered_pairs"]
        for a, b in ((f"{f}_proximal", f"{f}_distal"), (f"{f}_distal", f"{f}_servo_link"),
                     (f"{f}_distal", f"{f}_driver_link"), (f"{f}_proximal", f"{f}_servo_link"),
                     (f"{f}_servo_link", f"{f}_driver_link")):
            if [a, b] not in pairs and [b, a] not in pairs:
                pairs.append([a, b])
    xa_m, x0_m = axis_x(o, "middle"), body(o, "middle_proximal")["origin"][0]
    d["pads"] = {**d["pads"], "thickness_mm": h_mm, "mount_offset_mm": MOUNT_M * 1e3,
                 "proximal_start_mm": 0.0, "proximal_end_mm": (xa_m - gap / 2.0 - x0_m) * 1e3,
                 "fixed_axis_mm": (xa_m - x0_m) * 1e3, "gap_mm": gap * 1e3}
    return d


def grasps(o: dict) -> dict:
    """Placements where the 3 mm fixed-axis finger touches the cylinder with both pads at q0 (middle finger)."""
    x0 = body(o, "middle_proximal")["origin"][0]
    xa = axis_x(o, "middle") - x0
    xt = o["tips"]["middle"][0] - x0 - xa  # axis to fingertip along the distal face
    t = MOUNT_M + 0.003
    gap = 2.0 * t * math.tan(Q_REQ / 2.0)
    out = []
    for rho_mm in (15.0, 20.0):
        rho = rho_mm * 1e-3
        for q0 in range(40, 121, 5):
            s = (t + rho) * math.tan(math.radians(q0) / 2.0)  # tangent points' distance from the axis foot
            if xa - s < 0.002 or s < gap / 2.0 + 0.002 or s > xt - 0.002:
                continue
            out.append({"rho_mm": rho_mm, "q0_deg": float(q0), "x_c_mm": (xa - s) * 1e3, "O": float(q0)})
    return {"flexion_deg": 120.0, "note": "fixed-axis family: where its 3 mm finger touches with both pads at q0 "
            "(report/study3/study3.md, 14th experiment)", "axis_mm": xa * 1e3, "grasps": out}


def main() -> int:
    o = json.loads((HAND_DIR / "cfl_q120_h3to7_h0.json").read_text())
    for h in THICKNESS_MM:
        path = HAND_DIR / f"fa_q120_h{h}.json"
        path.write_text(json.dumps(build(o, h), indent=1))
        print(path, f"gap {2 * (MOUNT_M + h * 1e-3) * math.tan(Q_REQ / 2) * 1e3:.1f} mm, "
                    f"Delta_contact {2 * math.sin(Q_REQ / 2) * (h - 3):.2f} mm")
    g = grasps(o)
    GRASPS.write_text(json.dumps(g, indent=1))
    print(GRASPS, len(g["grasps"]), "placements, axis", f"{g['axis_mm']:.1f} mm from the MCP")
    for e in g["grasps"]:
        print(f"  rho {e['rho_mm']:g} q0 {e['q0_deg']:g}: x_c {e['x_c_mm']:.1f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
