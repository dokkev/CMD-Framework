#!/usr/bin/env python3
"""Export the CoDe hand's parametric model as a simulator-neutral kinematic description (JSON).

    python scripts/export/code_hand_sim.py [--params core/morphology/code_hand/prototype.yaml]
                                           [--out output/sim/code_hand.json]

The description is what ``sim/`` builds its robot assets from, so ``sim/`` never imports ``core``:
- ``bodies``: each rigid body's origin in the hand frame at the geometric zero (all joints 0), its mass and
  centre of mass, and a capsule (start, end, radius) for drawing and contact. Orientation is the hand frame's.
  With pads in the parameters, a finger's ``proximal`` and ``distal`` bodies also carry their pad as a box
  (``boxes``: centre, orientation as a ``wxyz`` quaternion in the hand frame, half extents), and their capsule
  radius is the pads' mounting offset, so the link structure stays above the pad's mounting face.
- ``collision``: ``filtered_pairs`` of bodies that must not collide with each other: every pair within one digit
  (the four-bar's links overlap at their pins by construction) except a finger's proximal and distal segments,
  whose pads must keep clear of each other, and the palm with every body (its capsule is a placeholder).
  Different digits collide with each other.
- ``joints``: the spanning tree, each a revolute about ``axis`` through ``anchor`` (hand frame, at zero), CCW
  positive, with its ``actuator`` (if a servo drives it) and limits.
- ``loops``: the cut joints that close each finger's four-bar (``B'``: driver link to distal segment). URDF
  cannot express them; a simulator closes them (Newton: a joint outside the articulation, USD:
  ``physics:excludeFromArticulation``).
- ``servo``: the XM335 drive model every actuated joint uses (``core.hardware.xm335``): datasheet values at the
  supply voltage, the position gain and the assumptions they rest on. Actuated joints name it in ``actuator.servo``.
- ``four_bar``: per finger, the tree joint angles of the four-bar as tables of the PIP angle, so a simulator can
  start in a closed pose and report the PIP angle without re-implementing the closure.

Units: metres, degrees, kilograms. Masses and inertias are placeholders (servo masses from the X330 datasheet
plus rough structure); they are marked as such and must be replaced before dynamics are interpreted.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import subprocess
import sys
from datetime import UTC, datetime
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.hardware.xm335 import XM335DriveModel
from core.morphology.code_hand import FINGERS, CoDeHand, four_bar, load, pip_servo_deg
from core.morphology.code_hand.parameters import PROTOTYPE

MM = 1e-3
SERVO_KG = 0.027  # X330 (core.hardware.xm335)
PIP_RANGE_DEG = (-10.0, 120.0)  # the studies' requirement: 10 deg hyperextension, 120 deg flexion
TABLE_STEP_DEG = 1.0
# Placeholder joint ranges [deg] for the direct-drive joints (no hardware stops are modelled yet).
LIMITS = {"aa": (-30.0, 30.0), "fe": (-10.0, 100.0), "ro": (-90.0, 90.0), "raa": (-30.0, 90.0),
          "mcp": (-10.0, 100.0), "ip": (-10.0, 100.0)}


def _cw_xz(u: np.ndarray, v: np.ndarray) -> float:
    """The clockwise angle [deg] from ``u`` to ``v`` in the linkage plane (x, z dorsal): CCW about the FE axis."""
    return -math.degrees(math.atan2(u[0] * v[1] - u[1] * v[0], u @ v))


def _m(point) -> list[float]:
    return [round(float(v) * MM, 9) for v in point]


def _body(name, origin, mass, com, start, end, radius_mm, servos=0):
    return {"name": name, "origin": _m(origin), "mass": mass + servos * SERVO_KG, "com": _m(com),
            "capsule": {"start": _m(start), "end": _m(end), "radius": radius_mm * MM}}


def _quat_wxyz(r: np.ndarray) -> list[float]:
    """A rotation matrix's unit quaternion ``(w, x, y, z)``, ``w >= 0``."""
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    q = np.array([w, x, y, z]) / math.sqrt(w * w + x * x + y * y + z * z)
    return [round(float(v), 9) for v in q]


def _box(name, pad) -> dict:
    return {"name": name, "center": _m(pad.frame[:3, 3]), "quat_wxyz": _quat_wxyz(pad.frame[:3, :3]),
            "half_extents": _m(pad.half_extents_mm)}


def _joint(name, parent, child, anchor, axis, limits, actuator=None):
    return {"name": name, "parent": parent, "child": child, "anchor": _m(anchor),
            "axis": [round(float(v), 9) for v in axis], "limits_deg": list(limits), "actuator": actuator}


def four_bar_table(finger) -> dict:
    """Tree joint angles [deg] of the four-bar against the PIP angle: ``a`` (driver link about ``A``), ``ap``
    (servo link about ``A'``, the PIP servo) and ``b`` (distal about ``B``, relative to the servo link)."""
    zero = four_bar(finger, 0.0)
    pip = np.arange(PIP_RANGE_DEG[0], PIP_RANGE_DEG[1] + 1e-9, TABLE_STEP_DEG)
    a, ap, b = [], [], []
    for q in pip:
        pins = four_bar(finger, float(q))
        a.append(_cw_xz(zero.bp - zero.a, pins.bp - pins.a))
        ap.append(_cw_xz(zero.b - zero.ap, pins.b - pins.ap))
        b.append(float(q) - ap[-1])  # the distal turns pip about parallel axes: ap + b = pip
    return {"pip_deg": pip.round(6).tolist(), "a_deg": np.round(a, 6).tolist(), "ap_deg": np.round(ap, 6).tolist(),
            "b_deg": np.round(b, 6).tolist()}


def export(params: Path, supply_voltage_v: float = 11.1) -> dict:
    parameters, _ = load(params)
    model = CoDeHand(parameters)
    frames = model.frames()
    f, t = parameters.finger, parameters.thumb
    bodies = [{"name": "palm", "origin": [0.0, 0.0, 0.0], "mass": 0.3, "com": [0.0, 0.0, 0.0], "fixed": True,
               "capsule": {"start": [0.0, 0.0, 0.0], "end": [0.0, -0.05, 0.0], "radius": 0.02}}]
    joints, loops, tables = [], [], {}
    servo_zero = pip_servo_deg(f, 0.0)
    pads = parameters.pads
    link_radius = 6.0 if pads is None else pads.mount_offset_mm
    digit_bodies = {}
    for d in FINGERS:
        aa, fe, a, ap, b, bp = (frames.axes[f"{d}/{k}"] for k in ("aa", "fe", "A", "Ap", "B", "Bp"))
        c = frames.points[f"{d}/C"]
        table = four_bar_table(f)
        tables[d] = table
        bodies += [
            _body(f"{d}_mcp", aa[0], 0.008, (aa[0] + fe[0]) / 2, aa[0], fe[0], 6.0, servos=1),
            _body(f"{d}_proximal", fe[0], 0.012, (fe[0] + ap[0]) / 2, fe[0], a[0], link_radius, servos=1),
            _body(f"{d}_servo_link", ap[0], 0.003, (ap[0] + b[0]) / 2, ap[0], b[0], 2.5),
            _body(f"{d}_driver_link", a[0], 0.003, (a[0] + bp[0]) / 2, a[0], bp[0], 2.5),
            _body(f"{d}_distal", b[0], 0.010, (b[0] + c) / 2, bp[0], c, link_radius),
        ]
        digit_bodies[d] = [body["name"] for body in bodies[-5:]]
        if pads is not None:
            bodies[-4]["boxes"] = [_box(f"{d}_proximal_pad", frames.pads[f"{d}/proximal_pad"])]
            bodies[-1]["boxes"] = [_box(f"{d}_distal_pad", frames.pads[f"{d}/distal_pad"])]
        joints += [
            _joint(f"{d}_aa", "palm", f"{d}_mcp", *aa, LIMITS["aa"],
                   {"zero_deg": f.aa_actuator_zero_deg}),
            _joint(f"{d}_fe", f"{d}_mcp", f"{d}_proximal", *fe, LIMITS["fe"],
                   {"zero_deg": f.fe_actuator_zero_deg}),
            # The PIP servo turns the servo link: actuator = servo_zero + q - pip_actuator_zero_deg.
            _joint(f"{d}_pip_servo", f"{d}_proximal", f"{d}_servo_link", *ap,
                   (min(table["ap_deg"]), max(table["ap_deg"])),
                   {"zero_deg": f.pip_actuator_zero_deg - servo_zero}),
            _joint(f"{d}_b", f"{d}_servo_link", f"{d}_distal", *b, (min(table["b_deg"]), max(table["b_deg"]))),
            _joint(f"{d}_a", f"{d}_proximal", f"{d}_driver_link", *a, (min(table["a_deg"]), max(table["a_deg"]))),
        ]
        loops.append({"name": f"{d}_bp", "parent": f"{d}_driver_link", "child": f"{d}_distal",
                      "anchor": _m(bp[0]), "axis": [round(float(v), 9) for v in bp[1]]})
    ro, raa, mcp, ip = (frames.axes[f"thumb/{k}"] for k in ("ro", "raa", "mcp", "ip"))
    tip = frames.points["thumb/tip"]
    bodies += [
        _body("thumb_cmc", raa[0], 0.008, (ro[0] + raa[0]) / 2, ro[0], raa[0], 6.0, servos=1),
        # The MCP servo body rides on the thumb's distal side, with the IP servo: both on thumb_proximal.
        _body("thumb_metacarpal", mcp[0], 0.008, (raa[0] + mcp[0]) / 2, raa[0], mcp[0], 6.0),
        _body("thumb_proximal", mcp[0], 0.008, (mcp[0] + ip[0]) / 2, mcp[0], ip[0], 6.0, servos=2),
        _body("thumb_distal", ip[0], 0.010, (ip[0] + tip) / 2, ip[0], tip, 6.0),
    ]
    digit_bodies["thumb"] = [body["name"] for body in bodies[-4:]]
    filtered = [["palm", body["name"]] for body in bodies[1:]]
    filtered += [[p, q] for names in digit_bodies.values() for i, p in enumerate(names) for q in names[i + 1:]
                 if {p.split("_", 1)[1], q.split("_", 1)[1]} != {"proximal", "distal"}]
    joints += [
        _joint("thumb_ro", "palm", "thumb_cmc", *ro, LIMITS["ro"], {"zero_deg": t.ro_actuator_zero_deg}),
        _joint("thumb_raa", "thumb_cmc", "thumb_metacarpal", *raa, LIMITS["raa"],
               {"zero_deg": t.raa_actuator_zero_deg}),
        _joint("thumb_mcp", "thumb_metacarpal", "thumb_proximal", *mcp, LIMITS["mcp"],
               {"zero_deg": t.mcp_actuator_zero_deg}),
        _joint("thumb_ip", "thumb_proximal", "thumb_distal", *ip, LIMITS["ip"], {"zero_deg": t.ip_actuator_zero_deg}),
    ]
    for joint in joints:
        if joint["actuator"] is not None:
            joint["actuator"]["servo"] = "xm335"
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True,
                            check=False).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "core", str(params)], cwd=ROOT,
                                capture_output=True, text=True, check=False).stdout.strip())
    return {
        "format": "code_hand_sim/1",
        "source": {"commit": commit, "dirty": dirty, "parameters": str(Path(params).resolve().relative_to(ROOT.parent)),
                   "date": datetime.now(UTC).date().isoformat()},
        "units": {"length": "m", "angle": "deg", "mass": "kg"},
        "conventions": "Joint angles CCW about axis; flexion positive. actuator angle = joint angle - zero_deg. "
                       "All bodies take the hand frame's orientation at the geometric zero.",
        "placeholders": ["mass", "com", "inertia (rod about the capsule)", "direct-drive joint limits",
                         "servo armature", "capsules (palm, links, thumb)", "pad width"],
        "bodies": bodies,
        "collision": {"filtered_pairs": filtered},
        "pads": None if pads is None else dataclasses.asdict(pads),
        "joints": joints,
        "loops": loops,
        "servo": XM335DriveModel(supply_voltage_v=supply_voltage_v).as_dict(),
        "four_bar": tables,
        "tips": {**{d: _m(frames.points[f"{d}/C"]) for d in FINGERS}, "thumb": _m(tip)},
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--params", type=Path, default=PROTOTYPE)
    parser.add_argument("--out", type=Path, default=ROOT / "output" / "sim" / "code_hand.json")
    parser.add_argument("--supply-voltage", type=float, default=11.1, help="XM335 supply [V] (rated 6.5-12)")
    args = parser.parse_args(argv)
    data = export(args.params, args.supply_voltage)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(data, indent=1) + "\n")
    boxes = sum(len(body.get("boxes", [])) for body in data["bodies"])
    print(f"{args.out}: {len(data['bodies'])} bodies, {len(data['joints'])} tree joints, {len(data['loops'])} loops, "
          f"{boxes} pads, "
          f"source {data['source']['commit'][:8]}{' (dirty)' if data['source']['dirty'] else ''}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
