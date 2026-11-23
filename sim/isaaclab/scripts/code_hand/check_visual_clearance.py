# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Check that the drawing-only plates, links and pins (``code_isaaclab.assets.visuals``) stay clear of the servo bodies.

    uv run python scripts/code_hand/check_visual_clearance.py [--hands h0 hr hu] [--step 2]

Poses: every finger closed at PIP flexion q = 0..120 deg (the description's ``four_bar`` tables, MCP and abduction at
zero, the thumb at zero); the thumb's MCP and IP together over -10..100 deg, then RO and RAA each over their range
(the fingers at q = 0). Reported, per hand, the smallest distance [mm] (negative: they intersect) of

- ``own``: the fingers' and the palm's pieces to every finger and palm servo, its own and its neighbours', over the
  flexion range; the thumb's pieces to the thumb's servos over its range (a servo's output hub excluded): the check
  of the visual layout;
- ``pads``: the pad boxes (the description's, drawn as they are) to the servos, reported;
- ``cross``: the thumb across its whole range against the palm and the fingers (and the fingers' pieces against the
  thumb's servos): reported, the thumb's range rather than the visuals.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from code_isaaclab.assets import Description
from code_isaaclab.assets.visuals import finger_poses, servo_clearances
from code_isaaclab.hands.description_hands import with_thumb_pad

ROOT = Path(__file__).resolve().parents[2]
HANDS = ROOT.parents[1] / "design" / "output" / "sim" / "study3" / "hands"


FINGERS = ("index", "middle", "ring")


def digit(name: str) -> str:
    return name.split("_")[0]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hands", nargs="+", default=["h0", "hr", "hu"])
    parser.add_argument("--step", type=float, default=2.0, help="PIP flexion step [deg]")
    args = parser.parse_args(argv)
    report = {}
    for hand in args.hands:
        desc = with_thumb_pad(Description.load(HANDS / f"cfl_q120_h3to7_{hand}.json"))
        fingers = servo_clearances(desc, finger_poses(desc, np.arange(0.0, 120.0 + 1e-9, args.step)))
        ro = next(j for j in desc.joints if j["child"] == "thumb_cmc")
        raa = next(j for j in desc.joints if j["child"] == "thumb_metacarpal")
        poses = [{"thumb_mcp": a, "thumb_ip": a} for a in np.arange(-10.0, 100.0 + 1e-9, 5.0)]
        poses += [{ro["name"]: a} for a in np.linspace(*ro["limits_deg"], 13)]
        poses += [{raa["name"]: a} for a in np.linspace(*raa["limits_deg"], 9)]
        thumb = servo_clearances(desc, poses)
        pads, own, cross = {}, {}, {}
        for result, moving in ((fingers, "finger"), (thumb, "thumb")):
            for key, value in result.items():
                piece, servo = digit(key[0]), digit(key[1].split("/")[0])
                if key[0].endswith("_pad_visual"):
                    group = pads
                elif moving == "finger" and piece in FINGERS + ("palm",) and servo in FINGERS + ("palm",):
                    group = own  # the fingers' and the palm's pieces against every finger servo, neighbours too
                elif moving == "thumb" and piece == servo == "thumb":
                    group = own
                else:
                    group = cross  # neighbouring fingers; the thumb across its range against the palm and fingers
                group[key] = min(group.get(key, np.inf), value)
        worst_own = min(own.items(), key=lambda kv: kv[1])
        report[hand] = {"own_min_mm": worst_own[1], "own_worst": list(worst_own[0]),
                        "own": {f"{p} | {s}": round(v, 3) for (p, s), v in sorted(own.items(), key=lambda kv: kv[1])},
                        "cross": {f"{p} | {s}": round(v, 3)
                                  for (p, s), v in sorted(cross.items(), key=lambda kv: kv[1]) if v < 0.5},
                        "pads": {f"{p} | {s}": round(v, 3)
                                 for (p, s), v in sorted(pads.items(), key=lambda kv: kv[1]) if v < 0.5}}
        print(f"{hand}: own min clearance {worst_own[1]:.3f} mm ({worst_own[0][0]} to {worst_own[0][1]})")
        for key, value in list(report[hand]["own"].items())[:8]:
            print(f"    {value:8.3f}  {key}")
        print(f"  pads (the description's boxes) closer than 0.5 mm to a servo: {report[hand]['pads']}")
        print(f"  cross-digit pairs closer than 0.5 mm: {len(report[hand]['cross'])}")
        for key, value in list(report[hand]["cross"].items())[:10]:
            print(f"    {value:8.3f}  {key}")
    out = ROOT / "logs" / "visuals" / "clearance.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=1) + "\n")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
