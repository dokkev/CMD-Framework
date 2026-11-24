# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Check the palm housing (``bodies[palm].housing``) against every moving body's visuals and colliders.

    uv run python scripts/code_hand/check_palm_clearance.py [--hands h0 hr hu]

Ranges (the description's joint limits): the fingers, all together, over abduction -30..30 deg (5 steps), MCP-FE
-10..100 (6 steps) and PIP 0..120 (4 steps), the thumb at zero; the thumb over RO -90..90 (7), RAA -30..90 (5) and
MCP = IP -10..100 (5), the fingers at zero (once: the hands differ in the fingers only). Reports, per hand, the
smallest gap [mm] per moving body (visual and
collider; negative: they intersect) with the pose, and for the thumb the largest MCP = IP flexion free of the housing
at each RO. Writes ``logs/visuals/palm_clearance.json``.
"""

from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

import numpy as np

from code_isaaclab.assets import Description
from code_isaaclab.assets.visuals import housing_clearances
from code_isaaclab.hands.description_hands import with_thumb_pad

ROOT = Path(__file__).resolve().parents[2]
HANDS = ROOT.parents[1] / "design" / "output" / "sim" / "study3" / "hands"


def finger_grid(desc: Description) -> list[dict[str, float]]:
    poses = []
    for aa, fe, q in itertools.product(np.linspace(-30, 30, 5), np.linspace(-10, 100, 6), (0.0, 40.0, 80.0, 120.0)):
        pose = {}
        for d in desc.four_bar_digits():
            pose.update(desc.closed_pose(d, q))
            pose[f"{d}_aa"], pose[f"{d}_fe"] = float(aa), float(fe)
        poses.append(pose)
    return poses


def thumb_grid() -> list[dict[str, float]]:
    return [{"thumb_ro": float(ro), "thumb_raa": float(raa), "thumb_mcp": float(f), "thumb_ip": float(f)}
            for ro, raa, f in itertools.product(np.linspace(-90, 90, 7), np.linspace(-30, 90, 5),
                                                (-10.0, 20.0, 50.0, 80.0, 100.0))]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hands", nargs="+", default=["h0", "hr", "hu"])
    args = parser.parse_args(argv)
    report = {}
    for hand in args.hands:
        desc = with_thumb_pad(Description.load(HANDS / f"cfl_q120_h3to7_{hand}.json"))
        result = {}
        groups = [("fingers", finger_grid(desc))]
        if hand == args.hands[0]:  # the hands differ in the fingers only: the thumb once
            groups.append(("thumb", thumb_grid()))
        for group, poses in groups:
            res = housing_clearances(desc, poses)
            moving = [k for k in res if (k[0].startswith("thumb") == (group == "thumb"))]
            result[group] = {f"{b} {kind}": {"gap_mm": round(res[(b, kind)][0], 3),
                                             "pose": {j: round(v, 1) for j, v in poses[res[(b, kind)][1]].items()
                                                      if not j.endswith(("_a", "_b", "_pip_servo")) or "index" in j}}
                             for b, kind in sorted(moving, key=lambda k: res[k][0])}
        free = {}  # the thumb's largest MCP = IP flexion free of the housing, per RO (RAA at zero)
        for ro in np.linspace(-90, 90, 7) if hand == args.hands[0] else ():
            ok = -10.0
            for f in np.arange(-10.0, 100.0 + 1e-9, 10.0):
                res = housing_clearances(desc, [{"thumb_ro": float(ro), "thumb_mcp": float(f), "thumb_ip": float(f)}])
                if min(v[0] for k, v in res.items() if k[0].startswith("thumb")) < 0.0:
                    break
                ok = float(f)
            free[f"{ro:g}"] = ok
        if free:
            result["thumb_free_mcp_ip_deg_by_ro"] = free
        report[hand] = result
        print(hand)
        for group in [g for g, _ in groups]:
            items = list(result[group].items())
            print(f"  {group}: smallest gaps")
            for key, value in items[:6]:
                print(f"    {value['gap_mm']:8.3f}  {key}  {value['pose']}")
        if free:
            print(f"  thumb MCP = IP free up to [deg] by RO: {free}")
    out = ROOT / "logs" / "visuals" / "palm_clearance.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=1) + "\n")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
