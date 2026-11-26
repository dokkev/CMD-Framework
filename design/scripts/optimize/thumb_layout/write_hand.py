#!/usr/bin/env python3
"""Write a hand parameter file with an optimised thumb layout (``core.optimization.thumb_layout``).

    python scripts/optimize/thumb_layout/write_hand.py --hand h0 --x -51 --z -4.5 --extension 42 \
        --orientation proximal [--yaw 0 --pitch 0 --roll 0] --out output/sim/study3/hands/cfl_q120_h3to7_h0_thumbopt.yaml

The file extends the morphology's own (``base:``) and sets the thumb base pose, ``mcp_to_ip_mm``, ``ee_y_mm`` and
``ip_servo_flipped`` as ``ThumbDesign`` derives them; ``scripts/export/code_hand_sim.py --params`` exports it.
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import asdict
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.morphology.code_hand import load  # noqa: E402
from core.optimization.thumb_layout import ThumbDesign, hand_parameters, lever_arms  # noqa: E402

HANDS = ROOT / "output" / "sim" / "study3" / "hands"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--hand", default="h0")
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--extension", type=float, required=True)
    parser.add_argument("--orientation", choices=("distal", "proximal"), required=True)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--note", default="", help="a line recorded in source.note")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    base_path = HANDS / f"cfl_q120_h3to7_{args.hand}.yaml"
    parameters, _ = load(base_path)
    design = ThumbDesign(args.x, args.z, args.extension, args.orientation, args.yaw, args.pitch, args.roll)
    hand = hand_parameters(parameters, design)
    data = {
        "base": os.path.relpath(base_path, args.out.resolve().parent),
        "layout": {"thumb": asdict(hand.layout.thumb)},
        "thumb": {"mcp_to_ip_mm": hand.thumb.mcp_to_ip_mm, "ee_y_mm": hand.thumb.ee_y_mm,
                  "ip_servo_flipped": design.ip_orientation == "proximal"},
        "source": {"study": "thumb layout optimisation (docs/experimental/thumb_layout_opt.md)",
                   "design": {k: v for k, v in design.__dict__.items()}, "levers_mm": lever_arms(design),
                   "note": args.note},
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text("# Thumb-layout optimised hand: written by scripts/optimize/thumb_layout/write_hand.py\n"
                        + yaml.safe_dump(data, sort_keys=False))
    check, _ = load(args.out)
    assert check.thumb.ip_servo_flipped == (design.ip_orientation == "proximal")
    assert abs(check.layout.thumb.x_mm - args.x) < 1e-9 and check.finger == parameters.finger
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
