# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Write the CoDe hand description as URDF (tree + loop markers) and as USD (tree + loop joints).

    uv run python scripts/code_hand/make_assets.py [--description ...] [--out-dir ...] [--loop ball|revolute]
        [--colliders] [--passive-limit-margin 0.0] [--geometry] [--thumb-pad] [--visuals] [--no-usd]

``--geometry`` writes the URDF with collision, coloured visuals (servo bodies as OBJ, beside it) and transmissions;
``--thumb-pad`` adds the sim-side thumb pad (``code_isaaclab.hands.description_hands.thumb_pad_box``) first; the
outputs then carry ``_thumbpad`` in their names. The URDF is plain: the four-bar loops are closed by the loader from
the description's ``loops`` (Drake: ``sim/drake/load_hand.py``). ``--visuals`` gives both the drawing-only geometry
(:mod:`code_isaaclab.assets.visuals`: plates, four-bar links, pins, pads, servo bodies; OBJ meshes beside the URDF),
with ``_vis`` in the names; collision and physics are as without it.

    uv run python scripts/code_hand/make_assets.py --description <hand>.json --out-dir logs/urdf \
        --geometry --thumb-pad --visuals --colliders

See :mod:`code_isaaclab.assets.builders` for the URDF and USD authoring conventions.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from code_isaaclab.assets import DEFAULT_DESCRIPTION_PATH, Description, write_urdf, write_usd


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--description", type=Path, default=DEFAULT_DESCRIPTION_PATH)
    parser.add_argument("--out-dir", type=Path, default=None, help="defaults to the description's parent directory")
    parser.add_argument("--loop", choices=("ball", "revolute"), default="ball")
    parser.add_argument("--colliders", action="store_true")
    parser.add_argument("--geometry", action="store_true", help="URDF with collision, visuals and transmissions")
    parser.add_argument("--thumb-pad", action="store_true", help="add the sim-side thumb pad")
    parser.add_argument("--visuals", action="store_true", help="add the drawing-only geometry (URDF and USD)")
    parser.add_argument("--no-usd", action="store_true", help="write only the URDF")
    parser.add_argument(
        "--passive-limit-margin",
        type=float,
        default=0.0,
        help="widen passive tree-joint limits by this margin [deg] to avoid simultaneous stop/loop locking",
    )
    args = parser.parse_args(argv)
    out_dir = args.description.parent if args.out_dir is None else args.out_dir
    desc = Description.load(args.description)
    stem = args.description.stem
    if args.thumb_pad:
        from code_isaaclab.hands.description_hands import with_thumb_pad

        desc = with_thumb_pad(desc)
        stem += "_thumbpad"
    if args.visuals:
        stem += "_vis"
    suffix = "_colliders" if args.colliders else ""
    print(write_urdf(desc, out_dir / f"{stem}.urdf", passive_limit_margin_deg=args.passive_limit_margin,
                     geometry=args.geometry, visuals=args.visuals))
    if args.no_usd:
        return 0
    print(
        write_usd(
            desc,
            out_dir / f"{stem}_{args.loop}{suffix}.usda",
            loop=args.loop,
            colliders=args.colliders,
            passive_limit_margin_deg=args.passive_limit_margin,
            visuals=args.visuals,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
