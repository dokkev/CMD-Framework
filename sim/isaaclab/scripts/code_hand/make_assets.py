# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Write the CoDe hand description as URDF (tree + loop markers) and as USD (tree + loop joints).

    uv run python scripts/code_hand/make_assets.py [--description ...] [--out-dir ...] [--loop ball|revolute]
        [--colliders] [--passive-limit-margin 0.0]

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
    suffix = "_colliders" if args.colliders else ""
    print(write_urdf(desc, out_dir / f"{stem}.urdf", passive_limit_margin_deg=args.passive_limit_margin))
    print(
        write_usd(
            desc,
            out_dir / f"{stem}_{args.loop}{suffix}.usda",
            loop=args.loop,
            colliders=args.colliders,
            passive_limit_margin_deg=args.passive_limit_margin,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
