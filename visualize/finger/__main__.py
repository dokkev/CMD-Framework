"""Any mechanism's design through its motion: the sketch at every ``--step``
of distal flexion up to its safe flexion (colliding parts outlined red),
over its distal pad's trail and its centre of rotation's path, and (``--gaps``)
each checked pair's gap.

    python -m visualize.finger <seeds.csv | study dir> [role]
    python -m visualize.finger --mechanism rcj --config <config.yaml> --x 30,6,40,... [--set key=value ...]

The first: a verified front's design (a study dir: its latest portfolio;
``role``: the representative, default ``balanced``). The second: any design
vector of a mechanism under a study config file. Options: ``--phi`` poses
instead of ``--step`` (deg, default 30), ``--to`` the last pose (default the
safe flexion, without a front the joint's reach), ``--output`` the name.
Saves output/finger/<name>.png.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from visualize.finger.figure import gaps_panel, motion_panel, poses_to  # noqa: E402
from visualize.finger.view import view_from_row, view_of_vector  # noqa: E402
from visualize.fronts import latest_portfolio, read_rows, with_role  # noqa: E402
from visualize.style import MUTED, OUTPUT, use  # noqa: E402

COLUMNS = 4
FINGER = OUTPUT / "finger"


def parse(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="python -m visualize.finger", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("front", nargs="?", type=Path, help="a portfolio's seeds.csv or a study dir")
    parser.add_argument("role", nargs="?", default="balanced")
    parser.add_argument("--mechanism", help="with --config and --x: a design vector's mechanism")
    parser.add_argument("--config", type=Path, help="the study config file the design vector is read under")
    parser.add_argument("--x", help="the design vector, comma separated")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="a config override")
    parser.add_argument("--step", type=float, default=30.0, help="deg between poses")
    parser.add_argument("--phi", type=float, nargs="+", help="the poses (deg) instead of --step")
    parser.add_argument("--to", type=float, help="the last pose (deg)")
    parser.add_argument("--gaps", action="store_true", help="add each checked pair's gap")
    parser.add_argument("--output", help="the figure's name")
    args = parser.parse_args(argv)
    if (args.front is None) == (args.x is None):
        parser.error("give a front, or --mechanism, --config and --x")
    if args.x is not None and (args.mechanism is None or args.config is None):
        parser.error("--x needs --mechanism and --config")
    return args


def main(argv: list[str]) -> int:
    args = parse(argv)
    use("paper")
    if args.front is not None:
        seeds = args.front if args.front.suffix == ".csv" else latest_portfolio(args.front)
        if seeds is None:
            print(f"no portfolio in {args.front}")
            return 1
        row = with_role(read_rows(seeds), args.role)
        if row is None:
            print(f"no {args.role} in {seeds}")
            return 1
        view = view_from_row(row, seeds.parent.parent)
        name = args.output or f"{seeds.parent.parent.name}_{args.role}"
        caption = f"{view.label}, {seeds.parent.parent.name} {args.role} ({row['run']})"
    else:
        x = [float(value) for value in args.x.split(",")]
        view = view_of_vector(args.mechanism, args.config, x, args.set)
        name = args.output or f"{args.mechanism}_design"
        caption = f"{view.label}: x = ({', '.join(f'{v:g}' for v in x)})"

    end = min(args.to if args.to is not None else view.phi_max_deg, view.reachable_deg())
    poses = [min(pose, view.reachable_deg()) for pose in args.phi] if args.phi else poses_to(end, args.step)
    rows = -(-len(poses) // COLUMNS)
    figure = plt.figure(figsize=(3.4 * COLUMNS, 3.0 * rows + (2.6 if args.gaps else 0.0)), constrained_layout=True)
    grid = figure.add_gridspec(rows + (1 if args.gaps else 0), COLUMNS)
    for i, pose in enumerate(poses):
        motion_panel(figure.add_subplot(grid[i // COLUMNS, i % COLUMNS]), view, pose)
    if args.gaps:
        gaps_panel(figure.add_subplot(grid[rows, :]), view)
    p, d = view.pads_mm
    reach = f"safe to {view.phi_max_deg:.0f}°" if view.row else f"reaches {view.reachable_deg():.0f}°"
    pads = "" if p != p else f", pads P {p:.1f} + D {d:.1f} mm"
    k_max = float(np.nanmax(view.k(np.linspace(0.0, end, 181))))
    figure.suptitle(f"{caption}{pads}, {reach}, k ≤ {k_max:.2f}",
                    color=MUTED, fontsize=8.5, x=0.01, ha="left")
    FINGER.mkdir(parents=True, exist_ok=True)
    out = FINGER / f"{name}.png"
    figure.savefig(out, dpi=160)
    plt.close(figure)
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
