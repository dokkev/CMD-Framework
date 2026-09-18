"""Command-line entry point for the 3D analytic fingertip visualization."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence

if __package__ in (None, ""):
    repository_root = Path(__file__).resolve().parents[2]
    if str(repository_root) not in sys.path:
        sys.path.insert(0, str(repository_root))
    from cmd.morphology import Fingertip
    from cmd.visualization.fingertip_visualizer import FingertipVisualizer
else:
    from ..morphology import Fingertip
    from .fingertip_visualizer import FingertipVisualizer

import matplotlib.pyplot as plt


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Visualize the analytic fingertip morphology with Matplotlib."
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="save a PNG/PDF/SVG figure instead of only opening a window",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=160,
        help="output resolution when --output is used (default: 160)",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=200,
        help="number of longitudinal samples (default: 200)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="open the interactive Matplotlib window after saving",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    visualizer = FingertipVisualizer(Fingertip(), samples=args.samples)
    figure = visualizer.create_figure()

    if args.output is not None:
        figure.savefig(args.output, dpi=args.dpi, bbox_inches="tight")

    if args.show or args.output is None:
        plt.show()
    else:
        plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
