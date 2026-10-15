"""Command-line entry point for the CLEAR actuator visualization."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence

if __package__ in (None, ""):
    repository_root = Path(__file__).resolve().parents[2]
    if str(repository_root) not in sys.path:
        sys.path.insert(0, str(repository_root))
    from core.hardware import XM335Actuator
    from core.visualization.actuator_visualizer import ActuatorVisualizer
else:
    from core.hardware import XM335Actuator
    from core.visualization.actuator_visualizer import ActuatorVisualizer

import matplotlib.pyplot as plt


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Visualize the CLEAR XM335 actuator in 3D.")
    parser.add_argument("--axis-marker-length", type=float, default=16.0, metavar="MM")
    parser.add_argument("--output", type=Path, help="save a PNG/PDF/SVG figure")
    parser.add_argument("--dpi", type=int, default=160)
    parser.add_argument("--show", action="store_true", help="open the Matplotlib window")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    visualizer = ActuatorVisualizer(
        actuator=XM335Actuator(),
        axis_marker_length_mm=args.axis_marker_length,
    )
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
