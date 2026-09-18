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
    from cmd.hardware import XM335Actuator
    from cmd.morphology import (
        AnchorYZ,
        CouplingAnchorParameters,
        FingerLinkage,
        FingerLinkageParameters,
        MainChainParameters,
    )
    from cmd.visualization.actuator_visualizer import ActuatorVisualizer
else:
    from ..hardware import XM335Actuator
    from ..morphology import (
        AnchorYZ,
        CouplingAnchorParameters,
        FingerLinkage,
        FingerLinkageParameters,
        MainChainParameters,
    )
    from .actuator_visualizer import ActuatorVisualizer

import matplotlib.pyplot as plt


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Visualize the CLEAR Hand actuator and reference linkage in 3D."
    )
    parser.add_argument("--proximal-length", type=float, default=20.0, metavar="MM")
    parser.add_argument("--middle-length", type=float, default=15.0, metavar="MM")
    parser.add_argument("--distal-length", type=float, default=10.0, metavar="MM")
    parser.add_argument("--o-prime-y", type=float, default=0.0, metavar="MM")
    parser.add_argument("--o-prime-z", type=float, default=3.0, metavar="MM")
    parser.add_argument("--a-prime-y", type=float, default=3.0, metavar="MM")
    parser.add_argument("--a-prime-z", type=float, default=2.0, metavar="MM")
    parser.add_argument("--b-prime-y", type=float, default=4.0, metavar="MM")
    parser.add_argument("--b-prime-z", type=float, default=-2.0, metavar="MM")
    parser.add_argument("--c-prime-y", type=float, default=5.0, metavar="MM")
    parser.add_argument("--c-prime-z", type=float, default=1.0, metavar="MM")
    parser.add_argument("--axis-marker-length", type=float, default=12.0, metavar="MM")
    parser.add_argument("--output", type=Path, help="save a PNG/PDF/SVG figure")
    parser.add_argument("--dpi", type=int, default=160)
    parser.add_argument("--show", action="store_true", help="open the Matplotlib window")
    return parser


def build_linkage(args: argparse.Namespace) -> FingerLinkage:
    return FingerLinkage(
        FingerLinkageParameters(
            main_chain=MainChainParameters(
                proximal_length_mm=args.proximal_length,
                middle_length_mm=args.middle_length,
                distal_length_mm=args.distal_length,
            ),
            coupling=CouplingAnchorParameters(
                base_anchor=AnchorYZ(args.o_prime_y, args.o_prime_z),
                proximal_anchor=AnchorYZ(args.a_prime_y, args.a_prime_z),
                middle_anchor=AnchorYZ(args.b_prime_y, args.b_prime_z),
                distal_anchor=AnchorYZ(args.c_prime_y, args.c_prime_z),
            ),
        )
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    linkage = build_linkage(args)
    visualizer = ActuatorVisualizer(
        linkage,
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
