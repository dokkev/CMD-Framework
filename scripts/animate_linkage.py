#!/usr/bin/env python3
"""Animate baseline linkage V0 and plot its coupled joint-angle curves.

Run from the repository root with no arguments to save the default GIF:

    python3 scripts/animate_linkage.py

The animation uses the XZ linkage frame (+X distal, +Z dorsal) and both
reference-derived couplers.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter
from matplotlib.patches import Polygon
from matplotlib.ticker import MultipleLocator

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from cmd.morphology import (  # noqa: E402
    BASELINE_LINKAGE_V0,
    KinematicClosureError,
    LinkageKinematics,
    LinkagePose,
    PointXZ,
)


DEFAULT_FLEXION_RAD = 0.8
DEFAULT_FRAMES_PER_HALF_CYCLE = 151
DEFAULT_FPS = 24
DEFAULT_DPI = 120
DEFAULT_OUTPUT = Path("output/linkage_flexion_extension.gif")
DEFAULT_CURVE_OUTPUT = Path("output/linkage_v0_q2_q3.png")


def build_q1_sequence(
    flexion_rad: float,
    frames_per_half_cycle: int,
) -> list[float]:
    """Return extension -> flexion -> extension input angles."""

    if not math.isfinite(flexion_rad) or flexion_rad <= 0.0:
        raise ValueError("flexion_rad must be a positive finite angle in radians")
    if frames_per_half_cycle < 2:
        raise ValueError("frames_per_half_cycle must be at least 2")

    extension_to_flexion = [
        -flexion_rad * index / (frames_per_half_cycle - 1)
        for index in range(frames_per_half_cycle)
    ]
    return extension_to_flexion + extension_to_flexion[-2::-1]


def solve_animation_poses(
    solver: LinkageKinematics,
    q1_values: Sequence[float],
) -> list[LinkagePose]:
    """Solve all frames before creating the animation or opening a window."""

    poses: list[LinkagePose] = []
    for frame_index, q1_rad in enumerate(q1_values):
        try:
            poses.append(solver.solve(q1_rad))
        except KinematicClosureError as exc:
            raise ValueError(
                f"animation frame {frame_index} is unreachable at q1={q1_rad:.6g} rad: {exc}"
            ) from exc
    return poses


def _set_line_data(line, points: Sequence[PointXZ]) -> None:
    line.set_data(
        [point.x_mm for point in points],
        [point.z_mm for point in points],
    )


def _draw_ground_support(axis, point: PointXZ) -> None:
    """Draw the fixed pivot support symbol below O or O'."""

    x_mm, z_mm = point.x_mm, point.z_mm
    axis.add_patch(Polygon(
        ((x_mm, z_mm - 0.15),
         (x_mm - 1.2, z_mm - 1.1),
         (x_mm + 1.2, z_mm - 1.1)),
        closed=True, facecolor="white", edgecolor="black",
        linewidth=1.5, zorder=1,
    ))
    axis.plot(
        (x_mm - 1.4, x_mm + 1.4), (z_mm - 1.1, z_mm - 1.1),
        color="black", linewidth=1.5, zorder=1,
    )
    for offset_mm in (-1.1, -0.5, 0.1, 0.7, 1.3):
        axis.plot(
            (x_mm + offset_mm, x_mm + offset_mm - 0.65),
            (z_mm - 1.1, z_mm - 1.75),
            color="black", linewidth=1.1, zorder=1,
        )


def create_animation(
    poses: Sequence[LinkagePose],
    fps: int = DEFAULT_FPS,
) -> tuple[plt.Figure, FuncAnimation]:
    """Create a 2D Matplotlib animation from solved linkage poses."""

    if not poses:
        raise ValueError("poses must contain at least one frame")
    if isinstance(fps, bool) or fps <= 0:
        raise ValueError("fps must be a positive integer")

    all_points = [
        point
        for pose in poses
        for point in (pose.o, pose.a, pose.b, pose.c, pose.op, pose.ap, pose.bp, pose.cp)
    ]
    x_values = [point.x_mm for point in all_points]
    z_values = [point.z_mm for point in all_points]
    x_limits = (min(x_values) - 4.0, max(x_values) + 4.0)
    z_limits = (min(z_values) - 4.0, max(z_values) + 4.0)
    width = 12.5
    height = max(4.0, width * (z_limits[1] - z_limits[0]) / (x_limits[1] - x_limits[0]))
    figure, axis = plt.subplots(figsize=(width, height))
    red, green, blue, black = "#ff1027", "#196b22", "#071c9e", "#111111"

    # Colored triangles show the three rigid bodies; black lines are couplers.
    proximal, = axis.plot([], [], color=red, linewidth=3.2, zorder=2)
    middle, = axis.plot([], [], color=green, linewidth=3.2, zorder=2)
    distal, = axis.plot([], [], color=blue, linewidth=3.2, zorder=2)
    coupler_1, = axis.plot([], [], color=black, linewidth=3.2, zorder=3)
    coupler_2, = axis.plot([], [], color=black, linewidth=3.2, zorder=3)

    marker_styles = {
        "o": (red, red),
        "op": (black, black),
        "a": (red, green),
        "ap": (red, black),
        "bp": (green, black),
        "b": (blue, green),
        "cp": (blue, black),
        "c": (blue, blue),
    }
    markers = {
        name: axis.plot(
            [], [], "o", markersize=13.5, markerfacecolor=face,
            markeredgecolor=edge, markeredgewidth=2.5, zorder=5,
        )[0]
        for name, (face, edge) in marker_styles.items()
    }
    label_styles = {
        "o": (r"$O$", red, (-12, 10)),
        "op": (r"$O'$", black, (-10, 10)),
        "a": (r"$A$", red, (0, 15)),
        "ap": (r"$A'$", red, (-22, -22)),
        "bp": (r"$B'$", green, (22, -24)),
        "b": (r"$B$", green, (0, -25)),
        "cp": (r"$C'$", blue, (0, 15)),
        "c": (r"$C$", blue, (0, -24)),
    }
    labels = {
        name: axis.annotate(
            text, (0.0, 0.0), xytext=offset, textcoords="offset points",
            ha="center", va="center", color=color,
            fontsize=22, fontfamily="serif", fontstyle="italic", zorder=6,
        )
        for name, (text, color, offset) in label_styles.items()
    }

    axis.set_xlim(*x_limits)
    axis.set_ylim(*z_limits)
    axis.set_xlabel("X / distal [mm]")
    axis.set_ylabel("Z / dorsal [mm]")
    axis.set_aspect("equal", adjustable="box")
    axis.xaxis.set_major_locator(MultipleLocator(5.0))
    axis.yaxis.set_major_locator(MultipleLocator(5.0))
    axis.xaxis.set_minor_locator(MultipleLocator(1.0))
    axis.yaxis.set_minor_locator(MultipleLocator(1.0))
    axis.grid(which="major", color="#e0e2e6", linewidth=0.6)
    axis.grid(which="minor", color="#f0f1f3", linewidth=0.4)
    axis.tick_params(which="both", bottom=False, left=False,
                     labelbottom=False, labelleft=False)
    for spine in axis.spines.values():
        spine.set_visible(False)
    _draw_ground_support(axis, poses[0].o)
    _draw_ground_support(axis, poses[0].op)
    figure.subplots_adjust(left=0.08, right=0.99, bottom=0.14, top=0.98)

    def update(frame_index: int):
        pose = poses[frame_index]
        _set_line_data(proximal, (pose.o, pose.a, pose.ap, pose.o))
        _set_line_data(middle, (pose.a, pose.b, pose.bp, pose.a))
        _set_line_data(distal, (pose.b, pose.c, pose.cp, pose.b))
        _set_line_data(coupler_1, (pose.op, pose.bp))
        _set_line_data(coupler_2, (pose.ap, pose.cp))
        for name, marker in markers.items():
            point = getattr(pose, name)
            marker.set_data((point.x_mm,), (point.z_mm,))
            labels[name].xy = (point.x_mm, point.z_mm)
        return (
            proximal, middle, distal, coupler_1, coupler_2,
            *markers.values(),
            *labels.values(),
        )

    animation = FuncAnimation(
        figure,
        update,
        frames=len(poses),
        interval=1000.0 / fps,
        repeat=True,
        blit=True,
    )
    update(0)
    return figure, animation


def save_animation(
    animation: FuncAnimation,
    output_path: Path,
    fps: int,
    dpi: int,
) -> None:
    """Save a GIF with Pillow or an MP4 with the system FFmpeg writer."""

    if isinstance(dpi, bool) or dpi <= 0:
        raise ValueError("dpi must be a positive integer")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    suffix = output_path.suffix.lower()
    if suffix == ".gif":
        animation.save(output_path, writer=PillowWriter(fps=fps), dpi=dpi)
    elif suffix in {".mp4", ".m4v"}:
        animation.save(output_path, writer=FFMpegWriter(fps=fps), dpi=dpi)
    else:
        raise ValueError("output must use .gif, .mp4, or .m4v")


def save_joint_curves(
    sweep_poses: Sequence[LinkagePose],
    output_path: Path,
    dpi: int,
) -> None:
    """Plot the solved relative q2 and q3 angles against commanded q1."""

    # Follow the sweep from extension toward flexion and lift each relative
    # angle by whole turns to avoid jumps at the solver's atan2 branch cut.
    ordered = sorted(sweep_poses, key=lambda pose: pose.q1_rad, reverse=True)
    q2_values: list[float] = []
    q3_values: list[float] = []
    for pose in ordered:
        q2 = pose.q2_rad
        q3 = pose.q3_rad
        if q2_values:
            q2 += 2.0 * math.pi * round((q2_values[-1] - q2) / (2.0 * math.pi))
            q3 += 2.0 * math.pi * round((q3_values[-1] - q3) / (2.0 * math.pi))
        q2_values.append(q2)
        q3_values.append(q3)

    q1_values = [pose.q1_rad for pose in reversed(ordered)]
    q2_values.reverse()
    q3_values.reverse()
    figure, axis = plt.subplots(figsize=(7.5, 4.5), constrained_layout=True)
    try:
        axis.plot(q1_values, q2_values,
                  color="#196b22", linewidth=2.4, label=r"$q_2$")
        axis.plot(q1_values, q3_values,
                  color="#071c9e", linewidth=2.4, label=r"$q_3$")
        axis.axhline(0.0, color="#8e959d", linewidth=0.8)
        axis.set(xlabel=r"Commanded $q_1$ [rad]",
                 ylabel="Joint flexion change from extension [rad]",
                 title="Baseline linkage V0: coupled joint angles")
        axis.grid(True, color="#e0e2e6", linewidth=0.7)
        axis.legend(frameon=False)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output_path, dpi=dpi)
    finally:
        plt.close(figure)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Animate baseline linkage V0 and plot q2/q3 versus q1."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="GIF or MP4 output path (default: output/linkage_flexion_extension.gif)",
    )
    parser.add_argument(
        "--curve-output",
        type=Path,
        default=DEFAULT_CURVE_OUTPUT,
        help="joint-angle plot path (default: output/linkage_v0_q2_q3.png)",
    )
    parser.add_argument(
        "--flexion-rad",
        type=float,
        default=DEFAULT_FLEXION_RAD,
        help="maximum negative q1 flexion magnitude in radians (default: 0.8)",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=DEFAULT_FRAMES_PER_HALF_CYCLE,
        metavar="N",
        help="frames per extension/flexion half-cycle (default: 151)",
    )
    parser.add_argument(
        "--fps", type=int, default=DEFAULT_FPS,
        help="animation playback rate in frames per second (default: 24)",
    )
    parser.add_argument("--dpi", type=int, default=DEFAULT_DPI)
    parser.add_argument("--show", action="store_true", help="open the Matplotlib window")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if isinstance(args.fps, bool) or args.fps <= 0:
        raise ValueError("fps must be a positive integer")

    q1_values = build_q1_sequence(args.flexion_rad, args.frames)
    solver = LinkageKinematics(BASELINE_LINKAGE_V0)
    poses = solve_animation_poses(solver, q1_values)
    figure, animation = create_animation(poses, fps=args.fps)
    try:
        save_animation(animation, args.output, fps=args.fps, dpi=args.dpi)
        save_joint_curves(poses[:args.frames], args.curve_output, dpi=args.dpi)
        print(f"saved {len(poses)} frames to {args.output}")
        print(f"saved q2/q3 sweep to {args.curve_output}")
        if args.show:
            plt.show()
    finally:
        plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
