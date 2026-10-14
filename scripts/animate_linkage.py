#!/usr/bin/env python3
"""Animate a finger linkage and plot its coupled joint-angle curves.

Run from the repository root with no arguments to animate baseline V0:

    python3 scripts/animate_linkage.py

animate the selected linkage V1:

    python3 scripts/animate_linkage.py --linkage v1 --flexion-rad 1.26

or animate one Stage-0 Pareto candidate (outputs go next to the CSV):

    python3 scripts/animate_linkage.py --pareto outputs/stage0/run_002/pareto.csv --candidate 153

The animation uses the XZ linkage frame (+X distal, +Z dorsal) and both
reference-derived couplers. The fingertip side silhouette (carrier and pad)
is drawn on the distal link with its rotation axis on ``B`` and its tip at
``C``; ``--no-fingertip`` hides it.
"""

from __future__ import annotations

import argparse
import csv
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

from cmd.experimental.linkage import (  # noqa: E402
    BASELINE_LINKAGE_V0,
    SELECTED_LINKAGE_V1,
    FingerLinkage,
    KinematicClosureError,
    LinkageKinematics,
    LinkagePose,
    PointXZ,
)
from cmd.morphology import Fingertip, FingertipParameters  # noqa: E402
from cmd.morphology.hand_layout import distal_link_transform, fingertip_attachment  # noqa: E402
from cmd.experimental.stage0 import VARIABLE_NAMES, decode  # noqa: E402


DEFAULT_FLEXION_RAD = 0.8
DEFAULT_FRAMES_PER_HALF_CYCLE = 151
DEFAULT_FPS = 24
DEFAULT_DPI = 120
LINKAGES = {"v0": BASELINE_LINKAGE_V0, "v1": SELECTED_LINKAGE_V1}


def load_pareto_candidate(csv_path: Path, candidate: int) -> FingerLinkage:
    """Return the linkage of one numbered row of a Stage-0 ``pareto.csv``."""

    with csv_path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            if int(row["candidate"]) == candidate:
                return FingerLinkage(decode([float(row[name]) for name in VARIABLE_NAMES]))
    raise ValueError(f"candidate {candidate} not found in {csv_path}")


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


def fingertip_silhouettes(
    fingertip: Fingertip,
) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """Return the ``(x, z)`` side outlines of the pad and the carrier.

    The pad spans from the shoulder plane down to its centreline lower
    surface, its deepest point at every ``x``.
    """

    length = fingertip.total_length_mm
    phalanx = fingertip.parameters.family.phalanx
    x_values = [length * index / 200 for index in range(201)]
    pad = [(0.0, 0.0)] + [(x, fingertip.lower_surface_z_mm(x, 0.0)) for x in x_values]
    pad.append((length, 0.0))
    carrier = [
        (0.0, -phalanx.stem_height_mm),
        (fingertip.carrier_length_mm, -phalanx.stem_height_mm),
        (fingertip.carrier_length_mm, 0.0),
        (fingertip.dorsal_plate_length_mm, 0.0),
        (fingertip.dorsal_plate_length_mm, phalanx.carrier_height_mm),
        (0.0, phalanx.carrier_height_mm),
    ]
    return pad, carrier


def posed_outline(
    pose: LinkagePose,
    fingertip: Fingertip,
    bc_length_mm: float,
    outline: Sequence[tuple[float, float]],
) -> list[tuple[float, float]]:
    """Map a fingertip-frame ``(x, z)`` outline into the posed linkage frame."""

    transform = distal_link_transform(pose) @ fingertip_attachment(fingertip, bc_length_mm)
    points = (transform.transform_point((x, 0.0, z)) for x, z in outline)
    return [(x, z) for x, _, z in points]


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
    fingertip: Fingertip | None = None,
    bc_length_mm: float | None = None,
) -> tuple[plt.Figure, FuncAnimation]:
    """Create a 2D Matplotlib animation from solved linkage poses.

    When ``fingertip`` is given, its side silhouette rides on the distal link
    with its rotation axis on ``B``, ``bc_length_mm`` from the tip.
    """

    if not poses:
        raise ValueError("poses must contain at least one frame")
    if isinstance(fps, bool) or fps <= 0:
        raise ValueError("fps must be a positive integer")

    tip_outlines: list[tuple[list[tuple[float, float]], list[tuple[float, float]]]] = []
    if fingertip is not None:
        pad, carrier = fingertip_silhouettes(fingertip)
        tip_outlines = [
            (posed_outline(pose, fingertip, bc_length_mm, pad),
             posed_outline(pose, fingertip, bc_length_mm, carrier))
            for pose in poses
        ]

    all_points = [
        (point.x_mm, point.z_mm)
        for pose in poses
        for point in (pose.o, pose.a, pose.b, pose.c, pose.op, pose.ap, pose.bp, pose.cp)
    ]
    all_points += [point for outlines in tip_outlines for outline in outlines for point in outline]
    x_values = [x for x, _ in all_points]
    z_values = [z for _, z in all_points]
    x_limits = (min(x_values) - 4.0, max(x_values) + 4.0)
    z_limits = (min(z_values) - 4.0, max(z_values) + 4.0)
    width = 12.5
    height = max(4.0, width * (z_limits[1] - z_limits[0]) / (x_limits[1] - x_limits[0]))
    figure, axis = plt.subplots(figsize=(width, height))
    red, green, blue, black = "#ff1027", "#196b22", "#071c9e", "#111111"

    # The fingertip sits behind the linkage so pins stay visible.
    pad_patch = Polygon([(0.0, 0.0)], closed=True, facecolor="#b8d5a8",
                        edgecolor="#50575b", linewidth=1.0, alpha=0.85, zorder=0.5)
    carrier_patch = Polygon([(0.0, 0.0)], closed=True, facecolor="#999c9f",
                            edgecolor="#50575b", linewidth=1.0, alpha=0.85, zorder=0.6)
    tip_artists = (pad_patch, carrier_patch) if tip_outlines else ()
    for patch in tip_artists:
        axis.add_patch(patch)

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
        if tip_outlines:
            pad_patch.set_xy(tip_outlines[frame_index][0])
            carrier_patch.set_xy(tip_outlines[frame_index][1])
        return (
            *tip_artists,
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
    title: str = "Baseline linkage V0: coupled joint angles",
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
                 title=title)
        axis.grid(True, color="#e0e2e6", linewidth=0.7)
        axis.legend(frameon=False)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output_path, dpi=dpi)
    finally:
        plt.close(figure)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Animate a finger linkage and plot q2/q3 versus q1."
    )
    parser.add_argument(
        "--linkage", choices=sorted(LINKAGES), default="v0",
        help="named linkage to animate when --pareto is not given (default: v0)",
    )
    parser.add_argument(
        "--pareto", type=Path,
        help="Stage-0 pareto.csv to read a candidate from instead of --linkage",
    )
    parser.add_argument("--candidate", type=int, help="candidate number in --pareto")
    parser.add_argument(
        "--output",
        type=Path,
        help="GIF or MP4 output path (default: output/linkage_<name>_flexion_extension.gif, "
        "or candidate_N.gif next to --pareto)",
    )
    parser.add_argument(
        "--curve-output",
        type=Path,
        help="joint-angle plot path (default: output/linkage_<name>_q2_q3.png, "
        "or candidate_N_q2_q3.png next to --pareto)",
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
    parser.add_argument("--no-fingertip", action="store_true", help="hide the fingertip")
    parser.add_argument("--dpi", type=int, default=DEFAULT_DPI)
    parser.add_argument("--show", action="store_true", help="open the Matplotlib window")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if isinstance(args.fps, bool) or args.fps <= 0:
        raise ValueError("fps must be a positive integer")

    if (args.pareto is None) != (args.candidate is None):
        raise ValueError("--pareto and --candidate must be given together")
    if args.pareto is None:
        linkage = LINKAGES[args.linkage]
        title = f"Linkage {args.linkage.upper()}: coupled joint angles"
        output = args.output or Path(f"output/linkage_{args.linkage}_flexion_extension.gif")
        curve_output = args.curve_output or Path(f"output/linkage_{args.linkage}_q2_q3.png")
    else:
        linkage = load_pareto_candidate(args.pareto, args.candidate)
        title = f"Stage-0 candidate #{args.candidate}: coupled joint angles"
        output = args.output or args.pareto.parent / f"candidate_{args.candidate}.gif"
        curve_output = args.curve_output or (
            args.pareto.parent / f"candidate_{args.candidate}_q2_q3.png"
        )

    q1_values = build_q1_sequence(args.flexion_rad, args.frames)
    solver = LinkageKinematics(linkage)
    poses = solve_animation_poses(solver, q1_values)
    fingertip = None
    if not args.no_fingertip:
        # The fingertip is as long as BC: axis on B, pad tip on C.
        fingertip = Fingertip(FingertipParameters(total_length_mm=linkage.parameters.bc_length_mm))
    figure, animation = create_animation(
        poses, fps=args.fps, fingertip=fingertip,
        bc_length_mm=linkage.parameters.bc_length_mm,
    )
    try:
        save_animation(animation, output, fps=args.fps, dpi=args.dpi)
        save_joint_curves(poses[:args.frames], curve_output, dpi=args.dpi, title=title)
        print(f"saved {len(poses)} frames to {output}")
        print(f"saved q2/q3 sweep to {curve_output}")
        if args.show:
            plt.show()
    finally:
        plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
