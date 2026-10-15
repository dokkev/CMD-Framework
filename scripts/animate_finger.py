#!/usr/bin/env python3
"""Animate the 2-DoF polycentric finger with its servos.

    python scripts/animate_finger.py --mcp-deg 60 --pip-deg 75

or one candidate of a finger study (the output goes next to its CSV):

    python scripts/animate_finger.py --pareto output/stage0/run_001/pareto.csv --candidate 1

The finger first flexes the PIP (the DIP follows through the four-bar), then the MCP, then returns. Drawn in the linkage plane (+X distal,
+Z dorsal): the MCP servo at O (rear body dorsal), the PIP servo at A riding
on the proximal link (rear body toward O), and the distal link B-C (Stage-0
has no fingertip). The PIP servo sits at A' and drives A'-B; the middle link
A-B' turns red where it comes closer than the body clearance to the PIP servo.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Polygon

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.hardware import XM335Actuator  # noqa: E402
from core.morphology.linkage import (  # noqa: E402
    SKETCH_LINKAGE,
    FingerLinkage,
    PointXZ,
    servo_clearance,
)
from core.morphology.linkage.packaging import (  # noqa: E402
    BODY_CLEARANCE_MM,
    HOUSING_RADIUS_MM,
    Packaging,
)
from core.optimization.stage0 import VARIABLE_NAMES, decode  # noqa: E402

Outline = list[tuple[float, float]]

RED, BLACK, BLUE, GRAY = "#ff1027", "#111111", "#071c9e", "#7a7a7a"
SERVO_COLOR = "#c9ced6"


def load_candidate(csv_path: Path, candidate: int) -> tuple[FingerLinkage, float]:
    """Return a Stage-0 candidate's linkage and the PIP flexion at the end of its usable range."""

    with csv_path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            if int(row["candidate"]) == candidate:
                x = [float(row[name]) for name in VARIABLE_NAMES]
                return FingerLinkage(decode(x)), float(row["usable_pip_deg"])
    raise ValueError(f"candidate {candidate} not found in {csv_path}")


def build_trajectory(mcp_rad: float, pip_rad: float, frames: int) -> list[tuple[float, float]]:
    """PIP flexes, then the MCP, then both return along the same path."""

    ramp = [index / (frames - 1) for index in range(frames)]
    forward = [(0.0, s * pip_rad) for s in ramp] + [(s * mcp_rad, pip_rad) for s in ramp[1:]]
    return forward + forward[-2::-1]


def servo_outline(shaft: PointXZ, rear_angle_rad: float, actuator: XM335Actuator) -> Outline:
    """D-shaped servo section with its rear body pointing along ``rear_angle_rad``."""

    radius, rear = actuator.geometry.radius_mm, actuator.geometry.rear_extent_mm
    cap = [(radius * math.cos(t), radius * math.sin(t))
           for t in (math.pi / 2 - math.pi * k / 24 for k in range(25))]
    local = cap + [(-rear, -radius), (-rear, radius)]  # rear body along local -x
    angle = rear_angle_rad + math.pi
    cosine, sine = math.cos(angle), math.sin(angle)
    return [(shaft.x_mm + cosine * x - sine * z, shaft.z_mm + sine * x + cosine * z)
            for x, z in local]


def link_clearance_mm(linkage: FingerLinkage, q_pip_rad: float) -> float:
    return servo_clearance(linkage, pip_samples_rad=(q_pip_rad,)).link_mm


def create_animation(
    linkage: FingerLinkage, trajectory: Sequence[tuple[float, float]], fps: int
) -> tuple[plt.Figure, FuncAnimation]:
    actuator = XM335Actuator()
    poses = [linkage.solve(q_mcp, q_pip) for q_mcp, q_pip in trajectory]
    clearances = [link_clearance_mm(linkage, q_pip) for _, q_pip in trajectory]
    servo = Packaging().pip_footprint(linkage).outline

    frames = []
    for pose in poses:
        cosine, sine = math.cos(-pose.q_mcp_rad), math.sin(-pose.q_mcp_rad)
        frames.append([(cosine * x - sine * z, sine * x + cosine * z) for x, z in servo])
    mcp_servo = servo_outline(PointXZ(0.0, 0.0), math.pi / 2, actuator)

    points = [xz for frame in frames for xz in frame] + mcp_servo
    points += [(p.x_mm, p.z_mm) for pose in poses for p in (pose.ap, pose.bp, pose.b, pose.c)]
    xs, zs = [x for x, _ in points], [z for _, z in points]
    x_limits, z_limits = (min(xs) - 5, max(xs) + 5), (min(zs) - 5, max(zs) + 5)
    width = 11.0
    height = width * (z_limits[1] - z_limits[0]) / (x_limits[1] - x_limits[0]) + 0.6
    figure, axis = plt.subplots(figsize=(width, height))

    axis.add_patch(Polygon(mcp_servo, closed=True, facecolor=SERVO_COLOR,
                           edgecolor="#6b7280", linewidth=1.0, zorder=0.4))
    pip_servo = Polygon(frames[0], closed=True, facecolor=SERVO_COLOR,
                        edgecolor="#6b7280", linewidth=1.0, zorder=0.4)
    axis.add_patch(pip_servo)
    # Dashed: the pin bushing housings.
    rings = {
        name: Circle((0.0, 0.0), radius, fill=False, linestyle="--", linewidth=1.0,
                     edgecolor=color, zorder=4)
        for name, radius, color in (("a", HOUSING_RADIUS_MM, RED),
                                    ("bp", HOUSING_RADIUS_MM, BLUE),
                                    ("b", HOUSING_RADIUS_MM, BLUE))
    }
    for ring in rings.values():
        axis.add_patch(ring)

    proximal, = axis.plot([], [], color=RED, linewidth=3.0, zorder=2)
    middle, = axis.plot([], [], color=BLACK, linewidth=3.0, zorder=3)
    distal, = axis.plot([], [], color=BLUE, linewidth=3.0, zorder=2)
    coupler, = axis.plot([], [], color=GRAY, linewidth=3.0, zorder=3)
    markers = {
        name: axis.plot([], [], "o", markersize=11, markerfacecolor=face,
                        markeredgecolor=edge, markeredgewidth=2.2, zorder=5)[0]
        for name, face, edge in (("o", "#ffd400", RED), ("a", "#ffd400", BLACK),
                                 ("ap", RED, GRAY), ("bp", BLUE, BLACK),
                                 ("b", BLUE, GRAY), ("c", BLUE, BLUE))
    }
    labels = {
        name: axis.annotate(text, (0, 0), xytext=offset, textcoords="offset points",
                            ha="center", va="center", color=color, fontsize=17,
                            fontfamily="serif", fontstyle="italic", zorder=6)
        for name, text, color, offset in (
            ("o", r"$O$", RED, (-12, 12)), ("a", r"$A$", RED, (0, -18)),
            ("ap", r"$A'$", RED, (0, 15)), ("bp", r"$B'$", BLUE, (0, 15)),
            ("b", r"$B$", BLUE, (0, -18)), ("c", r"$C$", BLUE, (12, -12)))
    }
    status = axis.text(0.01, 0.99, "", transform=axis.transAxes, va="top",
                       fontsize=10, fontfamily="monospace", zorder=7)

    axis.set(xlim=x_limits, ylim=z_limits, xlabel="X / distal [mm]", ylabel="Z / dorsal [mm]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, color="#e6e8eb", linewidth=0.6)
    for spine in axis.spines.values():
        spine.set_visible(False)
    figure.tight_layout()

    def xy(points: Sequence[PointXZ]) -> tuple[list[float], list[float]]:
        return [p.x_mm for p in points], [p.z_mm for p in points]

    def update(index: int):
        pose = poses[index]
        pip_servo.set_xy(frames[index])
        for name, ring in rings.items():
            point = getattr(pose, name)
            ring.center = (point.x_mm, point.z_mm)
        proximal.set_data(*xy((pose.o, pose.a, pose.ap, pose.o)))
        middle.set_data(*xy((pose.a, pose.bp)))
        distal.set_data(*xy((pose.bp, pose.b, pose.c, pose.bp)))
        coupler.set_data(*xy((pose.ap, pose.b)))
        middle.set_color(RED if clearances[index] < 0.0 else BLACK)
        for name, marker in markers.items():
            point = getattr(pose, name)
            marker.set_data((point.x_mm,), (point.z_mm,))
            labels[name].xy = (point.x_mm, point.z_mm)
        status.set_text(
            f"MCP {math.degrees(pose.q_mcp_rad):5.1f}°  PIP {math.degrees(pose.q_pip_rad):5.1f}°"
            f"  DIP {math.degrees(pose.q_dip_rad):5.1f}°\n"
            f"{'crossed' if pose.is_crossed else 'open':7s} "
            f"transmission at B' {math.degrees(pose.margin_at_bp_rad):3.0f}°"
            f"  input limit at B {math.degrees(pose.margin_at_b_rad):3.0f}°"
            f"  link-servo clearance {clearances[index] + BODY_CLEARANCE_MM:5.1f} mm"
        )
        return (pip_servo, *rings.values(),
                proximal, middle, distal, coupler,
                *markers.values(), *labels.values(), status)

    animation = FuncAnimation(figure, update, frames=len(poses),
                              interval=1000.0 / fps, blit=True)
    update(0)
    return figure, animation


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--pareto", type=Path, help="finger study pareto.csv")
    parser.add_argument("--candidate", type=int, help="candidate number in --pareto")
    parser.add_argument("--mcp-deg", type=float, default=60.0)
    parser.add_argument("--pip-deg", type=float,
                        help="PIP flexion (default: 75, or the end of the candidate's usable range)")
    parser.add_argument("--frames", type=int, default=61, help="frames per ramp (default: 61)")
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--dpi", type=int, default=110)
    parser.add_argument("--output", type=Path,
                        help="GIF path (default: output/finger_2dof.gif, or candidate_N.gif "
                        "next to --pareto)")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args(argv)

    if (args.pareto is None) != (args.candidate is None):
        raise ValueError("--pareto and --candidate must be given together")
    if args.pareto is None:
        linkage, pip_deg = SKETCH_LINKAGE, 75.0
        output = args.output or Path("output/finger_2dof.gif")
    else:
        linkage, pip_deg = load_candidate(args.pareto, args.candidate)
        output = args.output or args.pareto.parent / f"candidate_{args.candidate}.gif"
    pip_deg = args.pip_deg if args.pip_deg is not None else pip_deg

    trajectory = build_trajectory(math.radians(args.mcp_deg), math.radians(pip_deg), args.frames)
    figure, animation = create_animation(linkage, trajectory, args.fps)
    try:
        output.parent.mkdir(parents=True, exist_ok=True)
        animation.save(output, writer=PillowWriter(fps=args.fps), dpi=args.dpi)
        print(f"saved {len(trajectory)} frames to {output}")
        if args.show:
            plt.show()
    finally:
        plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
