#!/usr/bin/env python3
"""Animate the 2-DoF polycentric finger with its servos and fingertip.

    python scripts/animate_finger.py --mcp-deg 60 --pip-deg 75

The finger first flexes the PIP (the DIP follows through the crossed
four-bar), then the MCP, then returns. Drawn in the linkage plane (+X distal,
+Z dorsal): the MCP servo at O (rear body dorsal), the PIP servo at A riding
on the proximal link (rear body toward O), and the fingertip side silhouette
on the distal link from B to C. The coupler turns red where it comes closer
than the body clearance to the PIP servo.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from cmd.hardware import XM335Actuator  # noqa: E402
from cmd.morphology import Fingertip, FingertipParameters  # noqa: E402
from cmd.morphology.linkage import (  # noqa: E402
    SKETCH_LINKAGE,
    FingerLinkage,
    FingerPose,
    PointXZ,
    servo_clearance,
)
from cmd.morphology.linkage.packaging import BODY_CLEARANCE_MM  # noqa: E402

Outline = list[tuple[float, float]]

RED, BLACK, BLUE, GRAY = "#ff1027", "#111111", "#071c9e", "#7a7a7a"
SERVO_COLOR, PAD_COLOR, CARRIER_COLOR = "#c9ced6", "#b8d5a8", "#999c9f"


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


def fingertip_outlines(fingertip: Fingertip) -> tuple[Outline, Outline]:
    """Pad and carrier side outlines in the fingertip frame."""

    length = fingertip.total_length_mm
    phalanx = fingertip.parameters.family.phalanx
    xs = [length * index / 100 for index in range(101)]
    pad = [(0.0, 0.0)] + [(x, fingertip.lower_surface_z_mm(x, 0.0)) for x in xs] + [(length, 0.0)]
    carrier = [
        (0.0, -phalanx.stem_height_mm), (fingertip.carrier_length_mm, -phalanx.stem_height_mm),
        (fingertip.carrier_length_mm, 0.0), (length, 0.0),
        (length, phalanx.carrier_height_mm), (0.0, phalanx.carrier_height_mm),
    ]
    return pad, carrier


def place_on_distal(outline: Outline, pose: FingerPose, axis_z_mm: float) -> Outline:
    """Map fingertip-frame points onto the distal link: origin B, +X toward C.

    The ``B-C`` line runs through the fingertip at height ``axis_z_mm``.
    """

    angle = math.atan2(pose.c.z_mm - pose.b.z_mm, pose.c.x_mm - pose.b.x_mm)
    cosine, sine = math.cos(angle), math.sin(angle)
    return [(pose.b.x_mm + cosine * x - sine * (z - axis_z_mm),
             pose.b.z_mm + sine * x + cosine * (z - axis_z_mm)) for x, z in outline]


def coupler_clearance_mm(linkage: FingerLinkage, q_pip_rad: float) -> float:
    return servo_clearance(linkage, pip_samples_rad=(q_pip_rad,)).coupler_mm


def create_animation(
    linkage: FingerLinkage, trajectory: Sequence[tuple[float, float]], fps: int
) -> tuple[plt.Figure, FuncAnimation]:
    actuator = XM335Actuator()
    fingertip = Fingertip(FingertipParameters(total_length_mm=linkage.parameters.bc_length_mm))
    pad, carrier = fingertip_outlines(fingertip)
    poses = [linkage.solve(q_mcp, q_pip) for q_mcp, q_pip in trajectory]
    clearances = [coupler_clearance_mm(linkage, q_pip) for _, q_pip in trajectory]

    frames = []
    for pose in poses:
        rear_angle = math.atan2(-pose.a.z_mm, -pose.a.x_mm)  # from A back toward O
        frames.append((
            servo_outline(pose.a, rear_angle, actuator),
            place_on_distal(pad, pose, fingertip.rotation_axis_z_mm),
            place_on_distal(carrier, pose, fingertip.rotation_axis_z_mm),
        ))
    mcp_servo = servo_outline(PointXZ(0.0, 0.0), math.pi / 2, actuator)

    points = [xz for frame in frames for outline in frame for xz in outline] + mcp_servo
    points += [(p.x_mm, p.z_mm) for pose in poses for p in (pose.ap, pose.bp, pose.b, pose.c)]
    xs, zs = [x for x, _ in points], [z for _, z in points]
    x_limits, z_limits = (min(xs) - 5, max(xs) + 5), (min(zs) - 5, max(zs) + 5)
    width = 11.0
    height = width * (z_limits[1] - z_limits[0]) / (x_limits[1] - x_limits[0]) + 0.6
    figure, axis = plt.subplots(figsize=(width, height))

    axis.add_patch(Polygon(mcp_servo, closed=True, facecolor=SERVO_COLOR,
                           edgecolor="#6b7280", linewidth=1.0, zorder=0.4))
    pip_servo = Polygon(frames[0][0], closed=True, facecolor=SERVO_COLOR,
                        edgecolor="#6b7280", linewidth=1.0, zorder=0.4)
    pad_patch = Polygon(frames[0][1], closed=True, facecolor=PAD_COLOR,
                        edgecolor="#50575b", linewidth=0.8, alpha=0.85, zorder=0.5)
    carrier_patch = Polygon(frames[0][2], closed=True, facecolor=CARRIER_COLOR,
                            edgecolor="#50575b", linewidth=0.8, alpha=0.85, zorder=0.6)
    for patch in (pip_servo, pad_patch, carrier_patch):
        axis.add_patch(patch)

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
        servo, pad_outline, carrier_outline = frames[index]
        pip_servo.set_xy(servo)
        pad_patch.set_xy(pad_outline)
        carrier_patch.set_xy(carrier_outline)
        proximal.set_data(*xy((pose.o, pose.a, pose.ap, pose.o)))
        middle.set_data(*xy((pose.a, pose.bp)))
        distal.set_data(*xy((pose.bp, pose.b, pose.c, pose.bp)))
        coupler.set_data(*xy((pose.ap, pose.b)))
        coupler.set_color(RED if clearances[index] < 0.0 else GRAY)
        for name, marker in markers.items():
            point = getattr(pose, name)
            marker.set_data((point.x_mm,), (point.z_mm,))
            labels[name].xy = (point.x_mm, point.z_mm)
        status.set_text(
            f"MCP {math.degrees(pose.q_mcp_rad):5.1f}°  PIP {math.degrees(pose.q_pip_rad):5.1f}°"
            f"  DIP {math.degrees(pose.q_dip_rad):5.1f}°\n"
            f"transmission margin {math.degrees(pose.transmission_margin_rad):4.0f}°"
            f"  coupler-servo clearance {clearances[index] + BODY_CLEARANCE_MM:5.1f} mm"
        )
        return (pip_servo, pad_patch, carrier_patch, proximal, middle, distal, coupler,
                *markers.values(), *labels.values(), status)

    animation = FuncAnimation(figure, update, frames=len(poses),
                              interval=1000.0 / fps, blit=True)
    update(0)
    return figure, animation


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mcp-deg", type=float, default=60.0)
    parser.add_argument("--pip-deg", type=float, default=75.0)
    parser.add_argument("--frames", type=int, default=61, help="frames per ramp (default: 61)")
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--dpi", type=int, default=110)
    parser.add_argument("--output", type=Path, default=Path("output/finger_2dof.gif"))
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args(argv)

    trajectory = build_trajectory(math.radians(args.mcp_deg), math.radians(args.pip_deg),
                                  args.frames)
    figure, animation = create_animation(SKETCH_LINKAGE, trajectory, args.fps)
    try:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        animation.save(args.output, writer=PillowWriter(fps=args.fps), dpi=args.dpi)
        print(f"saved {len(trajectory)} frames to {args.output}")
        if args.show:
            plt.show()
    finally:
        plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
