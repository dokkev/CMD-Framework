"""Animate the 2-DoF polycentric finger with its servos.

    python -m visualize.animate --mcp-deg 60 --pip-deg 75

or one candidate of a finger study (the output goes next to its CSV):

    python -m visualize.animate --pareto output/linkage/config/paper/run_009/pareto.csv --candidate 2542

The finger first flexes the PIP (the DIP follows through the four-bar), then the MCP, then returns. Drawn in the linkage plane (+X distal,
+Z dorsal): the MCP servo at O (rear body dorsal), the PIP servo at A riding
on the proximal link (rear body toward O), and the distal link B-C (the linkage
has no fingertip). The PIP servo sits at A' and drives A'-B; the driver link
A-B' turns red where it comes closer than the body clearance to the PIP servo.

The contact pads are drawn as the solids the linkage evaluator checks
(``core.morphology.linkage.packaging``): blocks palmar of O-A'' and B''-C,
chamfered toward the PIP, riding on the proximal and distal links. They turn
red when a checked pair comes closer than the body clearance at that pose
(MCP at its frame angle); the status line names the closest pair.

Each part in the central layer is outlined in its collision group's colour
(``packaging.COLLISION_GROUPS``: palm, proximal, distal): parts of different
groups must not collide, parts of one group are one rigid body. The driver
link and coupler plates sit in another layer and are not outlined.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Callable, Sequence

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Polygon

from core.morphology.linkage import (
    SKETCH_LINKAGE,
    FingerLinkage,
    PointXZ,
    servo_clearance,
)
from core.morphology.linkage.packaging import (
    COMPONENTS,
    Packaging,
    fixed_margin_pairs,
    distal_pad_polygon,
    proximal_pad_polygon,
    pad_margins,
    turn_points,
)
from core.optimization.linkage import VARIABLE_NAMES, decode, read_design
from core.optimization.study import load_config
from visualize.style import DISTAL_PAD, LIGHT_GRID, OUTPUT, PROXIMAL_PAD, SERVO

Outline = list[tuple[float, float]]

RED, BLACK, BLUE, GRAY = "#ff1027", "#111111", "#071c9e", "#7a7a7a"
GROUP_COLORS = {"palm": "#7c3aed", "proximal": "#e11d48", "distal": "#2563eb"}
"""Outline colour of each collision group drawn."""
OUTLINE_WIDTH = 1.6


def load_candidate(csv_path: Path, candidate: int,
                   app_offset_mm: float | None = None) -> tuple[FingerLinkage, float, Packaging]:
    """Return a linkage optimization candidate's linkage, the PIP flexion at the end of its
    usable range and the run's packaging."""

    with csv_path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            if int(row["candidate"]) == candidate:
                x = read_design(row)
                if app_offset_mm is not None:  # the proximal pad run past A'
                    x[VARIABLE_NAMES.index("app_offset_mm")] = app_offset_mm
                config = load_config(csv_path.parent)
                linkage = FingerLinkage(decode(x, config.link_length_resolution_mm))
                return linkage, float(row["usable_pip_deg"]), config.packaging
    raise ValueError(f"candidate {candidate} not found in {csv_path}")


def build_trajectory(mcp_rad: float, pip_rad: float, frames: int) -> list[tuple[float, float]]:
    """PIP flexes, then the MCP, then both return along the same path."""

    ramp = [index / (frames - 1) for index in range(frames)]
    forward = [(0.0, s * pip_rad) for s in ramp] + [(s * mcp_rad, pip_rad) for s in ramp[1:]]
    return forward + forward[-2::-1]


def link_clearance_mm(linkage: FingerLinkage, q_pip_rad: float, packaging: Packaging) -> float:
    return servo_clearance(linkage, packaging, pip_samples_rad=(q_pip_rad,)).link_mm


def pad_status(linkage: FingerLinkage, packaging: Packaging,
               q_mcp_rad: float, q_pip_rad: float) -> tuple[float, str, bool]:
    """Least pad margin at one pose (less the fixed-margin pairs), the pair
    that sets it, and whether any checked pair collides."""

    sweep = linkage.sweep_pip((q_pip_rad,))
    margins = pad_margins(linkage, packaging, sweep.b, sweep.c, sweep.bp, (q_mcp_rad,))
    reported = {pair: margin for pair, margin in margins.items() if pair not in fixed_margin_pairs(packaging)}
    pair, margin = min(reported.items(), key=lambda item: float(item[1][0]))
    colliding = any(float(value[0]) < 0.0 for value in margins.values())
    return float(margin[0]), "/".join(pair), colliding


def create_animation(
    linkage: FingerLinkage, trajectory: Sequence[tuple[float, float]], fps: int,
    packaging: Packaging | None = None,
) -> tuple[plt.Figure, FuncAnimation, Callable[[int], object]]:
    packaging = packaging or Packaging()
    poses = [linkage.solve(q_mcp, q_pip) for q_mcp, q_pip in trajectory]
    clearances = [link_clearance_mm(linkage, q_pip, packaging) for _, q_pip in trajectory]
    pads = [pad_status(linkage, packaging, q_mcp, q_pip) for q_mcp, q_pip in trajectory]
    servo = packaging.pip_footprint(linkage).outline
    # The pad solids, posed: the proximal one turns with the MCP, the
    # distal one rides on the posed B and C.
    proximal = proximal_pad_polygon(linkage, packaging)
    proximal_frames = [[turn_points(proximal, -pose.q_mcp_rad)] for pose in poses]
    distal_frames = [list(distal_pad_polygon(
        linkage, packaging, np.array([[pose.b.x_mm, pose.b.z_mm]]),
        np.array([[pose.c.x_mm, pose.c.z_mm]]))) for pose in poses]

    frames = []
    for pose in poses:
        cosine, sine = math.cos(-pose.q_mcp_rad), math.sin(-pose.q_mcp_rad)
        frames.append([(cosine * x - sine * z, sine * x + cosine * z) for x, z in servo])
    # The configured MCP servo, as the evaluator checks it.
    mcp_servo = [tuple(point) for point in packaging.mcp_footprint().outline]

    points = [xz for frame in frames for xz in frame] + mcp_servo
    points += [(p.x_mm, p.z_mm) for pose in poses for p in (pose.ap, pose.bp, pose.b, pose.c)]
    points += [tuple(xz) for frame in proximal_frames + distal_frames for quad in frame for xz in quad]
    xs, zs = [x for x, _ in points], [z for _, z in points]
    x_limits, z_limits = (min(xs) - 5, max(xs) + 5), (min(zs) - 5, max(zs) + 5)
    width = 11.0
    height = width * (z_limits[1] - z_limits[0]) / (x_limits[1] - x_limits[0]) + 0.6
    figure, axis = plt.subplots(figsize=(width, height))

    axis.add_patch(Polygon(mcp_servo, closed=True, facecolor=SERVO,
                           edgecolor="#6b7280", linewidth=1.0, zorder=0.4))
    pip_servo = Polygon(frames[0], closed=True, facecolor=SERVO,
                        edgecolor="#6b7280", linewidth=1.0, zorder=0.4)
    axis.add_patch(pip_servo)
    pad_patches = [
        Polygon(quad, closed=True, facecolor=color, edgecolor=edge, linewidth=1.0, alpha=0.85, zorder=0.6)
        for frames_, color, edge in ((proximal_frames, PROXIMAL_PAD, RED),
                                     (distal_frames, DISTAL_PAD, BLUE))
        for quad in frames_[0]
    ]
    for patch in pad_patches:
        axis.add_patch(patch)
    # Dashed: the pin envelopes (bearing housing or flange) grown by the body clearance,
    # the zone the servo body must stay out of.
    rings = {
        name: Circle((0.0, 0.0), packaging.pivot_radius_mm(name) + packaging.body_clearance_mm, fill=False,
                     linestyle="--", linewidth=1.0, edgecolor=color, zorder=4)
        for name, color in (("a", RED), ("bp", BLUE), ("b", BLUE))
    }
    for ring in rings.values():
        axis.add_patch(ring)

    # Collision groups: each part outlined in its group's colour.
    pins = {"a_pin": "a", "ap_pin": "ap", "bp_pin": "bp", "b_pin": "b"}

    def part_shape(name: str, index: int):
        """A part at a frame: its outline points, or a pin's centre."""
        pose = poses[index]
        if name in pins:
            point = getattr(pose, pins[name])
            return (point.x_mm, point.z_mm)
        return {"proximal_pad": proximal_frames[index][0], "distal_pad": distal_frames[index][0],
                "pip_servo": frames[index], "mcp_servo": mcp_servo}[name]

    outlines = []
    for name, component in COMPONENTS.items():
        if component.group not in GROUP_COLORS:
            continue  # the link plates, in another layer
        if name == "mcp_servo" and not (packaging.pad_checks_mcp("proximal_pad")
                                        or packaging.pad_checks_mcp("distal_pad")):
            continue
        color = GROUP_COLORS[component.group]
        if name in pins:
            patch = Circle((0.0, 0.0), packaging.pivot_radius_mm(pins[name]), fill=False,
                           edgecolor=color, linewidth=OUTLINE_WIDTH, zorder=4.5)
        else:
            patch = Polygon(part_shape(name, 0), closed=True, fill=False,
                            edgecolor=color, linewidth=OUTLINE_WIDTH, joinstyle="round", zorder=0.7)
        axis.add_patch(patch)
        outlines.append((patch, name))
    axis.legend(handles=[Line2D([], [], color=color, linewidth=2.5, label=group)
                         for group, color in GROUP_COLORS.items()],
                loc="lower right", fontsize=8, framealpha=0.85,
                title="collision group\n(different groups\nmust not collide)", title_fontsize=8)

    proximal, = axis.plot([], [], color=RED, linewidth=3.0, zorder=2)
    driver, = axis.plot([], [], color=BLACK, linewidth=3.0, zorder=3)
    distal, = axis.plot([], [], color=BLUE, linewidth=3.0, zorder=2)
    coupler, = axis.plot([], [], color=GRAY, linewidth=3.0, zorder=3)
    markers = {
        name: axis.plot([], [], "o", markersize=11, markerfacecolor=face,
                        markeredgecolor=edge, markeredgewidth=2.2, zorder=5)[0]
        for name, face, edge in (("o", "#ffd400", RED), ("a", RED, BLACK),
                                 ("ap", "#ffd400", GRAY), ("bp", BLUE, BLACK),
                                 ("b", BLUE, GRAY), ("c", BLUE, BLUE),
                                 ("app", "white", RED), ("bpp", "white", BLUE))
    }
    labels = {
        name: axis.annotate(text, (0, 0), xytext=offset, textcoords="offset points",
                            ha="center", va="center", color=color, fontsize=17,
                            fontfamily="serif", fontstyle="italic", zorder=6)
        for name, text, color, offset in (
            ("o", r"$O$", RED, (-12, 12)), ("a", r"$A$", RED, (0, -18)),
            ("ap", r"$A'$", RED, (0, 15)), ("bp", r"$B'$", BLUE, (0, 15)),
            ("b", r"$B$", BLUE, (0, -18)), ("c", r"$C$", BLUE, (12, -12)),
            ("app", r"$A''$", RED, (0, -18)), ("bpp", r"$B''$", BLUE, (0, -18)))
    }
    status = axis.text(0.01, 0.99, "", transform=axis.transAxes, va="top",
                       fontsize=10, fontfamily="monospace", zorder=7)

    axis.set(xlim=x_limits, ylim=z_limits, xlabel="X / distal [mm]", ylabel="Z / dorsal [mm]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, color=LIGHT_GRID, linewidth=0.6)
    for spine in axis.spines.values():
        spine.set_visible(False)
    figure.tight_layout()

    def xy(points: Sequence[PointXZ]) -> tuple[list[float], list[float]]:
        return [p.x_mm for p in points], [p.z_mm for p in points]

    def transmission_text(pose) -> str:
        # The transmission angle is at the joint the driven link does not touch.
        on_a = packaging.pip_servo_on_a
        drive, limit = (("B", "B'") if on_a else ("B'", "B"))
        transmission = pose.margin_at_b_rad if on_a else pose.margin_at_bp_rad
        toggle = pose.margin_at_bp_rad if on_a else pose.margin_at_b_rad
        return (f"transmission at {drive} {math.degrees(transmission):3.0f}°"
                f"  limit position at {limit} {math.degrees(toggle):3.0f}°")

    def update(index: int):
        pose = poses[index]
        pip_servo.set_xy(frames[index])
        pad_margin, pad_pair, colliding = pads[index]
        for patch, quad in zip(pad_patches, proximal_frames[index] + distal_frames[index]):
            patch.set_xy(quad)
            patch.set_edgecolor(RED if colliding else BLACK)
        for name, ring in rings.items():
            point = getattr(pose, name)
            ring.center = (point.x_mm, point.z_mm)
        for patch, name in outlines:
            shape = part_shape(name, index)
            if name in pins:
                patch.center = shape
            else:
                patch.set_xy(shape)
        proximal.set_data(*xy((pose.o, pose.a, pose.ap, pose.o)))
        driver.set_data(*xy((pose.a, pose.bp)))
        distal.set_data(*xy((pose.bp, pose.b, pose.c, pose.bp)))
        coupler.set_data(*xy((pose.ap, pose.b)))
        driver.set_color(RED if clearances[index] < 0.0 else BLACK)
        for name, marker in markers.items():
            point = getattr(pose, name)
            x, z = point.x_mm, point.z_mm
            if name in ("app", "bpp"):  # on the pad's top face, its drop below the link line
                start, end, drop = ((pose.o, pose.a, packaging.proximal_drop(linkage.parameters)) if name == "app"
                                    else (pose.b, pose.c, packaging.distal_drop(linkage.parameters)))
                ux, uz = end.x_mm - start.x_mm, end.z_mm - start.z_mm
                length = math.hypot(ux, uz) or 1.0
                x, z = x - drop * -uz / length, z - drop * ux / length
            marker.set_data((x,), (z,))
            labels[name].xy = (x, z)
        status.set_text(
            f"MCP {math.degrees(pose.q_mcp_rad):5.1f}°  PIP {math.degrees(pose.q_pip_rad):5.1f}°"
            f"  DIP {math.degrees(pose.q_dip_rad):5.1f}°\n"
            f"{'crossed' if pose.is_crossed else 'open':7s} "
            f"{transmission_text(pose)}"
            f"  link-servo clearance {clearances[index] + packaging.body_clearance_mm:5.1f} mm\n"
            f"pad margin {pad_margin:5.1f} mm ({pad_pair})"
            f"{'  COLLISION' if colliding else ''}"
        )
        return (pip_servo, *pad_patches, *rings.values(), *(patch for patch, _ in outlines),
                proximal, driver, distal, coupler,
                *markers.values(), *labels.values(), status)

    animation = FuncAnimation(figure, update, frames=len(poses),
                              interval=1000.0 / fps, blit=True)
    update(0)
    return figure, animation, update


def save_key_frames(figure: plt.Figure, update: Callable[[int], object],
                    indices: Sequence[int], output: Path, dpi: int) -> None:
    """Tile the frames at ``indices`` two per row into one PNG."""

    figure.set_dpi(dpi)
    images = []
    for index in indices:
        # Blitting marks the moving artists animated, which a plain draw skips.
        for artist in update(index):
            artist.set_animated(False)
        figure.canvas.draw()
        images.append(Image.fromarray(np.asarray(figure.canvas.buffer_rgba())).convert("RGB"))
    width, height = images[0].size
    columns = 2
    rows = -(-len(images) // columns)
    sheet = Image.new("RGB", (columns * width, rows * height), "white")
    for position, image in enumerate(images):
        sheet.paste(image, ((position % columns) * width, (position // columns) * height))
    sheet.save(output)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--pareto", type=Path, help="finger study pareto.csv")
    parser.add_argument("--candidate", type=int, help="candidate number in --pareto")
    parser.add_argument("--app-offset-mm", type=float,
                        help="A''_x - A'_x to draw the candidate with (default: its own)")
    parser.add_argument("--mcp-deg", type=float, default=60.0)
    parser.add_argument("--pip-deg", type=float,
                        help="PIP flexion (default: 75, or the end of the candidate's usable range)")
    parser.add_argument("--frames", type=int, default=61, help="frames per ramp (default: 61)")
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--dpi", type=int, default=110)
    parser.add_argument("--output", type=Path,
                        help="GIF path (default: output/finger_2dof.gif, or candidate_N.gif "
                        "next to --pareto)")
    parser.add_argument("--no-png", action="store_true",
                        help="skip the key-frame PNG saved next to the GIF")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args(argv)

    if (args.pareto is None) != (args.candidate is None):
        raise ValueError("--pareto and --candidate must be given together")
    if args.pareto is None:
        linkage, pip_deg, packaging = SKETCH_LINKAGE, 75.0, Packaging()
        output = args.output or OUTPUT / "finger_2dof.gif"
    else:
        linkage, pip_deg, packaging = load_candidate(args.pareto, args.candidate, args.app_offset_mm)
        output = args.output or args.pareto.parent / f"candidate_{args.candidate}.gif"
    pip_deg = args.pip_deg if args.pip_deg is not None else pip_deg

    trajectory = build_trajectory(math.radians(args.mcp_deg), math.radians(pip_deg), args.frames)
    figure, animation, update = create_animation(linkage, trajectory, args.fps, packaging)
    try:
        output.parent.mkdir(parents=True, exist_ok=True)
        animation.save(output, writer=PillowWriter(fps=args.fps), dpi=args.dpi)
        print(f"saved {len(trajectory)} frames to {output}")
        if not args.no_png:
            # Extension, half the PIP, the full PIP, then the full MCP on top.
            ramp = args.frames - 1
            still = output.with_suffix(".png")
            save_key_frames(figure, update, (0, ramp // 2, ramp, 2 * ramp), still, args.dpi)
            print(f"saved key frames to {still}")
        if args.show:
            plt.show()
    finally:
        plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
