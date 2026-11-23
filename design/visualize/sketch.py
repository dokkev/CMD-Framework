"""The schematic finger in the flexion plane, coloured as ``visualize.finger.solid``:
servos grey, servo-driven joints yellow, the proximal body's links red and the
distal body's blue, a transmission bar between them black or grey (the CFL's
``A-B'`` and ``A'-B``, the RCJ's carrier), pads filled pink (proximal) and blue
(distal) outlined in their body's colour (a pad that collides outlined red).

The pieces each mechanism's view (``visualize.finger.mechanism``) draws its
finger from. Each function draws on a Matplotlib axis in the proximal frame
(``O`` at the origin, ``+X`` distal, ``+Z`` dorsal), in mm.
"""

from __future__ import annotations

import numpy as np
from matplotlib.patches import Circle, Polygon

from visualize.style import COLLIDE, DISTAL_PAD, INK, MUTED, PROXIMAL_PAD, SERVO, shades

LINK, JOINT, JOINT_EDGE, NODE = "#ff0a1e", "#ffff88", "#d01020", "#111111"
PROXIMAL, DISTAL, BLACK_BAR, GRAY_BAR = "#ff1027", "#071c9e", "#111111", "#7a7a7a"
"""The proximal body's links, the distal body's, and the bars between them (as ``visualize.finger.solid``)."""
PAD, PAD_EDGE, SERVO_EDGE = "#d5e8d4", "#82b366", INK
PADS = {"proximal": (PROXIMAL_PAD, "#e11d48"), "distal": (DISTAL_PAD, "#2563eb")}
"""Each pad's fill and outline (``visualize.finger.solid``'s)."""
ROLLER, ROLLER_EDGE = "#f2f2f2", "#555555"
SHOW_GROUND = True
"""Hatch the palm behind the MCP servo; a figure may turn it off."""
LINK_WIDTH = 3.0
"""Link line width [pt]; a figure drawn at print size may set it lower."""
XLIM, YLIM = (-14.0, 104.0), (-62.0, 30.0)
"""The paper's sketches, one scale for every mechanism."""


def sketch_axes(axis) -> None:
    axis.set_aspect("equal")
    axis.axis("off")


def frame(axis, xlim=XLIM, ylim=YLIM) -> None:
    sketch_axes(axis)
    axis.set_xlim(*xlim)
    axis.set_ylim(*ylim)


def servo(axis, outline) -> None:
    axis.add_patch(Polygon(np.asarray(outline), closed=True, facecolor=SERVO, edgecolor=SERVO_EDGE,
                           linewidth=1.2, joinstyle="round", zorder=1))


def pad(axis, polygon, hit: bool = False, which: str | None = None) -> None:
    """A pad, filled as its body's (``which``: proximal or distal; ``None``: plain green)."""

    fill, edge = PADS[which] if which else (PAD, PAD_EDGE)
    axis.add_patch(Polygon(np.asarray(polygon), closed=True, facecolor=fill, edgecolor=COLLIDE if hit else edge,
                           linewidth=2.0 if hit else 1.0, zorder=2))


def links(axis, *chains, color: str = LINK, zorder: float = 3) -> None:
    for chain in chains:
        points = np.asarray(chain, dtype=float)
        axis.plot(points[:, 0], points[:, 1], color=color, linewidth=LINK_WIDTH, solid_capstyle="round", zorder=zorder)


def body_links(axis, proximal=(), distal=(), black=(), gray=()) -> None:
    """Each body's links in its colour, and the bars between the bodies."""

    links(axis, *black, color=BLACK_BAR, zorder=2.8)
    links(axis, *gray, color=GRAY_BAR, zorder=2.9)
    links(axis, *proximal, color=PROXIMAL)
    links(axis, *distal, color=DISTAL)


def active(axis, *centers, radius: float = 4.0) -> None:
    """Servo-driven joints."""

    for center in centers:
        axis.add_patch(Circle(center, radius, facecolor=JOINT, edgecolor=JOINT_EDGE, linewidth=1.2, zorder=4))
        axis.add_patch(Circle(center, 0.9, facecolor=NODE, edgecolor="none", zorder=5))


def nodes(axis, *points) -> None:
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    axis.scatter(points[:, 0], points[:, 1], s=16, color=NODE, zorder=5)


SHOW_CORNERS = False  # rigid-body corners (a fingertip, a pad's end) are not pivots: no dot unless asked


def corners(axis, *points) -> None:
    """Corners of a rigid body that are not pivots; drawn as nodes only when ``SHOW_CORNERS``."""
    if SHOW_CORNERS:
        nodes(axis, *points)


def ground(axis, outline) -> None:
    """Hatching along the MCP servo's back: the palm."""

    outline = np.asarray(outline)
    x = outline[:, 0].min()
    low, high = outline[:, 1].min(), outline[:, 1].max()
    for z in np.arange(low, high + 1e-9, (high - low) / 6.0):
        axis.plot((x, x - 4.0), (z, z - 4.0), color=SERVO_EDGE, linewidth=0.9, zorder=0.5)


def roller(axis, center, radius: float, hit: bool = False) -> None:
    axis.add_patch(Circle(center, radius, facecolor=ROLLER, edgecolor=COLLIDE if hit else ROLLER_EDGE,
                          linewidth=1.8 if hit else 1.0, zorder=1.5))


def palm(axis, packaging) -> None:
    """The MCP servo on its ground."""

    outline = packaging.mcp_footprint().outline
    if SHOW_GROUND:
        ground(axis, outline)
    servo(axis, outline)


def pad_trail(axis, polygons, levels, color: str, *, alpha: float = 0.8, linewidth: float = 0.4,
              zorder: float = 2.4) -> None:
    """Distal pads over a motion, each shaded by its ``level`` in [0, 1] (darker: further)."""

    cmap = shades(color)
    for polygon, level in zip(polygons, levels):
        axis.add_patch(Polygon(polygon, closed=True, facecolor=cmap(level), edgecolor=MUTED, linewidth=linewidth,
                               alpha=alpha, zorder=zorder))


def icr_path(axis, centre, *, size: float = 22, linewidth: float = 1.3, end: str | None = INK) -> None:
    """The instantaneous centre of rotation's path, dotted, its start a ring
    and (``end``: its colour) its last point filled."""

    axis.plot(centre[:, 0], centre[:, 1], linestyle=":", color=INK, linewidth=linewidth, zorder=6)
    axis.scatter(*centre[0], s=size, facecolor="white", edgecolor=INK, linewidth=1.0, zorder=7)
    if end is not None:
        axis.scatter(*centre[-1], s=size, color=end, zorder=7)
