"""The schematic finger in the flexion plane: servos as boxes, servo-driven
joints yellow, links red, pads green (a pad that collides outlined red).

Each function draws on a Matplotlib axis in the proximal frame (``O`` at
the origin, ``+X`` distal, ``+Z`` dorsal), in mm.
"""

from __future__ import annotations

import numpy as np
from matplotlib.patches import Circle, Polygon

from core.morphology.linkage.packaging import distal_pad_polygon, proximal_pad_polygon
from visualize.style import COLLIDE, INK, MUTED, shades

LINK, JOINT, JOINT_EDGE, NODE = "#ff0a1e", "#ffff88", "#d01020", "#111111"
PAD, PAD_EDGE, SERVO_EDGE = "#d5e8d4", "#82b366", INK
ROLLER, ROLLER_EDGE = "#f2f2f2", "#555555"
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
    axis.add_patch(Polygon(np.asarray(outline), closed=True, facecolor="white", edgecolor=SERVO_EDGE,
                           linewidth=1.2, joinstyle="round", zorder=1))


def pad(axis, polygon, hit: bool = False) -> None:
    axis.add_patch(Polygon(np.asarray(polygon), closed=True, facecolor=PAD, edgecolor=COLLIDE if hit else PAD_EDGE,
                           linewidth=2.0 if hit else 1.0, zorder=2))


def links(axis, *chains) -> None:
    for chain in chains:
        points = np.asarray(chain, dtype=float)
        axis.plot(points[:, 0], points[:, 1], color=LINK, linewidth=3.0, solid_capstyle="round", zorder=3)


def active(axis, *centers, radius: float = 4.0) -> None:
    """Servo-driven joints."""

    for center in centers:
        axis.add_patch(Circle(center, radius, facecolor=JOINT, edgecolor=JOINT_EDGE, linewidth=1.2, zorder=4))
        axis.add_patch(Circle(center, 0.9, facecolor=NODE, edgecolor="none", zorder=5))


def nodes(axis, *points) -> None:
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    axis.scatter(points[:, 0], points[:, 1], s=16, color=NODE, zorder=5)


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
    ground(axis, outline)
    servo(axis, outline)


def draw_linkage(axis, linkage, packaging, sweep, i: int, hit: set[str] = frozenset()) -> None:
    """The crossed four-bar at sample ``i`` of its servo ``sweep``."""

    b, c, bp = sweep.b[i], sweep.c[i], sweep.bp[i]
    palm(axis, packaging)
    servo(axis, packaging.pip_footprint(linkage).outline)
    pad(axis, proximal_pad_polygon(linkage, packaging), "proximal_pad" in hit)
    pad(axis, distal_pad_polygon(linkage, packaging, b[None], c[None])[0], "distal_pad" in hit)
    o, a, ap = (0.0, 0.0), (linkage.a.x_mm, linkage.a.z_mm), (linkage.ap.x_mm, linkage.ap.z_mm)
    links(axis, (o, a), (o, ap), (ap, b), (a, bp), (bp, b, c, bp))
    nodes(axis, a, b, bp, c)
    active(axis, o, ap)


def draw_revolute(axis, geo, packaging, phi_rad: float, hit: set[str] = frozenset()) -> None:
    """The serial 2R with its servo on the joint, flexed by ``phi_rad``."""

    palm(axis, packaging)
    servo(axis, geo.distal_servo.outline)
    pad(axis, geo.proximal_pad, "proximal_pad" in hit)
    pad(axis, geo.distal_pad([phi_rad])[0], "distal_pad" in hit)
    a = np.array((geo.a_x_mm, 0.0))
    near, far = geo.posed(geo.distal_mount, [phi_rad])[0]
    links(axis, ((0.0, 0.0), a), (a, near, far, a))  # the distal phalanx: a truss to the pad's ends
    nodes(axis, near, far)
    active(axis, (0.0, 0.0), a)


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
