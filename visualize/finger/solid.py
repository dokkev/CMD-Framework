"""The finger drawn as its parts on the flexion plane, the way
``visualize.animate`` draws the linkage: servos grey, pads filled (proximal
pink, distal blue), every part in the central layer outlined in its
collision group's colour (palm, proximal, distal: parts of different groups
must not collide), links in their body's colour, pins with their clearance
circles, the points named. A part that collides is outlined thick in
``COLLIDE`` and hatched.

In the proximal frame (``O`` at the origin, ``+X`` distal, ``+Z`` dorsal), in mm.
"""

from __future__ import annotations

import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Polygon

from visualize.style import COLLIDE, DISTAL_PAD, LIGHT_GRID, PROXIMAL_PAD, SERVO

RED, BLACK, BLUE, GRAY, YELLOW = "#ff1027", "#111111", "#071c9e", "#7a7a7a", "#ffd400"
GROUP_COLORS = {"palm": "#7c3aed", "proximal": "#e11d48", "distal": "#2563eb"}
"""Outline colour of each collision group."""
BODY_COLORS = {"proximal": RED, "distal": BLUE}
"""The links' and the points' colour on each body."""
OUTLINE_WIDTH = 1.6
XLIM, YLIM = (-22.0, 106.0), (-66.0, 34.0)
"""One scale for every mechanism and pose, grown where a part reaches past it."""
PLANE_MARGIN_MM = 5.0


def plane(axis, xlim=XLIM, ylim=YLIM) -> None:
    """Equal scale in mm on a light grid, no spines; ``xlim`` and ``ylim``
    grown to what is drawn (a long finger flexed reaches past them)."""

    data = axis.dataLim
    if np.isfinite(data.x0) and np.isfinite(data.y0):
        xlim = (min(xlim[0], data.x0 - PLANE_MARGIN_MM), max(xlim[1], data.x1 + PLANE_MARGIN_MM))
        ylim = (min(ylim[0], data.y0 - PLANE_MARGIN_MM), max(ylim[1], data.y1 + PLANE_MARGIN_MM))
    axis.set(xlim=xlim, ylim=ylim, xlabel="X / distal [mm]", ylabel="Z / dorsal [mm]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, color=LIGHT_GRID, linewidth=0.6)
    axis.set_axisbelow(True)
    for spine in axis.spines.values():
        spine.set_visible(False)


def legend(axis) -> None:
    axis.legend(handles=[Line2D([], [], color=color, linewidth=2.5, label=group)
                         for group, color in GROUP_COLORS.items()],
                loc="lower right", fontsize=8, framealpha=0.85,
                title="collision group\n(different groups\nmust not collide)", title_fontsize=8)


def _outline(axis, points, group: str, hit: bool, zorder: float) -> None:
    axis.add_patch(Polygon(np.asarray(points), closed=True, fill=False, joinstyle="round", zorder=zorder,
                           edgecolor=COLLIDE if hit else GROUP_COLORS[group],
                           linewidth=2.8 if hit else OUTLINE_WIDTH, hatch="////" if hit else None))


def servo(axis, outline, group: str, hit: bool = False) -> None:
    """A servo body, grey, outlined in its group's colour."""

    axis.add_patch(Polygon(np.asarray(outline), closed=True, facecolor=SERVO, edgecolor="none", zorder=0.4))
    _outline(axis, outline, group, hit, 0.7)


def pad(axis, polygon, which: str, hit: bool = False) -> None:
    """A contact interface: ``which`` is ``proximal`` or ``distal``."""

    axis.add_patch(Polygon(np.asarray(polygon), closed=True, alpha=0.85, edgecolor="none", zorder=0.6,
                           facecolor=PROXIMAL_PAD if which == "proximal" else DISTAL_PAD))
    _outline(axis, polygon, which, hit, 0.7)


def pin(axis, center, radius: float, group: str, hit: bool = False, ring: float | None = None) -> None:
    """A pin's clearance circle (its housing) in its group's colour, and
    (``ring``) the zone grown by the body clearance, dashed."""

    axis.add_patch(Circle(center, radius, fill=False, zorder=4.5, edgecolor=COLLIDE if hit else GROUP_COLORS[group],
                          linewidth=2.8 if hit else OUTLINE_WIDTH))
    if ring is not None:
        axis.add_patch(Circle(center, ring, fill=False, linestyle="--", linewidth=1.0,
                              edgecolor=BODY_COLORS.get(group, GRAY), zorder=4))


def roller(axis, center, radius: float, group: str, hit: bool = False) -> None:
    axis.add_patch(Circle(center, radius, facecolor="#f2f2f2", edgecolor="none", zorder=0.5))
    pin(axis, center, radius, group, hit)


def link(axis, points, color: str, width: float = 3.0, zorder: float = 2.0, dashed: bool = False) -> None:
    """A link (``dashed``: a construction line within one rigid body)."""

    points = np.asarray(points, dtype=float)
    axis.plot(points[:, 0], points[:, 1], color=color, linewidth=width, solid_capstyle="round", zorder=zorder,
              linestyle=(0, (4, 3)) if dashed else "-")


def point(axis, at, face: str, edge: str, label: str | None = None, color: str | None = None,
          offset=(0, -18), size: float = 11) -> None:
    """A joint or a named point, its name in italics beside it."""

    at = np.asarray(at, dtype=float)
    axis.plot(*at, "o", markersize=size, markerfacecolor=face, markeredgecolor=edge, markeredgewidth=2.2, zorder=5)
    if label is not None:
        axis.annotate(label, at, xytext=offset, textcoords="offset points", ha="center", va="center",
                      color=color or edge, fontsize=15, fontfamily="serif", fontstyle="italic", zorder=6)


def status(axis, lines) -> None:
    """The pose's readings above the plane, one per line."""

    axis.set_title("\n".join(lines), loc="left", fontsize=9, fontfamily="monospace")


__all__ = ["BODY_COLORS", "GROUP_COLORS", "XLIM", "YLIM", "legend", "link", "pad", "pin", "plane", "point",
           "roller", "servo", "status"]
