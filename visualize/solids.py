"""The finger drawn as its solids in the flexion plane: servos grey, pads
filled (proximal pink, distal blue), links in the mechanism's colour,
servo-driven joints yellow and free ones white.

In the proximal frame (``O`` at the origin, ``+X`` distal, ``+Z`` dorsal), in mm.
"""

from __future__ import annotations

import numpy as np
from matplotlib.patches import Polygon

from core.morphology import Fingertip, FingertipParameters
from visualize.style import GRID, INK, MUTED, axes_style

ACTIVE, PASSIVE = "#f5c518", "white"
"""Joint fills: driven by a servo, or free."""
PHALANX = "#e8e4da"


def plane_axes(axis, linewidth: float = 0.6) -> None:
    """Equal scale on a light grid."""

    axis.set_aspect("equal")
    axes_style(axis, GRID, linewidth)


def patch(axis, points, color, alpha=0.9, edge=MUTED, width=0.8, zorder=1.0) -> None:
    axis.add_patch(Polygon(np.asarray(points), closed=True, facecolor=color, alpha=alpha,
                           edgecolor=edge, linewidth=width, zorder=zorder))


def line(axis, points, color, alpha=1.0, width=2.4) -> None:
    points = np.asarray(points)
    axis.plot(points[:, 0], points[:, 1], color=color, linewidth=width, alpha=alpha, solid_capstyle="round", zorder=3)


def joints(axis, active, passive) -> None:
    """The servo-driven joints filled yellow, the free ones white."""

    for points, fill in ((passive, PASSIVE), (active, ACTIVE)):
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        axis.scatter(points[:, 0], points[:, 1], s=70, facecolor=fill, edgecolor=INK, linewidth=1.4, zorder=5)


def phalanx(axis, joint, tip, color=PHALANX) -> None:
    """The distal phalanx: the fingertip's side silhouette (``core.morphology.Fingertip``)
    as long as its segment, from its joint along it."""

    joint, tip = np.asarray(joint, dtype=float), np.asarray(tip, dtype=float)
    length = float(np.linalg.norm(tip - joint))
    fingertip = Fingertip(FingertipParameters(total_length_mm=length))
    outline = np.array(fingertip.side_silhouette_mm(16), dtype=float)
    outline[:, 1] -= fingertip.rotation_axis_z_mm
    e = (tip - joint) / length
    n = np.array((-e[1], e[0]))
    points = joint + outline[:, :1] * e + outline[:, 1:] * n
    axis.add_patch(Polygon(points, closed=True, facecolor=color, edgecolor=MUTED, linewidth=0.8, zorder=0.3))


def linkage_bars(axis, linkage, b, c, bp, color, alpha=1.0, width=2.4) -> None:
    """The crossed four-bar's links: the proximal body ``O-A-A'`` and the
    distal ``B'-B-C`` in ``color``, the rocker ``A-B'`` dark, the crank ``A'-B`` grey."""

    o, a, ap = (0.0, 0.0), (linkage.a.x_mm, linkage.a.z_mm), (linkage.ap.x_mm, linkage.ap.z_mm)
    line(axis, (o, a, ap, o), color, alpha, width)
    line(axis, (a, bp), INK, alpha, width)
    line(axis, (ap, b), MUTED, alpha, width)
    line(axis, (bp, b, c, bp), color, alpha, width)
