"""A finger view's figures: its poses as solids, and the paper's schematic motion."""

from __future__ import annotations

import numpy as np

from visualize import sketch
from visualize.style import COLORS, INK

TRAIL_STEP_DEG = 15.0


def poses_to(end_deg: float, step_deg: float) -> list[float]:
    """Every ``step_deg`` from extension, then ``end_deg`` itself."""

    return [*np.arange(0.0, end_deg - 1e-9, step_deg), end_deg]


def motion_panel(axis, view, pose_deg: float, *, trail_step_deg: float | None = TRAIL_STEP_DEG,
                 icr: bool = True, title: bool = True) -> None:
    """The paper's schematic at ``pose_deg`` (colliding parts outlined red)
    over its distal pad every ``trail_step_deg`` before it, with the dotted
    path of the distal segment's centre of rotation from extension (its start a ring)."""

    view.draw(axis, pose_deg, style="sketch")
    if trail_step_deg:
        trail = np.arange(0.0, pose_deg + 1e-9, trail_step_deg)
        sketch.pad_trail(axis, [view.distal_pad(ghost) for ghost in trail], 0.2 + 0.8 * trail / 180.0,
                         COLORS[view.mechanism], alpha=0.6)
    if icr and pose_deg > 0.0:
        sketch.icr_path(axis, view.icr(np.linspace(0.0, pose_deg, 121)), size=18, linewidth=1.2, end=None)
    sketch.frame(axis)
    if title:
        axis.set_title(f"φ = {pose_deg:.0f}°", color=INK, fontsize=9, loc="left")


__all__ = ["motion_panel", "poses_to"]
