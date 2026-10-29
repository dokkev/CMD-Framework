"""Each checked pair's gap against the flexion: below 0 the pair collides,
and the first such flexion ends the motion."""

from __future__ import annotations

import numpy as np

from visualize.style import GRID, INK, axes_style

STEP_DEG = 0.25
SHOWN_BELOW_MM = 15.0
"""A pair is plotted when it comes this close somewhere in [0, 180 deg]."""
SHORT = {"distal_pad": "D pad", "proximal_pad": "P pad", "pip_servo": "PIP servo", "mcp_servo": "MCP servo",
         "a_pin": "A pin", "ap_pin": "A' pin", "b_pin": "B pin", "bp_pin": "B' pin", "distal_carrier": "D carrier",
         "proximal_roller": "P roller", "distal_roller": "D roller"}
PAIR_COLORS = ["#2a78d6", "#eb6834", "#1f9e74", "#8a5cd1", "#c9a227", "#d63d8f", "#5b6b7f", "#41a7c9"]
"""Fixed order, one per pair as first met."""


def clearance_axes(axis, phi, margins: dict, phi_max: float, colors: dict, title: str) -> None:
    """``margins`` against ``phi`` (deg), past ``phi_max`` shaded; ``colors``
    (pair label: colour) is shared between the axes of one figure."""

    axes_style(axis, GRID, 0.8)
    axis.axhline(0.0, color=INK, linewidth=1.0)
    axis.axvspan(phi_max, 180.0, color="#f3d6d6", alpha=0.5, linewidth=0)
    for pair, margin in margins.items():
        if not np.isfinite(margin).any() or np.nanmin(margin[np.isfinite(margin)]) > SHOWN_BELOW_MM:
            continue  # far all along: not part of the story
        label = " – ".join(SHORT.get(name, name) for name in pair)
        axis.plot(phi, np.where(np.isfinite(margin), margin, np.nan), color=colors.setdefault(
            label, PAIR_COLORS[len(colors) % len(PAIR_COLORS)]), linewidth=1.8, label=label)
    axis.set_xlim(0.0, 180.0)
    axis.set_ylim(-10.0, 30.0)
    axis.set_xticks(range(0, 181, 30))
    axis.set_xlabel("flexion [deg]")
    axis.set_ylabel("gap [mm]")
    axis.set_title(title, color=INK, fontsize=10, loc="left")
    axis.legend(loc="upper right", frameon=False, fontsize=8)
