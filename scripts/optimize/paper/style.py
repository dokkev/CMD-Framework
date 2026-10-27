"""One look for the paper's figures: the mechanisms' colours (each with its
own marker and dash, so no series is told by colour alone), recessive axes,
and where the figures are saved (output/paper/, PNG and PDF)."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
OUTPUT = REPOSITORY_ROOT / "output" / "paper"

COLORS = {"revolute_idealized": "#8a8984", "revolute": "#eb6834", "revolute_parallel": "#8a5cd1",
          "rcj": "#1f9e74", "linkage": "#2a78d6"}
MARKERS = {"revolute_idealized": "D", "revolute": "s", "revolute_parallel": "v", "rcj": "^", "linkage": "o"}
DASHES = {"revolute_idealized": (0, (3, 2)), "revolute": "-", "revolute_parallel": (0, (6, 2)), "rcj": "-",
          "linkage": "-"}
INK, MUTED, GRID = "#1f1f1e", "#6b6a66", "#e6e5e0"

plt.rcParams.update({"font.size": 9, "font.family": "DejaVu Sans", "axes.edgecolor": MUTED,
                     "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
                     "figure.facecolor": "white", "savefig.facecolor": "white"})


def axes_style(axis) -> None:
    axis.grid(color=GRID, linewidth=0.7)
    axis.set_axisbelow(True)
    for side in ("top", "right"):
        axis.spines[side].set_visible(False)


def save(figure, name: str) -> Path:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    path = OUTPUT / f"{name}.png"
    figure.savefig(path, dpi=200)
    figure.savefig(path.with_suffix(".pdf"))
    return path
