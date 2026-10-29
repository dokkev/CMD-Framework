"""One look for every figure: the mechanisms' colours (each with its own
marker and dash, so no series is told by colour alone), the solids' fills,
recessive axes, and where figures are saved (``output/``)."""

from __future__ import annotations

from pathlib import Path

import matplotlib
from matplotlib.colors import LinearSegmentedColormap

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
OUTPUT = REPOSITORY_ROOT / "output"
PAPER = OUTPUT / "paper"
COMPARISON = OUTPUT / "comparison"

MECHANISMS = ("revolute", "revolute_parallel", "revolute_idealized", "rcj", "linkage")
"""In the order the figures show them (the ablation): the 2R with its servo
on the joint, on ``A'`` (the actuator moved), without one (the fixed pivot
alone), then the two whose centre of rotation moves."""
COLORS = {"revolute_idealized": "#8a8984", "revolute": "#eb6834", "revolute_parallel": "#8a5cd1",
          "rcj": "#1f9e74", "linkage": "#2a78d6"}
MARKERS = {"revolute_idealized": "D", "revolute": "s", "revolute_parallel": "v", "rcj": "^", "linkage": "o"}
DASHES = {"revolute_idealized": (0, (3, 2)), "revolute": "-", "revolute_parallel": (0, (6, 2)), "rcj": "-",
          "linkage": "-"}
LABELS = {"revolute_idealized": "2R, ideal (no joint servo)", "revolute": "2R, servo at joint",
          "revolute_parallel": "2R, servo on A' (belt)", "rcj": "RCJ (rolling contact)",
          "linkage": "CoDe (crossed four-bar)"}
SHORT_LABELS = {"revolute_idealized": "2R ideal", "revolute": "2R", "revolute_parallel": "2R belt",
                "rcj": "RCJ", "linkage": "CoDe"}
NAMES = {"linkage": "CoDe linkage", "revolute": "Serial 2R"}
"""The two-mechanism comparisons' names."""

INK, MUTED, GRID = "#1f1f1e", "#6b6a66", "#e6e5e0"
LIGHT_GRID = "#e6e8eb"
SERVO, COLLIDE = "#c9ced6", "#d03238"
PROXIMAL_PAD, DISTAL_PAD = "#f7b6bd", "#b4bdf0"

THEMES = {
    "paper": {"font.size": 9, "font.family": "DejaVu Sans", "axes.edgecolor": MUTED, "axes.labelcolor": INK,
              "xtick.color": MUTED, "ytick.color": MUTED, "figure.facecolor": "white",
              "savefig.facecolor": "white"},
    "report": {"font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
               "ytick.color": MUTED},
}
"""Matplotlib settings: the paper's figures, and the studies' working plots."""


def use(theme: str) -> None:
    matplotlib.rcParams.update(THEMES[theme])


def axes_style(axis, grid: str = GRID, linewidth: float = 0.7) -> None:
    """A light grid behind the data, no top and right spines."""

    axis.grid(color=grid, linewidth=linewidth)
    axis.set_axisbelow(True)
    for side in ("top", "right"):
        axis.spines[side].set_visible(False)


def shades(color: str) -> LinearSegmentedColormap:
    """White to ``color``: darker as the finger flexes further."""

    return LinearSegmentedColormap.from_list("flexion", ["#ffffff", color])


def save(figure, name: str) -> Path:
    """A paper figure, as output/paper/<name>.png and .pdf."""

    PAPER.mkdir(parents=True, exist_ok=True)
    path = PAPER / f"{name}.png"
    figure.savefig(path, dpi=200)
    figure.savefig(path.with_suffix(".pdf"))
    return path


def save_comparison(figure, name: str, **options) -> Path:
    """A working figure, as output/comparison/<name>.png."""

    COMPARISON.mkdir(parents=True, exist_ok=True)
    path = COMPARISON / f"{name}.png"
    figure.savefig(path, **{"dpi": 130, **options})
    return path
