"""A linkage portfolio's verified Pareto front in its three objectives.

    python -m visualize.paper.fig_pareto <portfolio dir>

``portfolio dir``: a linkage study's ``portfolio_NNN`` (its ``seeds.csv``,
verified on the fine grid and merged over the runs). A 3D scatter of k_max,
phi_max and pad coverage (the colour too, for depth), with each design's
shadow on the three back walls (grey: the pairwise views). The
representative seeds are ringed and named. Saves
output/paper/fig_pareto_<study>_<preset>.png / .pdf.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from mpl_toolkits.mplot3d import proj3d  # noqa: E402

from visualize.fronts import column, read_rows  # noqa: E402
from visualize.style import COLORS, GRID, INK, MUTED, save, use  # noqa: E402

ROLES = ("best_transmission", "most_pad", "most_curl", "balanced")
SEQUENTIAL = LinearSegmentedColormap.from_list("blue", ("#cfe0f5", COLORS["linkage"], "#0b2a55"))
SHADOW = "#cfcfca"
VIEW = (20.0, -130.0)
"""Elevation and azimuth (deg)."""
LABEL_OFFSETS = {"best_transmission": (62, 10), "most_pad": (-30, 22), "most_curl": (-40, -38),
                 "balanced": (30, 34)}
"""Where each seed's name goes (points), clear of the front."""


def main(argv: list[str]) -> int:
    if not argv:
        print(__doc__)
        return 2
    use("paper")
    portfolio = Path(argv[0]).resolve()
    rows = read_rows(portfolio / "seeds.csv")
    k, phi, pad = column(rows, "torque_ratio_max"), column(rows, "phi_max_deg"), 100.0 * column(rows, "pad_coverage")
    figure = plt.figure(figsize=(6.4, 4.8))
    axis = figure.add_axes((0.03, -0.04, 0.82, 1.06), projection="3d")
    axis.view_init(*VIEW)

    def padded(values: np.ndarray) -> tuple[float, float]:
        margin = 0.03 * (values.max() - values.min())
        return values.min() - margin, values.max() + margin

    (k0, k1), (phi0, phi1), (pad0, pad1) = padded(k), padded(phi), padded(pad)
    # Shadows on the back walls: k_max at its far end, phi_max at its far end, the floor.
    shadow = dict(s=1.5, color=SHADOW, linewidths=0, depthshade=False, alpha=0.6)
    axis.scatter(np.full_like(k, k1), phi, pad, **shadow)
    axis.scatter(k, np.full_like(phi, phi1), pad, **shadow)
    axis.scatter(k, phi, np.full_like(pad, pad0), **shadow)
    order = np.argsort(pad)
    shown = axis.scatter(k[order], phi[order], pad[order], c=pad[order], cmap=SEQUENTIAL, s=4, linewidths=0,
                         depthshade=False)
    figure.colorbar(shown, cax=figure.add_axes((0.86, 0.3, 0.018, 0.42)), label="pad coverage [%]")

    axis.set_xlim(k0, k1)
    axis.set_ylim(phi0, phi1)
    axis.set_zlim(pad0, pad1)
    axis.set_xlabel("k_max  (↓)", color=INK)
    axis.set_ylabel("φ_max [deg]  (↑)", color=INK)
    axis.set_zlabel("pad coverage [%]  (↑)", color=INK)
    for each in (axis.xaxis, axis.yaxis, axis.zaxis):
        each.set_pane_color((1.0, 1.0, 1.0, 0.0))
        each._axinfo["grid"].update(color=GRID, linewidth=0.6)
        each.label.set_size(8.5)
    axis.tick_params(labelsize=7.5, colors=MUTED)
    axis.set_box_aspect(None, zoom=0.92)  # room for the axis labels

    # The seeds over the cloud: rings and names in the projected (2D) view.
    figure.canvas.draw()
    for row in rows:
        roles = [role for role in row["seed_role"].split(";") if role in ROLES]
        if not roles:
            continue
        point = float(row["torque_ratio_max"]), float(row["phi_max_deg"]), 100.0 * float(row["pad_coverage"])
        x, y, _ = proj3d.proj_transform(*point, axis.get_proj())
        label = " / ".join(role.replace("_", " ") for role in roles)
        axis.annotate(label, (x, y), xytext=LABEL_OFFSETS.get(roles[0], (10, 10)), textcoords="offset points",
                      fontsize=7.5, color=INK, ha="center", va="center", zorder=10,
                      bbox=dict(boxstyle="round,pad=0.2", facecolor="white", edgecolor="none", alpha=0.85),
                      arrowprops=dict(arrowstyle="-", color=INK, linewidth=0.8, shrinkA=0, shrinkB=4))
        axis.add_artist(Line2D([x], [y], marker="o", markersize=7, markerfacecolor="none", markeredgecolor=INK,
                               markeredgewidth=1.3, zorder=10, transform=axis.transData))
        print(f"{label:20s} k_max {point[0]:.2f}  phi_max {point[1]:.1f} deg  pad {point[2]:.1f} %")
    study = portfolio.parent
    figure.suptitle(f"{study.parent.name}/{study.name}: verified Pareto front, {len(rows)} designs",
                    color=MUTED, fontsize=8.5, x=0.01, ha="left")
    print(f"saved {save(figure, f'fig_pareto_{study.parent.name}_{study.name}')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
