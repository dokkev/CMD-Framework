#!/usr/bin/env python3
"""Study 2's landscape figures (problem formulation A.6.3), from a run's saved tables.

    python scripts/study/study2_morphology_adaptation/figure_landscape.py <output>

``<output>`` is a ``landscape.py`` run's folder (its ``landscape.csv``,
``markers.csv`` and ``manifest.json``). For each required flexion and section,
in ``<output>/figures/`` (PNG):

- ``eps_d_<section>_q<q_req>.png``: each reference's least external change, one
  cell per optimized grid point (nothing interpolated), the points not found
  hatched, the rechecked ones dotted, the catalog's pairs marked;
- ``excess_<section>_q<q_req>.png``: the same, less the mounting floor;
- ``regions_<section>_q<q_req>.png``: the references' regions ``eps_d* <= tau``
  for each threshold, and the region where no design was found, outlined on one
  domain (the architectures compared by region, not by pointwise differences).
"""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from visualize.style import COLORS, DASHES, INK, MECHANISMS, MUTED, SHORT_LABELS, use  # noqa: E402

CHANGE = LinearSegmentedColormap.from_list("change", ["#f4f7fb", "#9dbbe3", "#2a5fa8", "#0d2b57"])
NOT_FOUND = "#d9d8d4"
SECTION_LABELS = {"symmetric": ("module length $\\ell_p = \\ell_d$ (mm)", "module thickness $h_p = h_d$ (mm)"),
                  "proximal": ("proximal module length $\\ell_p$ (mm)", "proximal module thickness $h_p$ (mm)"),
                  "distal": ("distal module length $\\ell_d$ (mm)", "distal module thickness $h_d$ (mm)")}


def read(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def number(text: str) -> float:
    try:
        return float(text)
    except (TypeError, ValueError):
        return math.nan


def edges(values: list[float]) -> np.ndarray:
    """Cell edges about each grid value: halfway to its neighbours, half a step past the ends."""

    values = np.asarray(sorted(values), dtype=float)
    if len(values) == 1:
        return np.array([values[0] - 0.5, values[0] + 0.5])
    middle = (values[1:] + values[:-1]) / 2
    return np.concatenate(([values[0] - (middle[0] - values[0])], middle, [values[-1] + (values[-1] - middle[-1])]))


def region_segments(mask: np.ndarray, x_edges: np.ndarray, y_edges: np.ndarray) -> list:
    """The outline of the cells in ``mask`` (indexed ``[length, thickness]``)
    where they meet cells outside it; not along the domain's edge, where the
    region may go on past the grid."""

    segments = []
    n_i, n_j = mask.shape
    for i in range(n_i):
        for j in range(n_j):
            if not mask[i, j]:
                continue
            if i + 1 < n_i and not mask[i + 1, j]:
                segments.append([(x_edges[i + 1], y_edges[j]), (x_edges[i + 1], y_edges[j + 1])])
            if i > 0 and not mask[i - 1, j]:
                segments.append([(x_edges[i], y_edges[j]), (x_edges[i], y_edges[j + 1])])
            if j + 1 < n_j and not mask[i, j + 1]:
                segments.append([(x_edges[i], y_edges[j + 1]), (x_edges[i + 1], y_edges[j + 1])])
            if j > 0 and not mask[i, j - 1]:
                segments.append([(x_edges[i], y_edges[j]), (x_edges[i + 1], y_edges[j])])
    return segments


def grid_of(rows: list[dict[str, str]], column: str):
    """The rows' values of ``column`` on their grid: lengths, thicknesses, values
    ``[length, thickness]``, and the status and rechecked flags alike."""

    lengths = sorted({number(r["length_mm"]) for r in rows})
    thicknesses = sorted({number(r["thickness_mm"]) for r in rows})
    shape = (len(lengths), len(thicknesses))
    values, status, rechecked = np.full(shape, np.nan), np.full(shape, "", dtype=object), np.zeros(shape, bool)
    for r in rows:
        i, j = lengths.index(number(r["length_mm"])), thicknesses.index(number(r["thickness_mm"]))
        values[i, j] = number(r[column])
        status[i, j] = r["status"]
        rechecked[i, j] = r.get("rechecked") == "True"
    return lengths, thicknesses, values, status, rechecked


def ordered(references: dict[str, dict[str, str]]) -> list[str]:
    rank = {name: index for index, name in enumerate(MECHANISMS)}
    return sorted(references, key=lambda rid: (rank.get(references[rid]["mechanism"], 99), rid))


def label(reference: dict[str, str]) -> str:
    name = SHORT_LABELS.get(reference["mechanism"], reference["mechanism"])
    return f"{name} (upper bound)" if reference.get("upper_bound") == "True" else name


def draw_markers(axis, markers: list[dict[str, str]], section: str, lengths, thicknesses, cells: bool = True) -> None:
    """The catalog's pairs on this section: their cell outlined when they lie on
    the grid (``cells``), else a ring at their place; each named."""

    x_edges, y_edges = edges(lengths), edges(thicknesses)
    for m in markers:
        if m["section"] != section:
            continue
        x, y = number(m["length_mm"]), number(m["thickness_mm"])
        on = [i for i, v in enumerate(lengths) if math.isclose(v, x)], \
            [j for j, v in enumerate(thicknesses) if math.isclose(v, y)]
        if cells and on[0] and on[1]:
            i, j = on[0][0], on[1][0]
            axis.add_patch(plt.Rectangle((x_edges[i], y_edges[j]), x_edges[i + 1] - x_edges[i],
                                         y_edges[j + 1] - y_edges[j], fill=False, edgecolor=INK, linewidth=1.4,
                                         zorder=5))
            at = (x_edges[i] + 0.06 * (x_edges[i + 1] - x_edges[i]), y_edges[j] + 0.08 * (y_edges[j + 1] - y_edges[j]))
            axis.text(*at, m["module_id"], fontsize=5, color=INK, ha="left", va="bottom", zorder=6,
                      bbox={"boxstyle": "square,pad=0.1", "facecolor": "white", "edgecolor": "none", "alpha": 0.8})
        else:
            axis.plot(x, y, marker="o", markersize=5, markerfacecolor="none", markeredgecolor=INK,
                      markeredgewidth=0.9, zorder=5)
            axis.annotate(m["module_id"], (x, y), xytext=(4, 4), textcoords="offset points", fontsize=6, color=INK,
                          zorder=5)


def heatmaps(rows, references, markers, section, flexion, column, title, path) -> Path:
    ids = ordered(references)
    finite = [number(r[column]) for r in rows if math.isfinite(number(r[column]))]
    top = max(max(finite, default=0.0), 1e-6)
    figure, axes = plt.subplots(1, len(ids), figsize=(2.9 * len(ids) + 0.8, 3.2), squeeze=False, sharey=True)
    image = None
    for axis, rid in zip(axes[0], ids):
        own = [r for r in rows if r["reference_id"] == rid]
        lengths, thicknesses, values, status, rechecked = grid_of(own, column)
        x_edges, y_edges = edges(lengths), edges(thicknesses)
        image = axis.pcolormesh(x_edges, y_edges, np.ma.masked_invalid(values).T, cmap=CHANGE, vmin=0.0, vmax=top,
                                shading="flat")
        for i, length in enumerate(lengths):
            for j, thickness in enumerate(thicknesses):
                box = (x_edges[i], y_edges[j]), x_edges[i + 1] - x_edges[i], y_edges[j + 1] - y_edges[j]
                if status[i, j] == "not_found":
                    axis.add_patch(plt.Rectangle(*box, facecolor=NOT_FOUND, edgecolor=MUTED, hatch="///",
                                                 linewidth=0.0))
                    continue
                value = values[i, j]
                text = {"level1": "L1", "frozen_feasible": "0"}.get(status[i, j], f"{value:.2f}")
                dark = math.isfinite(value) and value > 0.55 * top
                axis.text(length, thickness, text, ha="center", va="center", fontsize=6,
                          color="white" if dark else INK)
                if rechecked[i, j]:
                    axis.plot(x_edges[i + 1] - 0.15 * (x_edges[i + 1] - x_edges[i]),
                              y_edges[j + 1] - 0.15 * (y_edges[j + 1] - y_edges[j]), marker=".", markersize=3,
                              color="white" if dark else MUTED)
        draw_markers(axis, markers, section, lengths, thicknesses)
        axis.set_title(label(references[rid]), fontsize=9, color=COLORS.get(references[rid]["mechanism"], INK))
        axis.set_xlabel(SECTION_LABELS[section][0])
        axis.set_xticks(lengths)
        axis.set_yticks(thicknesses)
    axes[0][0].set_ylabel(SECTION_LABELS[section][1])
    bar = figure.colorbar(image, ax=axes[0].tolist(), fraction=0.025, pad=0.01)
    bar.set_label(f"{title} (fraction of $L_f$)")
    figure.suptitle(f"{title}, {section} section, $q_{{req}}$ = {flexion:g}°   "
                    f"(hatched: not found; L1: mechanism only; dot: rechecked)", fontsize=9, color=INK)
    figure.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(figure)
    return path


def regions(rows, references, markers, section, flexion, thresholds, path) -> Path:
    ids = ordered(references)
    panels = [(f"$\\varepsilon_d^* \\leq {tau:g}$", tau) for tau in thresholds] + [("not found", None)]
    figure, axes = plt.subplots(1, len(panels), figsize=(2.9 * len(panels), 3.2), squeeze=False, sharey=True)
    for axis, (title, tau) in zip(axes[0], panels):
        for index, rid in enumerate(ids):
            own = [r for r in rows if r["reference_id"] == rid]
            lengths, thicknesses, values, status, _ = grid_of(own, "final_external_cost")
            x_edges, y_edges = edges(lengths), edges(thicknesses)
            mask = (status == "not_found") if tau is None else ((status != "not_found") & (values <= tau + 1e-9))
            mechanism = references[rid]["mechanism"]
            # Each reference's outline shifted a little, so coincident ones show side by side.
            shift = (index - (len(ids) - 1) / 2) * 0.012
            offset = np.array([shift * (x_edges[-1] - x_edges[0]), shift * (y_edges[-1] - y_edges[0])])
            segments = [np.asarray(segment) + offset for segment in region_segments(mask, x_edges, y_edges)]
            axis.add_collection(LineCollection(segments,
                                               colors=COLORS.get(mechanism, INK),
                                               linestyles=DASHES.get(mechanism, "-"), linewidths=1.6))
            axis.set_xlim(x_edges[0], x_edges[-1])
            axis.set_ylim(y_edges[0], y_edges[-1])
            axis.set_xticks(lengths)
            axis.set_yticks(thicknesses)
        draw_markers(axis, markers, section, lengths, thicknesses, cells=False)
        axis.set_title(title, fontsize=9)
        axis.set_xlabel(SECTION_LABELS[section][0])
    axes[0][0].set_ylabel(SECTION_LABELS[section][1])
    handles = [Line2D([], [], color=COLORS.get(references[rid]["mechanism"], INK),
                      linestyle=DASHES.get(references[rid]["mechanism"], "-"), label=label(references[rid]))
               for rid in ids]
    figure.legend(handles=handles, loc="center left", frameon=False, fontsize=8, bbox_to_anchor=(1.0, 0.5))
    figure.suptitle(f"Regions, {section} section, $q_{{req}}$ = {flexion:g}°   (outline: where the region ends "
                    f"inside the grid; smaller modules lie lower left)", fontsize=9, color=INK)
    figure.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(figure)
    return path


def figures(output: Path) -> list[Path]:
    use("report")
    rows = read(output / "landscape.csv")
    markers = read(output / "markers.csv") if (output / "markers.csv").is_file() else []
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    thresholds = [float(v) for v in manifest["config"].get("thresholds") or (0.0, 0.05, 0.10)]
    references = {r["reference_id"]: r for r in read(output / "references.csv")}
    folder = output / "figures"
    folder.mkdir(exist_ok=True)
    made = []
    for flexion in sorted({number(r["required_flexion_deg"]) for r in rows}):
        for section in [s for s in SECTION_LABELS if any(r["section"] == s for r in rows)]:
            own = [r for r in rows if number(r["required_flexion_deg"]) == flexion and r["section"] == section]
            present = {rid: references[rid] for rid in {r["reference_id"] for r in own}}
            tag = f"{section}_q{flexion:g}"
            made.append(heatmaps(own, present, markers, section, flexion, "final_external_cost",
                                 "Least external change $\\varepsilon_d^*$", folder / f"eps_d_{tag}.png"))
            made.append(heatmaps(own, present, markers, section, flexion, "external_excess",
                                 "Excess above the mounting floor", folder / f"excess_{tag}.png"))
            made.append(regions(own, present, markers, section, flexion, thresholds, folder / f"regions_{tag}.png"))
    return made


def main(argv: list[str]) -> int:
    if len(argv) != 1:
        print(__doc__)
        return 2
    output = Path(argv[0])
    if not output.is_absolute():
        output = REPOSITORY_ROOT / output
    for path in figures(output):
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
