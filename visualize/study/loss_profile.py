"""What each mechanism's contact faces lose as the finger flexes, by probe
depth and by what hides them (a diagnostic of the loss rule, not a result).

    python -m visualize.study.loss_profile <mechanism>=<seeds.csv>[:<role>] ... [--output name]

Each argument: a mechanism, a verified front (``portfolio_NNN/seeds.csv``)
and the design to take (a seed role; default the most pad coverage). For
every design, at ``phi`` 0 to 150 deg every 5 deg: the share of the two
faces' length lost under the probe rule at each depth of DEPTHS_MM (and
under the segment rule), with the length each category of solid alone
would hide, and how close each category comes in front of each face.
Writes output/comparison/<name>.csv (default ``loss_profile``) and
<name>_intrusion.csv, and a figure, one row per design: the loss against
the flexion, a line per depth; the loss by category at LARGE_DEPTH_MM; how
close the hardware comes in front of each face.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import core.morphology.linkage.packaging as packaging_module  # noqa: E402
from core.optimization.analysis.loss import CATEGORIES, face_loss, intrusion_mm, scene  # noqa: E402
from core.optimization.study import load_run  # noqa: E402
from visualize.fronts import read_rows, with_role  # noqa: E402
from visualize.style import COLORS, GRID, INK, MUTED, SHORT_LABELS, axes_style, save_comparison, use  # noqa: E402

packaging_module.EXACT_BAND_MM = 1.0e6
DEPTHS_MM = (0.0, 2.0, 3.0, 5.0, 10.0, 20.0)
LARGE_DEPTH_MM = 20.0
PHI_DEG = np.arange(0.0, 150.0 + 1e-9, 5.0)
SHADES = ("#d9e6f5", "#b3cdeb", "#86b0de", "#5590cf", "#2a6fc4", "#0b3d80")
"""One hue, light to dark with the depth."""
CATEGORY_COLORS = {"other_pad": "#2a78d6", "carrier": "#8a5cd1", "link": "#eb6834", "pin": "#c9a227",
                   "servo": "#5b6b7f", "pulley": "#d63d8f", "roller": "#1f9e74"}
HARDWARE = ("carrier", "link", "pin", "servo", "pulley", "roller")


def design_of(argument: str):
    """``mechanism=seeds.csv[:role]``: the mechanism, its config, the design and a label."""

    mechanism, _, rest = argument.partition("=")
    path, _, role = rest.partition(":")
    seeds = Path(path)
    rows = read_rows(seeds)
    row = with_role(rows, role) if role else max(rows, key=lambda r: float(r["pad_coverage"]))
    run = load_run(seeds.parent.parent / row["run"].split("/")[-1])
    label = f"{SHORT_LABELS.get(mechanism, mechanism)} {seeds.parent.parent.name}{f' {role}' if role else ' best'}"
    return mechanism, run.config, run.mechanism.read_design(row), label, row


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("designs", nargs="+")
    parser.add_argument("--output", default="loss_profile")
    args = parser.parse_args(argv)
    use("report")
    table, intrusions = [], []
    figure, axes = plt.subplots(len(args.designs), 3, figsize=(13.5, 2.9 * len(args.designs)), squeeze=False,
                                constrained_layout=True)
    for row_axes, argument in zip(axes, args.designs):
        mechanism, config, x, label, row = design_of(argument)
        built = scene(mechanism, config, x, PHI_DEG)
        phi = built.phi_deg
        total_length = None
        for depth in DEPTHS_MM:
            loss = face_loss(built, "probe", depth)
            length = loss["proximal_pad"].length + loss["distal_pad"].length
            total_length = length
            share = 100.0 * (loss["proximal_pad"].lost + loss["distal_pad"].lost) / length
            for i, angle in enumerate(phi):
                table.append({"design": label, "mechanism": mechanism, "rule": "probe", "depth_mm": depth,
                              "phi_deg": round(float(angle), 2), "lost_share_pct": float(share[i]),
                              **{f"{side}_lost_mm": float(loss[side].lost[i]) for side in loss},
                              **{f"{name}_mm": float(loss["proximal_pad"].by_category[name][i]
                                                     + loss["distal_pad"].by_category[name][i])
                                 for name in CATEGORIES}})
            index = DEPTHS_MM.index(depth)
            row_axes[0].plot(phi, share, color=SHADES[index], linewidth=1.6, label=f"probe {depth:g} mm")
            if depth == LARGE_DEPTH_MM:
                bottom = np.zeros(len(phi))
                for name in CATEGORIES:
                    part = 100.0 * (loss["proximal_pad"].by_category[name] + loss["distal_pad"].by_category[name]) \
                        / length
                    if part.max() <= 0.0:
                        continue
                    row_axes[1].plot(phi, part, color=CATEGORY_COLORS[name], linewidth=1.6, label=name)
        segment = face_loss(built, "segment")
        share = 100.0 * (segment["proximal_pad"].lost + segment["distal_pad"].lost) / total_length
        row_axes[0].plot(phi, share, color=INK, linestyle=(0, (3, 2)), linewidth=1.4, label="segment (v4)")
        for i, angle in enumerate(phi):
            table.append({"design": label, "mechanism": mechanism, "rule": "segment", "depth_mm": "",
                          "phi_deg": round(float(angle), 2), "lost_share_pct": float(share[i]),
                          **{f"{side}_lost_mm": float(segment[side].lost[i]) for side in segment}})
        for side, dash in (("proximal_pad", "-"), ("distal_pad", (0, (4, 2)))):
            for name in HARDWARE:
                near = intrusion_mm(built, side, [name], 30.0, 0.25)
                for i, angle in enumerate(phi):
                    intrusions.append({"design": label, "mechanism": mechanism, "face": side, "category": name,
                                       "phi_deg": round(float(angle), 2),
                                       "nearest_mm": "" if np.isnan(near[i]) else float(near[i])})
                if np.isfinite(near).any():
                    row_axes[2].plot(phi, near, color=CATEGORY_COLORS[name], linestyle=dash, linewidth=1.4,
                                     label=f"{name}, {side.split('_')[0]}")
        for axis in row_axes:
            axes_style(axis, GRID)
            axis.set_xlim(0.0, 150.0)
            axis.set_xticks(range(0, 151, 30))
        row_axes[0].set_ylim(0.0, 100.0)
        row_axes[0].set_ylabel("faces lost [% of P + D]", color=INK)
        row_axes[0].set_title(f"{label}: coverage {100 * float(row['pad_coverage']):.0f} %", color=COLORS.get(
            mechanism, INK), fontsize=9, loc="left")
        row_axes[1].set_ylim(0.0, 100.0)
        row_axes[1].set_title(f"what hides it, probe {LARGE_DEPTH_MM:g} mm (each alone)", color=MUTED, fontsize=9,
                              loc="left")
        row_axes[2].set_ylim(0.0, 30.0)
        row_axes[2].set_title("how close the hardware comes in front (solid: proximal face)", color=MUTED,
                              fontsize=9, loc="left")
        row_axes[2].set_ylabel("nearest [mm]", color=INK)
        for axis in row_axes:
            if axis.get_lines():
                axis.legend(frameon=False, fontsize=6.5, loc="upper left")
    for axis in axes[-1]:
        axis.set_xlabel("distal flexion φ [deg]", color=INK)
    figure.suptitle("Contact face lost to what stands in front of it (probe along the face's normal, any Y layer); "
                    "diagnostic", color=MUTED, fontsize=9, x=0.01, ha="left")
    path = save_comparison(figure, args.output, dpi=120)
    for name, rows in ((args.output, table), (f"{args.output}_intrusion", intrusions)):
        columns = list(dict.fromkeys(key for row in rows for key in row))
        with (path.parent / f"{name}.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)
    print(f"saved {path} and {path.parent / (args.output + '.csv')}, {path.parent / (args.output + '_intrusion.csv')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
