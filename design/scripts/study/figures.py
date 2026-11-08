#!/usr/bin/env python3
"""The studies' figures, from their saved results only.

    python scripts/study/figures.py [--out report/figures/study]

Reads ``output/study/study1_fixed_morphology`` (``assembly_results.csv``,
``references.csv``), ``output/study/study2_morphology_adaptation``
(``summary_best.csv``, ``references.csv``, each search's ``run.json``) and,
when present, ``output/study/study2_sensitivity_links``. Writes PNG and PDF.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from visualize.style import COLORS, GRID, INK, MUTED  # noqa: E402

STUDY1 = ROOT / "output/study/study1_fixed_morphology"
STUDY2 = ROOT / "output/study/study2_morphology_adaptation"
SENSITIVITY = ROOT / "output/study/study2_sensitivity_links"
PRIMARY = ("code_matched", "2r_dd_tilt0_matched", "2r_dd_tilt90_matched", "belt_2r_matched", "rcj_tilt0_matched",
           "rcj_tilt90_matched")
UPPER = ("rcj_ideal_matched",)
LABEL = {"code_matched": "CoDe (tilt 10°)", "2r_dd_tilt0_matched": "2R-DD, servo 0°",
         "2r_dd_tilt90_matched": "2R-DD, servo 90°", "belt_2r_matched": "Belt 2R (tilt 10°)",
         "rcj_tilt0_matched": "RCJ, servo 0°", "rcj_tilt90_matched": "RCJ, servo 90°",
         "rcj_ideal_matched": "Ideal RCJ (upper bound)"}
MECH = {"code_matched": "linkage", "2r_dd_tilt0_matched": "revolute", "2r_dd_tilt90_matched": "revolute",
        "belt_2r_matched": "revolute_parallel", "rcj_tilt0_matched": "rcj", "rcj_tilt90_matched": "rcj",
        "rcj_ideal_matched": "rcj_idealized"}
FILLED = {"2r_dd_tilt0_matched": False, "rcj_tilt0_matched": False}
STATE_COLORS = {"feasible": "#2a78d6", "infeasible": "#f2b134", "not_mounted": "#d9d8d3"}
PHI = 120.0


def _shared():
    """The studies' shared module (``shared.py``: the references, the catalog)."""

    if "study_shared" not in sys.modules:
        import importlib.util

        spec = importlib.util.spec_from_file_location("study_shared", HERE / "shared.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules["study_shared"] = module
        spec.loader.exec_module(module)
    return sys.modules["study_shared"]


def color(reference_id: str) -> str:
    return COLORS[MECH[reference_id]]


def read(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def number(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def style(axis) -> None:
    axis.spines[["top", "right"]].set_visible(False)
    axis.spines[["left", "bottom"]].set_color(MUTED)
    axis.tick_params(colors=INK, labelsize=8)
    axis.grid(color=GRID, linewidth=0.6)
    axis.set_axisbelow(True)


def save(figure, out: Path, name: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    for suffix in ("png", "pdf"):
        figure.savefig(out / f"{name}.{suffix}", dpi=200, bbox_inches="tight")
    plt.close(figure)


def state(row: dict[str, str]) -> str:
    if row["mounted"] != "True":
        return "not_mounted"
    return "feasible" if row["feasible"] == "True" else "infeasible"


# Study 1 ---------------------------------------------------------------------

def accommodation_maps(rows, out: Path) -> None:
    """Each reference's length x thickness grid, one panel per sweep, at the required flexion."""

    sweeps = ("proximal", "distal", "combined")
    references = (*PRIMARY, *UPPER)
    at = [r for r in rows if number(r["required_flexion_deg"]) == PHI]
    figure, axes = plt.subplots(len(references), len(sweeps), figsize=(7.2, 1.55 * len(references)), sharex=True,
                                sharey=True)
    code = {"feasible": 2, "infeasible": 1, "not_mounted": 0}
    cmap = matplotlib.colors.ListedColormap([STATE_COLORS[k] for k in ("not_mounted", "infeasible", "feasible")])
    for i, reference in enumerate(references):
        for j, sweep in enumerate(sweeps):
            axis = axes[i, j]
            cells = [r for r in at if r["reference_id"] == reference and r["group"] == sweep]
            side = "distal" if sweep == "distal" else "proximal"
            lengths = sorted({number(r[f"{side}_length_mm"]) for r in cells})
            thicknesses = sorted({number(r[f"{side}_thickness_mm"]) for r in cells})
            grid = np.full((len(thicknesses), len(lengths)), np.nan)
            for r in cells:
                grid[thicknesses.index(number(r[f"{side}_thickness_mm"])), lengths.index(number(r[f"{side}_length_mm"]))] \
                    = code[state(r)]
            axis.imshow(grid, cmap=cmap, vmin=0, vmax=2, origin="lower", aspect="auto",
                        extent=(lengths[0] - 2.5, lengths[-1] + 2.5, -0.5, len(thicknesses) - 0.5))
            axis.set_yticks(range(len(thicknesses)), [f"{t:g}" for t in thicknesses])
            axis.set_xticks(lengths, [f"{v:g}" for v in lengths])
            for x in lengths[:-1]:
                axis.axvline(x + 2.5, color="white", linewidth=1.0)
            for y in range(len(thicknesses) - 1):
                axis.axhline(y + 0.5, color="white", linewidth=1.0)
            axis.tick_params(labelsize=7, colors=INK, length=0)
            for spine in axis.spines.values():
                spine.set_visible(False)
            if i == 0:
                axis.set_title({"proximal": "proximal module varied", "distal": "distal module varied",
                                "combined": "both varied"}[sweep], fontsize=8, color=INK)
            if j == 0:
                axis.set_ylabel(LABEL[reference], fontsize=7.5, color=color(reference), rotation=0, ha="right",
                                va="center", labelpad=6)
    for axis in axes[-1]:
        axis.set_xlabel("module length (mm)", fontsize=8, color=INK)
    figure.text(0.005, 0.5, "thickness (mm)", rotation=90, va="center", fontsize=8, color=MUTED)
    figure.legend([Patch(color=STATE_COLORS[k]) for k in ("feasible", "infeasible", "not_mounted")],
                  ["carried to 120°", "mounts, fails by 120°", "does not mount"], loc="lower center", ncol=3,
                  fontsize=8, frameon=False, bbox_to_anchor=(0.55, -0.02))
    figure.suptitle(f"Study 1 — contact modules a frozen hand carries (φ_req = {PHI:g}°; the other side 20 × 3 mm)",
                    fontsize=9, color=INK, y=1.0)
    figure.tight_layout(rect=(0.02, 0.03, 1, 0.99))
    save(figure, out, "study1_accommodation_maps")


def coverage_versus_rom(rows, out: Path) -> None:
    """(a) The catalog share each reference carries against the flexion asked;
    (b) each mounted assembly's maximum safe flexion against its usable coverage at 120°."""

    references = (*PRIMARY, *UPPER)
    flexions = sorted({number(r["required_flexion_deg"]) for r in rows})
    figure, (left, right) = plt.subplots(1, 2, figsize=(7.2, 3.0))
    total = len({r["module_id"] for r in rows})
    for reference in references:
        mine = [r for r in rows if r["reference_id"] == reference]
        share = [sum(r["feasible"] == "True" for r in mine if number(r["required_flexion_deg"]) == f) / total
                 for f in flexions]
        dashed = reference in UPPER
        left.plot(flexions, share, color=color(reference), marker="o", markersize=5,
                  markerfacecolor=color(reference) if FILLED.get(reference, True) else "white",
                  linestyle=(0, (3, 2)) if dashed else "-", linewidth=1.6, label=LABEL[reference])
        mounted = [r for r in mine if number(r["required_flexion_deg"]) == PHI and r["mounted"] == "True"]
        right.scatter([number(r["phi_max_deg"]) for r in mounted], [number(r["coverage"]) for r in mounted],
                      s=16, color=color(reference) if FILLED.get(reference, True) else "white",
                      edgecolors=color(reference), linewidths=0.9, alpha=0.8, marker="D" if dashed else "o")
    left.set(xlabel="required flexion φ_req (deg)", ylabel=f"share of the {total} module pairs carried",
             xticks=flexions, ylim=(0, None))
    right.set(xlabel="maximum safe flexion φ_max (deg)", ylabel="usable contact coverage at 120°")
    right.axvline(PHI, color=MUTED, linewidth=0.8, linestyle=":")
    for axis, tag in ((left, "a"), (right, "b")):
        style(axis)
        axis.xaxis.label.set_size(8)
        axis.yaxis.label.set_size(8)
        axis.set_title(f"({tag})", loc="left", fontsize=9, color=INK)
    left.legend(fontsize=6.5, frameon=False, loc="lower left")
    figure.tight_layout()
    save(figure, out, "study1_coverage_vs_rom")


def matched_comparison(rows, out: Path) -> None:
    """The catalog at the required flexion, per reference: carried, mounted but failing, not mounted."""

    references = (*PRIMARY, *UPPER)
    at = [r for r in rows if number(r["required_flexion_deg"]) == PHI]
    counts = {k: [sum(state(r) == k for r in at if r["reference_id"] == ref) for ref in references]
              for k in ("feasible", "infeasible", "not_mounted")}
    positions = [*range(len(PRIMARY)), len(PRIMARY) + 0.6]
    figure, axis = plt.subplots(figsize=(7.2, 2.8))
    bottom = np.zeros(len(references))
    for key, label in (("feasible", "carried to 120°"), ("infeasible", "mounts, fails by 120°"),
                       ("not_mounted", "does not mount")):
        bars = axis.bar(positions, counts[key], bottom=bottom, color=STATE_COLORS[key], width=0.7,
                        edgecolor="white", linewidth=1.0, label=label)
        if key == "feasible":
            for bar, value in zip(bars, counts[key]):
                axis.text(bar.get_x() + bar.get_width() / 2, value - 1.5, f"{value}", ha="center", va="top",
                          fontsize=8, color="white", fontweight="bold")
        bottom += counts[key]
    axis.bar(positions[-1], bottom[-1], width=0.7, fill=False, hatch="///", edgecolor=MUTED, linewidth=0)
    axis.set_xticks(positions, [LABEL[r].replace(" (", "\n(").replace(", ", "\n") for r in references], fontsize=7)
    for tick, reference in zip(axis.get_xticklabels(), references):
        tick.set_color(color(reference))
    axis.set_ylabel("module pairs (of the catalog)", fontsize=8)
    axis.axvline(len(PRIMARY) - 0.2, color=MUTED, linewidth=0.8, linestyle=":")
    style(axis)
    axis.grid(axis="x", visible=False)
    axis.legend(fontsize=7, frameon=False, ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.15))
    figure.tight_layout()
    save(figure, out, "study1_matched_comparison")


# Study 2 ---------------------------------------------------------------------

def adaptation_costs(best, floor, out: Path, sensitivity=None) -> None:
    """Each target's least external change (eps_E) and the mechanism change
    within it (eps_Q), per reference, apart; the span floor the module forces."""

    targets = list(dict.fromkeys(r["target"] for r in best))
    references = (*PRIMARY, *UPPER)
    figure, axes = plt.subplots(1, 2, figsize=(7.2, 3.6), sharey=True)
    offsets = np.linspace(-0.33, 0.33, len(references))
    for axis, column, title in ((axes[0], "final_external_cost", "external change ε_E"),
                                (axes[1], "final_mechanism_cost", "mechanism change ε_Q")):
        for k, reference in enumerate(references):
            for i, target in enumerate(targets):
                row = next((r for r in best if r["reference_id"] == reference and r["target"] == target), None)
                if row is None:
                    continue
                y = i + offsets[k]
                if row["status"] == "not_found":
                    axis.scatter([0.0], [y], marker="x", s=22, color=color(reference), linewidths=1.2)
                    continue
                value = number(row[column])
                filled = FILLED.get(reference, True)
                axis.scatter([value], [y], s=26, marker="D" if reference in UPPER else "o",
                             color=color(reference) if filled else "white", edgecolors=color(reference),
                             linewidths=1.0, zorder=3)
                if row["status"] == "frozen_feasible":
                    continue
                low, high = number(row[column.replace("final_", "") + "_min"]), \
                    number(row[column.replace("final_", "") + "_max"])
                if math.isfinite(low) and high > low:
                    axis.plot([low, high], [y, y], color=color(reference), linewidth=0.8, alpha=0.6)
        if column == "final_external_cost":
            for i, target in enumerate(targets):
                axis.plot([floor[target]] * 2, [i - 0.45, i + 0.45], color=INK, linewidth=1.0, linestyle="--")
            if sensitivity:
                for i, target in enumerate(targets):
                    row = next((r for r in sensitivity if r["target"] == target and r["reference_id"] == "code_matched"), None)
                    if row is not None and row["status"] != "not_found":
                        axis.scatter([number(row[column])], [i + offsets[0]], s=40, facecolors="none",
                                     edgecolors=color("code_matched"), linewidths=0.8, linestyle=":", zorder=2)
        axis.set_title(title, fontsize=9, color=INK, loc="left")
        axis.set_xlabel("largest change / reference finger length", fontsize=8)
        style(axis)
        for i in range(len(targets) - 1):
            axis.axhline(i + 0.5, color=GRID, linewidth=0.8)
        axis.grid(axis="y", visible=False)
    axes[0].set_yticks(range(len(targets)), targets, fontsize=7.5)
    axes[0].invert_yaxis()
    handles = [plt.Line2D([], [], marker="D" if r in UPPER else "o", linestyle="", color=color(r),
                          markerfacecolor=color(r) if FILLED.get(r, True) else "white", label=LABEL[r])
               for r in references]
    handles += [plt.Line2D([], [], color=INK, linestyle="--", label="span floor the module forces"),
                plt.Line2D([], [], marker="x", linestyle="", color=MUTED, label="none found (any seed)")]
    if sensitivity:
        handles.append(plt.Line2D([], [], marker="o", linestyle="", markerfacecolor="none",
                                  color=color("code_matched"), label="CoDe without the link face rule"))
    figure.legend(handles=handles, fontsize=6.5, frameon=False, ncol=3, loc="lower center",
                  bbox_to_anchor=(0.5, -0.13))
    figure.suptitle("Study 2 — least change to carry each target (best of seeds; bars: spread over seeds)",
                    fontsize=9, color=INK)
    figure.tight_layout()
    save(figure, out, "study2_adaptation_costs")


def before_after(best, study2: Path, out: Path, cases) -> None:
    """Representative references before and after Level 2, extended and at 120°."""

    from core.optimization.analysis.tradeoff import with_modules
    from core.optimization.study import load_run
    from visualize.finger.view import view_of

    shared = _shared()
    references = {r["reference_id"]: r for r in read(study2 / "references.csv")}
    catalog = shared.catalog(ROOT)
    shown = [(ref, tgt) for ref, tgt in cases
             if any(r["reference_id"] == ref and r["target"] == tgt and r["status"] == "level2" for r in best)]
    figure, axes = plt.subplots(len(shown), 2, figsize=(7.2, 2.2 * len(shown)), squeeze=False)
    for i, (reference_id, target) in enumerate(shown):
        row = next(r for r in best if r["reference_id"] == reference_id and r["target"] == target)
        before_x = [float(v) for v in references[reference_id]["x"].split(";")]
        found, config = shared.load_config(references[reference_id]["config"], ROOT)
        before = view_of(found.name, with_modules(config, catalog["baseline"].modules), before_x)
        step = study2 / "targets" / target / reference_id / f"seed_{row['best_seed']}" / row["final_level"]
        after_config = load_run(step).config  # the config the search ran under, as saved
        after = view_of(found.name, after_config, [float(v) for v in row["final_x"].split(";")])
        for j, (view, tag) in enumerate(((before, "reference (20×3 mm)"),
                                         (after, f"Level 2, {target}"))):
            axis = axes[i, j]
            view.draw(axis, PHI, style="sketch")
            axis.set_aspect("equal")
            axis.set_xlim(-20, 95)
            axis.set_ylim(-70, 30)
            axis.axis("off")
            axis.set_title(f"{LABEL[reference_id]} — {tag}", fontsize=7.5, color=INK, loc="left")
        axes[i, 1].text(1.0, -0.02, f"ε_E {number(row['final_external_cost']):.3f}, ε_Q "
                                    f"{number(row['final_mechanism_cost']):.3f}", transform=axes[i, 1].transAxes,
                        ha="right", va="top", fontsize=7, color=MUTED)
    figure.suptitle(f"Study 2 — representative adaptations, shown at {PHI:g}°", fontsize=9, color=INK)
    figure.tight_layout()
    save(figure, out, "study2_before_after")


def main(argv) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT.parent / "report/figures/study")  # report/ is at the monorepo root
    args = parser.parse_args(argv)
    study1_rows = read(STUDY1 / "assembly_results.csv")
    accommodation_maps(study1_rows, args.out)
    coverage_versus_rom(study1_rows, args.out)
    matched_comparison(study1_rows, args.out)
    best = read(STUDY2 / "summary_best.csv")
    floor = {r["target"]: number(r["external_floor"]) for r in best}  # as the run saved it
    sensitivity = read(SENSITIVITY / "summary_best.csv") if (SENSITIVITY / "summary_best.csv").exists() else None
    adaptation_costs(best, floor, args.out, sensitivity)
    before_after(best, STUDY2, args.out, [("code_matched", "long_thick"), ("2r_dd_tilt90_matched", "long"),
                                          ("belt_2r_matched", "asym_distal_heavy"),
                                          ("rcj_tilt90_matched", "asym_long_proximal")])
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
