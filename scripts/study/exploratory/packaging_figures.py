#!/usr/bin/env python3
"""Paper figures and table: packaging against contact accommodation.

Formatting only. Every number comes from
``output/study/exploratory/packaging_tradeoff/`` (``packaging_tradeoff.py``)
and every outline from the existing scene solids, redrawn by category. The
outputs go to ``report/figures/packaging_tradeoff/``:

- ``packaging_comparison.{pdf,png}``;
- ``accommodation_vs_thickness.{pdf,png}``;
- ``mechanism_comparison.tex``;
- ``mechanism_comparison.csv`` and ``tilt_sweep.csv``.

    python scripts/study/exploratory/packaging_figures.py
"""
from __future__ import annotations

import csv
import json
import shutil
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
import packaging_tradeoff as measure  # noqa: E402
from visualize.style import COLORS, DISTAL_PAD, INK, MUTED, PROXIMAL_PAD, SERVO, THEMES  # noqa: E402

DATA = common.OUTPUT / "packaging_tradeoff"
OUT = common.ROOT / "report/figures/packaging_tradeoff"
PANELS = ("code_matched", "2r_dd_tilt0_matched", "2r_dd_tilt90_matched", "belt_2r_matched", "rcj_tilt0_matched",
          "rcj_tilt90_matched")
NAME = {"code_matched": "CoDe, servo 10°", "2r_dd_tilt0_matched": "2R-DD, servo 0°",
        "2r_dd_tilt90_matched": "2R-DD, servo 90°", "belt_2r_matched": "Belt 2R, servo 10°",
        "rcj_tilt0_matched": "RCJ, servo 0°", "rcj_tilt90_matched": "RCJ, servo 90°",
        "rcj_ideal_matched": "RCJ, idealized (no joint servo)"}
ARCH = {"code_matched": "linkage", "2r_dd": "revolute", "belt_2r_matched": "revolute_parallel", "rcj": "rcj"}
ARCH_NAME = {"code_matched": "CoDe", "2r_dd": "2R-DD (servo at joint)", "belt_2r_matched": "Belt 2R", "rcj": "RCJ"}
PIN_FILL, STRUCT = "#fff6d5", "#e8e4da"
plt.rcParams.update(THEMES["paper"])
plt.rcParams.update({"font.size": 7.5, "axes.spines.top": False, "axes.spines.right": False,
                     "pdf.fonttype": 42})


def read(name):
    with (DATA / name).open() as stream:
        return list(csv.DictReader(stream))


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return np.nan


def _poly(p):
    p = np.asarray(p, dtype=float)
    return p[0] if p.ndim == 3 else p


def draw_scene(ax, scene, color):
    s = scene.solids["proximal_pad"]
    for p in s["mcp_servo"].polygons:  # the palm's servo: shown, excluded from the measures
        ax.add_patch(Polygon(_poly(p), closed=True, facecolor="none", edgecolor="#b5b5b5", lw=0.6, ls=(0, (2, 2))))
    for p in s["carrier"].polygons:
        ax.add_patch(Polygon(_poly(p), closed=True, facecolor=STRUCT, edgecolor=MUTED, lw=0.5, zorder=1))
    for p in s["servo"].polygons:
        ax.add_patch(Polygon(_poly(p), closed=True, facecolor=SERVO, edgecolor=INK, lw=0.6, zorder=2))
    for pad, fill in ((s["other_pad"], DISTAL_PAD), (scene.solids["distal_pad"]["other_pad"], PROXIMAL_PAD)):
        for p in pad.polygons:
            ax.add_patch(Polygon(_poly(p), closed=True, facecolor=fill, edgecolor=MUTED, lw=0.5, zorder=2))
    for seg in s["link"].segments + s["pulley"].segments:
        a, b = np.asarray(seg, dtype=float).reshape(-1, 2, 2)[0]
        ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=1.8, solid_capstyle="round", zorder=3)
    for name in ("pin", "pulley", "roller"):
        for centers, radius in s[name].disks:
            c = np.asarray(centers, dtype=float).reshape(-1, 2)[0]
            ax.add_patch(Circle(c, radius, facecolor=PIN_FILL, edgecolor=INK,
                                lw=0.6, zorder=4))


def annotate(ax, row, x_at):
    """Full envelope H_bbox as the primary arrow in every panel; H_finger (to the pad surface) noted
    where hardware hangs below the pads."""
    top, bottom, hw = num(row["dorsal_top_mm"]), num(row["pad_bottom_mm"]), num(row["hardware_bottom_mm"])
    low = min(bottom, hw)
    ax.axhline(top, color=MUTED, lw=0.5, zorder=0)
    ax.axhline(bottom, color=MUTED, lw=0.5, zorder=0)
    ax.annotate("", xy=(x_at, top), xytext=(x_at, low),
                arrowprops=dict(arrowstyle="<->", lw=0.8, color=INK, shrinkA=0, shrinkB=0))
    label_z = (top + low) / 2 if top - low > 18 else top + 7.0  # short arrows: label above
    ax.text(x_at + 2.0, label_z, f"$H_{{bbox}}$\n{num(row['h_bbox_mm']):.1f} mm", fontsize=6.4, va="center",
            color=INK, fontweight="bold")
    ax.text(-12, top + 1.0, f"dorsal top {top:.1f}", fontsize=5.2, color=MUTED, va="bottom")
    ax.text(120, bottom - 1.0, f"pad surface {bottom:.1f}", fontsize=5.2, color=MUTED, va="top", ha="right")
    if row["hardware_below_pads"] == "True":
        ax.axhline(hw, color="#c0392b", lw=0.5, ls=(0, (3, 2)), zorder=0)
        ax.text(-12, -19.0, f"hardware {bottom - hw:.1f} mm below pad surface; "
                f"$H_{{finger}}$ {num(row['h_finger_mm']):.1f} mm", fontsize=5.2, va="bottom", color="#c0392b")


def packaging_comparison(refs, rows):
    fig, axes = plt.subplots(2, 3, figsize=(7.1, 3.6))
    for ax, rid, tag in zip(axes.ravel(), PANELS, "abcdef"):
        ref = refs[rid]
        scene, _, _ = measure.scene_of(ref, common.catalog()["baseline"].modules)
        draw_scene(ax, scene, COLORS[ref.mechanism])
        ax.axhline(0.0, color=INK, lw=0.6, ls=(0, (6, 3)), zorder=0)
        annotate(ax, rows[rid], 97.0)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(-14, 122)
        ax.set_ylim(-20, 46)
        ax.set_title(f"({tag}) {NAME[rid]}", loc="left", fontsize=7.5)
        ax.tick_params(labelsize=6)
    for ax in axes[1]:
        ax.set_xlabel("x (mm)")
    for ax in axes[:, 0]:
        ax.set_ylabel("z (mm)")
    handles = [Polygon([[0, 0]], facecolor=SERVO, edgecolor=INK, label="servo (finger)"),
               plt.Line2D([], [], color=MUTED, lw=1.8, label="links / belt"),
               Circle((0, 0), 1, facecolor=PIN_FILL, edgecolor=INK, label="pin / pulley / roller"),
               Polygon([[0, 0]], facecolor=PROXIMAL_PAD, edgecolor=MUTED, label="proximal pad (20×3 mm)"),
               Polygon([[0, 0]], facecolor=DISTAL_PAD, edgecolor=MUTED, label="distal pad (20×3 mm)"),
               plt.Line2D([], [], color=INK, lw=0.6, ls=(0, (6, 3)), label="proximal reference line"),
               Polygon([[0, 0]], facecolor="none", edgecolor="#b5b5b5", ls=(0, (2, 2)), label="MCP servo (palm, excluded)")]
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False, fontsize=6, bbox_to_anchor=(0.5, 0.03))
    fig.text(0.99, -0.01, "Nominal planar model: zero body clearance, Y layers assumed, extended pose.",
             ha="right", va="bottom", fontsize=5.5, color=MUTED)
    fig.subplots_adjust(left=0.06, right=0.99, top=0.95, bottom=0.2, wspace=0.12, hspace=0.32)
    save(fig, "packaging_comparison")


def accommodation_vs_thickness(sweep, rows, best):
    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.8), gridspec_kw={"width_ratios": [1.3, 1]})
    ax, bx = axes
    for arch, mech in ARCH.items():
        sel = [r for r in sweep if r["architecture"] == arch and r.get("baseline_mounts") == "True"]
        sel.sort(key=lambda r: num(r["tilt_deg"]))
        color = COLORS[mech]
        hb = np.array([num(r["h_bbox_mm"]) for r in sel])
        hf = np.array([num(r["h_finger_mm"]) for r in sel])
        n = np.array([num(r["carried_120"]) for r in sel])
        t = np.array([num(r["tilt_deg"]) for r in sel])
        ok = np.array([r["reference_feasible"] == "True" for r in sel])
        # connect consecutive tilts only where the reference is feasible; infeasible tilts stand alone
        for i in range(len(sel) - 1):
            if ok[i] and ok[i + 1]:
                ax.plot(hb[i:i + 2], n[i:i + 2], color=color, lw=0.8, alpha=0.6, zorder=1)
        ax.scatter(hb[ok], n[ok], color=color, s=12, zorder=2, label=ARCH_NAME[arch])
        ax.scatter(hb[~ok], n[~ok], facecolor="white", edgecolor=color, s=12, zorder=2)
        b = best[arch]
        bt, bh, bn = num(b["best_tilt_deg"]), num(b["h_bbox_mm"]), num(b["carried_120"])
        ax.scatter([bh], [bn], s=110, facecolor="none", edgecolor=color, lw=1.4, zorder=3)
        ax.annotate(f"{bt:g}°", (bh, bn), xytext=(-2, 7), textcoords="offset points", fontsize=6.2, color=color,
                    fontweight="bold", ha="center")
        for tilt in (0.0, 90.0):
            i = np.flatnonzero(t == tilt)
            if len(i) and tilt != bt:
                ax.annotate(f"{tilt:g}°", (hb[i[0]], n[i[0]]), xytext=(3, -8), textcoords="offset points",
                            fontsize=5.3, color=color)
        bx.plot(t, hb, color=color, lw=0.9, marker="o", ms=2.2, label=ARCH_NAME[arch])
        bx.plot(t, hf, color=color, lw=0.7, ls=(0, (3, 2)), alpha=0.8)
        bx.scatter([bt], [bh], s=60, facecolor="none", edgecolor=color, lw=1.2, zorder=3)
    for rid in PANELS:  # the Study 1 reference configurations
        r = rows[rid]
        ax.scatter(num(r["h_bbox_mm"]), num(r["feasible_120"]), marker="*", s=55, facecolor="none",
                   edgecolor=INK, lw=0.6, zorder=4)
    ax.scatter([], [], s=110, facecolor="none", edgecolor=MUTED, lw=1.4,
               label="thinnest evaluated tilt at the\narchitecture's highest count")
    ax.scatter([], [], marker="*", s=55, facecolor="none", edgecolor=INK, lw=0.6, label="Study 1 reference")
    ax.scatter([], [], facecolor="white", edgecolor=MUTED, s=12, label="reference infeasible at that tilt")
    ax.set_xlabel("full envelope $H_{bbox}$ (mm), extended, baseline pads")
    ax.set_ylabel("feasible module pairs at 120° (of 71)")
    ax.set_title("(a) accommodation vs. envelope across servo tilt", loc="left", fontsize=7.5)
    ax.set_xlim(right=56)
    ax.legend(frameon=False, fontsize=5.3, loc="lower right", bbox_to_anchor=(1.03, 0.02), handletextpad=0.3)
    bx.set_xlabel("servo tilt (deg)")
    bx.set_ylabel("height (mm)")
    bx.set_title("(b) $H_{bbox}$ (solid) and $H_{finger}$ (dashed) vs. tilt", loc="left", fontsize=7.5)
    fig.text(0.99, 0.0, "Evaluated fixed-reference tilt sweep (not optimized); nominal zero-clearance planar model.",
             ha="right", va="bottom", fontsize=5.5, color=MUTED)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    save(fig, "accommodation_vs_thickness")


def table(rows, refs, best):
    cols = ("dorsal_top_mm", "pad_bottom_mm", "h_finger_mm", "h_bbox_mm")
    out = []
    for rid in (*PANELS, "rcj_ideal_matched"):
        r = rows[rid]
        out.append({"reference_id": rid, "mechanism": NAME[rid], "model": refs[rid].mechanism,
                    "formulation": common.mechanism(refs[rid].mechanism).formulation,
                    "idealized": r["upper_bound"], **{c: round(num(r[c]), 2) for c in cols},
                    **{c: int(r[c]) for c in ("mounted", "feasible_90", "feasible_120", "feasible_150")}})
    common.shared.write_csv(OUT / "mechanism_comparison.csv", out)
    line = lambda r: (f"{r['mechanism']} & {r['dorsal_top_mm']:.1f} & {r['pad_bottom_mm']:.1f} & "  # noqa: E731
                      f"{r['h_finger_mm']:.1f} & {r['h_bbox_mm']:.1f} & {r['mounted']} & {r['feasible_90']} & "
                      f"{r['feasible_120']} & {r['feasible_150']} \\\\")
    compact = lambda r: (f"{r['mechanism']} & {r['h_bbox_mm']:.1f} & {r['h_finger_mm']:.1f} & "  # noqa: E731
                         f"{r['feasible_120']} & {r['feasible_150']} \\\\")
    for r in out:
        r["mechanism"] = r["mechanism"].replace("°", "$^\\circ$")
    physical = [r for r in out if r["idealized"] != "True"]
    ideal = [r for r in out if r["idealized"] == "True"]
    tex = [
        "% Generated by scripts/study/exploratory/packaging_figures.py from saved results; do not edit by hand.",
        "% Matched 90 mm reference morphology; baseline 20x3 mm pads; 2D side view; MCP servo excluded.",
        "\\begin{table}[t]", "\\centering", "\\footnotesize", "\\setlength{\\tabcolsep}{3pt}",
        "\\caption{Packaging and contact accommodation of the matched reference configurations "
        "(nominal zero-clearance planar model, extended pose, 71 module pairs). $H_\\mathrm{bbox}$: full envelope; "
        "$H_\\mathrm{finger}$: dorsal top to the pad surface.}",
        "\\label{tab:mechanism_comparison}",
        "\\begin{tabular}{lrrrr}", "\\hline",
        "Mechanism & $H_\\mathrm{bbox}$ & $H_\\mathrm{finger}$ & @120$^\\circ$ & @150$^\\circ$ \\\\",
        " & [mm] & [mm] & [pairs] & [pairs] \\\\", "\\hline",
        *[compact(r) for r in physical], "\\hline", "\\end{tabular}", "\\end{table}", "",
        "\\begin{table*}[t]", "\\centering", "\\footnotesize",
        "\\caption{Extended packaging and accommodation diagnostics. Heights in mm relative to the proximal "
        "reference line (extended pose, baseline 20$\\times$3 mm pads, MCP servo excluded). Counts: module pairs "
        "of the 71-pair catalog that mount, and that are feasible at each required flexion (Study 1). "
        "The idealized RCJ omits the joint servo and is not a physical implementation.}",
        "\\label{tab:mechanism_comparison_extended}",
        "\\begin{tabular}{lrrrrrrrr}", "\\hline",
        "Mechanism & Dorsal top & Pad bottom & $H_\\mathrm{finger}$ & $H_\\mathrm{bbox}$ & Mounted & "
        "@90$^\\circ$ & @120$^\\circ$ & @150$^\\circ$ \\\\", "\\hline",
        *[line(r) for r in physical], "\\hline", "\\multicolumn{9}{l}{\\emph{Idealized ablation}} \\\\",
        *[line(r) for r in ideal], "\\hline", "\\end{tabular}", "\\end{table*}", "",
        "\\begin{table}[t]", "\\centering", "\\footnotesize", "\\setlength{\\tabcolsep}{3pt}",
        "\\caption{Thinnest evaluated servo tilt reaching each architecture's highest 120$^\\circ$ count in the "
        "fixed-reference tilt sweep ($-30^\\circ$ to $120^\\circ$, 10$^\\circ$ steps; morphology frozen, not "
        "optimized; nominal zero-clearance planar model).}",
        "\\label{tab:best_observed_tilt}",
        "\\begin{tabular}{lrrrr}", "\\hline",
        "Architecture & Tilt & @120$^\\circ$ & $H_\\mathrm{bbox}$ & $H_\\mathrm{finger}$ \\\\",
        " & [deg] & [pairs] & [mm] & [mm] \\\\", "\\hline",
        *[f"{ARCH_NAME[a]} & {num(b['best_tilt_deg']):g} & {b['carried_120']} & {num(b['h_bbox_mm']):.1f} & "
          f"{num(b['h_finger_mm']):.1f} \\\\" for a, b in best.items()],
        "\\hline", "\\end{tabular}", "\\end{table}", ""]
    (OUT / "mechanism_comparison.tex").write_text("\n".join(tex), encoding="utf-8")


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / f"{name}.pdf", bbox_inches="tight")
    fig.savefig(OUT / f"{name}.png", dpi=300, bbox_inches="tight")
    plt.close(fig)


def main():
    refs = common.references()
    rows = {r["reference_id"]: r for r in read("references.csv")}
    sweep = read("tilt_sweep.csv")
    best = {r["architecture"]: r for r in read("best_observed.csv")}
    packaging_comparison(refs, rows)
    accommodation_vs_thickness(sweep, rows, best)
    table(rows, refs, best)
    for name in ("tilt_sweep.csv", "best_observed.csv"):
        shutil.copy2(DATA / name, OUT / name)
    manifest = json.loads((DATA / "manifest.json").read_text())
    (OUT / "manifest.json").write_text(json.dumps({"data": str(DATA.relative_to(common.ROOT)),
                                                   "data_source": manifest.get("source"),
                                                   "data_inputs_sha256": manifest.get("inputs"),
                                                   "regenerate_inputs": manifest.get("regenerate_inputs"),
                                                   "figures_source": common.shared.git_commit()}, indent=2) + "\n")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
