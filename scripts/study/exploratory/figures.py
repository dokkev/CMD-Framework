#!/usr/bin/env python3
"""Figures for E01-E10, read only from the saved results under output/study/exploratory/."""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

COLOR = {"code_matched": "#2a78d6", "2r_dd_tilt0_matched": "#e8643b", "2r_dd_tilt90_matched": "#e8643b",
         "belt_2r_matched": "#8a55d6", "rcj_tilt0_matched": "#1d9a6c", "rcj_tilt90_matched": "#1d9a6c",
         "rcj_ideal_matched": "#7fc4a9"}
DASH = {"2r_dd_tilt0_matched": "--", "rcj_tilt0_matched": "--", "rcj_ideal_matched": ":"}
INK = "#333333"
plt.rcParams.update({"font.size": 8, "axes.edgecolor": "#888888", "axes.labelcolor": INK, "xtick.color": INK,
                     "ytick.color": INK, "axes.spines.top": False, "axes.spines.right": False})


def read(path):
    with (common.OUTPUT / path).open() as stream:
        return list(csv.DictReader(stream))


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return np.nan


def save(fig, name):
    common.FIGURES.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(common.FIGURES / f"{name}.{ext}", dpi=200, bbox_inches="tight")
    plt.close(fig)


def e01():
    data = np.load(common.OUTPUT / "e01_packaging/profiles.npz")
    fig, ax = plt.subplots(figsize=(6.2, 2.6))
    for rid in common.LABEL:
        key = f"{rid}__baseline"
        if f"{key}__x_mm" not in data:
            continue
        ax.plot(data[f"{key}__x_mm"], -data[f"{key}__h_mm"], color=COLOR[rid], ls=DASH.get(rid, "-"), lw=1.2,
                label=common.LABEL[rid])
    ax.axhline(-3, color="#aaaaaa", lw=0.6, ls=":")
    ax.set_xlabel("x along the extended finger (mm)")
    ax.set_ylabel("deepest hardware below\nthe link line (mm)")
    ax.legend(fontsize=6.5, ncol=2, frameon=False, loc="lower left")
    ax.set_title("E01 — hardware intrusion into the palmar (contact) side, extended", loc="left", fontsize=8.5)
    save(fig, "e01_palmar_intrusion")


def e02():
    rows = read("e02_polycentric_ablation/results.csv")
    raise_rows = read("e02b_palmar_pivot/results.csv")
    path = read("e02b_palmar_pivot/code_icr_path.csv")
    fig, axes = plt.subplots(1, 3, figsize=(9.2, 2.7))
    ax = axes[0]
    ts = sorted({num(r["thickness_mm"]) for r in rows})
    code = [num(next(r for r in rows if num(r["thickness_mm"]) == t and r["design"] == "code")["phi_max_deg"]) for t in ts]
    at0 = [num(next(r for r in rows if num(r["thickness_mm"]) == t and r["design"] == "ideal_2r_code_depth"
                    and num(r["pin_offset_mm"]) == 0.0)["phi_max_deg"]) for t in ts]
    best_feasible = []
    for t in ts:
        ok = [num(r["pin_offset_mm"]) for r in rows if num(r["thickness_mm"]) == t and r["design"] == "ideal_2r_code_depth"
              and r["feasible_150"] == "True"]
        best_feasible.append(1 if ok else 0)
    ax.plot(ts, code, "o-", color=COLOR["code_matched"], label="CoDe")
    ax.plot(ts, at0, "s--", color="#666666", label="ideal 2R, pin at CoDe ICR0")
    ax.axhline(150, color="#bbbbbb", lw=0.6, ls=":")
    ax.set_xlabel("pad thickness, both sides (mm)")
    ax.set_ylabel("φ_max (deg)")
    ax.legend(frameon=False, fontsize=6.5)
    ax.set_title("(a) fold limit", loc="left", fontsize=8)
    ax = axes[1]
    for t, marker in zip((5.0, 8.0, 10.0, 12.0), "osD^"):
        sel = [r for r in raise_rows if r["design"] == "ideal_2r_at_icr0" and num(r["thickness_mm"]) == t]
        h = [num(r["raise_mm"]) for r in sel]
        ok = [1.0 if r["feasible_150"] == "True" else 0.0 for r in sel]
        ax.step(h, np.array(ok) + 0.04 * (t - 8), where="mid", label=f"{t:g} mm", marker=marker, ms=3)
    ax.set_yticks([0, 1], ["fails", "feasible"])
    ax.set_xlabel("ideal 2R pivot moved toward the faces (mm)")
    ax.set_title("(b) 150° feasibility, palmar-pivot counterfactual", loc="left", fontsize=8)
    ax.legend(frameon=False, fontsize=6.5, title="thickness", title_fontsize=6.5)
    ax = axes[2]
    x = [num(r["icr_x_mm"]) for r in path]
    z = [num(r["icr_z_mm"]) for r in path]
    sc = ax.scatter(x, z, c=[num(r["phi_deg"]) for r in path], cmap="viridis", s=10)
    ax.axhline(0, color="#999999", lw=0.6)
    ax.axhline(-8, color="#c9a227", lw=0.8, ls="--")
    ax.text(min(x), -8.6, "8 mm pad face", fontsize=6, color="#8a6d00")
    ax.set_xlabel("ICR x (mm)")
    ax.set_ylabel("ICR z (mm)")
    fig.colorbar(sc, ax=ax, label="flexion (deg)", fraction=0.05)
    ax.set_title("(c) CoDe virtual pivot path", loc="left", fontsize=8)
    fig.tight_layout()
    save(fig, "e02_polycentric_ablation")


def e04():
    rows = read("e04_actuator_orientation/summary.csv")
    fig, ax = plt.subplots(figsize=(5.2, 2.6))
    for rid, label in (("code_matched", "CoDe"), ("belt_2r_matched", "Belt 2R"), ("2r_dd_tilt0_matched", "2R-DD"),
                       ("rcj_tilt0_matched", "RCJ")):
        sel = [r for r in rows if r["reference_id"] == rid]
        ax.plot([num(r["tilt_deg"]) for r in sel], [num(r["carried"]) for r in sel], "o-", ms=3, color=COLOR[rid],
                label=label)
    ax.set_xlabel("servo tilt (deg)")
    ax.set_ylabel("catalog pairs carried at 120°")
    ax.legend(frameon=False, fontsize=7)
    ax.set_title("E04 — accommodation against actuator orientation (morphology frozen)", loc="left", fontsize=8.5)
    save(fig, "e04_actuator_orientation")


def e06():
    rows = read("e06_clearance/summary.csv")
    fig, axes = plt.subplots(1, 2, figsize=(8, 2.6), sharey=True)
    for ax, variant in zip(axes, ("all", "pins_exempt")):
        for rid in common.LABEL:
            sel = [r for r in rows if r["reference_id"] == rid and r["variant"] == variant and r["flexion_deg"] == "120.0"]
            ax.plot([num(r["clearance_mm"]) for r in sel], [num(r["carried"]) for r in sel], "o-", ms=3,
                    color=COLOR[rid], ls=DASH.get(rid, "-"), label=common.LABEL[rid])
        ax.set_xlabel("body clearance (mm)")
        ax.set_title(f"({'a' if variant == 'all' else 'b'}) clearance on {'every check' if variant == 'all' else 'all but pin housings'}",
                     loc="left", fontsize=8)
    axes[0].set_ylabel("pairs carried at 120°")
    axes[1].legend(frameon=False, fontsize=6, loc="center right")
    save(fig, "e06_clearance")


def e07():
    rows = read("e07_occlusion/results.csv")
    fig, ax = plt.subplots(figsize=(5.2, 2.6))
    for rid in common.LABEL:
        sel = [r for r in rows if r["reference_id"] == rid and r["module"] == "combined_L30_T3" and r["closes"] == "True"]
        if not sel:
            continue
        ax.plot([num(r["phi_deg"]) for r in sel], [100 * num(r["distal_d10_hardware"]) for r in sel], color=COLOR[rid],
                ls=DASH.get(rid, "-"), lw=1.2, label=common.LABEL[rid])
    ax.set_xlabel("distal flexion (deg)")
    ax.set_ylabel("distal face occluded\nby hardware (%)")
    ax.legend(frameon=False, fontsize=6)
    ax.set_title("E07 — hardware within 10 mm in front of the distal face (30 mm modules)", loc="left", fontsize=8.5)
    save(fig, "e07_occlusion")


def e10():
    rows = read("e10_design_drivers/results.csv")
    fig, axes = plt.subplots(1, 2, figsize=(7.6, 2.7), sharey=True)
    y = np.array([num(r["carried"]) for r in rows])
    span = np.array([num(r["e_proximal_span_mm"]) for r in rows])
    for ax, key, label in ((axes[0], "icr_drop_mm", "ICR palmar drop, 0→120° (mm)"),
                           (axes[1], "icr_travel_mm", "ICR travel, 0→120° (mm)")):
        sc = ax.scatter([num(r[key]) for r in rows], y, c=span, cmap="cividis", s=10)
        ax.set_xlabel(label)
    axes[0].set_ylabel("catalog pairs carried at 120°")
    fig.colorbar(sc, ax=axes, label="proximal span |OA''| (mm)", fraction=0.04)
    fig.suptitle("E10 — 150 archived CoDe designs (farthest-point cover of 22,678)", x=0.06, ha="left", fontsize=8.5)
    save(fig, "e10_design_drivers")


def e05_e09():
    tol = read("e05_tolerance/results.csv")
    hid = read("e09_hidden_transmission/results.csv")
    fig, axes = plt.subplots(1, 2, figsize=(8.4, 2.7))
    ax = axes[0]
    refs = [r for r in common.LABEL]
    xs = np.arange(len(refs))
    for k, (module, off) in enumerate((("baseline", -0.15), ("thick", 0.15))):
        vals = []
        for rid in refs:
            r = next((r for r in tol if r["reference_id"] == rid and r["module"] == module and r["sigma_mm"] == "0.2"), None)
            vals.append(num(r["p_feasible"]) if r and r.get("nominal_feasible") == "True" else np.nan)
        ax.bar(xs + off, vals, width=0.3, color=[COLOR[r] for r in refs], alpha=1.0 if k == 0 else 0.55,
               label=f"{module} module")
    ax.set_xticks(xs, [common.LABEL[r].replace(" (upper bound)", "*") for r in refs], rotation=40, ha="right", fontsize=6)
    ax.set_ylabel("P(feasible), σ = 0.2 mm")
    ax.legend(frameon=False, fontsize=6)
    ax.set_title("(a) E05 tolerance (200 draws each)", loc="left", fontsize=8)
    ax = axes[1]
    for r in hid:
        if r.get("reference_module") != r.get("target") or r.get("distal_lever_change") in (None, ""):
            continue
        ax.scatter(num(r["eps_E"]), num(r["distal_lever_change"]), color=COLOR[r["reference_id"]], s=14,
                   marker="D" if r["upper_bound"] == "True" else "o")
    ax.set_xlabel("ε_E of the Study 2 adaptation")
    ax.set_ylabel("max relative change of distal\nservo torque per contact force")
    ax.set_title("(b) E09 hidden transmission change", loc="left", fontsize=8)
    fig.tight_layout()
    save(fig, "e05_e09")


def main():
    for f in (e01, e02, e04, e06, e07, e10, e05_e09):
        f()
    print(f"wrote {common.FIGURES}")


if __name__ == "__main__":
    main()
