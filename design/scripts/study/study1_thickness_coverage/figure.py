#!/usr/bin/env python3
"""Study 1 (problem formulation A.6.2): the best pad coverage of each
architecture against the pad thickness, per required flexion.

    python scripts/study/study1_thickness_coverage/figure.py [--tag name]

Reads each condition's portfolio (``output/study/<mechanism>/<file>_q<q>[_h<h>]``;
thickness 5 mm is the native design's, no ``_h``), takes its best design, and
writes ``output/study/study1_thickness_coverage/summary.csv`` and
``report/study1/figures/study1_thickness_coverage_<tag>.png``: the objective
(cylinder-based C_pad, top) and the pads' share of the finger's length (bottom).
A condition not run yet is left out.

With Study 2's output there (``output/study/study2_one_finger_thickness/curves.csv``,
A.6.3), also ``report/study2/figures/study2_one_finger_<tag>.png``: per
architecture, Study 1's curve (a finger for each thickness) and the one
finger's, the gap between them shaded.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from visualize.style import COLORS, GRID, INK, MARKERS, use  # noqa: E402

OUTPUT = REPOSITORY_ROOT / "output" / "study"
FIGURES = REPOSITORY_ROOT.parent / "report" / "study1" / "figures"
ARCH = [("cfl", "linkage", "CFL"), ("2r_remote", "revolute_parallel", "2R-remote"), ("2r_dd", "revolute", "2R-DD"),
        ("rcj", "rcj", "RCJ"), ("cfl_ideal", "linkage_idealized", "CFL ideal*"),
        ("rcj_ideal", "rcj_idealized", "RCJ ideal*")]
THICKNESS, FLEXION, NATIVE = (3, 5, 7, 9), (90, 120, 150), 5


def _folder_best(folder: Path) -> dict | None:
    """The best design of a folder's latest portfolio, with its runs' spread."""

    portfolios = sorted(folder.glob("portfolio_*/seeds.csv"))
    if not portfolios:
        return None
    with portfolios[-1].open(encoding="utf-8", newline="") as stream:
        rows = [r for r in csv.DictReader(stream) if r.get("c_pad_mean") not in (None, "")]
    if not rows:
        return None
    top = dict(max(rows, key=lambda r: float(r["c_pad_mean"])))
    # The spread of the search: the best coverage of each run (optimizer and seed) on its own.
    runs = []
    for path in sorted(folder.glob("run_*/pareto.csv")):
        with path.open(encoding="utf-8", newline="") as stream:
            values = [float(r["c_pad_mean"]) for r in csv.DictReader(stream) if r.get("c_pad_mean") not in (None, "")]
        if values:
            runs.append(max(values))
    top["runs"], top["run_low"] = len(runs), min(runs, default=float(top["c_pad_mean"]))
    return top


def best(file: str, mechanism: str, q: int, h: int) -> dict | None:
    """The best design of one condition: of its independent runs or, when it
    was searched again from its neighbours' designs (``run_continuation.sh``,
    ``<folder>_cont``), of those if better (``None``: not run, or no design).
    ``independent``: the independent runs' own best."""

    folder = OUTPUT / mechanism / (f"{file}_q{q}" if h == NATIVE else f"{file}_q{q}_h{h}")
    own, continued = _folder_best(folder), _folder_best(folder.with_name(folder.name + "_cont"))
    top = max((row for row in (own, continued) if row is not None), key=lambda r: float(r["c_pad_mean"]), default=None)
    if top is None:
        return None
    top["source"] = "continuation" if top is continued and continued is not own else "independent"
    top["independent"] = "" if own is None else float(own["c_pad_mean"])
    return top


def collect() -> list[dict]:
    found = []
    for file, mechanism, name in ARCH:
        for q in FLEXION:
            for h in THICKNESS:
                row = best(file, mechanism, q, h)
                if row is None:
                    continue
                length = float(row["finger_length_mm"])
                pads = float(row["proximal_support_length_mm"]) + float(row["distal_support_length_mm"])
                found.append({"architecture": name, "mechanism": mechanism, "required_flexion_deg": q,
                              "thickness_mm": h, "c_pad_mean": float(row["c_pad_mean"]),
                              "c_pad_run_low": float(row["run_low"]), "runs": row["runs"],
                              "source": row["source"], "c_pad_independent": row["independent"],
                              "stop_reason": row.get("stop_reason", ""), "collision_pair": row.get("collision_pair", ""),
                              "pad_coverage": pads / length,
                              "proximal_pad_mm": float(row["proximal_support_length_mm"]),
                              "distal_pad_mm": float(row["distal_support_length_mm"]), "finger_length_mm": length,
                              "q_max_deg": row.get("q_max_deg", ""), "ma_wc": row.get("ma_wc", "")})
    return found


def figure(rows: list[dict], tag: str) -> Path:
    panels = (("c_pad_mean", "best $\\bar C_{pad}$ (cylinder, objective)"),
              ("pad_coverage", "pad coverage $(\\ell_p+\\ell_d)/L_f$"))
    fig, axes = plt.subplots(len(panels), len(FLEXION), figsize=(2.9 * len(FLEXION), 2.3 * len(panels)),
                             squeeze=False, sharex=True, sharey="row")
    for a, (key, label) in enumerate(panels):
        for b, q in enumerate(FLEXION):
            axis = axes[a][b]
            for _, mechanism, name in ARCH:
                own = sorted((r for r in rows if r["mechanism"] == mechanism and r["required_flexion_deg"] == q),
                             key=lambda r: r["thickness_mm"])
                if not own:
                    continue
                ideal = name.endswith("*")
                if key == "c_pad_mean":  # the runs' own bests: from the lowest of them up to the best
                    axis.vlines([r["thickness_mm"] for r in own], [100 * r["c_pad_run_low"] for r in own],
                                [100 * r["c_pad_mean"] for r in own], color=COLORS[mechanism], lw=3.0, alpha=0.3)
                axis.plot([r["thickness_mm"] for r in own], [100 * r[key] for r in own], color=COLORS[mechanism],
                          marker=MARKERS.get(mechanism, "o"), ms=4, lw=1.0 if ideal else 1.8,
                          ls=(0, (3, 2)) if ideal else "-", label=name)
            axis.set_xticks(THICKNESS)
            axis.set_ylim(0, 100)
            axis.grid(color=GRID, lw=0.6)
            for side in ("top", "right"):
                axis.spines[side].set_visible(False)
            if a == 0:
                axis.set_title(f"$q_{{req}}$ = {q}°", fontsize=8)
            if a == len(panels) - 1:
                axis.set_xlabel("pad thickness (mm)", fontsize=7.5)
            if b == 0:
                axis.set_ylabel(f"{label} (%)", fontsize=7)
    axes[0][-1].legend(fontsize=6.5, frameon=False, loc="lower left")
    fig.suptitle("The finger optimized for each pad thickness and required flexion ($L_f$ 90 mm)\n"
                 "Bars: the range of the runs' own bests (2 optimizers x 3 seeds). "
                 "*Upper bounds without the joint servo.", fontsize=8.5, color=INK, x=0.02, ha="left", y=1.0,
                 va="bottom")
    fig.tight_layout()
    FIGURES.mkdir(parents=True, exist_ok=True)
    path = FIGURES / f"study1_thickness_coverage_{tag}.png"
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)
    return path


def one_finger_figure(tag: str) -> Path | None:
    source = OUTPUT / "study2_one_finger_thickness" / "curves.csv"
    if not source.exists():
        return None
    with source.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    present = [a for a in ARCH if any(r["architecture"] == a[0] for r in rows)]
    flexions = sorted({float(r["required_flexion_deg"]) for r in rows})
    if not present:
        return None
    fig, axes = plt.subplots(len(flexions), len(present), figsize=(1.9 * len(present) + 0.6, 1.7 * len(flexions) + 0.5),
                             squeeze=False, sharex=True, sharey=True)
    for a, q in enumerate(flexions):
        for b, (key, mechanism, name) in enumerate(present):
            axis = axes[a][b]
            own = sorted((r for r in rows if r["architecture"] == key and float(r["required_flexion_deg"]) == q),
                         key=lambda r: float(r["thickness_mm"]))
            title = name if a == 0 else ""
            if own:
                h = [float(r["thickness_mm"]) for r in own]
                top = [100 * float(r["c_own"]) for r in own]
                color = COLORS[mechanism]
                axis.plot(h, top, color=color, lw=1.0, ls=(0, (3, 2)), marker="o", ms=3, mfc="white")
                if own[0]["one_finger"] == "True":
                    one = [100 * float(r["c_one"]) for r in own]
                    axis.fill_between(h, one, top, color=color, alpha=0.18, lw=0)
                    axis.plot(h, one, color=color, lw=1.8, marker="o", ms=3)
                    title += ("\n" if title else "") + f"gap up to {max(t - o for t, o in zip(top, one)):.1f} pt"
                else:
                    title += ("\n" if title else "") + "no one finger"
            axis.set_title(title, fontsize=6.5, color=INK)
            axis.set_xticks(THICKNESS)
            axis.set_ylim(0, 100)
            axis.grid(color=GRID, lw=0.6)
            for side in ("top", "right"):
                axis.spines[side].set_visible(False)
            if b == 0:
                axis.set_ylabel(f"$q_{{req}}$ {q:g}°\n$\\bar C_{{pad}}$ (%)", fontsize=6.5)
            if a == len(flexions) - 1:
                axis.set_xlabel("pad thickness (mm)", fontsize=6.5)
    fig.suptitle("One finger with pads of every thickness (solid) against a finger optimized for each thickness "
                 "(dashed, Study 1)\nShaded: what not redesigning the finger costs. *Upper bounds without the joint servo.",
                 fontsize=7.5, color=INK, x=0.02, ha="left", y=1.0, va="bottom")
    fig.tight_layout()
    folder = REPOSITORY_ROOT.parent / "report" / "study2" / "figures"
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"study2_one_finger_{tag}.png"
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)
    return path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tag", default="native")
    args = parser.parse_args(argv)
    rows = collect()
    out = OUTPUT / "study1_thickness_coverage"
    out.mkdir(parents=True, exist_ok=True)
    if rows:
        with (out / "summary.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    use("paper")
    print(figure(rows, args.tag), f"({len(rows)} of {len(ARCH) * len(FLEXION) * len(THICKNESS)} conditions)")
    second = one_finger_figure(args.tag)
    if second:
        print(second)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
