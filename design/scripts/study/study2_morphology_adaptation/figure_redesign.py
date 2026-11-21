#!/usr/bin/env python3
"""Study 2's figure (problem formulation A.6.3), from ``redesign.py``'s output.

    python scripts/study/study2_morphology_adaptation/figure_redesign.py [output dir] [--tag name]

``study2_redesign_<tag>.png``, one row per required flexion:

- left: how far the relative contact trajectory moves when the finger is redesigned
  from 3 mm pads for thicker ones, at the most pad coverage the redesign keeps
  (the front's coverage end), per architecture; the line is what a joint
  turning about a fixed axis must show, ``2 sin(q_req/2) dh``;
- middle: that coverage, against Study 1's best for the thickness (hollow);
- right: for the thickest pads, where over the flexion the redesign's contact
  path parts from the reference's (the two agree at the extended pose; what
  differs is how the contact faces move as the finger bends).

PNG to ``report/study2/figures``.
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

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from visualize.style import COLORS, GRID, INK, MARKERS, use  # noqa: E402

FIGURES = REPOSITORY_ROOT.parent / "report" / "study2" / "figures"
NAMES = {"cfl": "CFL", "2r_remote": "2R-remote", "2r_dd": "2R-DD", "rcj": "RCJ", "cfl_ideal": "CFL ideal*",
         "rcj_ideal": "RCJ ideal*"}
LENGTH_MM = 90.0


def read(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def figure(out: Path, tag: str) -> Path | None:
    rows = [r for r in read(out / "summary.csv") if r.get("most_c_pad") not in (None, "")]
    if not rows:
        return None
    flexions = sorted({float(r["required_flexion_deg"]) for r in rows})
    start = float(rows[0]["from_mm"])
    profiles = read(out / "profiles.csv")
    fig, axes = plt.subplots(len(flexions), 3, figsize=(12.0, 3.0 * len(flexions)), squeeze=False)
    for a, q in enumerate(flexions):
        change, kept, along = axes[a]
        thick = sorted({float(r["to_mm"]) for r in rows if float(r["required_flexion_deg"]) == q})
        bound = [2 * math.sin(math.radians(q) / 2) * (h - start) for h in [start, *thick]]
        for name, label in NAMES.items():
            own = sorted((r for r in rows if r["architecture"] == name and float(r["required_flexion_deg"]) == q),
                         key=lambda r: float(r["to_mm"]))
            if not own:
                continue
            mech, ideal = own[0]["mechanism"], label.endswith("*")
            style = dict(color=COLORS[mech], marker=MARKERS.get(mech, "o"), ms=4, lw=1.0 if ideal else 1.8,
                         ls=(0, (3, 2)) if ideal else "-")
            h = [float(r["to_mm"]) for r in own]
            change.plot([start, *h], [0.0, *(LENGTH_MM * float(r["change_at_most_c_pad"]) for r in own)], label=label,
                        **style)
            kept.plot(h, [100 * float(r["most_c_pad"]) for r in own], label=label, **style)
            kept.plot(h, [100 * float(r["study1_c_pad"]) for r in own], color=COLORS[mech], ls="",
                      marker=MARKERS.get(mech, "o"), ms=6, mfc="none", mew=0.9)
            deep = [p for p in profiles if p["architecture"] == name and float(p["required_flexion_deg"]) == q
                    and float(p["to_mm"]) == thick[-1]]
            if deep:
                along.plot([float(p["q_deg"]) for p in deep], [float(p["contact_apart_mm"]) for p in deep],
                           color=COLORS[mech], lw=1.0 if ideal else 1.8, ls=(0, (3, 2)) if ideal else "-")
        # What a fixed axis must show, drawn last so the architecture on it does not hide it.
        change.plot([start, *thick], bound, color=INK, lw=1.1, ls=(0, (1, 2)), zorder=6)
        change.text(thick[-1], bound[-1], "  fixed axis:\n  $2\\sin(q_{req}/2)\\,\\Delta h$", fontsize=6.5, color=INK,
                    va="center")
        angles = [q * k / 60 for k in range(61)]
        along.plot(angles, [2 * math.sin(math.radians(v) / 2) * (thick[-1] - start) for v in angles], color=INK,
                   lw=1.1, ls=(0, (1, 2)), zorder=6)
        along.set_ylabel("contact paths apart (mm)", fontsize=7.5)
        along.set_xlim(0, q)
        change.set_ylabel(f"$q_{{req}}$ {q:g}°\nrelative contact\ntrajectory moved (mm)", fontsize=7.5)
        kept.set_ylabel("pad coverage $\\bar C_{pad}$ kept (%)", fontsize=7.5)
        kept.set_ylim(40, 90)
        change.set_xlim(start - 0.3, thick[-1] + 2.6)
        for axis in (change, kept, along):
            if axis is not along:
                axis.set_xticks([start, *thick] if axis is change else thick)
            axis.grid(color=GRID, lw=0.6)
            for side in ("top", "right"):
                axis.spines[side].set_visible(False)
            if a == len(flexions) - 1:
                axis.set_xlabel("flexion q (deg)" if axis is along else "pad thickness redesigned for (mm)", fontsize=7.5)
        if a == 0:
            change.set_title("(a) relative contact trajectory, from the 3 mm finger", fontsize=8, loc="left")
            kept.set_title("(b) the coverage that redesign keeps (hollow: Study 1's best)", fontsize=8, loc="left")
            kept.legend(fontsize=6.5, frameon=False, ncol=2, loc="lower left")
            along.set_title(f"(c) over the flexion, redesigned for {thick[-1]:g} mm", fontsize=8, loc="left")
    fig.suptitle("Redesigned for thicker pads with the relative contact trajectory as a target\n"
                 "*Upper bounds without the joint servo.", fontsize=8.5, color=INK, x=0.02, ha="left", y=1.0,
                 va="bottom")
    fig.tight_layout()
    FIGURES.mkdir(parents=True, exist_ok=True)
    path = FIGURES / f"study2_redesign_{tag}.png"
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)
    return path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("output", nargs="?", default="output/study/study2_redesign")
    parser.add_argument("--tag", default="native")
    args = parser.parse_args(argv)
    out = Path(args.output)
    if not out.is_absolute():
        out = REPOSITORY_ROOT / out
    use("paper")
    print(figure(out, args.tag))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
