# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3 figure: interface-measurement error against the relative-contact-trajectory change, every
redesign at 3 -> 5, 7 and 9 mm, position reuse (condition A) and joint-torque reuse (condition C).

    python3 scripts/study3_measurement/figure_dose.py

Reads ``logs/study3/two_pad/dose_A_all.json`` and ``dose_C_all.json`` (``dose_analysis.py --set all``). Hands
that could not hold the cylinder under the command are drawn at the top as crosses. Writes
``report/study3/figures/study3_dose_response.png``.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dose_analysis as da  # noqa: E402

OUT = da.ROOT.parents[4] / "report" / "study3" / "figures" / "study3_dose_response.png"
COLORS = {5.0: "#1a7f37", 7.0: "#0969da", 9.0: "#8250df"}  # thickness


def main() -> int:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    hands = {**da.HANDS, **da.EXTENSION, **da.THICK}
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.3), sharex=True)
    for ax, cond, title in ((axes[0], "A", "Position reuse (condition A)"), (axes[1], "C", "Joint-torque reuse (condition C)")):
        d = json.loads((da.ROOT / f"dose_{cond}_all.json").read_text())
        top = 175.0
        for hand, (dc, ds, cp) in hands.items():
            t = da.thickness(hand)
            c = COLORS[t]
            name = "R" if hand in ("R",) or hand.endswith("_hr") else "U" if hand in ("U",) or hand.endswith("_hu") else None
            rec = d["hands"].get(hand)
            if rec is None:
                ax.plot(dc, top, marker="x", color=c, ms=8, mew=2, zorder=3)
                continue
            e = 100 * rec["E_rel_median"]
            ax.plot(dc, e, marker="o" if name is None else "s", color=c, ms=7 if name else 6,
                    mfc=c if name else "white", mew=1.6, zorder=3)
            if name:
                ax.annotate(f"{name}{int(t)}", (dc, e), xytext=(5, 4), textcoords="offset points", fontsize=7.5, color=c)
        tests = d.get("tests", {})
        ax.text(0.03, 0.85, f"ρ(error, D_c) = {tests.get('rho_E_Dc', float('nan')):+.2f}, p = "
                f"{tests.get('p_one_sided', float('nan')):.4f} (n {tests.get('n_hands', 0)})\n"
                f"ρ(error, D_s) = {tests.get('rho_E_Ds', float('nan')):+.2f}", transform=ax.transAxes, fontsize=8,
                va="top", ha="left", color="#1f2328")
        ax.axhline(top - 8, color="#d0d7de", lw=0.8, ls=":")
        ax.text(0.2, top, "no grasp →", fontsize=7.5, color="#57606a", va="center", ha="left")
        ax.set_ylim(-5, top + 10)
        ax.set_title(title, fontsize=9.5, loc="left", color="#1f2328")
        ax.set_xlabel("change in the relative contact trajectory D_c (mm)", fontsize=8.5, color="#57606a")
        ax.grid(color="#d0d7de", lw=0.5)
        ax.tick_params(labelsize=8, colors="#57606a")
        for s in ax.spines.values():
            s.set_color("#d0d7de")
    axes[0].set_ylabel("error in the measured friction effect vs O (%, median)", fontsize=8.5, color="#57606a")
    handles = [plt.Line2D([], [], marker="o", ls="", color=c, mfc=c, label=f"3 → {int(t)} mm pads") for t, c in COLORS.items()]
    handles += [plt.Line2D([], [], marker="s", ls="", color="#57606a", mfc="#57606a", label="R / U of each thickness"),
                plt.Line2D([], [], marker="o", ls="", color="#57606a", mfc="white", label="other redesigns")]
    fig.legend(handles=handles, loc="lower center", ncol=5, fontsize=8, frameon=False)
    fig.suptitle("The error in a measured interface effect follows the contact-trajectory change, not the skeleton's "
                 "(CFL, 120°, free cylinder, μ 1.3 → 2.0)", fontsize=9.5, color="#1f2328", x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0.07, 1, 0.94))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=200)
    print(OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
