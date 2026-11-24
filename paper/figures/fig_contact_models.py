"""Figure: does C2.3 survive a second contact model and other tasks? (PNG only, single column, print size.)

    conda run -n cmd python paper/figures/fig_contact_models.py [dest_dir]

(a) E against Delta_contact over the redesigned hands under the reused impedance setpoint (condition A, pad friction
1.3 -> 2.0): Isaac Lab point contact (the paper's Study 3), the Drake replica with point contact, and Drake with
hydroelastic pads (E = 0.1 MPa over 3 mm). (b) The same measure in two open-loop tasks beyond holding (Drake, point):
in-hand rolling by a flexion sweep, grasp acquisition under a load and turning a sprung dial. Each panel states Spearman rho(E, D_contact)
and its one-sided p. Inputs: sim/drake/logs/e21_point.json, e21_hydro.json (analyze.py --out), e22_point.json,
e23_point.json, e24_point.json (analyze_roll.py --out); experiments 21-24 in report/study3/experiments.md.
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
LOGS = ROOT / "sim" / "drake" / "logs"
sys.path.insert(0, str(ROOT / "sim" / "drake"))
DEST = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / "ContactCentricDesign-RSS2027" / "figures"
INK, INK2 = "#0b0b0b", "#52514e"
SERIES = {  # key: (label, colour, marker, filled)
    "isaac": ("Isaac Lab, point", "#52514e", "o", True),
    "point": ("Drake, point", "#2a78d6", "^", False),
    "hydro": ("Drake, hydroelastic", "#eb6834", "D", True),
    "grasp": ("hold (Drake, point)", "#2a78d6", "^", False),
    "roll": ("in-hand rolling", "#1baf7a", "s", True),
    "pullin": ("grasp acquisition", "#9b5de5", "v", True),
    "dial": ("dial turning", "#eda100", "P", True),
}


def spearman_p(x, y):
    from analyze import spearman_perm
    return spearman_perm(x, y)


def load(name):
    p = LOGS / name
    return json.loads(p.read_text()) if p.exists() else []


def scatter(ax, rows, key, ekey="E"):
    label, colour, marker, filled = SERIES[key]
    rows = [r for r in rows if r.get(ekey) is not None]
    if not rows:
        return
    x = np.array([r["d_contact"] for r in rows])
    y = 100 * np.array([r[ekey] for r in rows])
    rho, p = spearman_p(x, y)
    ax.plot(x, y, ls="none", marker=marker, ms=3.4, mec=colour, mfc=colour if filled else "white", mew=0.8,
            label=f"{label}: $\\rho$ {rho:.2f}, p {p:.3f}" if p >= 0.001 else f"{label}: $\\rho$ {rho:.2f}, p<0.001",
            zorder=3)


def main() -> int:
    plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
                         "mathtext.fontset": "stix", "font.size": 7})
    fig, axes = plt.subplots(1, 2, figsize=(3.45, 2.0), gridspec_kw=dict(wspace=0.3))
    point = load("e21_point.json")
    isaac = [{"d_contact": r["d_contact"], "E": r.get("E_isaac")} for r in point]
    scatter(axes[0], isaac, "isaac")
    scatter(axes[0], point, "point")
    scatter(axes[0], load("e21_hydro.json"), "hydro")
    scatter(axes[1], point, "grasp")
    scatter(axes[1], load("e22_point.json"), "roll")
    scatter(axes[1], load("e23_point.json"), "pullin")
    scatter(axes[1], load("e24_point.json"), "dial")
    for ax, title in zip(axes, ("(a) contact model", "(b) task")):
        ax.text(0.03, 0.98, title, transform=ax.transAxes, fontsize=7, fontweight="bold", va="top", ha="left")
        ax.set_xlabel(r"$\Delta_{\mathrm{contact}}$ (mm)", fontsize=7, labelpad=1)
        ax.set_yscale("log")
        ax.set_ylim(2, 800)
        ax.set_xlim(0, 5.5)
        ax.tick_params(labelsize=6, length=2)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.legend(fontsize=5.2, frameon=False, loc="lower left", bbox_to_anchor=(-0.05, 1.01), handletextpad=0.2,
                  borderaxespad=0.0, labelspacing=0.15, handlelength=1.0)
    axes[0].set_ylabel("$E$ (%)", fontsize=7, labelpad=1)
    fig.subplots_adjust(left=0.12, right=0.99, bottom=0.17, top=0.7)
    DEST.mkdir(parents=True, exist_ok=True)
    out = DEST / "fig_contact_models.png"
    fig.savefig(out, dpi=400)
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
