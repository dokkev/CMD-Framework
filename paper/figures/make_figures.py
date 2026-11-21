"""Paper figures for the RSS 2027 extended abstract (PNG only).

Reads the study outputs under design/output/study/ and writes PNGs into the manuscript repo's figures/.
Usage: python make_figures.py [dest_dir]   (run in the `cmd` conda env)
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Circle

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "design" / "output" / "study"
DEST = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / "ContactCentricDesign-RSS2027" / "figures"
DEST.mkdir(parents=True, exist_ok=True)

INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
# dataviz default categorical order, slots 1-4 (validated order); identity also carried by marker.
ARCH = {  # key: (label, color, marker)
    "CFL": ("CFL (CoDe)", "#2a78d6", "o"),
    "2R-remote": ("2R-remote", "#eb6834", "s"),
    "2R-DD": ("2R-DD", "#1baf7a", "^"),
    "RCJ": ("RCJ", "#eda100", "D"),
}
IDEAL = {"CFL ideal*": "CFL", "RCJ ideal*": "RCJ"}
S2KEY = {"cfl": "CFL", "2r_remote": "2R-remote", "2r_dd": "2R-DD", "rcj": "RCJ"}
LF = 90.0
W = 7.16  # IEEE text width, in

plt.rcParams.update({
    "font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
    "mathtext.fontset": "stix", "font.size": 8, "axes.labelsize": 8, "axes.titlesize": 8,
    "xtick.labelsize": 7, "ytick.labelsize": 7, "legend.fontsize": 7,
    "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
    "text.color": INK, "axes.linewidth": 0.6, "xtick.major.width": 0.6, "ytick.major.width": 0.6,
    "xtick.major.size": 2.5, "ytick.major.size": 2.5, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "axes.grid.axis": "y", "grid.color": GRID, "grid.linewidth": 0.5,
    "savefig.dpi": 400, "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
})


def rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def style_line(key, ideal=False):
    label, color, marker = ARCH[key]
    if ideal:
        return dict(color=color, ls=(0, (3, 2)), lw=1.0, marker=marker, ms=3.2, mfc="white", mec=color, mew=0.8, alpha=0.9)
    return dict(color=color, lw=1.6, marker=marker, ms=4.2, mec="white", mew=0.6)


# ---------------------------------------------------------------- Study 1
def fig_study1():
    data = rows(OUT / "study1_thickness_coverage" / "summary.csv")
    fig, axes = plt.subplots(1, 3, figsize=(W, 1.9), sharey=True)
    for ax, q in zip(axes, (90, 120, 150)):
        for name in list(ARCH) + list(IDEAL):
            key, ideal = (IDEAL[name], True) if name in IDEAL else (name, False)
            pts = sorted((float(r["thickness_mm"]), 100 * float(r["c_pad_mean"]))
                         for r in data if r["architecture"] == name and float(r["required_flexion_deg"]) == q)
            if not pts:
                continue
            h, c = map(np.array, zip(*pts))
            shown = c >= 48
            ax.plot(h[shown], c[shown], **style_line(key, ideal), zorder=3 if not ideal else 2)
            for hh in h[~shown]:  # off-scale degenerate designs: a marker pinned to the floor
                ax.plot(hh, 48.9, marker="v", color=ARCH[key][1], ms=4.2, mec="white", mew=0.5, clip_on=False, zorder=4)
            if name == "2R-DD" and q == 150:
                ax.text(5.55, 63.0, "2R-DD: none found\nbeyond 5 mm", fontsize=6.2, color=INK2, ha="left", va="center", linespacing=1.1)
        ax.set_title(rf"$q_{{\mathrm{{req}}}}={q}^\circ$", pad=3)
        ax.set_xticks([3, 5, 7, 9]); ax.set_xlim(2.5, 9.5); ax.set_ylim(48, 86)
        ax.set_xlabel("pad thickness $h$ (mm)")
    axes[0].set_ylabel(r"best $\bar C_{\mathrm{pad}}$ (%)")
    handles = [plt.Line2D([], [], **style_line(k), label=ARCH[k][0]) for k in ARCH]
    handles += [plt.Line2D([], [], **style_line(IDEAL[n], True), label=n.replace("*", " (bound)")) for n in IDEAL]
    fig.legend(handles=handles, loc="upper center", ncol=6, frameon=False, bbox_to_anchor=(0.5, 1.12),
               handlelength=2.2, columnspacing=1.4)
    fig.savefig(DEST / "fig_study1_coverage.png")
    plt.close(fig)


# ---------------------------------------------------------------- Study 2
def fig_study2():
    summ = rows(OUT / "study2_redesign_2026-11-21" / "summary.csv")
    prof = rows(OUT / "study2_redesign_2026-11-21" / "profiles.csv")
    fig, axes = plt.subplots(1, 4, figsize=(W, 1.85), gridspec_kw=dict(width_ratios=[1, 1, 1, 1.25], wspace=0.38))
    for ax, q in zip(axes[:3], (90.0, 120.0, 150.0)):
        hb = np.array([3, 5, 7, 9.0])
        bound = 2 * np.sin(np.radians(q) / 2) * (hb - 3)
        ax.plot(hb, bound, color=INK, ls=(0, (1, 1.2)), lw=1.1, zorder=2)
        ymax = bound[-1] * 1.18
        for k in ARCH:
            pts = sorted((float(r["to_mm"]), LF * float(r["change_at_most_c_pad"]))
                         for r in summ if S2KEY.get(r["architecture"]) == k and float(r["required_flexion_deg"]) == q
                         and r["change_at_most_c_pad"] not in ("", "nan"))
            if not pts:
                continue
            h, d = map(np.array, zip(*([(3.0, 0.0)] + pts)))
            ok = d <= ymax
            ax.plot(h[ok], d[ok], **style_line(k), zorder=3)
            if (~ok).any():
                ax.plot(h[ok][-1:].tolist() + [h[~ok][0]], d[ok][-1:].tolist() + [ymax * 1.05], color=ARCH[k][1], lw=1.6, clip_on=True)
            for hh, dd in zip(h[~ok], d[~ok]):
                ax.plot(hh, ymax, marker=ARCH[k][2], color=ARCH[k][1], ms=4.2, mec="white", mew=0.5, clip_on=False, zorder=4)
                ax.text(hh - 0.5, ymax * 0.97, f"{dd:.1f}", fontsize=6.3, color=INK2, ha="right", va="top")
        ax.set_ylim(0, ymax); ax.set_xlim(2.7, 9.3); ax.set_xticks([3, 5, 7, 9])
        ax.set_title(rf"$q_{{\mathrm{{req}}}}={int(q)}^\circ$", pad=3)
        ax.set_xlabel("redesigned for $h$ (mm)")
    axes[0].set_ylabel("trajectory change $D_cL_f$ (mm)")
    # (d) over the flexion, q_req 120, 3 -> 9 mm
    ax = axes[3]
    for k in ARCH:
        pts = sorted((float(r["q_deg"]), float(r["contact_apart_mm"])) for r in prof
                     if S2KEY.get(r["architecture"]) == k and float(r["required_flexion_deg"]) == 120.0 and float(r["to_mm"]) == 9.0)
        if pts:
            qq, dd = map(np.array, zip(*pts))
            ax.plot(qq, dd, color=ARCH[k][1], lw=1.6)
            ax.plot(qq[-1], dd[-1], marker=ARCH[k][2], color=ARCH[k][1], ms=4.2, mec="white", mew=0.6, ls="none")
    qq = np.linspace(0, 120, 100)
    ax.plot(qq, 2 * np.sin(np.radians(qq) / 2) * 6, color=INK, ls=(0, (1, 1.2)), lw=1.1)
    ax.set_xlim(0, 124); ax.set_xticks([0, 30, 60, 90, 120]); ax.set_ylim(0, 11)
    ax.set_xlabel(r"flexion $q$ ($^\circ$)"); ax.set_ylabel("trajectories apart (mm)")
    ax.set_title(r"$q_{\mathrm{req}}=120^\circ$, $3\to9$ mm", pad=3)
    for a, lab in zip(axes, "abcd"):
        a.text(-0.02, 1.07, f"({lab})", transform=a.transAxes, fontsize=8, fontweight="bold", ha="right")
    handles = [plt.Line2D([], [], **style_line(k), label=ARCH[k][0]) for k in ARCH]
    handles.append(plt.Line2D([], [], color=INK, ls=(0, (1, 1.2)), lw=1.1, label=r"fixed-axis bound $2\sin(q/2)\,\Delta h$"))
    fig.legend(handles=handles, loc="upper center", ncol=5, frameon=False, bbox_to_anchor=(0.5, 1.13),
               handlelength=2.2, columnspacing=1.4)
    fig.savefig(DEST / "fig_study2_redesign.png")
    plt.close(fig)


# ---------------------------------------------------------------- concept: what the joint takes from the pads
RED, BLUE = "#e34948", "#2a78d6"
PAD_R, PAD_B = "#f6c4c3", "#c4daf4"


def rot(q, pts, c):
    """Flexion by q (palmar) about centre c; x distal, z dorsal."""
    cq, sq = np.cos(q), np.sin(q)
    R = np.array([[cq, sq], [-sq, cq]])
    return (np.asarray(pts) - c) @ R.T + c


def panel(ax, t, z, title, half=40.7, q=np.radians(120), label_mirror=False):
    P = np.array([0.0, -z])
    tau = t - z
    s = tau * np.tan(q / 2)
    ell = half - s
    # proximal: link on z=0, pad hanging to depth t, joint-side end at x=-s
    ax.plot([-half - 2, 0], [0, 0], color=RED, lw=2.4, solid_capstyle="round", zorder=3)
    ax.add_patch(Polygon([[-s - ell, 0], [-s, 0], [-s, -t], [-s - ell, -t]], closed=True, fc=PAD_R, ec=RED, lw=0.8, zorder=2))
    # distal (reference pose mirrored about x=0), then flexed about the pole
    link = rot(q, [[0, 0], [half + 2, 0]], P)
    pad = rot(q, [[s, 0], [s + ell, 0], [s + ell, -t], [s, -t]], P)
    ax.plot(*link.T, color=BLUE, lw=2.4, solid_capstyle="round", zorder=3)
    ax.add_patch(Polygon(pad, closed=True, fc=PAD_B, ec=BLUE, lw=0.8, zorder=2))
    # mirror line through the pole, normal (cos q/2, -sin q/2)
    d = np.array([np.sin(q / 2), np.cos(q / 2)])
    m = np.array([P - 30 * d, P + 9 * d])
    ax.plot(*m.T, color=INK2, lw=0.6, ls=(0, (4, 2)), zorder=1)
    ax.add_patch(Circle(P, 1.3, fc="white", ec=INK, lw=1.0, zorder=5))
    ax.annotate("pole", xy=P + [1.2, 0.6], xytext=P + [7, 5.5], fontsize=7, color=INK,
                arrowprops=dict(arrowstyle="-", color=INK, lw=0.5))
    # gap annotation
    y = 4.0
    ax.annotate("", xy=(-s, y), xytext=(0, y), arrowprops=dict(arrowstyle="<->", color=INK, lw=0.7, shrinkA=0, shrinkB=0))
    ax.plot([-s, -s], [-t - 0.3, y + 1], color=INK2, lw=0.4, ls=(0, (1, 1)))
    ax.plot([0, 0], [-z, y + 1], color=INK2, lw=0.4, ls=(0, (1, 1)))
    ax.text(-s / 2, y + 1.3, f"{s:.1f} mm", ha="center", va="bottom", fontsize=7)
    ax.text(-s - ell / 2, -t / 2, f"pad {ell:.0f} mm", ha="center", va="center", fontsize=6.5, color=INK)
    if label_mirror:
        ax.text(*(P - 27 * d + [3.4, -4.6]), "mirror line", fontsize=6.3, color=INK2, rotation=np.degrees(np.arctan2(d[1], d[0])),
                ha="center", va="bottom")
    ax.set_title(title, fontsize=7.5, pad=2, loc="left")
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_xlim(-half - 3, 22); ax.set_ylim(-44, 11)


def fig_concept():
    fig, axes = plt.subplots(1, 3, figsize=(W, 1.85))
    panel(axes[0], 5, 0, "(a) fixed axis, face 5 mm deep", label_mirror=True)
    panel(axes[1], 9, 0, "(b) fixed axis, face 9 mm deep")
    panel(axes[2], 9, 5, "(c) pole 5 mm lower, face 9 mm deep")
    fig.subplots_adjust(wspace=0.05)
    fig.savefig(DEST / "fig_pole_concept.png")
    plt.close(fig)


if __name__ == "__main__":
    fig_concept()
    fig_study1()
    fig_study2()
    print("written to", DEST)
