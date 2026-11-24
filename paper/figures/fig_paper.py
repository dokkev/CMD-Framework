"""Figs. 2-4 and Table I of the RSS 2027 extended abstract, from the evaluator and the raw study data (PNG only).

    conda run -n cmd python paper/figures/fig_paper.py [fig2|fig3|fig4|table|all]

Figure plan (user, 2026-11-24): physical meaning first; no trend lines, Pareto fronts or statistics in the figures.
  Fig. 2  kinematic principle (Study 0): (a) clearance set by the pole, (b) achievable poles, (c) motion seen from
          the skeleton frame and from the contact frame. (a) and (c) are schematics from the closed-form model.
  Fig. 3  design validation (Studies 1, 2): (a) optimized geometry at 120 deg, 7 mm (evaluator) and coverage dots;
          (b) reference and redesign paths at 120 deg, 3 -> 7 mm, and contact-deviation dots with the fixed-axis value.
  Fig. 4  scientific validation (Study 3): four columns (hold with a friction change, hold with a stiffness change,
          grasp acquisition, dial turning): the task, R's and U's error, every redesign's error against its
          contact-surface change.
  Table I contact-model robustness (R and U errors), written as LaTeX to sections/table_robust.tex.
Colours: CFL / R blue, 2R-remote / U amber, 2R-DD / reference grey, RCJ teal, pole purple.
"""
import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Polygon, Circle, FancyArrowPatch, Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "paper" / "figures"), str(ROOT / "design"), str(ROOT / "design" / "scripts" / "study"),
                str(ROOT / "design" / "scripts" / "study" / "study2_morphology_adaptation")]
from pngout import save_rgb  # noqa: E402

MS = ROOT.parent / "ContactCentricDesign-RSS2027"
DEST = MS / "figures"
OUT = ROOT / "design" / "output" / "study"
S3 = ROOT / "report" / "study3" / "data"
W = 7.16
INK, INK2, LIGHT = "#111111", "#55534f", "#d9d9d6"
BLUE, AMBER, GREY, TEAL, PURPLE = "#2a6fc9", "#e0942b", "#7d7d7d", "#1b9e8a", "#8a3ffc"
ARCH = {"2r_dd": ("2R-DD", GREY, "^"), "2r_remote": ("2R-remote", AMBER, "s"), "rcj": ("RCJ", TEAL, "D"),
        "cfl": ("CFL", BLUE, "o")}
S1NAME = {"2r_dd": "2R-DD", "2r_remote": "2R-remote", "rcj": "RCJ", "cfl": "CFL"}
LF = 90.0

plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 7, "axes.linewidth": 0.6,
                     "xtick.major.width": 0.6, "ytick.major.width": 0.6, "xtick.major.size": 2,
                     "ytick.major.size": 2, "axes.labelsize": 7, "xtick.labelsize": 6.3, "ytick.labelsize": 6.3})


def rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def clean(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def label(fig, x, y, text, h):
    fig.text(x / W, y / h, text, fontsize=7.5, fontweight="bold", va="top", ha="left")


def rot(points, q, c):
    """Flexion by q (rad, palmar = clockwise) about c."""
    cq, sq = np.cos(q), np.sin(q)
    R = np.array([[cq, sq], [-sq, cq]])
    return (np.asarray(points, float) - c) @ R.T + c


# ------------------------------------------------------------------------------------------------------------ Fig. 2
def clearance_scene(ax, z, t=7.0, q=np.radians(90), L=30.0, title=""):
    """Two phalanges with pads of depth t; the pole z below the link line; the gap the pads need so that the distal
    pad, flexed by q about the pole, just meets the proximal one: G = 2 (t - z)_+ tan(q/2)."""
    s = max(t - z, 0.0) * np.tan(q / 2)
    P = np.array([0.0, -z])
    ax.plot([-L - 3, 0], [0, 0], color=INK2, lw=1.6, solid_capstyle="round")
    ax.add_patch(Polygon([[-L, 0], [-s, 0], [-s, -t], [-L, -t]], fc="#c9d9f2", ec=BLUE, lw=0.6))
    dist_link, dist_pad = [[0, 0], [L + 3, 0]], [[s, 0], [L, 0], [L, -t], [s, -t]]
    ax.plot(*np.array(dist_link).T, color=INK2, lw=1.6, alpha=0.25, solid_capstyle="round")
    ax.add_patch(Polygon(dist_pad, fc="#c9d9f2", ec=BLUE, lw=0.6, alpha=0.35))
    ax.plot(*rot(dist_link, q, P).T, color=INK2, lw=1.6, solid_capstyle="round")
    ax.add_patch(Polygon(rot(dist_pad, q, P), fc="#c9d9f2", ec=BLUE, lw=0.6))
    ax.plot(*P, marker="o", ms=4.2, color=PURPLE, mec="white", mew=0.4, zorder=6)
    if s > 0.2:
        for x in (-s, s):
            ax.plot([x, x], [-t, 3.0], color=INK, lw=0.4, ls=(0, (1, 1)))
        ax.annotate("", xy=(-s, 3.0), xytext=(s, 3.0),
                    arrowprops=dict(arrowstyle="<|-|>", lw=0.6, color=INK, mutation_scale=4, shrinkA=0, shrinkB=0))
        ax.text(s + 1.5, 3.0, f"gap {2 * s:.0f} mm", fontsize=6.2, ha="left", va="center")
    else:
        ax.text(1.5, 3.0, "no gap", fontsize=6.2, ha="left", va="center")
    ax.set_xlim(-L - 4, L * 0.9); ax.set_ylim(-L - 2, 7); ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(title, fontsize=6.6, pad=1)


def pole_shift_scene(ax, q=np.radians(120), L=28.0, t0=6.0, t1=10.0):
    """Closed-form geometry in the proximal contact frame (proximal contact face on y = 0): a reference finger whose
    contact faces lie t0 below its link line and a redesign for a thicker pad that keeps the links and the joint, so
    its faces lie t1 below. Relative to the contact face the joint has moved by dp = t1 - t0; the distal contact
    point (L, 0) at extension lands dc apart after flexing by q, |dc| = 2 sin(q/2) |dp|."""
    P0, P1 = np.array([0.0, t0]), np.array([0.0, t1])
    c0 = np.array([L, 0.0])
    c_ref, c_red = rot([c0], q, P0)[0], rot([c0], q, P1)[0]
    # proximal phalanx: the thick pad (redesign) and the reference link line
    ax.add_patch(Polygon([[-30, 0], [-3, 0], [-3, t1], [-30, t1]], fc="#f6dfbd", ec=AMBER, lw=0.5, zorder=1))
    ax.plot([-31, 0], [t1, t1], color=INK2, lw=1.5, solid_capstyle="round", zorder=2)
    ax.plot([-31, 0], [t0, t0], color=GREY, lw=1.0, ls=(0, (2.4, 1.4)), zorder=2)
    ax.plot([-31, 0], [0, 0], color=BLUE, lw=0.8, zorder=2)
    ax.text(-30, -1.2, "proximal contact face (frame)", fontsize=5.6, color=BLUE, va="top")
    # the distal contact point's path from 0 to q about each pole
    qs = np.linspace(0, q, 60)
    for P, col, ls, lw in ((P0, GREY, (0, (2.4, 1.4)), 1.0), (P1, AMBER, "-", 1.4)):
        arc = np.array([rot([c0], qq, P)[0] for qq in qs])
        ax.plot(*arc.T, color=col, lw=lw, ls=ls, zorder=3)
    ax.text(-30, t0 + 0.6, "reference link line", fontsize=5.4, color=GREY, va="bottom")
    ax.text(-30, t1 + 0.6, "link line, 4 mm thicker pad (joint kept)", fontsize=5.4, color=INK2, va="bottom")
    # poles and their shift
    ax.add_patch(FancyArrowPatch(P0, P1, arrowstyle="-|>", mutation_scale=6, lw=0.9, color=PURPLE, zorder=6))
    ax.plot(*P0, marker="o", ms=4.0, mfc="white", mec=PURPLE, mew=0.9, zorder=7)
    ax.plot(*P1, marker="o", ms=4.0, color=PURPLE, mec="white", mew=0.4, zorder=7)
    ax.text(1.5, (t0 + t1) / 2, "$\\Delta\\mathbf{p}$", fontsize=7, color=PURPLE, va="center")
    ax.text(1.5, t0 - 1.6, "keep it here: $\\Delta\\mathbf{c}=0$", fontsize=5.6, color=PURPLE, va="top")
    # contact points and their shift
    ax.plot(*c0, marker="o", ms=3.0, color=INK, zorder=5)
    ax.plot(*c_ref, marker="o", ms=3.0, color=GREY, zorder=5)
    ax.plot(*c_red, marker="o", ms=3.0, color=AMBER, zorder=5)
    ax.add_patch(FancyArrowPatch(c_ref, c_red, arrowstyle="-|>", mutation_scale=6, lw=0.9, color=INK, zorder=6))
    mid = (c_ref + c_red) / 2
    ax.text(mid[0] + 1.2, mid[1] - 0.5, "$\\Delta\\mathbf{c}$", fontsize=7, color=INK, va="center")
    ax.text(L - 1, -1.2, "contact point, $q=0$", fontsize=5.6, color=INK2, ha="right", va="top")
    ax.text(-30, -30.5, "$\\|\\Delta\\mathbf{c}(q)\\| = 2\\sin\\frac{q}{2}\\,\\|\\Delta\\mathbf{p}\\|$",
            fontsize=7.4, va="bottom")
    ax.set_xlim(-32, 31); ax.set_ylim(-31, 14); ax.set_aspect("equal"); ax.axis("off")


def clearance_row(ax, z, t=7.0, q=np.radians(120), L=16.0):
    """One finger end for Fig. 2(a): proximal link and pad, the distal pad extended (dashed ghost) and flexed by q
    about the pole z below the link line, with the gap G = 2 (t - z)_+ tan(q/2) the pads need."""
    s = max(t - z, 0.0) * np.tan(q / 2)
    P = np.array([s / 2 if z < t else 0.0, -z]) if False else np.array([0.0, -z])
    ax.plot([-L - 2, -s / 2 if z < t else 0], [0, 0], color=GREY, lw=1.8, solid_capstyle="butt")
    ax.add_patch(Polygon([[-L, 0], [-s / 2, 0], [-s / 2, -t], [-L, -t]], fc="#d6e4f5", ec=BLUE, lw=0.7))
    ghost = [[s / 2, 0], [s / 2 + L - 2, 0], [s / 2 + L - 2, -t], [s / 2, -t]]
    ax.add_patch(Polygon(ghost, fc="none", ec="#a8a8a6", lw=0.6, ls=(0, (2, 1.2))))
    ax.plot([-s / 2, s / 2 + L], [0, 0], color="#a8a8a6", lw=0.5, ls=(0, (1, 1)))
    P = np.array([0.0, -z])
    flexed = rot(ghost, q, P)
    ax.add_patch(Polygon(flexed, fc="#d6e4f5", ec=BLUE, lw=0.7))
    ax.plot(*rot([[s / 2, 0], [s / 2 + L - 1, 0]], q, P).T, color=GREY, lw=1.8, solid_capstyle="butt")
    ax.plot(*P, marker="o", ms=4.2, color=PURPLE, mec="white", mew=0.4, zorder=6)
    ax.text(P[0] + 1.2, P[1] - 0.4, "$P$", fontsize=6.6, color=PURPLE, va="top", style="italic")
    if s > 0.2:
        ax.annotate("", xy=(-s / 2, 3.0), xytext=(s / 2, 3.0),
                    arrowprops=dict(arrowstyle="<|-|>", lw=0.6, color=INK, mutation_scale=4, shrinkA=0, shrinkB=0))
        for x in (-s / 2, s / 2):
            ax.plot([x, x], [0, 3.6], color=INK, lw=0.4)
        ax.text(0, 4.2, f"$G={2 * s:.1f}$ mm", fontsize=6.6, ha="center", va="bottom")
    else:
        ax.text(0, 4.2, "$G=0$", fontsize=6.6, ha="center", va="bottom", color=BLUE)
    ax.set_xlim(-L - 3, L + 14); ax.set_ylim(-L - 6, 9); ax.set_aspect("equal"); ax.axis("off")


SERVO_DEPTH = 5.3  # mm below the link line: the 2R-DD pad mount (its 3 mm pad's face is 8.3 mm deep, Study 1, 120 deg)


def line_cross(p1, p2, p3, p4):
    """Intersection of the lines p1-p2 and p3-p4."""
    d1, d2 = np.subtract(p2, p1), np.subtract(p4, p3)
    t = np.cross(np.subtract(p3, p1), d2) / np.cross(d1, d2)
    return np.asarray(p1) + t * d1


def flexed_pair(ax, z, t=7.0, q=np.radians(120), Lp=30.0, colour=BLUE, servo=False, remote=False, linkage=False):
    """Both phalanges' pads at flexion q about a pole z below the link line, the phalanx (link) length Lp fixed on
    each side of the joint: each pad keeps Lp - (t - z)_+ tan(q/2); the part lost to the joint is hatched."""
    m = SERVO_DEPTH if servo else 0.0  # pads mount below the servo body
    s = max(t + m - z, 0.0) * np.tan(q / 2)
    P = np.array([0.0, -z])
    ax.plot([-Lp, 0], [0, 0], color=GREY, lw=1.8, solid_capstyle="butt", zorder=3)
    if servo:
        from matplotlib.patches import FancyBboxPatch
        ax.add_patch(FancyBboxPatch((-14, -m), 20, m + 9, boxstyle="round,pad=0,rounding_size=2.5", fc="#bdbdbb",
                                    ec="#6b6b69", lw=0.6, zorder=5))
        ax.text(-8, 2.5, "servo", fontsize=5.0, color="white", ha="center", va="center", zorder=6)
    if remote:  # the servo on the proximal phalanx, clear of the joint and the pads
        from matplotlib.patches import FancyBboxPatch
        ax.add_patch(FancyBboxPatch((-31, 0.6), 15, 8, boxstyle="round,pad=0,rounding_size=2.0", fc="#bdbdbb",
                                    ec="#6b6b69", lw=0.6, zorder=5))
        ax.text(-23.5, 4.6, "servo", fontsize=5.0, color="white", ha="center", va="center", zorder=6)
    ax.add_patch(Polygon([[-Lp, -m], [-s, -m], [-s, -m - t], [-Lp, -m - t]], fc="#d6e4f5", ec=colour, lw=0.7,
                         zorder=2))
    dist_pad = [[s, -m], [Lp, -m], [Lp, -m - t], [s, -m - t]]
    ax.add_patch(Polygon(rot(dist_pad, q, P), fc="#d6e4f5", ec=colour, lw=0.7, zorder=2))
    ax.plot(*rot([[0, 0], [Lp, 0]], q, P).T, color=GREY, lw=1.8, solid_capstyle="butt", zorder=3)
    if s > 0.2:
        for poly in ([[-s, -m], [0, -m], [0, -m - t], [-s, -m - t]],
                     rot([[0, -m], [s, -m], [s, -m - t], [0, -m - t]], q, P)):
            ax.add_patch(Polygon(poly, fc="none", ec="#c0392b", lw=0.4, hatch="//////", zorder=4))
    if linkage:  # crossed four-bar: proximal pivots on the link, distal pivots on the distal link, crossing at P
        A1, A2 = np.array([-12.0, 5.0]), np.array([-3.0, 5.0])
        ax.add_patch(Polygon([[-14, 0], [-1, 0], [-3, 6], [-12, 6]], fc="#e3e3e1", ec="#8c8c8c", lw=0.5, zorder=4))
        d0, d1 = rot([[0, 0], [Lp, 0]], q, P)
        B1, B2 = line_cross(A1, P, d0, d1), line_cross(A2, P, d0, d1)
        for A, B in ((A1, B1), (A2, B2)):
            ax.plot(*np.array([A, B]).T, color="#6b6b69", lw=1.1, solid_capstyle="round", zorder=7)
            for pt in (A, B):
                ax.plot(*pt, marker="o", ms=2.6, mfc="white", mec="#6b6b69", mew=0.6, zorder=8)
        ax.plot([-21, A1[0]], [4.6, A1[1]], color="#6b6b69", lw=0.8, zorder=6)  # servo horn to the driver link
    ax.plot(*P, marker="o", ms=3.8, color=PURPLE, mec="white", mew=0.4, zorder=9)
    ax.set_xlim(-Lp - 2, 18); ax.set_ylim(-Lp - 8, 10); ax.set_aspect("equal"); ax.axis("off")


def fig2():
    h = 2.45
    fig = plt.figure(figsize=(W, h))
    head = h - 0.04
    # (a) room: the optimized designs at 7 mm, 120 deg (evaluator geometry) and the pad length each keeps
    import one_finger
    from core.optimization.mechanism import read_config_file
    from visualize import sketch
    from visualize.finger import view_of
    sketch.SHOW_GROUND = False
    cfg = read_config_file(one_finger.DEFAULT_CONFIG)
    label(fig, 0.10, head, "(a) Interface clearance", h)
    s1 = rows(OUT / "study1_thickness_coverage" / "summary.csv")
    cases = (("2r_dd", "servo at the pin (2R-DD)"), ("cfl", "CoDe prototype (CFL)"))
    from core.optimization.mechanism import mechanism as mech_of
    for i, (key, name) in enumerate(cases):
        if key == "cfl":  # the CoDe prototype's finger, flexed to its 180 deg
            from visualize.finger.view import load_run
            from visualize.fronts import read_rows, with_role
            proto = ROOT / "report" / "study0" / "data" / "prototype_seeds.csv"
            row = with_role(read_rows(proto), "balanced")
            run = load_run(proto.parent / ("prototype_" + row["run"].split("/")[-1]))
            v = view_of(run.mechanism.name, run.config, list(run.mechanism.read_design(row)))
            qd = 180.0
        else:
            ds = one_finger.study1_designs(key, cfg["architectures"][key], 120.0, cfg)
            x = list(ds[7.0].x)  # drawn with the servo upright (the optimized design tilts it by 45 deg)
            x[list(mech_of(ds[7.0].mechanism).variable_names).index("servo_tilt_deg")] = 0.0
            v = view_of(ds[7.0].mechanism, ds[7.0].config, x)
            qd = 120.0
        ax = fig.add_axes(((0.10 + i * 1.06) / W, (head - 1.08) / h, 0.98 / W, 0.82 / h))
        ax.set_xlim(-6, 66); ax.set_ylim(-52, 34); ax.set_aspect("equal"); ax.axis("off")
        draw_geometry(ax, v, ARCH[key][1], qd, faint=False)
        pad = v.proximal_pad()
        ell = float(pad[:, 0].max() - pad[:, 0].min())
        hpad = float(pad[:, 1].max() - pad[:, 1].min())
        fig.text((0.10 + i * 1.06 + 0.49) / W, (head - 1.20) / h, f"$h={hpad:.0f}$ mm, $q={qd:.0f}^\\circ$",
                 fontsize=5.6, color=INK2, ha="center", va="top")
        fig.text((0.10 + i * 1.06 + 0.49) / W, (head - 0.20) / h, name, fontsize=6.0, color=ARCH[key][1],
                 ha="center", va="top", fontweight="bold" if key == "cfl" else "normal")
        fig.text((0.10 + i * 1.06 + 0.49) / W, (head - 1.08) / h, f"$\\ell_1={ell:.0f}$ mm", fontsize=6.0,
                 color=ARCH[key][1], ha="center", va="top")
    fig.text(1.12 / W, (head - 1.32) / h, "one scale", fontsize=5.6, color=INK2, ha="center", va="top")
    cax = fig.add_axes((0.45 / W, 0.28 / h, 1.60 / W, (head - 1.78) / h))
    for key in ("2r_dd", "cfl"):
        for qd, ls in ((120.0, "-"), (150.0, (0, (3, 1.6)))):
            pts = sorted((float(r["thickness_mm"]), float(r["proximal_pad_mm"]), float(r["c_pad_mean"]))
                         for r in s1 if r["architecture"] == S1NAME[key] and float(r["required_flexion_deg"]) == qd)
            pts = [(hh, l) for hh, l, c in pts if c > 0.3]  # designs that hold the cylinder
            if pts:
                hh, ll = map(np.array, zip(*pts))
                cax.plot(hh, ll, color=ARCH[key][1], lw=1.2, ls=ls, marker=ARCH[key][2], ms=2.6, mec="white",
                         mew=0.3)
    cax.plot([], [], color=INK2, lw=1.0, label="$q_{req}=120^\\circ$")
    cax.plot([], [], color=INK2, lw=1.0, ls=(0, (3, 1.6)), label="$150^\\circ$")
    cax.legend(loc="lower left", fontsize=5.4, frameon=False, handlelength=1.8, borderaxespad=0.1, ncol=2)
    cax.set_xlim(2.5, 9.5); cax.set_ylim(15, 38); cax.set_xticks([3, 5, 7, 9])
    cax.set_xlabel("pad thickness $h$ (mm)", labelpad=1); cax.set_ylabel("proximal pad $\\ell_1$ (mm)", labelpad=1)
    clean(cax)
    # (b) pole placement: the optimized designs' pole depth against their contact-face depth
    label(fig, 2.30, head, "(b) Pole placement", h)
    fig.text(3.65 / W, (head - 0.20) / h, "$q=120^\\circ$; optimized designs, 3–9 mm pads", fontsize=6.4,
             ha="center", va="top")
    pax = fig.add_axes((2.75 / W, 0.62 / h, 1.85 / W, (head - 0.95) / h))
    pts = json.loads((ROOT / "design" / "output" / "paper" / "fig2_pole_depths.json").read_text())
    tt = np.array([0, 19.5])
    pax.fill_between(tt, tt, 16, color="#eef3fa", lw=0, zorder=0)
    pax.plot(tt, tt, color=PURPLE, lw=0.9, ls=(0, (3, 1.6)), zorder=1)
    pax.axhline(0, color=INK2, lw=0.5, ls=(0, (1, 1.4)), zorder=1)
    pax.text(1.0, 14.6, "zero pad gap\n(symmetric case)", fontsize=6.0, color=PURPLE, va="top", linespacing=1.0)
    pax.text(9.4, 10.6, "$z=t$", fontsize=6.0, color=PURPLE, rotation=38, ha="center", va="center")
    for key in ("2r_remote", "2r_dd", "rcj", "cfl"):
        d = pts[key]
        t, z = np.array([r["t"] for r in d]), np.array([r["z"] for r in d])
        if key == "rcj":
            pax.plot([t.min() - 1.5, t.max() + 0.8], [z.mean()] * 2, color=TEAL, lw=0.7, alpha=0.6, zorder=2)
        pax.plot(t, z, ls="none", marker={"cfl": "o", "2r_remote": "s", "2r_dd": "D", "rcj": "^"}[key], ms=4.0,
                 color=ARCH[key][1], mec="white", mew=0.4, zorder=3, label=ARCH[key][0], clip_on=False)
    pax.set_xlim(0, 19.5); pax.set_ylim(-0.8, 16); pax.set_xticks([0, 5, 10, 15]); pax.set_yticks([0, 5, 10, 15])
    pax.set_xlabel("contact-face depth $t$ (mm)", labelpad=1); pax.set_ylabel("pole depth $z$ (mm)", labelpad=1)
    clean(pax)
    pax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.28), ncol=4, frameon=False, fontsize=6.0,
               handletextpad=0.1, columnspacing=0.6)
    # (c) motion preservation: the same three designs from the skeleton frame and from the contact frame
    label(fig, 4.80, head, "(c) Motion preservation", h)
    fig.text(5.85 / W, (head - 0.20) / h, "$t: 6\\to10$ mm; $q: 0\\to120^\\circ$", fontsize=6.4, ha="center",
             va="top")
    q = np.radians(np.linspace(0, 120, 61))
    L, z0, t0, t1 = 22.0, 0.0, 6.0, 10.0
    dt = t1 - t0

    def path(pt, pole):
        return np.array([rot([pt], qq, np.asarray(pole))[0] for qq in q])
    designs = (("reference", GREY, (0, -z0), t0, (0, (2.6, 1.6)), 1.0, 5),
               ("skeleton-preserving", AMBER, (0, -z0), t1, "-", 1.4, 3),
               ("contact-preserving", BLUE, (0, -z0 - dt), t1, "-", 1.4, 3))
    for row, (frame, name) in enumerate((("skel", "Skeleton frame"), ("contact", "Contact frame"))):
        y0 = head - 0.42 - (row + 1) * 0.60
        ax = fig.add_axes((5.05 / W, y0 / h, 1.10 / W, 0.55 / h))
        ends = {}
        for nm, col, pole, t, ls, lw, zo in designs:
            pth = path((L, 0.0), pole) if frame == "skel" else path((L, -t), pole) - np.array([0.0, -t])
            ax.plot(*pth.T, color=col, lw=lw, ls=ls, zorder=zo)
            ends[nm] = pth[-1]
        ax.plot([-L - 2, L + 2], [0, 0], color=INK2, lw=0.4, ls=(0, (1, 1.4)), zorder=0)
        ax.plot(L, 0, marker="o", ms=3.4, color=GREY, zorder=6)
        off = "contact-preserving" if frame == "skel" else "skeleton-preserving"
        a_, b_ = ends["reference"], ends[off]
        ax.plot(*np.array([a_, b_]).T, color=PURPLE, lw=0.8, zorder=7)
        ax.plot(*a_, marker="o", ms=3.0, color=GREY, zorder=7)
        ax.text(-L - 2, -4, f"{np.linalg.norm(a_ - b_):.2f} mm", fontsize=6.0, color=PURPLE, va="top")
        ax.set_aspect("equal"); ax.axis("off"); ax.set_xlim(-L - 3, L + 3); ax.set_ylim(-L - 4, 3)
        fig.text(4.92 / W, (y0 + 0.30) / h, name, fontsize=6.4, va="center", rotation=90, ha="center",
                 color=INK2)
    handles = [plt.Line2D([], [], color=GREY, lw=1.0, ls=(0, (2.6, 1.6)), label="reference"),
               plt.Line2D([], [], color=AMBER, lw=1.4, label="skeleton-preserving"),
               plt.Line2D([], [], color=BLUE, lw=1.4, label="contact-preserving")]
    fig.legend(handles=handles, loc="center left", bbox_to_anchor=(6.16 / W, 1.05 / h), frameon=False,
               fontsize=5.8, handlelength=1.4, labelspacing=0.3, handletextpad=0.3)
    save_rgb(fig, DEST / "fig_mechanisms.png", dpi=400)
    plt.close(fig)
    print("fig2", h)


# ------------------------------------------------------------------------------------------------------------ Fig. 3
def common_interval(pf, df):
    """The distances T = T_1 & T_2 from the faces' crossing X (problem formulation A.3, occlusion ignored), and X and
    the two open rays; None if parallel or empty."""
    rays = []
    for f in (pf, df):
        a = f[0] - f[1]
        rays.append(a / np.linalg.norm(a))
    cross = rays[0][0] * rays[1][1] - rays[0][1] * rays[1][0]
    if abs(cross) < 1e-9:
        return None
    gap = df[1] - pf[1]
    s = (gap[0] * rays[1][1] - gap[1] * rays[1][0]) / cross
    X = pf[1] + s * rays[0]
    spans = []
    for f, r in zip((pf, df), rays):
        a, b = float((f[0] - X) @ r), float((f[1] - X) @ r)
        spans.append((max(min(a, b), 0.0), max(a, b)))
    lo, hi = max(spans[0][0], spans[1][0]), min(spans[0][1], spans[1][1])
    return (X, rays, lo, hi) if hi > lo else None


def draw_geometry(ax, view, colour, q, faint=True):
    """The design at 0 (faint) and q deg (solid): mechanism grey, mounted pads light, the part of each contact face
    that holds the inscribed cylinder at q (paired coverage) dark."""
    import matplotlib.colors as mc
    light = mc.to_hex(np.array(mc.to_rgb(colour)) * 0.35 + 0.65)
    for qq, alpha in (((0.0, 0.22), (q, 1.0)) if faint else ((q, 1.0),)):
        n0p, n0l, n0c = len(ax.patches), len(ax.lines), len(ax.collections)
        view.draw(ax, qq, hit=set(), style="sketch")
        for p in ax.patches[n0p:]:
            p.set_facecolor("#e3e3e1" if alpha == 1 else "#f1f1f0"); p.set_edgecolor("#8c8c8c"); p.set_alpha(alpha)
            p.set_linewidth(0.5)
        for ln in ax.lines[n0l:]:
            ln.set_color("#8c8c8c"); ln.set_alpha(alpha); ln.set_linewidth(min(ln.get_linewidth(), 1.2))
            ln.set_markersize(min(ln.get_markersize(), 2.5))
            ln.set_markerfacecolor("#a8a8a6"); ln.set_markeredgecolor("#8c8c8c")
        for c in ax.collections[n0c:]:
            c.set_facecolor("#a8a8a6"); c.set_edgecolor("#8c8c8c"); c.set_alpha(alpha)
        for pad in (view.proximal_pad(), view.distal_pad(qq)):
            ax.add_patch(Polygon(pad, fc=light, ec=colour, lw=0.4, alpha=alpha, zorder=20))
    pf = view.proximal_pad()[[0, 1]]
    dpad = view.distal_pad(q)
    df = dpad[[1, 0]]
    got = common_interval(pf, df)
    if got:
        X, rays, lo, hi = got
        for r in rays:
            seg = np.array([X + lo * r, X + hi * r])
            ax.plot(*seg.T, color=colour, lw=2.4, solid_capstyle="butt", zorder=25)


def fig3():
    import one_finger
    from core.optimization.mechanism import read_config_file
    from visualize import sketch
    from visualize.finger import view_of
    sketch.SHOW_GROUND = False
    cfg = read_config_file(one_finger.DEFAULT_CONFIG)
    q_req, h_rep = 120.0, 7.0
    views = {}
    for key in ARCH:
        ds = one_finger.study1_designs(key, cfg["architectures"][key], q_req, cfg)
        views[key] = view_of(ds[h_rep].mechanism, ds[h_rep].config, ds[h_rep].x)
    probe_fig, probe = plt.subplots()
    for v in views.values():
        v.draw(probe, 0.0, hit=set(), style="sketch"); v.draw(probe, q_req, hit=set(), style="sketch")
    box = probe.dataLim
    plt.close(probe_fig)
    xlim, ylim = (box.x0 - 2, box.x1 + 2), (box.y0 - 2, box.y1 + 2)
    aspect = (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])
    s1 = rows(OUT / "study1_thickness_coverage" / "summary.csv")
    s2 = rows(OUT / "study2_redesign_2026-11-21" / "summary.csv")
    traj = json.loads((ROOT / "design" / "output" / "paper" / "fig3_paths_q120_3to7.json").read_text())
    h = 2.85
    fig = plt.figure(figsize=(W, h))
    row_a, row_b = h - 0.20, h / 2 - 0.08
    # (a) left: optimized geometry, one scale
    label(fig, 0.02, h - 0.02, f"(a) Coverage: optimized designs at $q_{{req}}={q_req:.0f}^\\circ$, {h_rep:.0f} mm pads",
          h)
    gw = 0.93
    gh = gw * aspect
    for i, key in enumerate(ARCH):
        ax = fig.add_axes(((0.04 + i * (gw + 0.02)) / W, (row_a - 0.12 - gh) / h, gw / W, gh / h))
        ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_aspect("equal"); ax.axis("off")
        draw_geometry(ax, views[key], ARCH[key][1], q_req)
        c = next(float(r["c_pad_mean"]) for r in s1 if r["architecture"] == S1NAME[key]
                 and float(r["required_flexion_deg"]) == q_req and float(r["thickness_mm"]) == h_rep)
        fig.text((0.04 + i * (gw + 0.02) + gw / 2) / W, (row_a - 0.06) / h, ARCH[key][0], fontsize=6.6,
                 color=ARCH[key][1], ha="center", va="top", fontweight="bold" if key == "cfl" else "normal")
        fig.text((0.04 + i * (gw + 0.02) + gw / 2) / W, (row_a - 0.14 - gh) / h, f"$\\bar C_{{pad}}$ = {100 * c:.0f}%",
                 fontsize=6.2, color=ARCH[key][1], ha="center", va="top")
    fig.text(0.05 / W, (row_b + 0.30) / h, "dark: contact surface holding the cylinder;  light: mounted pad;  "
             "grey: actuator, linkage, rollers, joints;  faint: extended", fontsize=5.6, color=INK2, va="center")
    # (a) right: coverage dots, every condition
    for j, q in enumerate((90.0, 120.0, 150.0)):
        ax = fig.add_axes(((4.32 + j * 0.95) / W, (row_b + 0.55) / h, 0.82 / W, (row_a - row_b - 0.85) / h))
        for k, key in enumerate(ARCH):
            for hh in (3.0, 5.0, 7.0, 9.0):
                r = [x for x in s1 if x["architecture"] == S1NAME[key] and float(x["required_flexion_deg"]) == q
                     and float(x["thickness_mm"]) == hh]
                x = hh + (k - 1.5) * 0.32
                if r and float(r[0]["c_pad_mean"]) > 0.3:
                    ax.plot(x, 100 * float(r[0]["c_pad_mean"]), ls="none", marker=ARCH[key][2], ms=3.0,
                            color=ARCH[key][1], mec="white", mew=0.3, zorder=3)
                else:
                    ax.plot(x, 46, ls="none", marker="x", ms=2.8, color=ARCH[key][1], mew=0.7, clip_on=False)
        ax.set_xticks([3, 5, 7, 9]); ax.set_xlim(2, 10); ax.set_ylim(45, 88)
        ax.set_title(f"$q_{{req}}={q:.0f}^\\circ$", fontsize=6.4, pad=1)
        if j == 0:
            ax.set_ylabel("$\\bar C_{pad}$ (%)", labelpad=1)
        else:
            ax.set_yticklabels([])
        if j == 1:
            ax.set_xlabel("pad thickness (mm);  $\\times$: no feasible design", labelpad=1, fontsize=6.2)
        clean(ax)
    # (b) left: reference and redesign paths, 120 deg, 3 -> 7 mm
    label(fig, 0.02, row_b + 0.02, "(b) Preservation: redesign from 3 to 7 mm at $q_{req}=120^\\circ$", h)
    axc = fig.add_axes((0.30 / W, 0.30 / h, 1.80 / W, (row_b - 0.48) / h))
    axs = fig.add_axes((2.30 / W, 0.30 / h, 1.80 / W, (row_b - 0.48) / h))
    for key in ("2r_remote", "cfl"):  # the fixed axis and the CFL; the others are in the dots
        t = traj[key]
        col = ARCH[key][1]
        for ax, a, b in ((axc, "ref", "red"), (axs, "ref_tip", "red_tip")):
            ref, red = np.array(t[a]), np.array(t[b])
            ax.plot(*ref.T, color=col, lw=0.8, ls=(0, (2.2, 1.4)), zorder=2)
            ax.plot(*red.T, color=col, lw=1.3, zorder=3)
    for ax, ttl in ((axc, "distal contact point, contact frame"), (axs, "fingertip, skeleton frame")):
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title(ttl, fontsize=6.4, pad=1)
        ax.set_xlabel("x (mm)", labelpad=1); ax.set_ylabel("z (mm)", labelpad=1)
        clean(ax)
    axc.plot([], [], color=INK2, lw=0.8, ls=(0, (2.2, 1.4)), label="3 mm reference")
    axc.plot([], [], color=INK2, lw=1.3, label="7 mm redesign")
    axc.text(0.98, 0.06, "CFL keeps it", transform=axc.transAxes, fontsize=5.8, color=BLUE, ha="right")
    axs.text(0.02, 0.06, "2R-remote keeps it", transform=axs.transAxes, fontsize=5.8, color=AMBER, ha="left")
    axc.legend(fontsize=5.6, frameon=False, loc="lower left", handlelength=1.8, borderaxespad=0.1)
    # (b) right: contact deviation dots, every condition, fixed-axis value dashed
    for j, q in enumerate((90.0, 120.0, 150.0)):
        ax = fig.add_axes(((4.32 + j * 0.95) / W, 0.30 / h, 0.82 / W, (row_b - 0.48) / h))
        for k, key in enumerate(ARCH):
            for hh in (5.0, 7.0, 9.0):
                r = [x for x in s2 if x["architecture"] == key and float(x["required_flexion_deg"]) == q
                     and float(x["to_mm"]) == hh]
                x = hh + (k - 1.5) * 0.32
                if r and r[0]["change_at_most_c_pad"] not in ("", "nan"):
                    ax.plot(x, LF * float(r[0]["change_at_most_c_pad"]), ls="none", marker=ARCH[key][2], ms=3.0,
                            color=ARCH[key][1], mec="white", mew=0.3, zorder=3)
                else:
                    ax.plot(x, 0.0, ls="none", marker="x", ms=2.8, color=ARCH[key][1], mew=0.7, clip_on=False)
        for hh in (5.0, 7.0, 9.0):
            fa = 2 * np.sin(np.radians(q) / 2) * (hh - 3.0)
            ax.plot([hh - 0.75, hh + 0.75], [fa, fa], color=INK, lw=0.8, ls=(0, (1.5, 1.2)), zorder=2)
        ax.set_xticks([5, 7, 9]); ax.set_xlim(4, 10); ax.set_ylim(0, 12.5)
        ax.set_title(f"$q_{{req}}={q:.0f}^\\circ$", fontsize=6.4, pad=1)
        if j == 0:
            ax.set_ylabel("$\\Delta_{contact}$ (mm)", labelpad=1)
        else:
            ax.set_yticklabels([])
        if j == 1:
            ax.set_xlabel("redesigned for (mm);  dashed: fixed axis", labelpad=1, fontsize=6.2)
        clean(ax)
    handles = [plt.Line2D([], [], ls="none", marker=ARCH[k][2], ms=3.4, color=ARCH[k][1], label=ARCH[k][0])
               for k in ARCH]
    fig.legend(handles=handles, loc="upper right", bbox_to_anchor=(1.0, 1.0), ncol=4, frameon=False, fontsize=6.0,
               handletextpad=0.2, columnspacing=0.8)
    save_rgb(fig, DEST / "fig_study12.png", dpi=400)
    plt.close(fig)
    print("fig3", h)


# ------------------------------------------------------------------------------------------------------------ Fig. 4
def task_icon(ax, kind):
    """A two-pad finger and its object, drawn for the task (schematic)."""
    ax.set_xlim(-11, 5); ax.set_ylim(-9, 6); ax.set_aspect("equal"); ax.axis("off")
    ax.plot([-9, 1], [4, 4], color=INK2, lw=1.4, solid_capstyle="round")
    ax.add_patch(Rectangle((-8.5, 2.6), 9, 1.4, fc="#c9d9f2", ec=BLUE, lw=0.5))
    d = rot([[1, 4], [1, -6]], 0, np.array([1, 4]))
    ax.plot(*d.T, color=INK2, lw=1.4, solid_capstyle="round")
    ax.add_patch(Rectangle((-0.4, -5.5), 1.4, 9, fc="#c9d9f2", ec=BLUE, lw=0.5))
    arrow = dict(arrowstyle="-|>", lw=0.9, color=INK, mutation_scale=6)
    if kind in ("friction", "stiffness"):
        ax.add_patch(Circle((-3.6, -1.7), 4.2, fc="#f2e2cc", ec="#9a7a52", lw=0.6))
        ax.add_patch(FancyArrowPatch((-3.6, -1.7), (-8.8, -7.5), **arrow))
        return ("pull-out load; pad friction $1.3\\to2.0$" if kind == "friction" else
                "pull-out load; pad stiffness $2.5\\to1.0$ kN/m")
    elif kind == "acquire":
        ax.add_patch(Circle((-5.0, -4.0), 3.4, fc="#f2e2cc", ec="#9a7a52", lw=0.6))
        ax.add_patch(FancyArrowPatch((-7.5, -6.8), (-4.0, -3.0), **arrow))
        return "close and draw the object in; pad friction"
    else:
        ax.add_patch(Circle((-3.6, -1.7), 4.2, fc="#f2e2cc", ec="#9a7a52", lw=0.6))
        ax.plot([-3.6, -3.6], [-1.7, 2.0], color="#9a7a52", lw=0.8)
        ax.add_patch(FancyArrowPatch((-8.6, -1.0), (-6.0, -5.6), connectionstyle="arc3,rad=0.4", **arrow))
        return "turn a sprung dial $45^\\circ$; pad friction"


def fig4():
    isaac_f = {k: (v["D_c_mm"], v["E_rel_median"]) for k, v in
               json.load(open(S3 / "isaac_dose_A_all.json"))["hands"].items() if v.get("E_rel_median") is not None}
    isaac_k = {k: (v["D_c_mm"], v["E_median"]) for k, v in json.load(open(S3 / "isaac_comp.json"))["hands"].items()}
    acq = {r["hand"]: (r["d_contact"], r["E"]) for r in json.load(open(S3 / "drake_e23_point.json"))}
    dial = {r["hand"]: (r["d_contact"], r["E"]) for r in json.load(open(S3 / "drake_e26_point.json"))}
    cols = (("Hold: pad friction", "Isaac Lab", "friction", isaac_f),
            ("Hold: pad stiffness", "Isaac Lab", "stiffness", isaac_k),
            ("Grasp acquisition", "Drake", "acquire", acq),
            ("Dial turning", "Drake", "dial", dial))
    h = 3.15
    fig = plt.figure(figsize=(W, h))
    cw = 1.62
    for i, (name, sim, kind, data) in enumerate(cols):
        x0 = 0.42 + i * (cw + 0.12)
        fig.text((x0 + cw / 2) / W, (h - 0.02) / h, name, fontsize=7, fontweight="bold", ha="center", va="top")
        fig.text((x0 + cw / 2) / W, (h - 0.16) / h, sim, fontsize=6.0, color=INK2, ha="center", va="top")
        ia = fig.add_axes((x0 / W, (h - 0.86) / h, cw / W, 0.58 / h))
        what = task_icon(ia, kind)
        fig.text((x0 + cw / 2) / W, (h - 0.90) / h, what, fontsize=5.6, color=INK2, ha="center", va="top")
        # error of R and U
        ba = fig.add_axes(((x0 + 0.25) / W, (h - 1.58) / h, (cw - 0.30) / W, 0.46 / h))
        er, eu = 100 * data["R"][1], 100 * data["U"][1]
        ba.barh([1, 0], [er, eu], height=0.6, color=[BLUE, AMBER])
        for y, v in ((1, er), (0, eu)):
            ba.text(v + 6, y, f"{v:.0f}%", va="center", fontsize=6.4)
        ba.set_xlim(0, 260); ba.set_yticks([1, 0]); ba.set_yticklabels(["R", "U"])
        ba.set_xticks([0, 100, 200])
        clean(ba)
        if i == 0:
            ba.set_xlabel("error in the measured effect (%)", labelpad=1, fontsize=6.2)
        # every redesign
        sa = fig.add_axes(((x0 + 0.25) / W, 0.50 / h, (cw - 0.30) / W, 0.72 / h))
        for k, (dc, e) in data.items():
            if k in ("R", "U", "O"):
                continue
            sa.plot(dc, 100 * e, ls="none", marker="o", ms=2.8, color="#b5b5b2", mec="none", zorder=2)
        sa.plot(*[data["R"][0], 100 * data["R"][1]], ls="none", marker="o", ms=4.0, color=BLUE, zorder=3)
        sa.plot(*[data["U"][0], 100 * data["U"][1]], ls="none", marker="o", ms=4.0, color=AMBER, zorder=3)
        sa.set_xlim(0, 5.8); sa.set_ylim(0, max(100 * max(e for _, e in data.values()) * 1.1, 100))
        sa.set_xlabel("$\\Delta_{contact}$ (mm)", labelpad=1)
        if i == 0:
            sa.set_ylabel("error (%)", labelpad=1)
        clean(sa)
    handles = [plt.Line2D([], [], ls="none", marker="o", ms=4, color=BLUE, label="R: contact-preserving"),
               plt.Line2D([], [], ls="none", marker="o", ms=4, color=AMBER, label="U: coverage-only"),
               plt.Line2D([], [], ls="none", marker="o", ms=3, color="#b5b5b2", label="other redesigns")]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, 0.0), ncol=3, frameon=False, fontsize=6.2,
               handletextpad=0.2, columnspacing=1.2)
    fig.text(0.02 / W, (h - 1.35) / h, "error of\nR and U", fontsize=6.0, color=INK2, rotation=90, ha="left",
             va="center", linespacing=1.0)
    fig.text(0.02 / W, 0.86 / h, "every redesign\n(3 to 5, 7, 9 mm)", fontsize=6.0, color=INK2, rotation=90,
             ha="left", va="center", linespacing=1.0)
    save_rgb(fig, DEST / "fig_study3.png", dpi=400)
    plt.close(fig)
    print("fig4", h)


# ----------------------------------------------------------------------------------------------------------- Table I
def table():
    rowsd = (("Isaac Lab, point contact", json.load(open(S3 / "isaac_dose_A_all.json"))["hands"], "E_rel_median"),
             ("Drake, point contact", {r["hand"]: r for r in json.load(open(S3 / "drake_e21_point.json"))}, "E"),
             ("Drake, hydroelastic", {r["hand"]: r for r in json.load(open(S3 / "drake_e21_hydro.json"))}, "E"),
             ("Drake, hydroelastic, thickness-dependent",
              {r["hand"]: r for r in json.load(open(S3 / "drake_e25_hydro_own.json"))}, "E"))
    lines = [r"\begin{table}[t]", r"  \centering",
             r"  \caption{Error of $R$ and $U$ by contact model (pull-out, pad friction change)}",
             r"  \label{tab:robust}", r"  \begin{tabular}{lrr}", r"    \toprule",
             r"    Contact model & $R$ & $U$ \\", r"    \midrule"]
    for name, d, key in rowsd:
        lines.append(f"    {name} & ${100 * d['R'][key]:.0f}\\%$ & ${100 * d['U'][key]:.0f}\\%$ \\\\")
    lines += [r"    \bottomrule", r"  \end{tabular}", r"\end{table}", ""]
    (MS / "sections" / "table_robust.tex").write_text("\n".join(lines))
    print("\n".join(lines))


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    for name, fn in (("fig2", fig2), ("fig3", fig3), ("fig4", fig4), ("table", table)):
        if which in (name, "all"):
            fn()
