"""Figs. 2-4 and Table I of the RSS 2027 extended abstract, from the evaluator and the raw study data (PNG only).

    conda run -n cmd python paper/figures/fig_paper.py [fig2|fig3|fig4|table|all]

Figure plan (user, 2026-11-24): physical meaning first; no trend lines, Pareto fronts or statistics in the figures.
  Fig. 2  kinematic principle (Study 0): (a) clearance set by the pole, (b) achievable poles, (c) motion seen from
          the skeleton frame and from the contact frame. (a) and (c) are schematics from the closed-form model.
  Fig. 3  design validation (Studies 1, 2): (a) optimized geometry at 120 deg, 7 mm (evaluator) and coverage dots;
          (b) reference and redesign paths at 120 deg, 3 -> 7 mm, and contact-deviation dots with the fixed-axis value.
  Fig. 4  scientific validation (Study 3): four Drake tasks (power-grasp hold, grasp acquisition, fingertip dial, box
          pivot): the task, the three morphologies' (CoDe, Skeleton, Coverage) response minus O's, the bias in
          the friction effect, and each task's error E for each with point and hydroelastic contact.
  Table I contact-model robustness (CoDe, Skeleton and Coverage errors), written as LaTeX to sections/table_robust.tex.
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
# figures4papers house palette: method blue, comparator warm red, reference grey (paper/SSoT.md §6)
BLUE, AMBER, GREY, TEAL, PURPLE = "#0F4D92", "#B64342", "#767676", "#42949E", "#9A4D8E"
ARCH = {"2r_dd": ("2R-DD", GREY, "^"), "2r_remote": ("2R-remote", AMBER, "s"), "rcj": ("RCJ", TEAL, "D"),
        "cfl": ("CoDe", BLUE, "o")}
S1NAME = {"2r_dd": "2R-DD", "2r_remote": "2R-remote", "rcj": "RCJ", "cfl": "CFL"}
LF = 90.0

from pngout import register_fonts, save_figure  # noqa: E402
register_fonts()
# Helvetica Light for text; Nimbus Sans (Helvetica metrics) for bold and math italic, which the Light cut lacks
plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["Helvetica Light", "Nimbus Sans", "DejaVu Sans"],
                     "mathtext.fontset": "custom", "mathtext.rm": "Helvetica Light", "mathtext.it": "Nimbus Sans:italic",
                     "mathtext.bf": "Nimbus Sans:bold", "font.size": 7, "font.weight": "light", "axes.linewidth": 0.6,
                     "xtick.major.width": 0.6, "ytick.major.width": 0.6, "xtick.major.size": 2,
                     "ytick.major.size": 2, "axes.labelsize": 7, "xtick.labelsize": 6.3, "ytick.labelsize": 6.3})


def rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def clean(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def label(fig, x, y, text, h):
    fig.text(x / W, y / h, text, fontsize=7.5, fontweight="bold", family="Nimbus Sans", va="top", ha="left")


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
    ax.add_patch(Polygon([[-30, 0], [-3, 0], [-3, t1], [-30, t1]], fc="#F6CFCB", ec=AMBER, lw=0.5, zorder=1))
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


def common_pad_extension(view_of, mech, configs, x, qd=150.0, step=0.25, margin=0.3, max_ed=0.0):
    """One lengthening (ep proximal, ed distal, toward the joint) for every pad thickness in ``configs``: as long as
    every pair stays ``margin`` clear over 0..qd. Returns (ep, ed, gap at qd of the thickest pads)."""
    from shapely.geometry import Polygon as SPoly
    data = []
    for cf in configs:
        v = view_of(mech, cf, x)
        d0 = v.distal_pad(0.0)
        poses = []
        for qq in np.linspace(0.0, qd, 61):
            dq = v.distal_pad(qq)
            ang = np.arctan2(*(dq[1] - dq[0])[::-1]) - np.arctan2(*(d0[1] - d0[0])[::-1])
            poses.append((np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]]), dq[1]))
        data.append((v.proximal_pad().copy(), d0, poses))

    def padded(pp0, d0, ep, ed):
        pp = pp0.copy(); pp[[1, 2], 0] += ep
        dd = d0.copy(); dd[[0, 3], 0] -= ed
        return pp, dd

    def clear(ep, ed):
        for pp0, d0, poses in data:
            pp, dd = padded(pp0, d0, ep, ed)
            sp = SPoly(pp).buffer(margin)
            if any(sp.intersection(SPoly((dd - d0[1]) @ R.T + t)).area > 1e-4 for R, t in poses):
                return False
        return True
    ep = ed = 0.0
    for _ in range(3):
        while ep < 30 and clear(ep + step, ed):
            ep += step
        while ed + step <= max_ed and clear(ep, ed + step):
            ed += step
    pp0, d0, poses = data[0]
    pp, dd = padded(pp0, d0, ep, ed)
    R, t = poses[-1]
    return ep, ed, SPoly(pp).distance(SPoly((dd - d0[1]) @ R.T + t))


def faint_extended(ax, v, pads, pins, bodies=True):
    """The finger extended (q = 0), faint, under the flexed drawing."""
    from matplotlib.patches import Circle as _C
    n0p, n0l, n0c = len(ax.patches), len(ax.lines), len(ax.collections)
    v.draw(ax, 0.0, hit=set(), style="sketch")
    for p_ in ax.patches[n0p:]:
        p_.set_facecolor("#f3f3f2"); p_.set_edgecolor("#b5b5b3"); p_.set_alpha(0.5); p_.set_linewidth(0.4)
        p_.set_zorder(0.5)
    for ln in ax.lines[n0l:]:
        ln.set_color("#c4c4c2"); ln.set_linewidth(min(ln.get_linewidth(), 1.0)); ln.set_markersize(0)
        ln.set_zorder(0.6)
    for c in list(ax.collections[n0c:]):
        c.remove()
    if not pins:
        for p_ in list(ax.patches[n0p:]):
            if isinstance(p_, _C):
                p_.remove()
    if not bodies:
        for p_ in list(ax.patches[n0p:]):
            if not isinstance(p_, _C):
                p_.remove()
    for pad in pads:
        ax.add_patch(Polygon(pad, fc="#dbe6f6", ec="#9db6dd", lw=0.4, ls=(0, (2, 1)), zorder=0.8))


def fig2():
    """Single column, two panels stacked: (a) the poles each joint can reach, (b) the CoDe kinematic model over the
    CAD side view (paper/figures/cad/finger_side.png, registered on pivots A and B)."""
    WC = 3.45
    ha, hb = 1.78, 1.20
    h = ha + hb + 0.10
    fig = plt.figure(figsize=(WC, h))
    head = h - 0.02

    def lab(x, y, text):
        fig.text(x / WC, y / h, text, fontsize=7.5, fontweight="bold", family="Nimbus Sans", va="top", ha="left")
    # (b) where each joint can put the pole (Study 0): the poles reachable over each design space
    from fig_mechanisms import cached_outlines, roller_pole
    lab(0.02, head, "(a) Achievable poles")
    fig.text((WC - 0.04) / WC, head / h, "over each joint's design space, $q\\in[0, 120^\\circ]$", fontsize=6.0,
             color=INK2, ha="right", va="top")
    pax = fig.add_axes((0.40 / WC, (hb + 0.10 + 0.30) / h, (WC - 0.46) / WC, (ha - 0.50) / h))
    # CFL: poles reachable at some q in [10, 120] deg, by two-position synthesis with closure checked
    # (fig2_pole_manifold.py, no sampling); RCJ: P(q) = r - i r tan(q/4) family over the same q and radii
    man = np.load(ROOT / "design" / "output" / "paper" / "fig2_pole_manifold.npz")
    reach = man["masks"].any(axis=0).astype(float)
    pax.axhspan(-12, -6, color="#e6e5e1", lw=0, zorder=0)
    pax.contourf(man["x"], man["z"], reach, levels=[0.5, 1.5], colors=["#dbe7f7"], zorder=0.5)
    pax.contour(man["x"], man["z"], reach, levels=[0.5], colors=[BLUE], linewidths=0.9, zorder=0.6)
    # RCJ, equal rollers, bound as the CFL's space: both pivots on the link line within the 90 mm finger, so
    # |AB| = 2r <= 90 mm, and r >= 9.5 mm for the pins' hardware; P(q) = r (1 - i tan(q/4)), q in [0, 120 deg]
    from scipy.spatial import ConvexHull
    rr = np.linspace(9.5, 45.0, 80)
    qq = np.radians(np.linspace(0.0, 120.0, 61))
    R_, Q_ = np.meshgrid(rr, qq)
    P_ = roller_pole(R_.ravel(), R_.ravel(), Q_.ravel())
    pts_ = np.c_[P_.real, P_.imag]
    pts_ = pts_[np.isfinite(pts_).all(1)]
    hull = pts_[ConvexHull(pts_).vertices]
    pax.fill(hull[:, 0], hull[:, 1], fc="#d3eeea", ec=TEAL, lw=0.9, zorder=3)  # the same style as CoDe's region
    pax.plot([-32, 60], [0, 0], color=GREY, lw=2.0, zorder=4)
    import matplotlib.patheffects as pe
    halo = [pe.withStroke(linewidth=2.2, foreground="white")]
    pax.text(-31, 1.2, "Revolute", fontsize=6.2, color=GREY, va="bottom", ha="left", path_effects=halo,
             zorder=6, fontweight="bold", family="Nimbus Sans")
    pax.text(-31, -13.2, "contact faces,\n3–9 mm pads", fontsize=5.6, color=INK2, va="top", linespacing=1.0,
             path_effects=halo, zorder=6)
    pax.text(-14, 18, "CoDe", fontsize=6.4, color=BLUE, fontweight="bold", family="Nimbus Sans", va="top",
             path_effects=halo, zorder=6)
    pax.text(27, -3.0, "Rolling\ncontact joint", fontsize=6.4, linespacing=1.0, ha="center", color=TEAL, fontweight="bold", family="Nimbus Sans", va="top",
             path_effects=halo, zorder=6)
    pax.set_xlim(-32, 55); pax.set_ylim(-27, 30); pax.set_aspect("equal")
    pax.set_xlabel("along the finger (mm)", labelpad=1); pax.set_ylabel("above the link line (mm)", labelpad=1)
    clean(pax)
    # (c) the CFL's kinematic model (the CoDe prototype at 120 deg): pivots, fingertip, frame
    lab(0.02, hb + 0.06, "(b) CoDe kinematic model")
    from visualize.finger import view_of
    from visualize.finger.view import load_run
    from visualize.fronts import read_rows, with_role
    proto = ROOT / "report" / "study0" / "data" / "prototype_seeds.csv"
    row = with_role(read_rows(proto), "balanced")
    run = load_run(proto.parent / ("prototype_" + row["run"].split("/")[-1]))
    v = view_of(run.mechanism.name, run.config, list(run.mechanism.read_design(row)))
    qk = 0.0  # the extended pose only
    kax = fig.add_axes((0.02 / WC, 0.02 / h, (WC - 0.04) / WC, (hb - 0.10) / h))
    kax.set_xlim(-16, 103); kax.set_ylim(-12, 26); kax.set_aspect("equal"); kax.axis("off")
    # the CAD side view underneath, semi-transparent, registered on the pivots A and B (pixel centres read from
    # finger_side.png: A (1448, 480), B (1975, 492)); similarity transform, the image rotated about A
    from PIL import Image as _Im
    cad = _Im.open(ROOT / "paper" / "figures" / "cad" / "finger_side.png").convert("RGBA")
    uA, vA, uB, vB = 1448.0, 480.0, 1975.0, 492.0
    Bm = np.array([72.415, -0.065]); Am = np.array([44.1, 0.0])
    s_px = np.hypot(uB - uA, vB - vA) / np.linalg.norm(Bm - Am)
    tilt = np.degrees(np.arctan2(vB - vA, uB - uA)) - np.degrees(np.arctan2(-(Bm - Am)[1], (Bm - Am)[0]))
    cad = cad.rotate(tilt, center=(uA, vA), resample=_Im.BICUBIC, fillcolor=(255, 255, 255, 0))
    arr = np.asarray(cad).astype(float) / 255.0
    white = arr[..., :3].min(axis=2) > 0.97
    a = np.where(white, 0.0, 0.55)[..., None]
    arr = a * arr[..., :3] + (1.0 - a)  # on white, so the PDF gets no soft mask
    Wp, Hp = cad.size
    kax.imshow(arr, extent=(Am[0] - uA / s_px, Am[0] + (Wp - uA) / s_px, Am[1] - (Hp - vA) / s_px, Am[1] + vA / s_px),
               zorder=1, interpolation="bilinear")
    # bodies only (servos, palm) from the sketch; the links are drawn here in grey shades
    n0p, n0l, n0c = len(kax.patches), len(kax.lines), len(kax.collections)
    v.draw(kax, qk, hit=set(), style="sketch")
    for p_ in list(kax.patches[n0p:]):  # the CAD shows the bodies; only the skeleton is drawn on top
        p_.remove()
    for ln in list(kax.lines[n0l:]):
        ln.remove()
    for c in list(kax.collections[n0c:]):
        c.remove()
    from matplotlib.patches import Circle as _C
    for p_ in list(kax.patches[n0p:]):
        if isinstance(p_, _C):
            p_.remove()
    four = v.joint
    st = four.sweep([qk])
    A, Ap = four.finger.joint.a.array, four.finger.joint.ap.array
    B, Bp, C = st.b[0], st.bp[0], st.tip[0]
    O = np.zeros(2)
    d0 = v.distal_pad(0.0); dq = v.distal_pad(qk)
    ang = np.arctan2(*(dq[1] - dq[0])[::-1]) - np.arctan2(*(d0[1] - d0[0])[::-1])
    # proximal phalanx (ground link): O-A-A', dark grey
    kax.plot(*np.array([O, A]).T, color="#4a4a48", lw=2.0, solid_capstyle="round", zorder=10)
    kax.plot(*np.array([A, Ap]).T, color="#4a4a48", lw=2.0, solid_capstyle="round", zorder=10)
    kax.plot(*np.array([O, Ap]).T, color="#4a4a48", lw=2.0, solid_capstyle="round", zorder=10)
    kax.add_patch(Polygon([O, A, Ap], fc="#dcdcda", ec="none", alpha=0.55, zorder=9))  # the proximal phalanx, one rigid body
    # distal phalanx: B-B'-C, mid grey
    kax.plot(*np.array([B, C]).T, color="#9a9a98", lw=2.0, solid_capstyle="round", zorder=10)
    kax.plot(*np.array([B, Bp]).T, color="#9a9a98", lw=2.0, solid_capstyle="round", zorder=10)
    kax.plot(*np.array([Bp, C]).T, color="#9a9a98", lw=2.0, solid_capstyle="round", zorder=10)
    kax.add_patch(Polygon([B, Bp, C], fc="#ededeb", ec="none", alpha=0.55, zorder=9))  # the distal phalanx, one rigid body
    # the crossed links A'-B and A-B', black
    for p1, p2 in ((Ap, B), (A, Bp)):
        kax.plot(*np.array([p1, p2]).T, color=INK, lw=1.1, solid_capstyle="round", zorder=12)
    # pivot clearance (dashed) and the pivots: fixed filled, moving open
    pk = run.config.packaging
    for nm_, pt in (("a", A), ("ap", Ap), ("b", B), ("bp", Bp)):
        try:
            r_ = pk.pivot_radius_mm(nm_)
        except Exception:
            r_ = pk.pivot_clearance_diameter_mm / 2.0
        kax.add_patch(_C(pt, r_, fc="none", ec="#6b6b69", lw=0.5, ls=(0, (1.6, 1.2)), zorder=13))
    for pt in (A, Ap):
        kax.plot(*pt, marker="o", ms=3.0, color=INK, zorder=30)
    for pt in (B, Bp):
        kax.plot(*pt, marker="o", ms=3.0, mfc="white", mec=INK, mew=0.7, zorder=30)
    for pt in (O, C):
        kax.plot(*pt, marker="o", ms=2.0, color=INK, zorder=30)
    pts = {"A": A, "A'": Ap, "B": B, "B'": Bp, "C": C, "O": O}
    offs = {"A": (-4, -7), "A'": (-6, 6), "B": (5, -7), "B'": (7, 6), "C": (6, -5), "O": (-6, -8)}
    for nm_, at in pts.items():
        kax.annotate(f"${nm_}$", xy=at, xytext=offs[nm_], textcoords="offset points", fontsize=7, ha="center",
                     va="center", color=INK, zorder=31)
    import matplotlib.patheffects as pe
    halo = [pe.withStroke(linewidth=2.6, foreground="white")]
    arrow = dict(arrowstyle="-|>", lw=0.8, color=INK, mutation_scale=7, shrinkA=0, shrinkB=0, path_effects=halo)
    kax.annotate("", xy=(0, 15), xytext=(0, 0), arrowprops=arrow, zorder=35)
    kax.annotate("", xy=(15, -0.01), xytext=(0, 0), arrowprops=dict(arrow, color="#2b2b2b"), zorder=35)
    kax.text(15.5, -2.0, "$x$", fontsize=6.5, va="top", zorder=36, path_effects=halo)
    kax.text(-1.5, 15.5, "$z$", fontsize=6.5, ha="right", zorder=36)
    kax.text(-3, 8, "$\\mathcal{F}_1$", fontsize=8, ha="right", math_fontfamily="stix", zorder=36)
    fig.text((WC - 0.04) / WC, (hb + 0.06) / h, "extended ($q=0$), over the CAD", fontsize=6.0, color=INK2, ha="right",
             va="top")
    save_figure(fig, DEST / "fig_mechanisms.pdf")
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


def draw_geometry(ax, view, colour, q, faint=True, pins=True):
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
        if not pins:  # no pivot dots or joint discs
            from matplotlib.patches import Circle as _C
            for c in list(ax.collections[n0c:]):
                c.remove()
            for p_ in list(ax.patches[n0p:]):
                if isinstance(p_, _C):
                    p_.remove()
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
                 color=ARCH[key][1], ha="center", va="top", fontweight="bold" if key == "cfl" else "normal", family="Nimbus Sans" if key == "cfl" else "Helvetica Light")
        fig.text((0.04 + i * (gw + 0.02) + gw / 2) / W, (row_a - 0.14 - gh) / h, f"$\\bar C_{{pad}}$ = {100 * c:.0f}%",
                 fontsize=6.2, color=ARCH[key][1], ha="center", va="top")
    fig.text(0.05 / W, (row_b + 0.22) / h, "dark: contact surface holding the cylinder;  light: mounted pad;  "
             "grey: actuator, linkage, rollers, joints;  faint: extended", fontsize=5.4, color=INK2, va="center")
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
    # (b) left: how coverage is measured (problem formulation A.3): at each pose of Q_eval the pads' faces, extended,
    # cross at X; a cylinder inscribed in the V touches both faces at the same distance t from X; the pads hold it
    # together over T = T1 & T2 (dark), shown with the smallest and the largest such cylinder
    label(fig, 0.02, row_b + 0.02, "(b) Coverage evaluation: inscribed cylinder at each $q\\in Q_{eval}$", h)
    vq = views["cfl"]
    for i, qe in enumerate((90.0, 105.0, 120.0)):
        ax = fig.add_axes(((0.06 + i * 1.36) / W, 0.12 / h, 1.30 / W, (row_b - 0.38) / h))
        ax.set_aspect("equal"); ax.axis("off")
        n0p, n0l, n0c = len(ax.patches), len(ax.lines), len(ax.collections)
        vq.draw(ax, qe, hit=set(), style="sketch")
        for p_ in ax.patches[n0p:]:
            p_.set_facecolor("#ececea"); p_.set_edgecolor("#a8a8a6"); p_.set_linewidth(0.4)
        for ln in ax.lines[n0l:]:
            ln.set_color("#b0b0ae"); ln.set_linewidth(min(ln.get_linewidth(), 1.0)); ln.set_markersize(0)
        for c in list(ax.collections[n0c:]):
            c.remove()
        for pad in (vq.proximal_pad(), vq.distal_pad(qe)):
            ax.add_patch(Polygon(pad, fc="#d6e4f5", ec=BLUE, lw=0.5, zorder=20))
        pf = vq.proximal_pad()[[0, 1]]
        df = vq.distal_pad(qe)[[1, 0]]
        got = common_interval(pf, df)
        GREEN = "#1f9a4a"
        if got:
            X, rays, lo, hi = got
            alpha = 0.5 * np.arccos(np.clip(rays[0] @ rays[1], -1, 1))
            bis = (rays[0] + rays[1]) / np.linalg.norm(rays[0] + rays[1])
            for r_ in rays:  # each face's line, dashed, across the panel
                ax.plot(*np.array([X - 120 * r_, X + 120 * r_]).T, color=GREEN, lw=0.5, ls=(0, (3, 2)), zorder=21)
                ax.plot(*np.array([X + lo * r_, X + hi * r_]).T, color=GREEN, lw=2.2, solid_capstyle="butt",
                        zorder=25)
            for tt in np.linspace(max(lo, 0.35 * hi), hi, 4):  # inscribed cylinders over T
                ctr = X + bis * tt / np.cos(alpha)
                ax.add_patch(Circle(ctr, tt * np.tan(alpha), fc="none", ec=GREEN, lw=0.6, zorder=22))
            ax.plot(*X, marker="x", ms=4.0, mew=1.0, color=INK, zorder=26)
            ax.text(X[0] + 2, X[1] + 2, "$X$", fontsize=6.2, zorder=27)
        ax.set_xlim(-42, 96); ax.set_ylim(-82, 20)
        ax.set_title(f"$q={qe:.0f}^\\circ$", fontsize=6.4, pad=1)
    fig.text(2.1 / W, 0.02 / h, "green: $T=T_1\\cap T_2$, where both pads hold a cylinder;  "
             "$\\bar C_{pad}$ averages $2|T|/L_f$ over $Q_{eval}$", fontsize=5.6, color=INK2, ha="center",
             va="bottom")
    label(fig, 4.05, row_b + 0.02, "(c) Preservation, $3\\to5,7,9$ mm", h)
    # (b) right: contact motion kept, 100 (1 - Delta_contact / Delta_fixed), every condition; 0: a fixed axis
    for j, q in enumerate((90.0, 120.0, 150.0)):
        ax = fig.add_axes(((4.32 + j * 0.95) / W, 0.30 / h, 0.82 / W, (row_b - 0.48) / h))
        ax.axhline(0, color=INK, lw=0.8, ls=(0, (1.5, 1.2)), zorder=1)
        for k, key in enumerate(ARCH):
            for hh in (5.0, 7.0, 9.0):
                r = [x for x in s2 if x["architecture"] == key and float(x["required_flexion_deg"]) == q
                     and float(x["to_mm"]) == hh]
                x = hh + (k - 1.5) * 0.32
                fa = 2 * np.sin(np.radians(q) / 2) * (hh - 3.0)
                if r and r[0]["change_at_most_c_pad"] not in ("", "nan"):
                    kept = 100 * (1 - LF * float(r[0]["change_at_most_c_pad"]) / fa)
                    ax.plot(x, max(kept, -38), ls="none", marker=ARCH[key][2], ms=3.0, color=ARCH[key][1],
                            mec="white", mew=0.3, zorder=3, clip_on=False)
                else:
                    ax.plot(x, -40, ls="none", marker="x", ms=2.8, color=ARCH[key][1], mew=0.7, clip_on=False)
        ax.set_xticks([5, 7, 9]); ax.set_xlim(4, 10); ax.set_ylim(-40, 100); ax.set_yticks([0, 50, 100])
        ax.set_title(f"$q_{{req}}={q:.0f}^\\circ$", fontsize=6.4, pad=1)
        if j == 0:
            ax.set_ylabel("contact motion\nkept (%)", labelpad=1, linespacing=1.25)
            ax.text(4.2, 3, "fixed axis", fontsize=5.4, color=INK2, va="bottom")
        else:
            ax.set_yticklabels([])
        if j == 1:
            ax.set_xlabel("redesigned for (mm)", labelpad=1, fontsize=6.2)
        clean(ax)
    handles = [plt.Line2D([], [], ls="none", marker=ARCH[k][2], ms=3.4, color=ARCH[k][1], label=ARCH[k][0])
               for k in ARCH]
    fig.legend(handles=handles, loc="upper right", bbox_to_anchor=(1.0, 1.0), ncol=4, frameon=False, fontsize=6.0,
               handletextpad=0.2, columnspacing=0.8)
    save_figure(fig, DEST / "fig_study12.pdf")
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
    """Study 3: four Drake tasks under O's reused command (the hook pull, 38, is in the text: paper/SSoT.md 6.1). Per task: three frames; R and U's object response minus O's
    over E's pairs (median, quartiles; friction 1.3 circles, 2.0 diamonds); each redesign's friction effect minus O's;
    and under the bottom panel each task's error E (median |bias| over placements) for CoDe, Skeleton and Coverage with point
    and hydroelastic contact. Data: report/study3/data/fig4_sync.json (fig4_sync.py)."""
    from PIL import Image
    from scipy.stats import rankdata, spearmanr
    d = json.load(open(S3 / "fig4_sync.json"))
    DV = d["deviation"]
    # (title, task key, image, x label, response label, effect label)
    cols = (("Power-grasp hold", "pull", "power_grasp_hold_iso.png", "cylinder displacement (mm)",
             "load deviation (N)", "effect bias (N)"),
            ("Grasp acquisition", "acquire", "grasp_acquisition.png", "time (s)", "drawn-in deviation (mm)",
             "effect bias (mm)"),
            ("Fingertip dial", "dial", "fingertip_disk.png", "time (s)", "rotation deviation (deg)",
             "effect bias (deg)"),
            ("Box pivot", "pivot", "fingertip_pivot.png", "time (s)", "tilt deviation (deg)",
             "effect bias (deg)"))
    HC = {"O": INK, "R": BLUE, "U": AMBER, "S": TEAL}
    RED = tuple(hh for hh in ("R", "U", "S") if f"{hh}_1.3" in next(iter(DV.values())))
    RB = 0.0  # the learned policies are Fig. 5 (fig_rl), after the v35 review found Fig. 4 overloaded
    B = 0.24 + RB  # room for the two motion-change rows under each task's E rows, and the band below
    LG = 0.20  # the legend's row between the renders and the trajectories
    D = 0.12  # the bottom legend's room, now unused
    NB = 0.85  # the bias row dropped to fit 5 pages (2026-11-27): E under each task carries it
    h = 3.40 + B + LG - D - NB
    fig = plt.figure(figsize=(W, h))
    left, gapc = 0.05, 0.06
    cw = (W - left - 0.04 - gapc * (len(cols) - 1)) / len(cols)
    for i, (name, key, img, xl, yl, el) in enumerate(cols):
        x0 = left + i * (cw + gapc)
        fig.text((x0 + cw / 2) / W, (h - 0.02) / h, name, fontsize=7, fontweight="bold", ha="center", va="top")
        ia = fig.add_axes((x0 / W, (h - 0.70) / h, cw / W, 0.52 / h))
        strip = np.asarray(Image.open(ROOT / "report" / "study3" / "figures" / img).convert("RGB"))
        wq = strip.shape[1] // 3
        rr = np.nonzero((strip.min(axis=2) < 240).any(axis=1))[0]
        frames = []
        for fr in range(3):  # the three time steps, each cropped to its ink, at one height
            sub = strip[rr.min():rr.max() + 1, fr * wq:(fr + 1) * wq]
            cc = np.nonzero((sub.min(axis=2) < 240).any(axis=0))[0]
            frames.append(sub[:, max(cc.min() - 6, 0):cc.max() + 7])
        gap = np.full((frames[0].shape[0], 12, 3), 255, np.uint8)
        ia.imshow(np.concatenate([frames[0], gap, frames[1], gap, frames[2]], axis=1))
        ia.axis("off"); ia.set_aspect("equal", anchor="C")
        dv = DV[key]
        x = np.asarray(dv["x"])
        # the friction effect each hand measures (same hand, 2.0 minus 1.3); O's is the effect of friction alone, the
        # gap to it (shaded) is what the changed finger adds; end labels: share of O's effect
        # the trajectories as deviation from O: R and U minus O on each placement, at each friction (0: O's own)
        ra = fig.add_axes(((x0 + 0.30) / W, (h - 1.52 - LG) / h, (cw - 0.36) / W, 0.74 / h))
        ra.axhline(0, color=INK, lw=0.6, zorder=2)
        step = max(len(x) // 6, 1)
        for hi, hh in enumerate(RED):
            for mi, (mu, mk_) in enumerate((("1.3", "o"), ("2", "D"))):
                bd = dv[f"{hh}_{mu}"]
                ra.fill_between(x, bd["q25"], bd["q75"], color=HC[hh], alpha=0.08, lw=0, zorder=1)
                ra.plot(x, bd["median"], color=HC[hh], lw=0.6, zorder=3, marker=mk_, ms=2.4 if mk_ == "o" else 2.1,
                        mfc="white", mew=0.5, markevery=((2 * hi + mi) * step // 6, step))
        ra.set_ylabel(yl, labelpad=1, fontsize=5.6)
        ra.set_xlabel(xl, labelpad=1, fontsize=6.3)
        clean(ra)
        # the task's error E (median |bias| over placements) in both contact models; "n/a": no valid pair
        pt = d["tasks"][key]["hands"]
        if key in ("dial", "pivot"):  # the randomized tasks: point on the 32 draws the hydro rerun used
            f32 = {"dial": "drake_e29z_point_first32.json", "pivot": "drake_e35_point_first32.json"}[key]
            pt = d["first32"][key] if "first32" in d else {r["hand"]: r for r in json.load(open(S3 / f32))["rows"]}
        for j, (model, src) in enumerate((("point", pt), ("hydro", d["hydro"][key]))):
            yy = (0.40 + B - D - 0.115 * j) / h
            xm = (x0 + 0.30 + (cw - 0.36) / 2 - 0.30) / W  # left of centre: room for the rho column
            fig.text(xm - 0.22 / W, yy, f"E {model}", fontsize=5.6, ha="right", va="center", color=INK2)
            for dx, hh in ((-0.17, "R"), (0.10, "S"), (0.37, "U")):
                e = src.get(hh)
                fig.text(xm + (dx - 0.02) / W, yy, "n/a" if e is None else f"{100 * e['E']:.0f}%", fontsize=5.8,
                         ha="left", va="center", color=HC[hh], fontweight="bold")
        # how much each morphology changes the contact-surface and the skeletal motion (mm, 7 mm; one hand each, so
        # the same under every task), under the E rows and aligned with them
        hd = d["tasks"]["pull"]["hands"]
        hdall = d["tasks"][key]["hands"]  # every hand's Delta_contact and Delta_skel
        for j, (lab, k_) in enumerate((("$\\Delta_\\mathrm{contact}$", "d_contact"), ("$\\Delta_\\mathrm{skel}$", "d_skel"))):
            yy = (0.40 + B - D - 0.115 * (j + 2) - 0.03) / h
            fig.text(xm - 0.22 / W, yy, lab, fontsize=5.6, ha="right", va="center", color=INK2)
            for dx, hh in ((-0.17, "R"), (0.10, "S"), (0.37, "U")):
                fig.text(xm + (dx - 0.02) / W, yy, f"{hd[hh][k_]:.1f}", fontsize=5.8, ha="left", va="center",
                         color=HC[hh])
            # Spearman rho of E with this quantity over the correlation's hands (Skeleton outside it), both contact
            # models in one value: each hand's E ranked within each model, the two ranks averaged (E's size differs
            # between the models, so pooling the raw errors would mix the models' scales into the ranking)
            pt_all = pt
            if key in ("dial", "pivot"):  # every hand on the 32 draws (fig4_sync's first32 holds R and U only)
                f32 = {"dial": "drake_e29z_point_first32.json", "pivot": "drake_e35_point_first32.json"}[key]
                pt_all = {r["hand"]: r for r in json.load(open(S3 / f32))["rows"]}
            hy = d["hydro"][key]
            ok = lambda src, hh: hh != "S" and src.get(hh) and src[hh].get("E") is not None and hh in hdall
            hs = [hh for hh in hdall if ok(pt_all, hh) and ok(hy, hh)]
            mrank = (rankdata([pt_all[hh]["E"] for hh in hs]) + rankdata([hy[hh]["E"] for hh in hs])) / 2
            rho = spearmanr(mrank, [hdall[hh][k_] for hh in hs]).correlation
            fig.text(xm + 0.50 / W, yy, f"$\\rho$ {rho:.2f}".replace("-", "\u2212"),
                     fontsize=5.4, ha="left", va="center", color=INK2)
    handles = [plt.Line2D([], [], color=INK, lw=0.9, label="0: as O (original)"),
               plt.Line2D([], [], color=BLUE, lw=1.1, label="CoDe"),
               plt.Line2D([], [], color=TEAL, lw=1.1, label="Skeleton"),
               plt.Line2D([], [], color=AMBER, lw=1.1, label="Coverage"),
               plt.Line2D([], [], color="#777", lw=1.0, marker="o", ms=3.2, mfc="white", mew=0.7,
                          label="pad friction 1.3"),
               plt.Line2D([], [], color="#777", lw=1.0, marker="D", ms=2.9, mfc="white", mew=0.7,
                          label="pad friction 2.0"),
               plt.Rectangle((0, 0), 1, 1, fc="#999", alpha=0.3, label="quartiles over placements")]
    fig.legend(handles=handles, loc="center", bbox_to_anchor=(0.5, (h - 0.74 - LG / 2) / h), ncol=7, frameon=False, fontsize=6.0,
               handletextpad=0.4, columnspacing=1.2, handlelength=1.8)
    save_figure(fig, DEST / "fig_study3.pdf")
    plt.close(fig)
    print("fig4", h)


# ------------------------------------------------------------------------------------------- Fig. 5 learned policies
def fig_rl():
    """Fig. 5: policies learned on O run unchanged on the 7 mm redesigns; the share of O's performance each keeps,
    per training seed (solid 20, hatched 21), with each task's governing quantity under its bars. Data:
    report/study4/data/rl_transfer.json (rl_sync.py) and fig4_sync.json (Delta_contact, Delta_skel)."""
    rl = json.load(open(ROOT / "report" / "study4" / "data" / "rl_transfer.json"))["tasks"]
    d = json.load(open(S3 / "fig4_sync.json"))
    hd7 = d["tasks"]["pull"]["hands"]
    HC = {"R": BLUE, "U": AMBER, "S": TEAL}
    gov = {"walk": ("$\\Delta_\\mathrm{contact}$", {h: hd7[h]["d_contact"] for h in ("R", "S", "U")}),
           "relay": ("$\\Delta_\\mathrm{skel}$", {h: hd7[h]["d_skel"] for h in ("R", "S", "U")}),
           "cube": ("$\\Delta t_p$", {"R": 4.0, "S": 4.0, "U": 4.0})}
    cols = (("walk", "Finger walk\n(on the pads)"), ("relay", "Relay dial\n(fingertip end)"),
            ("cube", "Cube rotation\n(whole hand)"))
    MAP = (("R", "R", "CoDe"), ("SK", "S", "Skel."), ("U", "U", "Cov."))
    W1, h = 3.45, 1.55
    fig = plt.figure(figsize=(W1, h))
    aw = 0.80
    for i, (k, title) in enumerate(cols):
        x0 = 0.48 + i * (aw + 0.30)
        ax = fig.add_axes((x0 / W1, 0.42 / h, aw / W1, 0.80 / h))
        for j, (hh, hk, nm) in enumerate(MAP):
            for t_, (v, hatch) in enumerate(zip(rl[k]["ratio"][hh], ("", "////"))):
                xx = j + (t_ - 0.5) * 0.36
                ax.bar(xx, v, width=0.34, color=HC[hk], alpha=0.9 if t_ == 0 else 0.45, lw=0, hatch=hatch,
                       edgecolor="white", zorder=2)
        ax.axhline(1.0, color=INK, lw=0.6, ls=(0, (2, 2)), zorder=1)
        ax.set_ylim(0, 1.1); ax.set_yticks([0, 0.5, 1.0])
        ax.set_yticklabels(["0", "50%", "100%"] if i == 0 else [], fontsize=5.6)
        ax.set_xticks(range(3)); ax.set_xticklabels([nm for _, _, nm in MAP], fontsize=5.6)
        for lab_, (_, hk, _) in zip(ax.get_xticklabels(), MAP):
            lab_.set_color(HC[hk])
        ax.set_xlim(-0.6, 2.6)
        ax.set_title(title, fontsize=6.0, pad=2, linespacing=1.0)
        clean(ax)
        gl, gv = gov[k]
        ax.text(-0.62, -0.36, gl, fontsize=5.4, ha="right", va="center", color=INK2, transform=ax.transData)
        for j, (_, hk, _) in enumerate(MAP):
            ax.text(j, -0.36, f"{gv[hk]:.1f}", fontsize=5.6, ha="center", va="center", color=HC[hk],
                    transform=ax.transData)
        if i == 0:
            ax.legend(handles=[plt.Rectangle((0, 0), 1, 1, fc="#888", label="seed 20"),
                               plt.Rectangle((0, 0), 1, 1, fc="#888", alpha=0.45, hatch="////", ec="white",
                                             label="seed 21")],
                      loc="upper right", bbox_to_anchor=(1.02, 0.92), frameon=False, fontsize=5.4, handlelength=1.2,
                      title="training", title_fontsize=5.4)
    fig.text(0.02 / W1, (0.42 + 0.40) / h, "share of O's performance kept", rotation=90, fontsize=5.8, ha="left",
             va="center")
    save_figure(fig, DEST / "fig_rl.pdf")
    plt.close(fig)
    print("fig_rl", h)


# ------------------------------------------------------------------------------------------- command protocols
def fig_protocols():
    """The bias in the measured pad effect (pull-out, friction 1.3 -> 2.0, 7 mm) of CoDe and Coverage under each
    command the redesigns reuse, ordered by the sensing it needs: median and quartiles over placements, + overstating
    O's effect. Data: report/study3/data/table_signed.json (paper/figures/table_signed.py)."""
    pr = json.load(open(S3 / "table_signed.json"))["protocols"]
    cols = (("setpoint", "joint\nsetpoint", "none"), ("contact setpoint", "contact\nsetpoint$^{*}$", "none"),
            ("torque", "joint\ntorque", "joint torque"), ("touch", "close at\ntouch", "touch"),
            ("force", "contact\nforce", "pad forces"))
    W1, h = 3.45, 1.85
    fig = plt.figure(figsize=(W1, h))
    ax = fig.add_axes((0.60 / W1, 0.62 / h, (W1 - 0.68) / W1, (h - 0.72) / h))
    ax.axhline(0, color=INK, lw=0.6, zorder=1)
    for i, (key, lab, sense) in enumerate(cols):
        r, u = pr[key]["R"], pr[key]["U"]
        ax.plot([i, i], [100 * r["bias_median"], 100 * u["bias_median"]], color="#b5b5b2", lw=2.2,
                solid_capstyle="round", zorder=2)
        for d, c, dx in ((r, BLUE, -0.09), (u, AMBER, 0.09)):
            ax.plot([i + dx] * 2, [100 * d["bias_q25"], 100 * d["bias_q75"]], color=c, lw=0.7, zorder=3)
            ax.plot(i + dx, 100 * d["bias_median"], marker="o", ms=4.2, color=c, mec="white", mew=0.5, zorder=4)
        ax.text(i, -0.30, sense, transform=ax.get_xaxis_transform(), fontsize=5.4, color=INK2, ha="center",
                va="top")
    ax.set_xticks(range(len(cols))); ax.set_xticklabels([c[1] for c in cols], fontsize=5.8, linespacing=1.0)
    ax.set_xlim(-0.5, len(cols) - 0.5)
    ax.set_ylabel("bias in pad effect (%)", labelpad=1, fontsize=6.0)
    ax.tick_params(axis="x", length=0, pad=2)
    clean(ax)
    ax.text(-0.62, -0.30, "sensing:", transform=ax.get_xaxis_transform(), fontsize=5.4, color=INK2, ha="right",
            va="top")
    handles = [plt.Line2D([], [], ls="none", marker="o", ms=4, color=BLUE, label="CoDe"),
               plt.Line2D([], [], ls="none", marker="o", ms=4, color=AMBER, label="Coverage"),
               plt.Line2D([], [], color="#b5b5b2", lw=2.2, label="gap")]
    ax.legend(handles=handles, loc="upper right", frameon=False, fontsize=5.8, handletextpad=0.3, borderaxespad=0.1)
    save_figure(fig, DEST / "fig_protocols.pdf")
    plt.close(fig)
    print("fig_protocols", h)


# ----------------------------------------------------------------------------------------------------------- Table I
def table():
    """Table I (Study 3B and the command protocols): the signed bias in the measured pad effect of CoDe and
    Coverage (pull-out, 7 mm), median over placements and load directions of (D(H) - D(O)) / D(O), + overstating
    O's effect, in two sections: the pad change (friction, stiffness and the soft thick pad in Isaac Lab, point
    contact; the soft thick pad with hydroelastic contact and the pad shape in Drake, each shape placed by O's own
    contact, experiment 34; all at the reused setpoint) and the reused command (friction change). The Drake point and
    hydroelastic friction rows repeat Fig. 4's hold column and are left out. Data: report/study3/data/table_signed.json
    (paper/figures/table_signed.py; median |bias| reproduces each setting's E)."""
    t = json.load(open(S3 / "table_signed.json"))
    st, pr = t["settings"], t["protocols"]
    # one wide table: every simulator, contact model, pad change and command side by side (user, 2026-11-27: the
    # table says how many simulations agree); rows Coverage and CoDe (bold, at the bottom); title-only caption
    groups = (("MJWarp, point contact", (("friction", st["friction, Isaac point"]),
                                            ("stiffness", st["stiffness, Isaac point"]),
                                            ("friction, compliant pad", st["friction softer pad, Isaac point"]),
                                            ("convex pad", st["crowned, Isaac point"]))),
              ("Drake, point contact", (("friction", st["friction, Drake point"]),
                                        ("convex pad", st["crowned, Drake point"]),
                                        ("contact setpoint", pr["contact setpoint"]))),
              ("Drake, hydroelastic", (("friction", st["friction, Drake hydro"]),
                                       ("friction, compliant pad", st["friction softer pad, Drake hydro"]))),
              ("MJWarp, other commands", (("joint torque", pr["torque"]),
                                             ("sensed touch", pr["touch"]),
                                             ("contact force", pr["force"]))))
    cols = [c for _, cs in groups for c in cs]
    n = len(cols)
    head1, rules, pos = [], [], 2
    for g, cs in groups:
        head1.append(f"\\multicolumn{{{len(cs)}}}{{c}}{{{g}}}")
        rules.append(f"\\cmidrule(lr){{{pos}-{pos + len(cs) - 1}}}")
        pos += len(cs)

    def stack(name):  # two-word headers on two lines
        w = name.split(" ", 1)
        return name if len(w) == 1 else f"\\begin{{tabular}}[b]{{@{{}}c@{{}}}}{w[0]}\\\\{w[1]}\\end{{tabular}}"

    def cell(d, hh):
        if hh not in d:
            return "--"
        v = 100 * d[hh]["bias_median"]
        num = f"{v:+.0f}" if round(v) != 0 else "0"
        return f"$\\mathbf{{{num}}}$\\textbf{{\\%}}" if hh == "R" else f"${num}\\%$"
    lines = [r"\begin{table*}[t]", r"  \centering", r"  \footnotesize", r"  \setlength{\tabcolsep}{3.2pt}",
             r"  \caption{Signed error in the measured pad effect across simulators, contact models and commands}",
             r"  \label{tab:robust}", f"  \\begin{{tabular}}{{l{'r' * n}}}", r"    \toprule",
             "    & " + " & ".join(head1) + r" \\", "    " + " ".join(rules),
             "    & " + " & ".join(stack(name) for name, _ in cols) + r" \\", r"    \midrule",
             "    Coverage & " + " & ".join(cell(d, "U") for _, d in cols) + r" \\",
             r"    \ourmethod{} & " + " & ".join(cell(d, "R") for _, d in cols) + r" \\",
             r"    \bottomrule", r"  \end{tabular}", r"\end{table*}", ""]
    (MS / "sections" / "table_robust.tex").write_text("\n".join(lines))
    print("\n".join(lines))


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    for name, fn in (("fig2", fig2), ("fig3", fig3), ("fig4", fig4), ("rl", fig_rl), ("protocols", fig_protocols), ("table", table)):
        if which in (name, "all"):
            fn()
