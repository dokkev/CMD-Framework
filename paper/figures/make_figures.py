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
    axes[0].set_ylabel(r"$\Delta_{\mathrm{contact}}$ (mm)")
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
    ax.set_xlabel(r"flexion $q$ ($^\circ$)"); ax.set_ylabel(r"$\|\Delta\mathbf{c}(q)\|$ (mm)")
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


def panel(ax, t, z, title, half=40.7, q=np.radians(120), label_mirror=False, k=1.0):
    """k scales fonts and line widths (k < 1 for the single-column version)."""
    P = np.array([0.0, -z])
    tau = t - z
    s = tau * np.tan(q / 2)
    ell = half - s
    # proximal: link on z=0, pad hanging to depth t, joint-side end at x=-s
    ax.plot([-half - 2, 0], [0, 0], color=RED, lw=2.4*k, solid_capstyle="round", zorder=3)
    ax.add_patch(Polygon([[-s - ell, 0], [-s, 0], [-s, -t], [-s - ell, -t]], closed=True, fc=PAD_R, ec=RED, lw=0.8*k, zorder=2))
    # distal (reference pose mirrored about x=0), then flexed about the pole
    link = rot(q, [[0, 0], [half + 2, 0]], P)
    pad = rot(q, [[s, 0], [s + ell, 0], [s + ell, -t], [s, -t]], P)
    ax.plot(*link.T, color=BLUE, lw=2.4*k, solid_capstyle="round", zorder=3)
    ax.add_patch(Polygon(pad, closed=True, fc=PAD_B, ec=BLUE, lw=0.8*k, zorder=2))
    # mirror line through the pole, normal (cos q/2, -sin q/2)
    d = np.array([np.sin(q / 2), np.cos(q / 2)])
    m = np.array([P - 30 * d, P + 9 * d])
    ax.plot(*m.T, color=INK2, lw=0.6*k, ls=(0, (4, 2)), zorder=1)
    ax.add_patch(Circle(P, 1.3, fc="white", ec=INK, lw=1.0*k, zorder=5))
    ax.annotate("pole", xy=P + [1.2, 0.6], xytext=P + [7, 5.5], fontsize=7*k, color=INK,
                arrowprops=dict(arrowstyle="-", color=INK, lw=0.5*k))
    # gap annotation
    y = 4.0
    ax.annotate("", xy=(-s, y), xytext=(0, y), arrowprops=dict(arrowstyle="<->", color=INK, lw=0.7*k, shrinkA=0, shrinkB=0))
    ax.plot([-s, -s], [-t - 0.3, y + 1], color=INK2, lw=0.4*k, ls=(0, (1, 1)))
    ax.plot([0, 0], [-z, y + 1], color=INK2, lw=0.4*k, ls=(0, (1, 1)))
    ax.text(-s / 2, y + 1.3, f"{s:.1f} mm", ha="center", va="bottom", fontsize=7*k)
    ax.text(-s - ell / 2, -t / 2, f"pad {ell:.0f} mm", ha="center", va="center", fontsize=6.5*k, color=INK)
    if label_mirror:
        ax.text(*(P - 27 * d + [3.4, -4.6]), "mirror line", fontsize=6.3*k, color=INK2, rotation=np.degrees(np.arctan2(d[1], d[0])),
                ha="center", va="bottom")
    ax.set_title(title, fontsize=7.5*k, pad=2, loc="left")
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_xlim(-half - 3, 22); ax.set_ylim(-44, 11)


def fig_concept():
    fig, axes = plt.subplots(1, 3, figsize=(3.45, 1.32))
    k = 0.72
    panel(axes[0], 5, 0, "(a) fixed axis, 5 mm", label_mirror=True, k=k)
    panel(axes[1], 9, 0, "(b) fixed axis, 9 mm", k=k)
    panel(axes[2], 9, 5, "(c) pole 5 mm lower, 9 mm", k=k)
    fig.subplots_adjust(wspace=0.02)
    fig.savefig(DEST / "fig_pole_concept.png")
    plt.close(fig)


# ---------------------------------------------------------------- Study 3 and Figure 1
TWO_PAD = ROOT / "sim" / "isaaclab" / "logs" / "experimental" / "two_pad"
HANDS3 = ROOT / "design" / "output" / "sim" / "study3" / "hands"
GREY, ORANGE, BLUE = "#8a8984", ARCH["2R-remote"][1], ARCH["CFL"][1]
THICK = {5: "#93bde9", 7: "#2a78d6", 9: "#0d3c78"}  # one hue, light to dark: pad thickness
FORCE_INK, FORCE_MM_PER_N = "#b8322a", 2.0  # contact-force arrows in Fig. 1: mm of arrow per newton


def two_pad_state(hand, mu=1.3, cond="A", rho=15.0, q0=105.0):
    import glob
    import json
    found = None
    for f in sorted(glob.glob(str(TWO_PAD / f"tp_{hand}_mu{mu:g}_{cond}_r{rho:g}_*.json"))):
        d = json.loads(Path(f).read_text())
        if d.get("impratio") == 1000.0:
            found = d
    return next(e for e in found["envs"] if e["q0_deg"] == q0 and e["direction"] == "-x")["pre"]


def finger_geometry(stem, q_deg):
    import yaml
    sys.path.insert(0, str(ROOT / "design"))
    from core.morphology.code_hand.model import four_bar
    from core.morphology.code_hand.parameters import FingerParameters
    d = yaml.safe_load((HANDS3 / f"cfl_q120_h3to7_{stem}.yaml").read_text())
    fp = FingerParameters(**{**FingerParameters().__dict__, **d["finger"]})
    pins = {k: np.asarray(getattr(four_bar(fp, q_deg), k), float) for k in ("a", "ap", "b", "bp", "c")}
    return pins, d["pads"]


def draw_finger(ax, stem, hand, rho, ref_centre=None):
    """Side view at the settled grasp, palm up (the linkage plane flipped so the object sits above the pads)."""
    st = two_pad_state(hand)
    q = float(np.mean(st["q_finger_deg"]))
    pins, pads = finger_geometry(stem, q)
    m, h = pads["mount_offset_mm"], pads["thickness_mm"]
    d = m + h
    qr = np.radians(q)
    u, v = np.array([np.cos(qr), -np.sin(qr)]), np.array([np.sin(qr), np.cos(qr)])
    F = np.array([1.0, -1.0])  # flip: palm up
    o = np.zeros(2)
    a, ap, b, bp, c = (pins[k] for k in ("a", "ap", "b", "bp", "c"))
    pad_fc, pad_ec = "#f2c46d", "#a26a00"
    s0, s1 = pads["proximal_start_mm"], pads["proximal_end_mm"]
    ax.add_patch(Polygon([np.array(pt) * F for pt in ((s0, -m), (s1, -m), (s1, -d), (s0, -d))], fc=pad_fc, ec=pad_ec,
                         lw=0.6, zorder=3))
    t0, t1 = pads["distal_start_mm"], pads["distal_end_mm"]
    ax.add_patch(Polygon([pt * F for pt in (b + t0 * u - m * v, b + t1 * u - m * v, b + t1 * u - d * v, b + t0 * u - d * v)],
                         fc=pad_fc, ec=pad_ec, lw=0.6, zorder=3))
    for p0, p1, lw in ((o, a, 2.0), (a, ap, 2.0), (ap, b, 1.0), (a, bp, 1.0), (b, bp, 2.0), (b, c, 2.0), (bp, c, 2.0)):
        ax.plot(*np.c_[p0 * F, p1 * F], color=INK if lw > 1.5 else INK2, lw=lw, solid_capstyle="round", zorder=4)
    for p in (o, a, ap, b, bp):
        ax.add_patch(Circle(p * F, 1.2, fc="white", ec=INK, lw=0.6, zorder=5))
    centre = (np.asarray(st["centre_face_mm"]) + np.array([0.0, -d])) * F
    if ref_centre is not None:
        ax.add_patch(Circle(ref_centre, rho, fill=False, ec=GREY, lw=0.7, ls=(0, (2.5, 1.5)), zorder=2))
    ax.add_patch(Circle(centre, rho, fc="#e8eef6", ec="#4a6f99", lw=0.8, zorder=2))
    for pad in st["pads"]:  # the middle finger's contact forces on the cylinder, to scale
        if pad["pad"] not in ("middle_proximal", "middle_distal"):
            continue
        at = (np.asarray(pad["point_face_mm"]) + np.array([0.0, -d])) * F
        force = np.asarray(pad["normal_face"]) * F * FORCE_MM_PER_N
        ax.annotate("", xy=at + force, xytext=at, zorder=6,
                    arrowprops=dict(arrowstyle="-|>", color=FORCE_INK, lw=1.3, mutation_scale=7, shrinkA=0,
                                    shrinkB=0))
    ax.set_aspect("equal")
    ax.set_xlim(-4, 64)
    ax.set_ylim(-14, 52)
    ax.axis("off")
    return centre, st


CAD = Path(__file__).resolve().parent / "cad"


def fig_overview():
    """Fig. 1: the CoDe hand (CAD, C1) and why a redesign keeps the contact Jacobian (simulation, C2)."""
    import json
    fig = plt.figure(figsize=(W, 1.68))
    outer = fig.add_gridspec(1, 3, width_ratios=[1.3, 1.8, 1.3], wspace=0.24)
    ax = fig.add_subplot(outer[0])
    img = plt.imread(CAD / "code_hand_iso.png")
    ax.imshow(img)
    ax.axis("off")
    cad_ax = ax
    callouts = [  # (label, point in the render [px], text position [axes fraction])
        ("PIP servo,\noff the joint", (900, 1040), (-0.02, 0.38)),
        ("crossed four-bar\nPIP joint", (880, 1420), (-0.02, 0.10)),
        ("proximal pad", (690, 1330), (-0.02, 0.24)),
        ("distal pad", (1210, 1620), (0.80, -0.02)),
    ]
    for label, xy, txt in callouts:
        ax.annotate(label, xy=xy, xycoords="data", xytext=txt, textcoords="axes fraction", fontsize=6.2,
                    color=INK, ha="right" if txt[0] < 0.5 else "left", va="center", linespacing=1.0,
                    arrowprops=dict(arrowstyle="-", color=INK2, lw=0.5, shrinkA=1, shrinkB=0))
    mid = outer[1].subgridspec(1, 2, wspace=0.0)
    axes = [fig.add_subplot(mid[0, i]) for i in range(2)]
    rho = 15.0
    probe = fig.add_axes([0, 0, 0.01, 0.01])
    o_centre, o_st = draw_finger(probe, "h0", "O", rho)
    probe.remove()
    _, u_st = draw_finger(axes[0], "hu", "U", rho, ref_centre=o_centre)
    _, r_st = draw_finger(axes[1], "hr", "R", rho, ref_centre=o_centre)
    titles = [("(b) U: coverage only", "skeleton change 3.4 mm\ncontact change 3.9 mm"),
              ("(c) R: contact kept", "skeleton change 7.7 mm\ncontact change 0.9 mm")]
    panel_titles = []
    for ax_, (t, sub), st in zip(axes, titles, (u_st, r_st)):
        panel_titles.append((ax_, t))
        ratio = st["n_distal_total"] / o_st["n_distal_total"]
        ax_.text(0.02, -0.01, f"{sub}\ndistal force {ratio:.1f}$\\times$ $O$'s", transform=ax_.transAxes, fontsize=6.4,
                 color=INK2, va="top", linespacing=1.2)
    ax = fig.add_subplot(outer[2])
    d = json.loads((TWO_PAD / "conclusion_study3.json").read_text())["hands"]
    pairs = {5: ("h3to5_hr", "h3to5_hu"), 7: ("R", "U"), 9: ("h3to9_hr", "h3to9_hu")}
    xs = np.arange(3)
    width = 0.26
    true = [d[r]["true_change_pct"] for r, _ in pairs.values()]
    meas_r = [d[r]["measured_change_pct"] for r, _ in pairs.values()]
    meas_u = [d[u]["measured_change_pct"] for _, u in pairs.values()]
    for off, vals, color, label in ((-width, true, GREY, "true"), (0.0, meas_r, BLUE, "on R"),
                                    (width, meas_u, ORANGE, "on U")):
        ax.bar(xs + off, vals, width * 0.92, color=color, label=label, zorder=3)
    ax.axhline(0.0, color=INK2, lw=0.6, zorder=2)
    ax.set_xticks(xs, [f"{t} mm" for t in pairs])
    ax.set_ylabel("change in holding load (%)", labelpad=1)
    ax.set_ylim(-80, 85)
    ax.tick_params(axis="x", length=0)
    ax.legend(loc="upper left", ncol=3, fontsize=6.0, frameon=False, handlelength=1.0, columnspacing=0.8,
              borderaxespad=0.1)
    ax.text(xs[1] + width, meas_u[1] + 3, "reversed", fontsize=6.0, color=ORANGE, ha="center", va="bottom")
    panel_titles.append((ax, "(d) a softer, thicker pad"))
    fig.canvas.draw()
    top = cad_ax.get_position().y1 + 0.02
    from matplotlib.transforms import blended_transform_factory
    for ax_, t in [(cad_ax, "(a) CoDe hand: room for thick pads"), *panel_titles]:
        ax_.text(0.0, top, t, transform=blended_transform_factory(ax_.transAxes, fig.transFigure), fontsize=7.6,
                 fontweight="bold", va="bottom")
    fig.savefig(DEST / "fig_overview.png")
    plt.close(fig)


def fig_study3():
    """Study 3: the error against Delta_contact at a reused impedance setpoint, with the design tolerance."""
    import json
    fig, ax = plt.subplots(figsize=(3.45, 1.25))
    grid = json.loads((TWO_PAD / "dose_A-grid_expanded.json").read_text())
    fr = json.loads((TWO_PAD / "dose_A_all.json").read_text())
    comp = json.loads((TWO_PAD / "comp.json").read_text())
    for rec in grid["hands"].values():
        ax.plot(rec["D_c_mm"], 100 * rec["E_rel_median"], marker="o", ms=2.6, color="#9a9a9a", mfc="none", mew=0.6,
                ls="none", zorder=2)
    for rec in fr["hands"].values():
        c = THICK[int(rec["thickness_mm"])]
        ax.plot(rec["D_c_mm"], 100 * rec["E_rel_median"], marker="o", ms=3.4, color=c, mfc=c, mew=0.8, ls="none", zorder=3)
    for hand, rec in comp["hands"].items():
        c = THICK[7 if not hand.startswith("h3to") else int(hand[4])]
        ax.plot(rec["D_c_mm"], 100 * rec["E_median"], marker="s", ms=3.0, color=c, mfc="white", mew=0.8, ls="none",
                zorder=3)
    ax.axhline(20, color="#555555", lw=0.6, ls=(0, (3, 2)), zorder=1)
    ax.text(5.45, 22, "20%", fontsize=5.6, ha="right", va="bottom", color="#555555")
    for x, t in ((0.35, 5), (0.86, 7), (2.32, 9)):  # CFL redesigns of Study 2 (q_req 120 deg)
        ax.axvline(x, color=THICK[t], lw=0.7, ls=":", zorder=1)
        ax.text(x + 0.05, 205, f"CFL {t}", fontsize=5.4, color=THICK[t], va="top", rotation=90)
    handles = [plt.Line2D([], [], marker="o", ls="none", color=c, mfc=c, ms=3.2, label=f"3→{t} mm") for t, c in THICK.items()]
    handles.append(plt.Line2D([], [], marker="o", ls="none", color="#9a9a9a", mfc="none", mew=0.6, ms=2.8, label="grid (7 mm)"))
    ax.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, 0.98), ncol=4, fontsize=5.6, frameon=False,
              handletextpad=0.1, columnspacing=1.0, borderaxespad=0.0)
    ax.set_xlabel(r"$\Delta_{\mathrm{contact}}$ (mm)")
    ax.set_ylabel("error $E$ (%)")
    ax.set_xlim(0, 5.5)
    ax.set_ylim(-5, 210)
    fig.savefig(DEST / "fig_study3.png")
    plt.close(fig)

if __name__ == "__main__":
    fig_concept()
    fig_study1()
    fig_study2()
    fig_overview()
    fig_study3()
    print("written to", DEST)
