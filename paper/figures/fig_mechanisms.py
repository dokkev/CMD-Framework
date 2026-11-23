"""Fig. 2 of the RSS 2027 extended abstract: the four PIP joints compared, and how far each one's pole sits from
the contact face (PNG only).

    conda run -n cmd python paper/figures/fig_mechanisms.py [dest_dir]

(a-d) Sketches at 120 deg of flexion, one scale, in the style of Study 1's design figures
(design/scripts/study/study1_thickness_coverage/figure_designs.py), with pivots dotted and no palm hatching.
2R-DD, 2R-remote, RCJ: each one's Study 1 design for q_req = 120 deg and 7 mm pads. CFL: the CoDe prototype's
finger (design/output/linkage/config/paper_single_objective/portfolio_001, balanced), drawn qualitatively: pads
at 7 mm like the others, the distal pad laid on its link, and A'' placed at A so the proximal triangle closes
there.
(e) The finite poles P(120 deg) each joint type can reach over its designs, in the proximal phalanx's frame
(origin: its end on its axis), from Study 0's analysis (no optimization run): a pin is the phalanx end (a point);
equal rollers, the proximal one centred at the phalanx end, put P = r - i r tan(q/4) (Study 0, Eq. 14) on one
segment for r in 3-8 mm; a crossed four-bar's P = b / (1 - e^{iq}) (Lemma 1), with b from the closure of its links
(feasible_space.md, F1), covers an area: shaded, the region holding 90% of the four-bars whose pivots lie in the
phalanges (A, B on the link line within 15 mm of the joint, A', B' up to 14 mm dorsal) that close through 120 deg;
Gaussian kernel density.
Drawn at print size (full text width, 7.16 in).
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
DESIGN = ROOT / "design"
for path in (DESIGN, DESIGN / "scripts" / "study", DESIGN / "scripts" / "study" / "study2_morphology_adaptation"):
    sys.path.insert(0, str(path))

import one_finger  # noqa: E402
from core.optimization.mechanism import read_config_file  # noqa: E402
from visualize import sketch  # noqa: E402
from visualize.finger import view_of  # noqa: E402
from dataclasses import replace  # noqa: E402

from visualize.finger.view import load_run  # noqa: E402
from visualize.fronts import read_rows, with_role  # noqa: E402
from visualize.style import use  # noqa: E402

DEST = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / "ContactCentricDesign-RSS2027" / "figures"
FLEXION, THICKNESS = 120.0, 7.0
POLE_LAW = DESIGN / "output" / "study" / "study2_pole_law" / "designs.csv"
MARK = {"2r_dd": "^", "2r_remote": "s", "rcj": "D", "cfl": "o"}
PROTOTYPE = DESIGN / "output" / "linkage" / "config" / "paper_single_objective" / "portfolio_001" / "seeds.csv"
WIDTH = 7.16
INK, INK2 = "#0b0b0b", "#52514e"
# (key, title, colour as in the paper's other figures, one-line description, ours)
ARCH = [("2r_dd", "2R-DD", "#1baf7a", "servo at the joint", False),
        ("2r_remote", "2R-remote", "#eb6834", "pin joint, belt from a proximal servo", False),
        ("rcj", "RCJ", "#eda100", "rolling contact joint", False),
        ("cfl", "CFL (ours)", "#2a78d6", "crossed four-bar, proximal servo", True)]


SAMPLES = DESIGN / "output" / "linkage" / "config" / "samples" / "seeds.csv"
FRONTS = DESIGN / "output" / "study" / "study2_redesign_2026-11-21" / "fronts.csv"


def finite_pole(view, q_deg: float) -> np.ndarray:
    """The point the distal phalanx turns about from extension to q (Lemma 1), from two distal-pad corners."""
    p0, p1 = view.distal_pad(0.0)[:2], view.distal_pad(q_deg)[:2]
    d0, d1 = p0[1] - p0[0], p1[1] - p1[0]
    th = np.arctan2(d1[1], d1[0]) - np.arctan2(d0[1], d0[0])
    rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return np.linalg.solve(np.eye(2) - rot, p1[0] - rot @ p0[0])


def pole_on_pad(view, q_deg: float) -> tuple[float, float]:
    """The pole relative to the proximal pad: along the finger from its joint-side end, and height above its
    mounting surface (negative: inside the pad, toward the contact face)."""
    pole, pad = finite_pole(view, q_deg), np.asarray(view.proximal_pad())
    return float(pole[0] - pad[:, 0].max()), float(pole[1] - pad[:, 1].max())


def four_bar_poles(q: float, n: int = 60000, seed: int = 0) -> np.ndarray:
    """P(q), relative to the proximal phalanx's end A, of crossed four-bars whose pivots lie in the phalanges: A, B on the link line within 15 mm of the joint,
    A', B' up to 14 mm dorsal; each closes through [0, q] (F1: the moving pivot is a circle intersection) and keeps
    its assembly branch. Returns the finite poles (Lemma 1) of those that do."""
    rng = np.random.default_rng(seed)
    qs = np.linspace(0.0, q, 61)

    def rot(t):  # palmar flexion: clockwise in (x distal, z dorsal)
        return np.array([[np.cos(t), np.sin(t)], [-np.sin(t), np.cos(t)]])

    out = []
    for _ in range(n):
        a, b = np.array([rng.uniform(-15, 0), 0.0]), np.array([rng.uniform(0, 15), 0.0])
        ap = np.array([rng.uniform(-10, 15), rng.uniform(0, 14)])
        bp = np.array([rng.uniform(-10, 15), rng.uniform(0, 14)])
        r1, r2, w = np.linalg.norm(b - ap), np.linalg.norm(bp - a), bp - b
        prev, ok = b.copy(), True
        for t in qs:
            ah = a - rot(t) @ w
            d = np.linalg.norm(ah - ap)
            if not abs(r1 - r2) < d < r1 + r2:
                ok = False
                break
            along = (r1 ** 2 - r2 ** 2 + d ** 2) / (2 * d)
            u = (ah - ap) / d
            mid, off = ap + along * u, np.sqrt(max(r1 ** 2 - along ** 2, 0.0)) * np.array([-u[1], u[0]])
            c1, c2 = mid + off, mid - off
            prev = c1 if np.linalg.norm(c1 - prev) < np.linalg.norm(c2 - prev) else c2
        if not ok:
            continue
        disp = prev - rot(q) @ b
        pole = np.linalg.solve(np.eye(2) - rot(q), disp)
        if np.all(np.isfinite(pole)) and np.linalg.norm(pole) < 40:
            out.append(pole - a)  # relative to A, the proximal phalanx's end on its axis
    return np.array(out)


def main() -> int:
    use("paper")
    sketch.LINK_WIDTH = 2.0  # thinner links at print size
    sketch.SHOW_GROUND = False  # no palm hatching beside the MCP servo
    plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
                         "mathtext.fontset": "stix"})
    config = read_config_file(one_finger.DEFAULT_CONFIG)
    views = {}
    for key, *_ in ARCH:
        if key == "cfl":  # the CoDe prototype's finger, its pads drawn as thick as the others' (qualitative)
            row = with_role(read_rows(PROTOTYPE), "balanced")
            run = load_run(PROTOTYPE.parent.parent / row["run"].split("/")[-1])
            pk = run.config.packaging
            config_cfl = replace(run.config, packaging=replace(
                pk, proximal_pad=replace(pk.proximal_pad, height_mm=THICKNESS),
                distal_pad=replace(pk.distal_pad, height_mm=THICKNESS)))
            x = list(run.mechanism.read_design(row))
            names = list(run.mechanism.variable_names)
            x[names.index("distal_mount_offset_mm")] = 0.0  # pad on its link, no pivot clearance (qualitative)
            # A'' (= A'_x + offset) at A: the red triangle O-A'-A'' closes at the pivot
            x[names.index("app_offset_mm")] = x[names.index("a_x_mm")] - x[names.index("ap_x_mm")]
            views[key] = view_of(run.mechanism.name, config_cfl, x)
            continue
        designs = one_finger.study1_designs(key, config["architectures"][key], FLEXION, config)
        ref = designs[THICKNESS]
        views[key] = view_of(ref.mechanism, ref.config, ref.x)
    probe_fig, probe = plt.subplots()
    for view in views.values():
        view.draw(probe, FLEXION, hit=set(), style="sketch")
    box = probe.dataLim
    plt.close(probe_fig)
    xlim, ylim = (box.x0 - 1.5, box.x1 + 1.5), (box.y0 - 1.5, box.y1 + 1.5)
    panel_w = 1.26  # in, per sketch; a plot panel follows
    panel_h = panel_w * (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])
    title_h, foot_h = 0.2, 0.0
    height = title_h + panel_h + foot_h
    fig = plt.figure(figsize=(WIDTH, height))
    for i, (key, title, colour, desc, ours) in enumerate(ARCH):
        x0 = 0.005 + i * panel_w / WIDTH
        ax = fig.add_axes((x0, foot_h / height, panel_w / WIDTH, panel_h / height))
        sketch.frame(ax, xlim, ylim)
        view = views[key]
        view.draw(ax, FLEXION, hit=set(), style="sketch")
        fig.text(x0 + panel_w / WIDTH / 2, 1 - 0.02 / height, title, ha="center", va="top", fontsize=8.5,
                 color=colour, fontweight="bold" if ours else "normal")
    for i, lab in enumerate("abcd"):
        fig.text(0.005 + i * panel_w / WIDTH + 0.004, 1 - 0.02 / height, f"({lab})", ha="left", va="top", fontsize=7.5,
                 fontweight="bold")
    # (e) the finite pole P(q_req) in the complex plane (Lemma 1, P = b / (1 - e^{iq})), by joint type, from
    # Study 0's analysis rather than any optimization run: a fixed axis's P is the pin (a point); equal rollers'
    # P = (a + r) - i r tan(q/4) lies on one vertical segment (Study 0, Eq. 14); a four-bar's P follows from the
    # closure of its two links (circle intersection, report/study0/feasible_space.md, F1) and covers an area.
    left = 0.005 + len(ARCH) * panel_w / WIDTH + 0.075
    pax = fig.add_axes((left, 0.25, 0.995 - left - 0.005, 0.6))
    from scipy.stats import gaussian_kde
    blue, yellow = ARCH[3][2], ARCH[2][2]
    poles = four_bar_poles(np.radians(FLEXION))
    gx, gy = np.meshgrid(np.linspace(-6, 30, 260), np.linspace(-10, 10, 150))
    kde = gaussian_kde(poles.T, bw_method=0.25)
    dens = kde(np.vstack([gx.ravel(), gy.ravel()])).reshape(gx.shape)
    level = np.sort(kde(poles.T))[int(0.10 * len(poles))]
    pax.contourf(gx, gy, dens, levels=[level, dens.max() * 1.01], colors=[blue], alpha=0.25, zorder=1)
    pax.contour(gx, gy, dens, levels=[level], colors=[blue], linewidths=0.8, zorder=1.1)
    # equal rollers, the proximal one's centre at the phalanx end: P = r - i r tan(q/4), r in 3-8 mm
    r = np.array([3.0, 8.0])
    pax.plot(r, -r * np.tan(np.radians(FLEXION) / 4), color=yellow, lw=3.0, solid_capstyle="butt", zorder=3)
    pax.plot(0, 0, marker="o", ms=4.5, color=INK, zorder=4)  # a pin at the phalanx end
    pax.axhline(0, color=INK2, lw=0.4, ls=(0, (2, 2)), zorder=0)
    pax.axvline(0, color=INK2, lw=0.4, ls=(0, (2, 2)), zorder=0)
    pax.text(-0.6, 0.5, "2R: a point", fontsize=6.3, color=INK, ha="right", va="bottom", zorder=5)
    pax.text(8.6, -4.6, "RCJ: a segment", fontsize=6.3, color="#a87000", ha="left", va="center", zorder=5)
    pax.text(29.5, 7.6, "CFL: an area", fontsize=6.3, color=blue, ha="right", va="top", fontweight="bold", zorder=5)
    pax.set_xlim(-6, 30); pax.set_ylim(-8, 8); pax.set_aspect("equal")
    pax.set_xlabel("from the proximal phalanx's end (mm)", fontsize=7, labelpad=1)
    pax.set_ylabel("above its axis (mm)", fontsize=7, labelpad=1)
    pax.tick_params(labelsize=6.5, length=2)
    for side in ("top", "right"):
        pax.spines[side].set_visible(False)
    pax.grid(False)
    fig.text(left - 0.045, 1 - 0.02 / height, "(e) pole $P$ at $120^\\circ$", ha="left", va="top", fontsize=7.5,
             fontweight="bold")
    DEST.mkdir(parents=True, exist_ok=True)
    path = DEST / "fig_mechanisms.png"
    fig.savefig(path, dpi=400)
    plt.close(fig)
    print(path, f"{WIDTH:.2f} x {height:.2f} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
