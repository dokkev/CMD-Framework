"""Fig. 2 of the RSS 2027 extended abstract: the four PIP joints compared, and how far each one's pole sits from
the contact face (PNG only).

    conda run -n cmd python paper/figures/fig_mechanisms.py [dest_dir]

(a-d) Sketches at 120 deg of flexion, one scale, in the style of Study 1's design figures
(design/scripts/study/study1_thickness_coverage/figure_designs.py), with pivots dotted and no palm hatching.
2R-DD, 2R-remote, RCJ: each one's Study 1 design for q_req = 120 deg and 7 mm pads. CFL: the CoDe prototype's
finger (design/output/linkage/config/paper_single_objective/portfolio_001, balanced), drawn qualitatively: pads
at 7 mm like the others, the distal pad laid on its link, and A'' placed at A so the proximal triangle closes
there. (c, d) also trace the distal phalanx's instantaneous centre relative to the proximal one over 0-120 deg
(purple; star: at 120 deg): the contact point rolling on the proximal roller, and the crossing of the four-bar's links.
(e) The finite poles P(120 deg) each joint type can reach over the study's design space, in the proximal
phalanx's frame (origin: its end on its axis), from Study 0's analysis (no optimization run): a pin is the phalanx
end (a point); equal rollers, the proximal one centred at the phalanx end, put P = r - i r tan(q/4) (Study 0,
Eq. 14) on one segment for the study's r of 9.5-20 mm; a crossed four-bar's P = b / (1 - e^{iq}) (Lemma 1), with b
from the closure of its links (feasible_space.md, F1), covers an area: shaded, the region holding 95% of the crossed
four-bars drawn from the study's CFL bounds that close through 120 deg (see four_bar_poles); Gaussian kernel
density. Dots: the CFL's Study 1 designs for 3-9 mm pads, all inside it; lines: how each one's P(q) moves for q
from 0 to 120 deg, and the same for Study 1's RCJ (r = 9.5 mm, straight down). No text labels: the caption names them.
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
ICR = "#8a3ffc"  # the instantaneous centre's path in (c, d)
# (key, title, colour as in the paper's other figures, one-line description, ours)
ARCH = [("2r_dd", "2R-DD", "#1baf7a", "servo at the joint", False),
        ("2r_remote", "2R-remote", "#eb6834", "pin joint, belt from a proximal servo", False),
        ("rcj", "RCJ", "#eda100", "rolling contact joint", False),
        ("cfl", "CFL (ours)", "#2a78d6", "crossed four-bar, proximal servo", True)]


SAMPLES = DESIGN / "output" / "linkage" / "config" / "samples" / "seeds.csv"
FRONTS = DESIGN / "output" / "study" / "study2_redesign_2026-11-21" / "fronts.csv"


def finite_pole(view, q_deg: float, q0_deg: float = 0.0) -> np.ndarray:
    """The point the distal phalanx turns about from q0 (default: extension) to q (Lemma 1), from two distal-pad
    corners. With q close to q0 it is the instantaneous centre at q0."""
    p0, p1 = view.distal_pad(q0_deg)[:2], view.distal_pad(q_deg)[:2]
    d0, d1 = p0[1] - p0[0], p1[1] - p1[0]
    th = np.arctan2(d1[1], d1[0]) - np.arctan2(d0[1], d0[0])
    rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return np.linalg.solve(np.eye(2) - rot, p1[0] - rot @ p0[0])


def icr_path(view, q_deg: float, n: int = 121) -> np.ndarray:
    """The instantaneous centre of the distal phalanx relative to the proximal one over [0, q] (its fixed centrode)."""
    return np.array([finite_pole(view, t + 0.05, t) for t in np.linspace(0.0, q_deg, n)])


def pole_on_pad(view, q_deg: float) -> tuple[float, float]:
    """The pole relative to the proximal pad: along the finger from its joint-side end, and height above its
    mounting surface (negative: inside the pad, toward the contact face)."""
    pole, pad = finite_pole(view, q_deg), np.asarray(view.proximal_pad())
    return float(pole[0] - pad[:, 0].max()), float(pole[1] - pad[:, 1].max())


def four_bar_poles(q: float, n: int = 200000, seed: int = 0) -> np.ndarray:
    """P(q), relative to the proximal pivot A (the proximal phalanx's end on its axis), of crossed four-bars drawn
    uniformly from the study's CFL design space (A, B on the link line in 0-90 mm with B distal of A; A' from A by
    -20..30 mm along and 0..25 mm dorsal; B' from B by -50..20 mm along and -60..80 mm across), kept if the links
    A'-B and A-B' cross and the linkage closes through [0, q] on its assembly branch (F1: the moving pivot is a
    circle intersection). Returns their finite poles (Lemma 1)."""
    rng = np.random.default_rng(seed)
    qs = np.linspace(0.0, q, 61)

    def rot(t):  # palmar flexion: clockwise in (x distal, z dorsal)
        return np.array([[np.cos(t), np.sin(t)], [-np.sin(t), np.cos(t)]])

    def crossed(p1, p2, p3, p4) -> bool:
        side = lambda a, b, c: (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])  # noqa: E731
        return side(p1, p2, p3) * side(p1, p2, p4) < 0 and side(p3, p4, p1) * side(p3, p4, p2) < 0

    out = []
    for _ in range(n):
        ax_ = rng.uniform(0.1, 90.0)
        a, b = np.array([ax_, 0.0]), np.array([rng.uniform(ax_, 90.0), 0.0])
        ap = a + np.array([rng.uniform(-20, 30), rng.uniform(0, 25)])
        bp = b + np.array([rng.uniform(-50, 20), rng.uniform(-60, 80)])
        if not crossed(ap, b, a, bp):
            continue
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
        pole = np.linalg.solve(np.eye(2) - rot(q), prev - rot(q) @ b)
        if np.all(np.isfinite(pole)):
            out.append(pole - a)
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
        if key in ("rcj", "cfl"):  # the moving instantaneous centre over 0-120 deg, now at 120 deg
            path = icr_path(view, FLEXION)
            ax.plot(*path.T, color=ICR, lw=1.1, zorder=20, solid_capstyle="round")
            ax.plot(*path[-1], marker="*", ms=5.5, color=ICR, mec="white", mew=0.3, zorder=21)
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
    gx, gy = np.meshgrid(np.linspace(-26, 50, 320), np.linspace(-15, 23, 170))
    kde = gaussian_kde(poles.T, bw_method=0.25)
    dens = kde(np.vstack([gx.ravel(), gy.ravel()])).reshape(gx.shape)
    level = np.sort(kde(poles.T))[int(0.05 * len(poles))]  # the region holding 95% of them
    pax.contourf(gx, gy, dens, levels=[level, dens.max() * 1.01], colors=[blue], alpha=0.25, zorder=1)
    pax.contour(gx, gy, dens, levels=[level], colors=[blue], linewidths=0.8, zorder=1.1)
    # the CFL's Study 1 designs (3-9 mm), in the same frame
    s1 = one_finger.study1_designs("cfl", config["architectures"]["cfl"], FLEXION, config)
    # and how each one's P moves as the joint flexes to 120 deg (Lemma 1 from extension to q)
    for ref in s1.values():
        v = view_of(ref.mechanism, ref.config, ref.x)
        a0 = v.joint.finger.joint.a.array
        track = np.array([finite_pole(v, t) - a0 for t in np.linspace(2.0, FLEXION, 60)])
        pax.plot(*track.T, color=blue, lw=0.7, zorder=3.5)
        pax.plot(*track[-1], marker="o", ms=2.8, color=blue, mec="white", mew=0.4, zorder=4)
    # Study 1's RCJ (r = 9.5 mm): P = r - i r tan(q/4) moves straight down as the joint flexes
    t = np.radians(np.linspace(0.0, FLEXION, 30))
    pax.plot(np.full_like(t, 9.5), -9.5 * np.tan(t / 4), color="#a87000", lw=0.7, zorder=3.5)
    # equal rollers, the proximal one's centre at the phalanx end: P = r - i r tan(q/4), r in the study's 9.5-20 mm
    r = np.array([9.5, 20.0])
    pax.plot(r, -r * np.tan(np.radians(FLEXION) / 4), color=yellow, lw=3.0, solid_capstyle="butt", zorder=3)
    pax.plot(0, 0, marker="o", ms=4.5, color=INK, zorder=4)  # a pin at the phalanx end
    pax.axhline(0, color=INK2, lw=0.4, ls=(0, (2, 2)), zorder=0)
    pax.axvline(0, color=INK2, lw=0.4, ls=(0, (2, 2)), zorder=0)
    pax.set_xlim(-26, 50); pax.set_ylim(-15, 23); pax.set_aspect("equal")
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
