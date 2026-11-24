"""Fig. 2 of the RSS 2027 extended abstract: the four PIP joints, where each can put its pole, and where the pole
should be (PNG only).

    conda run -n cmd python paper/figures/fig_mechanisms.py [dest_dir]

(a) Sketches in a 2x2 grid at 120 deg of flexion, one scale, in the style of Study 1's design figures
(design/scripts/study/study1_thickness_coverage/figure_designs.py), with pivots dotted and no palm hatching.
2R-DD, 2R-remote, RCJ: each one's Study 1 design for q_req = 120 deg and 7 mm pads. CFL: the CoDe prototype's
finger (design/output/linkage/config/paper_single_objective/portfolio_001, balanced), drawn qualitatively: pads
at 7 mm like the others, the distal pad laid on its link, and A'' placed at A so the proximal triangle closes
there. RCJ and CFL also trace the distal phalanx's instantaneous centre relative to the proximal one over 0-120 deg
(purple; star: at 120 deg): the contact point rolling on the proximal roller, and the crossing of the four-bar's links.
(b) The finite poles P each joint type can reach over the study's design space at 60, 90, 120 and 150 deg (light to
dark), in the proximal phalanx's frame (origin: its end on its axis), from Study 0's analysis (no optimization run):
a pin is the phalanx end (a point); rollers fill a narrow area (roller_pole); a crossed four-bar's P = b / (1 - e^{iq})
(Lemma 1), with b from the closure of its links (feasible_space.md, F1), covers a wide one: outlined, the region
holding 95% of the crossed four-bars drawn from the study's CFL bounds that close through q (four_bar_pole_sets,
region_outline; Gaussian kernel density, largest piece). Only the design space is drawn, no particular design.
Legend: 2R, RCJ, CFL, in the colours (c) shares.
Rollers: radii r1, r2 each in the study's 9.5-20 mm, equal (the study's RCJ, Eq. 14's segment) or not, filled as
one area (roller_pole).
(c) Study 0's coverage law (Eq. 10) over pad thickness and required flexion, one surface per joint type: a pin on
the link line, the largest roller inside the face (r = t), a virtual pole on the face. The pin's falls steeply toward
thick pads and deep flexion; the others stay nearly flat. Floor: where each crosses 0.8. Replaces the paper's
former Fig. 3.
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
REPORT = DESIGN.parent / "report"  # paper data, committed (copied from design/output)
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

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pngout import save_rgb  # noqa: E402

DEST = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / "ContactCentricDesign-RSS2027" / "figures"
FLEXION, THICKNESS = 120.0, 7.0
POLE_LAW = REPORT / "study2" / "data" / "pole_law_designs.csv"
MARK = {"2r_dd": "^", "2r_remote": "s", "rcj": "D", "cfl": "o"}
PROTOTYPE = REPORT / "study0" / "data" / "prototype_seeds.csv"
WIDTH = 7.16
INK, INK2 = "#0b0b0b", "#52514e"
ICR = "#8a3ffc"  # the instantaneous centre's path in (c, d)
# (key, title, colour as in the paper's other figures, one-line description, ours)
ARCH = [("2r_dd", "2R-DD", "#1baf7a", "servo at the joint", False),
        ("2r_remote", "2R-remote", "#eb6834", "pin joint, belt from a proximal servo", False),
        ("rcj", "RCJ", "#eda100", "rolling contact joint", False),
        ("cfl", "CFL (ours)", "#2a78d6", "crossed four-bar, proximal servo", True)]


SAMPLES = REPORT / "study0" / "data" / "linkage_samples_seeds.csv"
FRONTS = REPORT / "study2" / "data" / "redesign_fronts.csv"


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


def roller_pole(r1, r2, q: float):
    """P(q) of two rollers of radii r1 (proximal, centred at the phalanx end) and r2 rolling without slip, as x + i z
    (z dorsal). The distal roller's centre goes round the proximal roller by q r2 / (r1 + r2) while the distal
    phalanx turns by q; with r1 = r2 = r this is Study 0's Eq. 14, r - i r tan(q/4)."""
    r1, r2 = np.asarray(r1, float), np.asarray(r2, float)
    c0 = r1 + r2
    c = c0 * np.exp(-1j * q * r2 / (r1 + r2))
    return (c - np.exp(-1j * q) * c0) / (1 - np.exp(-1j * q))


def four_bar_pole_sets(degs, n: int = 200000, seed: int = 0) -> dict[int, np.ndarray]:
    """P(q), relative to the proximal pivot A (the proximal phalanx's end on its axis), for each q in degs, of crossed
    four-bars drawn uniformly from the study's CFL design space (A, B on the link line in 0-90 mm with B distal of
    A; A' from A by -20..30 mm along and 0..25 mm dorsal; B' from B by -50..20 mm along and -60..80 mm across). A
    design counts at q if its links A'-B and A-B' cross and the linkage closes through [0, q] on its assembly branch
    (F1: the moving pivot is a circle intersection, followed in 1 deg steps). Returns their finite poles (Lemma 1)."""
    rng = np.random.default_rng(seed)
    degs = sorted(int(d) for d in degs)
    steps = np.radians(np.arange(0, degs[-1] + 1))

    def rot(t):  # palmar flexion: clockwise in (x distal, z dorsal)
        return np.array([[np.cos(t), np.sin(t)], [-np.sin(t), np.cos(t)]])

    def crossed(p1, p2, p3, p4) -> bool:
        side = lambda a, b, c: (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])  # noqa: E731
        return side(p1, p2, p3) * side(p1, p2, p4) < 0 and side(p3, p4, p1) * side(p3, p4, p2) < 0

    out = {d: [] for d in degs}
    for _ in range(n):
        ax_ = rng.uniform(0.1, 90.0)
        a, b = np.array([ax_, 0.0]), np.array([rng.uniform(ax_, 90.0), 0.0])
        ap = a + np.array([rng.uniform(-20, 30), rng.uniform(0, 25)])
        bp = b + np.array([rng.uniform(-50, 20), rng.uniform(-60, 80)])
        if not crossed(ap, b, a, bp):
            continue
        r1, r2, w = np.linalg.norm(b - ap), np.linalg.norm(bp - a), bp - b
        prev = b.copy()
        for k, t in enumerate(steps):
            ah = a - rot(t) @ w
            d = np.linalg.norm(ah - ap)
            if not abs(r1 - r2) < d < r1 + r2:
                break
            along = (r1 ** 2 - r2 ** 2 + d ** 2) / (2 * d)
            u = (ah - ap) / d
            mid, off = ap + along * u, np.sqrt(max(r1 ** 2 - along ** 2, 0.0)) * np.array([-u[1], u[0]])
            c1, c2 = mid + off, mid - off
            prev = c1 if np.linalg.norm(c1 - prev) < np.linalg.norm(c2 - prev) else c2
            if k in out:
                pole = np.linalg.solve(np.eye(2) - rot(t), prev - rot(t) @ b)
                if np.all(np.isfinite(pole)):
                    out[k].append(pole - a)
    return {d: np.array(v) for d, v in out.items()}


POLE_CACHE = DESIGN / "output" / "paper" / "fig_mechanisms_poles.npz"


def cached_pole_sets(degs, n: int = 200000, seed: int = 0) -> dict[int, np.ndarray]:
    """four_bar_pole_sets, saved once to POLE_CACHE (keyed by n, seed and the flexions) and read back after, since
    the sampling takes minutes and does not change when only the sketches are edited. Delete the file to redraw."""
    key = f"n{n}_s{seed}_" + "_".join(str(int(d)) for d in sorted(degs))
    if POLE_CACHE.exists():
        data = np.load(POLE_CACHE)
        if str(data["key"]) == key:
            return {int(d): data[f"q{int(d)}"] for d in degs}
    sets = four_bar_pole_sets(degs, n, seed)
    POLE_CACHE.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(POLE_CACHE, key=key, **{f"q{d}": v for d, v in sets.items()})
    return sets


RING_CACHE = DESIGN / "output" / "paper" / "fig_mechanisms_rings.npz"


def cached_outlines(degs, gx, gy) -> dict[int, np.ndarray]:
    """The 95% outlines of the four-bar pole sets, saved to RING_CACHE: the kernel density over ~45,000 poles on the
    grid takes minutes. Keyed by the pole cache's key and the grid; delete the file to recompute."""
    key = f"{len(degs)}_{gx.shape}_{gx.min()}_{gx.max()}_{gy.min()}_{gy.max()}"
    if RING_CACHE.exists() and POLE_CACHE.exists() and RING_CACHE.stat().st_mtime >= POLE_CACHE.stat().st_mtime:
        data = np.load(RING_CACHE)
        if str(data["key"]) == key and all(f"q{int(d)}" in data.files for d in degs):
            return {int(d): data[f"q{int(d)}"] for d in degs}
    sets = cached_pole_sets(degs)
    rings = {int(d): region_outline(sets[int(d)], gx, gy) for d in degs}
    np.savez_compressed(RING_CACHE, key=key, **{f"q{d}": v for d, v in rings.items()})
    return rings


def region_outline(points: np.ndarray, gx, gy, share: float = 0.95) -> np.ndarray:
    """The outline of the region holding `share` of the points (Gaussian kernel density, bandwidth 0.25); its
    largest closed piece only, so that stray islands of the density do not show."""
    from contourpy import contour_generator
    from scipy.stats import gaussian_kde
    kde = gaussian_kde(points.T, bw_method=0.25)
    dens = kde(np.vstack([gx.ravel(), gy.ravel()])).reshape(gx.shape)
    level = np.sort(kde(points.T))[int((1 - share) * len(points))]
    lines = contour_generator(gx, gy, dens).lines(level)
    return max(lines, key=len)


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
            run = load_run(PROTOTYPE.parent / ("prototype_" + row["run"].split("/")[-1]))
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
    xlim, ylim = (box.x0 - 3.0, box.x1 + 3.0), (box.y0 - 2.0, box.y1 + 2.0)
    # (a) the CFL enlarged with its pivots and frame labelled, the other three small and stacked beside it (one scale
    # among the three); (b) the poles; (c) a third panel. Positions in inches.
    aspect = (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])
    head, cap = 0.17, 0.115  # panel-label row; a sketch's title row
    sw_small = 0.68  # in, per small sketch
    sh_small = sw_small * aspect
    height = head + 3 * (cap + sh_small)
    sw_big = 1.55
    sh_big = sw_big * aspect
    fig = plt.figure(figsize=(WIDTH, height))

    def box(x, y, w, h):
        return fig.add_axes((x / WIDTH, y / height, w / WIDTH, h / height))

    def panel(key, x0, y0, w, h, fontsize):
        _, title, colour, _, ours = next(a for a in ARCH if a[0] == key)
        ax = box(x0, y0, w, h)
        sketch.frame(ax, xlim, ylim)
        view = views[key]
        view.draw(ax, FLEXION, hit=set(), style="sketch")
        if key in ("rcj", "cfl"):  # the moving instantaneous centre over 0-120 deg, now at 120 deg
            path = icr_path(view, FLEXION)
            ax.plot(*path.T, color=ICR, lw=1.1, zorder=20, solid_capstyle="round")
            ax.plot(*path[-1], marker="*", ms=5.5, color=ICR, mec="white", mew=0.3, zorder=21)
        fig.text((x0 + w / 2) / WIDTH, (y0 + h + cap - 0.01) / height, title, ha="center", va="top",
                 fontsize=fontsize, color=colour, fontweight="bold" if ours else "normal")
        return ax

    # the CFL, enlarged, its pivots and the proximal frame named as in the text (Sec. III-A)
    big_y = 0.07
    cax_ = panel("cfl", 0.02, big_y, sw_big, sh_big, 7.5)
    four = views["cfl"].joint
    st = four.sweep([FLEXION])
    pts = {"A": four.finger.joint.a.array, "A'": four.finger.joint.ap.array, "B": st.b[0], "B'": st.bp[0],
           "C": st.tip[0], "O": np.zeros(2)}
    offsets = {"A": (-3, -7), "A'": (9, 7), "B": (6, -1), "B'": (5, -7), "C": (4, -5), "O": (-6, -8)}
    for name, at in pts.items():
        cax_.annotate(f"${name}$", xy=at, xytext=offsets[name], textcoords="offset points", fontsize=7,
                      ha="center", va="center", color=INK, zorder=30)
    arrow = dict(arrowstyle="-|>", lw=0.7, color=INK, mutation_scale=6, shrinkA=0, shrinkB=0)
    cax_.annotate("", xy=(14, 0), xytext=(0, 0), arrowprops=arrow, zorder=31)
    cax_.annotate("", xy=(0, 14), xytext=(0, 0), arrowprops=arrow, zorder=31)
    cax_.annotate("$x$", xy=(14, 0), xytext=(1, -5), textcoords="offset points", fontsize=6.5, ha="center",
                  va="center", color=INK, zorder=31)
    cax_.annotate("$z$", xy=(0, 14), xytext=(-4, 1), textcoords="offset points", fontsize=6.5, ha="center",
                  va="center", color=INK, zorder=31)
    cax_.annotate("$\\mathcal{F}_1$", xy=(0, 14), xytext=(-5, -7), textcoords="offset points", fontsize=6.5,
                  ha="right", va="center", color=INK, zorder=31)
    # the finger model as a legend above the CFL, extended (q = 0): phalanx lengths L_1, L_vp, L_2 along the link line,
    # and one pad's length, thickness and face depth t, measured palmar (Sec. III-A)
    ly = big_y + sh_big + cap + 0.03
    lax = box(0.04, ly, 1.62, height - head - ly)
    lax.set_xlim(-19, 94); lax.set_ylim(-17, 13); lax.set_aspect("equal"); lax.axis("off")
    L1, Lvp, L2 = 40.0, 8.0, 42.0
    lax.plot([-4, 94], [0, 0], color=INK2, lw=0.5, ls=(0, (3, 2)), zorder=1)
    lax.plot([0, L1], [0, 0], color="#d7263d", lw=2.0, solid_capstyle="butt", zorder=2)
    lax.plot([L1 + Lvp, L1 + Lvp + L2], [0, 0], color="#1f3fbf", lw=2.0, solid_capstyle="butt", zorder=2)
    d_, h_ = 2.5, 7.0
    lax.add_patch(plt.Rectangle((4, -d_ - h_), L1 - 6, h_, fc="#f6b8c2", ec="#d7263d", lw=0.7, zorder=2))
    lax.add_patch(plt.Rectangle((L1 + Lvp + 2, -d_ - h_), L2 - 8, h_, fc="#bcc8f2", ec="#1f3fbf", lw=0.7, zorder=2))
    dl = dict(arrowstyle="<|-|>", lw=0.45, color=INK2, mutation_scale=3.5, shrinkA=0, shrinkB=0)
    top = 4.0
    for x0_, x1_, name in ((0, L1, "$L_1$"), (L1, L1 + Lvp, "$L_{vp}$"), (L1 + Lvp, L1 + Lvp + L2, "$L_2$")):
        lax.annotate("", xy=(x1_, top), xytext=(x0_, top), arrowprops=dl)
        lax.text((x0_ + x1_) / 2, top + 1.0, name, fontsize=6.5, ha="center", va="bottom")
    lax.annotate("", xy=(L1 - 2, -d_ - h_ - 2.5), xytext=(4, -d_ - h_ - 2.5), arrowprops=dl)
    lax.text((L1 + 2) / 2, -d_ - h_ - 3.2, "$\\ell_1$", fontsize=6.5, ha="center", va="top")
    lax.annotate("", xy=(L1 + 1.0, -d_), xytext=(L1 + 1.0, -d_ - h_), arrowprops=dl)
    lax.text(L1 + 1.8, -d_ - h_ / 2, "$h_1$", fontsize=6.5, ha="left", va="center")
    lax.annotate("", xy=(-11, 0), xytext=(-11, -d_ - h_), arrowprops=dl)
    lax.text(-11.8, -(d_ + h_) / 2, "$t_1$", fontsize=6.5, ha="right", va="center")
    for i, key in enumerate(("2r_dd", "2r_remote", "rcj")):
        y0 = (2 - i) * (cap + sh_small)
        panel(key, 0.02 + sw_big + 0.08, y0, sw_small, sh_small, 6.5)
    label = dict(ha="left", va="top", fontsize=7.5, fontweight="bold")
    fig.text(0.02 / WIDTH, 1 - 0.02 / height, "(a) the four PIP joints at $120^\\circ$", **label)
    # (b) the finite pole P(q_req) in the complex plane (Lemma 1, P = b / (1 - e^{iq})), by joint type, from
    # Study 0's analysis rather than any optimization run: a fixed axis's P is the pin (a point); equal rollers'
    # P = (a + r) - i r tan(q/4) lies on one segment (Study 0, Eq. 14); a four-bar's P follows from the closure of
    # its two links (circle intersection, report/study0/feasible_space.md, F1) and covers an area.
    bx = 2.55
    fig.text(bx / WIDTH, 1 - 0.02 / height, "(b) pole $P$ over flexion", **label)
    pax = box(bx + 0.32, 0.3, 2.0, height - head - 0.36)
    blue, yellow = ARCH[3][2], ARCH[2][2]
    qr = np.radians(FLEXION)
    # at 60, 90, 120 and 150 deg, light to dark: the four-bars' 95% regions as outlines, the rollers' as fills
    shown = {60: ("#a9cbef", "#f7dc95"), 90: ("#6aa3e3", "#f2c25a"), 120: ("#2a78d6", "#eda100"),
             150: ("#0d3c78", "#a87000")}
    gx, gy = np.meshgrid(np.linspace(-45, 65, 330), np.linspace(-30, 50, 240))
    rings = cached_outlines(shown, gx, gy)
    radii = np.linspace(9.5, 20.0, 60)
    lo_r, hi_r = np.full_like(radii, 9.5), np.full_like(radii, 20.0)
    for deg, (c_four, c_roll) in shown.items():
        ring = rings[deg]
        pax.plot(ring[:, 0], ring[:, 1], color=c_four, lw=0.9, zorder=1 + deg / 1000)
        # rollers, the proximal one centred at the phalanx end, radii each in the study's 9.5-20 mm, equal or not
        t = np.radians(deg)
        edge = np.concatenate([roller_pole(radii, lo_r, t), roller_pole(hi_r, radii, t),
                               roller_pole(radii[::-1], hi_r, t), roller_pole(lo_r, radii[::-1], t)])
        pax.fill(edge.real, edge.imag, fc=c_roll, ec="none", zorder=2 + deg / 1000)
    pax.plot(0, 0, marker="o", ms=4.5, color=INK, zorder=4)  # a pin at the phalanx end
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    pax.legend(handles=[Line2D([], [], marker="o", ms=4.0, color=INK, lw=0, label="2R"),
                        Patch(fc=yellow, ec="none", label="RCJ"),
                        Line2D([], [], color=blue, lw=1.0, label="CFL")],
               loc="upper right", fontsize=6.5, frameon=False, handlelength=1.4, handletextpad=0.5,
               borderaxespad=0.1, labelspacing=0.3)
    pax.axhline(0, color=INK2, lw=0.4, ls=(0, (2, 2)), zorder=0)
    pax.axvline(0, color=INK2, lw=0.4, ls=(0, (2, 2)), zorder=0)
    pax.set_xlim(-32, 52); pax.set_ylim(-20, 38); pax.set_aspect("equal")
    pax.set_xlabel("from the proximal phalanx's end (mm)", fontsize=7, labelpad=1)
    pax.set_ylabel("above its axis (mm)", fontsize=7, labelpad=1)
    pax.tick_params(labelsize=6.5, length=2)
    for side in ("top", "right"):
        pax.spines[side].set_visible(False)
    pax.grid(False)
    # (c) how coverage falls with pad thickness h and required flexion q_req, per joint type, from Study 0 alone:
    # the coverage law (Eq. 10) with faces at depth t = h (mount depth 0), a pole at constant depth z, gap
    # G = 2 [t - z]_+ tan(q_req / 2), the finger of the paper's former Fig. 3 (L_f = 90 mm, pads of 40.7 mm each at
    # G = 0, so x0 = 8.6 mm), averaged over Q_eval = {90, (90 + q_req) / 2, q_req} deg. A pin sits on the link line
    # (z = 0); the largest roller inside the face (r = t) puts its pole r tan(q_req / 4) deep; a virtual pole sits on
    # the face (z = t). On the floor: where each surface crosses 0.8.
    cx = 4.92
    fig.text(cx / WIDTH, 1 - 0.02 / height, "(c) coverage over pad and flexion", **label)
    cax = fig.add_axes(((cx - 0.05) / WIDTH, 0.0, (WIDTH - cx + 0.05) / WIDTH, (height - head) / height),
                       projection="3d")
    cax.set_box_aspect((1.0, 1.0, 0.95), zoom=0.92)
    Lf, x0 = 90.0, 90.0 - 2 * 40.7
    hh, qd = np.meshgrid(np.linspace(1, 9, 60), np.linspace(90, 150, 60))
    qq = np.radians(qd)
    depth = {"2r": 0.0 * hh, "rcj": hh * np.tan(qq / 4), "cfl": hh}

    def coverage(z, t, qreq):
        g = 2 * np.maximum(t - z, 0.0) * np.tan(qreq / 2)
        evals = [np.radians(90.0) + 0 * qreq, (np.radians(90.0) + qreq) / 2, qreq]
        return np.mean([(Lf - x0 - g) / (Lf - g * (1 - np.cos(q / 2)) + 2 * z * np.sin(q / 2)) for q in evals],
                       axis=0)

    floor = 0.3
    sheet = {"2r": "#6f6e6a", "rcj": yellow, "cfl": blue}
    for key, colour in sheet.items():
        c = np.clip(coverage(depth[key], hh, qq), floor, 1.0)
        cax.plot_surface(hh, qd, c, color=colour, alpha=0.5, lw=0, antialiased=True, shade=True, rstride=2, cstride=2)
        cax.plot_wireframe(hh, qd, c, color=colour, lw=0.3, rstride=12, cstride=12, alpha=0.9)
        cax.contour(hh, qd, c, levels=[0.8], zdir="z", offset=floor, colors=[colour], linewidths=1.0)
    cax.set_xlim(1, 9); cax.set_ylim(90, 150); cax.set_zlim(floor, 0.9)
    cax.set_xticks([2, 5, 8]); cax.set_yticks([90, 120, 150]); cax.set_zticks([0.3, 0.5, 0.7, 0.9])
    cax.set_xlabel("pad $h$ (mm)", labelpad=-9, fontsize=6.5)
    cax.set_ylabel("flexion $q_{\\mathrm{req}}$ (deg)", labelpad=-8, fontsize=6.5)
    cax.set_zlabel(r"$\bar C_{\mathrm{pad}}$", labelpad=-9, fontsize=6.5)
    cax.tick_params(labelsize=5.5, pad=-3)
    for axis in (cax.xaxis, cax.yaxis, cax.zaxis):
        axis.pane.set_facecolor((1, 1, 1, 0)); axis.pane.set_edgecolor("#bdbcb8")
        axis._axinfo["grid"]["color"] = (0.85, 0.85, 0.85, 1); axis._axinfo["grid"]["linewidth"] = 0.3
    cax.view_init(elev=16, azim=60)
    DEST.mkdir(parents=True, exist_ok=True)
    path = DEST / "fig_mechanisms.png"
    save_rgb(fig, path, dpi=400)
    plt.close(fig)
    print(path, f"{WIDTH:.2f} x {height:.2f} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
