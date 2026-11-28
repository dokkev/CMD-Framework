"""Fig. 1 of the RSS 2027 extended abstract (PNG only, single column, print size). Every render is of the visual
USD used in Isaac Lab, drawn by Newton's GL viewer (sim/isaaclab/scripts/code_hand/render_visuals.py; never the Drake
renders): the hand palm up (--view palm_up -> cad/hand_usd_palm_up.png) and the fingers of O, R and U from the side
(logs/urdf/cfl_q120_h3to7_{h0,hr,hu}_thumbpad_vis_ball_colliders.usda, --view side --pip 0 60 120 ->
cad/usd_finger_<H>_q<q>.png). The dark viewer background is blended to white here.

    conda run -n cmd python paper/figures/fig_cad.py [dest_dir]

Single column (user, 2026-11-27): (a) the hand and (b) the interfaces it can carry on top; (c) the CoDe and Coverage
hands for a thicker interface running the reference hand's command (Isaac Lab compliant-pad pull-out and the Drake dial).
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
CAD = ROOT / "paper" / "figures" / "cad"
DEST = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / "ContactCentricDesign-RSS2027" / "figures"
INK, INK2, PAD = "#0b0b0b", "#52514e", "#2a5db0"


def usd_white(path, paper=244.0):
    """The Newton render with its uniform dark background blended to a light grey (white parts stay visible) and the
    mask of the hand (not cropped)."""
    from PIL import Image
    im = np.asarray(Image.open(path).convert("RGB")).astype(float)
    bg = np.median(np.concatenate([im[:20, :20].reshape(-1, 3), im[-20:, -20:].reshape(-1, 3)]), axis=0)
    alpha = np.clip((np.linalg.norm(im - bg, axis=2) - 1.5) / 6.0, 0.0, 1.0)[..., None]
    return (alpha * im + (1.0 - alpha) * paper).astype(np.uint8), alpha[..., 0]


def white_to_alpha(im, bg="#ffffff", lo=236, hi=252):
    """RGB (uint8) with the near-white background faded into the colour bg of the tinted box it sits on; opaque RGB,
    so the PDF holds no soft mask."""
    from matplotlib.colors import to_rgb
    im = np.asarray(im)[..., :3].astype(float) / 255.0
    a = np.clip((hi / 255.0 - im.min(axis=2)) / ((hi - lo) / 255.0), 0.0, 1.0)[..., None]
    return a * im + (1.0 - a) * np.array(to_rgb(bg))


def usd_render(path, gradient=False):
    """The Newton render with its background blended to white and cropped to the hand. gradient: the viewer's sky
    (a camera below the ground plane), keyed row by row against the left and right edge columns."""
    from PIL import Image
    im = np.asarray(Image.open(path).convert("RGB")).astype(float)
    if gradient:
        bg = np.median(np.concatenate([im[:, :24], im[:, -24:]], axis=1), axis=1)[:, None, :]
        alpha = np.clip((np.linalg.norm(im - bg, axis=2) - 10.0) / 12.0, 0.0, 1.0)[..., None]
    else:
        bg = np.median(np.concatenate([im[:20, :20].reshape(-1, 3), im[-20:, -20:].reshape(-1, 3)]), axis=0)
        alpha = np.clip((np.linalg.norm(im - bg, axis=2) - 2.0) / 6.0, 0.0, 1.0)[..., None]  # sharp key: dark servos stay dark
    out = alpha * im + (1.0 - alpha) * 255.0
    rows, cols = np.where(alpha[..., 0] > 0.05)
    out = out[max(rows.min() - 8, 0):rows.max() + 8, max(cols.min() - 8, 0):cols.max() + 8]
    return out.astype(np.uint8)


def oblique(ax, poly, depth=(1.6, 1.0), fc="#cccccc", ec="#555555", top="#e6e6e6", side="#b3b3b3", lw=0.4, z=1):
    """An extruded 2-D section drawn in oblique projection: front face, plus the visible top and right faces."""
    from matplotlib.patches import Polygon
    P = np.asarray(poly, float)
    d = np.asarray(depth, float)
    n = len(P)
    for k in range(n):  # side faces whose outward normal points up or right (visible)
        a, b = P[k], P[(k + 1) % n]
        e = b - a
        nrm = np.array([e[1], -e[0]])  # outward for counter-clockwise polygons
        if nrm @ d > 1e-9:
            ax.add_patch(Polygon([a, b, b + d, a + d], fc=top if abs(e[0]) > abs(e[1]) else side, ec=ec, lw=lw,
                                 zorder=z))
    ax.add_patch(Polygon(P, fc=fc, ec=ec, lw=lw, zorder=z + 0.1))


def honeycomb_section(x0, x1, y0, y1, cell=1.15, wall=0.22):
    """A honeycomb (hexagonal) lattice clipped to the box: returns the solid walls as a shapely geometry.
    Hexagons of circumradius ``cell`` (flat sides up), walls of thickness ``wall``."""
    from shapely.geometry import Polygon as SP, box
    from shapely.ops import unary_union
    r = cell
    w, hgt = 1.5 * r, np.sqrt(3) * r
    holes = []
    for i in range(int((x1 - x0) / w) + 3):
        for j in range(int((y1 - y0) / hgt) + 3):
            cx = x0 + i * w
            cy = y0 + j * hgt + (hgt / 2 if i % 2 else 0)
            ang = np.radians(np.arange(0, 360, 60))
            holes.append(SP(np.c_[cx + (r - wall / 2) * np.cos(ang), cy + (r - wall / 2) * np.sin(ang)]))
    outer = box(x0, y0, x1, y1)
    return outer.difference(unary_union(holes)).intersection(outer)


def rounded_rim(L, Wd, rr, inset=0.0, n=8):
    """Counter-clockwise points of a rounded rectangle [0, L] x [0, Wd] (corner radius rr), shrunk by ``inset``."""
    r = max(rr - inset, 0.05)
    t = np.linspace(0, np.pi / 2, n)
    corners = [(L - rr, rr, -np.pi / 2), (L - rr, Wd - rr, 0.0), (rr, Wd - rr, np.pi / 2), (rr, rr, np.pi)]
    return np.concatenate([np.c_[cu + r * np.cos(t + a0), cv + r * np.sin(t + a0)] for cu, cv, a0 in corners])


def iso_pad(ax, H, top, side1, side2, edge, L=6.5, Wd=4.5, decorate=None, rr=0.7, fil=0.35):
    """A pad block in 30 deg isometric view (long axis toward the upper left), contact face up, with rounded corners
    (radius rr) and a filleted top edge (radius fil): the visible sides shaded by their facing, the fillet, the face,
    and ``decorate(iso, L, Wd)`` drawn on it."""
    import matplotlib.colors as mc
    from matplotlib.patches import Polygon as Pg
    for p_ in list(ax.patches):
        p_.remove()
    c30, s30 = np.cos(np.radians(30)), np.sin(np.radians(30))

    def iso(u, v, w=H):
        return np.array([(v - u) * c30, (u + v) * s30 + w])
    c1, c2, ct = (np.array(mc.to_rgb(c)) for c in (side1, side2, top))

    def side_colour(nu, nv):
        wt = max(-nv, 0) / max(max(-nu, 0) + max(-nv, 0), 1e-9)
        return wt * c1 + (1 - wt) * c2
    rim = rounded_rim(L, Wd, rr)
    rim_in = rounded_rim(L, Wd, rr, inset=fil)
    k = len(rim)
    for i in range(k):  # sides: only the segments that face the viewer (outward normal toward -u, -v)
        p0, p1 = rim[i], rim[(i + 1) % k]
        e = p1 - p0
        nrm = np.array([e[1], -e[0]]) / (np.linalg.norm(e) + 1e-12)
        if nrm @ np.array([-1.0, -1.0]) <= 0:
            continue
        col = side_colour(*nrm)
        ax.add_patch(Pg([iso(*p0, 0), iso(*p1, 0), iso(*p1, H - fil), iso(*p0, H - fil)], fc=col, ec=col, lw=0.2,
                        zorder=1))
        q0, q1 = rim_in[i], rim_in[(i + 1) % k]  # the fillet, lighter toward the face
        fc = 0.5 * col + 0.5 * ct
        ax.add_patch(Pg([iso(*p0, H - fil), iso(*p1, H - fil), iso(*q1, H), iso(*q0, H)], fc=fc, ec=fc, lw=0.2,
                        zorder=1.1))
    ax.add_patch(Pg([iso(u, v) for u, v in rim_in], fc=top, ec=edge, lw=0.3, zorder=1.2))
    sil = [iso(u, v, 0) for u, v in rim] + [iso(u, v, H - fil) for u, v in rim]  # outline of the whole body
    from scipy.spatial import ConvexHull
    sil = np.array(sil)
    ax.add_patch(Pg(sil[ConvexHull(sil).vertices], fc="none", ec=edge, lw=0.35, zorder=1.3))
    if decorate:
        decorate(iso, L, Wd)
    ax.set_xlim(-L * c30 - 0.4, Wd * c30 + 0.4); ax.set_ylim(-0.6, (L + Wd) * s30 + 2.6)


def icon(ax, kind):
    """A cross-section sketch of one contact interface on its module base (illustrative, not to scale)."""
    from matplotlib.patches import Rectangle, Polygon, FancyBboxPatch
    ax.set_xlim(0, 10); ax.set_ylim(-5.2, 1.6); ax.set_aspect("equal"); ax.axis("off")
    ax.add_patch(Rectangle((0.5, 0.0), 9.0, 1.0, fc="#c9cdd2", ec=INK2, lw=0.5))  # module base on the phalanx
    from matplotlib.patches import Polygon as Pg
    if kind == "honeycomb":  # compliant structure: hexagonal cells whose walls take the load on the contact face
        H = 2.6

        def cells(iso, L, Wd):
            r, wall = 0.6, 0.11
            w_, h_ = np.sqrt(3) * r, 1.5 * r  # pointy-top hexagons in the side face (u along, w up)
            ang = np.radians(np.arange(30, 390, 60))
            for i in range(int(L / w_) + 2):
                for j in range(int(H / h_) + 2):
                    cu, cw = 0.55 + i * w_ + (w_ / 2 if j % 2 else 0), 0.68 + j * h_
                    hu, hw = cu + (r - wall) * np.cos(ang), cw + (r - wall) * np.sin(ang)
                    if hu.min() < 0.8 or hw.min() < 0.1 or hu.max() > L - 0.8 or hw.max() > H - 0.45:
                        continue
                    ax.add_patch(Pg([iso(u, 0, w) for u, w in zip(hu, hw)], fc="#2f5fa8", ec="none", zorder=2))
            for i in range(int(Wd / w_) + 2):  # and on the near end face (u = 0)
                for j in range(int(H / h_) + 2):
                    cv, cw = 0.55 + i * w_ + (w_ / 2 if j % 2 else 0), 0.68 + j * h_
                    hv, hw = cv + (r - wall) * np.cos(ang), cw + (r - wall) * np.sin(ang)
                    if hv.min() < 0.8 or hw.min() < 0.1 or hv.max() > Wd - 0.8 or hw.max() > H - 0.45:
                        continue
                    ax.add_patch(Pg([iso(0, v, w) for v, w in zip(hv, hw)], fc="#2a55a0", ec="none", zorder=2))
        iso_pad(ax, H, "#7aa6e3", "#5a8bd4", "#4677c4", PAD, decorate=cells)
        return
    if kind == "traction":  # black traction pad with a tyre-like chevron tread
        def tread(iso, L, Wd):
            for u0 in np.arange(0.95, L - 1.3, 0.7):
                for v_a, v_b in ((0.65, Wd / 2 - 0.2), (Wd - 0.65, Wd / 2 + 0.2)):
                    ax.plot(*np.array([iso(u0, v_a), iso(u0 + 0.45, v_b)]).T, color="#5a5a5a", lw=0.9,
                            solid_capstyle="round", zorder=2)
            ax.plot(*np.array([iso(0.8, Wd / 2), iso(L - 0.8, Wd / 2)]).T, color="#3a3a3a", lw=0.6, zorder=2)
        iso_pad(ax, 0.9, "#151515", "#262626", "#1a1a1a", "#000000", decorate=tread)
        return
    if kind == "thin":
        iso_pad(ax, 0.35, "#5b7fc7", "#4a6db5", "#3f60a6", PAD)
        return
    if kind == "thin_block":
        iso_pad(ax, 1.4, "#8fb6e4", "#5f91d2", "#3f78c4", PAD, rr=0.3, fil=0.15)
        return
    if kind == "soft":  # solid soft pad, a moulded silicone fingertip (as the PLATO hand's): a rounded loaf, shaded
        for p_ in list(ax.patches):
            p_.remove()
        L, a_, b_, R = 6.5, 2.25, 1.9, 1.8  # length, half width, height, end-cap radius
        c30, s30 = np.cos(np.radians(30)), np.sin(np.radians(30))
        light = np.array([-0.35, -0.55, 0.76]); light /= np.linalg.norm(light)
        nu, nph = 46, 28
        us = np.linspace(0, L, nu)
        phs = np.linspace(0, np.pi, nph)

        def surf(u, ph):
            d = min(u, L - u)
            sc = np.sqrt(max(1 - ((R - d) / R) ** 2, 0.0)) if d < R else 1.0
            return np.array([u, a_ + a_ * sc * np.cos(ph), b_ * sc * np.sin(ph)])
        P = np.array([[surf(u, ph) for ph in phs] for u in us])  # (nu, nph, 3)
        quads = []
        for i in range(nu - 1):
            for j in range(nph - 1):
                q = np.array([P[i, j], P[i + 1, j], P[i + 1, j + 1], P[i, j + 1]])
                n = np.cross(q[1] - q[0], q[3] - q[0])
                if np.linalg.norm(n) < 1e-9:
                    n = np.cross(q[2] - q[0], q[3] - q[1])
                n = n / (np.linalg.norm(n) + 1e-12)
                if n[2] < 0 and q[:, 2].mean() < 1e-6:
                    continue
                if n @ np.array([-0.5, -0.5, 0.7]) < 0:  # facing away from the viewer (u, v toward it, z up)
                    n = -n
                c = q.mean(axis=0)
                depth = c[0] + c[1] - 1.6 * c[2]
                shade = 0.55 + 0.45 * max(n @ light, 0.0)
                quads.append((depth, q, shade))
        base = np.array([0.80, 0.85, 0.93])
        for depth, q, sh in sorted(quads, key=lambda t: -t[0]):
            xy = np.array([[(v - u) * c30, (u + v) * s30 + w] for u, v, w in q])
            col = np.clip(base * sh + (1 - sh) * 0.05, 0, 1)
            ax.add_patch(Pg(xy, fc=col, ec=col, lw=0.15, zorder=1))
        from scipy.spatial import ConvexHull
        allxy = np.array([[(v - u) * c30, (u + v) * s30 + w] for u, v, w in P.reshape(-1, 3)])
        hull = allxy[ConvexHull(allxy).vertices]
        ax.add_patch(Pg(hull, fc="none", ec="#7f93b8", lw=0.4, zorder=3))
        ax.set_xlim(-L * c30 - 0.4, 2 * a_ * c30 + 0.4); ax.set_ylim(-0.6, (L + 2 * a_) * s30 + 2.6)
        return
    if kind == "thick":
        ax.add_patch(FancyBboxPatch((0.7, -3.6), 8.6, 3.5, boxstyle="round,pad=0.1,rounding_size=1.2",
                                    fc="#b9c9ee", ec=PAD, lw=0.5))
    elif kind == "textured":
        ax.add_patch(Rectangle((0.5, -1.6), 9.0, 1.6, fc="#8fa7dc", ec=PAD, lw=0.5))
        for x in np.arange(0.9, 9.4, 0.9):
            ax.add_patch(Polygon([(x - 0.4, -1.6), (x, -2.3), (x + 0.4, -1.6)], fc="#8fa7dc", ec=PAD, lw=0.4))
    elif kind == "tactile":
        ax.add_patch(Rectangle((0.5, -3.4), 9.0, 3.4, fc="#eef0f3", ec=INK2, lw=0.5))   # housing
        ax.add_patch(Rectangle((3.6, -1.4), 2.8, 1.2, fc="#6b6f75", ec=INK2, lw=0.4))   # camera
        ax.add_patch(Polygon([(4.4, -1.4), (5.6, -1.4), (6.4, -3.0), (3.6, -3.0)], fc="#fff6cc", ec="none",
                             alpha=0.9))                                                    # field of view
        ax.add_patch(Rectangle((0.5, -4.6), 9.0, 1.2, fc="#d6c9ee", ec="#7a5fb0", lw=0.5))  # gel
    import numpy as _np  # noqa: F401


HAND_C = {"O": "#4D4D4D", "R": "#0F4D92", "U": "#B64342"}


def crop_white(im, pad=6):
    rows, cols = np.where(im.min(axis=2) < 245)
    return im[max(rows.min() - pad, 0):rows.max() + pad, max(cols.min() - pad, 0):cols.max() + pad]


def finger_sketch(ax, colour, kind):
    """Qualitative finger at q = 0 in the proximal pad-face frame (O's contact faces at y = 0), not to scale.
    kind "O": thin pads; "U": thicker pads on O's links, so the contact faces drop by DT and turn about O's pole;
    "R": thicker pads with the links raised by DT, the pole kept where it is relative to the contact faces.
    Draws the distal contact point's path over 0-120 deg about the pole, O's dashed for reference."""
    from matplotlib.patches import Rectangle, Circle
    ax.set_aspect("equal"); ax.axis("off")
    t0, dt = 0.55, 1.0
    t = t0 if kind == "O" else t0 + dt
    face = 0.0 if kind != "U" else -dt
    bar0 = face + t  # bottom of the phalanx bars
    pole = np.array([0.0, 1.1 + (t0 - 0.55)])  # O's pole, kept for U (same links) and R (same place vs the faces)
    pad_c = "#bfbfbf" if kind == "O" else colour
    for x0, x1 in ((-4.2, -0.35), (0.35, 4.2)):
        ax.add_patch(Rectangle((x0, bar0), x1 - x0, 0.9, fc="#c9cdd2", ec=INK2, lw=0.5))
        ax.add_patch(Rectangle((x0 + 0.15, face), x1 - x0 - 0.3, t, fc=pad_c, ec="none", alpha=0.85))
    # crossed links between the phalanges
    for (xa, ya), (xb, yb) in (((-0.9, bar0 + 0.85), (0.9, bar0 + 0.05)), ((-0.9, bar0 + 0.05), (0.9, bar0 + 0.85))):
        ax.plot([xa, xb], [ya, yb], color=INK2, lw=1.4, solid_capstyle="round")
    ax.add_patch(Circle(pole, 0.17, fc="#9A4D8E", ec="white", lw=0.4, zorder=6))

    def path(c0):
        q = np.radians(np.linspace(0, 120, 61))
        rel = np.asarray(c0) - pole
        return np.stack([pole[0] + rel[0] * np.cos(-q) - rel[1] * np.sin(-q),
                         pole[1] + rel[0] * np.sin(-q) + rel[1] * np.cos(-q)], axis=1)
    ref = path((3.4, 0.0))
    ax.plot(ref[:, 0], ref[:, 1], color="#8a8a8a", lw=1.0, ls=(0, (2.5, 1.6)), zorder=6)
    if kind != "O":
        own = path((3.4, face))
        ax.plot(own[:, 0], own[:, 1], color=colour, lw=1.6, zorder=4)
        ax.plot(*own[-1], marker="o", ms=2.5, color=colour, zorder=5)
    ax.plot(*ref[-1], marker="o", ms=2.5, color="#8a8a8a", zorder=4)
    ax.set_xlim(-4.4, 4.6); ax.set_ylim(-3.9, 3.0)


def fit_icon(ax, kk):
    """An interface icon fitted to its axes (CAD render for NARI-Touch, drawn otherwise)."""
    from PIL import Image
    if kk == "nari":
        ax.imshow(white_to_alpha(Image.open(CAD / "nari_touch.png").convert("RGB"), "#f6f1e6")); ax.axis("off")
        return
    icon(ax, kk)
    pts = []
    for art in list(ax.patches) + list(ax.lines):
        xy = art.get_xy() if hasattr(art, "get_xy") else np.c_[art.get_xdata(), art.get_ydata()]
        pts.append(np.asarray(xy, float).reshape(-1, 2))
    pts = np.vstack(pts)
    (x0, y0), (x1, y1) = pts.min(0), pts.max(0)
    pad = 0.06 * max(x1 - x0, y1 - y0)
    ax.set_xlim(x0 - pad, x1 + pad); ax.set_ylim(y0 - pad, y1 + pad)
    ax.set_aspect("equal", adjustable="box", anchor="C")


def main() -> int:
    """Fig. 1 (user layout, 2026-11-25), page width, read left to right as a flow: what we have (the CoDe Hand and its
    crossed four-bar finger, CAD) and the contact interfaces it can carry -> the redesign for a new interface (the
    operator) -> two hands, CoDe (keeps the contact-surface motion) and Coverage (coverage only) -> what each does
    with a reused command: the measured interface effect, O for the same effect as the reference hand, X otherwise.
    Numbers: Isaac Lab softer-pad pull-out (isaac_conclusion_study3.json, 7 mm) and the Drake fingertip dial
    (fig4_sync.json, friction effect as a share of O's)."""
    import json
    from PIL import Image
    from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Circle as Ci, Circle, Rectangle, Polygon
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from pngout import register_fonts, save_figure
    register_fonts()
    plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["Helvetica Light", "Nimbus Sans"],
                         "font.weight": "light", "mathtext.fontset": "custom", "mathtext.rm": "Helvetica Light",
                         "mathtext.it": "Nimbus Sans:italic", "mathtext.bf": "Nimbus Sans:bold", "font.size": 7})
    BOLD = dict(fontweight="bold", family="Nimbus Sans")
    # single column (user, 2026-11-27): (a) the hand and (b) the interfaces on top; (c) the two redesigns running the
    # reference hand's command below; the redesign operator panel is dropped
    W_, H_ = 3.45, 3.60
    fig = plt.figure(figsize=(W_, H_))
    conc = json.load(open(ROOT / "report" / "study3" / "data" / "isaac_conclusion_study3.json"))["hands"]
    sync = json.load(open(ROOT / "report" / "study3" / "data" / "fig4_sync.json"))["tasks"]
    BLUE_, ORANGE_, GREEN_, RED_ = HAND_C["R"], HAND_C["U"], "#2e9b4f", "#d0342c"

    def inch(x, y):
        return x / W_, y / H_

    def box(x, y, w, h, fc, ec, r=0.06, lw=0.7, ls="-", z=-1):
        fig.patches.append(FancyBboxPatch(inch(x, y), w / W_, h / H_, boxstyle=f"round,pad=0,rounding_size={r / W_}",
                                          transform=fig.transFigure, fc=fc, ec=ec, lw=lw, ls=ls, zorder=z))

    def image(im, x, y, w=None, h=None, fit=None):
        H0, W0 = im.shape[:2]
        if fit is not None:  # the largest size inside a (w, h) cell
            w, h = fit
            if w / h > W0 / H0:
                w = None
            else:
                h = None
        if w is None:
            w = h * W0 / H0
        if h is None:
            h = w * H0 / W0
        a = fig.add_axes((*inch(x, y), w / W_, h / H_)); a.imshow(im); a.axis("off")
        return w, h

    def arrow(p0, p1, col="k", lw=0.9, rad=0.0):
        fig.patches.append(FancyArrowPatch(inch(*p0), inch(*p1), transform=fig.transFigure, arrowstyle="-|>",
                                           mutation_scale=7, lw=lw, color=col, connectionstyle=f"arc3,rad={rad}",
                                           shrinkA=0, shrinkB=0, zorder=5))

    def plabel(x, y, letter, title, col=INK):
        fig.text(*inch(x, y), f"({letter})", fontsize=7, va="top", ha="left", color=col, **BOLD)
        fig.text(*inch(x + 0.19, y), title, fontsize=7, va="top", ha="left", color=col)

    top = H_ - 0.03
    # ---- top: the hand (a), its finger and the interfaces it can carry (b)
    TB = H_ - 1.20  # the top band's bottom
    box(0.02, TB, W_ - 0.04, top - TB, "none", "0.55", r=0.08, lw=0.7, ls=(0, (3, 2)))
    plabel(0.07, top - 0.04, "a", "original design")
    hand = usd_render(CAD / "hand_usd_relaxed.png")
    image(hand, 0.03, TB + 0.02, fit=(1.24, top - TB - 0.14))
    # the policy learned on the original design (user, 2026-11-28): a small network badge, pi_O
    def nn_icon(x0, y0, sc, z=6):
        """A small policy network in the style the user gave (2026-11-28): 2-3-2 nodes, blue strokes, white fill."""
        NB = "#2f6db0"
        lay = (2, 3, 2)
        P = [[(x0 + li * 0.13 * sc, y0 + (k - (n - 1) / 2) * 0.11 * sc) for k in range(n)] for li, n in enumerate(lay)]
        for a_, b_ in zip(P[:-1], P[1:]):
            for p1 in a_:
                for p2 in b_:
                    fig.add_artist(plt.Line2D([p1[0] / W_, p2[0] / W_], [p1[1] / H_, p2[1] / H_], color=NB,
                                              lw=0.8 * sc, transform=fig.transFigure, zorder=z))
        for layer in P:
            for (xx, yy) in layer:
                fig.patches.append(Circle(inch(xx, yy), 0.026 * sc / W_, transform=fig.transFigure, fc="white",
                                          ec=NB, lw=1.0 * sc, zorder=z + 1))
    # the task policy learned on the original hand, beside it (user, 2026-11-28: no virtual object, no overlap)
    nn_icon(1.24, top - 0.36, 0.75, z=9)
    fig.text(*inch(1.34, top - 0.25), "task policy $\\pi_O$", fontsize=5.6, ha="center", va="bottom", color=INK)
    rx = 1.55
    box(rx, top - 0.47, 1.86, 0.43, "#eaf1fa", "#b9cce4", r=0.05, lw=0.6)
    fin = crop_white(np.asarray(Image.open(CAD / "finger_side.png").convert("RGB"))[:, 705:])
    image(white_to_alpha(fin, "#eaf1fa"), rx + 0.06, top - 0.43, fit=(1.05, 0.36))
    fig.text(*inch(rx + 1.46, top - 0.26), "crossed\nfour-bar\nfinger", fontsize=5.8, ha="center", va="center",
             color=INK2, linespacing=1.0)
    fig.text(*inch(rx + 0.93, top - 0.54), "\u2193", fontsize=10, ha="center", va="center", color="#8a8a87", **BOLD)
    box(rx, TB + 0.04, 1.86, top - 0.61 - TB - 0.04, "#f6f1e6", "#dccfb3", r=0.05, lw=0.6)
    plabel(rx + 0.04, top - 0.64, "b", "new contact interface")
    names = (("nari", "tactile"), ("honeycomb", "compliant"), ("traction", "traction"), ("soft", "gel pad"))
    for k, (kk, nm) in enumerate(names):
        cx, cy = rx + 0.08 + k * 0.45, TB + 0.16
        ax = fig.add_axes((*inch(cx, cy), 0.34 / W_, 0.22 / H_))
        fit_icon(ax, kk)
        fig.text(*inch(cx + 0.17, cy - 0.01), nm, fontsize=5.2, ha="center", va="top", color=INK2)
    # ---- (c) the principle: a thicker pad, two ways to redesign; what each keeps of O's motion (simulated fingers,
    # 3 -> 7 mm): the contact surfaces' path c(q) in the proximal pad's frame (the contact Jacobian) or the skeleton's
    # (the fingertip on the link line). Data: design/output/paper/fig1_paths_ORU.json (fig1_paths.py)
    TEAL_ = "#3F9AA0"
    paths = json.load(open(ROOT / "design" / "output" / "paper" / "fig1_paths_ORU.json"))
    qd = np.linspace(0.0, 120.0, len(paths["O"]["contact"]))
    yc = TB - 0.06
    plabel(0.07, yc, "c", "re-design optimization")
    # illustrative (user, 2026-11-28: a schematic like a design-concept figure, not a plot): O's finger dashed, the
    # redesign solid, flexed to one pose, differences exaggerated. CoDe: thicker interfaces whose contact faces sit where
    # O's do, the skeleton (links and fingertip) moved; Coverage: the contact faces themselves moved.
    def finger(ax, col, ls, lw, pad_t, skel=(0.0, 0.0), face=(0.0, 0.0), q=np.radians(62), fill=True):
        """skel / face: (shift down of the link line, shift of the pole along x) for the skeleton and for the contact
        faces. A redesign that keeps the contact motion keeps face = (0, 0) and moves skel; the faces' pads are drawn
        thick (pad_t) above the face."""
        L1, L2 = 40.0, 30.0
        R = np.array([[np.cos(-q), -np.sin(-q)], [np.sin(-q), np.cos(-q)]])
        nrm_d = R @ np.array([0.0, 1.0])
        # contact faces (proximal at depth 3, distal at depth 3 in the distal frame), from the face pole
        fdz, fdx = face
        pf = np.array([L1 + fdx, -1.5 - fdz])
        f0 = -3.0 - fdz
        d0 = pf + R @ np.array([4.0, -1.5])
        d1 = pf + R @ np.array([L2 - 1.0, -1.5])
        if fill:
            ax.add_patch(Rectangle((4, f0), 32, pad_t, fc=col, ec="none", alpha=0.25, zorder=2))
            ax.add_patch(Polygon([d0, d1, d1 + pad_t * nrm_d, d0 + pad_t * nrm_d], fc=col, ec="none", alpha=0.25,
                                 zorder=2))
        ax.plot([4, 36], [f0, f0], color=col, lw=lw, ls=ls, zorder=4)
        ax.plot(*np.array([d0, d1]).T, color=col, lw=lw, ls=ls, zorder=4)
        # skeleton: link lines and fingertip, from the skeleton pole
        sdz, sdx = skel
        ps = np.array([L1 + sdx, sdz - 1.5])
        tip = ps + R @ np.array([L2, 0.0])
        ax.plot([0, L1], [sdz, sdz], color=col, lw=lw * 0.8, ls=ls, solid_capstyle="round", zorder=3)
        ax.plot(*np.array([ps, tip]).T, color=col, lw=lw * 0.8, ls=ls, solid_capstyle="round", zorder=3)
        ax.plot(*tip, marker="o", ms=2.2, color=col, zorder=5)
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    from matplotlib.colors import LinearSegmentedColormap
    lax = fig.add_axes((0.05 / W_, (yc - 1.10) / H_, 1.55 / W_, 1.04 / H_), projection="3d")
    gx, gy = np.meshgrid(np.linspace(-1, 1, 60), np.linspace(-1, 1, 60))
    zz = (0.55 + 0.25 * (gx ** 2 + gy ** 2) - 0.55 * np.exp(-((gx - 0.45) ** 2 + (gy + 0.35) ** 2) / 0.10)
          - 0.28 * np.exp(-((gx + 0.50) ** 2 + (gy - 0.40) ** 2) / 0.12))
    cm = LinearSegmentedColormap.from_list("land", ["#0b3d91", "#4f86c6", "#bcd3ee", "#f6e7c8", "#f2b37b", "#d9603b"])
    lax.computed_zorder = False  # the path, the minima and their labels drawn over the surface
    lax.plot_surface(gx, gy, zz, cmap=cm, linewidth=0, antialiased=True, alpha=0.92, rstride=1, cstride=1, zorder=1)
    # gradient descent from O's design (a thicker interface on it) into the contact-preserving basin
    path = [np.array([-0.85, -0.80])]
    for _ in range(40):
        x_, y_ = path[-1]
        e1 = np.exp(-((x_ - 0.45) ** 2 + (y_ + 0.35) ** 2) / 0.10)
        e2 = np.exp(-((x_ + 0.50) ** 2 + (y_ - 0.40) ** 2) / 0.12)
        gxd = 0.5 * x_ + 0.55 * e1 * 2 * (x_ - 0.45) / 0.10 + 0.28 * e2 * 2 * (x_ + 0.50) / 0.12
        gyd = 0.5 * y_ + 0.55 * e1 * 2 * (y_ + 0.35) / 0.10 + 0.28 * e2 * 2 * (y_ - 0.40) / 0.12
        path.append(path[-1] - 0.06 * np.array([gxd, gyd]) + np.array([0.03, 0.015]) * (len(path) < 14))
    path = np.array(path)
    pz = (0.55 + 0.25 * (path[:, 0] ** 2 + path[:, 1] ** 2) - 0.55 * np.exp(-((path[:, 0] - 0.45) ** 2 + (path[:, 1] + 0.35) ** 2) / 0.10)
          - 0.28 * np.exp(-((path[:, 0] + 0.50) ** 2 + (path[:, 1] - 0.40) ** 2) / 0.12)) + 0.02
    lax.plot(path[:, 0], path[:, 1], pz, color="#fff3d6", lw=1.4, zorder=10)
    lax.scatter([path[0, 0]], [path[0, 1]], [pz[0]], color="#4D4D4D", s=8, zorder=11, depthshade=False)
    lax.scatter([path[-1, 0]], [path[-1, 1]], [pz[-1]], color=BLUE_, s=14, zorder=11, depthshade=False)
    fig.text(*inch(0.10, yc - 0.20), "O + new interface", fontsize=5.0, color=INK, ha="left", va="top")
    lax.text(path[-1, 0] + 0.05, path[-1, 1], pz[-1] - 0.25, "CoDe", fontsize=6.0, color=BLUE_, ha="left", zorder=12, **BOLD)
    zc = 0.55 + 0.25 * (0.5 ** 2 + 0.4 ** 2) - 0.28 - 0.55 * np.exp(-((-0.95) ** 2 + 0.75 ** 2) / 0.10)
    lax.scatter([-0.50], [0.40], [zc + 0.02], color=TEAL_, s=12, zorder=11, depthshade=False)
    lax.text(-0.50, 0.40, zc + 0.22, "Skeleton", fontsize=5.6, color=TEAL_, ha="center", zorder=12, **BOLD)
    lax.set_axis_off(); lax.view_init(elev=32, azim=-58); lax.set_box_aspect((1, 1, 0.55), zoom=1.45)
    fig.patches.append(FancyArrowPatch(inch(1.62, yc - 0.56), inch(1.78, yc - 0.56), transform=fig.transFigure,
                                       arrowstyle="-|>", mutation_scale=7, lw=0.8, color="#8a8a87", zorder=3))
    # the distal pad's contact-point path seen from the proximal pad, overlaid on O's (user, 2026-11-28: data, not a
    # schematic); design/output/paper/fig1_paths_ORU.json, 3 -> 7 mm, q = 0..120 deg
    # a finger at three flexions, from the data (fig1_paths_ORU.json): bones dark grey, interfaces filled; the tracked
    # quantity is the distal pad's surface seen from the proximal pad, with O's dashed (user, 2026-11-28: no numbers)
    DEPTH = {"O": 3.0, "mrmid_free": 4.0, "conv_e25": 7.0}  # contact-surface depth below the link line (3 + Delta t)
    BONE = "#5b5b59"
    for j, (key, nm, col, verdict) in enumerate((("mrmid_free", "CoDe", BLUE_, "pad surface tracks O's"),
                                                  ("conv_e25", "Skeleton", TEAL_, "skeleton tracks O's,\npad surface drifts"))):
        ax = fig.add_axes(((1.80 + j * 0.84) / W_, (yc - 1.00) / H_, 0.82 / W_, 0.94 / H_))
        co, ch = np.array(paths["O"]["contact"]), np.array(paths[key]["contact"])
        dep = DEPTH[key]
        pt = 0.75 * dep  # the interface drawn between its surface and the bone
        LP = 24.0
        # proximal phalanx: the interface on the pad surface (y = 0), the bone above it
        ax.add_patch(Rectangle((8, 0), 44, pt, fc=col, ec="none", alpha=0.35, zorder=2))
        ax.plot([8, 52], [0, 0], color=col, lw=1.4, zorder=4)
        # proximal phalanx as a rigid triangle (as Fig. 2(b)): J-A on the link line, A' above
        PJ, PA, PAp = np.array([0.0, dep]), np.array([44.0, dep]), np.array([36.0, dep + 13.0])
        ax.add_patch(Polygon([PJ, PA, PAp], fc="#e2e2e0", ec=BONE, lw=0.9, zorder=3))
        for n_, k_ in enumerate((0, 60, 120)):
            qr = np.radians(k_)
            d_ = np.array([np.cos(qr), -np.sin(qr)])
            nn_ = np.array([np.sin(qr), np.cos(qr)])
            a_ = 0.45 + 0.275 * n_
            c1, c0 = ch[k_], ch[k_] - LP * d_
            ax.add_patch(Polygon([c0, c1, c1 + pt * nn_, c0 + pt * nn_], fc=col, ec="none", alpha=0.35 * a_, zorder=2))
            ax.plot(*np.array([c0, c1]).T, color=col, lw=1.4, alpha=a_, solid_capstyle="butt", zorder=4)
            # distal phalanx as a rigid triangle on its bone line, B at the joint end, B' above, C at the tip
            DB, DC = c1 - 40.0 * d_ + dep * nn_, c1 + 3 * d_ + dep * nn_
            DBp = DB + 0.22 * (DC - DB) + 10.0 * nn_
            ax.add_patch(Polygon([DB, DBp, DC], fc="#e2e2e0", ec=BONE, lw=0.9, alpha=a_, zorder=3))
            if k_ == 60:  # the crossed links at one pose: A'-B and A-B'
                for p1, p2 in ((PAp, DB), (PA, DBp)):
                    ax.plot(*np.array([p1, p2]).T, color=INK, lw=0.8, zorder=4)
                for pt_ in (PA, PAp):
                    ax.plot(*pt_, marker="o", ms=1.8, color=INK, zorder=6)
                for pt_ in (DB, DBp):
                    ax.plot(*pt_, marker="o", ms=1.8, mfc="white", mec=INK, mew=0.5, zorder=6)
            o1 = co[k_]
            if key == "conv_e25":  # Skeleton is judged on O's links: O's distal triangle, dashed
                OB, OC = o1 - 40.0 * d_ + DEPTH["O"] * nn_, o1 + 3 * d_ + DEPTH["O"] * nn_
                ax.plot(*np.array([OB, OC]).T, color=INK, lw=0.9, ls=(0, (2.0, 1.4)), zorder=5)  # O's distal link only
            else:  # CoDe is judged on O's distal pad surface, dashed
                ax.plot(*np.array([o1 - LP * d_, o1]).T, color=INK, lw=0.8, ls=(0, (2.0, 1.4)), zorder=5)
        ax.set_xlim(-4, 100); ax.set_ylim(-46, 36); ax.set_aspect("equal"); ax.axis("off")
        ax.text(50, 36, nm, fontsize=6.2, color=col, ha="center", va="top", **BOLD)
        ax.text(50, -40, verdict, fontsize=5.2, color=col, ha="center", va="top", linespacing=1.1)
    fig.text(*inch(2.55, yc - 1.00), "dashed: O's distal pad (CoDe), O's distal link (Skeleton)", fontsize=4.8,
             color=INK2, ha="center", va="top")
    fig.text(*inch(W_ / 2, yc - 1.12), "CoDe keeps the contact surfaces' motion and lets the skeleton change",
             fontsize=5.8, ha="center", va="top", color=BLUE_, **BOLD)
    # ---- (d) (user, 2026-11-28): a policy learned on O (a neural network pi) runs unchanged on each redesign with the
    # thicker interface; on CoDe it keeps working, so the interface can be evaluated through it. Finger walk, Isaac Lab,
    # 7 mm, the share of O's performance kept (report/study4/data/rl_transfer.json, mean of the training seeds).
    yd = yc - 1.24
    plabel(0.07, yd, "d", "policy transfer")
    rl = json.load(open(ROOT / "report" / "study4" / "data" / "rl_transfer.json"))["tasks"]["walk"]["ratio"]
    kept = {"O": 1.0, "MRM": float(np.mean(rl["MRM"])), "U": float(np.mean(rl["U"]))}
    # the policy: a small neural network, labelled pi_O, then the task it runs (the finger-walk render), then the hands
    nx0, ny0 = 0.12, yd - 0.76
    nn_icon(nx0 + 0.02, ny0 + 0.26, 0.95, z=2)
    fig.text(*inch(0.08, ny0 + 0.50), "same task policy $\\pi_O$", fontsize=5.8, ha="left", va="bottom",
             color=INK)
    walk = usd_render(ROOT / "paper" / "figures" / "cad" / "rl_cube_O.png")
    wax = fig.add_axes((0.50 / W_, (yd - 0.88) / H_, 0.62 / W_, 0.66 / H_))
    wax.imshow(walk); wax.set_aspect("equal"); wax.axis("off")
    fig.patches.append(FancyArrowPatch(inch(nx0 + 0.33, ny0 + 0.26), inch(0.52, ny0 + 0.26), transform=fig.transFigure,
                                       arrowstyle="-|>", mutation_scale=7, lw=0.8, color="#8a8a87", zorder=3))
    # illustrative trajectories (user, 2026-11-28: a schematic, not data): the object's position along the pads under
    # the same policy; CoDe follows O's walk, Coverage leaves it
    fig.patches.append(FancyArrowPatch(inch(1.12, ny0 + 0.26), inch(1.30, ny0 + 0.26), transform=fig.transFigure,
                                       arrowstyle="-|>", mutation_scale=7, lw=0.8, color="#8a8a87", zorder=3))
    tax = fig.add_axes((1.42 / W_, (yd - 0.84) / H_, 1.90 / W_, 0.68 / H_))
    tt = np.linspace(0, 1, 400)
    # cube rotation over time: O turns it steadily, CoDe nearly as fast, Skeleton falls behind and stalls
    wig = lambda a, f: a * np.sin(f * tt) * tt
    xo = 0.10 + 0.80 * tt + wig(0.015, 23)
    xc = 0.10 + 0.72 * tt + wig(0.02, 19)
    xu = 0.10 + 0.32 * (1 - np.exp(-tt / 0.35)) + wig(0.015, 17)
    tax.plot(tt, xo, color="#4D4D4D", lw=1.0, ls=(0, (3, 2)), label="O")
    tax.plot(tt, xc, color=BLUE_, lw=1.4, label="CoDe")
    tax.plot(tt, xu, color=TEAL_, lw=1.4, label="Skeleton")
    tax.set_xticks([]); tax.set_yticks([]); tax.set_xlim(0, 1); tax.set_ylim(0.05, 0.95)
    for sp in ("top", "right"):
        tax.spines[sp].set_visible(False)
    tax.spines["left"].set_linewidth(0.5); tax.spines["bottom"].set_linewidth(0.5)
    tax.set_xlabel("time", fontsize=5.6, labelpad=1); tax.set_ylabel("cube rotation", fontsize=5.6, labelpad=1)
    tax.text(0.90, xo[-1] + 0.02, "O", fontsize=6.0, color="#4D4D4D", ha="right", va="bottom", **BOLD)
    tax.text(0.62, xc[248] + 0.08, "CoDe", fontsize=6.2, color=BLUE_, ha="right", va="bottom", **BOLD)
    tax.text(0.98, xu[-1] - 0.06, "Skeleton", fontsize=6.2, color=TEAL_, ha="right", va="top", **BOLD)
    fig.text(*inch(W_ / 2, 0.03), "On CoDe the policy keeps turning the cube; on Skeleton it falls behind", fontsize=5.8,
             ha="center", va="bottom", color=BLUE_, **BOLD)
    DEST.mkdir(parents=True, exist_ok=True)
    out = save_figure(fig, DEST / "fig_cad.pdf")
    print(out, f"{W_} x {H_} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
