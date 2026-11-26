"""Fig. 1 of the RSS 2027 extended abstract (PNG only, single column, print size). Every render is of the visual
USD used in Isaac Lab, drawn by Newton's GL viewer (sim/isaaclab/scripts/code_hand/render_visuals.py; never the Drake
renders): the hand palm up (--view palm_up -> cad/hand_usd_palm_up.png) and the fingers of O, R and U from the side
(logs/urdf/cfl_q120_h3to7_{h0,hr,hu}_thumbpad_vis_ball_colliders.usda, --view side --pip 0 60 120 ->
cad/usd_finger_<H>_q<q>.png). The dark viewer background is blended to white here.

    conda run -n cmd python paper/figures/fig_cad.py [dest_dir]

Page width (user layout, 2026-11-24): (a) the hand, (b) qualitative O / U / R sketches with the pole and the distal contact
point's path, (c) the error U and R add to a measured interface effect (Isaac Lab, isaac_dose_A_all.json).
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


def white_to_alpha(im, lo=236, hi=252):
    """RGB (uint8) to RGBA float with the near-white background faded out, so the image sits on a tinted box."""
    im = np.asarray(im)[..., :3].astype(float) / 255.0
    a = np.clip((hi / 255.0 - im.min(axis=2)) / ((hi - lo) / 255.0), 0.0, 1.0)
    return np.dstack([im, a])


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
        ax.imshow(white_to_alpha(Image.open(CAD / "nari_touch.png").convert("RGB"))); ax.axis("off")
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
    operator) -> two hands, CoDe (keeps the contact-surface motion) and Conventional (coverage only) -> what each does
    with a reused command: the measured interface effect, O for the same effect as the reference hand, X otherwise.
    Numbers: Isaac Lab softer-pad pull-out (isaac_conclusion_study3.json, 7 mm) and the Drake fingertip dial
    (fig4_sync.json, friction effect as a share of O's)."""
    import json
    from PIL import Image
    from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Circle as Ci
    from matplotlib import font_manager as _fm
    for _f in ("/usr/local/share/fonts/Helvetica Light Regular.otf",
               "/usr/share/fonts/opentype/urw-base35/NimbusSans-Bold.otf",
               "/usr/share/fonts/opentype/urw-base35/NimbusSans-Italic.otf"):
        if Path(_f).exists():
            _fm.fontManager.addfont(_f)
    plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["Helvetica Light", "Nimbus Sans"],
                         "font.weight": "light", "mathtext.fontset": "custom", "mathtext.rm": "Helvetica Light",
                         "mathtext.it": "Nimbus Sans:italic", "mathtext.bf": "Nimbus Sans:bold", "font.size": 7})
    BOLD = dict(fontweight="bold", family="Nimbus Sans")
    W_, H_ = 7.16, 2.12
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
    # ---- inputs: the hand we have and the interfaces it can carry (dashed group, as in a pipeline sketch)
    box(0.02, 0.04, 2.30, top - 0.06, "none", "0.55", r=0.08, lw=0.7, ls=(0, (3, 2)))
    plabel(0.07, top - 0.04, "a", "CoDe Hand")
    hand = usd_render(CAD / "hand_usd_relaxed.png")  # Isaac Lab visual USD, the relaxed pose
    hw_, hh_ = image(hand, 0.05, 0.10, fit=(1.30, 1.80))
    if hh_ < 1.70:  # centre the hand in the space under its label
        fig.axes[-1].set_position((0.05 / W_, (0.10 + (1.72 - hh_) / 2) / H_, hw_ / W_, hh_ / H_))
    # the finger and the interfaces it can carry, each in a light box, joined by a plus
    box(1.39, top - 0.64, 0.90, 0.60, "#eaf1fa", "#b9cce4", r=0.05, lw=0.6)
    fin = crop_white(np.asarray(Image.open(CAD / "finger_side.png").convert("RGB"))[:, 705:])
    fw, fh = image(white_to_alpha(fin), 1.43, top - 0.50, w=0.82)
    fig.text(*inch(1.84, top - 0.52), "crossed four-bar finger", fontsize=5.8, ha="center", va="top", color=INK2)
    fig.text(*inch(1.84, top - 0.74), "+", fontsize=11, ha="center", va="center", color=INK, **BOLD)
    box(1.39, 0.08, 0.90, top - 0.84 - 0.08, "#f6f1e6", "#dccfb3", r=0.05, lw=0.6)
    plabel(1.43, top - 0.87, "b", "interfaces")
    names = (("nari", "tactile"), ("honeycomb", "compliant"), ("traction", "traction"), ("soft", "soft pad"))
    for k, (kk, nm) in enumerate(names):
        cx, cy = 1.45 + (k % 2) * 0.43, top - 1.36 - (k // 2) * 0.48
        ax = fig.add_axes((*inch(cx, cy), 0.38 / W_, 0.28 / H_))
        fit_icon(ax, kk)
        fig.text(*inch(cx + 0.19, cy - 0.01), nm, fontsize=5.6, ha="center", va="top", color=INK2)
    # ---- the operator: a new interface goes on, the finger is redesigned for it
    mid = 1.06
    ox = 2.48
    fig.patches.append(Ci(inch(ox, mid), 0.085 / W_, transform=fig.transFigure, fc="white", ec=INK, lw=0.8, zorder=6))
    fig.text(*inch(ox, mid - 0.005), "+", fontsize=9, ha="center", va="center", zorder=7, **BOLD)
    arrow((2.32, mid), (ox - 0.09, mid))
    plabel(2.42, top - 0.04, "c", "redesign")
    box(2.62, mid - 0.40, 0.84, 0.80, "#eef3ea", "#8fb08a", r=0.07, lw=0.7)
    arrow((ox + 0.09, mid), (2.62, mid))
    chg = crop_white(np.asarray(Image.open(CAD / "distal_pad_change.png").convert("RGB")))
    image(chg, 2.68, mid - 0.04, w=0.72)
    fig.text(*inch(3.04, mid - 0.10), "re-optimize the finger\nfor a thicker interface", fontsize=5.8,
             ha="center", va="top", linespacing=1.05)
    # ---- the two hands it can give
    outs = (("R", "CoDe Hand", BLUE_, "keep contact-surface\nmotion", top - 0.18),
            ("U", "Conventional Hand", ORANGE_, "coverage only", 0.86))
    for kind, name, col, rule, yt in outs:
        im = usd_render(CAD / f"usd_contact_{kind}.png")[12:]
        w_, h_ = image(im, 3.86, yt - 0.80, h=0.66)
        fig.text(*inch(3.86 + w_ / 2, yt - 0.02), name, fontsize=6.6, ha="center", va="top", color=col, **BOLD)
        yc = yt - 0.45
        arrow((3.46, mid + (0.16 if kind == "R" else -0.16)), (3.83, yc), col=col, lw=1.1,
              rad=0.25 if kind == "R" else -0.25)
        fig.text(*inch(3.52, yc + (0.18 if kind == "R" else -0.14)), rule, fontsize=5.6, color=col,
                 ha="center", va="bottom" if kind == "R" else "top", linespacing=1.0)
    # ---- what each does with the same command: the interface effect it measures
    plabel(4.86, top - 0.04, "d", "same command, new interface")
    tasks = (("task_hold", "softer pad:\npull-out load"), ("task_dial", "higher friction:\ndial turned"))
    o_pull = conc["R"]["true_change_pct"]
    for r_, (kind, name, col, rule, yt) in enumerate(outs):
        yrow = yt - 0.20 - (0.0 if kind == "R" else 0.02)
        for c_, (fn, lab) in enumerate(tasks):
            x0 = 4.86 + c_ * 1.15
            im_ = crop_white(np.asarray(Image.open(CAD / f"{fn}.png").convert("RGB")))
            w_, h_ = image(im_, x0, yrow - 0.52, fit=(0.62, 0.50))
            if fn == "task_hold":
                meas = conc[kind]["measured_change_pct"]
                ok = np.sign(meas) == np.sign(o_pull)
                txt = f"{meas:+.0f}%\n(ref. {o_pull:+.0f}%)".replace("-", "\u2212")
            else:
                share = 100 * sync["dial"]["hands"][kind]["ratio_median"]
                ok = kind == "R"
                txt = f"{share:.0f}% of\nref.'s effect"
            fig.text(*inch(x0 + 0.64, yrow - 0.20), txt, fontsize=5.8, ha="left", va="top", linespacing=1.05,
                     color=INK)
            fig.text(*inch(x0 + 0.64, yrow + 0.0), "O" if ok else "X", fontsize=9, ha="left", va="top",
                     color=GREEN_ if ok else RED_, **BOLD)
            if r_ == 0:
                fig.text(*inch(x0 + 0.55, top - 0.20), lab, fontsize=5.6, ha="center", va="top", color=INK2,
                         linespacing=1.0)
        say = {"R": "measures the interface as the reference hand does",
               "U": "reverses or shrinks the interface's effect"}[kind]
        fig.text(*inch(6.00, yrow - 0.53), say, fontsize=6.0, ha="center", va="top", color=col,
                 **(BOLD if kind == "R" else {}))
        arrow((4.66, yt - 0.45), (4.82, yt - 0.45), col=col, lw=1.0)
    DEST.mkdir(parents=True, exist_ok=True)
    out = DEST / "fig_cad.png"
    fig.savefig(out, dpi=400)
    Image.open(out).convert("RGB").save(out)  # opaque RGB, no SMask (PaperPlaza)
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from pngout import trim_white
    trim_white(out)
    print(out, f"{W_} x {H_} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
