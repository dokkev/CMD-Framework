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


def usd_render(path):
    """The Newton render with its uniform dark background blended to white and cropped to the hand."""
    from PIL import Image
    im = np.asarray(Image.open(path).convert("RGB")).astype(float)
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


HAND_C = {"O": "#3a3a3a", "R": "#1f6fd1", "U": "#d9622b"}


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
    ax.add_patch(Circle(pole, 0.17, fc="#8a3ffc", ec="white", lw=0.4, zorder=6))

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
        ax.imshow(np.asarray(Image.open(CAD / "nari_touch.png").convert("RGB"))); ax.axis("off")
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
    """Fig. 1 (user layout, 2026-11-24), page width. Top: the CoDe Hand (Isaac Lab USD, render_grasp.py), one finger
    from the CAD with its pad seats dashed, and the contact interfaces. Middle: the redesign problem, 1. a thicker
    interface t', 2. the finger re-optimized (Eq. (opt2) of the paper), branching into R (keeps Delta_contact small)
    and U (coverage only). Bottom: R and U from their Isaac Lab visual USD closed on a 15 mm cylinder
    (render_contact.py), the distal contact point's path against the 3 mm reference (fig1_paths.py), and their
    numbers (report/study3/data/isaac_dose_A_all.json)."""
    import json
    from PIL import Image
    from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Ellipse, Circle as Ci
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
    W_, H_ = 3.45, 6.30
    fig = plt.figure(figsize=(W_, H_))
    d = json.load(open(ROOT / "report" / "study3" / "data" / "isaac_dose_A_all.json"))["hands"]
    paths = json.load(open(ROOT / "design" / "output" / "paper" / "fig1_paths_ORU.json"))
    conc = json.load(open(ROOT / "report" / "study3" / "data" / "isaac_conclusion_study3.json"))["hands"]
    BLUE_, ORANGE_ = HAND_C["R"], HAND_C["U"]
    LBLUE, LORANGE = "#eaf1fb", "#fdf0e7"

    def inch(x, y):
        return x / W_, y / H_

    def box(x, y, w, h, fc, ec, r=0.06, lw=0.8, z=-1):
        fig.patches.append(FancyBboxPatch(inch(x, y), w / W_, h / H_, boxstyle=f"round,pad=0,rounding_size={r / W_}",
                                          transform=fig.transFigure, fc=fc, ec=ec, lw=lw, zorder=z))

    # plain matplotlib look: panel letters, thin black lines, no filled boxes
    def plabel(x, y, letter, title):
        fig.text(*inch(x, y), f"({letter})", fontsize=7.5, va="top", ha="left", **BOLD)
        fig.text(*inch(x + 0.22, y), title, fontsize=7.5, va="top", ha="left")

    # ---------------- single column, top to bottom
    top = H_ - 0.03
    hand = usd_render(CAD / "hand_usd_relaxed.png")  # a relaxed pose, no object (render_grasp.py --object none)
    hw = 1.30
    hh = hw * hand.shape[0] / hand.shape[1]
    plabel(0.02, top, "a", "CoDe Hand")
    ax = fig.add_axes((*inch(0.02, top - 0.17 - hh), hw / W_, hh / H_)); ax.imshow(hand); ax.axis("off")
    CROP = 705
    fin = np.asarray(Image.open(CAD / "finger_side.png").convert("RGB"))[:, CROP:]
    fw = 1.98
    fh = fw * fin.shape[0] / fin.shape[1]
    plabel(1.40, top, "b", "crossed four-bar joint")
    f = fig.add_axes((*inch(1.44, top - 0.30 - fh), fw / W_, fh / H_)); f.imshow(fin); f.axis("off")
    for cx, cy, ew, eh in ((1250 - CROP, 530, 450, 190), (2215 - CROP, 485, 340, 220)):
        f.add_patch(Ellipse((cx, cy), ew, eh, fc="none", ec="k", lw=0.7, ls="--", clip_on=False, zorder=5))
    fig.text(*inch(1.44 + fw / 2, top - 0.34 - fh), "dashed: pad seats", fontsize=6.0, ha="center", va="top")
    y = top - 0.20 - max(hh, fh + 0.25)
    plabel(0.02, y, "c", "contact interfaces")
    names = (("nari", "tactile\n(NARI-Touch)"), ("honeycomb", "compliant\nstructure"), ("traction", "traction\npad"),
             ("soft", "solid\nsoft pad"))
    iw = 0.70
    for k, (kk, nm) in enumerate(names):
        x = 0.12 + k * (iw + 0.15)
        ax = fig.add_axes((*inch(x, y - 0.68), iw / W_, 0.50 / H_))
        fit_icon(ax, kk)
        fig.text(*inch(x + iw / 2, y - 0.71), nm, fontsize=6.0, ha="center", va="top", linespacing=1.0)
    # (d) the redesign problem
    y -= 1.05
    fig.lines.append(plt.Line2D([0.02 / W_, (W_ - 0.02) / W_], [(y + 0.05) / H_] * 2, transform=fig.transFigure,
                                color="0.6", lw=0.5))
    plabel(0.02, y, "d", "thicker pad, re-optimized finger")
    chg = np.asarray(Image.open(CAD / "distal_pad_change.png").convert("RGB"))
    cw_ = 0.80
    ch_ = cw_ * chg.shape[0] / chg.shape[1]
    ca = fig.add_axes((*inch(0.05, y - 0.20 - ch_), cw_ / W_, ch_ / H_)); ca.imshow(chg); ca.axis("off")
    fig.patches.append(FancyArrowPatch(inch(0.90, y - 0.42), inch(1.10, y - 0.42), transform=fig.transFigure,
                                       arrowstyle="->", mutation_scale=8, lw=0.8, color="k"))
    eqR = r"$\min_{M\in\mathcal{F}}\;(\Delta_\mathrm{contact}(M,M_0),\,-\bar{C}_\mathrm{pad}(M))$"
    eqU = r"$\max_{M\in\mathcal{F}}\;\bar{C}_\mathrm{pad}(M)$"
    for k, (lab, col, eq) in enumerate((("R", BLUE_, eqR), ("U", ORANGE_, eqU))):
        yy = y - 0.30 - k * 0.24
        fig.text(*inch(1.15, yy), f"{lab}:", fontsize=7, color=col, ha="left", va="center", **BOLD)
        fig.text(*inch(1.35, yy), eq, fontsize=6.8, ha="left", va="center", math_fontfamily="stixsans")
    # (e) the task: a fingertip turns a sprung dial (report/study3/figures/fingertip_disk.png)
    y -= 0.82
    fig.lines.append(plt.Line2D([0.02 / W_, (W_ - 0.02) / W_], [(y + 0.05) / H_] * 2, transform=fig.transFigure,
                                color="0.6", lw=0.5))
    plabel(0.02, y, "e", "tasks")
    tasks = (("task_hold", "hold (pull-out)"), ("task_dial", "dial turning"), ("task_coin", "coin drag"),
             ("task_inhand", "in-hand rotation (RL)"))
    tw, th_ = 0.80, 0.62
    BG = np.array([236, 237, 239], float)  # one light-grey backdrop for every task image
    for k, (fn, nm) in enumerate(tasks):
        im_ = np.asarray(Image.open(CAD / f"{fn}.png").convert("RGB")).astype(float)
        if fn != "task_inhand":
            ink = im_.min(axis=2) < 240
            rr, cc_ = np.nonzero(ink)
            im_ = im_[rr.min():rr.max() + 1, cc_.min():cc_.max() + 1]
            a_ = np.clip((250 - im_.min(axis=2)) / 15.0, 0, 1)[..., None]  # white -> backdrop
            im_ = a_ * im_ + (1 - a_) * BG
        im_ = im_.astype(np.uint8)
        # pad to the cell's aspect with the backdrop, so every cell is the same grey rectangle
        H0, W0 = im_.shape[:2]
        want = tw / th_
        if W0 / H0 < want:
            padw = int(round(H0 * want)) - W0
            im_ = np.pad(im_, ((0, 0), (padw // 2, padw - padw // 2), (0, 0)), constant_values=0)
            im_[:, :padw // 2] = BG; im_[:, W0 + padw // 2:] = BG
        else:
            padh = int(round(W0 / want)) - H0
            im_ = np.pad(im_, ((padh // 2, padh - padh // 2), (0, 0), (0, 0)), constant_values=0)
            im_[:padh // 2] = BG; im_[H0 + padh // 2:] = BG
        x = 0.04 + k * (tw + 0.05)
        ta = fig.add_axes((*inch(x, y - 0.20 - th_), tw / W_, th_ / H_)); ta.imshow(im_, aspect="auto"); ta.axis("off")
        fig.text(*inch(x + tw / 2, y - 0.23 - th_), nm, fontsize=5.8, ha="center", va="top")
    sh_ = th_ + 0.10
    # (f, g) the two redesigns: measured pad effect against O's, and the contact surface's offset from O
    y -= 0.30 + sh_
    ref = np.array(paths["O"]["contact"])
    for i, (kind, col, letter, title) in enumerate((("R", BLUE_, "f", "contact-preserving redesign (R)"),
                                                    ("U", ORANGE_, "g", "coverage-only redesign (U)"))):
        y0 = y - i * 0.98
        fig.lines.append(plt.Line2D([0.02 / W_, (W_ - 0.02) / W_], [(y0 + 0.05) / H_] * 2, transform=fig.transFigure,
                                    color="0.6", lw=0.5))
        plabel(0.02, y0, letter, title)
        cc = conc[kind]
        ba = fig.add_axes((*inch(0.32, y0 - 0.70), 1.15 / W_, 0.42 / H_))
        ba.barh([1, 0], [cc["true_change_pct"], cc["measured_change_pct"]], height=0.6, color=["0.6", col])
        ba.axvline(0, color="k", lw=0.6)
        ba.set_yticks([1, 0]); ba.set_yticklabels(["O", kind], fontsize=5.8)
        ba.set_xlim(-80, 80); ba.set_xticks([]); ba.set_ylim(-0.6, 1.6)
        ba.set_xlabel("pull-out change, softer pad", fontsize=5.6, labelpad=2)
        ba.tick_params(length=0, pad=2)
        for sp in ("top", "right"):
            ba.spines[sp].set_visible(False)
        ba.set_title("same sign" if kind == "R" else "sign reversed", fontsize=6.0, color=col, pad=2)
        da = fig.add_axes((*inch(2.05, y0 - 0.70), 1.30 / W_, 0.42 / H_))
        red = np.array(paths[kind]["contact"])
        dev = np.linalg.norm(red - ref, axis=1)
        qd = np.linspace(0, 120, len(dev))
        da.plot(qd, dev, color=col, lw=1.1)
        da.axhline(1.0, color="0.5", lw=0.6, ls=":")
        da.set_xlim(0, 120); da.set_ylim(0, 4.5); da.set_xticks([]); da.set_yticks([])
        for sp in ("top", "right"):
            da.spines[sp].set_visible(False)
        da.set_xlabel("flexion", fontsize=5.6, labelpad=2)
        da.set_ylabel("offset\nfrom O", fontsize=5.6, labelpad=2, linespacing=1.0)
        da.set_title("contact surface", fontsize=6.0, pad=2)
        words = {"R": "skeleton changed more, contact kept: reduced confounding",
                 "U": "skeleton changed less, contact moved: confounded"}[kind]
        fig.text(*inch(W_ / 2, y0 - 0.92), words, fontsize=6.0, ha="center", va="bottom", color=col)
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
