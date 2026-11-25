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
    alpha = np.clip((np.linalg.norm(im - bg, axis=2) - 8.0) / 40.0, 0.0, 1.0)[..., None]
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


def main() -> int:
    """Fig. 1 (2026-11-24, user brief): single column. Top: the CFL finger from the CAD (cad/finger_side.png), its room
    for contact interfaces across the PIP joint marked. Two arrows split into the two ways of redesigning it for a
    thicker interface: around the contact surfaces' motion (R) or around the skeleton (U, links kept); qualitative
    sketches (finger_sketch) with the error each adds to a measured interface effect (Isaac Lab,
    report/study3/data/isaac_dose_A_all.json). Bottom: contact interfaces the finger is meant to carry: the NARI-Touch
    tactile sensor from its CAD (Onshape Dex-Hand/Sensors/NARI-Touch.step -> cad/nari_touch.png) and illustrative
    pad cross-sections."""
    import json
    from PIL import Image
    from matplotlib.patches import FancyArrowPatch
    plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
                         "mathtext.fontset": "stix", "font.size": 7})
    width, height = 3.45, 4.15
    fig = plt.figure(figsize=(width, height))
    d = json.load(open(ROOT / "report" / "study3" / "data" / "isaac_dose_A_all.json"))["hands"]
    from matplotlib.patches import Ellipse

    def inch(x, y):
        return x / width, y / height
    # 1. top: the whole hand manipulating a cube (Isaac Lab USD) and one finger from the CAD, its pad seats circled
    hand = usd_render(CAD / "usd_hand_cube.png")
    hw = 1.30
    hh = hw * hand.shape[0] / hand.shape[1]
    fig.text(*inch(0.02, height - 0.02), "CoDe Hand", fontsize=6.8, fontweight="bold", va="top")
    ha = fig.add_axes((*inch(0.02, height - 0.16 - hh), hw / width, hh / height))
    ha.imshow(hand); ha.axis("off")
    fin = np.asarray(Image.open(CAD / "finger_side.png").convert("RGB"))
    fw = 2.05
    fh = fw * fin.shape[0] / fin.shape[1]
    fy = height - 0.32 - fh
    f = fig.add_axes((*inch(1.38, fy), fw / width, fh / height))
    f.imshow(fin); f.axis("off")
    fig.text(*inch(1.38, height - 0.02), "one finger (CAD): crossed four-bar PIP,\nremote PIP servo", fontsize=6.3,
             fontweight="bold", va="top", linespacing=1.0)
    seats = ((1250, 530, 450, 190), (2215, 485, 340, 220))  # proximal and distal pad seats (image px: x, y, w, h)
    for cx, cy, ew, eh in seats:
        f.add_patch(Ellipse((cx, cy), ew, eh, fc="none", ec=PAD, lw=0.9, ls=(0, (3, 2)), clip_on=False, zorder=5))
    # 2. the contact interfaces, which go into those seats
    row_top = min(fy, height - 0.16 - hh) - 0.30
    bw, bh, gap = 0.74, 0.40, 0.105
    by = row_top - bh
    fig.text(*inch(0.02, row_top + 0.16), "contact interfaces", fontsize=6.6, fontweight="bold", va="top")
    names = (("nari", "NARI-Touch Tactile"), ("honeycomb", "Compliant Structure"), ("traction", "Traction Pad"),
             ("soft", "Solid Soft Pad"))
    for k, (kk, nm) in enumerate(names):
        x = 0.08 + k * (bw + gap)
        ax = fig.add_axes((*inch(x, by), bw / width, bh / height))
        if kk == "nari":
            nari = np.asarray(Image.open(CAD / "nari_touch.png").convert("RGB"))
            ax.imshow(nari); ax.axis("off")
        else:
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
        fig.text(*inch(x + bw / 2, by - 0.03), nm, fontsize=5.8, ha="center", va="top")
    # arrow from the interfaces up into the circled seats
    seat_fig = fig.transFigure.inverted().transform(f.transData.transform((1300, 640)))
    seat2 = fig.transFigure.inverted().transform(f.transData.transform((2235, 610)))
    src = inch(2.20, row_top + 0.04)
    for tgt in (seat_fig, seat2):
        fig.patches.append(FancyArrowPatch(src, tuple(tgt), transform=fig.transFigure, arrowstyle="-|>",
                                           mutation_scale=7, lw=0.9, color=PAD, connectionstyle="arc3,rad=0.0"))
    fig.text(*inch(2.12, row_top + 0.08), "go into the circled seats", fontsize=5.8, color=PAD, style="italic",
             va="center", ha="right")
    # 3. design reoptimization for a thicker interface: around the contact-surface motion (R) or the skeleton (U)
    top = by - 0.30
    fig.text(*inch(1.73, top + 0.02), "design reoptimization for a thicker interface, around ...", fontsize=6.3,
             color=INK2, ha="center", va="bottom", style="italic")
    for xe, col in ((0.95, HAND_C["R"]), (2.50, HAND_C["U"])):
        fig.patches.append(FancyArrowPatch(inch(1.73, top), inch(xe, top - 0.28), transform=fig.transFigure,
                                           arrowstyle="-|>", mutation_scale=8, lw=1.2, color=col))
    panels = (("R", "contact-surface motion", d["R"]), ("U", "skeleton (links kept)", d["U"]))
    sw, sh = 1.55, 0.95
    py = top - 0.34 - sh
    for i, (kind, name, rec) in enumerate(panels):
        px = 0.12 + i * (sw + 0.10)
        col = HAND_C[kind]
        fig.text(*inch(px + sw / 2, top - 0.31), name, fontsize=6.8, color=col, fontweight="bold", ha="center",
                 va="top")
        ax = fig.add_axes((*inch(px, py - 0.05), sw / width, sh / height))
        finger_sketch(ax, col, kind)
        verdict = "interface effect read correctly" if kind == "R" else "interface effect distorted"
        fig.text(*inch(px + sw / 2, py - 0.04), f"error {rec['E_rel_median'] * 100:.0f}%: {verdict}", fontsize=6.3,
                 color=col, ha="center", va="top", fontweight="bold")
    fig.text(*inch(width / 2, py - 0.17), "purple: pole;  dashed: original contact motion", fontsize=5.6,
             color=INK2, ha="center", va="top")
    DEST.mkdir(parents=True, exist_ok=True)
    out = DEST / "fig_cad.png"
    fig.savefig(out, dpi=400)
    Image.open(out).convert("RGB").save(out)  # opaque RGB, no SMask (PaperPlaza)
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from pngout import trim_white
    trim_white(out)
    print(out, f"{width} x {height} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
