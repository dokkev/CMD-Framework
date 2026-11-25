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


def icon(ax, kind):
    """A cross-section sketch of one contact interface on its module base (illustrative, not to scale)."""
    from matplotlib.patches import Rectangle, Polygon, FancyBboxPatch
    ax.set_xlim(0, 10); ax.set_ylim(-5.2, 1.6); ax.set_aspect("equal"); ax.axis("off")
    ax.add_patch(Rectangle((0.5, 0.0), 9.0, 1.0, fc="#c9cdd2", ec=INK2, lw=0.5))  # module base on the phalanx
    if kind == "honeycomb":  # compliant pad: hexagonal lattice, front face with its cells, extruded (oblique)
        from matplotlib.patches import Polygon as Pg
        x0, x1, y0, y1, r, wall = 0.7, 9.3, -3.6, -0.05, 0.55, 0.16
        d = np.array([0.8, 0.55])
        oblique(ax, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)], depth=d, fc="#6f9fe0", ec=PAD, top="#a9c4ee",
                side="#4f7fc4", lw=0.3, z=1)
        w_, h_ = 1.5 * r, np.sqrt(3) * r
        ang = np.radians(np.arange(0, 360, 60))
        from shapely.geometry import Polygon as SP, box
        frame = box(x0 + wall, y0 + wall, x1 - wall, y1 - wall)
        for i in range(int((x1 - x0) / w_) + 2):
            for j in range(int((y1 - y0) / h_) + 2):
                cx = x0 + i * w_
                cy = y0 + j * h_ + (h_ / 2 if i % 2 else 0)
                cellp = SP(np.c_[cx + (r - wall / 2) * np.cos(ang), cy + (r - wall / 2) * np.sin(ang)])
                hole = cellp.intersection(frame)
                if hole.is_empty or hole.area < 0.05:
                    continue
                xy = np.array(hole.exterior.coords)
                ax.add_patch(Pg(xy, fc="#2f5fa8", ec="none", zorder=1.2))           # cell interior (shadow)
                inner = SP(xy + d * 0.55).intersection(hole)                          # far end of the cell, lit
                if not inner.is_empty and inner.geom_type == "Polygon":
                    ax.add_patch(Pg(np.array(inner.exterior.coords), fc="#dbe6f6", ec="none", zorder=1.3))
        return
    if kind == "traction":  # black traction pad with a tread
        pts = [(0.5, -2.0), (9.5, -2.0), (9.5, 0.0), (0.5, 0.0)]
        oblique(ax, pts, depth=(1.0, 0.7), fc="#1d1d1d", ec="#000000", top="#3a3a3a", side="#2a2a2a", lw=0.3)
        for x in np.arange(0.9, 9.3, 0.8):  # transverse ribs
            ax.add_patch(Rectangle((x, -2.45), 0.45, 0.45, fc="#111111", ec="#000000", lw=0.2, zorder=2))
        return
    if kind == "thin":
        ax.add_patch(Rectangle((0.5, -0.8), 9.0, 0.8, fc="#5b7fc7", ec=PAD, lw=0.5))
    elif kind == "thick":
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
    width, height = 3.45, 3.62
    fig = plt.figure(figsize=(width, height))
    d = json.load(open(ROOT / "report" / "study3" / "data" / "isaac_dose_A_all.json"))["hands"]

    def inch(x, y):
        return x / width, y / height
    # top: the whole hand manipulating a cube (Isaac Lab visual USD, render_grasp.py) and one finger from the CAD
    hand = usd_render(CAD / "usd_hand_cube.png")
    hw = 1.30
    hh = hw * hand.shape[0] / hand.shape[1]
    fig.text(*inch(0.02, height - 0.02), "CoDe Hand", fontsize=6.8, fontweight="bold", va="top")
    ha = fig.add_axes((*inch(0.02, height - 0.16 - hh), hw / width, hh / height))
    ha.imshow(hand); ha.axis("off")
    fin = np.asarray(Image.open(CAD / "finger_side.png").convert("RGB"))
    fw = 2.05
    fh = fw * fin.shape[0] / fin.shape[1]
    fy = height - 0.30 - fh
    f = fig.add_axes((*inch(1.38, fy), fw / width, fh / height))
    f.imshow(fin); f.axis("off")
    fig.text(*inch(1.38, height - 0.02), "one finger (CAD): crossed four-bar PIP,\nremote PIP servo", fontsize=6.3,
             fontweight="bold", va="top", linespacing=1.0)
    x0, x1, yb = 880, 2000, 690
    f.plot([x0, x0, x1, x1], [yb - 50, yb, yb, yb - 50], color=PAD, lw=0.8, clip_on=False)
    f.text((x0 + x1) / 2, yb + 30, "room for thick interfaces\non both phalanges", color=PAD, fontsize=6.0,
           ha="center", va="top", clip_on=False, linespacing=1.0)
    fy = min(fy - 0.30, height - 0.16 - hh)
    # two arrows into the two redesigns
    top = fy - 0.12
    for xe, col in ((0.95, HAND_C["R"]), (2.50, HAND_C["U"])):
        fig.patches.append(FancyArrowPatch(inch(1.73, top), inch(xe, top - 0.30), transform=fig.transFigure,
                                           arrowstyle="-|>", mutation_scale=8, lw=1.2, color=col,
                                           connectionstyle="arc3,rad=0.0"))
    fig.text(*inch(1.73, top + 0.02), "thicker interface: redesign the finger around ...", fontsize=6.3, color=INK2,
             ha="center", va="bottom", style="italic")
    panels = (("R", "contact-surface motion", d["R"]),
              ("U", "skeleton (links kept)", d["U"]))
    sw, sh = 1.55, 0.95
    py = top - 0.36 - sh
    for i, (kind, name, rec) in enumerate(panels):
        px = 0.12 + i * (sw + 0.10)
        col = HAND_C[kind]
        fig.text(*inch(px + sw / 2, top - 0.33), name, fontsize=6.8, color=col, fontweight="bold", ha="center",
                 va="top")
        ax = fig.add_axes((*inch(px, py - 0.05), sw / width, sh / height))
        finger_sketch(ax, col, kind)
        verdict = "interface effect read correctly" if kind == "R" else "interface effect distorted"
        fig.text(*inch(px + sw / 2, py - 0.04),
                 f"error {rec['E_rel_median'] * 100:.0f}%: {verdict}", fontsize=6.3, color=col, ha="center",
                 va="top", fontweight="bold")
    fig.text(*inch(width / 2, py - 0.17), "purple: pole;  dashed: original contact motion", fontsize=5.6,
             color=INK2, ha="center", va="top")
    # bottom: contact interfaces to carry
    by = 0.06
    fig.text(*inch(0.02, by + 0.72), "contact interfaces to carry", fontsize=6.6, fontweight="bold", va="top")
    nari = np.asarray(Image.open(CAD / "nari_touch.png").convert("RGB"))
    nw = 0.62
    nh = nw * nari.shape[0] / nari.shape[1]
    ax = fig.add_axes((*inch(0.08, by + 0.10), nw / width, nh / height))
    ax.imshow(nari); ax.axis("off")
    fig.text(*inch(0.08 + nw / 2, by + 0.08), "NARI-Touch tactile", fontsize=5.8, ha="center", va="top")
    kinds = (("honeycomb", "honeycomb compliant"), ("traction", "traction pad"), ("thin", "thin rigid"))
    iw, ih = 0.68, 0.30
    for k, (kk, nm) in enumerate(kinds):
        x = 0.95 + k * (iw + 0.15)
        ax = fig.add_axes((*inch(x, by + 0.16), iw / width, ih / height))
        icon(ax, kk)
        fig.text(*inch(x + iw / 2, by + 0.08), nm, fontsize=5.8, ha="center", va="top")
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
