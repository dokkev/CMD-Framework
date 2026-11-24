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


def icon(ax, kind):
    """A cross-section sketch of one contact interface on its module base (illustrative, not to scale)."""
    from matplotlib.patches import Rectangle, Polygon, FancyBboxPatch
    ax.set_xlim(0, 10); ax.set_ylim(-5.2, 1.6); ax.set_aspect("equal"); ax.axis("off")
    ax.add_patch(Rectangle((0.5, 0.0), 9.0, 1.0, fc="#c9cdd2", ec=INK2, lw=0.5))  # module base on the phalanx
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
    """Fig. 1 (user layout, 2026-11-24), page width. (a) the CoDe Hand (Isaac Lab USD render); (b) what a redesign for
    thicker pads should preserve: O, a coverage-only redesign U that keeps O's links, and R that keeps the contact
    surfaces' motion (qualitative sketches; the Delta_contact values are U's and R's); (c) the error each adds to a
    measured interface effect under the reused impedance setpoint (Isaac Lab, pad friction 1.3 -> 2.0, median over O's
    placements and four load directions: report/study3/data/isaac_dose_A_all.json, E_rel_median)."""
    import json
    plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
                         "mathtext.fontset": "stix", "font.size": 7})
    width, height = 7.16, 2.30
    fig = plt.figure(figsize=(width, height))
    lab = dict(fontsize=7.5, fontweight="bold", va="top")
    # (a)
    fig.text(0.01, 0.985, "(a) CoDe Hand", **lab)
    iso = usd_render(CAD / "hand_usd_palm_up.png")
    aw = 1.95
    ah = aw * iso.shape[0] / iso.shape[1]
    a = fig.add_axes((0.03 / width, (height - 0.25 - ah) / height, aw / width, ah / height))
    a.imshow(iso); a.axis("off")
    fig.text(1.0 / width, (height - 0.30 - ah) / height, "the hand as simulated (Isaac Lab; blue: pads)", fontsize=6.3,
             color=INK2, ha="center", va="top")
    # (b)
    bx = 2.15
    fig.text(bx / width, 0.985, "(b) What should a redesign for thicker pads preserve?", **lab)
    d = json.load(open(ROOT / "report" / "study3" / "data" / "isaac_dose_A_all.json"))["hands"]
    cols = (("O", "#6a6a6a", "original (O)", "reference contact motion"),
            ("U", HAND_C["U"], "coverage only (U): links kept", f"$\\Delta_{{contact}}$ = {d['U']['D_c_mm']:.2f} mm"),
            ("R", HAND_C["R"], "contact-preserving (R)", f"$\\Delta_{{contact}}$ = {d['R']['D_c_mm']:.2f} mm"))
    sw = 1.62
    for i, (kind, col, title, foot) in enumerate(cols):
        x0 = bx + 0.05 + i * (sw + 0.05)
        ax = fig.add_axes((x0 / width, 0.86 / height, sw / width, 1.20 / height))
        finger_sketch(ax, col, kind)
        fig.text((x0 + sw / 2) / width, 1.98 / height, title, fontsize=6.6, color=col, ha="center", va="bottom",
                 fontweight="bold")
        fig.text((x0 + sw / 2) / width, 0.90 / height, foot, fontsize=6.4, color=col, ha="center", va="top")
    fig.text((bx + 0.1) / width, 0.66 / height, "purple: pole;  dashed: O's contact motion;  solid: the redesign's",
             fontsize=5.8, color=INK2, va="center")
    # (c)
    fig.text(bx / width, 0.50 / height, "(c) Error in the measured interface effect", **lab)
    ex = (("U", d["U"]["E_rel_median"]), ("R", d["R"]["E_rel_median"]))
    cx = fig.add_axes(((bx + 0.15) / width, 0.08 / height, 3.4 / width, 0.30 / height))
    for k, (h, e) in enumerate(ex):
        cx.barh(1 - k, e * 100, height=0.62, color=HAND_C[h], edgecolor="none")
        cx.text(e * 100 + 3, 1 - k, f"{e * 100:.0f}% ({h})", va="center", fontsize=6.8, color=HAND_C[h],
                fontweight="bold")
    cx.set_xlim(0, 150); cx.set_ylim(-0.5, 1.5); cx.axis("off")
    fig.text((bx + 4.0) / width, 0.23 / height, "Preserve the contact\nmotion, not the skeleton.", fontsize=7.2,
             fontweight="bold", color=INK, ha="center", va="center", linespacing=1.1)
    DEST.mkdir(parents=True, exist_ok=True)
    out = DEST / "fig_cad.png"
    fig.savefig(out, dpi=400)
    from PIL import Image
    Image.open(out).convert("RGB").save(out)  # opaque RGB, no SMask (PaperPlaza)
    print(out, f"{width} x {height} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
