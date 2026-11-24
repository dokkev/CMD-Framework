"""Fig. 1 of the RSS 2027 extended abstract (PNG only, single column, print size). Every render is of the visual
USD used in Isaac Lab, drawn by Newton's GL viewer (sim/isaaclab/scripts/code_hand/render_visuals.py; never the Drake
renders): the hand palm up (--view palm_up -> cad/hand_usd_palm_up.png) and the fingers of O, R and U from the side
(logs/urdf/cfl_q120_h3to7_{h0,hr,hu}_thumbpad_vis_ball_colliders.usda, --view side --pip 0 60 120 ->
cad/usd_finger_<H>_q<q>.png). The dark viewer background is blended to white here.

    conda run -n cmd python paper/figures/fig_cad.py [dest_dir]

(c) is a qualitative schematic (no data): the same joint command and gain on R and U, O's distal contact surface dashed.
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


def schematic(ax, colour, keep):
    """Qualitative two-phalanx finger on an object (not to scale), the pads thicker than O's by DT. keep="skeleton":
    O's links and joint, so both contact surfaces move past O's into the object and the same setpoint presses
    harder; keep="contact": the links and joint offset so that the contact surfaces lie on O's, at the price of a
    different skeleton. Dashed grey: O's skeleton (link lines, joint); dashed black: O's contact surfaces."""
    from matplotlib.patches import Polygon, Circle, FancyArrowPatch
    ax.set_xlim(-0.6, 9.4); ax.set_ylim(-5.6, 2.0); ax.set_aspect("equal"); ax.axis("off")
    t0, dt = 0.45, 0.65
    j0 = np.array([5.4, 0.4])
    th = np.radians(-100.0)
    rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    n = rot @ np.array([0.0, -1.0])  # inward normal of the distal face (toward the object)
    if keep == "skeleton":
        up, joint, t = 0.0, j0, t0 + dt
    else:  # offset both phalanges outward by dt: proximal up, distal along -n; the joint moves to suit both
        m = np.linalg.solve(np.array([[0.0, 1.0], [-n[0], -n[1]]]), np.array([1.0, 1.0]))
        up, joint, t = dt, j0 + dt * m, t0 + dt
    dist = lambda pts, J: (rot @ np.asarray(pts, float).T).T + J
    # proximal phalanx and pad (face at -t0 + up - ... : bottom of the pad)
    face_p = up - t
    ax.add_patch(Polygon([(0, up), (joint[0] - 0.2, up), (joint[0] - 0.2, up + 0.8), (0, up + 0.8)],
                         fc="#c9cdd2", ec=INK2, lw=0.5))
    ax.add_patch(Polygon([(0.3, face_p), (4.5, face_p), (4.5, up), (0.3, up)], fc="#5b8fe0", ec=PAD, lw=0.5))
    # distal phalanx and pad
    ax.add_patch(Polygon(dist([(0.4, -0.4), (3.6, -0.4), (3.6, 0.4), (0.4, 0.4)], joint), fc="#c9cdd2", ec=INK2,
                         lw=0.5))
    ax.add_patch(Polygon(dist([(0.7, -0.4 - t), (3.4, -0.4 - t), (3.4, -0.4), (0.7, -0.4)], joint), fc="#5b8fe0",
                         ec=PAD, lw=0.5))
    # this hand's skeleton (link lines through the joint)
    sk = np.array([(0.0, up + 0.4), tuple(joint), tuple(dist([(3.9, 0.0)], joint)[0])])
    ax.plot(sk[:, 0], sk[:, 1], color=colour, lw=1.0, zorder=6)
    ax.add_patch(Circle(joint, 0.26, fc="white", ec=colour, lw=0.9, zorder=7))
    # O's skeleton and O's contact surfaces, dashed
    osk = np.array([(0.0, 0.4), tuple(j0), tuple(dist([(3.9, 0.0)], j0)[0])])
    ax.plot(osk[:, 0], osk[:, 1], ls=(0, (2, 1.4)), color="#8a8a8a", lw=0.8, zorder=8)
    o_dist = dist([(0.4, -0.4 - t0), (3.7, -0.4 - t0)], j0)
    for seg in (np.array([(0.0, -t0), (4.7, -t0)]), o_dist):
        ax.plot(seg[:, 0], seg[:, 1], ls=(0, (2, 1.4)), color=INK, lw=0.7, zorder=8)
    # the object, resting on O's two contact surfaces
    r = 1.45
    cy = -t0 - r
    p0 = o_dist[0]
    cx = p0[0] + (r - (cy - p0[1]) * n[1]) / n[0]
    ax.add_patch(Circle((cx, cy), r, fc="#f1e3cf", ec="#9a7a52", lw=0.6, zorder=0))
    cpt = np.array([cx, cy]) - n * r
    side = np.array([-n[1], n[0]]) * 0.45
    extra = 2.4 * dt if keep == "skeleton" else 0.0
    for start, length, col, lw in ((cpt + side, 1.0, "#9a9a9a", 1.0), (cpt, 1.0 + extra, colour, 1.3)):
        ax.add_patch(FancyArrowPatch(start, start + n * length, arrowstyle="-|>", mutation_scale=7, color=col,
                                     lw=lw, zorder=9))


def main() -> int:
    """Fig. 1 (money figure, 2026-11-24): keeping the contact-surface motion lets an interface be evaluated without a
    bias from the reused controller. (a) the hand (Isaac Lab USD); (b) O and two 7 mm redesigns at the same joint
    command; (c) how far each redesign's distal contact surface strays from O's over flexion; (d) the effect of a
    softer 7 mm pad on pull-out load as each hand reports it under the same impedance setpoint."""
    import json
    from PIL import Image
    plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
                         "mathtext.fontset": "stix", "font.size": 7})
    width, height = 3.45, 2.50
    fig = plt.figure(figsize=(width, height))
    lab = dict(fontsize=7, fontweight="bold", va="top")
    # (a) the hand
    iso = usd_render(CAD / "hand_usd_palm_up.png")
    aw = 0.88
    ah = aw * iso.shape[0] / iso.shape[1]
    a = fig.add_axes((0.02 / width, (2.37 - ah) / height, aw / width, ah / height))
    a.imshow(iso); a.axis("off")
    fig.text(0.005, 0.995, "(a)", **lab)
    fig.text(0.46 / width, (2.37 - ah - 0.03) / height, "CoDe hand\n(pads: blue)", fontsize=6, color=INK2,
             ha="center", va="top", linespacing=1.0)
    # (b) O, U, R at the same joint command
    fig.text(0.98 / width, 0.995, "(b)", **lab)
    fig.text(2.2 / width, 2.47 / height, "same joint command, $q=120^\\circ$", fontsize=6.5, ha="center", va="top")
    names = (("O", "O: 3 mm pads"), ("U", "U: 7 mm,\ncoverage only"), ("R", "R: 7 mm, keeps\ncontact motion"))
    keyed = [usd_white(CAD / f"usd_finger_{h}_q120.png") for h, _ in names]
    ims = [k[0] for k in keyed]
    ink = np.any(np.stack([k[1] > 0.5 for k in keyed]), axis=0)
    ink &= ink.sum(axis=1, keepdims=True) > 20  # ignore stray thin rows
    r, c = np.where(ink)
    c0 = c.min() + int(0.12 * (c.max() - c.min()))  # drop the palm block at the left edge
    ims = [im[max(r.min() - 4, 0):r.max() + 4, c0:c.max() + 4] for im in ims]
    fw = 0.78
    fh = fw * ims[0].shape[0] / ims[0].shape[1]
    for i, ((h, name), im) in enumerate(zip(names, ims)):
        x0 = 1.0 + i * (fw + 0.03)
        ax = fig.add_axes((x0 / width, (2.31 - fh) / height, fw / width, fh / height))
        ax.imshow(im); ax.axis("off")
        fig.text((x0 + fw / 2) / width, (2.31 - fh - 0.02) / height, name, fontsize=6, color=HAND_C[h],
                 ha="center", va="top", linespacing=1.0, fontweight="bold" if h != "O" else "normal")
    # (c) qualitative: the same joint command and gain on a redesign that keeps the contact-surface motion (R) and on
    # one that does not (U); no data
    fig.text(0.005, 1.30 / height, "(c)", **lab)
    fig.text(1.80 / width, 1.30 / height, "thicker pad, same command: keep the skeleton or the contact surfaces, not both",
             fontsize=6.2, ha="center", va="top")
    for i, (keep, col, title, verdict) in enumerate((
            ("skeleton", HAND_C["U"], "keep the skeleton (toward U)", "contact surfaces moved:\ninterface effect biased"),
            ("contact", HAND_C["R"], "keep the contact surfaces (R)", "skeleton moved:\ninterface effect read correctly"))):
        x0 = 0.08 + i * 1.72
        ax = fig.add_axes((x0 / width, 0.02 / height, 1.62 / width, 1.02 / height))
        schematic(ax, col, keep)
        ax.set_title(title, fontsize=6.3, color=col, fontweight="bold", pad=1)
        ax.text(0.5, 0.0, verdict, transform=ax.transAxes, fontsize=6, ha="center", va="bottom", color=INK,
                linespacing=1.0)
    DEST.mkdir(parents=True, exist_ok=True)
    out = DEST / "fig_cad.png"
    fig.savefig(out, dpi=400)
    Image.open(out).convert("RGB").save(out)  # opaque RGB, no SMask (PaperPlaza)
    print(out, f"{width} x {height} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
