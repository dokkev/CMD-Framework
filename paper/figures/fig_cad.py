"""Fig. 1 of the RSS 2027 extended abstract: the CoDe hand from its CAD (PNG only, single column, print size).

    conda run -n cmd python paper/figures/fig_cad.py [dest_dir]

(a) The hand as simulated: the visual USD used in Isaac Lab, rendered palm up by Newton's GL viewer
(sim/isaaclab/scripts/code_hand/render_visuals.py --view palm_up, copied to paper/figures/cad/hand_usd_palm_up.png;
its dark background replaced by white here); (b) one finger from the side, from the CAD (fetch_cad.py; document,
workspace and microversion in renders.json). Since 2026-11-24 (a) shows the finger in motion instead: the
simulated finger O rendered by Drake at 0, 60 and 120 deg (sim/drake/render_hands.py -> logs/hand_O_<q>.png, copied to
cad/finger_O_q<q>.png) and composited with the earlier poses faded. Callouts in (b): the MCP servo, the PIP servo on
the proximal phalanx, the crossed four-bar PIP joint and the proximal and distal pads (blue, translucent).
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


def motion_composite(qs=(0, 60, 120), alphas=(0.30, 0.55, 1.0)):
    """The finger at several flexions in one image, earlier poses faded (same camera for every pose)."""
    from PIL import Image
    ims = [np.asarray(Image.open(CAD / f"finger_O_q{q}.png").convert("RGB")).astype(float) for q in qs]
    out = np.full_like(ims[0], 255.0)
    for im, a in zip(ims, alphas):
        mask = np.clip((255 * 3 - im.sum(axis=2)) / 60.0, 0, 1)[..., None] * a
        out = out * (1 - mask) + im * mask
    rows, cols = np.where(out.min(axis=2) < 245)
    return out[rows.min() - 6:rows.max() + 6, cols.min() - 6:cols.max() + 6].astype(np.uint8)


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


def main() -> int:
    """Fig. 1 (2026-11-24 figure plan): single column, about 55 mm. (a) the hand and (b) one finger from the CAD,
    with the proximal and distal modules marked as exchangeable; (c) the kinds of contact interface the modules are
    meant to carry, drawn as illustrative cross-sections (not yet integrated)."""
    plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
                         "mathtext.fontset": "stix", "font.size": 7})
    width, height = 3.45, 2.2
    fig = plt.figure(figsize=(width, height))
    iso = usd_render(CAD / "hand_usd_palm_up.png")
    fin = motion_composite()
    lab = dict(fontsize=7, fontweight="bold", va="top")
    # (a) one finger flexing 0 -> 120 deg, earlier poses faded
    fw = 1.95
    fh = fw * fin.shape[0] / fin.shape[1]
    fy = 0.86
    f = fig.add_axes((0.02 / width, fy / height, fw / width, fh / height))
    f.imshow(fin); f.axis("off")
    fig.text(0.005, 0.995, "(a)", **lab)
    callouts = [  # (label, point in the composite [px], text position in inches, colour)
        ("PIP servo on the proximal phalanx", (450, 70), (1.05, 2.12), INK),
        ("crossed four-bar", (640, 300), (1.80, 1.93), INK),
        ("proximal pad", (330, 300), (0.42, 0.80), PAD),
        ("distal pad", (480, 470), (1.30, 0.80), PAD),
    ]
    for label, xy, txt, colour in callouts:
        f.annotate(label, xy=xy, xycoords="data", xytext=(txt[0] / width, txt[1] / height),
                   textcoords="figure fraction", fontsize=6.2, color=colour, ha="center", va="center",
                   linespacing=1.0, arrowprops=dict(arrowstyle="-", color=INK2, lw=0.5, shrinkA=1, shrinkB=0))
    f.annotate("", xy=(800, 470), xytext=(935, 280), xycoords="data",
               arrowprops=dict(arrowstyle="-|>", color=INK2, lw=0.8, mutation_scale=7,
                               connectionstyle="arc3,rad=-0.35"))
    f.text(870, 540, "flexion $0\\to120^\\circ$", fontsize=6.0, color=INK2, ha="center", va="top", clip_on=False)
    # (b) the hand as simulated (Isaac Lab USD), top right
    aw = 1.40
    ah = aw * iso.shape[0] / iso.shape[1]
    a = fig.add_axes((2.03 / width, (fy + 0.12) / height, aw / width, ah / height))
    a.imshow(iso); a.axis("off")
    fig.text(2.03 / width, 0.995, "(b)", **lab)
    fig.text(2.73 / width, (fy + 0.06) / height, "pads exchanged without\ndisassembling the joint", fontsize=6.0,
             color=INK2, ha="center", va="center", style="italic", linespacing=1.0)
    # (c) illustrative interfaces, bottom row
    fig.text(0.005, 0.64 / height, "(c)", **lab)
    fig.text(0.2 / width, 0.64 / height, "interfaces the pad modules are meant to carry (illustrative)", fontsize=6.3,
             color=INK2, va="top", style="italic")
    kinds = (("thin", "thin rigid pad"), ("thick", "thick compliant pad"), ("textured", "textured surface"),
             ("tactile", "vision-based tactile"))
    iw, ih = 0.72, 0.28
    for i, (k, name) in enumerate(kinds):
        x0 = 0.12 + i * (iw + 0.12)
        y0 = 0.14
        ax = fig.add_axes((x0 / width, y0 / height, iw / width, ih / height))
        icon(ax, k)
        fig.text((x0 + iw / 2) / width, (y0 - 0.005) / height, name, fontsize=5.8, ha="center", va="top", color=INK)
    DEST.mkdir(parents=True, exist_ok=True)
    out = DEST / "fig_cad.png"
    fig.savefig(out, dpi=400)
    from PIL import Image
    Image.open(out).convert("RGB").save(out)  # opaque RGB, no SMask (PaperPlaza)
    print(out, f"{width} x {height} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
