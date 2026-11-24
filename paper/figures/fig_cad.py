"""Fig. 1 of the RSS 2027 extended abstract: the CoDe hand from its CAD (PNG only, single column, print size).

    conda run -n cmd python paper/figures/fig_cad.py [dest_dir]

(a) The hand as simulated: the visual USD used in Isaac Lab, rendered palm up by Newton's GL viewer
(sim/isaaclab/scripts/code_hand/render_visuals.py --view palm_up, copied to paper/figures/cad/hand_usd_palm_up.png;
its dark background replaced by white here); (b) one finger from the side, from the CAD (fetch_cad.py; document,
workspace and microversion in renders.json). Callouts in (b): the MCP servo, the PIP servo on
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
    width, height = 3.45, 2.42
    fig = plt.figure(figsize=(width, height))
    iso = usd_render(CAD / "hand_usd_palm_up.png")
    side = plt.imread(CAD / "finger_side.png")
    lab = dict(fontsize=7, fontweight="bold", va="top")
    # (a) one finger from the CAD, full width, the modules marked
    bw = 3.25
    bh = bw * side.shape[0] / side.shape[1]
    by = 1.30
    b = fig.add_axes((0.12 / width, by / height, bw / width, bh / height))
    b.imshow(side); b.axis("off")
    fig.text(0.005, 0.995, "(a)", **lab)
    from matplotlib.patches import Rectangle
    for (x0, y0, x1, y1) in ((1050, 420, 1580, 640), (1780, 380, 2360, 620)):
        b.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec=PAD, lw=0.8, ls=(0, (2.5, 1.5)),
                              clip_on=False))
    callouts = [  # (label, point in the side view [px], text position in inches, colour)
        ("PIP servo on the proximal phalanx", (1200, 150), (1.45, 2.33), INK),
        ("crossed four-bar", (1600, 360), (2.95, 2.33), INK),
        ("proximal module", (1300, 640), (1.55, 1.13), PAD),
        ("distal module", (2100, 620), (2.85, 1.13), PAD),
    ]
    for label, xy, txt, colour in callouts:
        b.annotate(label, xy=xy, xycoords="data", xytext=(txt[0] / width, txt[1] / height),
                   textcoords="figure fraction", fontsize=6.3, color=colour, ha="center", va="center",
                   linespacing=1.0, arrowprops=dict(arrowstyle="-", color=INK2, lw=0.5, shrinkA=1, shrinkB=0))
    fig.text(2.2 / width, 0.99 / height, "exchanged without disassembling the joint", fontsize=6.1, color=INK2,
             ha="center", va="center", style="italic")
    # (b) the hand as simulated (Isaac Lab USD), bottom left
    aw = 1.42
    ah = aw * iso.shape[0] / iso.shape[1]
    a = fig.add_axes((0.02 / width, 0.08 / height, aw / width, ah / height))
    a.imshow(iso); a.axis("off")
    fig.text(0.005, 0.90 / height, "(b)", **lab)
    # (c) illustrative interfaces, bottom right, 2 x 2
    cx = 1.62
    fig.text(cx / width, 0.90 / height, "(c)", **lab)
    fig.text((cx + 0.17) / width, 0.90 / height, "interfaces to carry (illustrative)", fontsize=6.3, color=INK2,
             va="top", style="italic")
    kinds = (("thin", "thin rigid pad"), ("thick", "thick compliant pad"), ("textured", "textured surface"),
             ("tactile", "vision-based tactile"))
    iw, ih = 0.80, 0.22
    for i, (k, name) in enumerate(kinds):
        col, row = i % 2, i // 2
        x0 = cx + 0.08 + col * (iw + 0.12)
        y0 = 0.53 - row * 0.40
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
