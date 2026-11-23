"""Fig. 1 of the RSS 2027 extended abstract: the CoDe hand from its CAD (PNG only, single column, print size).

    conda run -n cmd python paper/figures/fig_cad.py [dest_dir]

(a) The hand (Onshape shaded view, isometric) and (b) one finger from the side, both from paper/figures/cad/
(fetch_cad.py; document, workspace and microversion in renders.json). Callouts in (b): the MCP servo, the PIP servo on
the proximal phalanx, the crossed four-bar PIP joint and the proximal and distal pads (blue, translucent).
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
CAD = ROOT / "paper" / "figures" / "cad"
DEST = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / "ContactCentricDesign-RSS2027" / "figures"
INK, INK2, PAD = "#0b0b0b", "#52514e", "#2a5db0"


def main() -> int:
    plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
                         "mathtext.fontset": "stix", "font.size": 7})
    width, height = 3.45, 1.32
    fig = plt.figure(figsize=(width, height))
    iso = plt.imread(CAD / "code_hand_iso.png")
    side = plt.imread(CAD / "finger_side.png")
    # (a) the hand, left; (b) the finger, right, with room for callouts above and below
    a = fig.add_axes((0.0, 0.02, 1.2 / width, 1.2 * iso.shape[0] / iso.shape[1] / height))
    a.imshow(iso)
    a.axis("off")
    bw = 2.08
    bh = bw * side.shape[0] / side.shape[1]
    b = fig.add_axes((1.35 / width, 0.48 / height, bw / width, bh / height))
    b.imshow(side)
    b.axis("off")
    fig.text(0.005, 0.985, "(a)", fontsize=7, fontweight="bold", va="top")
    fig.text(1.35 / width, 0.985, "(b)", fontsize=7, fontweight="bold", va="top")
    callouts = [  # (label, point in the side view [px], text position in figure fraction, colour)
        ("MCP servos", (560, 160), (1.55 / width, 1.1 / height), INK),
        ("PIP servo on the\nproximal phalanx", (1200, 150), (2.35 / width, 1.1 / height), INK),
        ("crossed four-bar", (1600, 360), (3.40 / width, 1.1 / height), INK),
        ("proximal pad", (1100, 440), (2.05 / width, 0.26 / height), PAD),
        ("distal pad", (2050, 480), (3.40 / width, 0.26 / height), PAD),
    ]
    for label, xy, txt, colour in callouts:
        b.annotate(label, xy=xy, xycoords="data", xytext=txt, textcoords="figure fraction", fontsize=6.3,
                   color=colour, ha="right" if txt[0] > 0.8 else "center", va="center", linespacing=1.0,
                   arrowprops=dict(arrowstyle="-", color=INK2, lw=0.5, shrinkA=1, shrinkB=0))
    fig.text(2.39 / width, 0.11 / height, "room for thick pads under the joint", fontsize=6.3, color=INK2,
             ha="center", va="center", style="italic")
    DEST.mkdir(parents=True, exist_ok=True)
    out = DEST / "fig_cad.png"
    fig.savefig(out, dpi=400)
    print(out, f"{width} x {height} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
