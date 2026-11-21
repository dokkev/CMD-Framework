"""Crop report figures into compact strips for the 6-page extended abstract (PNG only).

Interim until the figures are redrawn for the paper. Usage: python crop_figures.py <dest_figures_dir>
"""
import sys
from pathlib import Path
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
dest = Path(sys.argv[1])
dest.mkdir(parents=True, exist_ok=True)


def stack(parts):
    w = max(p.width for p in parts)
    out = Image.new("RGB", (w, sum(p.height for p in parts)), "white")
    y = 0
    for p in parts:
        out.paste(p, (0, y))
        y += p.height
    return out


def crop(path, boxes):
    im = Image.open(ROOT / path).convert("RGB")
    return stack([im.crop(b) for b in boxes])


# Native designs at q_req = 150 deg (pad coverage below each), relabelled in the paper's palette.
from PIL import ImageDraw, ImageFont
row = crop("report/study1/figures/native_designs_paper_2026-11-20.png", [(90, 800, 2148, 1125)])
head = Image.new("RGB", (row.width, 70), "white")
draw = ImageDraw.Draw(head)
font = None
for f in ("/usr/share/fonts/truetype/dejavu/DejaVuSerif-Bold.ttf", "/usr/share/fonts/truetype/liberation/LiberationSerif-Bold.ttf"):
    try:
        font = ImageFont.truetype(f, 40)
        break
    except OSError:
        pass
labels = [("CFL (CoDe)", "#2a78d6"), ("2R-remote", "#eb6834"), ("2R-DD", "#1baf7a"), ("RCJ", "#eda100"),
          ("CFL ideal*", "#2a78d6"), ("RCJ ideal*", "#eda100")]
centres = [248, 565, 880, 1197, 1513, 1830]  # column centres of the source figure, x minus the 90 px crop
for (text, colour), cx in zip(labels, centres):
    w = draw.textlength(text, font=font)
    draw.text((cx - 90 - w / 2, 12), text, fill=colour, font=font)
stack([head, row]).save(dest / "fig_designs_q150.png")
print("written to", dest)
