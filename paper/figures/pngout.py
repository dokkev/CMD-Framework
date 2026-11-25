"""Save a matplotlib figure as an opaque RGB PNG.

Matplotlib writes RGBA; LaTeX then embeds the alpha channel as a soft mask (SMask), i.e. transparency, which PDF
compliance checkers for conference submissions (PaperPlaza / PDF eXpress) flag. Flattening onto white removes it.
"""
from pathlib import Path

from PIL import Image


def trim_white(path, margin_px: int = 12, gap_px: int = 40, threshold: int = 250) -> None:
    """Remove empty rows: the white margins above and below, and any full-width white band taller than ``gap_px``
    (collapsed to ``gap_px``). The width is kept, so the figure still prints at its drawn width."""
    import numpy as np
    with Image.open(path) as im:
        dpi = im.info.get("dpi")
        a = np.asarray(im.convert("RGB"))
    ink = (a.min(axis=2) < threshold).any(axis=1)
    rows = np.nonzero(ink)[0]
    if not len(rows):
        return
    keep = np.zeros(len(ink), bool)
    keep[max(rows[0] - margin_px, 0):rows[-1] + margin_px + 1] = True
    run = 0
    for r in range(rows[0], rows[-1] + 1):  # collapse long white runs inside
        run = 0 if ink[r] else run + 1
        if run > gap_px:
            keep[r] = False
    out = Image.fromarray(a[keep])
    out.save(path, dpi=dpi) if dpi else out.save(path)


def save_rgb(fig, path, trim: bool = True, **kwargs) -> Path:
    path = Path(path)
    fig.savefig(path, facecolor="white", transparent=False, **kwargs)
    with Image.open(path) as im:
        dpi = im.info.get("dpi")
        flat = Image.new("RGB", im.size, "white")
        flat.paste(im, mask=im.getchannel("A") if im.mode == "RGBA" else None)
    flat.save(path, dpi=dpi) if dpi else flat.save(path)
    if trim:
        trim_white(path)
    return path
