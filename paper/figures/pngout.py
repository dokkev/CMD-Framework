"""Save a paper figure: a PDF for Overleaf and an opaque RGB PNG preview of the same figure.

Matplotlib writes RGBA; LaTeX then embeds the alpha channel as a soft mask (SMask), i.e. transparency, which PDF
compliance checkers for conference submissions (PaperPlaza / PDF eXpress) flag. The PNG is flattened onto white;
the PDF must not hold RGBA images (flatten them onto their background before drawing) and embeds its fonts as
TrueType (the CFF fonts are converted once into ~/.cache/paper_fonts, never committed).
"""
from pathlib import Path

import matplotlib

FONTS = {"Helvetica Light": "/usr/local/share/fonts/Helvetica Light Regular.otf",
         "NimbusSans-Bold": "/usr/share/fonts/opentype/urw-base35/NimbusSans-Bold.otf",
         "NimbusSans-Italic": "/usr/share/fonts/opentype/urw-base35/NimbusSans-Italic.otf"}
CACHE = Path.home() / ".cache" / "paper_fonts"


def _otf_to_ttf(src, dst) -> None:
    """CFF outlines to TrueType quadratics, so the PDF embeds plain TrueType (no Type 3, no CFF-in-TrueType)."""
    from fontTools.pens.cu2quPen import Cu2QuPen
    from fontTools.pens.ttGlyphPen import TTGlyphPen
    from fontTools.ttLib import TTFont, newTable
    f = TTFont(src)
    gs, order = f.getGlyphSet(), f.getGlyphOrder()
    glyphs = {}
    for n in order:
        pen = TTGlyphPen(gs)
        gs[n].draw(Cu2QuPen(pen, 1.0, reverse_direction=True))
        glyphs[n] = pen.glyph()
    f["loca"] = newTable("loca")
    f["glyf"] = g = newTable("glyf")
    g.glyphOrder, g.glyphs = order, glyphs
    del f["CFF "]
    f["maxp"] = m = newTable("maxp")
    m.tableVersion = 0x00010000
    for k in ("maxTwilightPoints", "maxStorage", "maxFunctionDefs", "maxInstructionDefs", "maxStackElements",
              "maxSizeOfInstructions", "maxComponentElements"):
        setattr(m, k, 0)
    m.maxZones = 1
    f["head"].indexToLocFormat = f["head"].glyphDataFormat = 0
    post = f["post"]
    post.formatType, post.extraNames, post.mapping, post.glyphOrder = 2.0, [], {}, order
    f.sfntVersion = "\x00\x01\x00\x00"
    f.save(dst)


def register_fonts() -> None:
    """Helvetica Light (text) and Nimbus Sans bold / italic (bold, math italic) as TrueType, replacing the CFF
    files of the same families in matplotlib's font list."""
    from matplotlib import font_manager as fm
    CACHE.mkdir(parents=True, exist_ok=True)
    fm.fontManager.ttflist = [e for e in fm.fontManager.ttflist
                              if not (e.fname.endswith(".otf") and e.name in ("Helvetica Light", "Nimbus Sans"))]
    for name, src in FONTS.items():
        dst = CACHE / (name.replace(" ", "") + ".ttf")
        if Path(src).exists():
            if not dst.exists():
                _otf_to_ttf(src, dst)
            fm.fontManager.addfont(str(dst))
    matplotlib.rcParams.update({"pdf.fonttype": 42, "ps.fonttype": 42})

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


def save_figure(fig, path, dpi: int = 400) -> Path:
    """The PDF Overleaf includes (path with .pdf) and its PNG preview (path with .png)."""
    path = Path(path)
    for ax in fig.axes:
        for im in ax.images:
            a = im.get_array()
            if a is not None and a.ndim == 3 and a.shape[2] == 4:
                raise ValueError(f"RGBA image in {path.stem}: flatten it onto its background (no SMask in the PDF)")
    fig.savefig(path.with_suffix(".pdf"), facecolor="white", transparent=False, bbox_inches="tight", pad_inches=0.01,
                dpi=dpi)
    return save_rgb(fig, path.with_suffix(".png"), dpi=dpi)


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
