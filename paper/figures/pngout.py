"""Save a matplotlib figure as an opaque RGB PNG.

Matplotlib writes RGBA; LaTeX then embeds the alpha channel as a soft mask (SMask), i.e. transparency, which PDF
compliance checkers for conference submissions (PaperPlaza / PDF eXpress) flag. Flattening onto white removes it.
"""
from pathlib import Path

from PIL import Image


def save_rgb(fig, path, **kwargs) -> Path:
    path = Path(path)
    fig.savefig(path, facecolor="white", transparent=False, **kwargs)
    with Image.open(path) as im:
        dpi = im.info.get("dpi")
        flat = Image.new("RGB", im.size, "white")
        flat.paste(im, mask=im.getchannel("A") if im.mode == "RGBA" else None)
    flat.save(path, dpi=dpi) if dpi else flat.save(path)
    return path
