"""Fetch the CAD renders used in Fig. 1 from Onshape (PNG only).

The CoDe Hand document's branch "dobule supprot link" (the double-support linkage), assembly "hand-layout", rendered
by Onshape's shaded-view service. Writes ``paper/figures/cad/<name>.png`` (composited on white) and
``cad/renders.json`` with the document microversion each render was read from.

    python paper/figures/fetch_cad.py      (cmd conda env; ONSHAPE_ACCESS_KEY / ONSHAPE_SECRET_KEY set)
"""
import base64
import io
import json
import sys
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "design"))
from core.cad.onshape import OnshapeClient  # noqa: E402

DOCUMENT = "70331145392063eb01686da5"  # "CoDe Hand"
WORKSPACE = "5e0189f99c1cf0c993f8846a"  # branch "dobule supprot link"
ASSEMBLY = "67c456d3940fbb4fa89384f6"  # "hand-layout"
FINGER = "0076b737ce3429d392cd82e5"  # "finger"
RENDERS = {"code_hand_iso": (ASSEMBLY, "isometric"), "finger_side": (FINGER, "front")}
DEST = Path(__file__).resolve().parent / "cad"


def main() -> int:
    client = OnshapeClient()
    DEST.mkdir(exist_ok=True)
    micro = client.get(f"documents/d/{DOCUMENT}/w/{WORKSPACE}/currentmicroversion")["microversion"]
    record = {"document": DOCUMENT, "workspace": WORKSPACE, "microversion": micro, "renders": {}}
    for name, (element, view) in RENDERS.items():
        reply = client.get(f"assemblies/d/{DOCUMENT}/w/{WORKSPACE}/e/{element}/shadedviews", viewMatrix=view,
                           outputHeight=1800, outputWidth=2400, pixelSize=0, edges="show")
        image = Image.open(io.BytesIO(base64.b64decode(reply["images"][0]))).convert("RGBA")
        white = Image.new("RGBA", image.size, "white")
        flat = Image.alpha_composite(white, image).convert("RGB")
        box = Image.eval(flat.convert("L"), lambda v: 255 if v < 250 else 0).getbbox()
        flat.crop(box).save(DEST / f"{name}.png")
        record["renders"][name] = {"element": element, "view": view, "crop": box}
    (DEST / "renders.json").write_text(json.dumps(record, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
