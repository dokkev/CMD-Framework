"""Exploratory e20: split Study 3's contact-surface change into the offset normal to the distal face and the slide
along it (v16 review, r3).

    conda run --no-capture-output -n cmd python scripts/study/exploratory/e20_face_offset.py

For every Study 3 hand (``output/sim/study3/hands/cfl_q120_h3to*_*.yaml``) the distal contact-face point c(q) is
rebuilt from the design in the hand file (the study's own kinematics, ``contact_kinematics.paths``) and compared with
O's over q in [0, 120] deg. The difference Delta c(q) splits into its component along the distal face's normal
n(q) = (sin q, cos q) (how far the face sits from O's face: what over-closes a reused command) and along the face
(how far the same face point slides along it). Writes ``output/study/exploratory/e20_face_offset/summary.csv`` with
Delta_contact, its normal and tangential maxima [mm], and the signed normal offset (positive: the face moved away
from the object) averaged over the flexion range each placement set grasps at (v18 review, r2/r3).
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import e15_transmission_change as e15  # noqa: E402
from core.optimization.analysis.contact_kinematics import paths  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

HANDS = ROOT / "output" / "sim" / "study3" / "hands"
OUT = ROOT / "output" / "study" / "exploratory" / "e20_face_offset"


def band(q: np.ndarray, lo_deg: float, hi_deg: float) -> np.ndarray:
    """Poses in the flexion range a placement set grasps at (O's contacts: 35-115 deg; the grid: 85-115 deg)."""
    return (q >= math.radians(lo_deg)) & (q <= math.radians(hi_deg))


def main() -> int:
    config = read_config_file(ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    spec = config["architectures"]["cfl"]
    designs = e15.study1.study1_designs("cfl", spec, 120.0, config)
    base = mechanism(designs[3.0].mechanism)
    ref = paths(base, designs[3.0].x, designs[3.0].config)
    rows = []
    for path in sorted(HANDS.glob("cfl_q120_h3to*_*.yaml")):
        doc = yaml.safe_load(path.read_text())
        name = path.stem.removeprefix("cfl_q120_")
        h = float(doc["source"]["pad_thickness_mm"])
        x = tuple(doc["source"]["design"])
        got = paths(base, x, designs[h].config)
        if got is None or ref is None:
            print(f"{name}: no path")
            continue
        q = got["q"]
        d = got["contact"] - ref["contact"]
        normal = np.stack((np.sin(q), np.cos(q)), axis=1)
        sn = np.sum(d * normal, axis=1)  # signed: positive moves the face away from the object
        dn = np.abs(sn)
        dt = np.abs(np.sum(d * np.stack((np.cos(q), -np.sin(q)), axis=1), axis=1))
        rows.append({"hand": name, "thickness_mm": h, "contact_mm": float(np.max(np.linalg.norm(d, axis=1))),
                     "normal_mm": float(np.max(dn)), "tangential_mm": float(np.max(dt)),
                     "normal_at_105_mm": float(np.interp(math.radians(105.0), q, dn)),
                     "signed_normal_contacts_mm": float(np.mean(sn[band(q, 35.0, 115.0)])),
                     "signed_normal_grid_mm": float(np.mean(sn[band(q, 85.0, 115.0)]))})
    OUT.mkdir(parents=True, exist_ok=True)
    with (OUT / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    for r in rows:
        print(f"{r['hand']:26s} h {r['thickness_mm']:g} contact {r['contact_mm']:.2f} normal {r['normal_mm']:.2f} "
              f"tangential {r['tangential_mm']:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
