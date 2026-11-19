#!/usr/bin/env python3
"""Study 2 for CoDe (15 degrees) at each pivot housing diameter: how far the linkage must move to take a bigger pin.

    python scripts/study/study2_morphology_adaptation/pivot_sweep.py

Study 1's pivot sweep (``../study1_fixed_morphology/pivot_sweep.py``) holds the reference fixed: past 6 mm CoDe's
A pin overlaps the PIP servo and nothing mounts. Here the search may move the linkage (Q: A, B, B'), and then the
finger (E), with the servo at A' held fixed as always in Study 2. The 2Rs are not run: their pins never meet a
servo body, so their result does not depend on the diameter (Study 1 sweep).

Each run sets every pivot (A, A', B, B') to the diameter (``goal_overrides``), on ``config.yaml``'s targets and
optimizer. Writes ``output/study/study2_pivot_sweep/d<diameter>/`` (a Study 2 run each, its config included) and
``pivot_summary.csv`` beside them (each run's ``summary_best.csv`` rows, with the diameter and A's move).
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run as study2  # noqa: E402

ROOT = study2.REPOSITORY_ROOT
OUT = ROOT / "output/study/study2_pivot_sweep"
DIAMETERS = (3, 4, 5, 6, 7, 8, 9)
REFERENCE = "code_matched__tilt15"


def _a_ap(x: list[float]) -> tuple[float, float]:
    """``a_x`` and ``|AA'|`` of a CoDe design vector (``core.morphology.mechanism.linkage.design``'s order)."""

    from core.morphology.mechanism.linkage.design import VARIABLE_NAMES

    v = dict(zip(VARIABLE_NAMES, x))
    return v["a_x_mm"], math.hypot(v["ap_x_mm"] - v["a_x_mm"], v["ap_z_mm"] - v["a_z_mm"])


def main() -> int:
    base = study2.read_config_file(HERE / "config.yaml")
    rows = []
    for diameter in DIAMETERS:
        folder = OUT / f"d{diameter}"
        folder.mkdir(parents=True, exist_ok=True)
        size = float(diameter)
        config = {**base, "output": str(folder.relative_to(ROOT)), "only_references": [REFERENCE],
                  "servo_tilt_sweep": {"angles_deg": [15], "references": ["code_matched"]},
                  "goal_overrides": {"packaging": {
                      "pivot_clearance_diameter_mm": size, "a_pivot_diameter_mm": size, "ap_pivot_diameter_mm": size,
                      "b_pivot_diameter_mm": size, "bp_pivot_diameter_mm": size, "ap_clearance_diameter_mm": size}}}
        (folder / "config.yaml").write_text(yaml.safe_dump(config, sort_keys=False))
        study2.run(folder / "config.yaml", ROOT)
        with (folder / "references.csv").open(encoding="utf-8") as stream:
            reference_x = [float(v) for v in next(csv.DictReader(stream))["x"].split(";")]
        a_ref, aap_ref = _a_ap(reference_x)
        with (folder / "summary_best.csv").open(encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                moved = {}
                if row["final_x"]:
                    a, aap = _a_ap([float(v) for v in row["final_x"].split(";")])
                    moved = {"a_x_change_mm": round(a - a_ref, 6), "aap_mm": round(aap, 6),
                             "aap_change_mm": round(aap - aap_ref, 6)}
                rows.append({"pivot_diameter_mm": diameter, **row, **moved})
        print(f"pivot {diameter} mm: done", flush=True)
    fields = list(dict.fromkeys(name for row in rows for name in row))
    with (OUT / "pivot_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {OUT / 'pivot_summary.csv'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
