#!/usr/bin/env python3
"""Study 1 at each pivot housing diameter: a sensitivity run of the study's packaging.

    python scripts/study/study1_fixed_morphology/pivot_sweep.py

One full Study 1 (``config.yaml``: the active 15-degree references and tilt sweep, 71 module pairs, 90/120/150
degrees) per diameter in ``DIAMETERS``. Each run sets every pivot (A, A', B, B') to that diameter on every
reference alike (``goal_overrides``), and nothing else. The study's rules tie two more things to it:
- A's and B's radii are where each pad starts below its link line, so the pads move with the pivots;
- the RCJ's smallest roller is bounded by B's radius.

The study's own value is A 6, A' 10, B 6, B' 6 mm (``baseline.yaml``). A' changes no result
(``baseline.yaml``). A study sensitivity, not the prototype's packaging (README.md, "Two optimizations").

Writes ``output/study/study1_pivot_sweep/d<diameter>/`` (a Study 1 run each, its config included) and
``pivot_counts.csv`` beside them.
"""

from __future__ import annotations

import csv
import sys
from collections import defaultdict
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run as study1  # noqa: E402

ROOT = study1.REPOSITORY_ROOT
OUT = ROOT / "output/study/study1_pivot_sweep"
DIAMETERS = (3, 4, 5, 6, 7, 8, 9)


def main() -> int:
    base = study1.read_config_file(HERE / "config.yaml")
    counts = defaultdict(dict)
    for diameter in DIAMETERS:
        folder = OUT / f"d{diameter}"
        folder.mkdir(parents=True, exist_ok=True)
        size = float(diameter)
        config = {**base, "output": str((folder).relative_to(ROOT)),
                  "goal_overrides": {"packaging": {
                      "pivot_clearance_diameter_mm": size, "a_pivot_diameter_mm": size, "ap_pivot_diameter_mm": size,
                      "b_pivot_diameter_mm": size, "bp_pivot_diameter_mm": size, "ap_clearance_diameter_mm": size}}}
        (folder / "config.yaml").write_text(yaml.safe_dump(config, sort_keys=False))
        study1.run(folder / "config.yaml", ROOT)
        with (folder / "assembly_results.csv").open(encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                key = (row["reference_id"], float(row["required_flexion_deg"]))
                cell = counts[key].setdefault(diameter, [0, 0])
                cell[0] += row["feasible"] == "True"
                cell[1] += row["mounted"] == "True"
        print(f"pivot {diameter} mm: done", flush=True)
    rows = [{"reference_id": ref, "required_flexion_deg": flexion,
             **{f"feasible_d{d}": counts[(ref, flexion)][d][0] for d in DIAMETERS},
             **{f"mounted_d{d}": counts[(ref, flexion)][d][1] for d in DIAMETERS}}
            for ref, flexion in sorted(counts)]
    with (OUT / "pivot_counts.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {OUT / 'pivot_counts.csv'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
