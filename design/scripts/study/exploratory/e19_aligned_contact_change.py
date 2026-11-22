"""Exploratory e19: how much of Study 2's contact-surface change is a start offset (v10 review).

    conda run --no-capture-output -n cmd python scripts/study/exploratory/e19_aligned_contact_change.py

For the coverage end of each Study 2 front (the redesign the paper reports), split Delta_contact into the offset at
q = 0, |Delta c(0)|, and the change of the motion itself, max_q |Delta c(q) - R_q Delta c(0)| (the start point moved
along with the distal face, so a pure start shift gives zero). Prints both per architecture, flexion and thickness,
and writes ``output/study/exploratory/e19_aligned_contact_change/summary.csv``.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
STUDY2 = HERE.parents[1] / "study2_morphology_adaptation"
for p in (STUDY2, HERE.parents[1], HERE.parents[3]):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import numpy as np  # noqa: E402
import one_finger as study1  # noqa: E402
from core.optimization.analysis.contact_kinematics import paths  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

DESIGN = HERE.parents[3]
RUN = DESIGN / "output" / "study" / "study2_redesign_2026-11-21"
OUT = DESIGN / "output" / "study" / "exploratory" / "e19_aligned_contact_change"


def main() -> int:
    config = read_config_file(DESIGN / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    front = list(csv.DictReader((RUN / "front.csv").open()))
    rows = []
    for name, spec in config["architectures"].items():
        for flexion in (float(q) for q in config["required_flexion_deg"]):
            designs = study1.study1_designs(name, spec, flexion, config)
            if 3.0 not in designs:
                continue
            first = designs[3.0]
            base = mechanism(first.mechanism)
            ref = paths(base, first.x, first.config)
            for target in sorted(h for h in designs if h > 3.0):
                own = [r for r in front if r["architecture"] == name and float(r["required_flexion_deg"]) == flexion
                       and float(r["to_mm"]) == target]
                if not own:
                    continue
                most = max(own, key=lambda r: float(r["c_pad_mean"]))
                x = tuple(float(v) for v in most["x"].split(";"))
                p = paths(base, x, designs[target].config)
                if p is None:
                    continue
                d = p["contact"] - ref["contact"]  # (Q, 2), mm, proximal-phalanx frame
                q = np.asarray(ref["q"])
                c, s = np.cos(q), np.sin(q)
                d0 = d[0]
                rot = np.stack((c * d0[0] - s * d0[1], s * d0[0] + c * d0[1]), axis=1)
                rot_neg = np.stack((c * d0[0] + s * d0[1], -s * d0[0] + c * d0[1]), axis=1)
                motion = min(np.linalg.norm(d - rot, axis=1).max(), np.linalg.norm(d - rot_neg, axis=1).max())
                rows.append({"architecture": name, "required_flexion_deg": flexion, "to_mm": target,
                             "c_pad": float(most["c_pad_mean"]), "change_mm": float(np.linalg.norm(d, axis=1).max()),
                             "offset_mm": float(np.linalg.norm(d0)), "motion_change_mm": float(motion),
                             "bound_mm": 2 * math.sin(math.radians(flexion) / 2) * (target - 3.0)})
                r = rows[-1]
                print(f"{name:10s} q{flexion:3.0f} {target:g} mm: change {r['change_mm']:5.2f}  offset {r['offset_mm']:5.2f}"
                      f"  motion {r['motion_change_mm']:5.2f}  bound {r['bound_mm']:5.2f}  C_pad {r['c_pad']:.3f}", flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    with (OUT / "summary.csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(OUT / "summary.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
