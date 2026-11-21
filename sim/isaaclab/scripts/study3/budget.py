# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3A main: the training budget from the budget pilot, by the rule fixed before it ran (docs/decisions.md).

    uv run python scripts/study3/budget.py

For each ``*_s100_budget`` run: the 100-iteration moving mean of ``consecutive_success``; the plateau, the mean
of its last 300 iterations; the first iteration where the moving mean reaches 95% of the plateau. The budget is
the larger of the two surfaces' iterations rounded up to a multiple of 500. A run whose last 500 iterations
still rise by more than 5% of their mean per 100 iterations is reported as not converged, and the budget is
then the pilot's 3000 iterations. Writes report/study3/tables/budget.json.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

TAG = "Metrics/object_pose/consecutive_success"


def curve(run_dir: Path) -> np.ndarray:
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

    acc = EventAccumulator(str(next(run_dir.glob("events.out.tfevents.*"))), size_guidance={"scalars": 0})
    acc.Reload()
    return np.array([e.value for e in acc.Scalars(TAG)])


def main() -> int:
    runs = sorted((common.SIM_ROOT / "logs" / "rsl_rl" / "study3_code_reorient").glob("*_s100_budget"))
    out = {"rule": __doc__.split("\n\n")[1].replace("\n", " "), "runs": {}}
    reached, converged, total = [], [], None
    for run in runs:
        v = curve(run)
        moving = np.convolve(v, np.ones(100) / 100, mode="valid")  # moving[i] = mean of v[i : i + 100]
        plateau = float(v[-300:].mean())
        first = int(np.argmax(moving >= 0.95 * plateau)) + 99  # iteration at the window's end
        tail = v[-500:]
        slope = float(np.polyfit(np.arange(len(tail)), tail, 1)[0] * 100)
        ok = slope <= 0.05 * float(tail.mean())
        variant = run.name.split("_")[2]
        out["runs"][variant] = {"run": run.name, "iterations": len(v), "plateau": plateau, "reach_95": first,
                                "tail_slope_per_100": slope, "tail_mean": float(tail.mean()), "converged": ok,
                                "at_488": float(moving[488 - 99]) if len(moving) > 389 else None}
        reached.append(first)
        converged.append(ok)
        total = len(v)
    out["budget"] = int(math.ceil(max(reached) / 500) * 500) if all(converged) else total
    out["all_converged"] = all(converged)
    path = common.REPO_ROOT / "report" / "study3" / "tables" / "budget.json"
    path.write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
