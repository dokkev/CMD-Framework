# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3: the pre-registered confirmatory analyses (docs/decisions.md, 2026-11-21), nothing else.

    uv run python scripts/study3/confirm.py

3A (seeds 0-9, logs/study3_pilot/eval, MuJoCo-Warp):
- primary: TransferGap(flat -> crowned) of first-goal success over seeds; exact sign-flip test (all 2^n sign
  patterns) and paired t test, two-sided, alpha 0.05;
- secondary: in-domain held success, crowned - flat; exact permutation test over seeds.

3B' (seeds 0-4 with action EMA 0.1, logs/study3_pilot/eval_ema):
- interpretable only if every VBD cell has a loop-valid fraction >= 0.95 and the MuJoCo-Warp in-domain
  first-goal success of both surfaces is >= 0.2 (seed mean);
- primary: SolverGap(crowned, crowned) - SolverGap(flat, flat), SolverGap = success(MuJoCo-Warp) -
  success(VBD), first goal; exact permutation over seeds.

Writes report/study3/tables/confirmatory.json and prints it.
"""

from __future__ import annotations

import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy import stats

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze  # noqa: E402
import common  # noqa: E402


def sign_flip(values: list[float]) -> float:
    observed = abs(np.mean(values))
    count = sum(abs(np.mean([v * s for v, s in zip(values, signs)])) >= observed - 1e-12
                for signs in itertools.product((1, -1), repeat=len(values)))
    return count / 2 ** len(values)


def permutation(a: list[float], b: list[float]) -> float:
    pooled, n = np.array(list(a) + list(b)), len(a)
    observed = abs(np.mean(a) - np.mean(b))
    total = count = 0
    for chosen in itertools.combinations(range(len(pooled)), n):
        mask = np.zeros(len(pooled), dtype=bool)
        mask[list(chosen)] = True
        total += 1
        count += abs(pooled[mask].mean() - pooled[~mask].mean()) >= observed - 1e-12
    return count / total


def by_seed(cells, tv, xv, physics, metric):
    return {int(c["train_seed"]): float(c[metric]) for c in cells
            if c["train_variant"] == tv and c["test_variant"] == xv and c["physics"] == physics}


def summary(values):
    values = list(values)
    return {"n": len(values), "mean": float(np.mean(values)), "sd": float(np.std(values, ddof=1)),
            "values": [round(v, 4) for v in values]}


def three_a() -> dict:
    cells = analyze.cell_rows(analyze.evaluations(common.LOG_ROOT / "eval"))
    own = by_seed(cells, "flat", "flat", "newton_mjwarp", "success_first_goal")
    cross = by_seed(cells, "flat", "crowned", "newton_mjwarp", "success_first_goal")
    seeds = sorted(set(own) & set(cross))
    gaps = [own[s] - cross[s] for s in seeds]
    t = stats.ttest_1samp(gaps, 0.0)
    flat_held = list(by_seed(cells, "flat", "flat", "newton_mjwarp", "success_held").values())
    crowned_held = list(by_seed(cells, "crowned", "crowned", "newton_mjwarp", "success_held").values())
    return {
        "seeds": seeds,
        "primary_transfer_gap_flat_to_crowned": {**summary(gaps), "positive": int(sum(g > 0 for g in gaps)),
                                                 "sign_flip_p": sign_flip(gaps), "paired_t": float(t.statistic),
                                                 "paired_t_p": float(t.pvalue)},
        "secondary_held_crowned_minus_flat": {"flat": summary(flat_held), "crowned": summary(crowned_held),
                                              "difference": float(np.mean(crowned_held) - np.mean(flat_held)),
                                              "permutation_p": permutation(flat_held, crowned_held)},
    }


def three_b() -> dict:
    root = common.LOG_ROOT / "eval_ema"
    if not root.exists():
        return {"status": "not run"}
    cells = analyze.cell_rows(analyze.evaluations(root))
    vbd_valid = [float(c["valid_fraction"]) for c in cells if c["physics"] == "newton_vbd"]
    mj = {v: list(by_seed(cells, v, v, "newton_mjwarp", "success_first_goal").values()) for v in ("flat", "crowned")}
    gaps = {}
    for v in ("flat", "crowned"):
        a, b = by_seed(cells, v, v, "newton_mjwarp", "success_first_goal"), by_seed(cells, v, v, "newton_vbd", "success_first_goal")
        gaps[v] = [a[s] - b[s] for s in sorted(set(a) & set(b))]
    ok = bool(vbd_valid) and min(vbd_valid) >= 0.95 and all(len(mj[v]) and np.mean(mj[v]) >= 0.2 for v in mj)
    out = {
        "cells": len(cells),
        "vbd_valid_fraction_min": min(vbd_valid) if vbd_valid else None,
        "mjwarp_in_domain_success": {v: summary(mj[v]) for v in mj if mj[v]},
        "solver_gap": {v: summary(gaps[v]) for v in gaps if gaps[v]},
        "interpretable": ok,
        "table": [{k: c[k] for k in ("train_variant", "train_seed", "test_variant", "physics", "success_first_goal",
                                      "success_held", "drop_rate", "valid_fraction")} for c in cells],
    }
    if ok and all(len(gaps[v]) >= 2 for v in gaps):
        out["primary_solver_gap_crowned_minus_flat"] = {
            "difference": float(np.mean(gaps["crowned"]) - np.mean(gaps["flat"])),
            "permutation_p": permutation(gaps["flat"], gaps["crowned"])}
    return out


def main() -> int:
    result = {"3A": three_a(), "3B_ema": three_b()}
    path = common.REPO_ROOT / "report" / "study3" / "tables" / "confirmatory.json"
    path.write_text(json.dumps(result, indent=2, default=str) + "\n")
    print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != "table"} for k, v in result.items()}, indent=1, default=str))
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
