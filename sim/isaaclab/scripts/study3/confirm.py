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

3A main (seeds 0-9 per surface, ``-Main`` tasks, logs/study3_main/eval, eval seed 2000, fixed goal sequence),
J = goals reached per episode; three primaries, Holm-corrected at alpha 0.05, two-sided:
- P1: in-domain interface effect, mean J(crowned policies on crowned) - mean J(flat policies on flat); exact
  permutation over seeds;
- P2: frozen transfer flat -> crowned, J(flat policy, flat) - J(flat policy, crowned); exact sign flip;
- P3: frozen transfer crowned -> flat, J(crowned policy, crowned) - J(crowned policy, flat); exact sign flip;
each with a 95% percentile bootstrap interval over seeds (10,000 resamples, fixed generator).

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


def holm(pvalues: dict[str, float]) -> dict[str, float]:
    """Holm-adjusted p-values (step-down, monotone, capped at 1)."""
    order = sorted(pvalues, key=pvalues.get)
    adjusted, running = {}, 0.0
    for rank, key in enumerate(order):
        running = max(running, min(1.0, (len(order) - rank) * pvalues[key]))
        adjusted[key] = running
    return adjusted


def bootstrap(fn, *samples, n: int = 10_000, seed: int = 0) -> list[float]:
    """95% percentile interval of ``fn`` over seed-level resamples of each sample independently."""
    rng = np.random.default_rng(seed)
    stats_ = [fn(*[np.asarray(x)[rng.integers(0, len(x), len(x))] for x in samples]) for _ in range(n)]
    return [float(np.percentile(stats_, 2.5)), float(np.percentile(stats_, 97.5))]


def three_a_main() -> dict:
    root = common.SIM_ROOT / "logs" / "study3_main" / "eval"
    if not root.exists():
        return {"status": "not run"}
    cells = [c for c in analyze.cell_rows(analyze.evaluations(root)) if c["physics"] == "newton_mjwarp"]
    J = {(tv, xv): by_seed(cells, tv, xv, "newton_mjwarp", "goals_reached_mean")
         for tv in ("flat", "crowned") for xv in ("flat", "crowned")}
    ff, cc = J[("flat", "flat")], J[("crowned", "crowned")]
    g_fc = [ff[s] - J[("flat", "crowned")][s] for s in sorted(set(ff) & set(J[("flat", "crowned")]))]
    g_cf = [cc[s] - J[("crowned", "flat")][s] for s in sorted(set(cc) & set(J[("crowned", "flat")]))]
    a, b = list(ff.values()), list(cc.values())
    raw = {"P1": permutation(a, b), "P2": sign_flip(g_fc), "P3": sign_flip(g_cf)}
    adj = holm(raw)
    return {
        "seeds": {k: sorted(v) for k, v in {"flat": ff, "crowned": cc}.items()},
        "J": {f"{tv}_on_{xv}": summary(v.values()) for (tv, xv), v in J.items() if v},
        "P1_in_domain_crowned_minus_flat": {"estimate": float(np.mean(b) - np.mean(a)), "ci95": bootstrap(
            lambda x, y: y.mean() - x.mean(), a, b), "p": raw["P1"], "p_holm": adj["P1"]},
        "P2_transfer_flat_to_crowned": {**summary(g_fc), "ci95": bootstrap(lambda x: x.mean(), g_fc),
                                        "positive": int(sum(g > 0 for g in g_fc)), "p": raw["P2"], "p_holm": adj["P2"]},
        "P3_transfer_crowned_to_flat": {**summary(g_cf), "ci95": bootstrap(lambda x: x.mean(), g_cf),
                                        "positive": int(sum(g > 0 for g in g_cf)), "p": raw["P3"], "p_holm": adj["P3"]},
        "validity": {"loop_valid_fraction_min": min(float(c["valid_fraction"]) for c in cells), "cells": len(cells)},
    }


def sign_flip_random(values: list[float], n: int = 100_000, seed: int = 0) -> float:
    """Two-sided sign-flip p-value from ``n`` random sign patterns (fixed generator), counting the observed one."""
    v = np.asarray(values, dtype=float)
    rng = np.random.default_rng(seed)
    signs = rng.choice((-1.0, 1.0), size=(n, len(v)))
    null = np.abs((signs * v).mean(axis=1))
    return float((1 + np.sum(null >= abs(v.mean()) - 1e-12)) / (n + 1))


MODELS = {"S1_pyramidal": "eval_cm_pyramidal", "S2_impratio1": "eval_cm_impratio1"}


def three_b_main() -> dict:
    """3B: change of each 3A-main policy's interface effect between contact models (docs/decisions.md)."""
    main_root = common.SIM_ROOT / "logs" / "study3_main"
    def effects(root: Path) -> dict[tuple[str, int], float]:
        if not root.exists():
            return {}
        cells = analyze.cell_rows(analyze.evaluations(root))
        out = {}
        for tv in ("flat", "crowned"):
            on_c = by_seed(cells, tv, "crowned", "newton_mjwarp", "goals_reached_mean")
            on_f = by_seed(cells, tv, "flat", "newton_mjwarp", "goals_reached_mean")
            out.update({(tv, s): on_c[s] - on_f[s] for s in set(on_c) & set(on_f)})
        return out

    base = effects(main_root / "eval")
    if not base:
        return {"status": "not run"}
    reference = float(np.mean(list(base.values())))
    result = {"interface_effect_S0": {**summary(base.values()), "by_policy_group": {
        tv: summary(v for (g, _), v in base.items() if g == tv) for tv in ("flat", "crowned")}}}
    raw = {}
    for name, folder in MODELS.items():
        eff = effects(main_root / folder)
        keys = sorted(set(eff) & set(base))
        if not keys:
            result[name] = {"status": "not run"}
            continue
        d = [eff[k] - base[k] for k in keys]
        raw[name] = sign_flip_random(d)
        ci = bootstrap(lambda x: x.mean(), d)
        band = 0.2 * abs(reference)
        verdict = ("robust" if -band <= ci[0] and ci[1] <= band
                   else "model-dependent" if ci[0] > 0 or ci[1] < 0 else "inconclusive")
        result[name] = {"delta_model": summary(d), "ci95": ci, "band": band, "verdict": verdict, "p": raw[name],
                        "interface_effect": summary(eff[k] for k in keys),
                        "by_policy_group": {tv: summary(eff[k] - base[k] for k in keys if k[0] == tv)
                                            for tv in ("flat", "crowned")}}
    for name, p in holm(raw).items():
        result[name]["p_holm"] = p
    return result


def main() -> int:
    result = {"3A": three_a(), "3B_ema": three_b(), "3A_main": three_a_main(), "3B_main": three_b_main()}
    path = common.REPO_ROOT / "report" / "study3" / "tables" / "confirmatory.json"
    path.write_text(json.dumps(result, indent=2, default=str) + "\n")
    print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != "table"} for k, v in result.items()}, indent=1, default=str))
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
