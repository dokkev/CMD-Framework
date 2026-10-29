#!/usr/bin/env python3
"""Grow the pads of a linkage front's designs as far as they stay usable.

    python scripts/optimize/extend_pads.py <portfolio dir> [--all] [--to-required]

The study searched with each pad on its own link (``goal.pads.on_links``):
the proximal pad to ``A``, the distal one from ``B``. Each design keeps its
links (so its ``k``) and gets the pad ends ``A''`` (toward ``B``) and
``B''`` (toward ``A``) that give the most coverage (``pad_coverage``: each
face up to where the two faces cross in the fold) while the design stays
feasible and flexes as far as it did (its ``phi_max``, so its curl;
``--to-required``: only as far as the required flexion, trading curl for
pad): a grid of EXTENSION_STEP_MM up to EXTENSION_MAX_MM each, refined to
REFINE_STEP_MM (screened every SCREEN_STEP_DEG of servo), the best then
checked on the study's verification grid. Of
equal coverage the shortest extension is taken, then each cut back as far
as the coverage holds (a pad grown past where its face folds in adds nothing). ``--all``: every design of the front; else its
representatives (``balanced`` and the like).

Writes ``seeds_extended.csv`` next to the portfolio's ``seeds.csv``: the
extended designs (their variables and report), with ``coverage_on_links``
(the searched coverage) and the extensions.
"""

from __future__ import annotations

import csv
import functools
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.linkage.design import VARIABLE_NAMES  # noqa: E402
from core.optimization.mechanism import mechanism  # noqa: E402
from core.optimization.study import load_run  # noqa: E402

EXTENSION_STEP_MM = 1.0
EXTENSION_MAX_MM = 30.0
REFINE_STEP_MM = 0.2
SCREEN_STEP_DEG = 0.2
"""The screening's servo step: fine enough that a flexion lost to a grown pad shows."""
TRIES = 60
"""Candidates checked on the verification grid, best first, before giving up."""
TIE_COVERAGE = 1e-4
"""Coverages closer than this count as equal (then the shorter extension wins)."""

MECHANISM = mechanism("linkage")
APP, BPP = VARIABLE_NAMES.index("app_offset_mm"), VARIABLE_NAMES.index("bpp_offset_mm")


@functools.lru_cache(maxsize=16)
def _configs(run_dir: str):
    """The screening's config (SCREEN_STEP_DEG) and the verification grid's."""

    config = load_run(Path(run_dir)).config
    return (MECHANISM.refine(config, math.radians(SCREEN_STEP_DEG)),
            MECHANISM.refine(config, config.verify_step_rad))


def _extended(x: list[float], proximal_mm: float, distal_mm: float) -> list[float]:
    """``x`` with ``A''`` moved ``proximal_mm`` toward ``B`` and ``B''`` ``distal_mm`` toward ``A``."""

    y = list(x)
    y[APP] = round(x[APP] + proximal_mm, 9)
    y[BPP] = round(x[BPP] + distal_mm, 9)
    return y


def _score(x, config, phi_max_rad: float) -> float:
    e = MECHANISM.evaluate(x, config)
    if e is None or not e.feasible or e.phi_max_rad < phi_max_rad - 1e-9:
        return -math.inf
    return e.pad_coverage


def extend(task: tuple[str, list[float], bool]) -> tuple[list[float], dict, float, float]:
    run_dir, x, to_required = task
    config, verify = _configs(run_dir)
    base = MECHANISM.evaluate(x, verify)
    # The flexion to keep: the design's own (verified; the screening compares
    # on its own grid), or only the required one (feasibility holds it).
    phi_max = 0.0 if to_required else base.phi_max_rad
    coarse_phi_max = 0.0 if to_required else MECHANISM.evaluate(x, config).phi_max_rad
    grid = np.arange(0.0, EXTENSION_MAX_MM + 1e-9, EXTENSION_STEP_MM)
    scored = [(_score(_extended(x, p, d), config, coarse_phi_max), p, d) for p in grid for d in grid]
    _, p0, d0 = max(scored)
    fine = np.arange(-EXTENSION_STEP_MM, EXTENSION_STEP_MM + 1e-9, REFINE_STEP_MM)
    scored += [(_score(_extended(x, max(p0 + dp, 0.0), max(d0 + dd, 0.0)), config, coarse_phi_max),
                max(p0 + dp, 0.0), max(d0 + dd, 0.0)) for dp in fine for dd in fine]
    ranked = sorted(scored, key=lambda item: (round(item[0] / TIE_COVERAGE) if math.isfinite(item[0])
                                              else -math.inf, -(item[1] + item[2])), reverse=True)
    def verified(p: float, d: float):
        e = MECHANISM.evaluate(_extended(x, p, d), verify)
        return e if e is not None and e.feasible and e.phi_max_rad >= phi_max - 1e-9 else None

    for score, p, d in ranked[:TRIES]:
        if not math.isfinite(score):
            break
        e = verified(p, d)
        if e is None:
            continue
        # Each extension back as far as the coverage holds: no pad past where it counts.
        for side in (0, 1):
            low, high = 0.0, (p, d)[side]
            while high - low > REFINE_STEP_MM / 2:
                mid = round((low + high) / 2, 3)
                trial = verified(*((mid, d) if side == 0 else (p, mid)))
                if trial is not None and trial.pad_coverage >= e.pad_coverage - TIE_COVERAGE:
                    high = mid
                else:
                    low = mid
            if high < (p, d)[side]:
                p, d = (high, d) if side == 0 else (p, high)
                e = verified(p, d)
        return _extended(x, p, d), MECHANISM.report(e), float(p), float(d)
    return list(x), MECHANISM.report(base), 0.0, 0.0


def main(argv: list[str]) -> int:
    if not argv or argv[0].startswith("-"):
        print(__doc__.strip())
        return 2
    portfolio = Path(argv[0]).resolve()
    rows = list(csv.DictReader((portfolio / "seeds.csv").open(encoding="utf-8", newline="")))
    if "--all" not in argv:
        rows = [row for row in rows if row["seed_role"] != "pareto"]
    to_required = "--to-required" in argv
    tasks = [(str(portfolio.parent / row["run"].split("/")[-1]), MECHANISM.read_design(row), to_required)
             for row in rows]
    with ProcessPoolExecutor(os.cpu_count() or 1) as pool:
        results = list(pool.map(extend, tasks, chunksize=1))
    out = portfolio / ("seeds_extended_to_required.csv" if to_required else "seeds_extended.csv")
    columns = None
    with out.open("w", encoding="utf-8", newline="") as stream:
        writer = None
        for row, (y, report, p, d) in zip(rows, results):
            values = {"seed_role": row["seed_role"], "run": row["run"], "candidate": row["candidate"],
                      **dict(zip(VARIABLE_NAMES, y)), **report,
                      "coverage_on_links": row["pad_coverage"], "proximal_extension_mm": p,
                      "distal_extension_mm": d}
            if writer is None:
                columns = list(values)
                writer = csv.DictWriter(stream, columns)
                writer.writeheader()
            writer.writerow(values)
            if row["seed_role"] != "pareto":
                print(f"{row['seed_role']:28s} coverage {100 * float(row['pad_coverage']):5.1f} -> "
                      f"{100 * float(report['pad_coverage']):5.1f} %  (A'' +{p:.1f}, B'' -{d:.1f} mm; "
                      f"P + D {report['proximal_effective_mm']:.1f} + {report['distal_effective_mm']:.1f}, "
                      f"k {report['torque_ratio_max']:.2f}, phi_max {report['phi_max_deg']:.1f})")
    print(f"wrote {len(rows)} designs -> {out.relative_to(REPOSITORY_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
