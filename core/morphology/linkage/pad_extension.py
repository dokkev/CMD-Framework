"""Growing a linkage finger's pads along its links, the links kept.

A design whose pads sit on their own links (the proximal pad to ``A``, the
distal one from ``B``) keeps its four-bar, so its ``k``, and gets the pad
ends ``A''`` (moved toward ``B``) and ``B''`` (toward ``A``) that give the
most coverage while the design stays acceptable: a grid of
:data:`EXTENSION_STEP_MM` up to :data:`EXTENSION_MAX_MM` each, refined to
:data:`REFINE_STEP_MM` about the best, screened by the caller's ``screen``;
the best then checked by the caller's ``check`` (on a finer motion). Of
equal coverage the shortest extension is taken, then each cut back as far
as the coverage holds (a pad grown past where its face folds in adds nothing).
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from typing import TypeVar

import numpy as np

from core.morphology.linkage.design import VARIABLE_NAMES

EXTENSION_STEP_MM = 1.0
EXTENSION_MAX_MM = 30.0
REFINE_STEP_MM = 0.2
TRIES = 60
"""Candidates checked, best first, before giving up."""
TIE_COVERAGE = 1e-4
"""Coverages closer than this count as equal (then the shorter extension wins)."""

APP, BPP = VARIABLE_NAMES.index("app_offset_mm"), VARIABLE_NAMES.index("bpp_offset_mm")

Checked = TypeVar("Checked")


def extended(x: Sequence[float], proximal_mm: float, distal_mm: float) -> list[float]:
    """The design ``x`` with ``A''`` moved ``proximal_mm`` toward ``B`` and ``B''`` ``distal_mm`` toward ``A``."""

    y = list(x)
    y[APP] = round(x[APP] + proximal_mm, 9)
    y[BPP] = round(x[BPP] + distal_mm, 9)
    return y


def grow_pads(x: Sequence[float], screen: Callable[[list[float]], float],
              check: Callable[[list[float]], Checked | None],
              coverage: Callable[[Checked], float]) -> tuple[list[float], float, float, Checked | None]:
    """The design with its pads grown as far as they add coverage.

    ``screen``: an extended design's coverage on the screening motion, or
    ``-inf`` when it is not acceptable there; ``check``: the design checked
    for good (``None``: not acceptable), whose coverage ``coverage`` reads.
    Returns the grown design, the two extensions (mm) and its check;
    ``(x, 0, 0, None)`` when no candidate passes.
    """

    grid = np.arange(0.0, EXTENSION_MAX_MM + 1e-9, EXTENSION_STEP_MM)
    scored = [(screen(extended(x, p, d)), p, d) for p in grid for d in grid]
    _, p0, d0 = max(scored)
    fine = np.arange(-EXTENSION_STEP_MM, EXTENSION_STEP_MM + 1e-9, REFINE_STEP_MM)
    scored += [(screen(extended(x, max(p0 + dp, 0.0), max(d0 + dd, 0.0))),
                max(p0 + dp, 0.0), max(d0 + dd, 0.0)) for dp in fine for dd in fine]
    ranked = sorted(scored, key=lambda item: (round(item[0] / TIE_COVERAGE) if math.isfinite(item[0])
                                              else -math.inf, -(item[1] + item[2])), reverse=True)
    for score, p, d in ranked[:TRIES]:
        if not math.isfinite(score):
            break
        e = check(extended(x, p, d))
        if e is None:
            continue
        # Each extension back as far as the coverage holds: no pad past where it counts.
        for side in (0, 1):
            low, high = 0.0, (p, d)[side]
            while high - low > REFINE_STEP_MM / 2:
                mid = round((low + high) / 2, 3)
                trial = check(extended(x, *((mid, d) if side == 0 else (p, mid))))
                if trial is not None and coverage(trial) >= coverage(e) - TIE_COVERAGE:
                    high = mid
                else:
                    low = mid
            if high < (p, d)[side]:
                p, d = (high, d) if side == 0 else (p, high)
                e = check(extended(x, p, d))
        return extended(x, p, d), p, d, e
    return list(x), 0.0, 0.0, None


__all__ = ["EXTENSION_MAX_MM", "EXTENSION_STEP_MM", "REFINE_STEP_MM", "TIE_COVERAGE", "TRIES", "extended",
           "grow_pads"]
