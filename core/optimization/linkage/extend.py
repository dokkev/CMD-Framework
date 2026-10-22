"""Lengthen a design's proximal pad past ``A'`` for free.

The pads are requirements, not an objective, so the optimizer leaves
``A''_x - A'_x`` (``app_offset_mm``) wherever it lands. Afterwards each design
can take the longest offset that changes nothing it was chosen for: it stays
feasible with the same objective values (v1: ``k_max`` and ``phi_safe``; v2:
``k_max`` and ``|OC|``), and the pads keep the requested clearance.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from core.optimization.linkage.config import FingerStudyConfig
from core.optimization.linkage.design import VARIABLE_NAMES
from core.optimization.linkage.evaluate import FingerEvaluation
from core.optimization.linkage.mechanism import MECHANISM, LinkageMechanism

_OFFSET = VARIABLE_NAMES.index("app_offset_mm")


@dataclass(frozen=True)
class Extension:
    x: tuple[float, ...]
    """The design with its longest offset."""
    evaluation: FingerEvaluation
    added_mm: float
    """Offset added to the design's own."""


def longest_proximal_pad(
    x: Sequence[float], config: FingerStudyConfig, *, extra_clearance_mm: float = 0.0,
    step_mm: float = 0.1, most_mm: float = 40.0, mechanism: LinkageMechanism = MECHANISM,
) -> Extension:
    """The design with the longest ``app_offset_mm`` (on a ``step_mm`` grid, at
    most ``most_mm`` added) that keeps it feasible with the same objective
    values (``k_max`` and ``phi_safe`` in v1), every pad ``extra_clearance_mm``
    beyond the body clearance. ``mechanism``: the linkage formulation (v1 or v2).

    A longer pad only adds what can collide, so the first offset that fails
    ends the search (bisection over the grid).
    """

    x = tuple(float(v) for v in x)
    names = mechanism.active_objectives(config)
    base = mechanism.evaluate(x, config)
    if base is None or not base.feasible:
        raise ValueError("the design is not feasible as it is")
    objectives = mechanism.objective_values(base, names)

    def at(added: float) -> tuple[tuple[float, ...], FingerEvaluation | None]:
        changed = list(x)
        changed[_OFFSET] = round(x[_OFFSET] + added, 9)
        return tuple(changed), mechanism.evaluate(changed, config)

    def keeps(evaluation: FingerEvaluation | None) -> bool:
        return (evaluation is not None and evaluation.feasible
                and all(math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-9)
                        for a, b in zip(mechanism.objective_values(evaluation, names), objectives))
                and evaluation.pad_clearance_mm >= min(extra_clearance_mm, base.pad_clearance_mm))

    low, high = 0, int(math.floor(most_mm / step_mm + 1e-9))  # grid steps: low keeps, high may not
    if keeps(at(high * step_mm)[1]):
        low = high
    while high - low > 1:
        middle = (low + high) // 2
        if keeps(at(middle * step_mm)[1]):
            low = middle
        else:
            high = middle
    added = round(low * step_mm, 9)
    extended, evaluation = at(added)
    return Extension(extended, evaluation, added)


__all__ = ["Extension", "longest_proximal_pad"]
