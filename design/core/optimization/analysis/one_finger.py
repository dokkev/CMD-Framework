"""A finger frozen (its mechanism, actuation and structure) with the pads on it
changed for pads of another thickness, each as long as its own support and
mounted by the rule of A.6.2 (problem formulation). Used to check Study 0's
relations on a fixed finger (A.6.0): where the law says the pads first meet
against what the study's evaluator finds.

- :func:`fitted`: the pads that fill a finger's supports;
- :func:`mount`: the finger with those pads, evaluated;
- :func:`coverage`: its cylinder-based pad coverage.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from core.optimization.analysis.adaptation import Partition
from core.optimization.analysis.tradeoff import Modules, goal_of, with_modules
from core.optimization.mechanism import Mechanism


# One finger, pads of one thickness ----------------------------------------------

def fitted(partition: Partition, x: Sequence[float], frame, thickness_mm: float) -> Modules | None:
    """Pads of ``thickness_mm`` as long as the finger ``x``'s own supports (down
    to a micrometre), ``None`` where the finger has no supports to
    read or they are shorter than the study goal's least pads."""

    goal = goal_of(frame)
    packaging = goal.packaging
    try:
        external = partition.external(x, frame)
    except (ValueError, KeyError, ZeroDivisionError):
        return None
    start = packaging.proximal_pad_start_mm - packaging.proximal_pad.offset_mm
    # Down to a micrometre: a pad as long as its support must not pass it by rounding.
    lengths = [math.floor(value * 1e3) / 1e3
               for value in (external["proximal_span_mm"] - start, external["distal_span_mm"])]
    least = (goal.minimum_proximal_support_mm, goal.minimum_distal_support_mm)
    if any(not math.isfinite(length) or length <= 0.0 or length < floor - 1e-9
           for length, floor in zip(lengths, least)):
        return None
    return Modules(lengths[0], float(thickness_mm), lengths[1], float(thickness_mm))


def mount(base: Mechanism, partition: Partition, x: Sequence[float], frame, thickness_mm: float):
    """The finger ``x`` frozen, pads of ``thickness_mm`` filling its supports and
    mounted by the rule of A.6.2 under the config ``frame``: the composed
    design (``None``: they do not mount) and its evaluation (``None``: none)."""

    modules = fitted(partition, x, frame, thickness_mm)
    if modules is None:
        return None, None
    carried = with_modules(frame, modules)
    try:
        composed = partition.compose(partition.external(x, carried), partition.coordinate_values(x, carried), x,
                                     carried)
    except (ValueError, KeyError, ZeroDivisionError):
        composed = None
    if composed is None:
        return None, None
    try:
        evaluation = base.evaluate(composed, carried)
    except ValueError:
        evaluation = None
    return composed, evaluation


def coverage(base: Mechanism, evaluation) -> float:
    """The cylinder-based pad coverage of an evaluated design (0: none, or not feasible)."""

    if evaluation is None or not evaluation.feasible:
        return 0.0
    value = base.report(evaluation).get("c_pad_mean")
    return float(value) if value not in (None, "") and math.isfinite(float(value)) else 0.0


__all__ = ["coverage", "fitted", "mount"]
