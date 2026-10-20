"""The Stage-0 objectives, and which of them a study optimizes.

Every objective is minimized by ``value``; ``column`` is the evaluation
field people read for it (in ``pareto.csv`` and ``seeds.csv``), better when
larger if ``larger_is_better``, and ``role`` names the seed that is best at
it. A study optimizes :func:`active_objectives` of its config: the ones it
lists, or else the default ones: ``transmission`` and ``curl``. The pads are
requirements (minimum or fixed lengths), not an objective; ``support_length``
(less when every pad's length is fixed, a constant) and ``flatness`` are
there to opt into.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from core.optimization.stage0.config import FingerStudyConfig
from core.optimization.stage0.evaluate import INVALID, FingerEvaluation


@dataclass(frozen=True)
class Objective:
    value: Callable[[FingerEvaluation], float]
    """What the optimizer minimizes."""
    column: str
    label: str
    role: str
    larger_is_better: bool
    default: bool = True
    """Optimized when the config does not list its objectives."""


def _finite(value: float) -> float:
    # An unswept design has no k_max: rank it behind every swept one.
    return value if math.isfinite(value) else INVALID


OBJECTIVES = MappingProxyType({
    "transmission": Objective(
        value=lambda e: _finite(e.torque_ratio_max), column="torque_ratio_max",
        label="worst torque ratio k_max over the usable range", role="best_transmission",
        larger_is_better=False),
    "flatness": Objective(
        value=lambda e: _finite(e.torque_ratio_spread), column="torque_ratio_spread",
        label="torque ratio spread k_max / k_min over the usable range", role="best_flatness",
        larger_is_better=False, default=False),
    "support_length": Objective(
        value=lambda e: -e.support_length_mm, column="support_length_mm",
        label="pad length, proximal + distal a-b [mm]", role="best_length",
        larger_is_better=True, default=False),
    "curl": Objective(
        value=lambda e: e.curl_shortfall_deg, column="phi_safe_deg",
        label="phi_safe_max [deg]", role="best_curl", larger_is_better=True),
})
"""Every objective, by the name configs use."""


def active_objectives(config: FingerStudyConfig) -> tuple[str, ...]:
    """The objectives a study optimizes, in :data:`OBJECTIVES` order."""

    if config.objectives is not None:
        unknown = set(config.objectives) - set(OBJECTIVES)
        if unknown:
            raise ValueError(f"unknown objectives {sorted(unknown)}; known: {list(OBJECTIVES)}")
        return tuple(name for name in OBJECTIVES if name in config.objectives)
    packaging = config.packaging
    fixed = packaging.proximal_pad.length_mm is not None and packaging.distal_pad.length_mm is not None
    return tuple(name for name, objective in OBJECTIVES.items()
                 if objective.default and not (fixed and name == "support_length"))


def objective_values(evaluation: FingerEvaluation, names: Sequence[str]) -> tuple[float, ...]:
    """The named objectives of one evaluation, as the optimizer minimizes them."""

    return tuple(OBJECTIVES[name].value(evaluation) for name in names)


def seed_roles(
    objectives: Sequence[Sequence[float]],
    columns: Sequence[Mapping[str, float]],
    names: Sequence[str],
) -> dict[str, int]:
    """The representative seeds of a front: role -> candidate index.

    ``objectives`` are the candidates' minimized objectives and ``columns``
    their reported values (at least each objective's column and
    ``torque_ratio_max``). Each objective's ``role`` goes to the candidate best
    at it (ties to the better transmission); ``balanced`` to the one closest to
    the ideal point with each objective scaled to ``[0, 1]`` over the front.
    """

    if not objectives:
        return {}
    scores = np.asarray(objectives, dtype=float)
    low, high = scores.min(axis=0), scores.max(axis=0)
    scaled = (scores - low) / np.where(high > low, high - low, 1.0)
    roles = {}
    for name in names:
        objective = OBJECTIVES[name]
        sign = 1.0 if objective.larger_is_better else -1.0
        roles[objective.role] = max(range(len(columns)), key=lambda i: (
            sign * columns[i][objective.column], -columns[i]["torque_ratio_max"]))
    roles["balanced"] = int(np.argmin(np.linalg.norm(scaled, axis=1)))
    return roles


def role_order(names: Sequence[str]) -> list[str]:
    """The seed roles of a study, in the order they are listed."""

    return [OBJECTIVES[name].role for name in names] + ["balanced"]


__all__ = ["OBJECTIVES", "Objective", "active_objectives", "objective_values", "role_order", "seed_roles"]
