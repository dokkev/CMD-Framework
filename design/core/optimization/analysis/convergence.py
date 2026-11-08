"""How a run converged: the best value of one objective after each generation,
from the run's ``history.npz`` (every feasible evaluation's objectives, as
the optimizer minimized them, and its generation)."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Convergence:
    """One run's running best of one objective, per generation."""

    objective: str
    best: np.ndarray
    """``(generations,)``: the best value found up to each generation, in the
    objective's own terms (``pad_coverage`` as a share, larger better); NaN
    before the first feasible design."""
    evaluations: np.ndarray
    """``(generations,)``: evaluations made up to each generation."""

    @property
    def final(self) -> float:
        return float(self.best[-1])

    def generation_within(self, tolerance: float) -> int | None:
        """The first generation (0-based) whose best is within ``tolerance``
        of the final (``None``: none feasible)."""

        if not math.isfinite(self.final):
            return None
        close = np.flatnonzero(np.abs(self.best - self.final) <= tolerance)
        return int(close[0]) if close.size else None

    def gain_after(self, share: float) -> float:
        """How much the best improved over the last ``1 - share`` of the
        generations (0 when it had settled by then)."""

        index = min(max(int(math.ceil(share * len(self.best))) - 1, 0), len(self.best) - 1)
        before = self.best[index]
        return 0.0 if not math.isfinite(before) else abs(self.final - float(before))


def best_by_generation(history: Mapping[str, np.ndarray], objective: str, larger_is_better: bool) -> Convergence:
    """The running best of ``objective`` (stored minimized: negated when
    ``larger_is_better``) after each generation."""

    names = [str(name) for name in history["objective_names"]]
    column = names.index(objective)
    generations = len(history["evaluations"])
    values = np.asarray(history["objectives"], dtype=float)[:, column] if len(history["objectives"]) else np.empty(0)
    per_generation = np.full(generations, np.inf)
    if values.size:
        np.minimum.at(per_generation, np.asarray(history["generation"], dtype=int), values)
    running = np.minimum.accumulate(per_generation)
    best = np.where(np.isfinite(running), -running if larger_is_better else running, np.nan)
    return Convergence(objective, best, np.cumsum(np.asarray(history["evaluations"], dtype=int)))


def load_history(run_dir: Path) -> dict[str, np.ndarray] | None:
    """A run's ``history.npz`` (``None``: saved before it was recorded)."""

    path = Path(run_dir) / "history.npz"
    return dict(np.load(path)) if path.exists() else None


__all__ = ["Convergence", "best_by_generation", "load_history"]
