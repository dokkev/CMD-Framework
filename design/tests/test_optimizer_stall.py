"""A single-objective run stops once its best has stopped improving (``stall_generations``)."""

from __future__ import annotations

import numpy as np
import pytest
from pymoo.core.problem import ElementwiseProblem

from core.optimization.optimizer import OptimizerSettings, initial_population, optimizer


class Bowl(ElementwiseProblem):
    def __init__(self, objectives: int = 1) -> None:
        super().__init__(n_var=3, n_obj=objectives, n_ieq_constr=1, xl=np.zeros(3), xu=np.full(3, 40.0))

    def _evaluate(self, x, out, *args, **kwargs):
        value = float(np.sum((np.asarray(x) - 17.0) ** 2))
        out["F"] = [value] * self.n_obj
        out["G"] = [5.0 - float(x[0])]  # a design needs x0 >= 5


def run(options, generations=300):
    problem = Bowl()
    settings = OptimizerSettings("ga", 20, generations, 0, options)
    return optimizer("ga").minimize(problem, initial_population(problem, np.empty((0, 3)), settings), settings)


def test_a_stalled_run_stops_early_with_the_same_best():
    full, stalled = run({}), run({"stall_generations": 40})
    assert full.evaluation_count == 20 * 300
    assert stalled.evaluation_count < full.evaluation_count / 2
    assert stalled.x == pytest.approx(full.x)  # the grid optimum, reached long before the stop
    assert stalled.x[0] == pytest.approx((17.0, 17.0, 17.0))


def test_a_patience_longer_than_the_run_changes_nothing():
    assert run({"stall_generations": 1000}, 60).evaluation_count == run({}, 60).evaluation_count == 20 * 60


def test_it_needs_one_objective():
    problem = Bowl(2)
    settings = OptimizerSettings("nsga2", 20, 10, 0, {"stall_generations": 5})
    with pytest.raises(ValueError):
        optimizer("nsga2").minimize(problem, initial_population(problem, np.empty((0, 3)), settings), settings)
