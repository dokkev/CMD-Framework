from dataclasses import dataclass
import math
from typing import Mapping

import numpy as np
import pytest

from cmd.optimization import (
    BoxVariable,
    FunctionalEqualityConstraint,
    FunctionalInequalityConstraint,
    FunctionalObjective,
    OptimizationProblem,
)
from cmd.optimization.solvers.pymoo import (
    NSGA2Config,
    PymooIntegrationError,
    PymooProblemAdapter,
    run_nsga2,
)


@dataclass(frozen=True)
class _MappingParameterization:
    variable_names: tuple[str, ...] = ("x", "y")

    def decode(self, named_values: Mapping[str, float]) -> dict[str, float]:
        return dict(named_values)


class _MappingEvaluator:
    def __init__(self) -> None:
        self.calls = 0

    def evaluate(self, parameters: dict[str, float]) -> dict[str, float]:
        self.calls += 1
        return parameters


def _problem(*, objectives=(), equalities=(), inequalities=(), evaluator=None):
    return OptimizationProblem(
        variables=(BoxVariable("x", 0.0, 1.0), BoxVariable("y", -2.0, 2.0)),
        parameterization=_MappingParameterization(),
        evaluator=evaluator or _MappingEvaluator(),
        objectives=objectives,
        equalities=equalities,
        inequalities=inequalities,
    )


def test_adapter_maps_bounds_objective_order_and_constraint_residuals() -> None:
    problem = _problem(
        objectives=(
            FunctionalObjective("maximize_x", "max", lambda context: context["x"]),
            FunctionalObjective("minimize_y", "min", lambda context: context["y"]),
        ),
        inequalities=(
            FunctionalInequalityConstraint(
                "first", lambda context: (context["x"] - 0.5, context["x"] + 0.5)
            ),
            FunctionalInequalityConstraint("second", lambda context: (context["y"] - 0.25,)),
        ),
        equalities=(
            FunctionalEqualityConstraint(
                "equality",
                0.1,
                lambda context: (context["x"] - 0.2, context["y"] - 0.5),
            ),
        ),
    )
    adapter = PymooProblemAdapter(problem, (0.2, 0.5))
    out = {}

    adapter._evaluate(np.asarray((0.4, 0.2)), out)

    assert adapter.n_var == 2
    assert adapter.n_obj == 2
    assert adapter.n_ieq_constr == 5
    assert adapter.xl == pytest.approx((0.0, -2.0))
    assert adapter.xu == pytest.approx((1.0, 2.0))
    assert out["F"] == pytest.approx((-0.4, 0.2))
    assert out["G"] == pytest.approx((-0.1, 0.9, -0.05, 0.1, 0.2))


def test_adapter_evaluates_cmd_once_per_candidate_and_exposes_latest_result() -> None:
    evaluator = _MappingEvaluator()
    problem = _problem(
        objectives=(FunctionalObjective("x", "min", lambda context: context["x"]),),
        evaluator=evaluator,
    )
    adapter = PymooProblemAdapter(problem, (0.2, 0.0))
    assert evaluator.calls == 1  # prototype shape evaluation

    adapter._evaluate(np.asarray((0.7, 0.0)), {})

    assert evaluator.calls == 2
    assert adapter.last_evaluation.design_vector == pytest.approx((0.7, 0.0))


def test_nonfinite_objective_is_transported_without_changing_raw_evaluation() -> None:
    problem = _problem(
        objectives=(
            FunctionalObjective(
                "incomplete_objective",
                "min",
                lambda context: math.inf if context["x"] > 0.5 else context["x"],
            ),
        ),
        inequalities=(FunctionalInequalityConstraint("finite_g", lambda context: (-1.0,)),),
    )
    adapter = PymooProblemAdapter(
        problem,
        (0.2, 0.0),
        nonfinite_objective_penalty=12345.0,
    )
    out = {}

    adapter._evaluate(np.asarray((0.8, 0.0)), out)

    assert out["F"] == pytest.approx((12345.0,))
    assert np.isfinite(out["F"]).all()
    assert adapter.last_evaluation.raw_objective_values["incomplete_objective"] == math.inf
    assert adapter.last_evaluation.minimization_objective_values[
        "incomplete_objective"
    ] == math.inf


def test_adapter_rejects_candidate_dependent_constraint_shape() -> None:
    problem = _problem(
        objectives=(FunctionalObjective("x", "min", lambda context: context["x"]),),
        inequalities=(
            FunctionalInequalityConstraint(
                "variable_shape",
                lambda context: (context["x"],)
                if context["x"] < 0.5
                else (context["x"], -1.0),
            ),
        ),
    )
    adapter = PymooProblemAdapter(problem, (0.2, 0.0))

    with pytest.raises(PymooIntegrationError, match="variable_shape.*length changed"):
        adapter._evaluate(np.asarray((0.8, 0.0)), {})


@pytest.mark.parametrize("bad_value", (math.inf, math.nan))
def test_adapter_rejects_nonfinite_constraint_with_candidate_context(bad_value) -> None:
    problem = _problem(
        objectives=(FunctionalObjective("x", "min", lambda context: context["x"]),),
        inequalities=(
            FunctionalInequalityConstraint(
                "bad_constraint",
                lambda context: (bad_value if context["x"] > 0.5 else -1.0,),
            ),
        ),
    )
    adapter = PymooProblemAdapter(problem, (0.2, 0.0))

    with pytest.raises(
        PymooIntegrationError,
        match=r"bad_constraint.*residual_index=0.*design_vector=\(0.8, 0.0\)",
    ):
        adapter._evaluate(np.asarray((0.8, 0.0)), {})


def test_nsga2_rejects_out_of_bounds_initial_design_instead_of_clipping() -> None:
    problem = _problem(
        objectives=(FunctionalObjective("x", "min", lambda context: context["x"]),)
    )

    with pytest.raises(ValueError, match=r"initial_designs\[0\].*outside"):
        run_nsga2(
            problem,
            (0.5, 0.0),
            NSGA2Config(population_size=4, generations=1, seed=1),
            initial_designs=((1.1, 0.0),),
        )


@pytest.mark.parametrize(
    "kwargs",
    [
        {"population_size": 1, "generations": 1, "seed": 0},
        {"population_size": 2, "generations": 0, "seed": 0},
        {"population_size": 2, "generations": 1, "seed": -1},
    ],
)
def test_nsga2_config_rejects_invalid_run_dimensions(kwargs) -> None:
    with pytest.raises(ValueError):
        NSGA2Config(**kwargs)


def test_tiny_nsga2_run_returns_cmd_reevaluations() -> None:
    problem = _problem(
        objectives=(
            FunctionalObjective("left", "min", lambda context: context["x"]),
            FunctionalObjective("right", "min", lambda context: 1.0 - context["x"]),
        ),
        inequalities=(
            FunctionalInequalityConstraint("y_floor", lambda context: (0.2 - context["y"],)),
        ),
    )
    result = run_nsga2(
        problem,
        (0.5, 0.5),
        NSGA2Config(population_size=8, generations=3, seed=7),
        initial_designs=((0.5, 0.5),),
    )

    assert result.pymoo_version
    assert result.evaluation_count >= 8
    assert result.final_population
    assert all(len(item.design_vector) == 2 for item in result.final_population)
    assert result.pareto_candidates
    assert all(
        set(item.raw_objective_values) == {"left", "right"}
        for item in result.pareto_candidates
    )
