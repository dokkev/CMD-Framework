from dataclasses import dataclass
from typing import Mapping

import pytest

from cmd.optimization import (
    BoxVariable,
    FunctionalEqualityConstraint,
    FunctionalInequalityConstraint,
    FunctionalObjective,
    OptimizationProblem,
)


@dataclass(frozen=True)
class _NamedParameterization:
    variable_names: tuple[str, ...] = ("x", "y")

    def decode(self, named_values: Mapping[str, float]) -> tuple[tuple[str, float], ...]:
        return tuple(named_values.items())


@dataclass(frozen=True)
class _MappingEvaluator:
    def evaluate(self, parameters: tuple[tuple[str, float], ...]) -> dict[str, float]:
        return dict(parameters)


def make_problem(
    *,
    objectives=(),
    equalities=(),
    inequalities=(),
) -> OptimizationProblem:
    return OptimizationProblem(
        variables=(BoxVariable("x", -1.0, 1.0), BoxVariable("y", 0.0, 2.0)),
        parameterization=_NamedParameterization(),
        evaluator=_MappingEvaluator(),
        objectives=objectives,
        equalities=equalities,
        inequalities=inequalities,
    )


@pytest.mark.parametrize(
    "name,lower,upper",
    [
        ("", 0.0, 1.0),
        ("x", 1.0, 1.0),
        ("x", 2.0, 1.0),
        ("x", float("-inf"), 1.0),
        ("x", 0.0, float("nan")),
    ],
)
def test_box_variable_rejects_invalid_names_and_bounds(name, lower, upper) -> None:
    with pytest.raises(ValueError):
        BoxVariable(name, lower, upper)


def test_positional_vector_maps_to_named_values_in_declared_order() -> None:
    objective = FunctionalObjective(
        name="weighted_sum",
        sense="max",
        function=lambda context: context["x"] + 2.0 * context["y"],
    )
    result = make_problem(objectives=(objective,)).evaluate((0.25, 0.5))

    assert tuple(result.variable_values.items()) == (("x", 0.25), ("y", 0.5))
    assert result.decoded_parameters == (("x", 0.25), ("y", 0.5))
    assert result.raw_objective_values == {"weighted_sum": 1.25}
    assert result.minimization_objective_values == {"weighted_sum": -1.25}


def test_box_violations_are_reported_and_make_problem_infeasible() -> None:
    result = make_problem().evaluate((2.0, 0.5))

    assert result.box_violations == {"x": 1.0, "y": 0.0}
    assert not result.feasible


def test_equality_uses_zero_residual_with_absolute_tolerance() -> None:
    equality = FunctionalEqualityConstraint(
        name="coordinates_match",
        tolerance=0.1,
        function=lambda context: (context["x"] - 1.0, context["y"] - 2.0),
    )
    passing = make_problem(equalities=(equality,)).evaluate((1.05, 1.95))
    failing = make_problem(equalities=(equality,)).evaluate((0.75, 2.0))

    assert passing.equality_results[0].residuals == pytest.approx((0.05, -0.05))
    assert passing.equality_results[0].maximum_violation == 0.0
    assert passing.equality_results[0].satisfied
    assert failing.equality_results[0].maximum_violation == pytest.approx(0.15)
    assert not failing.equality_results[0].satisfied


def test_inequality_uses_nonpositive_residual_and_preserves_vectors() -> None:
    inequality = FunctionalInequalityConstraint(
        name="two_sided_vector",
        function=lambda context: (
            context["x"] - 1.0,
            context["y"] - 2.0,
            -1.0,
        ),
    )
    result = make_problem(inequalities=(inequality,)).evaluate((0.5, 1.5))
    diagnostic = result.inequality_results[0]

    assert diagnostic.residuals == (-0.5, -0.5, -1.0)
    assert diagnostic.maximum_violation == 0.0
    assert diagnostic.satisfied

    violating = make_problem(inequalities=(inequality,)).evaluate((1.5, 2.5))
    assert violating.inequality_results[0].residuals == (0.5, 0.5, -1.0)
    assert violating.inequality_results[0].maximum_violation == 0.5
    assert not violating.feasible


def test_problem_composition_allows_objectives_and_constraints_to_be_omitted() -> None:
    objective = FunctionalObjective("x", "min", lambda context: context["x"])
    unrestricted = make_problem().evaluate((0.0, 1.0))
    composed = make_problem(
        objectives=(objective,),
        inequalities=(
            FunctionalInequalityConstraint("x_nonnegative", lambda context: (-context["x"],)),
        ),
    ).evaluate((0.0, 1.0))

    assert unrestricted.raw_objective_values == {}
    assert unrestricted.feasible
    assert composed.raw_objective_values == {"x": 0.0}
    assert composed.feasible
