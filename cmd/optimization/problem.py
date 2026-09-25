"""Solver-independent composition and evaluation of optimization studies."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from numbers import Real
from types import MappingProxyType
from typing import Generic, Protocol, TypeVar

from .constraint import (
    ConstraintResult,
    EqualityConstraint,
    InequalityConstraint,
    equality_result,
    inequality_result,
)
from .objective import Objective, minimization_value
from .variable import BoxVariable


ParameterT = TypeVar("ParameterT")
ContextT = TypeVar("ContextT")


class Parameterization(Protocol[ParameterT]):
    """Map the ordered named design vector into a domain parameter object."""

    @property
    def variable_names(self) -> tuple[str, ...]: ...

    def decode(self, named_values: Mapping[str, float]) -> ParameterT: ...


class Evaluator(Protocol[ParameterT, ContextT]):
    """Evaluate one decoded domain parameterization into a context."""

    def evaluate(self, parameters: ParameterT) -> ContextT: ...


@dataclass(frozen=True)
class ProblemEvaluation(Generic[ParameterT, ContextT]):
    """Immutable diagnostics for one candidate evaluated by a problem."""

    design_vector: tuple[float, ...]
    variable_values: Mapping[str, float]
    decoded_parameters: ParameterT
    domain_evaluation: ContextT
    raw_objective_values: Mapping[str, float]
    minimization_objective_values: Mapping[str, float]
    box_violations: Mapping[str, float]
    equality_results: tuple[ConstraintResult, ...]
    inequality_results: tuple[ConstraintResult, ...]

    def __post_init__(self) -> None:
        for name in (
            "variable_values",
            "raw_objective_values",
            "minimization_objective_values",
            "box_violations",
        ):
            object.__setattr__(
                self,
                name,
                MappingProxyType(dict(getattr(self, name))),
            )
        object.__setattr__(self, "design_vector", tuple(self.design_vector))
        object.__setattr__(self, "equality_results", tuple(self.equality_results))
        object.__setattr__(self, "inequality_results", tuple(self.inequality_results))

    @property
    def feasible(self) -> bool:
        """Whether box bounds and all declared constraints are satisfied."""

        return (
            all(violation == 0.0 for violation in self.box_violations.values())
            and all(result.satisfied for result in self.equality_results)
            and all(result.satisfied for result in self.inequality_results)
        )


@dataclass(frozen=True)
class OptimizationProblem(Generic[ParameterT, ContextT]):
    """A deterministic, solver-agnostic candidate evaluation pipeline."""

    variables: Sequence[BoxVariable]
    parameterization: Parameterization[ParameterT]
    evaluator: Evaluator[ParameterT, ContextT]
    objectives: Sequence[Objective[ContextT]]
    equalities: Sequence[EqualityConstraint[ContextT]] = ()
    inequalities: Sequence[InequalityConstraint[ContextT]] = ()

    def __post_init__(self) -> None:
        variables = tuple(self.variables)
        objectives = tuple(self.objectives)
        equalities = tuple(self.equalities)
        inequalities = tuple(self.inequalities)
        if any(not isinstance(variable, BoxVariable) for variable in variables):
            raise TypeError("variables must contain only BoxVariable instances")
        _require_unique_names((variable.name for variable in variables), "variable")
        _require_unique_names((objective.name for objective in objectives), "objective")
        _require_unique_names(
            (constraint.name for constraint in equalities), "equality constraint"
        )
        _require_unique_names(
            (constraint.name for constraint in inequalities), "inequality constraint"
        )
        for objective in objectives:
            if objective.sense not in ("min", "max"):
                raise ValueError(
                    f"objective {objective.name!r} sense must be 'min' or 'max'"
                )

        variable_names = tuple(variable.name for variable in variables)
        parameter_names = tuple(self.parameterization.variable_names)
        if parameter_names != variable_names:
            raise ValueError(
                "variable order/names must exactly match parameterization.variable_names"
            )

        object.__setattr__(self, "variables", variables)
        object.__setattr__(self, "objectives", objectives)
        object.__setattr__(self, "equalities", equalities)
        object.__setattr__(self, "inequalities", inequalities)

    def evaluate(self, design_vector: Sequence[float]) -> ProblemEvaluation[ParameterT, ContextT]:
        """Decode and evaluate one positional vector in declared order."""

        values = tuple(design_vector)
        if len(values) != len(self.variables):
            raise ValueError(
                f"design vector has {len(values)} values; expected {len(self.variables)}"
            )

        named_values: dict[str, float] = {}
        box_violations: dict[str, float] = {}
        for variable, raw_value in zip(self.variables, values):
            value = _finite_design_value(variable.name, raw_value)
            named_values[variable.name] = value
            box_violations[variable.name] = variable.violation(value)

        parameters = self.parameterization.decode(MappingProxyType(named_values))
        context = self.evaluator.evaluate(parameters)

        raw_objectives: dict[str, float] = {}
        minimization_objectives: dict[str, float] = {}
        for objective in self.objectives:
            raw_value = objective.evaluate(context)
            min_value = minimization_value(raw_value, objective.sense)
            raw_objectives[objective.name] = float(raw_value)
            minimization_objectives[objective.name] = min_value

        equality_diagnostics = tuple(
            equality_result(constraint, context) for constraint in self.equalities
        )
        inequality_diagnostics = tuple(
            inequality_result(constraint, context) for constraint in self.inequalities
        )
        return ProblemEvaluation(
            design_vector=tuple(named_values.values()),
            variable_values=named_values,
            decoded_parameters=parameters,
            domain_evaluation=context,
            raw_objective_values=raw_objectives,
            minimization_objective_values=minimization_objectives,
            box_violations=box_violations,
            equality_results=equality_diagnostics,
            inequality_results=inequality_diagnostics,
        )


def _finite_design_value(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"design variable {name!r} must be finite; got {value!r}")
    return float(value)


def _require_unique_names(names, category: str) -> None:
    seen: set[str] = set()
    for name in names:
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f"{category} name must be a non-empty string")
        if name in seen:
            raise ValueError(f"duplicate {category} name {name!r}")
        seen.add(name)


__all__ = [
    "Evaluator",
    "OptimizationProblem",
    "Parameterization",
    "ProblemEvaluation",
]
