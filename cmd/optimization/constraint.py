"""Solver-independent equality and inequality constraint contracts."""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real
from typing import Callable, Generic, Iterable, Literal, Protocol, TypeVar


ContextT = TypeVar("ContextT", contravariant=True)
ConstraintKind = Literal["equality", "inequality"]


class EqualityConstraint(Protocol[ContextT]):
    """An equality residual vector with convention ``h(x) = 0``."""

    name: str
    tolerance: float

    def residual(self, context: ContextT) -> Iterable[float]: ...


class InequalityConstraint(Protocol[ContextT]):
    """An inequality residual vector with convention ``g(x) <= 0``."""

    name: str

    def residual(self, context: ContextT) -> Iterable[float]: ...


@dataclass(frozen=True)
class FunctionalEqualityConstraint(Generic[ContextT]):
    """A callable-backed equality constraint using ``h(x) = 0``."""

    name: str
    tolerance: float
    function: Callable[[ContextT], Iterable[float]]

    def __post_init__(self) -> None:
        _validate_name(self.name)
        _validate_tolerance(self.tolerance)
        if not callable(self.function):
            raise TypeError("equality constraint function must be callable")

    def residual(self, context: ContextT) -> tuple[float, ...]:
        return tuple(self.function(context))


@dataclass(frozen=True)
class FunctionalInequalityConstraint(Generic[ContextT]):
    """A callable-backed inequality constraint using ``g(x) <= 0``."""

    name: str
    function: Callable[[ContextT], Iterable[float]]

    def __post_init__(self) -> None:
        _validate_name(self.name)
        if not callable(self.function):
            raise TypeError("inequality constraint function must be callable")

    def residual(self, context: ContextT) -> tuple[float, ...]:
        return tuple(self.function(context))


@dataclass(frozen=True)
class ConstraintResult:
    """Named residuals and feasibility diagnostic for one constraint."""

    name: str
    kind: ConstraintKind
    residuals: tuple[float, ...]
    maximum_violation: float
    satisfied: bool
    tolerance: float | None = None


def equality_result(
    constraint: EqualityConstraint[ContextT],
    context: ContextT,
) -> ConstraintResult:
    """Evaluate ``h(x)=0`` and report violation beyond its tolerance."""

    _validate_name(constraint.name)
    tolerance = _validate_tolerance(constraint.tolerance)
    residuals = _residual_values(constraint.residual(context), constraint.name)
    maximum_violation = max(
        (max(abs(value) - tolerance, 0.0) for value in residuals),
        default=0.0,
    )
    return ConstraintResult(
        name=constraint.name,
        kind="equality",
        residuals=residuals,
        maximum_violation=maximum_violation,
        satisfied=maximum_violation == 0.0,
        tolerance=tolerance,
    )


def inequality_result(
    constraint: InequalityConstraint[ContextT],
    context: ContextT,
) -> ConstraintResult:
    """Evaluate ``g(x)<=0`` and report its positive residual violation."""

    _validate_name(constraint.name)
    residuals = _residual_values(constraint.residual(context), constraint.name)
    maximum_violation = max((max(value, 0.0) for value in residuals), default=0.0)
    return ConstraintResult(
        name=constraint.name,
        kind="inequality",
        residuals=residuals,
        maximum_violation=maximum_violation,
        satisfied=maximum_violation == 0.0,
    )


def _residual_values(values: Iterable[float], name: str) -> tuple[float, ...]:
    residuals: list[float] = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, Real):
            raise TypeError(f"constraint {name!r} residuals must be real scalars")
        residual = float(value)
        if math.isnan(residual):
            raise ValueError(f"constraint {name!r} residuals must not contain NaN")
        residuals.append(residual)
    return tuple(residuals)


def _validate_name(name: str) -> None:
    if not isinstance(name, str) or not name.strip():
        raise ValueError("constraint name must be a non-empty string")


def _validate_tolerance(tolerance: float) -> float:
    if (
        isinstance(tolerance, bool)
        or not isinstance(tolerance, Real)
        or not math.isfinite(tolerance)
        or tolerance < 0.0
    ):
        raise ValueError("equality tolerance must be finite and non-negative")
    return float(tolerance)


__all__ = [
    "ConstraintResult",
    "EqualityConstraint",
    "FunctionalEqualityConstraint",
    "FunctionalInequalityConstraint",
    "InequalityConstraint",
    "equality_result",
    "inequality_result",
]
