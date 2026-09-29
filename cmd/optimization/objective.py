"""Solver-independent objective definitions and sense conversion."""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real
from typing import Callable, Generic, Literal, Protocol, TypeVar


ContextT = TypeVar("ContextT", contravariant=True)
ObjectiveSense = Literal["min", "max"]


class Objective(Protocol[ContextT]):
    """An objective that preserves its domain-valued raw result."""

    name: str
    sense: ObjectiveSense

    def evaluate(self, context: ContextT) -> float: ...


@dataclass(frozen=True)
class FunctionalObjective(Generic[ContextT]):
    """A small callable-backed objective for composing application studies."""

    name: str
    sense: ObjectiveSense
    function: Callable[[ContextT], float]

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("objective name must be a non-empty string")
        if self.sense not in ("min", "max"):
            raise ValueError("objective sense must be 'min' or 'max'")
        if not callable(self.function):
            raise TypeError("objective function must be callable")

    def evaluate(self, context: ContextT) -> float:
        return self.function(context)


def minimization_value(raw_value: float, sense: ObjectiveSense) -> float:
    """Convert an objective to minimization form, rejecting NaN but preserving infinity."""

    if sense not in ("min", "max"):
        raise ValueError("objective sense must be 'min' or 'max'")
    if isinstance(raw_value, bool) or not isinstance(raw_value, Real):
        raise TypeError("objective result must be a real scalar")
    value = float(raw_value)
    if math.isnan(value):
        raise ValueError("objective result must not be NaN")
    return value if sense == "min" else -value


__all__ = ["FunctionalObjective", "Objective", "ObjectiveSense", "minimization_value"]
