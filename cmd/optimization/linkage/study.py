"""Small immutable study value object; JSON I/O lives with the run script."""

from __future__ import annotations

from dataclasses import dataclass

from cmd.optimization import BoxVariable
from cmd.optimization.solvers.pymoo import NSGA2Config

from .parameterization import LinkageBaselineParameterization
from .stage0 import LinkageStage0Config


@dataclass(frozen=True)
class LinkageStage0Study:
    """Named Stage-0 inputs plus solver settings for one reproducible run."""

    name: str
    variables: tuple[BoxVariable, ...]
    stage0: LinkageStage0Config
    solver: NSGA2Config
    include_baseline_seed: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("study name must be a non-empty string")
        variables = tuple(self.variables)
        if any(not isinstance(variable, BoxVariable) for variable in variables):
            raise TypeError("study variables must contain only BoxVariable instances")
        expected = LinkageBaselineParameterization().variable_names
        names = tuple(variable.name for variable in variables)
        if len(names) != len(set(names)):
            raise ValueError("study variables contain duplicate names")
        if names != expected:
            missing = tuple(name for name in expected if name not in names)
            unexpected = tuple(name for name in names if name not in expected)
            raise ValueError(
                "study variables must exactly match canonical names in order; "
                f"missing={missing!r}, unexpected={unexpected!r}"
            )
        bc = variables[expected.index("bc_length_mm")]
        if bc.lower <= 0.0:
            raise ValueError("bc_length_mm lower bound must be greater than zero")
        if not isinstance(self.stage0, LinkageStage0Config):
            raise TypeError("stage0 must be LinkageStage0Config")
        if not isinstance(self.solver, NSGA2Config):
            raise TypeError("solver must be NSGA2Config")
        if not isinstance(self.include_baseline_seed, bool):
            raise ValueError("include_baseline_seed must be a bool")
        object.__setattr__(self, "variables", variables)


__all__ = ["LinkageStage0Study"]
