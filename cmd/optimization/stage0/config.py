"""Single flat configuration for one Stage-0 NSGA-II study."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

from cmd._validation import require_finite, require_nonnegative, require_positive
from cmd.morphology.linkage import LinkageWorkspaceCriteria

from .design import VARIABLE_NAMES


@dataclass(frozen=True)
class Stage0Config:
    """Search bounds, motion requirements, and solver settings.

    Input flexion workspaces nest as ``core <= required <= horizon``:

    - core: q2/q3 must reach their minimum flexion at its end, and both
      objectives are measured over it.
    - required: every sample through it must close with healthy
      transmission and motion ratios.
    - horizon: the sweep continues to here for usable-workspace diagnostics.
    """

    bounds: Mapping[str, tuple[float, float]]
    core_input_workspace_rad: float
    required_input_workspace_rad: float
    search_horizon_rad: float
    sample_count: int
    criteria: LinkageWorkspaceCriteria
    q2_minimum_endpoint_flexion_rad: float
    q3_minimum_endpoint_flexion_rad: float
    minimum_anchor_dorsal_offset_mm: float
    population_size: int = 100
    generations: int = 200
    seed: int = 0
    include_baseline_seed: bool = True

    def __post_init__(self) -> None:
        if tuple(self.bounds) != VARIABLE_NAMES:
            raise ValueError(f"bounds must list exactly {VARIABLE_NAMES} in order")
        bounds = {}
        for name, (lower, upper) in self.bounds.items():
            lower = require_finite(f"{name} lower bound", lower)
            upper = require_finite(f"{name} upper bound", upper)
            if lower >= upper:
                raise ValueError(f"{name} lower bound must be below its upper bound")
            bounds[name] = (lower, upper)
        if bounds["bc_length_mm"][0] <= 0.0:
            raise ValueError("bc_length_mm lower bound must be greater than zero")
        object.__setattr__(self, "bounds", MappingProxyType(bounds))

        for name in (
            "core_input_workspace_rad",
            "required_input_workspace_rad",
            "search_horizon_rad",
        ):
            object.__setattr__(self, name, require_positive(name, getattr(self, name)))
        if not (
            self.core_input_workspace_rad
            <= self.required_input_workspace_rad
            <= self.search_horizon_rad
        ):
            raise ValueError("input workspaces must satisfy core <= required <= horizon")
        if self.core_input_workspace_rad == self.search_horizon_rad:
            raise ValueError("core_input_workspace_rad must be below search_horizon_rad")
        for name in (
            "q2_minimum_endpoint_flexion_rad",
            "q3_minimum_endpoint_flexion_rad",
            "minimum_anchor_dorsal_offset_mm",
        ):
            object.__setattr__(self, name, require_nonnegative(name, getattr(self, name)))
        if not isinstance(self.criteria, LinkageWorkspaceCriteria):
            raise TypeError("criteria must be LinkageWorkspaceCriteria")
        _require_int("sample_count", self.sample_count, minimum=3)
        _require_int("population_size", self.population_size, minimum=2)
        _require_int("generations", self.generations, minimum=1)
        _require_int("seed", self.seed, minimum=0)

    @property
    def lower(self) -> tuple[float, ...]:
        return tuple(lower for lower, _ in self.bounds.values())

    @property
    def upper(self) -> tuple[float, ...]:
        return tuple(upper for _, upper in self.bounds.values())


def _require_int(name: str, value: object, *, minimum: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{name} must be an integer of at least {minimum}; got {value!r}")


__all__ = ["Stage0Config"]
