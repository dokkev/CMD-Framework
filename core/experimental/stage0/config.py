"""Single flat configuration for one Stage-0 NSGA-II study."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

from core._validation import require_finite, require_nonnegative, require_positive
from core.experimental.linkage import LinkageWorkspaceCriteria

from core.experimental.stage0.design import VARIABLE_NAMES


@dataclass(frozen=True)
class Stage0Config:
    """Search bounds, motion requirements, and solver settings.

    The MCP input sweeps ``[0, search_horizon_rad]`` on ``sample_count``
    uniform samples. No MCP angle is prescribed: each design is evaluated from
    extension to its own motion limit, the end of its contiguous healthy range
    under ``criteria``, which must reach at least ``minimum_motion_range_rad``.

    Every design variable is searched on a ``resolution_mm`` grid; bounds are
    tightened inward to the nearest grid values.
    """

    bounds: Mapping[str, tuple[float, float]]
    search_horizon_rad: float
    sample_count: int
    criteria: LinkageWorkspaceCriteria
    extended_length_bounds_mm: tuple[float, float]
    minimum_pin_spacing_mm: float
    minimum_grasp_gap_mm: float
    minimum_post_grasp_travel_rad: float = 0.0
    minimum_motion_range_rad: float = 0.0
    resolution_mm: float = 0.1
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

        object.__setattr__(
            self, "search_horizon_rad", require_positive("search_horizon_rad", self.search_horizon_rad)
        )
        for name in (
            "minimum_pin_spacing_mm",
            "minimum_grasp_gap_mm",
            "minimum_post_grasp_travel_rad",
            "minimum_motion_range_rad",
        ):
            object.__setattr__(self, name, require_nonnegative(name, getattr(self, name)))
        object.__setattr__(
            self,
            "extended_length_bounds_mm",
            _positive_range("extended_length_bounds_mm", self.extended_length_bounds_mm),
        )
        object.__setattr__(
            self, "resolution_mm", require_positive("resolution_mm", self.resolution_mm)
        )
        for name, (lower, upper) in self.bounds.items():
            if math.floor(upper / self.resolution_mm + 1e-9) < math.ceil(
                lower / self.resolution_mm - 1e-9
            ):
                raise ValueError(f"{name} bounds contain no {self.resolution_mm} mm grid value")
        if not isinstance(self.criteria, LinkageWorkspaceCriteria):
            raise TypeError("criteria must be LinkageWorkspaceCriteria")
        _require_int("sample_count", self.sample_count, minimum=3)
        if self.minimum_motion_range_rad > self.search_horizon_rad:
            raise ValueError("minimum_motion_range_rad must not exceed search_horizon_rad")
        _require_int("population_size", self.population_size, minimum=2)
        _require_int("generations", self.generations, minimum=1)
        _require_int("seed", self.seed, minimum=0)
        if not isinstance(self.include_baseline_seed, bool):
            raise ValueError("include_baseline_seed must be a bool")

    @property
    def lower(self) -> tuple[float, ...]:
        return tuple(lower for lower, _ in self.bounds.values())

    @property
    def upper(self) -> tuple[float, ...]:
        return tuple(upper for _, upper in self.bounds.values())


def _positive_range(name: str, bounds: tuple[float, float]) -> tuple[float, float]:
    lower, upper = bounds
    lower = require_positive(f"{name} lower", lower)
    upper = require_finite(f"{name} upper", upper)
    if lower >= upper:
        raise ValueError(f"{name} lower must be below upper")
    return (lower, upper)


def _require_int(name: str, value: object, *, minimum: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{name} must be an integer of at least {minimum}; got {value!r}")


__all__ = ["Stage0Config"]
