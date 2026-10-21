"""The interface a multi-objective optimizer plugs into a study with.

An optimizer gets a :class:`~core.optimization.study.MechanismProblem` (integer
grid steps, objectives ``F`` and constraints ``G <= 0``) and its starting
population, and returns its final non-dominated designs; the study keeps the
feasible ones and re-evaluates them. The settings every optimizer reads
(population, generations, seed) are :class:`OptimizerSettings`; anything
particular to one algorithm goes in ``options``.

To add one (e.g. an MO-CMA-ES): subclass :class:`Optimizer` and list it in
:data:`OPTIMIZERS`.
"""

from __future__ import annotations

import importlib
from abc import ABC, abstractmethod
from collections.abc import Mapping
from dataclasses import dataclass, field, fields, replace
from types import MappingProxyType
from typing import Any

import numpy as np


@dataclass(frozen=True)
class OptimizerSettings:
    algorithm: str = "nsga2"
    """Name of the optimizer (:data:`OPTIMIZERS`)."""
    population_size: int = 200
    generations: int = 200
    seed: int = 0
    options: Mapping[str, Any] = field(default_factory=dict)
    """Settings of this algorithm only, as its optimizer documents them."""

    def __post_init__(self) -> None:
        if self.algorithm not in OPTIMIZERS:
            raise ValueError(f"unknown optimizer {self.algorithm!r}; known: {sorted(OPTIMIZERS)}")
        for name in ("population_size", "generations"):
            if int(getattr(self, name)) < 1:
                raise ValueError(f"{name} must be at least 1")
        object.__setattr__(self, "options", MappingProxyType(dict(self.options)))

    def to_json(self) -> dict[str, Any]:
        return {f.name: getattr(self, f.name) for f in fields(self)} | {"options": dict(self.options)}


SETTING_NAMES = tuple(f.name for f in fields(OptimizerSettings))


def split_settings(values: Mapping[str, Any]) -> tuple[dict[str, Any], OptimizerSettings]:
    """Separate the optimizer settings from a config file's values.

    They are read from its ``optimizer`` table (or the top level); everything
    else is left for the mechanism's config.
    """

    values = dict(values)
    table = dict(values.pop("optimizer", None) or {})
    settings = {name: table.pop(name) for name in SETTING_NAMES if name in table}
    for name in SETTING_NAMES:
        if name in values:
            if name in settings:
                raise ValueError(f"optimizer setting {name!r} given twice")
            settings[name] = values.pop(name)
    if table:  # the rest of the table belongs to the mechanism (e.g. its objectives)
        values["optimizer"] = table
    return values, OptimizerSettings(**settings)


@dataclass(frozen=True)
class OptimizerResult:
    x: np.ndarray
    """Final non-dominated designs, in grid steps (one per row)."""
    evaluation_count: int


class Optimizer(ABC):
    name: str

    @abstractmethod
    def minimize(self, problem, initial: np.ndarray, settings: OptimizerSettings) -> OptimizerResult:
        """Minimize ``problem`` from the ``initial`` population (grid steps)."""


def initial_population(problem, warm_start: np.ndarray, settings: OptimizerSettings) -> np.ndarray:
    """The warm-start designs (at most half the population, so the rest still
    explores), then a Latin hypercube, all on the grid."""

    from pymoo.operators.sampling.lhs import LHS

    seeds = np.clip(np.asarray(warm_start).reshape(-1, problem.n_var), problem.xl, problem.xu)
    seeds = seeds[: settings.population_size // 2]
    lhs = LHS().do(problem, settings.population_size - len(seeds),
                   random_state=np.random.default_rng(settings.seed)).get("X")
    return np.vstack((seeds, np.round(lhs)))


class PymooOptimizer(Optimizer):
    """A pymoo multi-objective algorithm with grid-rounding SBX and PM.

    ``options``: ``crossover_prob`` (0.9), ``crossover_eta`` (15) and
    ``mutation_eta`` (20).
    """

    def __init__(self, name: str, algorithm: str) -> None:
        self.name, self.algorithm = name, algorithm
        """``module:Class`` of the pymoo algorithm."""

    def _algorithm(self, settings: OptimizerSettings, sampling: np.ndarray, **operators):
        module, kind = self.algorithm.split(":")
        kind = getattr(importlib.import_module(module), kind)
        return kind(pop_size=settings.population_size, sampling=sampling, **operators)

    def minimize(self, problem, initial: np.ndarray, settings: OptimizerSettings) -> OptimizerResult:
        from pymoo.operators.crossover.sbx import SBX
        from pymoo.operators.mutation.pm import PM
        from pymoo.operators.repair.rounding import RoundingRepair
        from pymoo.optimize import minimize

        options = dict(settings.options)
        crossover = SBX(prob=options.pop("crossover_prob", 0.9), eta=options.pop("crossover_eta", 15),
                        vtype=float, repair=RoundingRepair())
        mutation = PM(eta=options.pop("mutation_eta", 20), vtype=float, repair=RoundingRepair())
        if options:
            raise ValueError(f"unknown {self.name} options {sorted(options)}")
        result = minimize(
            problem,
            self._algorithm(settings, initial, crossover=crossover, mutation=mutation),
            termination=("n_gen", settings.generations),
            seed=settings.seed,
            verbose=False,
        )
        x = np.empty((0, problem.n_var)) if result.opt is None else np.atleast_2d(result.opt.get("X"))
        return OptimizerResult(x, int(result.algorithm.evaluator.n_eval))


OPTIMIZERS: dict[str, Optimizer] = {
    "nsga2": PymooOptimizer("nsga2", "pymoo.algorithms.moo.nsga2:NSGA2"),
    "smsemoa": PymooOptimizer("smsemoa", "pymoo.algorithms.moo.sms:SMSEMOA"),
    "agemoea": PymooOptimizer("agemoea", "pymoo.algorithms.moo.age:AGEMOEA"),  # needs numba
}
"""Every optimizer, by the name configs use. All are multi-objective."""


def portfolio(settings: OptimizerSettings, algorithms, runs_per_algorithm: int = 1) -> list[OptimizerSettings]:
    """Complementary runs: each optimizer, ``runs_per_algorithm`` rounds, each
    run from its own seed (``settings.seed``, then the next ones). The seed sets
    the starting population, and runs from one population fill the same part
    of the front, whatever the optimizer."""

    if runs_per_algorithm < 1:
        raise ValueError("runs_per_algorithm must be at least 1")
    plan: list[OptimizerSettings] = []
    for _ in range(runs_per_algorithm):
        for algorithm in algorithms:
            plan.append(replace(settings, algorithm=algorithm, seed=settings.seed + len(plan)))
    return plan


def optimizer(name: str) -> Optimizer:
    if name not in OPTIMIZERS:
        raise ValueError(f"unknown optimizer {name!r}; known: {sorted(OPTIMIZERS)}")
    return OPTIMIZERS[name]


__all__ = [
    "OPTIMIZERS",
    "Optimizer",
    "OptimizerResult",
    "OptimizerSettings",
    "PymooOptimizer",
    "initial_population",
    "optimizer",
    "portfolio",
    "split_settings",
]
