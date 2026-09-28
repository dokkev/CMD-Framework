"""pymoo adapter and NSGA-II runner for solver-independent CMD problems."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Integral, Real
from typing import Any

import numpy as np
import pymoo
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.core.problem import ElementwiseProblem
from pymoo.operators.sampling.lhs import LHS
from pymoo.optimize import minimize

from cmd.optimization import (
    ConstraintResidualNaNError,
    OptimizationProblem,
    ProblemEvaluation,
)


_DEFAULT_NONFINITE_OBJECTIVE_PENALTY = 1.0e12


class PymooIntegrationError(RuntimeError):
    """Raised when a CMD evaluation violates pymoo's fixed finite shapes."""


@dataclass(frozen=True)
class NSGA2Config:
    """Solver-only settings for one NSGA-II run."""

    population_size: int
    generations: int
    seed: int
    eliminate_duplicates: bool = True
    nonfinite_objective_penalty: float = _DEFAULT_NONFINITE_OBJECTIVE_PENALTY

    def __post_init__(self) -> None:
        for name in ("population_size", "generations", "seed"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Integral):
                raise ValueError(f"{name} must be an integer")
        if self.population_size < 2:
            raise ValueError("population_size must be at least 2")
        if self.generations < 1:
            raise ValueError("generations must be at least 1")
        if self.seed < 0:
            raise ValueError("seed must be non-negative")
        if not isinstance(self.eliminate_duplicates, bool):
            raise ValueError("eliminate_duplicates must be a bool")
        penalty = self.nonfinite_objective_penalty
        if (
            isinstance(penalty, bool)
            or not isinstance(penalty, Real)
            or not math.isfinite(penalty)
            or penalty <= 0.0
        ):
            raise ValueError("nonfinite_objective_penalty must be finite and positive")
        object.__setattr__(self, "population_size", int(self.population_size))
        object.__setattr__(self, "generations", int(self.generations))
        object.__setattr__(self, "seed", int(self.seed))
        object.__setattr__(self, "nonfinite_objective_penalty", float(penalty))


class PymooProblemAdapter(ElementwiseProblem):
    """Map one CMD ``OptimizationProblem`` into pymoo's elementwise contract."""

    def __init__(
        self,
        problem: OptimizationProblem,
        prototype_x: Sequence[float],
        *,
        nonfinite_objective_penalty: float = _DEFAULT_NONFINITE_OBJECTIVE_PENALTY,
    ) -> None:
        if not isinstance(problem, OptimizationProblem):
            raise TypeError("problem must be an OptimizationProblem")
        if not problem.objectives:
            raise ValueError("pymoo requires at least one objective")
        if (
            isinstance(nonfinite_objective_penalty, bool)
            or not isinstance(nonfinite_objective_penalty, Real)
            or not math.isfinite(nonfinite_objective_penalty)
            or nonfinite_objective_penalty <= 0.0
        ):
            raise ValueError("nonfinite_objective_penalty must be finite and positive")

        self.cmd_problem = problem
        self.objective_names = tuple(objective.name for objective in problem.objectives)
        self.nonfinite_objective_penalty = float(nonfinite_objective_penalty)
        self._last_evaluation: ProblemEvaluation | None = None

        prototype = _finite_design_vector(prototype_x, len(problem.variables), "prototype_x")
        prototype_evaluation = _evaluate_cmd_problem(problem, prototype)
        self._expected_inequality_shapes = tuple(
            (result.name, len(result.residuals))
            for result in prototype_evaluation.inequality_results
        )
        self._expected_equality_shapes = tuple(
            (result.name, len(result.residuals))
            for result in prototype_evaluation.equality_results
        )
        n_ieq_constr = sum(size for _, size in self._expected_inequality_shapes)
        n_ieq_constr += sum(size for _, size in self._expected_equality_shapes)

        super().__init__(
            n_var=len(problem.variables),
            n_obj=len(self.objective_names),
            n_ieq_constr=n_ieq_constr,
            xl=np.asarray([variable.lower for variable in problem.variables]),
            xu=np.asarray([variable.upper for variable in problem.variables]),
        )
        self._last_evaluation = prototype_evaluation
        self._format_solver_values(prototype_evaluation, prototype)

    @property
    def last_evaluation(self) -> ProblemEvaluation | None:
        """Most recent raw CMD evaluation, without retaining evaluation history."""

        return self._last_evaluation

    def _evaluate(self, x, out: dict[str, Any], *args, **kwargs) -> None:
        design_vector = _finite_design_vector(x, self.n_var, "design_vector")
        evaluation = _evaluate_cmd_problem(self.cmd_problem, design_vector)
        self._last_evaluation = evaluation
        objective_values, constraint_values = self._format_solver_values(
            evaluation,
            design_vector,
        )
        out["F"] = objective_values
        out["G"] = constraint_values

    def _format_solver_values(
        self,
        evaluation: ProblemEvaluation,
        design_vector: tuple[float, ...],
    ) -> tuple[np.ndarray, np.ndarray]:
        objective_mapping = evaluation.minimization_objective_values
        if tuple(objective_mapping) != self.objective_names:
            raise PymooIntegrationError(
                "objective names/order changed; expected "
                f"{self.objective_names!r}, got {tuple(objective_mapping)!r}"
            )
        objective_values = np.asarray(
            [
                value if math.isfinite(value) else self.nonfinite_objective_penalty
                for value in (objective_mapping[name] for name in self.objective_names)
            ],
            dtype=float,
        )

        self._check_constraint_shapes(
            "inequality",
            self._expected_inequality_shapes,
            evaluation.inequality_results,
            design_vector,
        )
        self._check_constraint_shapes(
            "equality",
            self._expected_equality_shapes,
            evaluation.equality_results,
            design_vector,
        )

        constraint_values: list[float] = []
        for result in evaluation.inequality_results:
            constraint_values.extend(
                self._finite_constraint_value(
                    result.name,
                    residual_index,
                    residual,
                    design_vector,
                )
                for residual_index, residual in enumerate(result.residuals)
            )
        for result in evaluation.equality_results:
            if result.tolerance is None:
                raise PymooIntegrationError(
                    f"equality constraint {result.name!r} has no tolerance"
                )
            constraint_values.extend(
                self._finite_constraint_value(
                    result.name,
                    residual_index,
                    abs(residual) - result.tolerance,
                    design_vector,
                )
                for residual_index, residual in enumerate(result.residuals)
            )
        return objective_values, np.asarray(constraint_values, dtype=float)

    @staticmethod
    def _check_constraint_shapes(
        kind: str,
        expected: tuple[tuple[str, int], ...],
        results,
        design_vector: tuple[float, ...],
    ) -> None:
        if len(results) != len(expected):
            raise PymooIntegrationError(
                f"{kind} constraint count changed from {len(expected)} to "
                f"{len(results)} for design vector {design_vector!r}"
            )
        for (expected_name, expected_size), result in zip(expected, results):
            if result.name != expected_name:
                raise PymooIntegrationError(
                    f"{kind} constraint order/name changed: expected "
                    f"{expected_name!r}, got {result.name!r} for design vector "
                    f"{design_vector!r}"
                )
            actual_size = len(result.residuals)
            if actual_size != expected_size:
                raise PymooIntegrationError(
                    f"{kind} constraint {result.name!r} residual length changed "
                    f"from {expected_size} to {actual_size} for design vector "
                    f"{design_vector!r}"
                )

    @staticmethod
    def _finite_constraint_value(
        name: str,
        residual_index: int,
        value: float,
        design_vector: tuple[float, ...],
    ) -> float:
        if not math.isfinite(value):
            raise PymooIntegrationError(
                f"non-finite constraint residual: constraint={name!r}, "
                f"residual_index={residual_index}, value={value!r}, "
                f"design_vector={design_vector!r}"
            )
        return float(value)


@dataclass(frozen=True)
class NSGA2RunResult:
    """Pymoo run metadata and authoritative CMD evaluations of final designs."""

    config: NSGA2Config
    pymoo_version: str
    termination: str
    generation_count: int
    evaluation_count: int
    pareto_candidates: tuple[ProblemEvaluation, ...]
    final_population: tuple[ProblemEvaluation, ...]


def run_nsga2(
    problem: OptimizationProblem,
    prototype_x: Sequence[float],
    config: NSGA2Config,
    initial_designs: Sequence[Sequence[float]] = (),
) -> NSGA2RunResult:
    """Run NSGA-II with LHS completion and re-evaluate returned designs in CMD."""

    if not isinstance(config, NSGA2Config):
        raise TypeError("config must be an NSGA2Config")
    adapter = PymooProblemAdapter(
        problem,
        prototype_x,
        nonfinite_objective_penalty=config.nonfinite_objective_penalty,
    )
    initial = tuple(
        _validate_initial_design(problem, design, index)
        for index, design in enumerate(initial_designs)
    )
    if len(initial) > config.population_size:
        raise ValueError(
            f"received {len(initial)} initial designs for population_size="
            f"{config.population_size}"
        )

    remaining = config.population_size - len(initial)
    lhs_designs = (
        LHS()
        .do(
            adapter,
            remaining,
            random_state=np.random.default_rng(config.seed),
        )
        .get("X")
        if remaining
        else np.empty((0, adapter.n_var), dtype=float)
    )
    initial_population = np.vstack(
        [np.asarray(initial, dtype=float).reshape((-1, adapter.n_var)), lhs_designs]
    )

    algorithm = NSGA2(
        pop_size=config.population_size,
        sampling=initial_population,
        eliminate_duplicates=config.eliminate_duplicates,
    )
    pymoo_result = minimize(
        adapter,
        algorithm,
        termination=("n_gen", config.generations),
        seed=config.seed,
        verbose=False,
    )

    if pymoo_result.pop is None:
        raise PymooIntegrationError("pymoo completed without a final population")
    final_designs = _design_matrix(pymoo_result.pop.get("X"), adapter.n_var)
    final_population = tuple(problem.evaluate(x) for x in final_designs)

    pareto_designs: tuple[tuple[float, ...], ...] = ()
    if pymoo_result.opt is not None:
        optimum = pymoo_result.opt
        optimum_x = _design_matrix(optimum.get("X"), adapter.n_var)
        feasible = np.asarray(optimum.get("FEAS"), dtype=bool).reshape(-1)
        pareto_designs = tuple(
            design for design, is_feasible in zip(optimum_x, feasible) if is_feasible
        )
    pareto_candidates = tuple(problem.evaluate(x) for x in pareto_designs)

    return NSGA2RunResult(
        config=config,
        pymoo_version=pymoo.__version__,
        termination=type(pymoo_result.algorithm.termination).__name__,
        generation_count=int(pymoo_result.algorithm.n_gen),
        evaluation_count=int(pymoo_result.algorithm.evaluator.n_eval),
        pareto_candidates=pareto_candidates,
        final_population=final_population,
    )


def _validate_initial_design(
    problem: OptimizationProblem,
    design: Sequence[float],
    index: int,
) -> tuple[float, ...]:
    vector = _finite_design_vector(
        design,
        len(problem.variables),
        f"initial_designs[{index}]",
    )
    for variable, value in zip(problem.variables, vector):
        violation = variable.violation(value)
        if violation > 0.0:
            raise ValueError(
                f"initial_designs[{index}] variable {variable.name!r}={value} is "
                f"outside [{variable.lower}, {variable.upper}]"
            )
    return vector


def _evaluate_cmd_problem(
    problem: OptimizationProblem,
    design_vector: tuple[float, ...],
) -> ProblemEvaluation:
    try:
        return problem.evaluate(design_vector)
    except ConstraintResidualNaNError as exc:
        raise PymooIntegrationError(
            "non-finite constraint residual: "
            f"constraint={exc.constraint_name!r}, "
            f"residual_index={exc.residual_index}, value=nan, "
            f"design_vector={design_vector!r}"
        ) from exc


def _finite_design_vector(
    design: Sequence[float],
    expected_size: int,
    name: str,
) -> tuple[float, ...]:
    values = tuple(design)
    if len(values) != expected_size:
        raise ValueError(f"{name} has {len(values)} values; expected {expected_size}")
    result: list[float] = []
    for index, value in enumerate(values):
        if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
            raise ValueError(f"{name}[{index}] must be finite; got {value!r}")
        result.append(float(value))
    return tuple(result)


def _design_matrix(values, n_var: int) -> tuple[tuple[float, ...], ...]:
    matrix = np.asarray(values, dtype=float)
    if matrix.size == 0:
        return ()
    if matrix.ndim == 1:
        matrix = matrix.reshape((1, -1))
    if matrix.ndim != 2 or matrix.shape[1] != n_var:
        raise PymooIntegrationError(
            f"pymoo returned design matrix shape {matrix.shape}; expected (*, {n_var})"
        )
    return tuple(tuple(float(value) for value in row) for row in matrix)


__all__ = [
    "NSGA2Config",
    "NSGA2RunResult",
    "PymooIntegrationError",
    "PymooProblemAdapter",
    "run_nsga2",
]
