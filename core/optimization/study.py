"""Run any multi-objective :class:`~core.optimization.optimizer.Optimizer` on any
:class:`~core.optimization.mechanism.Mechanism`, and save and reload its runs."""

from __future__ import annotations

import csv
import dataclasses
import json
import multiprocessing
import pickle
from collections.abc import Mapping
from types import MappingProxyType
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import pymoo
from pymoo.core.problem import ElementwiseProblem

from core.optimization.mechanism import (
    INVALID,
    Evaluation,
    Mechanism,
    StudyConfig,
    mechanism,
    read_config_file,
)
from core.optimization.optimizer import OptimizerSettings, initial_population, optimizer


LEGACY_MECHANISM = "linkage"
"""The mechanism of runs that predate recording it."""


class MechanismProblem(ElementwiseProblem):
    """Minimize a mechanism's active objectives subject to ``G <= 0``.

    pymoo searches integer grid steps of the mechanism's search coordinates
    (:meth:`~core.optimization.mechanism.Mechanism.search_names`); ``to_mm``
    maps them to a design vector.
    """

    def __init__(self, mechanism: Mechanism, config: StudyConfig, **kwargs) -> None:
        self.mechanism, self.config = mechanism, config
        self.objective_names = mechanism.active_objectives(config)
        lower, upper = mechanism.search_box(config)
        self.steps = np.asarray(mechanism.search_steps(config), dtype=float)
        super().__init__(
            **kwargs,
            n_var=len(mechanism.search_names(config)),
            n_obj=len(self.objective_names),
            n_ieq_constr=len(mechanism.constraint_names),
            xl=np.ceil(np.asarray(lower) / self.steps - 1e-9),
            xu=np.floor(np.asarray(upper) / self.steps + 1e-9),
        )

    def to_mm(self, steps) -> tuple[float, ...] | None:
        """The design vector of grid steps (``None``: no design)."""

        coordinates = tuple(round(float(step) * size, 9) for step, size in zip(steps, self.steps))
        return self.mechanism.to_design(coordinates, self.config)

    def to_steps(self, values_mm) -> np.ndarray:
        """Grid steps of a design vector."""

        return np.round(np.asarray(self.mechanism.from_design(values_mm, self.config), dtype=float) / self.steps)

    def _evaluate(self, x, out, *args, **kwargs) -> None:
        design = self.to_mm(x)
        evaluation = None if design is None else self.mechanism.evaluate(design, self.config)
        if evaluation is None:
            out["F"] = np.full(self.n_obj, INVALID)
            out["G"] = np.full(self.n_ieq_constr, INVALID)
            return
        out["F"] = np.asarray(self.mechanism.objective_values(evaluation, self.objective_names))
        out["G"] = np.asarray(evaluation.g)


@dataclass(frozen=True)
class Candidate:
    x: tuple[float, ...]
    evaluation: Evaluation


@dataclass(frozen=True)
class StudyResult:
    mechanism: Mechanism
    config: StudyConfig
    settings: OptimizerSettings
    pareto: tuple[Candidate, ...]
    evaluation_count: int
    feasible_evaluation_count: int | None = None
    """Evaluations that met every constraint (on the optimizer's grid)."""


_WORKER_PROBLEM: MechanismProblem | None = None
"""The problem a worker process evaluates (set when its pool starts)."""


def _start_worker(problem: MechanismProblem) -> None:
    global _WORKER_PROBLEM
    _WORKER_PROBLEM = problem


def _evaluate_design_in_worker(x):
    """A design's full evaluation, in a form that crosses processes (``None``: infeasible)."""

    evaluation = _WORKER_PROBLEM.mechanism.evaluate(x, _WORKER_PROBLEM.config)
    if evaluation is None or not evaluation.feasible:
        return None
    if dataclasses.is_dataclass(evaluation) and isinstance(getattr(evaluation, "constraints", None), Mapping):
        evaluation = dataclasses.replace(evaluation, constraints=dict(evaluation.constraints))
    return evaluation


def _restored(evaluation):
    if dataclasses.is_dataclass(evaluation) and isinstance(getattr(evaluation, "constraints", None), dict):
        return dataclasses.replace(evaluation, constraints=MappingProxyType(evaluation.constraints))
    return evaluation


def _evaluate_in_worker(x) -> dict:
    out: dict = {}
    _WORKER_PROBLEM._evaluate(x, out)
    return out


ARCHIVE_PRUNE = 4096
"""The archive is cut back to its non-dominated designs past this many."""


def non_dominated(objectives: np.ndarray) -> np.ndarray:
    """Indices of the rows of ``objectives`` (minimized) no other row dominates."""

    from pymoo.util.nds.non_dominated_sorting import NonDominatedSorting

    if not len(objectives):
        return np.zeros(0, dtype=int)
    return np.asarray(NonDominatedSorting().do(objectives, only_non_dominated_front=True), dtype=int)


class _CountingRunner:
    """pymoo's elementwise runner, in this process or over a process pool (each
    worker holds its own problem, so only design vectors and results cross
    processes), counting the evaluations that meet every constraint and
    archiving the feasible ones no other feasible one dominates: designs a
    later generation loses stay in the run's front."""

    def __init__(self, pool=None, workers: int = 1) -> None:
        self.pool, self.workers = pool, workers
        self.evaluated = self.feasible = 0
        self.archive_x: list[np.ndarray] = []
        self.archive_f: list[np.ndarray] = []

    def __call__(self, f, X):
        if self.pool is None:
            outputs = [f(x) for x in X]
        else:
            outputs = self.pool.map(_evaluate_in_worker, list(X),
                                    chunksize=max(1, len(X) // (4 * self.workers)))
        self.evaluated += len(outputs)
        for x, out in zip(X, outputs):
            if np.all(np.asarray(out["G"]) <= 0.0):
                self.feasible += 1
                self.archive_x.append(np.asarray(x, dtype=float))
                self.archive_f.append(np.asarray(out["F"], dtype=float))
        if len(self.archive_x) > ARCHIVE_PRUNE:
            self.prune()
        return outputs

    def prune(self) -> None:
        keep = non_dominated(np.asarray(self.archive_f))
        self.archive_x = [self.archive_x[i] for i in keep]
        self.archive_f = [self.archive_f[i] for i in keep]


def run_study(
    mechanism: Mechanism, config: StudyConfig, settings: OptimizerSettings = OptimizerSettings(),
    initial_designs: Sequence[Sequence[float]] = (), workers: int = 1,
) -> StudyResult:
    """Run the optimizer ``settings`` names on the resolution grid from a Latin
    hypercube start, plus any ``initial_designs`` (design vectors) to warm-start from.

    At most half the population is warm-started, so the rest still explores;
    the study keeps the feasible Pareto set, sorted by the first objective.
    ``workers`` > 1 evaluates each population in that many forked processes;
    the result is the same.
    """

    mechanism.prepare(config)  # before forking: the workers inherit its tables
    if workers <= 1:
        problem = MechanismProblem(mechanism, config, elementwise_runner=_CountingRunner())
        return _run_study(problem, settings, initial_designs)
    context = multiprocessing.get_context("fork")
    with context.Pool(workers, _start_worker, (MechanismProblem(mechanism, config),)) as pool:
        problem = MechanismProblem(mechanism, config, elementwise_runner=_CountingRunner(pool, workers))
        return _run_study(problem, settings, initial_designs)


def _run_study(problem: MechanismProblem, settings: OptimizerSettings,
               initial_designs: Sequence[Sequence[float]]) -> StudyResult:
    mechanism, config = problem.mechanism, problem.config
    warm = np.array([problem.to_steps(x) for x in initial_designs])
    result = optimizer(settings.algorithm).minimize(
        problem, initial_population(problem, warm, settings), settings)
    # The final population's front and every feasible design archived on the way.
    runner = problem.elementwise_runner
    runner.prune()
    archived = np.asarray(runner.archive_x).reshape(-1, problem.n_var)
    designs: list[tuple[float, ...]] = []
    seen: set[tuple[float, ...]] = set()
    for steps in np.vstack((np.atleast_2d(result.x).reshape(-1, problem.n_var), archived)):
        x = problem.to_mm(steps)
        if x is not None and x not in seen:
            seen.add(x)
            designs.append(x)
    evaluations = None
    if runner.pool is not None:  # in the workers, when the evaluations cross processes
        try:
            evaluations = [None if e is None else _restored(e)
                           for e in runner.pool.map(_evaluate_design_in_worker, designs,
                                                    chunksize=max(1, len(designs) // (4 * runner.workers)))]
        except (pickle.PicklingError, TypeError, AttributeError):
            evaluations = None
    if evaluations is None:
        evaluations = [mechanism.evaluate(x, config) for x in designs]
    pareto = [Candidate(x, e) for x, e in zip(designs, evaluations) if e is not None and e.feasible]
    scores = np.array([mechanism.objective_values(c.evaluation, problem.objective_names) for c in pareto])
    pareto = [pareto[i] for i in non_dominated(scores)] if pareto else []
    pareto.sort(key=lambda c: mechanism.objective_values(c.evaluation, problem.objective_names)[0])
    return StudyResult(mechanism, config, settings, tuple(pareto), result.evaluation_count,
                       problem.elementwise_runner.feasible)


def save_result(result: StudyResult, directory: str | Path, *, source_git_commit: str | None = None) -> Path:
    """Write ``run.json`` (mechanism, config and metadata) and ``pareto.csv`` to ``directory``."""

    mechanism = result.mechanism
    output = Path(directory)
    output.mkdir(parents=True, exist_ok=False)
    metadata = {
        "mechanism": mechanism.name,
        "formulation": mechanism.formulation,
        "objectives": list(mechanism.active_objectives(result.config)),
        "config": mechanism.config_to_json(result.config),
        "optimizer": result.settings.to_json(),
        "pymoo_version": pymoo.__version__,
        "source_git_commit": source_git_commit,
        "evaluation_count": result.evaluation_count,
        "feasible_evaluation_count": result.feasible_evaluation_count,
        "pareto_count": len(result.pareto),
    }
    (output / "run.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    rows = [{"candidate": index, **dict(zip(mechanism.variable_names, candidate.x)),
             **mechanism.report(candidate.evaluation)}
            for index, candidate in enumerate(result.pareto, start=1)]
    write_rows(output / "pareto.csv", rows, ("candidate", *mechanism.variable_names))
    return output


def write_rows(path: Path, rows: Sequence[dict[str, Any]], leading: Sequence[str] = ()) -> None:
    """Write dict rows to a CSV, the ``leading`` columns first."""

    fields = list(leading) + [key for key in (rows[0] if rows else {}) if key not in leading]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


@dataclass(frozen=True)
class Run:
    """A saved run, as :func:`load_run` reads it."""

    directory: Path
    mechanism: Mechanism
    formulation: str | None
    objectives: tuple[str, ...]
    metadata: dict[str, Any]

    @property
    def config(self) -> StudyConfig:
        return self.mechanism.config_from_json(json.loads(json.dumps(self.metadata["config"])))

    @property
    def settings(self) -> OptimizerSettings:
        """The optimizer it ran; runs that predate recording it ran NSGA-II with
        the GA settings in their config."""

        if "optimizer" in self.metadata:
            return OptimizerSettings(**self.metadata["optimizer"])
        config = self.metadata["config"]
        return OptimizerSettings(**{name: config[name] for name in ("population_size", "generations", "seed")
                                    if name in config})

    @property
    def current(self) -> bool:
        """Whether it used its mechanism's current formulation."""

        return self.formulation == self.mechanism.formulation

    def candidates(self) -> list[dict[str, str]]:
        with (self.directory / "pareto.csv").open(encoding="utf-8", newline="") as stream:
            return list(csv.DictReader(stream))


def load_run(directory: str | Path) -> Run:
    """Read a run's ``run.json``; runs that predate a field get its legacy value."""

    directory = Path(directory)
    metadata = json.loads((directory / "run.json").read_text(encoding="utf-8"))
    found = mechanism(metadata.get("mechanism", LEGACY_MECHANISM))
    objectives = metadata.get("objectives", found.legacy_objectives)
    return Run(directory, found, metadata.get("formulation", found.legacy_formulation),
               tuple(objectives), metadata)


def load_yaml(mechanism: Mechanism, path: str | Path,
              overrides: Sequence[str] = ()) -> tuple[StudyConfig, OptimizerSettings]:
    """Read a study from a config file (``read_config_file``: its ``base`` and
    ``overrides``): the mechanism's config and the optimizer settings (its
    ``optimizer`` table, or as :meth:`Mechanism.split_settings` reads it)."""

    values, settings = mechanism.split_settings(read_config_file(path, overrides))
    return mechanism.config_from_mapping(values), settings


def load_config(directory: str | Path) -> StudyConfig:
    """The study configuration recorded in a run."""

    return load_run(directory).config


__all__ = [
    "ARCHIVE_PRUNE",
    "Candidate",
    "MechanismProblem",
    "Run",
    "StudyResult",
    "load_config",
    "load_run",
    "load_yaml",
    "non_dominated",
    "run_study",
    "save_result",
    "write_rows",
]
