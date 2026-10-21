"""Run any multi-objective :class:`~core.optimization.optimizer.Optimizer` on any
:class:`~core.optimization.mechanism.Mechanism`, and save and reload its runs."""

from __future__ import annotations

import csv
import json
import multiprocessing
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import pymoo
import yaml
from pymoo.core.problem import ElementwiseProblem

from core.optimization.mechanism import INVALID, Evaluation, Mechanism, StudyConfig, mechanism
from core.optimization.optimizer import OptimizerSettings, initial_population, optimizer, split_settings


LEGACY_MECHANISM = "linkage"
"""The mechanism of runs that predate recording it."""


class MechanismProblem(ElementwiseProblem):
    """Minimize a mechanism's active objectives subject to ``G <= 0``.

    pymoo searches integer grid steps; ``to_mm`` maps them to design values.
    """

    def __init__(self, mechanism: Mechanism, config: StudyConfig, **kwargs) -> None:
        self.mechanism, self.config = mechanism, config
        self.objective_names = mechanism.active_objectives(config)
        lower, upper = mechanism.search_box(config)
        super().__init__(
            **kwargs,
            n_var=len(mechanism.variable_names),
            n_obj=len(self.objective_names),
            n_ieq_constr=len(mechanism.constraint_names),
            xl=np.ceil(np.asarray(lower) / config.resolution_mm - 1e-9),
            xu=np.floor(np.asarray(upper) / config.resolution_mm + 1e-9),
        )

    def to_mm(self, steps) -> tuple[float, ...]:
        return tuple(round(float(step) * self.config.resolution_mm, 9) for step in steps)

    def to_steps(self, values_mm) -> np.ndarray:
        return np.round(np.asarray(values_mm, dtype=float) / self.config.resolution_mm)

    def _evaluate(self, x, out, *args, **kwargs) -> None:
        evaluation = self.mechanism.evaluate(self.to_mm(x), self.config)
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


_WORKER_PROBLEM: MechanismProblem | None = None
"""The problem a worker process evaluates (set when its pool starts)."""


def _start_worker(problem: MechanismProblem) -> None:
    global _WORKER_PROBLEM
    _WORKER_PROBLEM = problem


def _evaluate_in_worker(x) -> dict:
    out: dict = {}
    _WORKER_PROBLEM._evaluate(x, out)
    return out


class _PoolRunner:
    """pymoo's elementwise runner over a process pool: each worker holds its own
    problem, so only design vectors and results cross processes."""

    def __init__(self, pool, workers: int) -> None:
        self.pool, self.workers = pool, workers

    def __call__(self, f, X):
        return self.pool.map(_evaluate_in_worker, list(X),
                             chunksize=max(1, len(X) // (4 * self.workers)))


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

    if workers <= 1:
        return _run_study(MechanismProblem(mechanism, config), settings, initial_designs)
    context = multiprocessing.get_context("fork")
    with context.Pool(workers, _start_worker, (MechanismProblem(mechanism, config),)) as pool:
        problem = MechanismProblem(mechanism, config, elementwise_runner=_PoolRunner(pool, workers))
        return _run_study(problem, settings, initial_designs)


def _run_study(problem: MechanismProblem, settings: OptimizerSettings,
               initial_designs: Sequence[Sequence[float]]) -> StudyResult:
    mechanism, config = problem.mechanism, problem.config
    warm = np.array([problem.to_steps(x) for x in initial_designs])
    result = optimizer(settings.algorithm).minimize(
        problem, initial_population(problem, warm, settings), settings)
    pareto: list[Candidate] = []
    seen: set[tuple[float, ...]] = set()
    for steps in result.x:
        x = problem.to_mm(steps)
        if x in seen:
            continue
        seen.add(x)
        evaluation = mechanism.evaluate(x, config)
        if evaluation is not None and evaluation.feasible:
            pareto.append(Candidate(x, evaluation))
    pareto.sort(key=lambda c: mechanism.objective_values(c.evaluation, problem.objective_names)[0])
    return StudyResult(mechanism, config, settings, tuple(pareto), result.evaluation_count)


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


def load_yaml(mechanism: Mechanism, path: str | Path) -> tuple[StudyConfig, OptimizerSettings]:
    """Read a study from a config file: the mechanism's config and the
    optimizer settings (its ``optimizer`` table, see :func:`split_settings`)."""

    with Path(path).open(encoding="utf-8") as stream:
        values, settings = split_settings(yaml.safe_load(stream) or {})
    return mechanism.config_from_mapping(values), settings


def load_config(directory: str | Path) -> StudyConfig:
    """The study configuration recorded in a run."""

    return load_run(directory).config


__all__ = [
    "Candidate",
    "MechanismProblem",
    "Run",
    "StudyResult",
    "load_config",
    "load_run",
    "load_yaml",
    "run_study",
    "save_result",
    "write_rows",
]
