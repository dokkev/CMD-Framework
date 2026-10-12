"""Run Stage-0 NSGA-II with pymoo and save the feasible Pareto candidates."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass, fields
from pathlib import Path

import numpy as np
import pymoo
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.core.problem import ElementwiseProblem
from pymoo.operators.sampling.lhs import LHS
from pymoo.optimize import minimize

from cmd.morphology.linkage import BASELINE_LINKAGE_V0

from .config import Stage0Config
from .design import VARIABLE_NAMES, decode, encode
from .evaluate import Stage0Evaluation, Stage0Evaluator


# Stands in for the infinite ratio variation of designs that fail before the
# core endpoint; such designs are already infeasible through workspace_health.
INCOMPLETE_CORE_OBJECTIVE = 1.0e12


class Stage0Problem(ElementwiseProblem):
    """Minimize (-worst transmission quality, ratio variation) subject to ``G <= 0``."""

    def __init__(self, config: Stage0Config) -> None:
        self.evaluator = Stage0Evaluator(config)
        super().__init__(
            n_var=len(VARIABLE_NAMES),
            n_obj=2,
            n_ieq_constr=self.evaluator.constraint_count,
            xl=np.asarray(config.lower),
            xu=np.asarray(config.upper),
        )

    def _evaluate(self, x, out, *args, **kwargs) -> None:
        evaluation = self.evaluator(decode(x))
        out["F"] = objective_vector(evaluation)
        out["G"] = np.asarray(evaluation.g)


def objective_vector(evaluation: Stage0Evaluation) -> np.ndarray:
    variation = evaluation.ratio_variation
    return np.asarray([
        -evaluation.worst_transmission_quality,
        variation if math.isfinite(variation) else INCOMPLETE_CORE_OBJECTIVE,
    ])


@dataclass(frozen=True)
class Stage0Candidate:
    x: tuple[float, ...]
    evaluation: Stage0Evaluation


@dataclass(frozen=True)
class Stage0Result:
    config: Stage0Config
    pareto: tuple[Stage0Candidate, ...]
    evaluation_count: int


def run_stage0(config: Stage0Config) -> Stage0Result:
    """Run NSGA-II from LHS, optionally seeded with BASELINE_LINKAGE_V0."""

    problem = Stage0Problem(config)
    initial = np.empty((0, problem.n_var))
    if config.include_baseline_seed:
        baseline = np.asarray(encode(BASELINE_LINKAGE_V0))
        outside = [
            name
            for name, value, (lower, upper) in zip(VARIABLE_NAMES, baseline, config.bounds.values())
            if not lower <= value <= upper
        ]
        if outside:
            raise ValueError(f"BASELINE_LINKAGE_V0 is outside the bounds of {outside}")
        initial = baseline.reshape(1, -1)
    remaining = config.population_size - len(initial)
    lhs = LHS().do(problem, remaining, random_state=np.random.default_rng(config.seed)).get("X")

    result = minimize(
        problem,
        NSGA2(pop_size=config.population_size, sampling=np.vstack([initial, lhs])),
        termination=("n_gen", config.generations),
        seed=config.seed,
        verbose=False,
    )
    pareto: list[Stage0Candidate] = []
    if result.opt is not None:
        for x in np.atleast_2d(result.opt.get("X")):
            evaluation = problem.evaluator(decode(x))
            if evaluation.feasible:
                pareto.append(Stage0Candidate(tuple(float(v) for v in x), evaluation))
    pareto.sort(key=lambda candidate: -candidate.evaluation.worst_transmission_quality)
    return Stage0Result(config, tuple(pareto), int(result.algorithm.evaluator.n_eval))


def save_stage0_result(
    result: Stage0Result, directory: str | Path, *, source_git_commit: str | None = None
) -> Path:
    """Write ``run.json`` (config and metadata) and ``pareto.csv`` to ``directory``."""

    output = Path(directory)
    output.mkdir(parents=True, exist_ok=False)
    config = {field.name: getattr(result.config, field.name) for field in fields(result.config)}
    config["bounds"] = dict(result.config.bounds)
    config["criteria"] = asdict(result.config.criteria)
    metadata = {
        "config": config,
        "pymoo_version": pymoo.__version__,
        "source_git_commit": source_git_commit,
        "evaluation_count": result.evaluation_count,
        "pareto_count": len(result.pareto),
    }
    (output / "run.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    with (output / "pareto.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow((
            "candidate", *VARIABLE_NAMES,
            "worst_transmission_quality", "ratio_variation",
            "q2_flexion_at_core_rad", "q3_flexion_at_core_rad",
            "usable_input_workspace_rad", "distal_endpoint_excursion_mm", "limiting_reasons",
        ))
        core = Stage0Evaluator(result.config).core_count - 1
        for index, candidate in enumerate(result.pareto, start=1):
            evaluation = candidate.evaluation
            writer.writerow((
                index, *candidate.x,
                evaluation.worst_transmission_quality, evaluation.ratio_variation,
                evaluation.motion.q2_flexion_rad[core], evaluation.motion.q3_flexion_rad[core],
                evaluation.workspace.usable_input_workspace_rad,
                evaluation.workspace.distal_endpoint_excursion_mm,
                ";".join(evaluation.workspace.limiting_reasons),
            ))
    return output


__all__ = [
    "INCOMPLETE_CORE_OBJECTIVE",
    "Stage0Candidate",
    "Stage0Problem",
    "Stage0Result",
    "objective_vector",
    "run_stage0",
    "save_stage0_result",
]
