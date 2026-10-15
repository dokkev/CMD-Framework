"""Run the 2-DoF finger NSGA-II study and save its feasible Pareto set."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Sequence

import numpy as np
import pymoo
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.core.problem import ElementwiseProblem
from pymoo.operators.crossover.sbx import SBX
from pymoo.operators.mutation.pm import PM
from pymoo.operators.repair.rounding import RoundingRepair
from pymoo.operators.sampling.lhs import LHS
from pymoo.optimize import minimize

from cmd.hardware import DShapedExtrusion
from cmd.morphology.linkage.packaging import Packaging

from .config import FingerStudyConfig
from .design import VARIABLE_NAMES, decode
from .evaluate import CONSTRAINT_NAMES, INVALID, FingerEvaluation, evaluate


class FingerProblem(ElementwiseProblem):
    """Minimize ``(objective 1, -(OA + BC), 180 deg - phi_safe_max)`` subject to ``G <= 0``.

    Objective 1 is the worst torque ratio ``k`` (default), the worst lever ``r``
    (mm) or ``-worst transmission quality``, per ``config.objective``.

    pymoo searches integer grid steps; ``to_mm`` maps them to design values.
    """

    def __init__(self, config: FingerStudyConfig) -> None:
        self.config = config
        super().__init__(
            n_var=len(VARIABLE_NAMES),
            n_obj=3,
            n_ieq_constr=len(CONSTRAINT_NAMES),
            xl=np.ceil(np.asarray(config.lower) / config.resolution_mm - 1e-9),
            xu=np.floor(np.asarray(config.upper) / config.resolution_mm + 1e-9),
        )

    def to_mm(self, steps) -> tuple[float, ...]:
        return tuple(round(float(step) * self.config.resolution_mm, 9) for step in steps)

    def to_steps(self, values_mm) -> np.ndarray:
        return np.round(np.asarray(values_mm, dtype=float) / self.config.resolution_mm)

    def _evaluate(self, x, out, *args, **kwargs) -> None:
        try:
            evaluation = evaluate(decode(self.to_mm(x)), self.config)
        except ValueError:  # B not beyond A: no linkage to evaluate
            out["F"] = np.full(3, INVALID)
            out["G"] = np.full(len(CONSTRAINT_NAMES), INVALID)
            return
        out["F"] = np.asarray(objectives(evaluation, self.config))
        out["G"] = np.asarray(evaluation.g)


def objectives(evaluation: FingerEvaluation, config: FingerStudyConfig) -> tuple[float, float, float]:
    """The three objectives to minimize."""

    return (
        objective_1(evaluation, config),
        -evaluation.structural_length_mm,
        evaluation.curl_shortfall_deg,
    )


def objective_1(evaluation: FingerEvaluation, config: FingerStudyConfig) -> float:
    """The first objective to minimize; unswept designs get a large finite value."""

    value = {
        "transmission": -evaluation.worst_transmission_quality,
        "lever": evaluation.lever_max_mm,
        "torque_ratio": evaluation.torque_ratio_max,
    }[config.objective]
    return value if math.isfinite(value) else INVALID


@dataclass(frozen=True)
class FingerCandidate:
    x: tuple[float, ...]
    evaluation: FingerEvaluation


@dataclass(frozen=True)
class FingerStudyResult:
    config: FingerStudyConfig
    pareto: tuple[FingerCandidate, ...]
    evaluation_count: int


def run_study(
    config: FingerStudyConfig, initial_designs: Sequence[Sequence[float]] = ()
) -> FingerStudyResult:
    """Run NSGA-II on the resolution grid from a Latin hypercube start, plus any
    ``initial_designs`` (design vectors in mm and degrees) to warm-start from.

    A Pareto study keeps the feasible Pareto set; a single-objective study keeps
    the ``keep_candidates`` best distinct feasible designs of the final population.
    """

    problem = FingerProblem(config)
    seeds = np.array([problem.to_steps(x) for x in initial_designs]).reshape(-1, problem.n_var)
    seeds = np.clip(seeds, problem.xl, problem.xu)
    lhs = LHS().do(problem, config.population_size - len(seeds),
                   random_state=np.random.default_rng(config.seed)).get("X")
    result = minimize(
        problem,
        NSGA2(
            pop_size=config.population_size,
            sampling=np.vstack((seeds, np.round(lhs))),
            crossover=SBX(prob=0.9, eta=15, vtype=float, repair=RoundingRepair()),
            mutation=PM(eta=20, vtype=float, repair=RoundingRepair()),
        ),
        termination=("n_gen", config.generations),
        seed=config.seed,
        verbose=False,
    )
    final = result.opt
    pareto: list[FingerCandidate] = []
    seen: set[tuple[float, ...]] = set()
    if final is not None:
        for steps in np.atleast_2d(final.get("X")):
            x = problem.to_mm(steps)
            if x in seen:
                continue
            seen.add(x)
            evaluation = evaluate(decode(x), config)
            if evaluation.feasible:
                pareto.append(FingerCandidate(x, evaluation))
    pareto.sort(key=lambda candidate: objective_1(candidate.evaluation, config))
    return FingerStudyResult(config, tuple(pareto), int(result.algorithm.evaluator.n_eval))


def save_result(
    result: FingerStudyResult, directory: str | Path, *, source_git_commit: str | None = None
) -> Path:
    """Write ``run.json`` (config and metadata) and ``pareto.csv`` to ``directory``."""

    output = Path(directory)
    output.mkdir(parents=True, exist_ok=False)
    config = {f.name: getattr(result.config, f.name) for f in fields(result.config)}
    config["bounds"] = dict(result.config.bounds)
    config["search_bounds"] = dict(result.config.search_bounds)
    config["packaging"] = asdict(result.config.packaging)
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
            "torque_ratio_max", "structural_length_mm", "phi_safe_deg", "usable_pip_deg",
            "stop_reason", "worst_transmission_quality", "min_margin_deg", "lever_max_mm",
            "servo_travel_deg", "ab_mm", "curl_pip_deg", "curl_dip_deg",
            "curl_tip_x_mm", "closes_to_deg", "link_clearance_mm", "servo_lowest_z_mm",
        ))
        for index, candidate in enumerate(result.pareto, start=1):
            e = candidate.evaluation
            writer.writerow((
                index, *candidate.x,
                e.torque_ratio_max, e.structural_length_mm, math.degrees(e.phi_safe_rad),
                math.degrees(e.usable_pip_rad), e.stop_reason, e.worst_transmission_quality,
                math.degrees(e.min_margin_rad), e.lever_max_mm, math.degrees(e.servo_travel_rad),
                e.ab_mm, math.degrees(e.curl_pip_rad), math.degrees(e.curl_dip_rad),
                e.curl_tip_x_mm, math.degrees(e.closes_to_rad), e.link_clearance_mm,
                e.servo_lowest_z_mm,
            ))
    return output


def load_config(run_directory: str | Path) -> FingerStudyConfig:
    """Rebuild the study configuration recorded in a run's ``run.json``."""

    values = json.loads((Path(run_directory) / "run.json").read_text(encoding="utf-8"))["config"]
    values["bounds"] = {name: tuple(bounds) for name, bounds in values["bounds"].items()}
    values["extended_length_bounds_mm"] = tuple(values["extended_length_bounds_mm"])
    if "packaging" in values:
        packaging = values["packaging"]
        values["packaging"] = Packaging(**{
            **packaging,
            "pip_servo": DShapedExtrusion(**packaging["pip_servo"]),
            "mcp_servo": DShapedExtrusion(**packaging["mcp_servo"]),
            "mcp_servo_rear": tuple(packaging["mcp_servo_rear"]),
        })
    known = {f.name for f in fields(FingerStudyConfig)}
    return FingerStudyConfig(**{name: value for name, value in values.items() if name in known})


__all__ = [
    "FingerCandidate",
    "FingerProblem",
    "FingerStudyResult",
    "objectives",
    "load_config",
    "run_study",
    "save_result",
]
