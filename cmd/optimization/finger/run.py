"""Run the 2-DoF finger NSGA-II study and save its feasible Pareto set."""

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
    """Minimize ``(-worst transmission quality, AB)`` subject to ``G <= 0``.

    pymoo searches integer grid steps; ``to_mm`` maps them to design values.
    """

    def __init__(self, config: FingerStudyConfig) -> None:
        self.config = config
        super().__init__(
            n_var=len(VARIABLE_NAMES),
            n_obj=2,
            n_ieq_constr=len(CONSTRAINT_NAMES),
            xl=np.ceil(np.asarray(config.lower) / config.resolution_mm - 1e-9),
            xu=np.floor(np.asarray(config.upper) / config.resolution_mm + 1e-9),
        )

    def to_mm(self, steps) -> tuple[float, ...]:
        return tuple(round(float(step) * self.config.resolution_mm, 9) for step in steps)

    def _evaluate(self, x, out, *args, **kwargs) -> None:
        try:
            evaluation = evaluate(decode(self.to_mm(x), self.config.bc_length_mm), self.config)
        except ValueError:  # B not beyond A: no linkage to evaluate
            out["F"] = np.array((0.0, INVALID))
            out["G"] = np.full(len(CONSTRAINT_NAMES), INVALID)
            return
        out["F"] = np.array((-evaluation.worst_transmission_quality, evaluation.ab_mm))
        out["G"] = np.asarray(evaluation.g)


@dataclass(frozen=True)
class FingerCandidate:
    x: tuple[float, ...]
    evaluation: FingerEvaluation


@dataclass(frozen=True)
class FingerStudyResult:
    config: FingerStudyConfig
    pareto: tuple[FingerCandidate, ...]
    evaluation_count: int


def run_study(config: FingerStudyConfig) -> FingerStudyResult:
    """Run NSGA-II on the resolution grid from a Latin hypercube start."""

    problem = FingerProblem(config)
    lhs = LHS().do(problem, config.population_size,
                   random_state=np.random.default_rng(config.seed)).get("X")
    result = minimize(
        problem,
        NSGA2(
            pop_size=config.population_size,
            sampling=np.round(lhs),
            crossover=SBX(prob=0.9, eta=15, vtype=float, repair=RoundingRepair()),
            mutation=PM(eta=20, vtype=float, repair=RoundingRepair()),
        ),
        termination=("n_gen", config.generations),
        seed=config.seed,
        verbose=False,
    )
    pareto: list[FingerCandidate] = []
    if result.opt is not None:
        for steps in np.atleast_2d(result.opt.get("X")):
            x = problem.to_mm(steps)
            evaluation = evaluate(decode(x, config.bc_length_mm), config)
            if evaluation.feasible:
                pareto.append(FingerCandidate(x, evaluation))
    pareto.sort(key=lambda candidate: -candidate.evaluation.worst_transmission_quality)
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
            "candidate", *VARIABLE_NAMES, "worst_transmission_quality", "ab_mm",
            "curl_pip_deg", "curl_dip_deg", "curl_gap_mm", "min_margin_deg", "closes_to_deg",
            "link_clearance_mm", "servo_lowest_z_mm",
        ))
        for index, candidate in enumerate(result.pareto, start=1):
            e = candidate.evaluation
            writer.writerow((
                index, *candidate.x, e.worst_transmission_quality, e.ab_mm,
                math.degrees(e.curl_pip_rad), math.degrees(e.curl_dip_rad), e.curl_gap_mm,
                math.degrees(e.min_margin_rad), math.degrees(e.closes_to_rad),
                e.link_clearance_mm, e.servo_lowest_z_mm,
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
    "load_config",
    "run_study",
    "save_result",
]
