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
from pymoo.operators.crossover.sbx import SBX
from pymoo.operators.mutation.pm import PM
from pymoo.operators.repair.rounding import RoundingRepair
from pymoo.operators.sampling.lhs import LHS
from pymoo.optimize import minimize

from cmd.experimental.linkage import BASELINE_LINKAGE_V0

from .config import Stage0Config
from .design import VARIABLE_NAMES, decode, encode
from .evaluate import (
    Stage0Evaluation,
    Stage0Evaluator,
    extended_length_mm,
    pin_spacings_mm,
)


# Stands in for infinite objectives of designs without a usable motion range;
# such designs are already infeasible through usable_motion.
INCOMPLETE_CORE_OBJECTIVE = 1.0e12


class Stage0Problem(ElementwiseProblem):
    """Minimize (-quality, facing error, grasp MCP angle) subject to ``G <= 0``.

    pymoo searches integer grid steps; ``to_mm`` maps them to design values.
    """

    def __init__(self, config: Stage0Config) -> None:
        self.evaluator = Stage0Evaluator(config)
        self.resolution_mm = config.resolution_mm
        super().__init__(
            n_var=len(VARIABLE_NAMES),
            n_obj=3,
            n_ieq_constr=self.evaluator.constraint_count,
            xl=np.ceil(np.asarray(config.lower) / self.resolution_mm - 1e-9),
            xu=np.floor(np.asarray(config.upper) / self.resolution_mm + 1e-9),
        )

    def to_mm(self, steps) -> tuple[float, ...]:
        return tuple(round(float(step) * self.resolution_mm, 9) for step in steps)

    def parameters(self, steps):
        return decode(self.to_mm(steps))

    def to_steps(self, values_mm) -> np.ndarray:
        return np.round(np.asarray(values_mm) / self.resolution_mm)

    def _evaluate(self, x, out, *args, **kwargs) -> None:
        evaluation = self.evaluator(self.parameters(x))
        out["F"] = objective_vector(evaluation)
        out["G"] = np.asarray(evaluation.g)


def objective_vector(evaluation: Stage0Evaluation) -> np.ndarray:
    values = (
        -evaluation.worst_transmission_quality,
        evaluation.facing_error_rad,
        evaluation.grasp_mcp_rad,
    )
    return np.asarray([v if math.isfinite(v) else INCOMPLETE_CORE_OBJECTIVE for v in values])


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
    """Run NSGA-II on the resolution grid from LHS, optionally seeded with V0."""

    problem = Stage0Problem(config)
    initial = np.empty((0, problem.n_var))
    if config.include_baseline_seed:
        baseline = problem.to_steps(encode(BASELINE_LINKAGE_V0))
        outside = [
            name
            for name, step, lower, upper in zip(VARIABLE_NAMES, baseline, problem.xl, problem.xu)
            if not lower <= step <= upper
        ]
        if outside:
            raise ValueError(f"BASELINE_LINKAGE_V0 is outside the bounds of {outside}")
        initial = baseline.reshape(1, -1)
    remaining = config.population_size - len(initial)
    lhs = LHS().do(problem, remaining, random_state=np.random.default_rng(config.seed)).get("X")

    result = minimize(
        problem,
        NSGA2(
            pop_size=config.population_size,
            sampling=np.vstack([initial, np.round(lhs)]),
            crossover=SBX(prob=0.9, eta=15, vtype=float, repair=RoundingRepair()),
            mutation=PM(eta=20, vtype=float, repair=RoundingRepair()),
        ),
        termination=("n_gen", config.generations),
        seed=config.seed,
        verbose=False,
    )
    pareto: list[Stage0Candidate] = []
    if result.opt is not None:
        for steps in np.atleast_2d(result.opt.get("X")):
            x = problem.to_mm(steps)
            evaluation = problem.evaluator(decode(x))
            if evaluation.feasible:
                pareto.append(Stage0Candidate(x, evaluation))
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
            "worst_transmission_quality", "ratio_variation", "facing_error_deg",
            "grasp_mcp_deg", "post_grasp_travel_deg", "safe_mcp_limit_deg", "mcp_at_limit_deg",
            "grasp_pip_deg", "grasp_dip_deg", "grasp_gap_mm",
            "grasp_b_along_oa_mm", "grasp_c_along_oa_mm",
            "extended_length_mm", "pin_spacing_a_ap_mm", "pin_spacing_a_bp_mm",
            "pin_spacing_b_bp_mm", "pin_spacing_b_cp_mm",
            "usable_input_workspace_rad", "distal_endpoint_excursion_mm", "limiting_reasons",
        ))
        for index, candidate in enumerate(result.pareto, start=1):
            evaluation = candidate.evaluation
            motion, grasp = evaluation.motion, evaluation.grasp
            writer.writerow((
                index, *candidate.x,
                evaluation.worst_transmission_quality, evaluation.ratio_variation,
                math.degrees(grasp.facing_error_rad),
                math.degrees(evaluation.grasp_mcp_rad),
                math.degrees(evaluation.post_grasp_travel_rad),
                math.degrees(evaluation.grasp_mcp_rad + evaluation.post_grasp_travel_rad),
                math.degrees(evaluation.motion_range_rad),
                math.degrees(motion.q2_flexion_rad[grasp.index]),
                math.degrees(motion.q3_flexion_rad[grasp.index]),
                grasp.gap_mm,
                grasp.b_along_mm,
                grasp.c_along_mm,
                extended_length_mm(motion.parameters),
                *pin_spacings_mm(motion.parameters),
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
