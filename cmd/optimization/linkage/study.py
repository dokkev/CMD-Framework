"""Human-editable JSON studies for CLEAR Stage-0 linkage optimization."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from numbers import Real
from pathlib import Path
from typing import Any

from cmd.optimization import BoxVariable
from cmd.optimization.solvers.pymoo import NSGA2Config

from .baseline import LinkageBaselineConstraintConfig
from .evaluator import LinkageSweepConfig
from .parameterization import LinkageBaselineParameterization
from .workspace import LinkageWorkspaceCriteria


@dataclass(frozen=True)
class LinkageStage0StudyConfig:
    """Complete immutable inputs for one Stage-0 optimization study."""

    name: str
    variables: tuple[BoxVariable, ...]
    constraint_config: LinkageBaselineConstraintConfig
    sweep_config: LinkageSweepConfig
    solver_config: NSGA2Config
    include_baseline_seed: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("study name must be a non-empty string")
        variables = tuple(self.variables)
        if any(not isinstance(variable, BoxVariable) for variable in variables):
            raise TypeError("study variables must contain only BoxVariable instances")

        names = tuple(variable.name for variable in variables)
        expected = LinkageBaselineParameterization().variable_names
        if len(names) != len(set(names)):
            raise ValueError("study variables contain duplicate names")
        unexpected = tuple(name for name in names if name not in expected)
        missing = tuple(name for name in expected if name not in names)
        if unexpected or missing:
            raise ValueError(
                "study variables must exactly match canonical names; "
                f"missing={missing!r}, unexpected={unexpected!r}"
            )
        if names != expected:
            raise ValueError(
                "study variables must follow LinkageBaselineParameterization order"
            )
        bc_variable = variables[expected.index("bc_length_mm")]
        if bc_variable.lower <= 0.0:
            raise ValueError("bc_length_mm lower bound must be greater than zero")

        if not isinstance(self.constraint_config, LinkageBaselineConstraintConfig):
            raise TypeError("constraint_config must be LinkageBaselineConstraintConfig")
        if not isinstance(self.sweep_config, LinkageSweepConfig):
            raise TypeError("sweep_config must be LinkageSweepConfig")
        if not isinstance(self.solver_config, NSGA2Config):
            raise TypeError("solver_config must be NSGA2Config")
        if not isinstance(self.include_baseline_seed, bool):
            raise ValueError("include_baseline_seed must be a bool")

        if self.constraint_config.core_input_workspace_rad >= self.sweep_config.search_horizon_rad:
            raise ValueError("core_input_workspace_rad must be below search_horizon_rad")
        if (
            self.constraint_config.minimum_required_input_workspace_rad
            > self.sweep_config.search_horizon_rad
        ):
            raise ValueError(
                "minimum_required_input_workspace_rad must not exceed search_horizon_rad"
            )
        object.__setattr__(self, "variables", variables)


def study_to_dict(study: LinkageStage0StudyConfig) -> dict[str, object]:
    """Serialize a complete study using a stable, explicit JSON schema."""

    if not isinstance(study, LinkageStage0StudyConfig):
        raise TypeError("study must be a LinkageStage0StudyConfig")
    criteria = study.constraint_config.workspace_criteria
    solver = study.solver_config
    return {
        "name": study.name,
        "variables": [
            {"name": variable.name, "lower": variable.lower, "upper": variable.upper}
            for variable in study.variables
        ],
        "workspace": {
            "core_input_workspace_rad": study.constraint_config.core_input_workspace_rad,
            "minimum_required_input_workspace_rad": (
                study.constraint_config.minimum_required_input_workspace_rad
            ),
            "search_horizon_rad": study.sweep_config.search_horizon_rad,
            "sample_count": study.sweep_config.sample_count,
        },
        "criteria": {
            "minimum_transmission_margin_rad": criteria.minimum_transmission_margin_rad,
            "q2_minimum_motion_ratio": criteria.q2_minimum_motion_ratio,
            "q3_minimum_motion_ratio": criteria.q3_minimum_motion_ratio,
            "q2_maximum_motion_ratio": criteria.q2_maximum_motion_ratio,
            "q3_maximum_motion_ratio": criteria.q3_maximum_motion_ratio,
        },
        "constraints": {
            "minimum_anchor_dorsal_offset_mm": (
                study.constraint_config.minimum_anchor_dorsal_offset_mm
            ),
            "q2_minimum_endpoint_flexion_rad": (
                study.constraint_config.q2_minimum_endpoint_flexion_rad
            ),
            "q3_minimum_endpoint_flexion_rad": (
                study.constraint_config.q3_minimum_endpoint_flexion_rad
            ),
        },
        "solver": {
            "population_size": solver.population_size,
            "generations": solver.generations,
            "seed": solver.seed,
            "eliminate_duplicates": solver.eliminate_duplicates,
            "nonfinite_objective_penalty": solver.nonfinite_objective_penalty,
        },
        "include_baseline_seed": study.include_baseline_seed,
    }


def study_from_dict(data: object) -> LinkageStage0StudyConfig:
    """Validate and materialize one complete study mapping."""

    root = _mapping(data, "study")
    _require_keys(
        root,
        "study",
        (
            "name",
            "variables",
            "workspace",
            "criteria",
            "constraints",
            "solver",
            "include_baseline_seed",
        ),
    )
    variable_data = root["variables"]
    if not isinstance(variable_data, list):
        raise ValueError("study.variables must be an ordered JSON list")
    variables: list[BoxVariable] = []
    for index, item in enumerate(variable_data):
        item_path = f"study.variables[{index}]"
        entry = _mapping(item, item_path)
        _require_keys(entry, item_path, ("name", "lower", "upper"))
        if not isinstance(entry["name"], str):
            raise ValueError(f"{item_path}.name must be a string")
        variables.append(
            BoxVariable(
                entry["name"],
                _number(entry["lower"], f"{item_path}.lower"),
                _number(entry["upper"], f"{item_path}.upper"),
            )
        )

    workspace = _mapping(root["workspace"], "study.workspace")
    _require_keys(
        workspace,
        "study.workspace",
        (
            "core_input_workspace_rad",
            "minimum_required_input_workspace_rad",
            "search_horizon_rad",
            "sample_count",
        ),
    )
    criteria_data = _mapping(root["criteria"], "study.criteria")
    criteria_keys = (
        "minimum_transmission_margin_rad",
        "q2_minimum_motion_ratio",
        "q3_minimum_motion_ratio",
        "q2_maximum_motion_ratio",
        "q3_maximum_motion_ratio",
    )
    _require_keys(criteria_data, "study.criteria", criteria_keys)
    constraint_data = _mapping(root["constraints"], "study.constraints")
    constraint_keys = (
        "minimum_anchor_dorsal_offset_mm",
        "q2_minimum_endpoint_flexion_rad",
        "q3_minimum_endpoint_flexion_rad",
    )
    _require_keys(constraint_data, "study.constraints", constraint_keys)

    solver_data = _mapping(root["solver"], "study.solver")
    _require_keys(
        solver_data,
        "study.solver",
        (
            "population_size",
            "generations",
            "seed",
            "eliminate_duplicates",
            "nonfinite_objective_penalty",
        ),
    )
    for key in ("population_size", "generations", "seed"):
        if isinstance(solver_data[key], bool) or not isinstance(solver_data[key], int):
            raise ValueError(f"study.solver.{key} must be an integer")
    if not isinstance(solver_data["eliminate_duplicates"], bool):
        raise ValueError("study.solver.eliminate_duplicates must be a bool")

    if not isinstance(root["name"], str):
        raise ValueError("study.name must be a string")
    if not isinstance(root["include_baseline_seed"], bool):
        raise ValueError("study.include_baseline_seed must be a bool")

    workspace_criteria = LinkageWorkspaceCriteria(
        **{
            key: _number(criteria_data[key], f"study.criteria.{key}")
            for key in criteria_keys
        }
    )
    constraint_config = LinkageBaselineConstraintConfig(
        core_input_workspace_rad=_number(
            workspace["core_input_workspace_rad"],
            "study.workspace.core_input_workspace_rad",
        ),
        minimum_required_input_workspace_rad=_number(
            workspace["minimum_required_input_workspace_rad"],
            "study.workspace.minimum_required_input_workspace_rad",
        ),
        minimum_anchor_dorsal_offset_mm=_number(
            constraint_data["minimum_anchor_dorsal_offset_mm"],
            "study.constraints.minimum_anchor_dorsal_offset_mm",
        ),
        q2_minimum_endpoint_flexion_rad=_number(
            constraint_data["q2_minimum_endpoint_flexion_rad"],
            "study.constraints.q2_minimum_endpoint_flexion_rad",
        ),
        q3_minimum_endpoint_flexion_rad=_number(
            constraint_data["q3_minimum_endpoint_flexion_rad"],
            "study.constraints.q3_minimum_endpoint_flexion_rad",
        ),
        workspace_criteria=workspace_criteria,
    )
    sweep_config = LinkageSweepConfig(
        search_horizon_rad=_number(
            workspace["search_horizon_rad"], "study.workspace.search_horizon_rad"
        ),
        sample_count=_integer(workspace["sample_count"], "study.workspace.sample_count"),
    )
    solver_config = NSGA2Config(
        population_size=solver_data["population_size"],
        generations=solver_data["generations"],
        seed=solver_data["seed"],
        eliminate_duplicates=solver_data["eliminate_duplicates"],
        nonfinite_objective_penalty=_number(
            solver_data["nonfinite_objective_penalty"],
            "study.solver.nonfinite_objective_penalty",
        ),
    )
    return LinkageStage0StudyConfig(
        name=root["name"],
        variables=tuple(variables),
        constraint_config=constraint_config,
        sweep_config=sweep_config,
        solver_config=solver_config,
        include_baseline_seed=root["include_baseline_seed"],
    )


def load_linkage_stage0_study(path: str | Path) -> LinkageStage0StudyConfig:
    """Load a complete JSON study; incomplete templates are rejected."""

    with Path(path).open(encoding="utf-8") as stream:
        data = json.load(stream, parse_constant=_reject_json_constant)
    return study_from_dict(data)


def save_linkage_stage0_study(
    study: LinkageStage0StudyConfig,
    path: str | Path,
) -> Path:
    """Write one complete, resolved study as human-readable JSON."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8", newline="") as stream:
        json.dump(study_to_dict(study), stream, indent=2, allow_nan=False)
        stream.write("\n")
    return destination


def study_template_to_dict() -> dict[str, object]:
    """Return an intentionally incomplete template without physical bounds."""

    parameterization = LinkageBaselineParameterization()
    return {
        "name": "linkage_stage0_template",
        "variables": [
            {"name": name, "lower": None, "upper": None}
            for name in parameterization.variable_names
        ],
        "workspace": {
            "core_input_workspace_rad": None,
            "minimum_required_input_workspace_rad": None,
            "search_horizon_rad": None,
            "sample_count": None,
        },
        "criteria": {
            "minimum_transmission_margin_rad": None,
            "q2_minimum_motion_ratio": None,
            "q3_minimum_motion_ratio": None,
            "q2_maximum_motion_ratio": None,
            "q3_maximum_motion_ratio": None,
        },
        "constraints": {
            "minimum_anchor_dorsal_offset_mm": None,
            "q2_minimum_endpoint_flexion_rad": None,
            "q3_minimum_endpoint_flexion_rad": None,
        },
        "solver": {
            "population_size": None,
            "generations": None,
            "seed": None,
            "eliminate_duplicates": True,
            "nonfinite_objective_penalty": 1.0e12,
        },
        "include_baseline_seed": True,
    }


def write_linkage_stage0_study_template(path: str | Path) -> Path:
    """Create a template at a new path, refusing to overwrite existing data."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x", encoding="utf-8", newline="") as stream:
        json.dump(study_template_to_dict(), stream, indent=2, allow_nan=False)
        stream.write("\n")
    return destination


def _mapping(value: object, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a JSON object")
    if any(not isinstance(key, str) for key in value):
        raise ValueError(f"{path} keys must be strings")
    return value


def _require_keys(mapping: dict[str, Any], path: str, expected: tuple[str, ...]) -> None:
    actual = set(mapping)
    expected_set = set(expected)
    missing = tuple(key for key in expected if key not in actual)
    unexpected = tuple(sorted(actual - expected_set))
    if missing or unexpected:
        raise ValueError(
            f"{path} keys mismatch; missing={missing!r}, unexpected={unexpected!r}"
        )


def _number(value: object, path: str) -> float:
    if value is None:
        raise ValueError(f"{path} is null; complete the study template before running")
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{path} must be a JSON number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{path} must be finite")
    return number


def _integer(value: object, path: str) -> int:
    if value is None:
        raise ValueError(f"{path} is null; complete the study template before running")
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{path} must be a JSON integer")
    return value


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"JSON constant {value!r} is not allowed; use finite numbers")


__all__ = [
    "LinkageStage0StudyConfig",
    "load_linkage_stage0_study",
    "save_linkage_stage0_study",
    "study_from_dict",
    "study_template_to_dict",
    "study_to_dict",
    "write_linkage_stage0_study_template",
]
