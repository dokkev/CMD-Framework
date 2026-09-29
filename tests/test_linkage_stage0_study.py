import json

import pytest

from cmd.morphology import BASELINE_LINKAGE_V0
from cmd.optimization import BoxVariable
from cmd.optimization.linkage import (
    LinkageBaselineConstraintConfig,
    LinkageBaselineParameterization,
    LinkageStage0StudyConfig,
    LinkageSweepConfig,
    LinkageWorkspaceCriteria,
    load_linkage_stage0_study,
    save_linkage_stage0_study,
    study_from_dict,
    study_template_to_dict,
    study_to_dict,
)
from cmd.optimization.solvers.pymoo import NSGA2Config


def _test_study() -> LinkageStage0StudyConfig:
    parameterization = LinkageBaselineParameterization()
    seed = parameterization.encode(BASELINE_LINKAGE_V0)
    # TEST FIXTURE ONLY: these artificial V0-centered bounds are not physical limits.
    variables = tuple(
        BoxVariable(name, value - 1.0, value + 1.0)
        for name, value in zip(parameterization.variable_names, seed)
    )
    return LinkageStage0StudyConfig(
        name="test_stage0",
        variables=variables,
        constraint_config=LinkageBaselineConstraintConfig(
            core_input_workspace_rad=0.4,
            minimum_required_input_workspace_rad=0.4,
            minimum_anchor_dorsal_offset_mm=0.0,
            q2_minimum_endpoint_flexion_rad=0.0,
            q3_minimum_endpoint_flexion_rad=0.0,
            workspace_criteria=LinkageWorkspaceCriteria(
                minimum_transmission_margin_rad=0.0,
                q2_minimum_motion_ratio=0.0,
                q3_minimum_motion_ratio=0.0,
                q2_maximum_motion_ratio=100.0,
                q3_maximum_motion_ratio=100.0,
            ),
        ),
        sweep_config=LinkageSweepConfig(search_horizon_rad=0.8, sample_count=5),
        solver_config=NSGA2Config(
            population_size=6,
            generations=2,
            seed=17,
            eliminate_duplicates=False,
            nonfinite_objective_penalty=1234.0,
        ),
        include_baseline_seed=False,
    )


def test_study_json_round_trip_preserves_values_and_variable_order(tmp_path) -> None:
    study = _test_study()
    json_data = json.loads(json.dumps(study_to_dict(study)))

    decoded = study_from_dict(json_data)
    path = save_linkage_stage0_study(decoded, tmp_path / "study.json")
    loaded = load_linkage_stage0_study(path)

    assert loaded == study
    assert tuple(variable.name for variable in loaded.variables) == (
        LinkageBaselineParameterization().variable_names
    )
    assert loaded.include_baseline_seed is False
    assert loaded.solver_config == study.solver_config


@pytest.mark.parametrize("mutation", ["missing", "duplicate", "unexpected", "reordered"])
def test_study_rejects_noncanonical_variable_names_or_order(mutation) -> None:
    data = study_to_dict(_test_study())
    variables = data["variables"]
    if mutation == "missing":
        variables.pop()
    elif mutation == "duplicate":
        variables[1]["name"] = variables[0]["name"]
    elif mutation == "unexpected":
        variables[1]["name"] = "unexpected_variable"
    else:
        variables[0], variables[1] = variables[1], variables[0]

    with pytest.raises(ValueError, match="duplicate|missing|unexpected|order"):
        study_from_dict(data)


def test_template_has_all_names_but_null_bounds_are_not_executable() -> None:
    template = study_template_to_dict()
    expected_names = LinkageBaselineParameterization().variable_names

    assert tuple(item["name"] for item in template["variables"]) == expected_names
    assert all(item["lower"] is None and item["upper"] is None for item in template["variables"])
    with pytest.raises(ValueError, match="null; complete the study template"):
        study_from_dict(template)


def test_study_rejects_nonpositive_bc_length_lower_bound() -> None:
    study = _test_study()
    variables = list(study.variables)
    variables[9] = BoxVariable("bc_length_mm", 0.0, 30.0)

    with pytest.raises(ValueError, match="bc_length_mm lower bound"):
        LinkageStage0StudyConfig(
            name=study.name,
            variables=tuple(variables),
            constraint_config=study.constraint_config,
            sweep_config=study.sweep_config,
            solver_config=study.solver_config,
            include_baseline_seed=study.include_baseline_seed,
        )


def test_study_rejects_inconsistent_core_and_required_workspaces() -> None:
    study = _test_study()
    criteria = study.constraint_config.workspace_criteria
    core_at_horizon = LinkageBaselineConstraintConfig(
        core_input_workspace_rad=0.8,
        minimum_required_input_workspace_rad=0.8,
        minimum_anchor_dorsal_offset_mm=0.0,
        q2_minimum_endpoint_flexion_rad=0.0,
        q3_minimum_endpoint_flexion_rad=0.0,
        workspace_criteria=criteria,
    )
    with pytest.raises(ValueError, match="core_input_workspace_rad.*below"):
        LinkageStage0StudyConfig(
            study.name,
            study.variables,
            core_at_horizon,
            study.sweep_config,
            study.solver_config,
        )

    required_above_horizon = LinkageBaselineConstraintConfig(
        core_input_workspace_rad=0.4,
        minimum_required_input_workspace_rad=0.9,
        minimum_anchor_dorsal_offset_mm=0.0,
        q2_minimum_endpoint_flexion_rad=0.0,
        q3_minimum_endpoint_flexion_rad=0.0,
        workspace_criteria=criteria,
    )
    with pytest.raises(ValueError, match="minimum_required_input_workspace_rad.*not exceed"):
        LinkageStage0StudyConfig(
            study.name,
            study.variables,
            required_above_horizon,
            study.sweep_config,
            study.solver_config,
        )


def test_template_writer_refuses_to_overwrite(tmp_path) -> None:
    from cmd.optimization.linkage import write_linkage_stage0_study_template

    path = tmp_path / "template.json"
    write_linkage_stage0_study_template(path)
    original = path.read_text(encoding="utf-8")

    with pytest.raises(FileExistsError):
        write_linkage_stage0_study_template(path)
    assert path.read_text(encoding="utf-8") == original
