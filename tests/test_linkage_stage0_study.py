import json

import pytest

from cmd.morphology import BASELINE_LINKAGE_V0, LinkageWorkspaceCriteria
from cmd.optimization import BoxVariable
from cmd.optimization.linkage.parameterization import LinkageBaselineParameterization
from cmd.optimization.linkage.stage0 import LinkageStage0Config
from cmd.optimization.linkage.study import LinkageStage0Study
from cmd.optimization.solvers.pymoo import NSGA2Config
from scripts.optimize.optimize_linkages import (
    load_linkage_stage0_study,
    save_linkage_stage0_study,
    study_from_dict,
    study_template_to_dict,
    study_to_dict,
    write_linkage_stage0_study_template,
)


def _test_study() -> LinkageStage0Study:
    parameterization = LinkageBaselineParameterization()
    seed = parameterization.encode(BASELINE_LINKAGE_V0)
    # TEST FIXTURE ONLY: artificial V0-centered bounds; not physical limits.
    variables = tuple(
        BoxVariable(name, value - 1.0, value + 1.0)
        for name, value in zip(parameterization.variable_names, seed)
    )
    stage0 = LinkageStage0Config(
        core_input_workspace_rad=0.4,
        minimum_required_input_workspace_rad=0.4,
        search_horizon_rad=0.8,
        sample_count=5,
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
    )
    return LinkageStage0Study(
        "test_stage0", variables, stage0,
        NSGA2Config(6, 2, 17, False, 1234.0), False,
    )


def test_study_json_round_trip_preserves_values_and_order(tmp_path):
    study = _test_study()
    encoded = json.loads(json.dumps(study_to_dict(study)))
    decoded = study_from_dict(encoded)
    path = save_linkage_stage0_study(decoded, tmp_path / "study.json")
    loaded = load_linkage_stage0_study(path)
    assert loaded == study
    assert tuple(item.name for item in loaded.variables) == LinkageBaselineParameterization().variable_names
    assert loaded.include_baseline_seed is False
    assert loaded.solver == study.solver


@pytest.mark.parametrize("mutation", ["missing", "duplicate", "unexpected", "reordered"])
def test_study_rejects_noncanonical_variable_names_or_order(mutation):
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


def test_template_has_all_names_but_null_bounds_are_not_executable():
    template = study_template_to_dict()
    assert tuple(item["name"] for item in template["variables"]) == LinkageBaselineParameterization().variable_names
    assert all(item["lower"] is None and item["upper"] is None for item in template["variables"])
    with pytest.raises(ValueError, match="null; complete the study template"):
        study_from_dict(template)


def test_study_rejects_nonpositive_bc_length_lower_bound():
    study = _test_study()
    variables = list(study.variables)
    variables[9] = BoxVariable("bc_length_mm", 0.0, 30.0)
    with pytest.raises(ValueError, match="bc_length_mm lower bound"):
        LinkageStage0Study(study.name, tuple(variables), study.stage0, study.solver)


def test_study_rejects_inconsistent_workspace_lengths():
    study = _test_study()
    base = study.stage0
    with pytest.raises(ValueError, match="core_input_workspace_rad.*below"):
        LinkageStage0Config(**{
            **base.__dict__, "core_input_workspace_rad": base.search_horizon_rad,
        })
    with pytest.raises(ValueError, match="minimum_required_input_workspace_rad.*not exceed"):
        LinkageStage0Config(**{
            **base.__dict__, "minimum_required_input_workspace_rad": base.search_horizon_rad + 0.1,
        })


def test_template_writer_refuses_to_overwrite(tmp_path):
    path = tmp_path / "template.json"
    write_linkage_stage0_study_template(path)
    original = path.read_text(encoding="utf-8")
    with pytest.raises(FileExistsError):
        write_linkage_stage0_study_template(path)
    assert path.read_text(encoding="utf-8") == original
