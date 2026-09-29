import json

import pytest

from cmd.morphology import BASELINE_LINKAGE_V0
from cmd.optimization.linkage import LinkageBaselineParameterization
from scripts.optimize import optimize_linkages


def _complete_test_study() -> dict[str, object]:
    names = LinkageBaselineParameterization().variable_names
    seed = LinkageBaselineParameterization().encode(BASELINE_LINKAGE_V0)
    # TEST FIXTURE ONLY: V0-centered limits exercise the runner, not physical bounds.
    return {
        "name": "runner_smoke_test",
        "variables": [
            {"name": name, "lower": value - 1.0, "upper": value + 1.0}
            for name, value in zip(names, seed)
        ],
        "workspace": {
            "core_input_workspace_rad": 0.4,
            "minimum_required_input_workspace_rad": 0.4,
            "search_horizon_rad": 0.8,
            "sample_count": 5,
        },
        "criteria": {
            "minimum_transmission_margin_rad": 0.0,
            "q2_minimum_motion_ratio": 0.0,
            "q3_minimum_motion_ratio": 0.0,
            "q2_maximum_motion_ratio": 100.0,
            "q3_maximum_motion_ratio": 100.0,
        },
        "constraints": {
            "minimum_anchor_dorsal_offset_mm": 0.0,
            "q2_minimum_endpoint_flexion_rad": 0.0,
            "q3_minimum_endpoint_flexion_rad": 0.0,
        },
        "solver": {
            "population_size": 4,
            "generations": 1,
            "seed": 5,
            "eliminate_duplicates": True,
            "nonfinite_objective_penalty": 1.0e12,
        },
        "include_baseline_seed": True,
    }


def test_no_argument_script_creates_template_and_runs_study(
    tmp_path,
    monkeypatch,
    capsys,
) -> None:
    study_path = tmp_path / "study.json"
    output_root = tmp_path / "runs"
    monkeypatch.setattr(optimize_linkages, "STUDY_PATH", study_path)
    monkeypatch.setattr(optimize_linkages, "OUTPUT_ROOT", output_root)
    monkeypatch.setattr(optimize_linkages, "SOLVER_OVERRIDES", {})

    assert optimize_linkages.main() == 0
    template = json.loads(study_path.read_text(encoding="utf-8"))
    assert len(template["variables"]) == 12
    assert all(item["lower"] is None and item["upper"] is None for item in template["variables"])
    assert not output_root.exists()

    study_path.write_text(json.dumps(_complete_test_study()), encoding="utf-8")
    monkeypatch.setattr(
        optimize_linkages,
        "SOLVER_OVERRIDES",
        {"population_size": 4, "generations": 1, "seed": 9},
    )
    assert optimize_linkages.main() == 0
    output = output_root / "run_001"
    assert "Stage-0 linkage optimization complete" in capsys.readouterr().out
    assert {
        "run.json",
        "study.resolved.json",
        "diagnostics.json",
        "pareto.csv",
        "final_population.csv",
    } == {path.name for path in output.iterdir()}

    metadata = json.loads((output / "run.json").read_text(encoding="utf-8"))
    resolved = json.loads((output / "study.resolved.json").read_text(encoding="utf-8"))
    diagnostics = json.loads((output / "diagnostics.json").read_text(encoding="utf-8"))
    assert metadata["study_name"] == "runner_smoke_test"
    assert metadata["include_baseline_seed"] is True
    assert metadata["initialization"]["remaining_population_method"] == "lhs"
    assert metadata["source_git_commit"]
    assert metadata["seed"] == 9
    assert resolved["solver"]["population_size"] == 4
    assert resolved["solver"]["generations"] == 1
    assert resolved["solver"]["seed"] == 9
    assert json.loads(study_path.read_text(encoding="utf-8"))["solver"]["seed"] == 5
    assert diagnostics["final_population_count"] == 4

    assert optimize_linkages.main() == 0
    assert (output_root / "run_002").is_dir()


def test_run_study_rejects_invalid_solver_override(tmp_path) -> None:
    study_path = tmp_path / "study.json"
    study_path.write_text(json.dumps(_complete_test_study()), encoding="utf-8")

    with pytest.raises(ValueError, match="population_size must be at least 2"):
        optimize_linkages.run_study(
            study_path,
            tmp_path / "runs",
            solver_overrides={"population_size": 1},
        )
