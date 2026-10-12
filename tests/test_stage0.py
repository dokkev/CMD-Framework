"""Stage-0 design vector, configuration, evaluation, and NSGA-II run."""

from dataclasses import replace
import json
import math

import pytest

from cmd.morphology import BASELINE_LINKAGE_V0, FingerLinkage, LinkageWorkspaceCriteria
from cmd.morphology.linkage import KinematicClosureError, LinkageKinematics
from cmd.optimization.stage0 import (
    VARIABLE_NAMES,
    Stage0Config,
    Stage0Evaluator,
    bounds_around,
    decode,
    encode,
    input_samples,
)
from cmd.optimization.stage0.evaluate import workspace_health_residuals


def permissive_criteria(**overrides) -> LinkageWorkspaceCriteria:
    values = dict(
        minimum_transmission_margin_rad=0.0,
        q2_minimum_motion_ratio=0.0,
        q3_minimum_motion_ratio=0.0,
        q2_maximum_motion_ratio=100.0,
        q3_maximum_motion_ratio=100.0,
    )
    return LinkageWorkspaceCriteria(**(values | overrides))


def make_config(**overrides) -> Stage0Config:
    # TEST FIXTURE ONLY: artificial V0-centred bounds, not physical design limits.
    values = dict(
        bounds=bounds_around(BASELINE_LINKAGE_V0, 1.0),
        core_input_workspace_rad=0.4,
        required_input_workspace_rad=0.4,
        search_horizon_rad=0.8,
        sample_count=5,
        criteria=permissive_criteria(),
        q2_minimum_endpoint_flexion_rad=0.0,
        q3_minimum_endpoint_flexion_rad=0.0,
        minimum_anchor_dorsal_offset_mm=0.0,
        population_size=6,
        generations=2,
        seed=11,
    )
    return Stage0Config(**(values | overrides))


def baseline_x() -> tuple[float, ...]:
    return encode(BASELINE_LINKAGE_V0)


def fail_closure_after(monkeypatch, success_limit: int) -> None:
    original_solve = LinkageKinematics.solve
    counts: dict[int, int] = {}

    def solve(self, q1_rad):
        solved = counts.get(id(self), 0)
        if solved >= success_limit:
            raise KinematicClosureError("synthetic closure failure")
        counts[id(self)] = solved + 1
        return original_solve(self, q1_rad)

    monkeypatch.setattr(LinkageKinematics, "solve", solve)


# Design vector -------------------------------------------------------------


def test_decoded_designs_place_b_and_c_on_the_distal_ray() -> None:
    reference = FingerLinkage(decode(baseline_x())).reference_positions

    assert len(VARIABLE_NAMES) == 12
    assert reference.b.z_mm == 0.0
    assert reference.c.z_mm == 0.0
    assert reference.c.x_mm - reference.b.x_mm == pytest.approx(20.0)


# Sample grid ---------------------------------------------------------------


def test_core_and_required_endpoints_are_exact_samples() -> None:
    assert input_samples(make_config(core_input_workspace_rad=0.35)) == pytest.approx(
        (0.0, 0.2, 0.35, 0.4, 0.6, 0.8)
    )
    config = make_config(core_input_workspace_rad=0.401, required_input_workspace_rad=0.7)
    # 0.401 replaces the nearby 0.4 sample instead of creating a 0.001 rad interval.
    assert input_samples(config) == pytest.approx((0.0, 0.2, 0.401, 0.6, 0.7, 0.8))
    at_horizon = make_config(required_input_workspace_rad=0.8)
    assert input_samples(at_horizon) == pytest.approx((0.0, 0.2, 0.4, 0.6, 0.8))


# Evaluation ----------------------------------------------------------------


def test_baseline_is_feasible_with_fixed_constraint_shape() -> None:
    evaluator = Stage0Evaluator(make_config(required_input_workspace_rad=0.6))
    evaluation = evaluator(decode(baseline_x()))

    assert evaluation.feasible
    assert tuple(evaluation.constraints) == (
        "workspace_health",
        "minimum_flexion_at_core_q2",
        "minimum_flexion_at_core_q3",
        "minimum_anchor_dorsal_offset",
    )
    assert len(evaluation.g) == evaluator.constraint_count == 1 + 2 * 4 + 2 * 3 + 4
    assert 0.0 < evaluation.worst_transmission_quality <= 1.0
    assert math.isfinite(evaluation.ratio_variation)


def test_closure_failure_before_required_endpoint_is_infeasible(monkeypatch) -> None:
    fail_closure_after(monkeypatch, success_limit=2)
    evaluator = Stage0Evaluator(make_config())
    evaluation = evaluator(decode(baseline_x()))

    assert not evaluation.feasible
    assert evaluation.max_violation("workspace_health") == pytest.approx(1 / 3)
    assert len(evaluation.g) == evaluator.constraint_count
    assert all(math.isfinite(value) for value in evaluation.g)
    assert evaluation.worst_transmission_quality == 0.0
    assert evaluation.ratio_variation == math.inf


def test_workspace_health_checks_every_sample_through_required_endpoint() -> None:
    evaluator = Stage0Evaluator(make_config())
    motion = evaluator(decode(baseline_x())).motion
    angles = list(motion.transmission_angle_loop_2_rad)
    angles[3] = 0.1
    motion = replace(motion, transmission_angle_loop_2_rad=tuple(angles))
    criteria = permissive_criteria(minimum_transmission_margin_rad=0.2)

    assert max(workspace_health_residuals(motion, criteria, required_count=3)) <= 0.0
    assert max(workspace_health_residuals(motion, criteria, required_count=4)) == pytest.approx(0.1)


@pytest.mark.parametrize("ratio,violation", [(0.25, 0.25), (2.0, 0.5)])
def test_workspace_health_bounds_motion_ratio_from_both_sides(ratio, violation) -> None:
    motion = Stage0Evaluator(make_config())(decode(baseline_x())).motion
    ratios = [1.0] * len(motion.dq3_flexion_dq1_flexion)
    ratios[1] = ratio
    motion = replace(motion, dq3_flexion_dq1_flexion=tuple(ratios))
    criteria = permissive_criteria(q3_minimum_motion_ratio=0.5, q3_maximum_motion_ratio=1.5)

    assert max(workspace_health_residuals(motion, criteria, required_count=3)) == pytest.approx(violation)


def test_objectives_ignore_motion_after_core(monkeypatch) -> None:
    config = make_config(required_input_workspace_rad=0.8)
    evaluator = Stage0Evaluator(config)
    reference = evaluator(decode(baseline_x()))
    motion = reference.motion
    changed = replace(
        motion,
        dq2_flexion_dq1_flexion=motion.dq2_flexion_dq1_flexion[:2] + (50.0, -50.0),
        transmission_quality_loop_1=motion.transmission_quality_loop_1[:3] + (0.0, 0.0),
    )
    monkeypatch.setattr(type(evaluator._motion), "evaluate", lambda self, parameters: changed)
    evaluation = evaluator(decode(baseline_x()))

    assert evaluation.worst_transmission_quality == reference.worst_transmission_quality
    assert evaluation.ratio_variation == reference.ratio_variation
    assert not evaluation.feasible


def test_endpoint_flexion_and_anchor_offset_constraints() -> None:
    reference = Stage0Evaluator(make_config())(decode(baseline_x()))
    core_q3 = reference.motion.q3_flexion_rad[2]
    evaluation = Stage0Evaluator(
        make_config(q3_minimum_endpoint_flexion_rad=core_q3 + 0.1, minimum_anchor_dorsal_offset_mm=1000.0)
    )(decode(baseline_x()))

    assert evaluation.max_violation("minimum_flexion_at_core_q3") == pytest.approx(0.1)
    assert evaluation.max_violation("minimum_flexion_at_core_q2") == 0.0
    assert evaluation.max_violation("minimum_anchor_dorsal_offset") > 900.0


# NSGA-II run ---------------------------------------------------------------


def test_tiny_run_returns_feasible_pareto_and_saves_it(tmp_path) -> None:
    from cmd.optimization.stage0.run import run_stage0, save_stage0_result

    result = run_stage0(make_config())

    assert result.pareto
    assert all(candidate.evaluation.feasible for candidate in result.pareto)
    output = save_stage0_result(result, tmp_path / "run", source_git_commit="test")
    metadata = json.loads((output / "run.json").read_text(encoding="utf-8"))
    rows = (output / "pareto.csv").read_text(encoding="utf-8").splitlines()
    assert metadata["pareto_count"] == len(result.pareto) == len(rows) - 1
    assert metadata["config"]["bounds"]["bc_length_mm"] == pytest.approx([19.0, 21.0])
    assert metadata["source_git_commit"] == "test"
