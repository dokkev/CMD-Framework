"""Stage-0 design vector, configuration, evaluation, and NSGA-II run."""

from dataclasses import replace
import json
import math

import pytest

from cmd.morphology import BASELINE_LINKAGE_V0, FingerLinkage, LinkageWorkspaceCriteria, PointXZ
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
from cmd.optimization.stage0.evaluate import (
    extended_length_mm,
    facing_error_rad,
    grasp_violation_mm,
    pad_offsets_from_oa_mm,
)


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
        # V0's pad comes in front of OA' (a grasp pose) from about 1.0 rad MCP.
        search_horizon_rad=1.6,
        sample_count=9,
        criteria=permissive_criteria(),
        extended_length_bounds_mm=(80.0, 110.0),
        minimum_pin_spacing_mm=0.0,
        minimum_grasp_gap_mm=0.0,
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


def test_decoded_designs_align_ap_bp_b_and_c_on_the_x_axis() -> None:
    x = baseline_x()
    reference = FingerLinkage(decode(x)).reference_positions

    assert len(x) == len(VARIABLE_NAMES) == 10
    for point in (reference.ap, reference.bp, reference.b, reference.c):
        assert point.z_mm == pytest.approx(0.0, abs=1e-12)
    assert decode(x) == BASELINE_LINKAGE_V0.parameters
    assert encode(FingerLinkage(decode(x))) == x


# Sample grid ---------------------------------------------------------------


def test_input_samples_are_uniform_up_to_the_horizon() -> None:
    assert input_samples(make_config()) == pytest.approx(tuple(0.2 * i for i in range(9)))


# Evaluation ----------------------------------------------------------------


def with_motion(monkeypatch, evaluator, **changes):
    """Make ``evaluator`` see its V0 motion with ``changes`` applied."""

    motion = replace(evaluator(decode(baseline_x())).motion, **changes)
    monkeypatch.setattr(type(evaluator._motion), "evaluate", lambda self, parameters: motion)
    return motion


def test_baseline_is_feasible_with_fixed_constraint_shape() -> None:
    evaluator = Stage0Evaluator(make_config())
    evaluation = evaluator(decode(baseline_x()))

    assert evaluation.feasible
    assert tuple(evaluation.constraints) == (
        "usable_motion",
        "extended_length",
        "minimum_pin_spacing",
        "grasp_pose",
    )
    assert len(evaluation.g) == evaluator.constraint_count == 7
    # V0's DIP starts to unflex (negative motion ratio) just before 1.6 rad.
    assert evaluation.limit_index == 7
    assert 0.0 < evaluation.worst_transmission_quality <= 1.0
    assert math.isfinite(evaluation.ratio_variation)
    # V0 grasp poses: 32.5, 13.6 and 0.5 deg facing error at 1.0, 1.2 and 1.4 rad.
    assert evaluation.grasp.index == 7
    assert evaluation.facing_error_rad == pytest.approx(math.radians(0.5), abs=math.radians(0.1))
    assert evaluation.grasp_mcp_rad == pytest.approx(1.4)
    motion = evaluation.motion
    assert evaluation.worst_transmission_quality == min(
        *motion.transmission_quality_loop_1[:8], *motion.transmission_quality_loop_2[:8]
    )


def test_facing_error_gap_and_pad_position_of_a_pose() -> None:
    pose = Stage0Evaluator(make_config())(decode(baseline_x())).motion.poses[0]
    # O-A' along +X (30 mm); a 20 mm pad 15 mm on the palmar side, pointing back.
    facing = replace(
        pose,
        o=PointXZ(0.0, 0.0), ap=PointXZ(30.0, 0.0), b=PointXZ(25.0, -15.0), c=PointXZ(5.0, -15.0),
    )
    tilted = replace(facing, c=PointXZ(5.0, -15.0 - 20.0 * math.tan(math.radians(10.0))))
    behind_o = replace(facing, b=PointXZ(5.0, -15.0), c=PointXZ(-15.0, -15.0))

    assert facing_error_rad(facing) == pytest.approx(0.0)
    assert facing_error_rad(tilted) == pytest.approx(math.radians(10.0))
    b, c, length = pad_offsets_from_oa_mm(facing)
    assert (b, c, length) == (pytest.approx((25.0, 15.0)), pytest.approx((5.0, 15.0)), pytest.approx(30.0))
    assert grasp_violation_mm(facing, 15.0) == 0.0
    # Too close: each end 5 mm short of the gap. Slid back past O: C 15 mm behind O.
    assert grasp_violation_mm(facing, 20.0) == pytest.approx(10.0)
    assert grasp_violation_mm(behind_o, 15.0) == pytest.approx(15.0)


def test_no_grasp_pose_is_infeasible_with_a_graded_residual() -> None:
    evaluator = Stage0Evaluator(make_config(minimum_grasp_gap_mm=1000.0))
    evaluation = evaluator(decode(baseline_x()))
    violations = [grasp_violation_mm(pose, 1000.0) for pose in evaluation.motion.poses]

    assert evaluation.grasp is None
    assert evaluation.facing_error_rad == evaluation.grasp_mcp_rad == math.inf
    assert evaluation.max_violation("grasp_pose") == pytest.approx(min(violations))


def test_motion_limit_ends_before_the_first_unhealthy_sample(monkeypatch) -> None:
    evaluator = Stage0Evaluator(
        make_config(criteria=permissive_criteria(minimum_transmission_margin_rad=0.2))
    )
    reference = evaluator(decode(baseline_x()))
    angles = list(reference.motion.transmission_angle_loop_2_rad)
    angles[7] = 0.1
    motion = with_motion(monkeypatch, evaluator, transmission_angle_loop_2_rad=tuple(angles))
    evaluation = evaluator(decode(baseline_x()))

    assert evaluation.limit_index == 6
    assert evaluation.feasible
    # Only samples 5 and 6 remain as grasp poses; 6 faces OA' better.
    assert evaluation.grasp.index == 6
    assert evaluation.facing_error_rad == pytest.approx(facing_error_rad(motion.poses[6]))
    assert evaluation.worst_transmission_quality == min(
        *motion.transmission_quality_loop_1[:7], *motion.transmission_quality_loop_2[:7]
    )


def test_no_usable_motion_is_infeasible_with_finite_residuals(monkeypatch) -> None:
    fail_closure_after(monkeypatch, success_limit=1)
    evaluator = Stage0Evaluator(make_config())
    evaluation = evaluator(decode(baseline_x()))

    assert evaluation.limit_index == 0
    assert not evaluation.feasible
    assert evaluation.max_violation("usable_motion") == pytest.approx(0.2)
    assert len(evaluation.g) == evaluator.constraint_count
    assert all(math.isfinite(value) for value in evaluation.g)
    assert evaluation.worst_transmission_quality == 0.0
    assert evaluation.ratio_variation == evaluation.facing_error_rad == math.inf


def test_minimum_motion_range_requires_healthy_motion_up_to_it(monkeypatch) -> None:
    fail_closure_after(monkeypatch, success_limit=3)
    evaluation = Stage0Evaluator(make_config(minimum_motion_range_rad=0.7))(decode(baseline_x()))

    assert evaluation.limit_index == 2
    assert evaluation.max_violation("usable_motion") == pytest.approx(0.7 - 0.4)


def test_extended_length_constraint() -> None:
    evaluation = Stage0Evaluator(
        make_config(
            extended_length_bounds_mm=(extended_length_mm(BASELINE_LINKAGE_V0.parameters) + 4.5, 200.0),
        )
    )(decode(baseline_x()))

    assert evaluation.max_violation("extended_length") == pytest.approx(4.5)


def test_pin_spacing_constraint_uses_pins_on_the_same_link() -> None:
    # V0: AA' = (-1.5, -20.0), AB' = (7.8, -20.0), BC' = (-11.6, 7.3).
    spacings = (math.hypot(1.5, 20.0), math.hypot(7.8, 20.0), math.hypot(11.6, 7.3))
    evaluation = Stage0Evaluator(make_config(minimum_pin_spacing_mm=14.0))(decode(baseline_x()))

    assert evaluation.constraints["minimum_pin_spacing"] == pytest.approx(
        tuple(14.0 - spacing for spacing in spacings)
    )
    assert evaluation.max_violation("minimum_pin_spacing") == pytest.approx(14.0 - min(spacings))


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
    assert metadata["config"]["bounds"]["bc_length_mm"] == pytest.approx([19.4, 21.4])
    assert all(round(value, 1) == value for candidate in result.pareto for value in candidate.x)
    assert metadata["source_git_commit"] == "test"
