"""The serial 2R on the linkage study's goal."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from core.optimization.mechanism import mechanism
from core.optimization.study import load_yaml

ROOT = Path(__file__).resolve().parents[1] / "scripts" / "optimize"
PINNED = ("goal.link.finger_length_mm=[90, 100]", "goal.pads.proximal.length_mm=[20, null]",
          "goal.pads.distal.length_mm=[25, null]", "goal.motion.dip_flexion_deg=120")
"""The study these tests were written for, whatever the config file now asks."""
MECHANISM = mechanism("revolute")
X = (45.0, 55.0, 30.0, 29.0)
"""A feasible design (pads near their min, |OC| 100); it folds to 137 deg."""


def load(*overrides: str):
    return load_yaml(MECHANISM, ROOT / "revolute" / "config.yaml", (*PINNED, *overrides))[0]


def test_it_reads_the_linkage_goal_as_is():
    config = load()
    linkage = load_yaml(mechanism("linkage"), ROOT / "linkage" / "config.yaml", PINNED)[0]
    assert config.goal == linkage
    assert config.objectives == linkage.objectives and config.seeds == linkage.seeds
    assert MECHANISM.hypervolume_reference(config) == mechanism("linkage").hypervolume_reference(linkage)
    # |OA| and |AC| take the linkage's A_x and BC bounds; the pads start past their min.
    assert config.bounds["ac_length_mm"] == linkage.bounds["bc_length_mm"]
    assert config.bounds["app_x_mm"][0] == pytest.approx(linkage.packaging.proximal_pad_start_mm + 20.0)
    assert "servo_tilt_deg" not in config.bounds


def test_the_pads_and_servo_follow_the_goal():
    from core.optimization.revolute import geometry

    config = load()
    geo = geometry(X, config)
    packaging = config.goal.packaging
    # The proximal pad on the distal servo's palmar face, the distal pad lined
    # up with it below A-C; each laid from its outer end.
    assert geo.proximal_pad[:, 1].max() == pytest.approx(-packaging.pip_servo.radius_mm)
    assert geo.proximal_pad[:, 0].min() == pytest.approx(packaging.proximal_pad_start_mm)
    assert geo.distal_pad_local[:, 0].max() == pytest.approx(55.0)  # at C
    assert geo.distal_pad_local[:, 1].max() == pytest.approx(-packaging.pip_servo.radius_mm)  # aligned
    # Flexed by phi about A, the distal pad keeps its distance to A.
    rest, bent = geo.distal_pad([0.0])[0], geo.distal_pad([math.radians(90.0)])[0]
    a = np.array((45.0, 0.0))
    assert np.linalg.norm(rest - a, axis=1) == pytest.approx(np.linalg.norm(bent - a, axis=1))
    assert geo.tip([math.radians(90.0)])[0] == pytest.approx((45.0, -55.0))
    # The distal servo on A, laid back along -X, untilted.
    assert geo.distal_servo.pivot == (45.0, 0.0)
    assert geo.distal_servo.rear == (-1.0, 0.0)


def test_a_design_is_evaluated_on_the_linkage_objectives():
    config = load()
    e = MECHANISM.evaluate(X, config)
    assert e.feasible
    assert e.torque_ratio_max == 1.0  # direct drive
    assert e.pad_coverage == pytest.approx((e.proximal_face_mm + e.distal_face_mm) / e.finger_length_mm)
    assert e.curl_deg == pytest.approx(180.0 - math.degrees(e.phi_max_rad))
    assert math.degrees(e.phi_safe_rad) >= 120.0
    report = MECHANISM.report(e)
    for column in ("torque_ratio_max", "pad_coverage", "curl_deg", "phi_max_deg", "finger_length_mm",
                   "proximal_effective_mm", "distal_effective_mm", "bound_hits"):
        assert column in report


def test_folding_brings_the_pads_together():
    # Past the required flexion the pads, folding about A, end the safe motion.
    e = MECHANISM.evaluate(X, load())
    assert e.stop_reason == "pad_collision" and math.degrees(e.phi_safe_rad) < 180.0
    # The proximal pad may run under the distal servo, not past its front.
    long = list(X)
    long[2] = 45.0 + 9.5 + 1.0
    assert MECHANISM.evaluate(long, load()).constraints["proximal_support_mm"] == pytest.approx(1.0)


def test_runs_reload(tmp_path):
    from core.optimization.optimizer import OptimizerSettings
    from core.optimization.study import Candidate, StudyResult, load_run, save_result

    config = load()
    run = save_result(StudyResult(MECHANISM, config, OptimizerSettings(),
                                  (Candidate(X, MECHANISM.evaluate(X, config)),), 1), tmp_path / "run")
    loaded = load_run(run)
    assert loaded.mechanism is MECHANISM and loaded.config == config
    assert MECHANISM.refine(loaded.config, math.radians(0.05)).goal.servo_step_rad == pytest.approx(
        math.radians(0.05))


def test_the_search_coordinates_keep_the_lengths():
    from core.optimization.revolute import search
    from core.optimization.study import MechanismProblem

    config = load()
    problem = MechanismProblem(MECHANISM, config)
    rng = np.random.default_rng(0)
    floor = search.least_a_x(config.goal.packaging, tuple(config.goal.mcp_servo_angles_rad))
    for _ in range(200):
        x = problem.to_mm(np.round(problem.xl + rng.random(problem.n_var) * (problem.xu - problem.xl)))
        if x is None:
            continue
        a, ac, app, bpp = x[:4]
        assert 90.0 - 0.1 <= a + ac <= 100.0 + 0.1 and ac >= 25.0 - 0.1 and a >= floor - 0.6
        assert app <= a + config.goal.packaging.pip_servo.radius_mm + 0.1
        assert ac - bpp >= 25.0 - 0.1
    assert problem.to_mm(problem.to_steps(X)) == pytest.approx(X, abs=0.11)
