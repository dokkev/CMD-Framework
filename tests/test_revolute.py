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
          "goal.pads.distal.length_mm=[25, null]", "goal.motion.dip_flexion_deg=120",
          "goal.hardware.pip_servo.tilt_angle_deg=90", "goal.link.proximal_length_ratio=[null, null]",
          "goal.link.distal_length_ratio=[null, null]")
"""The study these tests were written for, whatever the config file now asks."""
MECHANISM = mechanism("revolute")
X = (45.0, 55.0, 30.0, 29.0, 0.0, 0.0, 90.0)
"""A feasible design (pads near their min, |OC| 100, the distal pad level
with the proximal one, both on their segments' lines (the servo's shaft is
the joint), the servo standing dorsal: the pads clear its rounded front,
9.5 mm either side of A)."""


def load(*overrides: str):
    return load_yaml(MECHANISM, ROOT / "linkage" / "config.yaml", (*PINNED, *overrides))[0]


def test_it_reads_the_linkage_goal_as_is():
    config = load()
    linkage = load_yaml(mechanism("linkage"), ROOT / "linkage" / "config.yaml", PINNED)[0]
    assert config.goal == linkage
    assert config.objectives == linkage.objectives and config.seeds == linkage.seeds
    assert MECHANISM.hypervolume_reference(config) == mechanism("linkage").hypervolume_reference(linkage)
    # |OA| and |AC| take the linkage's A_x and BC bounds as they are (the
    # segment ratios bound |OA''| and |B''C|: checked, not narrowed by); the
    # pads start past their min.
    assert config.bounds["ac_length_mm"] == tuple(linkage.bounds["bc_length_mm"])
    assert config.bounds["a_x_mm"] == tuple(linkage.bounds["a_x_mm"])
    assert config.bounds["app_x_mm"][0] == pytest.approx(linkage.packaging.proximal_pad_start_mm + 20.0)
    assert config.bounds["servo_tilt_deg"] == (90.0, 90.0)  # the goal's PIP servo tilt
    # Each pad may reach the joint's housing (the servo at A) and past it by
    # its overhang: no limit set here (as the linkage's), only collisions and the finger.
    assert config.bounds["bpp_from_a_mm"][0] == pytest.approx(-100.0)
    limited = load("goal.pads.proximal.overhang_max_mm=0", "goal.pads.distal.overhang_max_mm=2")
    assert limited.bounds["bpp_from_a_mm"][0] == pytest.approx(-11.5)
    assert limited.search_bounds["app_x_mm"][1] == pytest.approx(
        min(limited.search_bounds["a_x_mm"][1] + 9.5, limited.goal.extended_length_bounds_mm[1]))


def test_the_pads_and_servo_follow_the_goal():
    from core.optimization.revolute import geometry

    config = load()
    geo = geometry(X, config)
    packaging = config.goal.packaging
    # The proximal pad on O-A (the servo a collision), the distal pad lined
    # up with it on A-C; each laid from its outer end.
    assert geo.proximal_pad[:, 1].max() == pytest.approx(0.0)
    assert geo.proximal_pad[:, 0].min() == pytest.approx(packaging.proximal_pad_start_mm)
    assert geo.distal_pad_local[:, 0].max() == pytest.approx(55.0)  # at C
    assert geo.distal_pad_local[:, 1].max() == pytest.approx(0.0)  # aligned
    # Flexed by phi about A, the distal pad keeps its distance to A.
    rest, bent = geo.distal_pad([0.0])[0], geo.distal_pad([math.radians(90.0)])[0]
    a = np.array((45.0, 0.0))
    assert np.linalg.norm(rest - a, axis=1) == pytest.approx(np.linalg.norm(bent - a, axis=1))
    assert geo.tip([math.radians(90.0)])[0] == pytest.approx((45.0, -55.0))
    # The distal servo on A, laid back along -X, tilted dorsally.
    assert geo.distal_servo.pivot == (45.0, 0.0)
    assert geo.distal_servo.rear == pytest.approx((0.0, 1.0), abs=1e-12)  # standing dorsal


SHORT_PADS = ("goal.pads.proximal.length_mm=[5, null]", "goal.pads.distal.length_mm=[5, null]")
"""Pads of any useful length (the fold, not the least lengths, sets them)."""


def test_a_design_is_evaluated_on_the_linkage_objectives():
    config = load(*SHORT_PADS)
    # On the lines at 120 deg the faces cross 5 / tan 30 deg = 8.7 mm from A,
    # inside the servo's front (9.5 mm): no pad clear of the servo is wasted.
    assert MECHANISM.evaluate(X, config).constraints["pad_waste_mm"] < 0.0
    e = MECHANISM.evaluate(X, config)
    assert e.feasible
    assert e.torque_ratio_max == 1.0  # direct drive
    effective = e.proximal_effective_mm + e.distal_effective_mm
    assert e.pad_coverage == pytest.approx(effective / e.finger_length_mm)
    assert effective == pytest.approx(e.proximal_support_length_mm + e.distal_support_length_mm, abs=0.1)
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
    # No setback: each pad reaches past the servo's housing by its overhang at
    # most (none here), the housing itself a collision.
    no_overhang = load("goal.pads.proximal.overhang_max_mm=0", "goal.pads.distal.overhang_max_mm=0")
    long = list(X)
    long[2] = 45.0 + 9.5 + 1.0
    assert MECHANISM.evaluate(long, no_overhang).constraints["proximal_support_mm"] == pytest.approx(1.0)
    under = list(X)
    under[3] = -9.5 - 2.0
    assert MECHANISM.evaluate(under, no_overhang).constraints["distal_support_mm"] == pytest.approx(2.0)
    # No overhang limit (null, as the linkage's): no reach limit either.
    assert MECHANISM.evaluate(long, load()).constraints["proximal_support_mm"] <= 0.0
    # On O-A, the proximal pad clears the servo's rounded front (9.5 mm about
    # A) only 9.5 mm short of A, whatever drop the vector holds.
    short_of = list(X)
    short_of[2] = 45.0 - 9.6
    assert MECHANISM.evaluate(short_of, load()).constraints["pip_servo_pad_mm"] <= 0.0
    short_of[5] = 9.5
    assert MECHANISM.evaluate(short_of, load()).constraints["pip_servo_pad_mm"] <= 0.0
    under = list(X)
    under[2] = 45.0
    assert MECHANISM.evaluate(under, load()).constraints["pip_servo_pad_mm"] > 0.0


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
    floor = search.least_a_x(config.goal.packaging, tuple(config.goal.mcp_servo_angles_rad), 90.0)  # standing
    for _ in range(200):
        x = problem.to_mm(np.round(problem.xl + rng.random(problem.n_var) * (problem.xu - problem.xl)))
        if x is None:
            continue
        a, ac, app, bpp = x[:4]
        assert 90.0 - 0.1 <= a + ac <= 100.0 + 0.1 and a >= floor - 0.6
        assert app <= a + ac + 0.1 and bpp >= -a - 0.1  # within the finger (no overhang limit here)
        assert ac - bpp >= 25.0 - 0.1
    assert problem.to_mm(problem.to_steps(X)) == pytest.approx(X, abs=0.11)


FIXED_PADS = ("goal.pads.proximal.length_mm=[20, 20]", "goal.pads.distal.length_mm=[25, 25]",
              "goal.link.finger_length_mm=[60, 300]")


def test_each_pad_mounts_on_its_segment_line():
    from core.optimization.revolute import geometry

    config = load()
    assert config.bounds["distal_drop_mm"] == config.bounds["proximal_drop_mm"] == (0.0, 0.0)  # on the lines
    for drop in (0.0, 4.0, 9.5):  # whatever the vector holds (searched before 2026-11-03)
        x = (*X[:4], drop, drop, *X[6:])
        geo = geometry(x, config)
        assert geo.distal_pad_local[:, 1].max() == pytest.approx(0.0)
        assert geo.proximal_pad[:, 1].max() == pytest.approx(0.0)
        assert geo.distal_mount == pytest.approx(np.array(((29.0, 0.0), (55.0, 0.0))))
        # The finger's thickness at the distal pad: from A-C to the pad's face.
        assert MECHANISM.evaluate(x, config).distal_depth_mm == pytest.approx(5.0)
    # Not searched: the design comes back from its coordinates with the housing's drops.
    assert len(MECHANISM.search_names(config)) == 5
    x = X
    assert MECHANISM.to_design(MECHANISM.from_design(x, config), config) == pytest.approx(x)


def test_each_pad_counts_outside_the_fold_at_the_required_flexion():
    from core.morphology.mechanism.revolute.solids import crossing_free
    from core.optimization.revolute import geometry

    # Folded to 150 deg the faces, 5 mm palmar of the shaft, cross on the
    # bisector 5 / tan(15 deg) = 18.7 mm from A along each segment, past the servo's front.
    config = load(*SHORT_PADS, "goal.motion.dip_flexion_deg=150")
    long = (60.0, 40.0, 50.0, 10.0, 0.0, 0.0, 90.0)
    e = MECHANISM.evaluate(long, config)
    phi = np.asarray(config.goal.servo_samples_rad)
    last = phi[np.flatnonzero(phi >= config.goal.required_rom_rad - 1e-12)[0]]
    free = crossing_free(geometry(long, config), [last])
    assert e.proximal_usable_length_mm == pytest.approx(free["proximal_pad"][0])
    assert e.proximal_usable_length_mm == pytest.approx(60.0 - 5.0 / math.tan(math.radians(15.0)) - 9.5, abs=0.3)
    assert e.distal_usable_length_mm == pytest.approx(min(free["distal_pad"][0], 30.0))


def test_a_collision_between_samples_is_caught_on_the_optimization_grid():
    # The pads graze each other near 153.7 deg, between the 1 deg steps:
    # the coarse sweep finds what the verification grid does.
    overrides = (*FIXED_PADS, "goal.motion.dip_flexion_deg=180")
    x = (52.0, 58.0, 42.0, 33.0, 0.0, 0.0, 90.0)
    coarse = load(*overrides)
    fine = MECHANISM.refine(coarse, coarse.goal.verify_step_rad)
    found, verified = MECHANISM.evaluate(x, coarse), MECHANISM.evaluate(x, fine)
    assert not found.feasible and not verified.feasible
    assert found.collision_pair == verified.collision_pair == "distal_pad/proximal_pad"
    assert math.degrees(verified.phi_max_rad) - 1.0 <= math.degrees(found.phi_max_rad) <= math.degrees(verified.phi_max_rad)


def test_the_idealized_2r_has_only_a_pin_at_the_joint():
    from core.optimization.revolute import geometry

    idealized = mechanism("revolute_idealized")
    config = load_yaml(idealized, ROOT / "linkage" / "config.yaml", PINNED)[0]
    assert not config.joint_servo and config.joint_radius_mm == 3.0  # A's pin housing, 6 mm
    assert config.bounds["distal_drop_mm"] == (3.0, 3.0)  # tangent to the pin's housing
    assert config.bounds["servo_tilt_deg"] == (0.0, 0.0)  # no servo to tilt
    x = (*X[:4], 3.0, 3.0, 0.0)
    geo = geometry(x, config)
    assert geo.proximal_pad[:, 1].max() == pytest.approx(-3.0)
    e = idealized.evaluate(x, config)
    # Nothing at the joint but the pin: the pads are checked against it, not a servo.
    assert e.constraints["mcp_servo_mm"] == 0.0 and e.constraints["pip_servo_pad_mm"] <= 0.0
    under = (*X[:2], 45.0, *X[3:4], 0.0, 0.0, 0.0)  # the proximal pad to A, tangent to the pin whatever the drops
    assert idealized.evaluate(under, config).constraints["pip_servo_pad_mm"] <= 0.0
    assert idealized.config_from_json(idealized.config_to_json(config)) == config


def test_the_segment_ratios_bound_the_segments_as_the_linkages():
    from core.optimization.revolute.evaluate import segment_lengths

    # |OA''| = A''_x and |B''C| = |AC| - B'' from A, as shares of |OC| (here 100).
    assert segment_lengths(X) == pytest.approx((30.0, 26.0))
    config = load("goal.link.proximal_length_ratio=[0.35, null]")
    assert MECHANISM.evaluate(X, config).constraints["proximal_length_mm"] == pytest.approx(5.0)
    config = load("goal.link.distal_length_ratio=[0.25, null]")
    assert MECHANISM.evaluate(X, config).constraints["proximal_length_mm"] <= 0.0
