"""The mechanism comparison study: one frozen goal for every mechanism, the
link shares held by each, and the runs' convergence record."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from core.optimization.analysis.convergence import best_by_generation
from core.optimization.mechanism import mechanism, read_config_file

STUDY = Path(__file__).resolve().parents[1] / "scripts" / "optimize" / "comparison"
MECHANISMS = ("revolute", "revolute_parallel", "revolute_idealized", "rcj", "linkage")


def load(path: Path):
    values = read_config_file(path)
    found = mechanism(values.pop("mechanism"))
    output = values.pop("output")
    values, settings = found.split_settings(values)
    return found, found.config_from_mapping(values), settings, output


def goal_of(found, config):
    return config if found.name == "linkage" else config.goal


def test_the_base_fixes_the_optimizer_output_and_requirements():
    raw = yaml.safe_load((STUDY / "base.yaml").read_text(encoding="utf-8"))
    optimizer = raw["solver"]["optimizer"]
    assert "preset" not in optimizer and "presets" not in optimizer
    assert (optimizer["algorithm"], optimizer["population"], optimizer["warm_start"]) == ("nsga2", 200, [])
    assert raw["output"] == "output/comparison"
    assert raw["goal"]["pads"]["proximal"]["length_mm"][0] == 20
    assert raw["goal"]["pads"]["on_links"] is False
    assert "joint_span_ratio" not in raw["goal"]["link"]


@pytest.mark.parametrize("name", MECHANISMS)
def test_every_mechanism_reads_the_same_goal(name):
    found, config, settings, output = load(STUDY / f"{name}_h5_flex120.yaml")
    goal = goal_of(found, config)
    reference_found, reference, *_ = load(STUDY / "linkage_h5_flex120.yaml")
    assert output == "output/comparison" and settings.population_size == 200 and settings.algorithm == "nsga2"
    assert goal.warm_start == () and goal.seeds == (0, 1, 2)
    for field in ("extended_length_bounds_mm", "proximal_length_ratio_bounds", "distal_length_ratio_bounds",
                  "joint_span_ratio_bounds", "minimum_proximal_support_mm", "minimum_distal_support_mm",
                  "required_rom_rad", "objectives", "transmission_angle_range_rad"):
        assert getattr(goal, field) == getattr(reference, field), field
    assert goal.packaging.proximal_pad.height_mm == goal.packaging.distal_pad.height_mm == 5.0


def test_every_baseline_gets_at_least_the_linkage_budget():
    budgets = {}
    for name in MECHANISMS:
        _, _, settings, _ = load(STUDY / f"{name}_h10_flex150.yaml")
        budgets[name] = settings.population_size * settings.generations
    assert all(budgets[name] >= budgets["linkage"] for name in MECHANISMS)


def test_the_onlinks_variant_differs_only_in_the_pads():
    found, onlinks, *_ = load(STUDY / "linkage_onlinks_h10_flex150.yaml")
    _, free, *_ = load(STUDY / "linkage_h10_flex150.yaml")
    assert onlinks.pads_on_links and not free.pads_on_links
    import dataclasses
    assert dataclasses.replace(onlinks, pads_on_links=False) == free


@pytest.mark.parametrize("name, x, column", [
    # |OA| 60 of |OC| 100: 5 mm past the 0.55 share.
    ("revolute", (60.0, 40.0, 30.0, 20.0, 0.0, 0.0, 20.0), "a_x_mm"),
    ("revolute_idealized", (60.0, 40.0, 30.0, 20.0, 0.0, 0.0, 0.0), "a_x_mm"),
    ("revolute_parallel", (60.0, 40.0, 30.0, 20.0, 0.0, 0.0, 20.0, 5.0, 12.0, 10.0, 90.0), "a_x_mm"),
    # BC 60 of |OC| 100 (|OA| 30, r 5): 5 mm past 0.55, and |OA| 5 mm short of 0.35.
    ("rcj", (30.0, 5.0, 60.0, 25.0, 0.0, 0.0, 5.0), "bc_length_mm"),
])
def test_the_baselines_hold_the_link_shares(name, x, column):
    found, config, *_ = load(STUDY / f"{name}_h5_flex120.yaml")
    evaluation = found.evaluate(x, config)
    assert evaluation.constraints["proximal_length_mm"] == pytest.approx(5.0)
    assert not evaluation.feasible
    within = list(x)
    if name == "rcj":
        within[0], within[2] = 40.0, 50.0  # |OA| 0.40, BC 0.50 of 100
    else:
        within[0], within[1] = 50.0, 50.0
    assert found.evaluate(tuple(within), config).constraints["proximal_length_mm"] <= 0.0


def test_the_search_box_takes_the_shares():
    found, config, *_ = load(STUDY / "revolute_h5_flex120.yaml")
    assert config.bounds["a_x_mm"] == (pytest.approx(0.35 * 99.5), pytest.approx(0.55 * 100.0))
    assert config.bounds["ac_length_mm"] == (pytest.approx(25.0), pytest.approx(55.0))


def test_the_convergence_curve_is_the_running_best_per_generation():
    history = {
        "objective_names": np.array(["pad_coverage"]),
        # Coverage stored negated (minimized); generation 0 found nothing feasible.
        "generation": np.array([1, 1, 2, 4]),
        "objectives": np.array([[-0.40], [-0.55], [-0.50], [-0.60]]),
        "evaluations": np.array([10, 10, 10, 10, 10]),
    }
    curve = best_by_generation(history, "pad_coverage", larger_is_better=True)
    assert np.isnan(curve.best[0])
    assert curve.best[1:] == pytest.approx([0.55, 0.55, 0.55, 0.60])
    assert list(curve.evaluations) == [10, 20, 30, 40, 50]
    assert curve.generation_within(0.01) == 4
    assert curve.gain_after(0.6) == pytest.approx(0.05)


@pytest.mark.parametrize("label, field, value", [
    ("ratio_wide", ("proximal_length_ratio_bounds", "distal_length_ratio_bounds"), (0.25, 0.65)),
    ("icr_control", ("icr0_share_bounds",), (0.45, 0.55)),
])
def test_each_check_differs_from_the_grid_only_in_its_bounds(label, field, value):
    import dataclasses

    names = MECHANISMS if label == "ratio_wide" else ("rcj", "linkage")
    for name in names:
        found, checked, settings, output = load(STUDY / f"{name}_{label}_h5_flex120.yaml")
        _, plain, plain_settings, plain_output = load(STUDY / f"{name}_h5_flex120.yaml")
        goal, plain_goal = goal_of(found, checked), goal_of(found, plain)
        assert all(getattr(goal, f) == value for f in field)
        assert dataclasses.replace(goal, **{f: getattr(plain_goal, f) for f in field}) == plain_goal
        assert (settings, output) == (plain_settings, plain_output)


def test_the_extended_lengths_add_up_to_the_finger():
    # A_z = B_z = 0: |OA| + |AB| + BC = |OC| in the extended pose, every mechanism.
    from core.optimization.linkage.design import decode
    from core.optimization.linkage.evaluate import finger_length_mm
    from core.optimization.rcj.config import finger_length_mm as rcj_length

    p = decode((35.0, 68.0, 0.0, 39.7, 11.6, 61.9, 10.7, 70.0, 32.0, 0.0, 19.4, -3.1, 0.0, 0.0, 3.0, 3.0), 0.1)
    assert p.a_x_mm + (p.b_x_mm - p.a_x_mm) + p.bc_length_mm == pytest.approx(finger_length_mm(p))
    assert 47.5 + 2 * 9.5 + 33.5 == pytest.approx(rcj_length(47.5, 9.5, 33.5))


@pytest.mark.parametrize("name, x, icr_x", [
    # The RCJ's centre at A + r: 47.5 + 9.5 = 57 of |OC| 100, 2 mm past 0.55.
    ("rcj", (47.5, 9.5, 33.5, 43.7, 8.5, 4.3, 9.5), 57.0),
    # The linkage's where A-B' and A'-B cross.
    ("linkage", (35.0, 68.0, 0.0, 39.7, 11.6, 61.9, 10.7, 70.0, 32.0, 0.0, 19.4, -3.1, 0.0, 0.0, 3.0, 3.0), None),
])
def test_the_icr_control_holds_the_extended_centre_of_rotation(name, x, icr_x):
    from core.optimization.analysis.icr import icr0_x_mm

    found, config, *_ = load(STUDY / f"{name}_icr_control_h5_flex150.yaml")
    _, plain, *_ = load(STUDY / f"{name}_h5_flex150.yaml")
    evaluation, free = found.evaluate(x, config), found.evaluate(x, plain)
    found_x = icr0_x_mm(name, x, found.variable_names)
    if icr_x is not None:
        assert found_x == pytest.approx(icr_x)
    length = evaluation.finger_length_mm
    outside = max(0.45 * length - found_x, found_x - 0.55 * length)
    # The report's from the design as built (the links' lengths on their 0.1 mm grid).
    assert found.report(evaluation)["icr0_share"] == pytest.approx(found_x / length, abs=1e-3)
    column = "proximal_length_mm"
    if outside > 0.0:
        assert evaluation.constraints[column] == pytest.approx(max(outside, free.constraints[column]))
    else:
        assert evaluation.constraints[column] == pytest.approx(free.constraints[column])
