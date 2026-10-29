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
