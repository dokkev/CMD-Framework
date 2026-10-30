"""The comparison model (contact module accommodation): links as capsules
against the pads, no loss rule, the dorsal space, fixed-finger module sweeps."""

from pathlib import Path

import numpy as np
import pytest

from core.morphology.linkage.packaging import segment_polygon_gaps
from core.optimization.analysis.tradeoff import Modules, load, phi_max_deg, with_modules
from core.optimization.mechanism import mechanism, read_config_file

STUDY = Path(__file__).resolve().parents[1] / "scripts" / "optimize" / "accommodation"
COMPARISON = STUDY.parent / "comparison"
LINKAGE_X = (40.2, 72.8, 0.0, 44.9, 11.6, 63.8, 10.0, 25.0, 27.2, 0.0, -4.7, -9.0, 0.0, 0.0, 3.0, 3.0)
"""The production CoDe (paper, balanced), its pads' drops at its pins' radius."""
REVOLUTE_X = (47.5, 52.5, 29.5, 15.0, 0.0, 0.0, 90.0)


def load_config(path, *overrides):
    values = read_config_file(path, overrides)
    found = mechanism(values.pop("mechanism"))
    values.pop("output")
    values, settings = found.split_settings(values)
    return found, found.config_from_mapping(values), settings


def test_a_capsule_clears_a_polygon_by_the_segments_distance():
    square = np.array([[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]] * 4)
    start = np.array([[0.0, 13.0], [12.0, 12.0], [5.0, 5.0], [-5.0, 5.0]])
    end = np.array([[10.0, 13.0], [20.0, 20.0], [6.0, 6.0], [15.0, 5.0]])
    # Above it, off its corner, inside it, through it.
    assert segment_polygon_gaps(square, start, end) == pytest.approx([3.0, 2.0 * np.sqrt(2.0), 0.0, 0.0])


def test_the_study_is_the_comparison_model():
    for stem in ("revolute_P25x5_D25x5_flex120", "linkage_P35x10_D25x5_flex120"):
        found, config, _ = load_config(STUDY / f"{stem}.yaml")
        goal = getattr(config, "goal", config)
        assert found.active_objectives(config) == ("finger_length",)
        assert goal.link_radius_mm == 3.0 and not goal.count_fold and goal.integrity_depth_mm is None
        assert goal.packaging.body_clearance_mm == 0.0 and goal.dorsal_space_mm == 34.0
        assert goal.required_rom_rad == pytest.approx(np.radians(120.0))
    _, config, _ = load_config(STUDY / "linkage_P35x10_D25x5_flex120.yaml")
    assert config.packaging.proximal_pad.height_mm == 10.0 and config.packaging.distal_pad.height_mm == 5.0
    assert config.minimum_proximal_support_mm == 35.0 and config.packaging.proximal_pad.max_length_mm == 35.0


def test_experiments_vary_one_module_each():
    import importlib.util

    spec = importlib.util.spec_from_file_location("make_configs", STUDY / "make_configs.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    pairs = module.pairs()
    assert len(pairs) == len(set(pairs)) == 2 * 9 - 1
    assert all(p == module.REFERENCE or d == module.REFERENCE for p, d in pairs)
    # Every required flexion for CoDe; the 2R not at 180 deg (no fixed pivot folds its pads flat).
    for flexion in module.FLEXIONS_DEG:
        assert (STUDY / f"linkage_P25x5_D25x5_flex{flexion}.yaml").exists()
        assert (STUDY / f"revolute_P25x5_D25x5_flex{flexion}.yaml").exists() == (flexion != 180)
        _, config, _ = load_config(STUDY / f"linkage_P25x5_D25x5_flex{flexion}.yaml")
        assert config.required_rom_rad == pytest.approx(np.radians(flexion))


@pytest.mark.parametrize("name, x", [("linkage", LINKAGE_X), ("revolute", REVOLUTE_X)])
def test_a_module_clear_through_the_motion_counts_whole(name, x):
    # The config's modules, their lengths given 0.1 mm (the links' grid) to round to.
    found, config, _ = load_config(STUDY / f"{name}_P25x5_D25x5_flex120.yaml", "goal.pads.proximal.length_mm=[24.8, 25]",
                                   "goal.pads.distal.length_mm=[24.8, 25]")
    modules = Modules(24.9, 5.0, 24.9, 5.0)
    evaluation = found.evaluate(with_modules(found, config, x, modules), config)
    assert np.isfinite(evaluation.phi_max_rad)
    report = found.report(evaluation)
    assert report["proximal_effective_mm"] == pytest.approx(report["proximal_face_mm"])
    assert report["distal_effective_mm"] == pytest.approx(report["distal_face_mm"])


def test_the_capsules_stop_the_motion_where_a_bar_meets_a_pad():
    # The same finger and modules with and without the bars' capsules: they can only stop it sooner.
    for name, x in (("linkage", LINKAGE_X), ("revolute", REVOLUTE_X)):
        modules = Modules(25.0, 5.0, 25.0, 5.0)
        found, bare = load(COMPARISON / f"{name}_h5_flex150.yaml", modules)
        _, capsules = load(COMPARISON / f"{name}_h5_flex150.yaml", modules,
                           "goal.collision.param.link_radius_mm=3")
        assert phi_max_deg(found, capsules, x, modules) <= phi_max_deg(found, bare, x, modules) + 1e-9
        _, wide = load(COMPARISON / f"{name}_h5_flex150.yaml", modules, "goal.collision.param.link_radius_mm=30")
        assert phi_max_deg(found, wide, x, modules) < phi_max_deg(found, bare, x, modules) or \
            np.isnan(phi_max_deg(found, wide, x, modules))


def test_the_dorsal_space_is_measured_from_the_pads_mounting_surface():
    from core.optimization.analysis.loss import dorsal_space_mm, scene

    modules = Modules(25.0, 5.0, 25.0, 5.0)
    found, config = load(COMPARISON / "revolute_h5_flex150.yaml", modules)
    built = scene("revolute", config, with_modules(found, config, REVOLUTE_X, modules), [0.0])
    servo = config.goal.packaging.pip_servo
    assert dorsal_space_mm(built) == pytest.approx(servo.rear_extent_mm)  # upright, the pads on the link line
    found, tight = load(COMPARISON / "revolute_h5_flex150.yaml", modules, "goal.link.dorsal_space_mm=20")
    evaluation = found.evaluate(with_modules(found, tight, REVOLUTE_X, modules), tight)
    assert evaluation.constraints["dorsal_space_mm"] == pytest.approx(servo.rear_extent_mm - 20.0)


def test_a_fixed_fingers_module_sweep_keeps_its_outer_ends():
    found, config = load(COMPARISON / "linkage_h5_flex150.yaml", Modules(0.0, 5.0, 25.0, 5.0))
    short, long = (with_modules(found, config, LINKAGE_X, Modules(length, 5.0, 20.0, 5.0)) for length in (10.0, 30.0))
    from core.morphology.linkage.packaging import pad_spans
    from core.optimization.linkage.design import decode

    spans = [pad_spans(decode(x, 0.1), config.packaging) for x in (short, long)]
    assert spans[0]["proximal_pad"].start_mm == spans[1]["proximal_pad"].start_mm
    assert [s["proximal_pad"].length_mm for s in spans] == pytest.approx([10.0, 30.0], abs=0.11)
    assert [s["distal_pad"].length_mm for s in spans] == pytest.approx([20.0, 20.0], abs=0.11)
    # Thicker modules flex no further.
    thick = Modules(30.0, 10.0, 25.0, 10.0)
    found, heavy = load(COMPARISON / "linkage_h5_flex150.yaml", thick)
    found, light = load(COMPARISON / "linkage_h5_flex150.yaml", Modules(30.0, 5.0, 25.0, 5.0))
    assert phi_max_deg(found, heavy, LINKAGE_X, thick) <= phi_max_deg(found, light, LINKAGE_X,
                                                                     Modules(30.0, 5.0, 25.0, 5.0))
