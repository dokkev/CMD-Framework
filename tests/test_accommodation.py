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
REVOLUTE_X = (47.5, 52.5, 36.0, 15.0, 0.0, 0.0, 90.0)
"""A 2R with room for 26.5 mm proximal and 37.5 mm distal modules, upright servo, pads on the link lines."""


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
        assert goal.hyperextension_rad == 0.0  # from extension only, the same check for both
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
    # Every required flexion for both (none ruled out beforehand).
    for flexion in module.FLEXIONS_DEG:
        assert (STUDY / f"linkage_P25x5_D25x5_flex{flexion}.yaml").exists()
        assert (STUDY / f"revolute_P25x5_D25x5_flex{flexion}.yaml").exists()
        _, config, _ = load_config(STUDY / f"linkage_P25x5_D25x5_flex{flexion}.yaml")
        assert config.required_rom_rad == pytest.approx(np.radians(flexion))


@pytest.mark.parametrize("name, x", [("linkage", LINKAGE_X), ("revolute", REVOLUTE_X)])
def test_a_module_clear_through_the_motion_counts_whole(name, x):
    found, config, _ = load_config(STUDY / f"{name}_P25x5_D25x5_flex120.yaml")
    evaluation = found.evaluate(x, config)
    assert np.isfinite(evaluation.phi_max_rad)
    report = found.report(evaluation)
    assert report["proximal_effective_mm"] == pytest.approx(report["proximal_face_mm"]) == pytest.approx(25.0)
    assert report["distal_effective_mm"] == pytest.approx(report["distal_face_mm"]) == pytest.approx(25.0)


def test_the_capsules_stop_the_motion_where_a_bar_meets_a_pad():
    # The same finger and modules with and without the bars' capsules: they can only stop it sooner.
    modules = Modules(25.0, 5.0, 25.0, 5.0)
    for name, x in (("linkage", LINKAGE_X), ("revolute", REVOLUTE_X)):
        found, bare = load(COMPARISON / f"{name}_h5_flex150.yaml")
        _, capsules = load(COMPARISON / f"{name}_h5_flex150.yaml", "goal.collision.param.link_radius_mm=3")
        _, wide = load(COMPARISON / f"{name}_h5_flex150.yaml", "goal.collision.param.link_radius_mm=30")
        reach = [phi_max_deg(found, config, x, modules) for config in (bare, capsules, wide)]
        assert reach[1] <= reach[0] + 1e-9
        assert np.isnan(reach[2]) or reach[2] < reach[0]


def test_the_dorsal_space_is_measured_from_os_line():
    from core.optimization.analysis.loss import dorsal_space_mm, scene

    found, config = load(COMPARISON / "revolute_h5_flex150.yaml")
    config = with_modules(config, Modules(25.0, 5.0, 25.0, 5.0))
    servo = config.goal.packaging.pip_servo
    assert dorsal_space_mm(scene("revolute", config, REVOLUTE_X, [0.0])) == pytest.approx(servo.rear_extent_mm)
    # A pad's drop does not move the datum.
    dropped = (*REVOLUTE_X[:4], 4.0, 4.0, REVOLUTE_X[6])
    assert dorsal_space_mm(scene("revolute", config, dropped, [0.0])) == pytest.approx(servo.rear_extent_mm)
    tight = with_modules(load(COMPARISON / "revolute_h5_flex150.yaml", "goal.link.dorsal_space_mm=20")[1],
                         Modules(25.0, 5.0, 25.0, 5.0))
    evaluation = found.evaluate(REVOLUTE_X, tight)
    assert evaluation.constraints["dorsal_space_mm"] == pytest.approx(servo.rear_extent_mm - 20.0)


@pytest.mark.parametrize("name, x", [("linkage", LINKAGE_X), ("revolute", REVOLUTE_X)])
def test_a_fixed_finger_changes_its_modules_not_its_supports(name, x):
    from core.optimization.analysis.loss import scene

    found, config = load(COMPARISON / f"{name}_h5_flex150.yaml")
    short, full = (with_modules(config, Modules(length, 5.0, 20.0, 5.0)) for length in (10.0, 25.0))
    built = [scene(name, c, x, [0.0]) for c in (short, full)]
    # The proximal module from its outer end, as long as asked; the distal
    # carrier the same whatever module it carries.
    proximal = [b.solids["distal_pad"]["other_pad"].polygons[0] for b in built]
    assert [float(np.ptp(np.asarray(p)[..., 0])) for p in proximal] == pytest.approx([10.0, 25.0], abs=0.11)
    assert np.asarray(proximal[0])[..., 0].min() == pytest.approx(np.asarray(proximal[1])[..., 0].min())
    carriers = [b.solids["proximal_pad"]["carrier"].polygons for b in built]
    assert all(np.allclose(a, b) for a, b in zip(*carriers))
    # The design's x is untouched; a module longer than its support does not mount.
    assert np.isnan(phi_max_deg(found, config, x, Modules(60.0, 5.0, 20.0, 5.0)))
    # Thicker modules flex no further.
    thick, thin = Modules(20.0, 10.0, 20.0, 10.0), Modules(20.0, 5.0, 20.0, 5.0)
    assert phi_max_deg(found, config, x, thick) <= phi_max_deg(found, config, x, thin)


def test_the_proximal_support_stays_whatever_module_it_carries():
    from core.morphology.linkage import FingerLinkage
    from core.morphology.linkage.packaging import proximal_carrier_polygon
    from core.optimization.analysis.loss import scene
    from core.optimization.linkage.design import decode
    from core.optimization.linkage.evaluate import support_margins

    found, config = load(COMPARISON / "linkage_h5_flex150.yaml", "goal.collision.param.proximal_carrier_height_mm=4")
    linkage = FingerLinkage(decode(LINKAGE_X, 0.1))
    supports = []
    for length in (10.0, 25.0):
        carried = with_modules(config, Modules(length, 5.0, 20.0, 5.0))
        polygon = proximal_carrier_polygon(linkage, carried.packaging, 4.0)
        # From the pad's outer end to A'', 4 mm dorsal of the mounting line, whatever the module.
        assert polygon[:, 0].min() == pytest.approx(carried.packaging.proximal_pad_start_mm)
        assert polygon[:, 0].max() == pytest.approx(linkage.parameters.app_x_mm)
        assert np.ptp(polygon[:, 1]) == pytest.approx(4.0)
        supports.append(polygon)
        assert any(np.allclose(np.asarray(p), polygon) for p in scene("linkage", carried, LINKAGE_X, [0.0])
                   .solids["distal_pad"]["carrier"].polygons)
    assert np.allclose(*supports)
    # The distal pad folded onto the support is a collision, even with a short proximal module.
    b, c = np.array([[30.0, -9.0]]), np.array([[2.8, -9.0]])  # B-C laid back, the pad through the support
    margins = support_margins(linkage, with_modules(config, Modules(10.0, 5.0, 20.0, 5.0)).packaging, b, c, 4.0)
    assert margins[("distal_pad", "proximal_carrier")][0] < 0.0


def test_the_dorsal_space_counts_the_bars_capsules():
    from core.optimization.analysis.loss import dorsal_space_mm, scene

    found, config = load(COMPARISON / "linkage_h5_flex150.yaml")
    built = scene("linkage", config, LINKAGE_X, [0.0])
    bars = max(float(np.asarray(s)[..., 1].max()) for s in built.solids["proximal_pad"]["link"].segments)
    assert dorsal_space_mm(built, 3.0) == pytest.approx(max(dorsal_space_mm(built), bars + 3.0))
    assert dorsal_space_mm(built, 3.0) >= dorsal_space_mm(built)
