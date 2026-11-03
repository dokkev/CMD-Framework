"""The figures' shared pieces: every module imports, and the sketch linkage poses and draws."""

import importlib
import math
import pkgutil
from types import SimpleNamespace

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402

import core.morphology.packaging.gaps as packaging_module  # noqa: E402
import visualize  # noqa: E402
from core.morphology.mechanism.linkage import SKETCH_LINKAGE  # noqa: E402
from core.morphology.mechanism.linkage.packaging import LinkagePackaging  # noqa: E402
from core.optimization.mechanism import mechanism  # noqa: E402
from core.optimization.study import load_yaml  # noqa: E402
from visualize import sketch, solids  # noqa: E402
from visualize.finger import FingerView, view_of  # noqa: E402
from visualize.finger.mechanism import JOINTS, FourBarView  # noqa: E402
from visualize.motion import collisions, index_at, linkage_margins, servo_sweep  # noqa: E402

CONFIG = "scripts/optimize/linkage/config.yaml"
OPEN = ("goal.hardware.pulley_radius_mm=6", "goal.pads.proximal.length_mm=[0, null]",
        "goal.pads.distal.length_mm=[0, null]")
"""The other mechanisms read the linkage's goal, with a belt's pulleys and pads of any length."""
MODULES = sorted(name for _, name, _ in pkgutil.walk_packages(visualize.__path__, "visualize."))


@pytest.mark.parametrize("name", MODULES)
def test_every_module_imports(name, monkeypatch):
    # A figure may make every gap exact for itself; not for the other tests.
    monkeypatch.setattr(packaging_module, "EXACT_BAND_MM", packaging_module.EXACT_BAND_MM)
    importlib.import_module(name)


def test_the_views_gaps_are_exact_only_while_they_measure():
    band = packaging_module.EXACT_BAND_MM
    view = FingerView("linkage", FourBarView(SKETCH_LINKAGE, LinkagePackaging()))
    view.margins([0.0, 30.0])
    assert packaging_module.EXACT_BAND_MM == band


def test_index_at_is_the_first_sample_reaching_the_flexion():
    sweep = SimpleNamespace(total_flexion_rad=np.array([0.0, 0.2, 0.5, 0.4, 0.9]))
    assert list(index_at(sweep, [0.0, 0.3, 0.45, 0.9, 2.0])) == [0, 2, 2, 4, 4]


def test_the_servo_sweep_flexes_the_finger():
    servo, sweep = servo_sweep(SKETCH_LINKAGE)
    flexion = sweep.total_flexion_rad
    assert len(flexion) > 10 and flexion[0] == pytest.approx(0.0, abs=1e-9)
    assert flexion[-1] > math.radians(60.0)
    assert np.all(np.diff(np.sign(servo[1:])) == 0)  # one way from extension


def test_the_sketch_linkage_draws_at_every_flexion():
    packaging = LinkagePackaging()
    view = FingerView("linkage", FourBarView(SKETCH_LINKAGE, packaging))
    phi = np.arange(0.0, view.reachable_deg() + 1e-9, 30.0)
    margins = view.margins(phi)
    _, _, sweep, index = linkage_margins(SKETCH_LINKAGE, packaging, 30.0)
    figure, (left, right) = plt.subplots(1, 2)
    try:
        for k, pose in enumerate(phi):
            view.draw(left, pose)
            assert view.collisions(pose) == collisions(margins, k)
        i = int(index[-1])
        solids.linkage_bars(right, SKETCH_LINKAGE, sweep.b[i], sweep.tip[i], sweep.bp[i], "#2a78d6")
        sketch.frame(left)
        assert left.patches and right.lines
    finally:
        plt.close(figure)


def test_the_four_bar_view_poses_the_flexion_asked():
    view = FingerView("linkage", FourBarView(SKETCH_LINKAGE, LinkagePackaging()))
    phi = np.array([60.0, 0.0, 30.0, 500.0])  # any order; past the reach, at the reach
    posed = np.degrees(view.joint.sweep(phi).q_pip_rad)
    assert posed[:3] == pytest.approx(phi[:3])
    assert posed[3] == pytest.approx(view.reachable_deg())
    assert view.icr(phi).shape == (4, 2) and np.all(view.k(phi) > 0.0)
    # k: d(phi)/d(servo), as the servo sweep differences it
    servo, swept = servo_sweep(SKETCH_LINKAGE)
    i = int(index_at(swept, np.radians(30.0)))
    rate = np.gradient(swept.total_flexion_rad, np.abs(servo[:len(swept.total_flexion_rad)]))[i]
    assert view.k([30.0])[0] == pytest.approx(rate, rel=1e-3)


@pytest.mark.parametrize("name", sorted(JOINTS))
def test_every_mechanism_has_a_view(name):
    found = mechanism(name)
    config = load_yaml(found, CONFIG, () if name == "linkage" else OPEN)[0]
    low, high = map(np.asarray, found.search_box(config))
    x = found.to_design(np.round((low + high) / 2.0), config)
    view = view_of(name, config, x)
    pose = min(45.0, view.reachable_deg())
    assert view.margins([0.0, pose]) and view.tip(pose).shape == (2,)
    assert view.proximal_pad().shape[1] == 2 and view.distal_pad(pose).shape[1] == 2
    assert np.isnan(view.pads_mm[0]) and view.phi_max_deg == view.reachable_deg()
    figure, axis = plt.subplots()
    try:
        view.draw(axis, pose)
        assert axis.patches and axis.lines
    finally:
        plt.close(figure)
