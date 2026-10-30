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
from core.morphology.linkage import SKETCH_LINKAGE  # noqa: E402
from core.morphology.linkage.packaging import LinkagePackaging  # noqa: E402
from visualize import sketch, solids  # noqa: E402
from visualize.motion import collisions, index_at, linkage_margins, servo_sweep  # noqa: E402

MODULES = sorted(name for _, name, _ in pkgutil.walk_packages(visualize.__path__, "visualize."))


@pytest.mark.parametrize("name", MODULES)
def test_every_module_imports(name, monkeypatch):
    # visualize.finger makes every gap exact for its figures; not for the other tests.
    monkeypatch.setattr(packaging_module, "EXACT_BAND_MM", packaging_module.EXACT_BAND_MM)
    importlib.import_module(name)


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
    phi, margins, sweep, index = linkage_margins(SKETCH_LINKAGE, packaging, 30.0)
    figure, (left, right) = plt.subplots(1, 2)
    try:
        for k in range(len(phi)):
            sketch.draw_linkage(left, SKETCH_LINKAGE, packaging, sweep, int(index[k]), collisions(margins, k))
        i = int(index[-1])
        solids.linkage_bars(right, SKETCH_LINKAGE, sweep.b[i], sweep.c[i], sweep.bp[i], "#2a78d6")
        sketch.frame(left)
        assert left.patches and right.lines
    finally:
        plt.close(figure)
