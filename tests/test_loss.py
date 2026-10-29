"""The loss of contact face by rule and depth (core.optimization.analysis.loss)."""

from pathlib import Path

import numpy as np
import pytest

from core.morphology.linkage.packaging import contact_faces, fold_free_lengths
from core.optimization.analysis.loss import CATEGORIES, face_loss, intrusion_mm, scene
from core.optimization.mechanism import mechanism
from core.optimization.study import load_yaml

CONFIGS = Path(__file__).resolve().parents[1] / "scripts" / "optimize"
PHI = np.arange(0.0, 151.0, 10.0)
DESIGNS = {
    "linkage": (34.3, 76.0, 0.0, 39.7, 7.7, 61.8, 14.5, 20.0, 25.0, 0.0, 9.7, 2.9, 0.0, 0.0),
    "revolute": (45.0, 55.0, 30.0, 29.0, 9.5, 9.5, 20.0),
    "revolute_idealized": (45.0, 55.0, 30.0, 29.0, 3.0, 3.0, 0.0),
    "revolute_parallel": (50.0, 50.0, 39.5, 25.0, 3.0, 3.0, 20.0, 5.0, 15.0),
    "rcj": (52.0, 9.5, 29.0, 39.5, 1.0, 9.5, 9.5),
}
"""A design of each mechanism (the mechanisms' own tests' designs)."""
OVERRIDES = ("goal.hardware.pulley_radius_mm=6", "goal.link.finger_length_mm=[90, 110]",
             "goal.pads.proximal.length_mm=[0, null]", "goal.pads.distal.length_mm=[0, null]")


def config_of(name):
    return load_yaml(mechanism(name), CONFIGS / "linkage" / "config.yaml", OVERRIDES)[0]


@pytest.mark.parametrize("name", list(DESIGNS))
def test_every_mechanism_has_a_scene(name):
    built = scene(name, config_of(name), DESIGNS[name], PHI)
    for side in ("proximal_pad", "distal_pad"):
        face, normal = built.faces[side]
        assert face.shape == (len(PHI), 2, 2) and normal.shape == (len(PHI), 2)
        assert np.linalg.norm(normal, axis=1) == pytest.approx(1.0)
        assert set(built.solids[side]) == set(CATEGORIES)
    loss = face_loss(built, "probe", 5.0)
    for side, result in loss.items():
        # The union hides at least what any one category hides, at most the face.
        for name in CATEGORIES:
            assert np.all(result.lost >= result.by_category[name] - 1e-9)
        assert np.all(result.lost <= result.length + 1e-9) and np.all(result.lost >= 0.0)
    if name == "revolute_parallel":
        assert built.solids["proximal_pad"]["pulley"].disks  # the belt's pulleys stand in front, outer layer or not
    if name == "rcj":
        assert built.solids["proximal_pad"]["roller"].disks


def test_the_segment_rule_is_the_fold():
    config = config_of("linkage")
    built = scene("linkage", config, DESIGNS["linkage"], PHI)
    loss = face_loss(built, "segment")
    (pf, pn), (df, dn) = built.faces["proximal_pad"], built.faces["distal_pad"]
    free = fold_free_lengths(pf, pn, df, dn)
    for side in ("proximal_pad", "distal_pad"):
        assert loss[side].free == pytest.approx(free[side])
        assert loss[side].by_category["link"] == pytest.approx(0.0)


def test_a_deeper_probe_loses_at_least_as_much():
    config = config_of("revolute")
    built = scene("revolute", config, DESIGNS["revolute"], PHI)
    shallow, deep = face_loss(built, "probe", 2.0), face_loss(built, "probe", 20.0)
    for side in ("proximal_pad", "distal_pad"):
        assert np.all(deep[side].lost >= shallow[side].lost - 1e-9)
    # At no depth only what overlaps the face is lost: nothing before the pads meet (here past 120 deg).
    zero = face_loss(built, "probe", 0.0)["proximal_pad"].lost
    assert zero[PHI <= 120.0] == pytest.approx(0.0)


def test_the_intrusion_is_the_least_depth_that_hides_something():
    config = config_of("linkage")
    built = scene("linkage", config, DESIGNS["linkage"], PHI)
    near = intrusion_mm(built, "proximal_pad", ["link", "pin"], 30.0, 0.5)
    for depth, flexion in zip(near, range(len(PHI))):
        if np.isnan(depth):
            continue
        one = scene("linkage", config, DESIGNS["linkage"], PHI[flexion:flexion + 1])
        only = {side: {name: (shape if name in ("link", "pin") else type(shape)()) for name, shape in solids.items()}
                for side, solids in one.solids.items()}
        one.solids = only
        assert face_loss(one, "probe", depth)["proximal_pad"].lost[0] > 0.0
        assert face_loss(one, "probe", depth - 0.5)["proximal_pad"].lost[0] == pytest.approx(0.0)
