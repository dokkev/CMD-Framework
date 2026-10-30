"""The loss of contact face by rule and depth (core.optimization.analysis.loss)."""

from pathlib import Path

import numpy as np
import pytest

from core.morphology.linkage.packaging import contact_faces, fold_free_lengths, fold_hidden_points
from core.optimization.analysis.loss import CATEGORIES, face_loss, integrity_free, intrusion_mm, scene
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


def test_the_fold_points_measure_the_fold():
    config = config_of("revolute_idealized")
    built = scene("revolute_idealized", config, DESIGNS["revolute_idealized"], PHI)
    (pf, pn), (df, dn) = built.faces["proximal_pad"], built.faces["distal_pad"]
    free = fold_free_lengths(pf, pn, df, dn)
    for face, other, normal, side in ((pf, df, dn, "proximal_pad"), (df, pf, pn, "distal_pad")):
        length = np.linalg.norm(face[:, 1] - face[:, 0], axis=1)
        hidden = fold_hidden_points(face, other, normal, 400)
        assert length * (1.0 - hidden.mean(axis=1)) == pytest.approx(free[side], abs=2 * length.max() / 400)


@pytest.mark.parametrize("name", list(DESIGNS))
def test_integrity_keeps_what_no_pose_loses(name):
    built = scene(name, config_of(name), DESIGNS[name], PHI)
    (pf, pn), (df, dn) = built.faces["proximal_pad"], built.faces["distal_pad"]
    fold = fold_free_lengths(pf, pn, df, dn)
    shallow, deep = integrity_free(built, 0.0), integrity_free(built, 3.0)
    for side in ("proximal_pad", "distal_pad"):
        # Depth 0: the fold at its worst pose (to the probe's spacing); deeper: no more kept.
        assert shallow[side] <= float(fold[side].min()) + 0.5
        assert deep[side] <= shallow[side] + 1e-9
        # A point any one pose's probe loses is lost to the whole motion.
        assert deep[side] <= float(face_loss(built, "probe", 3.0)[side].free.min()) + 1e-9


def test_the_loss_depths_are_read_from_the_goal():
    base = load_yaml(mechanism("linkage"), CONFIGS / "linkage" / "config.yaml", OVERRIDES)[0]
    assert base.integrity_depth_mm is None and base.access_depth_mm == 20.0
    given = load_yaml(mechanism("linkage"), CONFIGS / "linkage" / "config.yaml",
                      (*OVERRIDES, "goal.loss.integrity_depth_mm=3", "goal.loss.access_depth_mm=15"))[0]
    assert given.integrity_depth_mm == 3.0 and given.access_depth_mm == 15.0
    with pytest.raises(ValueError):
        load_yaml(mechanism("linkage"), CONFIGS / "linkage" / "config.yaml",
                  (*OVERRIDES, "goal.loss.integrity_depth_mm=-1"))
    with pytest.raises(ValueError):
        load_yaml(mechanism("linkage"), CONFIGS / "linkage" / "config.yaml", (*OVERRIDES, "goal.loss.depth_mm=3"))


ACCOMMODATION = CONFIGS / "accommodation"
FEASIBLE = {
    "revolute": (47.5, 52.5, 36.0, 15.0, 0.0, 0.0, 90.0),
    "linkage": (40.2, 72.8, 0.0, 44.9, 11.6, 63.8, 10.0, 25.0, 27.2, 0.0, -4.7, -9.0, 0.0, 0.0, 3.0, 3.0),
}
"""Each mechanism's design that carries the accommodation's 25 x 5 mm modules through 120 deg."""


def integrity_config(name, *overrides):
    from core.optimization.mechanism import read_config_file

    values = read_config_file(ACCOMMODATION / f"{name}_P25x5_D25x5_flex120.yaml",
                              ("goal.loss.integrity_depth_mm=3", "goal.loss.fold=true", *overrides))
    found = mechanism(values.pop("mechanism"))
    values.pop("output")
    values, _ = found.split_settings(values)
    return found, found.config_from_mapping(values)


@pytest.mark.parametrize("name", list(FEASIBLE))
def test_every_evaluator_keeps_the_integrity_rules_length(name):
    found, config = integrity_config(name)
    goal = getattr(config, "goal", config)
    assert goal.integrity_depth_mm == 3.0
    evaluation = found.evaluate(FEASIBLE[name], config)
    assert evaluation.feasible
    phi = np.arange(0.0, np.degrees(goal.required_rom_rad) + 1e-9, 1.0)
    kept = integrity_free(scene(name, config, FEASIBLE[name], phi), 3.0)
    assert evaluation.proximal_usable_length_mm == pytest.approx(kept["proximal_pad"], abs=1.0)
    assert evaluation.distal_usable_length_mm == pytest.approx(kept["distal_pad"], abs=1.0)
    # The fold alone (no depth) keeps at least as much.
    _, fold = integrity_config(name, "goal.loss.integrity_depth_mm=null")
    plain = found.evaluate(FEASIBLE[name], fold)
    assert evaluation.proximal_usable_length_mm <= plain.proximal_usable_length_mm + 0.5
    assert evaluation.distal_usable_length_mm <= plain.distal_usable_length_mm + 0.5


def test_the_probe_meets_what_stands_within_its_depth():
    from core.morphology.linkage.packaging import probe_blocked

    face = np.array([[[0.0, 0.0], [10.0, 0.0]]] * 2)
    normal = np.array([0.0, -1.0])
    square = np.array([[3.0, -2.0], [5.0, -2.0], [5.0, -4.0], [3.0, -4.0]])  # 2 mm in front of x 3-5
    bar = np.array([[7.0, -1.0], [9.0, -1.5]])                              # 1-1.5 mm in front of x 7-9
    disk = (np.array([1.0, -3.0]), 0.5)                                     # 2.5-3 mm in front of x 0.5-1.5
    # The bar from x 7 to 7.8 within 1.2 mm, all of it within 1.6; at 2.6 the
    # square too and the disk where |x - 1| <= 0.3.
    for depth, share in ((0.5, 0.0), (1.2, 0.08), (1.6, 0.2), (2.6, 0.46)):
        length, blocked = probe_blocked(face, normal, depth, [square], [bar], [disk], step_mm=0.1)
        assert length == pytest.approx(10.0)
        assert blocked.mean(axis=1) == pytest.approx(share, abs=0.011)
    # A point inside a solid is blocked at any depth; one behind the face is not.
    _, blocked = probe_blocked(face, normal, 0.0, [square - (0.0, -3.0)], step_mm=0.1)
    assert blocked.mean() == pytest.approx(0.2, abs=0.011)
    _, blocked = probe_blocked(face, normal, 5.0, [square * (1.0, -1.0)], step_mm=0.1)
    assert not blocked.any()


def test_the_thickness_spans_the_fingers_own_solids():
    from core.optimization.analysis.loss import thickness_mm

    found, config = mechanism("revolute_idealized"), config_of("revolute_idealized")
    x = DESIGNS["revolute_idealized"]
    built = scene("revolute_idealized", config, x, [0.0])
    dorsal, palmar = thickness_mm(built)
    names = found.variable_names
    drops = max(x[names.index("proximal_drop_mm")], x[names.index("distal_drop_mm")])
    # Palmar: the lower pad's face; dorsal: A's pin housing or the distal pad's
    # carrier on its top face (no servo; the MCP servo in the palm).
    packaging = config.goal.packaging
    assert palmar == pytest.approx(-(drops + 5.0))
    assert dorsal == pytest.approx(max(packaging.pivot_radius_mm("a"), packaging.distal_carrier_height_mm
                                       - x[names.index("distal_drop_mm")]))
    dorsal_servo, _ = thickness_mm(scene("revolute", config_of("revolute"), DESIGNS["revolute"], [0.0]))
    assert dorsal_servo > dorsal + 10.0
