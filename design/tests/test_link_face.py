"""Linkage occlusion in the coverage (``goal.loss.include_linkage_occlusion``):
a crossing link (A-B' or A'-B) of the CFL, or the 2R-remote's belt and
pulleys, hides the part of a face it passes in front of, before the
cylinder rule pairs the two faces. Defaults: ``profile: study`` off,
``profile: prototype`` on."""

from __future__ import annotations

import dataclasses
from pathlib import Path

import numpy as np
import pytest

from core.optimization.linkage.mechanism import resolve_preset
from core.optimization.mechanism import mechanism, read_config_file
from core.optimization.study import load_yaml

ROOT = Path(__file__).resolve().parents[1]
LINKAGE = mechanism("linkage")
BALANCED = (27.2, 89.9, 0.0, 36.3, 11.1, 65.7, 10.1, 10.0, 0.1, 0.0, 8.3, 12.3, 3.0, 3.0)
"""The CFL's (tilt 10) balanced seed before the rule (B on C): at 120 deg the coupler
runs from A' down past the proximal pad's inner end and across the distal face."""


def study():
    values = read_config_file(ROOT / "scripts" / "study" / "cfl.yaml",
                              ("goal.hardware.pip_servo.tilt_angle_deg=[10, 10]",))
    values.pop("mechanism"), values.pop("output")
    resolve_preset(values)
    values, _ = LINKAGE.split_settings(values)
    return LINKAGE.config_from_mapping(values)


def test_profile_defaults_study_false_and_prototype_true():
    study_cfg = study()
    assert study_cfg.profile == "study"
    assert not study_cfg.include_linkage_occlusion
    production = load_yaml(LINKAGE, ROOT / "scripts" / "optimize" / "prototype" / "config.yaml", ())[0]
    assert production.profile == "prototype"
    assert production.include_linkage_occlusion


def test_a_link_in_front_of_a_face_takes_that_part_of_it():
    off = study()
    on = dataclasses.replace(off, include_linkage_occlusion=True)
    before, after = LINKAGE.evaluate(BALANCED, off), LINKAGE.evaluate(BALANCED, on)
    # The coupler runs across the distal face at 120 deg: the pads share less there.
    assert [pose.common_mm for pose in after.poses] <= [pose.common_mm for pose in before.poses]
    assert after.c_pad_mean < before.c_pad_mean
    assert after.feasible == before.feasible  # the coverage is the objective, not a constraint


def test_the_removed_loss_rules_are_errors():
    for removed in ("goal.loss.integrity_depth_mm=5", "goal.loss.fold=false"):
        with pytest.raises(ValueError, match="was removed"):
            load_yaml(LINKAGE, ROOT / "scripts" / "optimize" / "prototype" / "config.yaml", (removed,))


def test_a_pad_without_a_partner_counts_nothing():
    on = dataclasses.replace(study(), include_linkage_occlusion=True)
    e = LINKAGE.evaluate(BALANCED, on)
    for pose in e.poses:  # only the length both pads cover from the crossing counts
        assert pose.common_mm <= min(e.proximal_support_length_mm, e.distal_support_length_mm) + 1e-9


def test_the_belt_and_its_pulleys_in_front_of_a_face_take_that_part_of_it():
    import math

    from core.morphology.mechanism.revolute.solids import contact_faces
    from core.optimization.revolute_parallel.evaluate import belt_hidden, geometry

    belt = mechanism("revolute_parallel")
    config = load_yaml(belt, ROOT / "scripts" / "optimize" / "prototype" / "config.yaml",
                       ("goal.hardware.pulley_radius_mm=6", "goal.loss.include_linkage_occlusion=true",
                        "goal.collision.param.pivot_diameter_mm.a=6", "goal.collision.param.pivot_diameter_mm.b=6",
                        "goal.pads.proximal.height_mm=5", "goal.pads.distal.height_mm=5"))[0]
    # A at 45 mm, A' 15 mm above it; the proximal face 5 mm below OA, the pulley on A 6 mm round:
    # it stands in front of the face from 45 - sqrt(6^2 - 5^2) = 45 - sqrt(11) mm on.
    exact_start = 45.0 - math.sqrt(6.0**2 - 5.0**2)
    q = math.radians(120.0)
    for app_x, lost in ((41.0, 0.0), (42.0, 42.0 - exact_start)):
        geo = geometry([45.0, 45.0, app_x, 3.0, 0.0, 0.0, 10.0, 0.0, 15.0], config)
        proximal, proximal_normal, distal, distal_normal = contact_faces(geo.joint, np.array([q]))
        hidden = belt_hidden(geo)(q, proximal, proximal_normal, distal[0], distal_normal[0])
        assert hidden[0].measure == pytest.approx(lost, abs=1e-9)
        # The distal pad starts 3 mm past A: the pulley on A takes its first few tenths.
        assert 0.0 < hidden[1].measure < 1.0
