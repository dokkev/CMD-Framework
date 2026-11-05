"""The study's link rule: at the required flexion, a CoDe pad's face is lost
where a crossing link (the driver A-B' or the coupler A'-B) passes in front of
it (goal.loss.links_block_face)."""

from __future__ import annotations

import dataclasses
from pathlib import Path

import numpy as np
import pytest

from core.morphology.mechanism.linkage.design import decode
from core.morphology.mechanism.linkage.motion import sweep_motion
from core.morphology.mechanism.linkage.packaging import contact_faces, link_kept_lengths
from core.morphology.packaging.faces import crossing_free_lengths
from core.optimization.linkage.evaluate import motion_settings
from core.optimization.linkage.mechanism import resolve_preset
from core.optimization.mechanism import mechanism, read_config_file
from core.optimization.study import load_yaml

ROOT = Path(__file__).resolve().parents[1]
LINKAGE = mechanism("linkage")
BALANCED = (27.2, 89.9, 0.0, 36.3, 11.1, 65.7, 10.1, 10.0, 0.1, 0.0, 8.3, 12.3, 3.0, 3.0)
"""code_tilt10's balanced seed before the rule (B on C): at 120 deg the coupler
runs from A' down past the proximal pad's inner end and across the distal face."""


def study():
    values = read_config_file(ROOT / "scripts" / "study" / "code_tilt10.yaml")
    values.pop("mechanism"), values.pop("output")
    resolve_preset(values)
    values, _ = LINKAGE.split_settings(values)
    return LINKAGE.config_from_mapping(values)


def kept_at(config, x, phi_deg: float, driver: bool = True):
    """Each face's length, its fold-free length and what the links keep at
    the first servo sample reaching ``phi_deg`` (``driver`` false: the driver
    link moved far dorsal, the coupler alone)."""

    from core.morphology.mechanism.linkage.motion import servo_sense

    finger = decode(x, config.link_length_resolution_mm)
    sweep = finger.sweep_servo(servo_sense(finger, 1e-3) * np.radians(np.arange(0.0, 180.0, 0.25)))
    i = int(np.flatnonzero(np.degrees(sweep.total_flexion_rad) >= phi_deg - 1e-9)[0])
    b, bp, c = sweep.b[i:i + 1], sweep.bp[i:i + 1], sweep.tip[i:i + 1]
    faces = contact_faces(finger, config.packaging, b, c)
    free = crossing_free_lengths(faces[0], faces[2])
    length = {"proximal_pad": float(np.linalg.norm(faces[0][1] - faces[0][0])),
              "distal_pad": float(np.linalg.norm(faces[2][0, 1] - faces[2][0, 0]))}
    if not driver:  # the driver moved far dorsal, beyond any probe: the coupler alone
        from core.morphology.finger import PointXZ

        finger = dataclasses.replace(finger, joint=dataclasses.replace(finger.joint, a=PointXZ(1.0, 1e4)))
        bp = np.array(((2.0, 1e4),))
    kept = link_kept_lengths(finger, b, bp, faces, free)
    return length, free, kept


def test_the_study_counts_the_links_and_production_does_not():
    assert study().links_block_face
    production = load_yaml(LINKAGE, ROOT / "scripts" / "optimize" / "linkage" / "config.yaml", ())[0]
    assert not production.links_block_face


def test_a_link_in_front_of_a_face_takes_that_part_of_it():
    on = study()
    off = dataclasses.replace(on, links_block_face=False)
    before, after = LINKAGE.evaluate(BALANCED, off), LINKAGE.evaluate(BALANCED, on)
    # The coupler takes most of the distal face, the driver A-B' the proximal face's inner 10 mm.
    assert after.distal_usable_length_mm < 20.0 < before.distal_usable_length_mm
    assert after.proximal_usable_length_mm == pytest.approx(before.proximal_usable_length_mm - 9.9, abs=1.0)
    assert after.pad_coverage < before.pad_coverage
    assert after.constraints["pad_usable_mm"] > 0.0  # its 25 mm distal module no longer fits


def test_the_driver_link_counts_as_the_coupler_does():
    config = study()
    length, free, both = kept_at(config, BALANCED, 120.0)
    _, _, coupler = kept_at(config, BALANCED, 120.0, driver=False)
    for side in free:  # each link only takes: the two together keep at most what the coupler alone does
        assert both[side][0] <= coupler[side][0] + 1e-9 <= min(free[side][0], length[side]) + 1e-9
    # The driver runs under the proximal face's inner end: it, not the coupler, takes that strip.
    assert both["proximal_pad"][0] < coupler["proximal_pad"][0] - 5.0


def test_links_behind_the_faces_take_nothing():
    config = study()
    length, free, kept = kept_at(config, BALANCED, 0.0)  # extended: both links dorsal of both faces
    for side in free:
        assert kept[side][0] == pytest.approx(min(free[side][0], length[side]), abs=1e-6)


def test_the_rule_needs_the_links_bent_around_the_pads_and_no_integrity_depth():
    config = study()
    finger = decode(BALANCED, config.link_length_resolution_mm)
    packaging = dataclasses.replace(config.packaging, links_clear_pads=False)
    with pytest.raises(ValueError, match="links_clear_pads"):
        sweep_motion(finger, packaging, motion_settings(config), config.required_rom_rad, links_block_face=True)
    with pytest.raises(ValueError, match="integrity_depth_mm"):
        load_yaml(LINKAGE, ROOT / "scripts" / "optimize" / "linkage" / "config.yaml",
                  ("goal.loss.links_block_face=true", "goal.loss.integrity_depth_mm=5"))


def test_a_pad_past_where_it_counts_is_left_out_of_the_coverage_not_forbidden():
    on = study()
    assert "pad_waste_mm" not in LINKAGE.constraint_names
    e = LINKAGE.evaluate(BALANCED, on)
    # The distal pad mounted past what the coupler leaves: that part counts for nothing, nothing more.
    assert e.distal_support_length_mm > e.distal_usable_length_mm
    assert e.distal_effective_mm == pytest.approx(e.distal_usable_length_mm)
