"""SSoT mathematical verification for set-based Pad Coverage, linkage occlusion
policies, context defaults, material-coordinate intersection, and empirical behavior."""

from __future__ import annotations

import dataclasses
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "scripts" / "study") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts" / "study"))

import shared  # noqa: E402

from core.morphology.coverage import (  # noqa: E402
    PROFILE_DEFAULT_LINKAGE_OCCLUSION,
    IntervalSet,
    accessible_face_sets,
    evaluate_pad_coverage,
    face_occluded_set,
    resolve_linkage_occlusion,
)
from core.optimization.canonical_mapping import morphology_from_design  # noqa: E402
from core.optimization.linkage.mechanism import resolve_preset  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402
from core.optimization.study import load_yaml  # noqa: E402

BALANCED_OCCLUDED = (27.2, 89.9, 0.0, 36.3, 11.1, 65.7, 10.1, 10.0, 0.1, 0.0, 8.3, 12.3, 3.0, 3.0)
"""CoDe candidate whose coupler A'-B and driver A-B' cross in front of the contact faces at 120 deg."""


def _study_linkage_config():
    link_mech = mechanism("linkage")
    values = read_config_file(ROOT / "scripts" / "study" / "code_tilt10.yaml")
    values.pop("mechanism")
    values.pop("output")
    resolve_preset(values)
    values, _ = link_mech.split_settings(values)
    return link_mech.config_from_mapping(values)


def test_context_profile_defaults_are_explicit_and_unambiguous():
    assert PROFILE_DEFAULT_LINKAGE_OCCLUSION == {"study": False, "prototype": True}
    assert resolve_linkage_occlusion(None, profile="study") is False
    assert resolve_linkage_occlusion(None, profile="prototype") is True
    assert resolve_linkage_occlusion(True, profile="study") is True
    assert resolve_linkage_occlusion(False, profile="prototype") is False
    with pytest.raises(ValueError, match="profile must be one of"):
        resolve_linkage_occlusion(None, profile="global")  # type: ignore[arg-type]

    study_cfg = _study_linkage_config()
    assert study_cfg.profile == "study" and study_cfg.include_linkage_occlusion is False
    proto_cfg = load_yaml(mechanism("linkage"), ROOT / "scripts" / "optimize" / "linkage" / "config.yaml")[0]
    assert proto_cfg.profile == "prototype" and proto_cfg.include_linkage_occlusion is True


def test_disconnected_interval_sets_match_independent_fine_riemann_quadrature():
    # Horizontal face from (0, -5) to (40, -5) with palmar normal (0, -1)
    face = np.array(((0.0, -5.0), (40.0, -5.0)), dtype=float)
    normal = np.array((0.0, -1.0), dtype=float)
    # Two disjoint occluders strictly in front of the face (z < -5):
    # 1) Segment from (8, -6) to (14, -8) -> occludes [8, 14]
    # 2) Disk centered at (28, -4) with radius 3 -> in front (z <= -5) over |x - 28| <= sqrt(9 - 1) = sqrt(8)
    segments = [(np.array((8.0, -6.0)), np.array((14.0, -8.0)))]
    disks = [(np.array((28.0, -4.0)), 3.0)]

    occ = face_occluded_set(face, normal, segments=segments, disks=disks)
    usable = IntervalSet.closed(0.0, 40.0).difference(occ)
    # Usable face consists of THREE disconnected intervals: [0, 8], [14, 28 - sqrt(8)], [28 + sqrt(8), 40]
    assert len(usable.intervals) == 3

    # Independent fine Riemann quadrature on 400,000 midpoint samples along xi in [0, 40]
    n_quad = 400_000
    xi = (np.arange(n_quad) + 0.5) * (40.0 / n_quad)
    # Ray from (xi, -5) in direction (0, -1): blocked if x in [8, 14] or (x - 28)^2 + (z + 4)^2 <= 9 for some z <= -5
    seg_hit = (xi >= 8.0) & (xi <= 14.0)
    disk_hit = (xi - 28.0) ** 2 + (-5.0 - (-4.0)) ** 2 <= 9.0
    quad_usable_mm = 40.0 * float((~(seg_hit | disk_hit)).mean())

    exact_expected = 40.0 - 6.0 - 2.0 * math.sqrt(8.0)
    assert usable.measure == pytest.approx(exact_expected, abs=1e-12)
    assert usable.measure == pytest.approx(quad_usable_mm, abs=2e-4)


def test_coverage_with_occlusion_on_off_monotonicity_and_collision_independence():
    link_mech = mechanism("linkage")
    cfg_off = _study_linkage_config()
    cfg_on = dataclasses.replace(cfg_off, include_linkage_occlusion=True)

    morph = morphology_from_design("linkage", BALANCED_OCCLUDED, cfg_off)
    phi_req = cfg_off.required_rom_rad

    cov_off = evaluate_pad_coverage(morph, phi_req, include_linkage_occlusion=False)
    cov_on = evaluate_pad_coverage(morph, phi_req, include_linkage_occlusion=True)

    # Enabling linkage occlusion strictly reduces coverage on this occluded design and never increases any interval
    assert cov_on.proximal_endpoint_effective_mm < cov_off.proximal_endpoint_effective_mm
    assert cov_on.distal_endpoint_effective_mm < cov_off.distal_endpoint_effective_mm
    assert cov_on.endpoint_coverage < cov_off.endpoint_coverage
    assert cov_on.motion_robust_coverage <= cov_off.motion_robust_coverage + 1e-12

    # SSoT analytical evaluator agrees with fine-grid Mechanism.evaluate() as servo_step -> 0
    ev_off = link_mech.evaluate(BALANCED_OCCLUDED, link_mech.refine(cfg_off, math.radians(0.01)))
    ev_on = link_mech.evaluate(BALANCED_OCCLUDED, link_mech.refine(cfg_on, math.radians(0.01)))
    assert ev_off.pad_coverage == pytest.approx(cov_off.endpoint_coverage, abs=2e-4)
    assert ev_on.pad_coverage == pytest.approx(cov_on.endpoint_coverage, abs=2e-4)

    # Also check coarse-grid collision/ROM invariance under toggling include_linkage_occlusion
    ev_off = link_mech.evaluate(BALANCED_OCCLUDED, cfg_off)
    ev_on = link_mech.evaluate(BALANCED_OCCLUDED, cfg_on)

    # Toggle must NOT alter collision clearances, ROM, hyperextension, or structural support constraints!
    for cname in ev_off.constraints:
        if cname == "pad_usable_mm":
            continue
        assert ev_on.constraints[cname] == pytest.approx(ev_off.constraints[cname], abs=1e-12), cname
    assert ev_on.pad_clearance_mm == pytest.approx(ev_off.pad_clearance_mm, abs=1e-12)
    assert ev_on.phi_max_rad == pytest.approx(ev_off.phi_max_rad, abs=1e-12)


def test_unoccluded_faces_produce_identical_coverage_for_on_and_off():
    refs = {r.reference_id: r for r in shared.load_references()}
    # 1) Architectures without crossing links/belts (2R, RCJ) and 2) unoccluded CoDe & Belt-2R poses
    for rid in ("2r_dd_tilt90_matched", "rcj_tilt90_matched", "belt_2r_matched"):
        ref = refs[rid]
        morph = morphology_from_design(ref.mechanism, ref.x, ref.config)
        c_off = evaluate_pad_coverage(morph, math.radians(120.0), include_linkage_occlusion=False)
        c_on = evaluate_pad_coverage(morph, math.radians(120.0), include_linkage_occlusion=True)
        assert c_on.endpoint_sets == c_off.endpoint_sets
        assert c_on.robust_sets == c_off.robust_sets
        assert c_on.endpoint_coverage == c_off.endpoint_coverage
        assert c_on.motion_robust_coverage == c_off.motion_robust_coverage

    # Extended pose (phi = 0) of CoDe: links lie dorsal of contact faces -> ON == OFF identically
    code_ref = refs["code_matched"]
    code_morph = morphology_from_design("linkage", code_ref.x, code_ref.config)
    s_off = accessible_face_sets(code_morph, 0.0, include_linkage_occlusion=False)
    s_on = accessible_face_sets(code_morph, 0.0, include_linkage_occlusion=True)
    assert s_on == s_off


@pytest.mark.parametrize("reference", shared.load_references(), ids=lambda r: r.reference_id)
def test_endpoint_and_motion_robust_material_coordinate_bounds_and_consistency(reference):
    morph = morphology_from_design(reference.mechanism, reference.x, reference.config)
    phi_req = math.radians(120.0)
    for occ in (False, True):
        cov = evaluate_pad_coverage(morph, phi_req, include_linkage_occlusion=occ)
        # Material coordinates lie within [0, l_i]
        for side, l_mount in (("proximal_pad", cov.proximal_mounted_mm), ("distal_pad", cov.distal_mounted_mm)):
            for iv_set in (cov.endpoint_sets[side], cov.robust_sets[side]):
                for lo, hi in iv_set.intervals:
                    assert -1e-12 <= lo <= hi <= l_mount + 1e-12
            # Robust set is an intersection including phi_req, so U_rob subseteq U_endpoint!
            assert cov.robust_sets[side].difference(cov.endpoint_sets[side]).is_empty
            assert cov.robust_sets[side].measure <= cov.endpoint_sets[side].measure + 1e-12

        assert 0.0 <= cov.motion_robust_coverage <= cov.endpoint_coverage <= cov.mounted_coverage + 1e-12
        # Monotonicity wrt occlusion
        if occ:
            cov_off = evaluate_pad_coverage(morph, phi_req, include_linkage_occlusion=False)
            assert cov.endpoint_coverage <= cov_off.endpoint_coverage + 1e-12
            assert cov.motion_robust_coverage <= cov_off.motion_robust_coverage + 1e-12
