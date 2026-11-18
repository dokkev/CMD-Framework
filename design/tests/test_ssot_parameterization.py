"""SSoT mathematical verification for planar SE(2) frames, canonical morphology
parameterization, parameter dependencies, kinematic closure, and derived properties."""

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

from core.geometry import Transform2D  # noqa: E402
from core.morphology.canonical import (  # noqa: E402
    ActuationParams,
    ActuatorSpec,
    CoDeMechanismParams,
    ContactParams,
    FingerMorphology,
    MorphologyError,
    RcjMechanismParams,
    RevoluteMechanismParams,
    StructureParams,
)
from core.optimization.canonical_mapping import (  # noqa: E402
    VARIABLE_SLOT_MAPPING,
    design_from_morphology,
    morphology_from_design,
)
from core.optimization.mechanism import mechanism  # noqa: E402
from core.optimization.study import load_run  # noqa: E402

# ---------------------------------------------------------------------------
# 1. Planar SE(2) Lie-group axioms and coordinate conventions
# ---------------------------------------------------------------------------


def test_se2_group_axioms_and_matrix_isomorphism():
    t1 = Transform2D(12.5, -4.2, 0.73)
    t2 = Transform2D(-3.1, 8.9, -1.15)
    t3 = Transform2D(5.0, 2.5, 2.40)
    ident = Transform2D.identity()

    # Identity & inverse
    for t in (t1, t2, t3):
        np.testing.assert_allclose((t @ ident).matrix(), t.matrix(), atol=1e-13)
        np.testing.assert_allclose((ident @ t).matrix(), t.matrix(), atol=1e-13)
        np.testing.assert_allclose((t @ t.inverse()).matrix(), np.eye(3), atol=1e-13)
        np.testing.assert_allclose((t.inverse() @ t).matrix(), np.eye(3), atol=1e-13)

    # Associativity & matrix homomorphism
    left = (t1 @ t2) @ t3
    right = t1 @ (t2 @ t3)
    np.testing.assert_allclose(left.matrix(), right.matrix(), atol=1e-13)
    np.testing.assert_allclose((t1 @ t2).matrix(), t1.matrix() @ t2.matrix(), atol=1e-13)

    # Round-trip from 3x3 homogeneous matrix
    rebuilt = Transform2D.from_matrix(left.matrix())
    np.testing.assert_allclose(rebuilt.matrix(), left.matrix(), atol=1e-13)


def test_flexion_sign_convention_turns_distal_toward_palmar():
    # +X is distal (1, 0), +Z is dorsal (0, 1), -Z is palmar (0, -1).
    # Standard CCW rotation R(+pi/2) sends +X -> +Z (dorsal).
    np.testing.assert_allclose(Transform2D.rot(0.5 * math.pi).apply_vector((1.0, 0.0)), (0.0, 1.0), atol=1e-15)
    # Palmar flexion phi = +pi/2 sends +X -> -Z (palmar), i.e., R_flex(phi) = R(-phi).
    flex90 = Transform2D.flexion(0.5 * math.pi)
    np.testing.assert_allclose(flex90.apply_vector((1.0, 0.0)), (0.0, -1.0), atol=1e-15)
    assert flex90.ori == pytest.approx(-0.5 * math.pi)


# ---------------------------------------------------------------------------
# 2. Architecture-specific DOFs, charts, and dependency enforcement
# ---------------------------------------------------------------------------


def test_architecture_specific_mechanism_dofs_and_chart_roundtrips():
    code = CoDeMechanismParams.from_extended_pivots(
        a_xz=(35.7, 0.0), ap_xz=(37.5, 12.4), b0_xz=(67.2, 0.0), bp0_xz=(60.5, 6.5)
    )
    assert code.dim == 8
    assert CoDeMechanismParams.from_coordinates(code.coordinates) == code

    # Intrinsic machined bar-length chart <-> extended-pivot chart bijection
    abp, apb = code.bar_lengths_mm
    u = (code.t_a.pos_array - code.t_bp_local.pos_array) - code.t_ap.pos_array
    cross = u[0] * (code.t_b0.z_mm - code.t_ap.z_mm) - u[1] * (code.t_b0.x_mm - code.t_ap.x_mm)
    branch = 1 if cross >= 0.0 else -1
    from_bars = CoDeMechanismParams.from_bar_lengths(
        code.t_a.pos, code.t_ap.pos, code.t_bp_local.pos, abp, apb, branch=branch
    )
    np.testing.assert_allclose(from_bars.coordinates, code.coordinates, atol=1e-11)

    rev = RevoluteMechanismParams.from_coordinates((55.0,))
    assert rev.dim == 1
    assert rev.coordinates == (55.0,)

    rcj = RcjMechanismParams.from_coordinates((48.0, 4.5))
    assert rcj.dim == 2
    assert rcj.coordinates == (48.0, 4.5)
    assert rcj.distal_origin_extended == pytest.approx((57.0, 0.0))


def test_actuator_pose_orientation_and_shaft_alignment_dependencies():
    code = CoDeMechanismParams.from_extended_pivots(
        a_xz=(35.7, 0.0), ap_xz=(37.5, 12.4), b0_xz=(67.2, 0.0), bp0_xz=(60.5, 6.5)
    )
    tilt = math.radians(15.0)
    act = ActuationParams.from_minimal("CoDe", code, pip_tilt_rad=tilt)
    # Shaft coincides with A' by construction
    assert act.pip.pose.pos == pytest.approx((37.5, 12.4))
    # Dorsal tilt psi from -X toward +Z gives rear = (-cos psi, sin psi) and ori(T_2) = -psi
    assert act.pip.pose.ori == pytest.approx(-tilt)
    np.testing.assert_allclose(act.pip.rear_direction, (-math.cos(tilt), math.sin(tilt)), atol=1e-14)

    struct = StructureParams.from_axial(43.3, -2.5, 22.8)
    contact = ContactParams.outer_pinned(
        struct,
        proximal_length_mm=20.0,
        proximal_height_mm=3.0,
        distal_length_mm=20.0,
        distal_height_mm=3.0,
        proximal_outer_start_mm=9.5,
        proximal_mount_offset_mm=3.0,
        distal_mount_offset_mm=3.0,
    )
    morph = FingerMorphology("CoDe", code, act, struct, contact)
    morph.validate()

    # Violating shaft alignment must fail Layer 2 admissibility
    misaligned_act = dataclasses.replace(
        act, pip=ActuatorSpec(act.pip.footprint, Transform2D(38.0, 12.4, -tilt))
    )
    bad_shaft = dataclasses.replace(morph, act=misaligned_act)
    with pytest.raises(MorphologyError, match="align with pivot A'"):
        bad_shaft.validate()

    # Violating structural support containment must fail Layer 2 admissibility when required
    overhung_contact = ContactParams.outer_pinned(
        struct,
        proximal_length_mm=40.0,  # 9.5 + 40 = 49.5 > s_{p,end} = 43.3
        proximal_height_mm=3.0,
        distal_length_mm=20.0,
        distal_height_mm=3.0,
        proximal_outer_start_mm=9.5,
    )
    bad_support = dataclasses.replace(morph, contact=overhung_contact)
    with pytest.raises(MorphologyError, match="proximal contact span"):
        bad_support.validate(require_support_containment=True)
    # Without containment requirement, geometry remains kinematically well-formed
    bad_support.validate(require_support_containment=False)


# ---------------------------------------------------------------------------
# 3. Kinematic closure, analytical transmission ratio, and frame hierarchy
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("reference", shared.load_references(), ids=lambda r: r.reference_id)
def test_canonical_mapping_roundtrip_and_derived_properties_across_architectures(reference):
    # Exact round-trip prior to prototyping 0.1 mm bar-length rounding
    morph_exact = morphology_from_design(
        reference.mechanism, reference.x, reference.config, apply_grid_assembly=False
    )
    np.testing.assert_allclose(
        design_from_morphology(morph_exact, reference.mechanism), reference.x, atol=1e-6
    )

    # Assembled morphology (including 0.1 mm linkage bar re-assembly) is idempotent and matches Study 2 E(M)
    morph = morphology_from_design(reference.mechanism, reference.x, reference.config)
    morph.validate(require_support_containment=True)
    re_encoded = design_from_morphology(morph, reference.mechanism)
    morph_redecoded = morphology_from_design(
        reference.mechanism, re_encoded, reference.config, apply_grid_assembly=False
    )
    np.testing.assert_allclose(
        design_from_morphology(morph_redecoded, reference.mechanism), re_encoded, atol=1e-6
    )

    # Check common finger geometry L(M): exact chart matches Study 2 E(M), assembled chart matches evaluate()
    part = shared.partitions.partition(reference.mechanism)
    ext = part.external(reference.x, reference.config)
    cg_exact = morph_exact.common_geometry
    assert cg_exact.finger_length_mm == pytest.approx(ext["finger_length_mm"], abs=1e-6)
    assert cg_exact.proximal_length_mm == pytest.approx(ext["proximal_span_mm"], abs=1e-6)
    assert cg_exact.distal_length_mm == pytest.approx(ext["distal_span_mm"], abs=1e-6)
    assert morph_exact.icr0_x_mm == pytest.approx(ext["articulation_mm"], abs=1e-9)

    ev = mechanism(reference.mechanism).evaluate(reference.x, reference.config)
    assert ev is not None
    assert morph.common_geometry.finger_length_mm == pytest.approx(ev.finger_length_mm, abs=1e-9)

    # Unified Finger.kinematics() works identically across FourBar, Pin, and RollingJoint
    finger = morph.to_finger()
    phis = np.radians([0.0, 30.0, 60.0, 90.0, 120.0])
    kin = finger.kinematics(phis)
    assert len(kin.tip) == len(phis)
    for idx, phi in enumerate(phis):
        t_wd = morph.frame_pose("D", float(phi), q_mcp_rad=0.0)
        tip_expected = t_wd.apply_point((morph.struct.s_d_end_mm, 0.0))
        np.testing.assert_allclose(kin.tip[idx], tip_expected, atol=1e-9)

    # Frame hierarchy composition: T_WD(q_MCP, phi) == T_WP(q_MCP) @ T_PD(phi)
    for q_mcp in (0.0, 0.4, -0.2):
        for phi in (0.0, 0.8, 1.9):
            lhs = morph.frame_pose("D", phi, q_mcp).matrix()
            rhs = (morph.frame_pose("P", phi, q_mcp) @ morph.distal_pose(phi)).matrix()
            np.testing.assert_allclose(lhs, rhs, atol=1e-13)


def test_analytical_transmission_ratio_matches_independent_finite_difference():
    refs = {r.reference_id: r for r in shared.load_references()}

    # 1. CoDe four-bar: compare implicit differentiation against central FD of theta_servo(phi)
    code_morph = morphology_from_design("linkage", refs["code_matched"].x, refs["code_matched"].config)
    assert isinstance(code_morph.mech, CoDeMechanismParams)
    fb = code_morph.mech.four_bar
    h = 1e-6
    for phi_deg in (0.0, 25.0, 60.0, 95.0, 120.0):
        phi = math.radians(phi_deg)
        p_plus = fb.solve(phi + h)
        p_minus = fb.solve(phi - h)
        # Servo shaft at A' drives link A'-B; angle of (B - A') in {P}
        th_plus = math.atan2(p_plus.b.z_mm - fb.ap.z_mm, p_plus.b.x_mm - fb.ap.x_mm)
        th_minus = math.atan2(p_minus.b.z_mm - fb.ap.z_mm, p_minus.b.x_mm - fb.ap.x_mm)
        dtheta_dphi = math.remainder(th_plus - th_minus, 2.0 * math.pi) / (2.0 * h)
        fd_k = abs(1.0 / dtheta_dphi)
        assert code_morph.transmission_ratio(phi) == pytest.approx(fd_k, rel=1e-7, abs=1e-7)

    # 2. 2R & 2R-remote: exact unit ratio k = 1
    for rid in ("2r_dd_tilt0_matched", "belt_2r_matched"):
        m = morphology_from_design(refs[rid].mechanism, refs[rid].x, refs[rid].config)
        for phi in (0.0, 1.0, 2.0):
            assert m.transmission_ratio(phi) == pytest.approx(1.0, abs=1e-15)

    # 3. Symmetric 1:1 no-slip RCJ: exact ratio k = 2; verify rolling no-slip kinematics independently
    rcj_morph = morphology_from_design("rcj", refs["rcj_tilt0_matched"].x, refs["rcj_tilt0_matched"].config)
    assert isinstance(rcj_morph.mech, RcjMechanismParams)
    r = rcj_morph.mech.roller_radius_mm
    x_a = rcj_morph.mech.t_a.x_mm
    for phi in (0.0, 0.5, 1.2, 2.0):
        assert rcj_morph.transmission_ratio(phi) == pytest.approx(2.0, abs=1e-15)
        # Independent rolling contact geometry: line of centers rotates by phi/2
        b_indep = (x_a + 2.0 * r * math.cos(0.5 * phi), -2.0 * r * math.sin(0.5 * phi))
        icr_indep = (x_a + r * math.cos(0.5 * phi), -r * math.sin(0.5 * phi))
        t_pd = rcj_morph.distal_pose(phi)
        assert t_pd.pos == pytest.approx(b_indep, abs=1e-12)
        icr = rcj_morph.icr(phi)
        assert icr is not None
        assert (icr.x_mm, icr.z_mm) == pytest.approx(icr_indep, abs=1e-12)


def test_variable_slot_mapping_covers_every_mechanism_variable_order_and_legacy_json():
    for mech_key, expected_slots in VARIABLE_SLOT_MAPPING.items():
        mech = mechanism(mech_key)
        assert tuple(s.name for s in expected_slots) == tuple(mech.variable_names)

    # Verify legacy run.json with 'links_block_face' deserializes cleanly into LinkageConfig
    archived = ROOT / "output" / "study" / "linkage" / "code_tilt10" / "run_001"
    if (archived / "run.json").exists():
        loaded = load_run(archived)
        assert loaded.config.profile == "study"
        assert isinstance(loaded.config.include_linkage_occlusion, bool)
