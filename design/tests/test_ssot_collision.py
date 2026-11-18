"""SSoT mathematical verification for static and dynamic collision formulations."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "scripts" / "study") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts" / "study"))

import shared  # noqa: E402

from core.morphology.canonical import (  # noqa: E402
    ContactModuleSpec,
    ContactParams,
)
from core.morphology.collision import (  # noqa: E402
    disk_disk_clearance_mm,
    disk_servo_clearance_mm,
    dynamic_checkpoints_deg,
    evaluate_dynamic_collision,
    evaluate_static_collision,
    pad_pad_clearance_mm,
    point_rect_distance_mm,
    polygon_polygon_clearance_mm,
    polygon_servo_clearance_mm,
    rect_disk_clearance_mm,
    verify_dynamic_collision_refined,
)
from core.morphology.packaging.shapes import ServoFootprint  # noqa: E402
from core.optimization.canonical_mapping import morphology_from_design  # noqa: E402


def _independent_rect_boundary_points(box: tuple[float, float, float, float], n_edge: int = 2000) -> np.ndarray:
    """Independent dense sampling of an axis-aligned rectangle boundary (no production helpers)."""

    u0, u1, v0, v1 = box
    t = np.linspace(0.0, 1.0, n_edge)
    bottom = np.column_stack((u0 + t * (u1 - u0), np.full_like(t, v0)))
    right = np.column_stack((np.full_like(t, u1), v0 + t * (v1 - v0)))
    top = np.column_stack((u1 - t * (u1 - u0), np.full_like(t, v1)))
    left = np.column_stack((np.full_like(t, u0), v1 - t * (v1 - v0)))
    return np.vstack((bottom, right, top, left))


def _independent_servo_points(servo: ServoFootprint, n_arc: int = 2000, n_edge: int = 1000) -> np.ndarray:
    """Independent boundary + interior sampling of the D-shaped stadium-rect servo body."""

    s = np.asarray(servo.pivot, dtype=float)
    rear = np.asarray(servo.rear, dtype=float)
    rear = rear / np.linalg.norm(rear)
    fwd = -rear
    perp = np.array((-fwd[1], fwd[0]), dtype=float)
    r = servo.radius_mm
    b = servo.rear_extent_mm

    # Front semicircular arc: angle in [-pi/2, +pi/2] around fwd
    angles = np.linspace(-0.5 * math.pi, 0.5 * math.pi, n_arc)
    arc = s + r * (np.cos(angles)[:, None] * fwd + np.sin(angles)[:, None] * perp)
    # Rear rectangle edges: u in [-b, 0], v in [-r, r] in (fwd, perp) basis
    u = np.linspace(-b, 0.0, n_edge)
    v = np.linspace(-r, r, n_edge)
    top = s + u[:, None] * fwd + r * perp
    bot = s + u[:, None] * fwd - r * perp
    back = s - b * fwd + v[:, None] * perp
    return np.vstack((arc, top, bot, back))


def test_analytical_rect_disk_and_disk_disk_clearances_match_independent_geometry():
    box = (10.0, 32.0, -8.0, -3.0)
    boundary = _independent_rect_boundary_points(box, n_edge=4000)

    test_cases = [
        ((5.0, -5.0), 3.0),     # strictly left of face -> dist = 5.0, clr = 2.0
        ((20.0, 0.0), 3.0),     # strictly above top -> dist = 3.0, clr = 0.0 (touching!)
        ((35.0, 1.0), 2.5),     # diagonal corner -> dist = hypot(3, 4) = 5.0, clr = 2.5
        ((20.0, -5.0), 4.0),    # center strictly inside rectangle -> dist = 0.0, clr = -4.0
    ]
    for center, radius in test_cases:
        cx, cz = center
        inside = (box[0] <= cx <= box[1]) and (box[2] <= cz <= box[3])
        indep_dist = 0.0 if inside else float(np.linalg.norm(boundary - np.array(center), axis=1).min())
        assert point_rect_distance_mm(box, center) == pytest.approx(indep_dist, abs=1e-5)
        assert rect_disk_clearance_mm(box, center, radius) == pytest.approx(indep_dist - radius, abs=1e-5)

    assert disk_disk_clearance_mm((0.0, 0.0), 3.0, (10.0, 0.0), 5.0) == pytest.approx(2.0, abs=1e-14)
    assert disk_disk_clearance_mm((0.0, 0.0), 5.0, (8.0, 0.0), 3.0) == pytest.approx(0.0, abs=1e-14)
    assert disk_disk_clearance_mm((0.0, 0.0), 5.0, (6.0, 0.0), 3.0) == pytest.approx(-2.0, abs=1e-14)


def test_analytical_disk_servo_clearance_matches_independent_boundary_reference():
    servo = ServoFootprint((37.5, 12.4), (-math.cos(math.radians(15.0)), math.sin(math.radians(15.0))), 9.5, 24.5)
    pts = _independent_servo_points(servo)

    # Exterior points around front cap, side flanks, and rear corners
    for center, radius in [
        ((52.0, 10.0), 3.0),
        ((35.7, 0.0), 3.0),
        ((10.0, 25.0), 4.0),
        ((20.0, 2.0), 2.5),
    ]:
        indep_dist = float(np.linalg.norm(pts - np.array(center), axis=1).min())
        assert disk_servo_clearance_mm(center, radius, servo) == pytest.approx(indep_dist - radius, abs=1e-4)

    # Point at shaft center is strictly inside the servo -> distance is 0, clearance is -radius
    assert disk_servo_clearance_mm((37.5, 12.4), 3.0, servo) == pytest.approx(-3.0, abs=1e-12)


def test_intra_body_collision_clearances_are_strictly_configuration_invariant():
    from core.optimization.analysis.tradeoff import with_modules

    refs = {r.reference_id: r for r in shared.load_references()}
    carried = with_modules(refs["code_matched"].config, shared.catalog()["baseline"].modules)
    morph = morphology_from_design("linkage", refs["code_matched"].x, carried)
    static = evaluate_static_collision(morph)
    assert static.feasible

    # Pose both proximal pad, proximal pins, and PIP servo under arbitrary world/proximal transforms
    pip_fp0 = morph.act.pip.to_servo_footprint()
    assert pip_fp0 is not None
    p_local = morph.contact.proximal.body_polygon()
    d_local = morph.contact.distal.body_polygon()

    for q_mcp in (-0.3, 0.0, 0.6, 1.1):
        for phi in (0.0, 0.5, 1.2, 2.0):
            t_wp = morph.frame_pose("P", phi, q_mcp)
            t_wd = morph.frame_pose("D", phi, q_mcp)
            p_posed = t_wp.apply_points(p_local)
            d_posed = t_wd.apply_points(d_local)

            # Proximal pad vs PIP servo (both rigidly fixed to {P})
            pip_posed = ServoFootprint(
                t_wp.apply_point(pip_fp0.pivot),
                t_wp.apply_vector(pip_fp0.rear),
                pip_fp0.radius_mm,
                pip_fp0.rear_extent_mm,
            )
            c_pip_pad = polygon_servo_clearance_mm(p_posed, pip_posed)
            assert c_pip_pad == pytest.approx(static.clearances_mm[("pip_servo", "proximal_pad")], abs=1e-11)

            # Distal pad vs distal pivot B (both rigidly fixed to {D})
            b_world = t_wd.apply_point((0.0, 0.0))
            # Independent polygon-to-point check via tiny square around b_world
            eps = 1e-7
            tiny = np.array([
                (b_world[0] - eps, b_world[1] - eps),
                (b_world[0] + eps, b_world[1] - eps),
                (b_world[0] + eps, b_world[1] + eps),
                (b_world[0] - eps, b_world[1] + eps),
            ])
            dist_b = polygon_polygon_clearance_mm(d_posed, tiny)
            assert (dist_b - morph.pivot_radii_mm["b"]) == pytest.approx(
                static.clearances_mm[("b_pin", "distal_pad")], abs=1e-5
            )


def test_dynamic_checkpoint_policy_formula_and_boundary_cases():
    # Below 60 deg: includes required angle itself
    assert dynamic_checkpoints_deg(45.0) == pytest.approx((45.0,))
    assert dynamic_checkpoints_deg(0.0) == pytest.approx((0.0,))
    # Exact multiples of 30 deg from 60 deg
    assert dynamic_checkpoints_deg(60.0) == pytest.approx((60.0,))
    assert dynamic_checkpoints_deg(90.0) == pytest.approx((60.0, 90.0))
    assert dynamic_checkpoints_deg(120.0) == pytest.approx((60.0, 90.0, 120.0))
    assert dynamic_checkpoints_deg(150.0) == pytest.approx((60.0, 90.0, 120.0, 150.0))
    # Non-multiple includes {60 + 30k <= phi_req} U {phi_req} without duplicates
    assert dynamic_checkpoints_deg(135.0) == pytest.approx((60.0, 90.0, 120.0, 135.0))


def test_dynamic_pad_pad_screening_touching_boundary_and_separate_refinement():
    refs = {r.reference_id: r for r in shared.load_references()}
    rev = morphology_from_design("revolute", refs["2r_dd_tilt90_matched"].x, refs["2r_dd_tilt90_matched"].config)

    #Matched baseline 2R-DD is collision-free at all checkpoints up to 120 deg
    res = evaluate_dynamic_collision(rev, math.radians(120.0))
    assert res.feasible
    assert res.is_continuous_guarantee is False  # discrete screening never claims continuous proof
    assert len(res.checkpoints_rad) == 3

    refined = verify_dynamic_collision_refined(rev, math.radians(120.0), step_rad=math.radians(1.0))
    assert refined.feasible
    assert len(refined.checkpoints_rad) == 121

    # Construct an exact touching boundary at phi = 90 deg on a 2R hinge at A = (50, 0):
    # At phi = 90 deg, {D}'s +X points along -Z and {D}'s +Z points along +X:
    # A distal point (u_d, v_d) in {D} maps to (50 + v_d, -u_d) in {P}!
    # For distal pad u_d in [5, 25], v_d in [-4, 0] -> in {P}: x in [46, 50], z in [-25, -5].
    # Choose proximal pad x in [20, 46], z in [-8, -3] -> touches along x = 46, z in [-8, -5] with clearance = 0!
    touching = morphology_from_design("revolute", refs["2r_dd_tilt90_matched"].x, refs["2r_dd_tilt90_matched"].config)
    a_x = touching.mech.t_a.x_mm
    c_touch = ContactParams(
        proximal=ContactModuleSpec.parallel(
            length_mm=20.0, height_mm=5.0, s_start_mm=a_x - 4.0 - 20.0, mount_offset_mm=3.0
        ),
        distal=ContactModuleSpec.parallel(length_mm=20.0, height_mm=4.0, s_start_mm=5.0, mount_offset_mm=0.0),
    )
    m_touch = type(touching)(
        architecture=touching.architecture,
        mech=touching.mech,
        act=touching.act,
        struct=touching.struct,
        contact=c_touch,
        pivot_radii_mm=touching.pivot_radii_mm,
    )
    assert pad_pad_clearance_mm(m_touch, math.radians(90.0)) == pytest.approx(0.0, abs=1e-9)
    assert evaluate_dynamic_collision(m_touch, math.radians(90.0)).feasible  # touching (c == 0) is allowed

    # Extending proximal pad 0.5 mm further right causes overlap at 90 deg -> fails dynamic checkpoint
    c_overlap = ContactParams(
        proximal=ContactModuleSpec.parallel(
            length_mm=20.5, height_mm=5.0, s_start_mm=a_x - 4.0 - 20.0, mount_offset_mm=3.0
        ),
        distal=ContactModuleSpec.parallel(length_mm=20.0, height_mm=4.0, s_start_mm=5.0, mount_offset_mm=0.0),
    )
    m_overlap = type(touching)(
        architecture=touching.architecture,
        mech=touching.mech,
        act=touching.act,
        struct=touching.struct,
        contact=c_overlap,
        pivot_radii_mm=touching.pivot_radii_mm,
    )
    dyn_overlap = evaluate_dynamic_collision(m_overlap, math.radians(90.0))
    assert not dyn_overlap.feasible
    assert math.radians(90.0) in dyn_overlap.failed_checkpoints_rad
