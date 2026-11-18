"""Static and Dynamic Collision Formulation (SSoT Section 6 / INSTRUCTION Section 4).

Separates collision evaluation into two mathematically distinct stages:

1. **Static collision** at the reference configuration ``q = 0``:
   evaluates signed clearances ``c_{ij}(M, 0)`` (``> 0`` separated, ``= 0``
   touching, ``< 0`` interpenetrating) using closed-form analytical geometry
   wherever applicable. Configuration-invariant intra-body pairs belong strictly
   here and are never re-evaluated along motion sweeps.
2. **Dynamic collision** parameterized by relative distal flexion ``phi``:
   evaluates cross-body pad--pad signed clearance ``c_{pd}(M, phi) >= 0`` on
   the discrete screening checkpoint set

   .. math::
       \\Phi_{\\mathrm{check}}(\\phi_{\\min}^{\\mathrm{des}}) =
       \\{60^\\circ + 30^\\circ k : k \\in \\mathbb Z_{\\ge 0},\\;
         60^\\circ + 30^\\circ k \\le \\phi_{\\min}^{\\mathrm{des}}\\}
       \\cup \\{\\phi_{\\min}^{\\mathrm{des}}\\}.

   Dense-grid interval verification is provided as a separate validation
   procedure and is never conflated with discrete checkpoint screening.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from core._validation import require_finite, require_nonnegative
from core.morphology.canonical import (
    CoDeMechanismParams,
    FingerMorphology,
    MorphologyError,
    RcjMechanismParams,
    RevoluteMechanismParams,
)
from core.morphology.mechanism.linkage.model import KinematicClosureError
from core.morphology.packaging.hardware import MARGIN_TOLERANCE_MM
from core.morphology.packaging.shapes import (
    ServoFootprint,
    polygon_to_footprint,
    polygons_gap,
    separation,
    signed_gaps,
)

# ---------------------------------------------------------------------------
# Closed-Form Analytical Clearance Primitives
# ---------------------------------------------------------------------------


def point_rect_distance_mm(
    box_bounds: tuple[float, float, float, float],
    point_uv: Sequence[float],
) -> float:
    """Closed-form Euclidean distance from ``point_uv`` to solid rectangle ``[u0, u1] x [v0, v1]``."""

    u0, u1, v0, v1 = (float(v) for v in box_bounds)
    if u0 > u1 or v0 > v1:
        raise ValueError(f"invalid rectangle bounds {box_bounds!r}")
    cu, cv = float(point_uv[0]), float(point_uv[1])
    du = max(u0 - cu, 0.0, cu - u1)
    dv = max(v0 - cv, 0.0, cv - v1)
    return math.hypot(du, dv)


def rect_disk_clearance_mm(
    box_bounds: tuple[float, float, float, float],
    center_uv: Sequence[float],
    radius_mm: float,
) -> float:
    """Closed-form signed clearance ``dist(center, rect) - r`` between a rectangular block and a disk.

    Positive: separated; zero: tangent contact; negative: interpenetration
    (matching ``packaging.shapes.polygon_to_disk`` without polygon edge iteration).
    """

    r = require_nonnegative("radius_mm", radius_mm)
    return point_rect_distance_mm(box_bounds, center_uv) - float(r)


def disk_disk_clearance_mm(
    center1_xz: Sequence[float],
    radius1_mm: float,
    center2_xz: Sequence[float],
    radius2_mm: float,
) -> float:
    """Closed-form signed clearance ``||c1 - c2||_2 - (r1 + r2)`` between two circular housings."""

    r1 = require_nonnegative("radius1_mm", radius1_mm)
    r2 = require_nonnegative("radius2_mm", radius2_mm)
    dx = float(center1_xz[0]) - float(center2_xz[0])
    dz = float(center1_xz[1]) - float(center2_xz[1])
    return math.hypot(dx, dz) - (float(r1) + float(r2))


def point_servo_distance_mm(point_xz: Sequence[float], footprint: ServoFootprint) -> float:
    """Closed-form Euclidean distance from ``point_xz`` to a D-shaped servo footprint ``G_D(r_s, e_rear)``."""

    px, pz = float(point_xz[0]) - footprint.pivot[0], float(point_xz[1]) - footprint.pivot[1]
    fx, fz = -footprint.rear[0], -footprint.rear[1]
    sx, sz = -fz, fx
    u = px * fx + pz * fz
    v = px * sx + pz * sz
    r = footprint.radius_mm
    rear = footprint.rear_extent_mm
    d_disk = max(math.hypot(u, v) - r, 0.0)
    du_box = max(-rear - u, 0.0, u)
    dv_box = max(abs(v) - r, 0.0)
    d_box = math.hypot(du_box, dv_box)
    return min(d_disk, d_box)


def disk_servo_clearance_mm(
    center_xz: Sequence[float], radius_mm: float, footprint: ServoFootprint
) -> float:
    """Closed-form signed clearance between circular housing ``D(c, r)`` and ``ServoFootprint``."""

    r = require_nonnegative("radius_mm", radius_mm)
    return point_servo_distance_mm(center_xz, footprint) - float(r)


def polygon_polygon_clearance_mm(first_xz: np.ndarray, second_xz: np.ndarray) -> float:
    """Signed clearance between two convex polygons ``(m, 2)`` and ``(k, 2)``."""

    p1 = np.asarray(first_xz, dtype=float)[None]
    p2 = np.asarray(second_xz, dtype=float)[None]
    gap = polygons_gap(p1, p2)
    return float(signed_gaps(gap, p1, p2)[0])


def polygon_servo_clearance_mm(polygon_xz: np.ndarray, footprint: ServoFootprint) -> float:
    """Signed clearance between convex polygon ``(m, 2)`` and a D-shaped ``ServoFootprint``."""

    poly = np.asarray(polygon_xz, dtype=float)[None]
    gap = polygon_to_footprint(poly, footprint)
    outline = footprint.outline[None]
    return float(signed_gaps(gap, poly, outline)[0])


def servo_servo_clearance_mm(first: ServoFootprint, second: ServoFootprint) -> float:
    """Signed clearance between two D-shaped servo footprints."""

    gap = first.distance_to_footprint(second)
    if gap > 0.0:
        return float(gap)
    depth = float(separation(first.outline[None], second.outline[None])[0])
    return depth if depth < 0.0 else 0.0


# ---------------------------------------------------------------------------
# Static Collision Evaluation (q = 0)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class StaticCollisionResult:
    """Result of static collision evaluation at the reference pose ``q = 0``."""

    clearances_mm: Mapping[tuple[str, str], float]
    tolerance_mm: float = MARGIN_TOLERANCE_MM

    @property
    def min_clearance_mm(self) -> float:
        if not self.clearances_mm:
            return math.inf
        return min(self.clearances_mm.values())

    @property
    def violated_pairs(self) -> tuple[tuple[str, str], ...]:
        return tuple(
            pair
            for pair, val in self.clearances_mm.items()
            if val < -self.tolerance_mm
        )

    @property
    def feasible(self) -> bool:
        return len(self.violated_pairs) == 0


def evaluate_static_collision(
    morphology: FingerMorphology,
    *,
    check_a_pin_pip_servo: bool = False,
    check_a_pin_pip_shaft: bool = True,
    check_pip_servo_proximal_pad: bool = True,
    check_mcp_servo: bool = True,
    check_pad_pad: bool = True,
    tolerance_mm: float = MARGIN_TOLERANCE_MM,
) -> StaticCollisionResult:
    """Evaluate signed clearances ``c_{ij}(M, 0)`` at ``q = 0`` for all static & intra-body pairs.

    Intra-body pairs are computed directly in their shared parent body frame
    using closed-form analytical geometry; cross-body reference-pose pairs are
    evaluated once at ``(q_MCP = 0, phi = 0)``.
    """

    morphology.validate(require_support_containment=False)
    clearances: dict[tuple[str, str], float] = {}

    p_box = morphology.contact.proximal.box_bounds_local
    d_box = morphology.contact.distal.box_bounds_local
    polys0 = morphology.posed_contact_polygons(0.0, 0.0)
    p_poly0, d_poly0 = polys0["proximal_pad"], polys0["distal_pad"]

    mcp_fp = morphology.act.mcp.to_servo_footprint() if check_mcp_servo else None
    pip_fp = morphology.act.pip.to_servo_footprint()
    radii = morphology.pivot_radii_mm

    if morphology.architecture == "CoDe":
        assert isinstance(morphology.mech, CoDeMechanismParams)
        mech = morphology.mech
        r_a = radii.get("a", 3.0)
        r_ap = radii.get("ap", 5.0)
        r_b = radii.get("b", 3.0)
        r_bp = radii.get("bp", 3.0)

        # Intra-body {P}: proximal pad vs own pins (closed-form point-to-rectangle)
        clearances[("a_pin", "proximal_pad")] = rect_disk_clearance_mm(p_box, mech.t_a.pos, r_a)
        clearances[("ap_pin", "proximal_pad")] = rect_disk_clearance_mm(p_box, mech.t_ap.pos, r_ap)

        # Intra-body {D}: distal pad vs own pins (closed-form in {D}: B is at (0, 0), B' at t_bp_local)
        clearances[("b_pin", "distal_pad")] = rect_disk_clearance_mm(d_box, (0.0, 0.0), r_b)
        clearances[("bp_pin", "distal_pad")] = rect_disk_clearance_mm(d_box, mech.t_bp_local.pos, r_bp)

        # Intra-link pin spacing (same rigid bar/body)
        bp0 = mech.t_bp0.pos
        b0 = mech.t_b0.pos
        clearances[("a_pin", "bp_pin")] = disk_disk_clearance_mm(mech.t_a.pos, r_a, bp0, r_bp)
        clearances[("b_pin", "bp_pin")] = disk_disk_clearance_mm(b0, r_b, bp0, r_bp)

        if check_a_pin_pip_shaft:
            clearances[("a_pin", "pip_shaft")] = disk_disk_clearance_mm(
                mech.t_a.pos, r_a, mech.t_ap.pos, morphology.ap_shaft_clearance_radius_mm
            )
        if check_a_pin_pip_servo and pip_fp is not None:
            clearances[("a_pin", "pip_servo")] = disk_servo_clearance_mm(mech.t_a.pos, r_a, pip_fp)

        # Cross-body reference-pose pin-pad & pin-servo checks at q = 0
        d_box_in_p = (
            d_box[0] + b0[0],
            d_box[1] + b0[0],
            d_box[2] + b0[1],
            d_box[3] + b0[1],
        )
        clearances[("a_pin", "distal_pad")] = rect_disk_clearance_mm(d_box_in_p, mech.t_a.pos, r_a)
        clearances[("ap_pin", "distal_pad")] = rect_disk_clearance_mm(d_box_in_p, mech.t_ap.pos, r_ap)
        clearances[("b_pin", "proximal_pad")] = rect_disk_clearance_mm(p_box, b0, r_b)
        clearances[("bp_pin", "proximal_pad")] = rect_disk_clearance_mm(p_box, bp0, r_bp)
        if pip_fp is not None:
            clearances[("b_pin", "pip_servo")] = disk_servo_clearance_mm(b0, r_b, pip_fp)

    elif morphology.architecture in ("2R", "2R-remote"):
        assert isinstance(morphology.mech, RevoluteMechanismParams)
        a_pos = morphology.mech.t_a.pos
        r_a = radii.get("a", 3.0)
        if morphology.architecture == "2R-remote" or pip_fp is None:
            # Joint holds pin housing A (closed-form in {P} and {D})
            clearances[("a_pin", "proximal_pad")] = rect_disk_clearance_mm(p_box, a_pos, r_a)
            clearances[("a_pin", "distal_pad")] = rect_disk_clearance_mm(d_box, (0.0, 0.0), r_a)
        if morphology.architecture == "2R-remote" and check_a_pin_pip_servo and pip_fp is not None:
            clearances[("a_pin", "pip_servo")] = disk_servo_clearance_mm(a_pos, r_a, pip_fp)

    elif morphology.architecture == "RCJ":
        assert isinstance(morphology.mech, RcjMechanismParams)
        a_pos = morphology.mech.t_a.pos
        b0 = morphology.mech.distal_origin_extended
        r = morphology.mech.roller_radius_mm
        # Intra-body pad vs own roller (closed-form)
        clearances[("proximal_pad", "proximal_roller")] = rect_disk_clearance_mm(p_box, a_pos, r)
        clearances[("distal_pad", "distal_roller")] = rect_disk_clearance_mm(d_box, (0.0, 0.0), r)
        # Cross-body reference-pose pad vs opposite roller
        d_box_in_p = (d_box[0] + b0[0], d_box[1] + b0[0], d_box[2], d_box[3])
        clearances[("distal_pad", "proximal_roller")] = rect_disk_clearance_mm(d_box_in_p, a_pos, r)
        clearances[("distal_roller", "proximal_pad")] = rect_disk_clearance_mm(p_box, b0, r)
        if mcp_fp is not None:
            clearances[("mcp_servo", "proximal_roller")] = disk_servo_clearance_mm(a_pos, r, mcp_fp)
            clearances[("distal_roller", "mcp_servo")] = disk_servo_clearance_mm(b0, r, mcp_fp)

    # Common actuator & pad checks at q = 0
    if mcp_fp is not None and pip_fp is not None:
        clearances[("mcp_servo", "pip_servo")] = servo_servo_clearance_mm(mcp_fp, pip_fp)
    if pip_fp is not None and check_pip_servo_proximal_pad:
        clearances[("pip_servo", "proximal_pad")] = polygon_servo_clearance_mm(p_poly0, pip_fp)
    if pip_fp is not None:
        clearances[("distal_pad", "pip_servo")] = polygon_servo_clearance_mm(d_poly0, pip_fp)
    if mcp_fp is not None:
        clearances[("mcp_servo", "proximal_pad")] = polygon_servo_clearance_mm(p_poly0, mcp_fp)
        clearances[("distal_pad", "mcp_servo")] = polygon_servo_clearance_mm(d_poly0, mcp_fp)
    if check_pad_pad:
        clearances[("distal_pad", "proximal_pad")] = polygon_polygon_clearance_mm(d_poly0, p_poly0)

    return StaticCollisionResult(MappingProxyType(clearances), tolerance_mm=tolerance_mm)


# ---------------------------------------------------------------------------
# Dynamic Collision Evaluation (Checkpoints & Separate Interval Verification)
# ---------------------------------------------------------------------------


def dynamic_checkpoints_deg(phi_min_des_deg: float) -> tuple[float, ...]:
    """Discrete dynamic collision checkpoint angles [deg] for ``phi_min_des_deg``:

    .. math::
        \\Phi_{\\mathrm{check}} =
        \\{60^\\circ + 30^\\circ k : k \\in \\mathbb Z_{\\ge 0},\\;
          60^\\circ + 30^\\circ k \\le \\phi_{\\min}^{\\mathrm{des}}\\}
        \\cup \\{\\phi_{\\min}^{\\mathrm{des}}\\}.
    """

    target = require_nonnegative("phi_min_des_deg", phi_min_des_deg)
    points: list[float] = []
    k = 0
    while True:
        angle = 60.0 + 30.0 * k
        if angle > target + 1e-9:
            break
        points.append(min(angle, target))
        k += 1
    if not points or abs(points[-1] - target) > 1e-9:
        points.append(float(target))
    return tuple(points)


def dynamic_checkpoints_rad(phi_min_des_rad: float) -> tuple[float, ...]:
    """Discrete dynamic collision checkpoint angles [rad] corresponding to :func:`dynamic_checkpoints_deg`."""

    deg = math.degrees(require_nonnegative("phi_min_des_rad", phi_min_des_rad))
    return tuple(math.radians(a) for a in dynamic_checkpoints_deg(deg))


def pad_pad_clearance_mm(morphology: FingerMorphology, phi_rad: float) -> float:
    """Signed clearance ``c_{pd}(M, phi)`` between proximal and distal pads at relative flexion ``phi``.

    Returns ``-inf`` if the mechanism fails kinematic loop closure at ``phi``.
    """

    phi = require_finite("phi_rad", phi_rad)
    try:
        polys = morphology.posed_contact_polygons(phi, 0.0)
    except (KinematicClosureError, MorphologyError, ValueError):
        return -math.inf
    return polygon_polygon_clearance_mm(polys["distal_pad"], polys["proximal_pad"])


@dataclass(frozen=True)
class DynamicCollisionResult:
    """Result of discrete checkpoint dynamic collision screening ``c_{pd}(M, phi) >= 0``."""

    checkpoints_rad: tuple[float, ...]
    clearances_mm: tuple[float, ...]
    tolerance_mm: float = MARGIN_TOLERANCE_MM
    is_continuous_guarantee: bool = False
    """Always ``False``: finite checkpoint screening does not mathematically prove continuous clearance."""

    @property
    def min_clearance_mm(self) -> float:
        return min(self.clearances_mm) if self.clearances_mm else math.inf

    @property
    def failed_checkpoints_rad(self) -> tuple[float, ...]:
        return tuple(
            phi
            for phi, c in zip(self.checkpoints_rad, self.clearances_mm, strict=True)
            if not math.isfinite(c) or c < -self.tolerance_mm
        )

    @property
    def feasible(self) -> bool:
        return len(self.failed_checkpoints_rad) == 0


def evaluate_dynamic_collision(
    morphology: FingerMorphology,
    phi_min_des_rad: float,
    *,
    checkpoints_rad: Sequence[float] | None = None,
    tolerance_mm: float = MARGIN_TOLERANCE_MM,
) -> DynamicCollisionResult:
    """Screen dynamic pad--pad collision ``c_{pd}(M, phi) >= 0`` over ``Phi_check(phi_min^des)``."""

    pts = (
        dynamic_checkpoints_rad(phi_min_des_rad)
        if checkpoints_rad is None
        else tuple(float(require_finite("checkpoint", p)) for p in checkpoints_rad)
    )
    vals = tuple(pad_pad_clearance_mm(morphology, phi) for phi in pts)
    return DynamicCollisionResult(pts, vals, tolerance_mm=tolerance_mm)


def verify_dynamic_collision_refined(
    morphology: FingerMorphology,
    phi_min_des_rad: float,
    *,
    step_rad: float = math.radians(0.5),
    tolerance_mm: float = MARGIN_TOLERANCE_MM,
) -> DynamicCollisionResult:
    """Separate dense-grid validation procedure over ``[0, phi_min_des_rad]``."""

    target = require_nonnegative("phi_min_des_rad", phi_min_des_rad)
    if step_rad <= 0.0 or not math.isfinite(step_rad):
        raise ValueError("step_rad must be finite and positive")
    if target == 0.0:
        grid = (0.0,)
    else:
        n = max(int(math.ceil(target / step_rad)), 1)
        grid = tuple(float(v) for v in np.linspace(0.0, target, n + 1))
    return evaluate_dynamic_collision(
        morphology, target, checkpoints_rad=grid, tolerance_mm=tolerance_mm
    )


__all__ = [
    "DynamicCollisionResult",
    "StaticCollisionResult",
    "disk_disk_clearance_mm",
    "disk_servo_clearance_mm",
    "dynamic_checkpoints_deg",
    "dynamic_checkpoints_rad",
    "evaluate_dynamic_collision",
    "evaluate_static_collision",
    "pad_pad_clearance_mm",
    "point_rect_distance_mm",
    "point_servo_distance_mm",
    "polygon_polygon_clearance_mm",
    "polygon_servo_clearance_mm",
    "rect_disk_clearance_mm",
    "servo_servo_clearance_mm",
    "verify_dynamic_collision_refined",
]
