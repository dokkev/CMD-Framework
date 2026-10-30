"""Evaluate one rolling contact joint (RCJ) finger against the linkage study's goal.

Its solids and their gaps are the morphology's (``core.morphology.rcj``);
this module holds them to the goal's bounds as constraints.

The MCP is at ``O``. Two rollers of one radius ``r`` roll on each other
without slip: the proximal one centred on ``A = (|OA|, 0)``, fixed to the
proximal segment, the distal one on ``B``, fixed to the distal segment
``B-C``. The servo's shaft is on ``A`` and turns the carrier ``A-B``
(``|AB| = 2r``) by ``theta``; the distal segment then turns by
``phi = 2 theta`` relative to the proximal one (the same ``phi`` as the
linkage's ``q_pip + q_dip`` and the 2R's), so ``k = 2`` throughout
(``core.morphology.rcj``'s kinematics). Ideal no-slip rolling: the crossed
cables that hold it are not modelled (provisional, as the retention study's
RCJ).

Packaging, in one plane, by the linkage's rules: the servo lies back along
``-X`` from ``A`` (``SERVO_REAR``; its rounded front within the roller); the proximal pad hangs a
searched drop below ``O-A`` (at most ``r``), laid from the MCP servo to
``A''``, at most ``A_x + r`` (plus ``goal.pads.proximal.overhang_max_mm``);
the distal pad hangs a searched drop below ``B-C`` (at most ``r``), laid
from ``C`` to ``B''`` (behind ``B`` at most the distal overhang past the
roller). Each pad clears its own roller (statically: each is fixed to its
roller). As the linkage's and the 2R's: each face counts less what the
other hides in the fold, the distal pad's carrier is checked against the
servo, and the joint hyperextends with every pair clear. The carrier and the links holding
the distal pad are in other layers, so they are not checked. No clearance,
overlaps by depth; checked over ``[0, 180 deg]`` from extension: the distal
pad against the proximal pad, the proximal roller, the servo and the MCP
servo (MCP flexion up to its max), the distal roller against the proximal
pad; statically the proximal pad against the servo and the MCP servo, and
the two servos. A pad's effective length is its face.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from core.morphology.packaging.gaps import footprint_gaps
from core.morphology.packaging.shapes import polygon_to_disk
from core.morphology.rcj import solids
from core.morphology.rcj.solids import TORQUE_RATIO, RcjGeometry

from core.optimization.rcj.config import VARIABLE_NAMES, RcjConfig, distal_reach, proximal_reach
from core.optimization.revolute.evaluate import (
    UNSWEPT,
    RevoluteEvaluation,
    _between_samples,
    hyperextension_violation,
    usable_lengths,
)


STATIC_CONSTRAINTS = ("extended_length_mm", "proximal_length_mm", "pad_drop_mm", "mcp_servo_mm",
                      "pip_servo_pad_mm", "proximal_support_mm", "distal_support_mm", "roller_pad_mm",
                      "pad_collision_mm")
MOTION_CONSTRAINTS = ("rom_deg", "hyperextension_mm", "curl_tip_x_mm", "pad_usable_mm", "torque_ratio")
CONSTRAINT_NAMES = STATIC_CONSTRAINTS + MOTION_CONSTRAINTS
def geometry(x: Sequence[float], config: RcjConfig) -> RcjGeometry:
    """The solids of a design vector, with the study's packaging."""

    return solids.geometry(x, config.goal.packaging)


@dataclass(frozen=True)
class RcjEvaluation(RevoluteEvaluation):
    """Constraints, objectives and diagnostics, named as the linkage's and the 2R's."""

    torque_ratio_max: float = TORQUE_RATIO
    roller_radius_mm: float = math.nan

    @property
    def g(self) -> tuple[float, ...]:
        return tuple(self.constraints[name] for name in CONSTRAINT_NAMES)


def evaluate(x: Sequence[float], config: RcjConfig) -> RcjEvaluation:
    """Evaluate one design (static checks first; a design failing one is not swept)."""

    goal = config.goal
    packaging = goal.packaging
    clearance = packaging.body_clearance_mm
    geo = geometry(x, config)
    r = geo.roller_radius_mm
    length_low, length_high = goal.extended_length_bounds_mm
    oa_low, oa_high = goal.proximal_length_bounds_mm
    oa = [value for value in (None if oa_low is None else oa_low - geo.a_x_mm,
                              None if oa_high is None else geo.a_x_mm - oa_high) if value is not None]
    # |OA|, BC and |AB| = 2r as shares of |OC|, as the linkage's.
    oa += goal.length_ratio_violations(geo.a_x_mm, geo.bc_length_mm, 2.0 * r, geo.finger_length_mm)
    oa += goal.icr0_violations(geo.a_x_mm + r, geo.finger_length_mm)  # the rollers' contact
    static = {
        "extended_length_mm": max(length_low - geo.finger_length_mm, geo.finger_length_mm - length_high),
        "proximal_length_mm": max(oa, default=0.0),
        # Each pad at most the joint's radius below its line.
        "pad_drop_mm": max(geo.distal_drop_mm, geo.proximal_drop_mm) - r,
        # At least its min, and on its body: no further than the roller's front
        # (plus the overhang), the distal pad no further behind B than its reach.
        "proximal_support_mm": max(goal.minimum_proximal_support_mm - geo.proximal_length_mm,
                                   -geo.proximal_room_mm,
                                   x[VARIABLE_NAMES.index("app_x_mm")] - (geo.a_x_mm + proximal_reach(goal, r))),
        "distal_support_mm": max(goal.minimum_distal_support_mm - geo.distal_length_mm, -geo.distal_room_mm,
                                 -x[VARIABLE_NAMES.index("bpp_from_b_mm")] - distal_reach(goal, r)),
    }
    stages = (("mcp_servo_mm", "pip_servo_pad_mm", "roller_pad_mm"), ("pad_collision_mm",))
    if max(static.values()) <= 0.0:
        mcp_angles = np.asarray(goal.mcp_servo_angles_rad)
        static["mcp_servo_mm"] = -(packaging.mcp_servo_gap_mm(geo.servo, mcp_angles) - clearance)
        static["pip_servo_pad_mm"] = (-(float(footprint_gaps(geo.proximal_pad[None], geo.servo, packaging)[0])
                                        - clearance) if packaging.pip_servo_pad_check else 0.0)
        # Each pad on its own roller's body: touching, not inside.
        static["roller_pad_mm"] = -min(
            float(polygon_to_disk(geo.proximal_pad[None], geo.a[None], r)[0]),
            float(polygon_to_disk(geo.distal_pad_local[None], np.zeros((1, 2)), r)[0]))
        static["roller_pad_mm"] = 0.0 if abs(static["roller_pad_mm"]) < 1e-9 else static["roller_pad_mm"]
    if max(static.values()) <= 0.0:
        margins_at_rest = _margins(geo, np.zeros(1), config)
        static["pad_collision_mm"] = -min(float(margin[0]) for margin in margins_at_rest.values())
    for name in stages[0] + stages[1]:
        static.setdefault(name, UNSWEPT)
    common = dict(x=tuple(float(value) for value in x), finger_length_mm=geo.finger_length_mm,
                  proximal_length_mm=geo.a_x_mm, required_rom_rad=goal.required_rom_rad,
                  proximal_support_length_mm=geo.proximal_length_mm,
                  distal_support_length_mm=geo.distal_length_mm,
                  proximal_face_mm=geo.proximal_face_mm, distal_face_mm=geo.distal_face_mm,
                  proximal_depth_mm=geo.proximal_drop_mm + packaging.proximal_pad.height_mm,
                  distal_depth_mm=geo.distal_drop_mm + packaging.distal_pad.height_mm,
                  roller_radius_mm=r)
    if max(static.values()) > 0.0:
        constraints = {**static, **{name: UNSWEPT for name in MOTION_CONSTRAINTS}}
        return RcjEvaluation(constraints=MappingProxyType(constraints), **common)

    # phi on the servo grid, as the 2R's (the servo turns half as far).
    phi = np.asarray(goal.servo_samples_rad)
    margins = _between_samples(geo, phi, _margins(geo, phi, config, until_unsafe=True), config, _margins)
    least = np.min(np.stack(list(margins.values())), axis=0)
    unsafe = least < 0.0
    first_unsafe = int(np.flatnonzero(unsafe)[0]) if unsafe.any() else len(phi)
    reach = float(min(phi[first_unsafe - 1], math.pi)) if first_unsafe > 0 else 0.0
    stop_reason, collision_pair = ("curl" if reach >= math.pi - 1e-12 else "horizon"), ""
    if first_unsafe < len(phi) and reach < math.pi - 1e-12:
        stop_reason = "pad_collision"
        collision_pair = "/".join(next(pair for pair, margin in margins.items() if margin[first_unsafe] < 0.0))
    at = np.flatnonzero(phi >= goal.required_rom_rad - 1e-12)
    last = int(at[0]) if at.size else max(first_unsafe - 1, 0)
    window = np.zeros(len(phi), dtype=bool)
    window[:last + 1] = True
    tip_x = geo.tip(phi)[:, 0]
    failing = unsafe | (tip_x < 0.0)
    first_failing = int(np.flatnonzero(failing)[0]) if failing.any() else len(phi)
    phi_max = float(min(phi[first_failing - 1], math.pi)) if first_failing > 0 else 0.0
    reported = [margin for pair, margin in margins.items() if pair != ("mcp_servo", "proximal_pad")]
    proximal_usable, distal_usable = usable_lengths(geo, phi[window], goal, "rcj")
    constraints = {
        **static,
        "rom_deg": math.degrees(goal.required_rom_rad - reach),
        "hyperextension_mm": hyperextension_violation(geo, config, _margins),
        "curl_tip_x_mm": -float(tip_x[window].min()),
        "pad_usable_mm": max(goal.minimum_proximal_support_mm - min(geo.proximal_face_mm, proximal_usable),
                             goal.minimum_distal_support_mm - min(geo.distal_face_mm, distal_usable)),
        "torque_ratio": 0.0 if goal.torque_ratio_max is None else TORQUE_RATIO - goal.torque_ratio_max,
    }
    return RcjEvaluation(
        constraints=MappingProxyType(constraints),
        proximal_usable_length_mm=proximal_usable,
        distal_usable_length_mm=distal_usable,
        phi_safe_rad=reach,
        phi_max_rad=phi_max,
        stop_reason=stop_reason,
        collision_pair=collision_pair,
        least_tip_x_mm=float(tip_x[window].min()),
        pad_clearance_mm=float(np.min(np.stack(reported), axis=0)[window].min()) if reported else math.inf,
        **common,
    )


def _margins(geo: RcjGeometry, phi: np.ndarray, config: RcjConfig,
             until_unsafe: bool = False) -> dict[tuple[str, str], np.ndarray]:
    """Per-sample margin of every checked pair (``solids.margins``, the study's MCP samples)."""

    goal = config.goal
    return solids.margins(geo, phi, goal.packaging, goal.mcp_samples_rad, until_unsafe)


__all__ = ["CONSTRAINT_NAMES", "RcjEvaluation", "evaluate", "geometry"]
