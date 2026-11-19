"""Evaluate one rolling contact joint (RCJ) finger against the linkage study's goal.

Its solids and their gaps are the morphology's (``core.morphology.mechanism.rcj``);
this module holds them to the goal's bounds as constraints.

The MCP is at ``O``. Two rollers of one radius ``r`` roll on each other
without slip: the proximal one centred on ``A = (|OA|, 0)``, fixed to the
proximal segment, the distal one on ``B``, fixed to the distal segment
``B-C``. The servo's shaft is on ``A`` and turns the carrier ``A-B``
(``|AB| = 2r``) by ``theta``; the distal segment then turns by
``phi = 2 theta`` relative to the proximal one (the same ``phi`` as the
linkage's ``q_pip + q_dip`` and the 2R's), so ``k = 2`` throughout
(``core.morphology.mechanism.rcj``'s kinematics). Ideal no-slip rolling: the crossed
cables that hold it are not modelled (provisional, as the retention study's
RCJ).

Packaging, in one plane, by the linkage's rules: the servo lies back along
``-X`` from ``A``, tilted dorsally by ``servo_tilt_deg`` (its rounded front within the roller); the proximal pad hangs a
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

from core.geometry.primitives import rect_disk_gap_mm
from core.morphology.packaging.gaps import footprint_gaps
from core.morphology.mechanism.rcj import solids
from core.morphology.mechanism.rcj.solids import TORQUE_RATIO, RcjGeometry

from core.optimization.rcj.config import VARIABLE_NAMES, RcjConfig, distal_reach, proximal_reach
from core.optimization.revolute.evaluate import MOTION_CONSTRAINTS, UNSWEPT, RevoluteEvaluation, measure_motion


STATIC_CONSTRAINTS = ("bounds_finger_length_mm", "bounds_proportions_mm", "bounds_pad_drop_mm",
                      "bounds_proximal_pad_mm", "bounds_distal_pad_mm", "bounds_mount_depth_mm",
                      "static_mcp_servo_mm", "static_pip_servo_pad_mm", "static_roller_pad_mm",
                      "static_pad_collision_mm")
CONSTRAINT_NAMES = STATIC_CONSTRAINTS + MOTION_CONSTRAINTS
def geometry(x: Sequence[float], config: RcjConfig) -> RcjGeometry:
    """The solids of a design vector, with the study's packaging."""

    return solids.geometry(x, config.goal.packaging, config.joint_servo)


@dataclass(frozen=True)
class RcjEvaluation(RevoluteEvaluation):
    """Constraints, objective and diagnostics, named as the CFL's and the 2R's."""

    roller_radius_mm: float = math.nan

    constraint_names = CONSTRAINT_NAMES


def roller_mcp_gap_mm(geo: RcjGeometry, packaging, mcp_samples_rad) -> float:
    """Least gap (mm) from the proximal roller to the MCP servo over the MCP
    flexion samples (negative: overlapping; at least ``-r``)."""

    return float(solids.roller_mcp_gaps(geo.a, geo.roller_radius_mm, packaging, mcp_samples_rad)[0])


def mount_depth_violation(x: Sequence[float], config: RcjConfig) -> float:
    """How far the drops are from the mounting policy's (mm; 0 when they are searched)."""

    if config.goal.mount_policy == "searched":
        return 0.0
    from core.optimization.mounting import rcj_mounting

    v = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    cap = max(config.goal.bounds["proximal_mount_offset_mm"][1], config.goal.bounds["distal_mount_offset_mm"][1])
    return rcj_mounting(x, config).violation_mm((v["proximal_drop_mm"], v["distal_drop_mm"]), cap)


def evaluate(x: Sequence[float], config: RcjConfig) -> RcjEvaluation:
    """Evaluate one design (static checks first; a design failing one is not swept)."""

    goal = config.goal
    packaging = goal.packaging
    clearance = 0.0
    geo = geometry(x, config)
    r = geo.roller_radius_mm
    length_low, length_high = goal.extended_length_bounds_mm
    oa_low, oa_high = goal.proximal_length_bounds_mm
    oa = [value for value in (None if oa_low is None else oa_low - geo.a_x_mm,
                              None if oa_high is None else geo.a_x_mm - oa_high) if value is not None]
    # The segments |OA''| and |B''C| and |AB| = 2r as shares of |OC|, as the linkage's.
    v = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    oa += goal.length_ratio_violations(v["app_x_mm"], v["bc_length_mm"] - v["bpp_from_b_mm"], 2.0 * r,
                                       geo.finger_length_mm)
    oa += goal.icr0_violations(geo.a_x_mm + r, geo.finger_length_mm)  # the rollers' contact
    static = {
        "bounds_finger_length_mm": max(length_low - geo.finger_length_mm, geo.finger_length_mm - length_high),
        "bounds_proportions_mm": max(oa, default=0.0),
        # Each pad at most the joint's radius below its line.
        "bounds_pad_drop_mm": max(geo.distal_drop_mm, geo.proximal_drop_mm) - r,
        # At least its min, and on its body: no further than the roller's front
        # (plus the overhang), the distal pad no further behind B than its reach.
        "bounds_proximal_pad_mm": max(goal.minimum_proximal_support_mm - geo.proximal_length_mm,
                                   -geo.proximal_room_mm, geo.proximal_length_mm - geo.proximal_room_mm,
                                   x[VARIABLE_NAMES.index("app_x_mm")] - (geo.a_x_mm + proximal_reach(goal, r))),
        "bounds_distal_pad_mm": max(goal.minimum_distal_support_mm - geo.distal_length_mm, -geo.distal_room_mm,
                                 geo.distal_length_mm - geo.distal_room_mm,
                                 -x[VARIABLE_NAMES.index("bpp_from_b_mm")] - distal_reach(goal, r)),
        "bounds_mount_depth_mm": mount_depth_violation(x, config),
    }
    stages = (("static_mcp_servo_mm", "static_pip_servo_pad_mm", "static_roller_pad_mm"), ("static_pad_collision_mm",))
    if max(static.values()) <= 0.0:
        mcp_angles = np.asarray(goal.mcp_servo_angles_rad)
        # The servo on A and the proximal roller, which may outgrow the servo's front, against the
        # MCP servo (the idealized RCJ: the roller alone).
        roller = -(roller_mcp_gap_mm(geo, packaging, mcp_angles) - clearance)
        static["static_mcp_servo_mm"] = (roller if geo.servo is None
                                  else max(roller, -(packaging.mcp_servo_gap_mm(geo.servo, mcp_angles) - clearance)))
        static["static_pip_servo_pad_mm"] = (-(float(footprint_gaps(geo.proximal_pad[None], geo.servo, packaging)[0])
                                        - clearance) if packaging.pip_servo_pad_check and geo.servo is not None
                                      else 0.0)
        # Each pad on its own roller's body: touching, not inside (closed-form point-to-rectangle).
        p_box = (
            float(geo.proximal_pad[:, 0].min()),
            float(geo.proximal_pad[:, 0].max()),
            float(geo.proximal_pad[:, 1].min()),
            float(geo.proximal_pad[:, 1].max()),
        )
        d_box = (
            float(geo.distal_pad_local[:, 0].min()),
            float(geo.distal_pad_local[:, 0].max()),
            float(geo.distal_pad_local[:, 1].min()),
            float(geo.distal_pad_local[:, 1].max()),
        )
        static["static_roller_pad_mm"] = -min(
            rect_disk_gap_mm(p_box, geo.a, r),
            rect_disk_gap_mm(d_box, (0.0, 0.0), r),
        )
        static["static_roller_pad_mm"] = 0.0 if abs(static["static_roller_pad_mm"]) < 1e-9 else static["static_roller_pad_mm"]
    if max(static.values()) <= 0.0:
        margins_at_rest = _margins(geo, np.zeros(1), config)
        static["static_pad_collision_mm"] = -min(float(margin[0]) for margin in margins_at_rest.values())
    for name in stages[0] + stages[1]:
        static.setdefault(name, UNSWEPT)
    shortfalls = goal.length_ratio_shortfalls(v["app_x_mm"], v["bc_length_mm"] - v["bpp_from_b_mm"],
                                              geo.finger_length_mm)
    common = dict(x=tuple(float(value) for value in x), finger_length_mm=geo.finger_length_mm,
                  proximal_ratio_shortfall_mm=shortfalls[0], distal_ratio_shortfall_mm=shortfalls[1],
                  proximal_length_mm=geo.a_x_mm, required_rom_rad=goal.required_rom_rad,
                  proximal_support_length_mm=geo.proximal_length_mm,
                  distal_support_length_mm=geo.distal_length_mm,
                  proximal_depth_mm=geo.proximal_drop_mm + packaging.proximal_pad.height_mm,
                  distal_depth_mm=geo.distal_drop_mm + packaging.distal_pad.height_mm,
                  proximal_room_mm=geo.proximal_room_mm, distal_room_mm=geo.distal_room_mm,
                  required_proximal_mm=goal.minimum_proximal_support_mm,
                  required_distal_mm=goal.minimum_distal_support_mm,
                  roller_radius_mm=r)
    if max(static.values()) > 0.0:
        constraints = {**static, **{name: UNSWEPT for name in MOTION_CONSTRAINTS}}
        return RcjEvaluation(constraints=MappingProxyType(constraints), **common)

    # The servo turns the carrier by half the flexion: MA = 1/2.
    motion, fields = measure_motion(geo, geo, config, _margins, ma=1.0 / TORQUE_RATIO, l1_mm=v["app_x_mm"],
                                    l2_mm=v["bc_length_mm"] - v["bpp_from_b_mm"], distal_start_u_mm=v["bpp_from_b_mm"])
    return RcjEvaluation(constraints=MappingProxyType({**static, **motion}), **fields, **common)


def _margins(geo: RcjGeometry, phi: np.ndarray, config: RcjConfig,
             until_unsafe: bool = False, *, include_static_pairs: bool = True) -> dict[tuple[str, str], np.ndarray]:
    """Per-sample margin of every checked pair (``solids.margins``, the study's MCP samples)."""

    goal = config.goal
    return solids.margins(
        geo, phi, goal.packaging, goal.mcp_samples_rad, until_unsafe, include_static_pairs=include_static_pairs
    )


__all__ = ["CONSTRAINT_NAMES", "RcjEvaluation", "evaluate", "geometry", "mount_depth_violation", "roller_mcp_gap_mm"]
