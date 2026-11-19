"""Evaluate one serial 2R driven from a servo on ``A'`` through a belt
against the linkage study's goal.

The distal segment ``A-C`` turns about the pin ``A`` by ``phi``, as the 2R's.
The servo's shaft is on ``A' = A + (dx, dz)``, its body laid back along
``-X`` tilted dorsally by the searched tilt (as the linkage's); a belt from
its pulley turns the driven pulley on ``A``, fixed to the distal segment,
by the same angle: ``phi = theta``, so ``k = 1``, with no dead centre (a
parallelogram from ``A'`` turning 1:1 keeps its transmission angle within
[30, 150] deg for at most 120 deg of flexion; the belt has no such limit).

Its solids are the morphology's (``core.morphology.mechanism.revolute_parallel``).
Every other rule is the idealized 2R's (``core.optimization.revolute``,
``joint_servo=False``): the pads hang at most ``A``'s pin radius below their
segment lines, keep off its housing, and are checked against each other,
the joint and the MCP servo. The servo is checked against the MCP servo,
the proximal pad, ``A``'s pin housing (as the linkage's), the distal pad and
its carrier. The belt and its pulleys run in the outer layer (unchecked, as
the linkage's links). Each face counts less what the other hides in the
fold, and the joint hyperextends with every pair clear (the linkage's v4 rules).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from core.geometry.intervals import face_occluded_set
from core.morphology.packaging.gaps import footprint_gaps
from core.morphology.mechanism.revolute_parallel import solids
from core.morphology.mechanism.revolute_parallel.solids import ParallelGeometry

from core.optimization.revolute.evaluate import (
    MOTION_CONSTRAINTS,
    UNSWEPT,
    RevoluteEvaluation,
    joint_checks,
    measure_motion,
    segment_lengths,
)
from core.optimization.revolute_parallel.config import VARIABLE_NAMES, ParallelConfig


STATIC_CONSTRAINTS = ("bounds_finger_length_mm", "bounds_proportions_mm", "bounds_proximal_pad_mm",
                      "bounds_distal_pad_mm", "bounds_mount_depth_mm", "static_mcp_servo_mm",
                      "static_pip_servo_pad_mm", "static_a_pin_servo_mm", "static_pad_collision_mm")
CONSTRAINT_NAMES = STATIC_CONSTRAINTS + MOTION_CONSTRAINTS


def geometry(x: Sequence[float], config: ParallelConfig) -> ParallelGeometry:
    """The solids of a design vector, with the study's packaging and supports."""

    return solids.geometry(x, config.goal.packaging, config.pulley_radius_mm)


@dataclass(frozen=True)
class ParallelEvaluation(RevoluteEvaluation):
    """Constraints, objective and diagnostics, named as the CFL's and the 2R's."""

    constraint_names = CONSTRAINT_NAMES


def _belt_occluders(geo: ParallelGeometry):
    a, ap, radius = geo.a, geo.ap, geo.pulley_radius_mm
    diff = a - ap
    norm = float(np.linalg.norm(diff))
    strands = []
    if norm > 1e-9:
        along = diff / norm
        side = radius * np.array((-along[1], along[0]), dtype=float)
        strands = [(ap + side, a + side), (ap - side, a - side)]
    return strands, [(a, radius), (ap, radius)], [(a, radius)]


def belt_hidden(geo: ParallelGeometry):
    """What the belt's strands and pulleys hide on each face (``measure_motion``'s ``hidden``):
    the strands and both pulleys on the proximal face, the joint's pulley on the distal one."""

    strands, proximal_disks, distal_disks = _belt_occluders(geo)

    def hidden(q, proximal_face, proximal_normal, distal_face, distal_normal):
        return (face_occluded_set(proximal_face, proximal_normal, segments=strands, disks=proximal_disks),
                face_occluded_set(distal_face, distal_normal, disks=distal_disks))

    return hidden


def mount_depth_violation(x: Sequence[float], config: ParallelConfig) -> float:
    """How far the drops are from the mounting policy's (mm; 0 when they are searched)."""

    if config.goal.mount_policy == "searched":
        return 0.0
    from core.optimization.mounting import parallel_mounting

    v = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    cap = max(config.goal.bounds["proximal_mount_offset_mm"][1], config.goal.bounds["distal_mount_offset_mm"][1])
    return parallel_mounting(x, config).violation_mm((v["proximal_drop_mm"], v["distal_drop_mm"]), cap)


def evaluate(x: Sequence[float], config: ParallelConfig) -> ParallelEvaluation:
    """Evaluate one design (static checks first; a design failing one is not swept)."""

    goal = config.goal
    packaging = goal.packaging
    clearance = 0.0
    geo = geometry(x, config)
    joint = geo.joint
    length_low, length_high = goal.extended_length_bounds_mm
    oa_low, oa_high = goal.proximal_length_bounds_mm
    oa = [value for value in (None if oa_low is None else oa_low - joint.a_x_mm,
                              None if oa_high is None else joint.a_x_mm - oa_high) if value is not None]
    oa += goal.length_ratio_violations(*segment_lengths(x), 0.0, joint.finger_length_mm)
    oa += goal.icr0_violations(joint.a_x_mm, joint.finger_length_mm)  # the pin A
    static = {
        "bounds_finger_length_mm": max(length_low - joint.finger_length_mm, joint.finger_length_mm - length_high),
        "bounds_proportions_mm": max(oa, default=0.0),
        # At least its min, and no longer than its segment's room (a module of a set length, 2026-11-06).
        "bounds_proximal_pad_mm": max(goal.minimum_proximal_support_mm - joint.proximal_length_mm,
                                   -joint.proximal_room_mm, joint.proximal_length_mm - joint.proximal_room_mm,
                                   x[VARIABLE_NAMES.index("app_x_mm")] - (joint.a_x_mm - config.joint_radius_mm)),
        "bounds_distal_pad_mm": max(goal.minimum_distal_support_mm - joint.distal_length_mm, -joint.distal_room_mm,
                                 joint.distal_length_mm - joint.distal_room_mm,
                                 config.joint_radius_mm - x[VARIABLE_NAMES.index("bpp_from_a_mm")]),
        "bounds_mount_depth_mm": mount_depth_violation(x, config),
    }
    if max(static.values()) <= 0.0:
        mcp_angles = np.asarray(goal.mcp_servo_angles_rad)
        static["static_mcp_servo_mm"] = -(packaging.mcp_servo_gap_mm(geo.servo, mcp_angles) - clearance)
        static["static_pip_servo_pad_mm"] = (-(float(footprint_gaps(joint.proximal_pad[None], geo.servo, packaging)[0])
                                        - clearance) if packaging.pip_servo_pad_check else 0.0)
        # A's pin housing clear of the servo body, as the linkage's.
        static["static_a_pin_servo_mm"] = (-(geo.servo.distance_to_point(geo.a) - packaging.pivot_radius_mm("a")
                                      - clearance) if packaging.a_pin_servo_check else 0.0)
    if max(static.values()) <= 0.0:
        margins_at_rest = _margins(geo, np.zeros(1), config)
        static["static_pad_collision_mm"] = -min(float(margin[0]) for margin in margins_at_rest.values())
    for name in STATIC_CONSTRAINTS:
        static.setdefault(name, UNSWEPT)
    shortfalls = goal.length_ratio_shortfalls(*segment_lengths(x), joint.finger_length_mm)
    common = dict(x=tuple(float(value) for value in x), finger_length_mm=joint.finger_length_mm,
                  proximal_ratio_shortfall_mm=shortfalls[0], distal_ratio_shortfall_mm=shortfalls[1],
                  proximal_length_mm=joint.a_x_mm, required_rom_rad=goal.required_rom_rad,
                  proximal_support_length_mm=joint.proximal_length_mm,
                  distal_support_length_mm=joint.distal_length_mm,
                  proximal_depth_mm=joint.proximal_drop_mm + packaging.proximal_pad.height_mm,
                  distal_depth_mm=joint.distal_drop_mm + packaging.distal_pad.height_mm,
                  proximal_room_mm=joint.proximal_room_mm, distal_room_mm=joint.distal_room_mm,
                  required_proximal_mm=goal.minimum_proximal_support_mm,
                  required_distal_mm=goal.minimum_distal_support_mm)
    if max(static.values()) > 0.0:
        constraints = {**static, **{name: UNSWEPT for name in MOTION_CONSTRAINTS}}
        return ParallelEvaluation(constraints=MappingProxyType(constraints), **common)

    v = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    motion, fields = measure_motion(geo, joint, config, _margins, ma=1.0, l1_mm=v["app_x_mm"],
                                    l2_mm=v["ac_length_mm"] - v["bpp_from_a_mm"], distal_start_u_mm=v["bpp_from_a_mm"],
                                    hidden=belt_hidden(geo) if goal.include_linkage_occlusion else None)
    return ParallelEvaluation(constraints=MappingProxyType({**static, **motion}), **fields, **common)


def _margins(geo: ParallelGeometry, phi: np.ndarray, config: ParallelConfig,
             until_unsafe: bool = False, *, include_static_pairs: bool = True) -> dict[tuple[str, str], np.ndarray]:
    """Per-sample margin of every checked pair (``solids.margins``, the study's checks)."""

    return solids.margins(geo, phi, joint_checks(config.base), until_unsafe, include_static_pairs=include_static_pairs)


__all__ = ["CONSTRAINT_NAMES", "ParallelEvaluation", "belt_hidden", "evaluate", "geometry",
           "mount_depth_violation"]
