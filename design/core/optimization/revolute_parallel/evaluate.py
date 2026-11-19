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

import functools
import math
from collections.abc import Sequence
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from core.geometry.intervals import IntervalSet, face_occluded_set
from core.morphology.mechanism.revolute.solids import contact_faces, crossing_free
from core.optimization.mechanism import shortfall_mm
from core.morphology.packaging.gaps import footprint_gaps
from core.morphology.mechanism.revolute_parallel import solids
from core.morphology.mechanism.revolute_parallel.solids import ParallelGeometry

from core.optimization.revolute.evaluate import (
    UNSWEPT,
    RevoluteEvaluation,
    _between_samples,
    hyperextension_violation,
    joint_checks,
    robust_usable_lengths,
    segment_lengths,
    usable_lengths,
)
from core.optimization.revolute_parallel.config import VARIABLE_NAMES, ParallelConfig


STATIC_CONSTRAINTS = ("extended_length_mm", "proximal_length_mm", "mcp_servo_mm", "pip_servo_pad_mm",
                      "a_pin_servo_mm", "proximal_support_mm", "distal_support_mm", "mount_depth_mm",
                      "pad_collision_mm")
MOTION_CONSTRAINTS = ("rom_deg", "hyperextension_mm", "curl_tip_x_mm", "pad_usable_mm")
CONSTRAINT_NAMES = STATIC_CONSTRAINTS + MOTION_CONSTRAINTS


def geometry(x: Sequence[float], config: ParallelConfig) -> ParallelGeometry:
    """The solids of a design vector, with the study's packaging and supports."""

    return solids.geometry(x, config.goal.packaging, config.pulley_radius_mm)


@dataclass(frozen=True)
class ParallelEvaluation(RevoluteEvaluation):
    """Constraints, objectives and diagnostics, named as the linkage's and the 2R's."""

    @property
    def g(self) -> tuple[float, ...]:
        return tuple(self.constraints[name] for name in CONSTRAINT_NAMES)


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


def belt_kept_lengths(geo: ParallelGeometry, phi_rad: float, free: tuple[float, float], goal) -> tuple[float, float]:
    """Exact usable length at ``phi_rad`` after belt and pulley occlusion (:mod:`core.geometry.intervals`)."""

    p_face, p_norm, d_face, d_norm = contact_faces(geo.joint, np.array([phi_rad], dtype=float))
    strands, p_disks, d_disks = _belt_occluders(geo)
    p_len = float(np.linalg.norm(p_face[1] - p_face[0]))
    d_len = float(np.linalg.norm(d_face[0, 1] - d_face[0, 0]))
    p_base = IntervalSet.closed(0.0, min(float(free[0]), p_len))
    d_base = IntervalSet.closed(0.0, min(float(free[1]), d_len))
    p_occ = face_occluded_set(p_face, p_norm, segments=strands, disks=p_disks)
    d_occ = face_occluded_set(d_face[0], d_norm[0], disks=d_disks)
    return p_base.difference(p_occ).measure, d_base.difference(d_occ).measure


def _belt_robust_lengths(geo: ParallelGeometry, phi_window: np.ndarray, goal) -> tuple[float, float]:
    """Motion-robust usable lengths across ``phi_window`` with belt/pulley occlusion."""

    p_face, p_norm, d_faces, d_norms = contact_faces(geo.joint, phi_window)
    p_len = float(np.linalg.norm(p_face[1] - p_face[0]))
    d_len = float(np.linalg.norm(d_faces[0, 1] - d_faces[0, 0])) if len(phi_window) else 0.0
    if goal.count_fold:
        fold = crossing_free(geo.joint, phi_window)
        p_lims = np.minimum(fold["proximal_pad"], p_len)
        d_lims = np.minimum(fold["distal_pad"], d_len)
    else:
        p_lims = np.full(len(phi_window), p_len)
        d_lims = np.full(len(phi_window), d_len)
    strands, p_disks, d_disks = _belt_occluders(geo)
    p_occ = face_occluded_set(p_face, p_norm, segments=strands, disks=p_disks)
    p_sets = [IntervalSet.closed(0.0, float(lim)).difference(p_occ) for lim in p_lims]
    d_sets = [
        IntervalSet.closed(0.0, float(d_lims[i])).difference(
            face_occluded_set(d_faces[i], d_norms[i], disks=d_disks)
        )
        for i in range(len(phi_window))
    ]
    return (
        functools.reduce(IntervalSet.intersection, p_sets).measure if p_sets else 0.0,
        functools.reduce(IntervalSet.intersection, d_sets).measure if d_sets else 0.0,
    )


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
        "extended_length_mm": max(length_low - joint.finger_length_mm, joint.finger_length_mm - length_high),
        "proximal_length_mm": max(oa, default=0.0),
        # At least its min, and no longer than its segment's room (a module of a set length, 2026-11-06).
        "proximal_support_mm": max(goal.minimum_proximal_support_mm - joint.proximal_length_mm,
                                   -joint.proximal_room_mm, joint.proximal_length_mm - joint.proximal_room_mm,
                                   x[VARIABLE_NAMES.index("app_x_mm")] - (joint.a_x_mm - config.joint_radius_mm)),
        "distal_support_mm": max(goal.minimum_distal_support_mm - joint.distal_length_mm, -joint.distal_room_mm,
                                 joint.distal_length_mm - joint.distal_room_mm,
                                 config.joint_radius_mm - x[VARIABLE_NAMES.index("bpp_from_a_mm")]),
        "mount_depth_mm": mount_depth_violation(x, config),
    }
    if max(static.values()) <= 0.0:
        mcp_angles = np.asarray(goal.mcp_servo_angles_rad)
        static["mcp_servo_mm"] = -(packaging.mcp_servo_gap_mm(geo.servo, mcp_angles) - clearance)
        static["pip_servo_pad_mm"] = (-(float(footprint_gaps(joint.proximal_pad[None], geo.servo, packaging)[0])
                                        - clearance) if packaging.pip_servo_pad_check else 0.0)
        # A's pin housing clear of the servo body, as the linkage's.
        static["a_pin_servo_mm"] = (-(geo.servo.distance_to_point(geo.a) - packaging.pivot_radius_mm("a")
                                      - clearance) if packaging.a_pin_servo_check else 0.0)
    if max(static.values()) <= 0.0:
        margins_at_rest = _margins(geo, np.zeros(1), config)
        static["pad_collision_mm"] = -min(float(margin[0]) for margin in margins_at_rest.values())
    for name in STATIC_CONSTRAINTS:
        static.setdefault(name, UNSWEPT)
    shortfalls = goal.length_ratio_shortfalls(*segment_lengths(x), joint.finger_length_mm)
    common = dict(x=tuple(float(value) for value in x), finger_length_mm=joint.finger_length_mm,
                  proximal_ratio_shortfall_mm=shortfalls[0], distal_ratio_shortfall_mm=shortfalls[1],
                  proximal_length_mm=joint.a_x_mm, required_rom_rad=goal.required_rom_rad,
                  proximal_support_length_mm=joint.proximal_length_mm,
                  distal_support_length_mm=joint.distal_length_mm,
                  proximal_depth_mm=joint.proximal_drop_mm + packaging.proximal_pad.height_mm,
                  distal_depth_mm=joint.distal_drop_mm + packaging.distal_pad.height_mm)
    if max(static.values()) > 0.0:
        constraints = {**static, **{name: UNSWEPT for name in MOTION_CONSTRAINTS}}
        return ParallelEvaluation(constraints=MappingProxyType(constraints), **common)

    phi = np.asarray(goal.servo_samples_rad)
    margins = _between_samples(
        geo,
        phi,
        _margins(geo, phi, config, until_unsafe=True, include_static_pairs=False),
        config,
        _margins,
    )
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
    tip_x = joint.tip(phi)[:, 0]
    failing = unsafe | (tip_x < 0.0)
    first_failing = int(np.flatnonzero(failing)[0]) if failing.any() else len(phi)
    phi_max = float(min(phi[first_failing - 1], math.pi)) if first_failing > 0 else 0.0
    proximal_usable, distal_usable = usable_lengths(geo, phi[window], goal, "revolute_parallel")
    if goal.include_linkage_occlusion and goal.integrity_depth_mm is None:
        proximal_usable, distal_usable = belt_kept_lengths(geo, float(phi[window][-1]),
                                                           (proximal_usable, distal_usable), goal)
        proximal_robust, distal_robust = _belt_robust_lengths(geo, phi[window], goal)
    else:
        proximal_robust, distal_robust = robust_usable_lengths(geo, phi[window], goal, "revolute_parallel")
    reported = [margin for pair, margin in margins.items() if pair != ("mcp_servo", "proximal_pad")]
    constraints = {
        **static,
        "rom_deg": math.degrees(goal.required_rom_rad - reach),
        "hyperextension_mm": hyperextension_violation(geo, config, _margins),
        "curl_tip_x_mm": -float(tip_x[window].min()),
        "pad_usable_mm": max(
            shortfall_mm(goal.minimum_proximal_support_mm, min(max(joint.proximal_length_mm, 0.0), proximal_usable)),
            shortfall_mm(goal.minimum_distal_support_mm, min(max(joint.distal_length_mm, 0.0), distal_usable))),
    }
    return ParallelEvaluation(
        constraints=MappingProxyType(constraints),
        phi_safe_rad=reach,
        phi_max_rad=phi_max,
        stop_reason=stop_reason,
        collision_pair=collision_pair,
        least_tip_x_mm=float(tip_x[window].min()),
        pad_clearance_mm=float(np.min(np.stack(reported), axis=0)[window].min()) if reported else math.inf,
        proximal_usable_length_mm=proximal_usable,
        distal_usable_length_mm=distal_usable,
        proximal_robust_usable_length_mm=proximal_robust,
        distal_robust_usable_length_mm=distal_robust,
        **common,
    )


def _margins(geo: ParallelGeometry, phi: np.ndarray, config: ParallelConfig,
             until_unsafe: bool = False, *, include_static_pairs: bool = True) -> dict[tuple[str, str], np.ndarray]:
    """Per-sample margin of every checked pair (``solids.margins``, the study's checks)."""

    return solids.margins(geo, phi, joint_checks(config.base), until_unsafe, include_static_pairs=include_static_pairs)


__all__ = ["CONSTRAINT_NAMES", "ParallelEvaluation", "belt_kept_lengths", "evaluate", "geometry",
           "mount_depth_violation"]
