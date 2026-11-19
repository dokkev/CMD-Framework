"""Evaluate one CFL (linkage) design: the prototype formulation's constraints
and objective (``design/SSoT/problem_formulation.md`` A.4-A.5,
``code_formulation.md`` §5).

The geometry is the morphology's (``core.morphology.mechanism.linkage``);
this module holds it to the goal.

- **Static checks first** (the reference pose alone, closed forms): the
  morphology bounds (``bounds_*``: finger length, proportions, each pad's
  length and room, the mounting policy's depths) and ``F_static``
  (``static_*``: the four-bar closes, the servos, pins and pads do not
  overlap). A design that fails one is not swept: its motion constraints are
  :data:`UNSWEPT`.
- **The motion** (``linkage.motion.sweep_flexion``): the flexion ``q_2`` on a
  uniform grid. ``F_motion`` asks every pose on ``[0, q_req]`` to be valid;
  the transmission angle at ``B'`` stays in its range there; the worst-case
  mechanical advantage ``MA_wc`` is at least ``ma_min`` when one is set.
- **The objective**: the mean cylinder-based contact pad coverage over
  ``Q_eval = {90 deg, (90 deg + q_req) / 2, q_req}``.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from core.evaluation import evaluation_poses_rad, worst_mechanical_advantage
from core.morphology.finger import Finger
from core.morphology.mechanism.linkage.measures import finger_length_mm, linkage_icr_x, overhang_mm, pad_lengths
from core.morphology.mechanism.linkage.model import four_bar
from core.evaluation import PoseCoverage, mean_coverage
from core.morphology.mechanism.linkage.motion import MotionSettings, coverage, sweep_flexion
from core.morphology.mechanism.linkage.packaging import (
    PADS,
    assembly,
    least_margin_mm,
    pad_offsets,
    pad_spans,
    pip_footprint,
    reference_poses,
)
from core.morphology.packaging.assembly import margins
from core.morphology.packaging.hardware import violation_mm
from core.morphology.packaging.shapes import ServoFootprint
from core.optimization.linkage.config import LinkageConfig

STATIC_CONSTRAINTS: tuple[str, ...] = (
    "bounds_finger_length_mm",
    "bounds_proportions_mm",
    "bounds_proximal_pad_mm",
    "bounds_distal_pad_mm",
    "bounds_mount_depth_mm",
    "bounds_dorsal_space_mm",
    "static_closure",
    "static_pin_spacing_mm",
    "static_pivot_pin_mm",
    "static_mcp_servo_mm",
    "static_pip_servo_pad_mm",
    "static_pad_collision_mm",
)
"""The morphology bounds (``bounds_*``) and ``F_static`` (``static_*``), checked before any sweep."""
MOTION_CONSTRAINTS: tuple[str, ...] = ("f_motion_deg", "gamma_range_deg", "ma_min")
"""``F_motion`` (how far short of ``q_req`` the valid motion stops), the
transmission angle range on ``[0, q_req]`` (the most it leaves the range,
deg) and ``MA_min - MA_wc`` (0 without ``ma_min``)."""
CONSTRAINT_NAMES: tuple[str, ...] = STATIC_CONSTRAINTS + MOTION_CONSTRAINTS
INVALID = 1.0e3
"""Violation assigned to every other constraint of a design whose reference pose does not close."""
UNSWEPT = 1.0e2
"""Violation assigned to the motion constraints of a design that fails a static one."""


@dataclass(frozen=True)
class LinkageEvaluation:
    """Constraints, objective and diagnostics of one design."""

    finger: Finger
    constraints: Mapping[str, float]
    finger_length_mm: float = math.nan
    """``L_f`` in the reference pose."""
    proximal_length_mm: float = math.nan
    """``|OA|``."""
    ab_mm: float = math.nan
    required_rom_rad: float = math.nan
    """``q_req``."""
    q_max_rad: float = 0.0
    """The most flexion with every pose valid and the transmission angle in range (A.5.3)."""
    valid_reach_rad: float = 0.0
    """The most flexion with every pose valid (``F_motion`` holds when it reaches ``q_req``)."""
    stop_reason: str = ""
    """What ends ``q_max`` (``linkage.motion.LinkageMotion.stop_reason``)."""
    collision_pair: str = ""
    """The first pair that overlaps along the motion, ``"first/second"``."""
    ma_wc: float = math.nan
    """``MA_wc``: the least ``|d theta / d q_2|`` on ``[0, q_req]``."""
    gamma_min_rad: float = math.nan
    gamma_max_rad: float = math.nan
    """The transmission angle's range over ``[0, q_req]``."""
    least_tip_x_mm: float = math.nan
    """``x`` of ``C`` at its least on ``[0, q_req]``."""
    pad_clearance_mm: float = math.nan
    """Least pad gap on ``[0, q_req]``, less the pairs set by definition."""
    poses: tuple[PoseCoverage, ...] = ()
    """The coverage at each evaluation pose ``Q_eval``."""
    servo_lowest_z_mm: float = math.nan
    proximal_support_length_mm: float = math.nan
    distal_support_length_mm: float = math.nan
    """Each pad's length ``l_i`` (``packaging.pad_spans``)."""
    proximal_room_mm: float = math.nan
    distal_room_mm: float = math.nan
    """Each support's length from the pad's outer end to ``A''`` / ``B''``."""
    required_proximal_mm: float = math.nan
    required_distal_mm: float = math.nan
    proximal_depth_mm: float = math.nan
    distal_depth_mm: float = math.nan
    """How far each pad's contact face lies palmar of its link's line (``O-A``, ``B-C``)."""
    bound_hits: tuple[str, ...] = ()
    """Search limits the design sits at (set by the mechanism, which knows the box)."""
    proximal_ratio_shortfall_mm: float = 0.0
    distal_ratio_shortfall_mm: float = 0.0

    @property
    def g(self) -> tuple[float, ...]:
        return tuple(self.constraints[name] for name in CONSTRAINT_NAMES)

    @property
    def max_violation(self) -> float:
        return max(0.0, *self.g)

    @property
    def feasible(self) -> bool:
        return self.max_violation <= 0.0

    @property
    def c_pad_mean(self) -> float:
        """``C_pad`` averaged over ``Q_eval`` (the objective; 0 when unswept)."""

        return mean_coverage(self.poses)

    @property
    def q_max_deg(self) -> float:
        return math.degrees(self.q_max_rad)


def evaluate(finger: Finger, config: LinkageConfig) -> LinkageEvaluation:
    """Evaluate one design (see the module docstring)."""

    packaging, joint = config.packaging, four_bar(finger)
    offsets = pad_offsets(finger, packaging)
    shortfalls = config.length_ratio_shortfalls(finger.proximal.length_mm, finger.distal.length_mm,
                                                finger_length_mm(finger))
    common = dict(
        proximal_ratio_shortfall_mm=shortfalls[0], distal_ratio_shortfall_mm=shortfalls[1],
        ab_mm=math.hypot(joint.b.x_mm - joint.a.x_mm, joint.b.z_mm - joint.a.z_mm),
        finger_length_mm=finger_length_mm(finger),
        proximal_length_mm=math.hypot(joint.a.x_mm, joint.a.z_mm),
        required_rom_rad=config.required_rom_rad,
        required_proximal_mm=config.minimum_proximal_support_mm,
        required_distal_mm=config.minimum_distal_support_mm,
        proximal_depth_mm=offsets["proximal_pad"] + packaging.proximal_pad.height_mm,
        distal_depth_mm=offsets["distal_pad"] + packaging.distal_pad.height_mm,
        **pad_lengths(finger, packaging),
    )
    try:
        joint.check()
    except ValueError:  # the reference pose is a dead point: no geometry
        constraints = {name: INVALID for name in CONSTRAINT_NAMES}
        constraints["static_closure"] = 1.0
        return LinkageEvaluation(finger, MappingProxyType(constraints), **common)

    footprint = pip_footprint(packaging, finger)
    static = _static_constraints(finger, footprint, config, common)
    if max(static.values()) > 0.0:
        constraints = {**static, **{name: UNSWEPT for name in MOTION_CONSTRAINTS}}
        return LinkageEvaluation(finger, MappingProxyType(constraints),
                                 servo_lowest_z_mm=footprint.lowest_z_mm, **common)

    q_req = config.required_rom_rad
    motion = sweep_flexion(finger, packaging, motion_settings(config))
    samples = motion.samples
    window = samples.window(q_req)
    ma_wc = worst_mechanical_advantage(samples, q_req)
    gamma = samples.gamma_rad[window]
    constraints = {
        **static,
        "f_motion_deg": samples.f_motion_violation_deg(q_req),
        "gamma_range_deg": samples.gamma_violation_deg(q_req),
        "ma_min": 0.0 if config.ma_min is None else config.ma_min - ma_wc,
    }
    return LinkageEvaluation(
        finger=finger,
        constraints=MappingProxyType(constraints),
        q_max_rad=samples.q_max_rad(),
        valid_reach_rad=samples.valid_reach_rad(),
        stop_reason=motion.stop_reason,
        collision_pair=motion.first_collision,
        ma_wc=ma_wc,
        gamma_min_rad=float(gamma.min()) if len(gamma) else math.nan,
        gamma_max_rad=float(gamma.max()) if len(gamma) else math.nan,
        least_tip_x_mm=float(motion.tip[window, 0].min()) if window.any() else math.nan,
        pad_clearance_mm=float(motion.reported_margin_mm[window].min()) if window.any() else math.nan,
        poses=coverage(finger, packaging, evaluation_poses_rad(q_req), config.include_linkage_occlusion),
        servo_lowest_z_mm=footprint.lowest_z_mm,
        **common,
    )


def motion_settings(config: LinkageConfig) -> MotionSettings:
    """How a design's flexion is sampled and what is checked along it."""

    return MotionSettings(
        step_rad=config.flexion_step_rad,
        transmission_angle_range_rad=config.transmission_angle_range_rad,
        mcp_samples_rad=config.mcp_samples_rad,
        link_radius_mm=config.link_radius_mm,
    )


def dorsal_space_violation(finger: Finger, config: LinkageConfig) -> float:
    """How far (mm) the extended finger rises past ``goal.link.dorsal_space_mm`` above ``O``'s line (0: none set)."""

    if config.dorsal_space_mm is None:
        return 0.0
    from core.morphology.contact.loss import dorsal_space_mm, linkage_scene  # only with a dorsal space set

    b, c, bp = (np.array([[point.x_mm, point.z_mm]]) for point in (finger.joint.b, finger.tip, finger.joint.bp))
    built = linkage_scene(finger, config.packaging, b, c, bp, [0.0])
    return dorsal_space_mm(built, config.link_radius_mm or 0.0) - config.dorsal_space_mm


def mount_depth_violation(finger: Finger, config: LinkageConfig) -> float:
    """How far the pads' depths are from the mounting policy's (mm; 0 when they are searched)."""

    if config.mount_policy == "searched":
        return 0.0
    from core.optimization.mounting import depth_violation_mm, linkage_mounting

    return depth_violation_mm(linkage_mounting(finger, config), finger.pad_offsets_mm, config)


def _static_constraints(finger: Finger, footprint: ServoFootprint, config: LinkageConfig,
                        common: Mapping[str, float]) -> dict[str, float]:
    """Constraints of the reference geometry and the packaging alone, in three
    stages by cost: lengths and pins, then the servos, then the pads' polygons.
    A stage that fails leaves the later ones unchecked, at :data:`UNSWEPT`
    (a feasible design checks them all)."""

    packaging = config.packaging

    # Every gap is the packaging's (its rigid bodies and their keep-outs), in the extended pose.
    built, rest = assembly(finger, packaging), reference_poses(finger, config.mcp_samples_rad)
    # A keep-out's violation: its least margin, negated (0 within rounding of touching).
    least = lambda *pairs: least_margin_mm(built, rest, packaging, *pairs)  # noqa: E731
    short = lambda *pairs: violation_mm(least(*pairs))  # noqa: E731

    # 1. Lengths, pins and the pads' room: closed forms.
    # Two pins on one link: A-B' (the driver link) and B'-B; their housings may touch.
    spacing = short(("a_pin", "bp_pin"), ("bp_pin", "b_pin"))
    length, length_low, length_high = common["finger_length_mm"], *config.extended_length_bounds_mm
    proximal_length, (proximal_low, proximal_high) = common["proximal_length_mm"], config.proximal_length_bounds_mm
    proximal_violations = [value for value in (
        None if proximal_low is None else proximal_low - proximal_length,
        None if proximal_high is None else proximal_length - proximal_high) if value is not None]
    spans = pad_spans(finger, packaging)
    proximal, distal = spans["proximal_pad"], spans["distal_pad"]
    if packaging.pads_off_pins:  # each pad clear of its own body's pin housings, wherever it ends
        topology = (short(("proximal_pad", "a_pin"), ("proximal_pad", "ap_pin")),
                    short(("distal_pad", "b_pin"), ("distal_pad", "bp_pin")))
        icr_x = linkage_icr_x(finger)
        if config.proximal_pad_past_icr_max_mm is not None and icr_x is not None:  # not past the fold
            topology = (max(topology[0], finger.proximal.end_mm - icr_x - config.proximal_pad_past_icr_max_mm),
                        topology[1])
        topology = tuple(max(value, over) for value, over in zip(topology, overhang_violations(finger, config)))
    else:  # the pads run from A and B outward
        topology = (finger.joint.a.x_mm - finger.proximal.end_mm, finger.distal.start_mm)
    span, (span_low, span_high) = finger.joint.a.distance_to(finger.joint.b), config.joint_span_bounds_mm
    span_violations = [value for value in (None if span_low is None else span_low - span,
                                           None if span_high is None else span - span_high) if value is not None]
    # The segments (|OA''|, |B''C|) and |AB| as shares of |OC|, their violations in mm.
    span_violations += config.length_ratio_violations(finger.proximal.length_mm, finger.distal.length_mm, span, length)
    # The extended centre of rotation within its share of |OC| (the bending-point control).
    span_violations += config.icr0_violations(linkage_icr_x(finger), length)
    # A clears the keep-out at the servo's shaft A'.
    pivot_pin = (0.0 if packaging.allow_collision_pip_servo_to_joint_a else
                 short(("a_pin", "pip_shaft")))
    constraints = {
        "static_closure": 0.0,
        "static_pivot_pin_mm": pivot_pin,
        "static_pin_spacing_mm": spacing,
        "bounds_finger_length_mm": max(length_low - length, length - length_high),
        # |OA|, |AB| (extended) and the links' shares of |OC| within their bounds.
        "bounds_proportions_mm": max(proximal_violations + span_violations, default=0.0),
        # The pads' mounting lines run from the pivots outward, with room for
        # the pads: at least the minimum, at most the cap.
        "bounds_proximal_pad_mm": max(topology[0],
                                   proximal.length_mm - proximal.limit_mm,
                                   config.minimum_proximal_support_mm - proximal.length_mm,
                                   proximal.length_mm - proximal.room_mm),
        "bounds_distal_pad_mm": max(topology[1],
                                 distal.length_mm - distal.limit_mm,
                                 config.minimum_distal_support_mm - distal.length_mm,
                                 distal.length_mm - distal.room_mm),
        "bounds_dorsal_space_mm": dorsal_space_violation(finger, config),
        "bounds_mount_depth_mm": mount_depth_violation(finger, config),
    }
    servos, pads = ("static_mcp_servo_mm", "static_pip_servo_pad_mm"), ("static_pad_collision_mm",)
    if max(constraints.values()) > 0.0:
        return {**constraints, **{name: UNSWEPT for name in servos + pads}}

    # 2. The servos: the two bodies, A against the PIP servo, the PIP servo
    # and the proximal pad.
    servo_rest = reference_poses(finger, np.asarray(config.mcp_servo_angles_rad))
    constraints["static_mcp_servo_mm"] = -least_margin_mm(built, servo_rest, packaging, ("mcp_servo", "pip_servo"))
    if packaging.a_pin_servo_check:
        constraints["static_pivot_pin_mm"] = max(pivot_pin, short(("a_pin", "pip_servo")))
    # The PIP servo and the proximal pad (one body, still kept apart), and the
    # older rule: the servo dorsal of the OA line.
    interface = -footprint.lowest_z_mm if packaging.pip_servo_above_oa else 0.0
    if packaging.pip_servo_pad_check:
        interface = max(interface, short(("pip_servo", "proximal_pad")))
    constraints["static_pip_servo_pad_mm"] = interface
    if max(constraints.values()) > 0.0:
        return {**constraints, **{name: UNSWEPT for name in pads}}

    # 3. The pads against every checked part, extended.
    found = margins(built, rest, packaging, built.layout.pairs_with(PADS))
    constraints["static_pad_collision_mm"] = -min(float(margin.min()) for margin in found.values())
    return constraints


def overhang_violations(finger: Finger, config: LinkageConfig) -> tuple[float, float]:
    """How far (mm) each pad runs past its own body (``measures.overhang_mm``)
    beyond ``goal.pads.*.overhang_max_mm`` (0 within it, or without a limit)."""

    over = overhang_mm(finger, config.packaging)
    limits = (config.proximal_overhang_max_mm, config.distal_overhang_max_mm)
    return tuple(0.0 if limit is None else max(value - limit, 0.0) for value, limit in zip(over, limits))


__all__ = [
    "CONSTRAINT_NAMES",
    "INVALID",
    "MOTION_CONSTRAINTS",
    "STATIC_CONSTRAINTS",
    "UNSWEPT",
    "LinkageEvaluation",
    "evaluate",
    "motion_settings",
]
