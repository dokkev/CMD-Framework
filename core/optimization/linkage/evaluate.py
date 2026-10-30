"""Evaluate one linkage design against the study's requirements.

What the finger is and does is the morphology's (``core.morphology.linkage``:
its kinematics, packaging, measures and swept motion); this module holds
those measurements to the goal's bounds as constraints.

The PIP servo sits on ``A'`` and drives the coupler ``A'-B``. ``phi = q_pip +
q_dip`` is the distal link's rotation relative to the proximal link (180 deg:
``BC`` antiparallel to ``OA``). The contact pads hang palmar of ``O-A''``
(proximal body) and ``B''-C`` (distal body), solids whose geometry and
collision pairs ``core.morphology.linkage.packaging`` owns.

- **Static checks first** (the reference pose and the packaging alone): the
  servos clear each other, the pins their neighbours, ``|OC|`` and ``|OA|``
  their bounds, the PIP servo the proximal pad, the pads their mounting
  lines' room and every checked pair. A design that fails one is not swept.
- **The motion** is swept by the PIP servo from extension, and by the PIP
  into hyperextension. It is safe while the four-bar closes, the transmission
  angle at ``B'`` stays in its range, the DIP and the PIP keep turning with
  the servo, the ``B'`` pin clears the PIP servo (when checked) and, from
  extension on, the pads clear the parts of other bodies (the MCP servo at
  every MCP flexion up to its max). It must stay safe on ``[0, phi_req]``
  (``rom_deg``) and back to the hyperextension stopper, and the fingertip
  ``C`` must stay ahead of ``O`` (MCP at 0) on ``[0, phi_req]``.
- **Measured on ``[0, phi_req]``:** ``k_max``, the effective pads (each
  pad's face the links leave free) and the least pad gap. ``phi_max`` is the
  most flexion every check still allows (``C_x`` included), for ``curl``.

On the servo grid an interval is taken to the first sample at or past its end,
so a design is judged on slightly more than ``[0, phi_req]``.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from core.morphology.linkage import FingerLinkage, FingerLinkageParameters
from core.morphology.linkage.measures import finger_length_mm, linkage_icr_x, overhang_mm, pad_lengths
from core.morphology.linkage.motion import MotionSettings, sweep_motion
from core.morphology.linkage.packaging import (
    PAD_PARTS,
    assembly,
    distal_drop,
    least_margin_mm,
    pad_spans,
    pip_footprint,
    proximal_drop,
    reference_poses,
)
from core.morphology.packaging.assembly import margins
from core.morphology.packaging.shapes import ServoFootprint

from core.optimization.linkage.config import LinkageConfig


STATIC_CONSTRAINTS: tuple[str, ...] = (
    "valid_reference",
    "mcp_servo_mm",
    "pivot_pin_mm",
    "pin_spacing_mm",
    "extended_length_mm",
    "proximal_length_mm",
    "pip_servo_pad_mm",
    "proximal_support_mm",
    "distal_support_mm",
    "dorsal_space_mm",
    "pad_collision_mm",
)
"""Constraints of the reference geometry alone, checked before any sweep."""
MOTION_CONSTRAINTS: tuple[str, ...] = ("rom_deg", "hyperextension_deg", "curl_tip_x_mm", "pad_usable_mm",
                                       "torque_ratio")
"""``rom_deg``: how far short of ``phi_req`` the safe motion stops (``<= 0``:
it reaches it); the hyperextension still missing; how far ``C`` passes behind
``O``; how far either pad's effective length (the face no link crosses on
``[0, phi_req]``, or with ``packaging.links_clear_pads`` the face outside
where the two faces cross) falls short of its least length: the module must
be usable;
how far ``k_max`` exceeds its limit (``config.torque_ratio_max``, 0 without one)."""
CONSTRAINT_NAMES: tuple[str, ...] = STATIC_CONSTRAINTS + MOTION_CONSTRAINTS
INVALID = 1.0e3
"""Violation assigned to every other constraint of an invalid reference design."""
UNSWEPT = 1.0e2
"""Violation assigned to the motion constraints of a design that fails a static one."""


@dataclass(frozen=True)
class LinkageEvaluation:
    """Constraints, objectives and diagnostics of one design."""

    parameters: FingerLinkageParameters
    constraints: Mapping[str, float]
    worst_transmission_quality: float = 0.0
    ab_mm: float = math.nan
    finger_length_mm: float = math.nan
    """``|OC|`` in the extended pose."""
    proximal_length_mm: float = math.nan
    """``|OA|``."""
    required_rom_rad: float = math.nan
    phi_safe_rad: float = 0.0
    """How far the safe motion reaches (at most pi), ``C_x`` aside."""
    phi_max_rad: float = math.nan
    """The most flexion every check still allows, ``C_x`` included (at most pi)."""
    stop_reason: str = ""
    """What ends the safe motion: ``"curl"`` at 180 deg, an unsafe condition
    (``core.morphology.linkage.motion.STOP_REASONS``), ``"closure"`` or ``"horizon"``."""
    collision_pair: str = ""
    """With ``"pad_collision"``, the first pair that collides, ``"first/second"``."""
    usable_pip_rad: float = 0.0
    usable_servo_rad: float = 0.0
    """PIP and servo rotation at ``phi_req`` (the end of the measured interval)."""
    curl_tip_x_mm: float = math.nan
    least_tip_x_mm: float = math.nan
    """``x`` of ``C`` at ``phi_req``, and its least on ``[0, phi_req]``, MCP at 0."""
    min_margin_rad: float = math.nan
    """Least transmission margin (from 0 or 180 deg) at ``B'``, hyperextension to ``phi_req``."""
    toggle_margin_rad: float = math.nan
    """Least margin from 0 or 180 deg of the angle at ``B`` (the limit position) over the same range."""
    hyperextension_phi_rad: float = math.nan
    driver_link_servo_clearance_mm: float = math.nan
    """The driver link ``A-B'`` to the PIP servo in the side view (reported;
    its plates straddle the servo in other layers)."""
    servo_lowest_z_mm: float = math.nan
    closes_to_rad: float = math.nan
    lever_max_mm: float = math.nan
    torque_ratio_max: float = math.nan
    """``k_max``: the worst ``d(phi)/d(theta_servo)`` on ``[0, phi_req]``."""
    torque_ratio_min: float = math.nan
    servo_travel_rad: float = math.nan
    pad_clearance_mm: float = math.nan
    """Least pad margin on ``[0, phi_req]``, less the pairs set by definition."""
    proximal_support_length_mm: float = math.nan
    distal_support_length_mm: float = math.nan
    """Each pad's mounting length ``a-b`` (``packaging.pad_spans``)."""
    proximal_room_mm: float = math.nan
    distal_room_mm: float = math.nan
    """Each support's length, from the pad's outer end to ``A''`` / ``B''``:
    the longest module it can carry (``PadSpan.room_mm``)."""
    proximal_face_mm: float = math.nan
    distal_face_mm: float = math.nan
    """Each pad's palmar face, its ``a-b`` less the chamfer."""
    proximal_usable_length_mm: float = math.nan
    distal_usable_length_mm: float = math.nan
    """Each pad's length the links leave free from its outer end, the least on
    ``[0, phi_req]`` (``packaging.usable_pad_lengths``); with the links bent
    around the pads (``packaging.links_clear_pads``), the length outside where
    the two faces cross (``packaging.face_crossing_lengths``)."""
    required_proximal_mm: float = math.nan
    required_distal_mm: float = math.nan
    proximal_depth_mm: float = math.nan
    distal_depth_mm: float = math.nan
    """How far each pad's contact face lies palmar of its link's line through
    the pivots (``O-A``, ``B-C``): the finger's thickness there, measured from
    the centres of rotation."""
    bound_hits: tuple[str, ...] = ()
    """Search limits the design sits at (set by the mechanism, which knows the box)."""

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
    def support_balance_mm(self) -> float:
        """How much longer than its module the shorter-spare support is:
        ``min(S_p - l_p, S_d - l_d)`` (the accommodation study's reference layouts)."""

        return min(self.proximal_room_mm - self.proximal_support_length_mm,
                   self.distal_room_mm - self.distal_support_length_mm)

    @property
    def proximal_effective_mm(self) -> float:
        """The proximal pad's face the links leave free all over ``[0, phi_req]``."""

        return min(self.proximal_usable_length_mm, self.proximal_face_mm)

    @property
    def distal_effective_mm(self) -> float:
        return min(self.distal_usable_length_mm, self.distal_face_mm)

    @property
    def pad_coverage(self) -> float:
        """``(L_p,eff + L_d,eff) / |OC|``: the share of the extended finger the
        pads' effective faces cover (the objective)."""

        return (self.proximal_effective_mm + self.distal_effective_mm) / self.finger_length_mm

    @property
    def mounted_coverage(self) -> float:
        """``(L_p + L_d) / |OC|`` with the mounting lengths (reported)."""

        return (self.proximal_support_length_mm + self.distal_support_length_mm) / self.finger_length_mm

    @property
    def curl_deg(self) -> float:
        """``180 deg - phi_max``."""

        return 180.0 - math.degrees(self.phi_max_rad)

    @property
    def phi_max_stop(self) -> str:
        """What ends ``phi_max``: ``C`` passing behind ``O`` (``tip_x``), else the safe motion's stop."""

        return "tip_x" if self.phi_max_rad < self.phi_safe_rad - 1e-12 else self.stop_reason


def evaluate(parameters: FingerLinkageParameters, config: LinkageConfig) -> LinkageEvaluation:
    """Evaluate one design (see the module docstring). A design that fails a
    static check gets :data:`UNSWEPT` for the motion constraints; an invalid
    reference pose :data:`INVALID` for every other constraint."""

    packaging = config.packaging
    common = dict(
        ab_mm=math.hypot(parameters.b_x_mm - parameters.a_x_mm, parameters.b_z_mm - parameters.a_z_mm),
        finger_length_mm=finger_length_mm(parameters),
        proximal_length_mm=math.hypot(parameters.a_x_mm, parameters.a_z_mm),
        required_rom_rad=config.required_rom_rad,
        required_proximal_mm=config.minimum_proximal_support_mm,
        required_distal_mm=config.minimum_distal_support_mm,
        proximal_depth_mm=proximal_drop(packaging, parameters) + packaging.proximal_pad.height_mm,
        distal_depth_mm=distal_drop(packaging, parameters) + packaging.distal_pad.height_mm,
        **pad_lengths(parameters, packaging),
    )
    try:
        linkage = FingerLinkage(parameters)
    except ValueError:  # the reference pose is a dead point
        constraints = {name: INVALID for name in CONSTRAINT_NAMES}
        constraints["valid_reference"] = 1.0
        return LinkageEvaluation(parameters, MappingProxyType(constraints), **common)

    footprint = pip_footprint(packaging, linkage)
    static = _static_constraints(linkage, footprint, config, common)
    if max(static.values()) > 0.0:
        constraints = {**static, **{name: UNSWEPT for name in MOTION_CONSTRAINTS}}
        return LinkageEvaluation(parameters, MappingProxyType(constraints),
                                 servo_lowest_z_mm=footprint.lowest_z_mm, **common)

    motion = sweep_motion(linkage, packaging, motion_settings(config), config.required_rom_rad, footprint,
                          config.integrity_depth_mm)
    # Hyperextension: the PIP must reach the stopper safely; short of it, the
    # violation is the PIP still missing.
    hyper_violation = max(0.0, math.degrees(config.hyperextension_rad + motion.deepest_pip_rad))
    if hyper_violation < 1e-9:
        hyper_violation = 0.0
    free = motion.fold_free
    if not config.count_fold:  # a module clear through the motion counts whole
        free = {side: math.inf for side in free}
    if config.integrity_depth_mm is not None:  # linkage-v5: no solid within the depth in front, at any pose
        kept = motion.integrity_free
        free = kept if packaging.links_clear_pads else {side: min(free[side], kept[side]) for side in free}

    constraints = {
        **static,
        "rom_deg": math.degrees(config.required_rom_rad - motion.phi_safe_rad),
        "hyperextension_deg": hyper_violation,
        "curl_tip_x_mm": -motion.least_tip_x_mm,
        "pad_usable_mm": max(
            config.minimum_proximal_support_mm - min(free["proximal_pad"], common["proximal_face_mm"]),
            config.minimum_distal_support_mm - min(free["distal_pad"], common["distal_face_mm"])),
        "torque_ratio": 0.0 if config.torque_ratio_max is None else motion.torque_ratio_max - config.torque_ratio_max,
    }
    return LinkageEvaluation(
        parameters=parameters,
        constraints=MappingProxyType(constraints),
        worst_transmission_quality=motion.worst_transmission_quality,
        phi_safe_rad=motion.phi_safe_rad,
        phi_max_rad=motion.phi_max_rad,
        stop_reason=motion.stop_reason,
        collision_pair=motion.collision_pair,
        usable_pip_rad=motion.usable_pip_rad,
        usable_servo_rad=motion.usable_servo_rad,
        curl_tip_x_mm=motion.curl_tip_x_mm,
        least_tip_x_mm=motion.least_tip_x_mm,
        min_margin_rad=motion.min_margin_rad,
        toggle_margin_rad=motion.toggle_margin_rad,
        hyperextension_phi_rad=motion.hyperextension_phi_rad,
        driver_link_servo_clearance_mm=motion.driver_link_servo_clearance_mm,
        servo_lowest_z_mm=footprint.lowest_z_mm,
        closes_to_rad=motion.closes_to_rad,
        lever_max_mm=motion.lever_max_mm,
        torque_ratio_max=motion.torque_ratio_max,
        torque_ratio_min=motion.torque_ratio_min,
        servo_travel_rad=motion.servo_travel_rad,
        pad_clearance_mm=motion.pad_clearance_mm,
        proximal_usable_length_mm=free["proximal_pad"],
        distal_usable_length_mm=free["distal_pad"],
        **common,
    )


def motion_settings(config: LinkageConfig) -> MotionSettings:
    """How the study sweeps a design's motion and what it checks along it."""

    return MotionSettings(
        servo_samples_rad=config.servo_samples_rad,
        servo_step_rad=config.servo_step_rad,
        hyperextension_rad=config.hyperextension_rad,
        mcp_samples_rad=config.mcp_samples_rad,
        transmission_angle_range_rad=config.transmission_angle_range_rad,
        dip_backoff_tolerance_rad=config.dip_backoff_tolerance_rad,
        link_radius_mm=config.link_radius_mm,
        proximal_carrier_height_mm=config.proximal_carrier_height_mm,
    )


def dorsal_space_violation(linkage: FingerLinkage, config: LinkageConfig) -> float:
    """How far (mm) the extended finger rises past ``goal.link.dorsal_space_mm`` above ``O``'s line (0: none set)."""

    if config.dorsal_space_mm is None:
        return 0.0
    from core.morphology.contact.loss import dorsal_space_mm, linkage_scene  # only with a dorsal space set

    b, c, bp = (np.array([[point.x_mm, point.z_mm]]) for point in (linkage.b, linkage.c, linkage.bp))
    built = linkage_scene(linkage, config.packaging, b, c, bp, [0.0], config.proximal_carrier_height_mm)
    return dorsal_space_mm(built, config.link_radius_mm or 0.0) - config.dorsal_space_mm


def _static_constraints(linkage: FingerLinkage, footprint: ServoFootprint, config: LinkageConfig,
                        common: Mapping[str, float]) -> dict[str, float]:
    """Constraints of the reference geometry and the packaging alone, in three
    stages by cost: lengths and pins, then the servos, then the pads' polygons.
    A stage that fails leaves the later ones unchecked, at :data:`UNSWEPT`
    (a feasible design checks them all)."""

    packaging = config.packaging
    parameters = linkage.parameters

    # Every gap is the packaging's (its rigid bodies and their keep-outs), in the extended pose.
    built, rest = assembly(linkage, packaging), reference_poses(linkage, config.mcp_samples_rad)
    least = lambda *pairs: least_margin_mm(built, rest, packaging, *pairs)  # noqa: E731

    # 1. Lengths, pins and the pads' room: closed forms.
    # Two pins on one link: A-B' (the driver link) and B'-B; their housings may touch.
    spacing = -least(("a_pin", "bp_pin"), ("bp_pin", "b_pin"))
    length, length_low, length_high = common["finger_length_mm"], *config.extended_length_bounds_mm
    proximal_length, (proximal_low, proximal_high) = common["proximal_length_mm"], config.proximal_length_bounds_mm
    proximal_violations = [value for value in (
        None if proximal_low is None else proximal_low - proximal_length,
        None if proximal_high is None else proximal_length - proximal_high) if value is not None]
    spans = pad_spans(parameters, packaging)
    proximal, distal = spans["proximal_pad"], spans["distal_pad"]
    if packaging.pads_off_pins:  # each pad clear of its own body's pin housings, wherever it ends
        topology = (-least(("proximal_pad", "a_pin"), ("proximal_pad", "ap_pin")),
                    -least(("distal_pad", "b_pin"), ("distal_pad", "bp_pin")))
        icr_x = linkage_icr_x(linkage)
        if config.proximal_pad_past_icr_max_mm is not None and icr_x is not None:  # not past the fold
            topology = (max(topology[0], parameters.app_x_mm - icr_x - config.proximal_pad_past_icr_max_mm),
                        topology[1])
        topology = tuple(max(value, over) for value, over in zip(topology, overhang_violations(linkage, config)))
    else:  # the pads run from A and B outward
        topology = (parameters.a_x_mm - parameters.app_x_mm, parameters.bpp_x_mm - parameters.b_x_mm)
    span, (span_low, span_high) = linkage.a.distance_to(linkage.b), config.joint_span_bounds_mm
    span_violations = [value for value in (None if span_low is None else span_low - span,
                                           None if span_high is None else span - span_high) if value is not None]
    # |OA| and BC as shares of |OC|, their violations in mm.
    span_violations += config.length_ratio_violations(proximal_length, parameters.bc_length_mm, span, length)
    # The extended centre of rotation within its share of |OC| (the bending-point control).
    span_violations += config.icr0_violations(linkage_icr_x(linkage), length)
    # A clears the keep-out at the servo's shaft A'.
    pivot_pin = (0.0 if packaging.allow_collision_pip_servo_to_joint_a else
                 -least(("ap_pin" if packaging.pip_servo_on_a else "a_pin", "pip_shaft")))
    constraints = {
        "valid_reference": 0.0,
        "pivot_pin_mm": pivot_pin,
        "pin_spacing_mm": spacing,
        "extended_length_mm": max(length_low - length, length - length_high),
        # |OA|, |AB| (extended) and the links' shares of |OC| within their bounds.
        "proximal_length_mm": max(proximal_violations + span_violations, default=0.0),
        # The pads' mounting lines run from the pivots outward, with room for
        # the pads: at least the minimum, at most the cap.
        "proximal_support_mm": max(topology[0],
                                   proximal.length_mm - proximal.limit_mm,
                                   config.minimum_proximal_support_mm - proximal.length_mm,
                                   proximal.length_mm - proximal.room_mm),
        "distal_support_mm": max(topology[1],
                                 distal.length_mm - distal.limit_mm,
                                 config.minimum_distal_support_mm - distal.length_mm,
                                 distal.length_mm - distal.room_mm),
        "dorsal_space_mm": dorsal_space_violation(linkage, config),
    }
    servos, pads = ("mcp_servo_mm", "pip_servo_pad_mm"), ("pad_collision_mm",)
    if max(constraints.values()) > 0.0:
        return {**constraints, **{name: UNSWEPT for name in servos + pads}}

    # 2. The servos: the two bodies, A against the PIP servo, the PIP servo
    # and the proximal pad.
    servo_rest = reference_poses(linkage, np.asarray(config.mcp_servo_angles_rad))
    constraints["mcp_servo_mm"] = -least_margin_mm(built, servo_rest, packaging, ("mcp_servo", "pip_servo"))
    if packaging.a_pin_servo_check:
        constraints["pivot_pin_mm"] = max(pivot_pin, -least(("a_pin", "pip_servo")))
    # The PIP servo and the proximal pad (one body, still kept apart), and the
    # older rule: the servo dorsal of the OA line.
    interface = -footprint.lowest_z_mm if packaging.pip_servo_above_oa else 0.0
    if packaging.pip_servo_pad_check:
        interface = max(interface, -least(("pip_servo", "proximal_pad")))
    constraints["pip_servo_pad_mm"] = interface
    if max(constraints.values()) > 0.0:
        return {**constraints, **{name: UNSWEPT for name in pads}}

    # 3. The pads against every checked part, extended.
    found = margins(built, rest, packaging, built.layout.pairs_with(PAD_PARTS))
    constraints["pad_collision_mm"] = -min(float(margin.min()) for margin in found.values())
    return constraints


def overhang_violations(linkage: FingerLinkage, config: LinkageConfig) -> tuple[float, float]:
    """How far (mm) each pad runs past its own body (``measures.overhang_mm``)
    beyond ``goal.pads.*.overhang_max_mm`` (0 within it, or without a limit)."""

    over = overhang_mm(linkage, config.packaging)
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
