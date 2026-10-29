"""Evaluate one linkage design.

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

from core.morphology.linkage import FingerLinkage, FingerLinkageParameters, PipSweep
from core.morphology.linkage.packaging import (
    ServoFootprint,
    _polygon_to_disk,
    distal_pad_polygon,
    contact_faces,
    fold_free_lengths,
    fixed_margin_pairs,
    pad_chamfer_mm,
    pad_margins,
    pad_spans,
    pip_servo_pad_gap_mm,
    proximal_pad_polygon,
    usable_pad_lengths,
)

from core.optimization.linkage.config import LinkageConfig
from core.optimization.linkage.search import extended_icr_x


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
STOP_REASONS = ("closure", "transmission", "dip_backoff", "pip_backoff", "dip_pin", "pad_collision")
"""The unsafe conditions, in the order they are reported."""


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
    (:data:`STOP_REASONS`), ``"closure"`` or ``"horizon"``."""
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


def finger_length_mm(parameters: FingerLinkageParameters) -> float:
    """``|OC|`` in the extended pose, ``C = (B_x + BC, B_z)``."""

    return math.hypot(parameters.b_x_mm + parameters.bc_length_mm, parameters.b_z_mm)


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
        proximal_depth_mm=packaging.proximal_drop(parameters) + packaging.proximal_pad.height_mm,
        distal_depth_mm=packaging.distal_drop(parameters) + packaging.distal_pad.height_mm,
        **_pad_lengths(parameters, packaging),
    )
    try:
        linkage = FingerLinkage(parameters)
    except ValueError:  # the reference pose is a dead point
        constraints = {name: INVALID for name in CONSTRAINT_NAMES}
        constraints["valid_reference"] = 1.0
        return LinkageEvaluation(parameters, MappingProxyType(constraints), **common)

    footprint = packaging.pip_footprint(linkage)
    static = _static_constraints(linkage, footprint, config, common)
    if max(static.values()) > 0.0:
        constraints = {**static, **{name: UNSWEPT for name in MOTION_CONSTRAINTS}}
        return LinkageEvaluation(parameters, MappingProxyType(constraints),
                                 servo_lowest_z_mm=footprint.lowest_z_mm, **common)

    step = config.servo_step_rad
    # The servo sense that flexes the PIP.
    sense = 1.0 if linkage.sweep_servo([0.0, step]).q_pip_rad[-1] >= 0.0 else -1.0
    forward = np.asarray(config.servo_samples_rad)
    sweep = linkage.sweep_servo(sense * forward)
    # Hyperextension is a PIP requirement: solved by the PIP, to the stopper exactly.
    count = int(math.ceil(config.hyperextension_rad / step - 1e-9))
    back = linkage.sweep_pip(-np.minimum(np.arange(count + 1) * step, config.hyperextension_rad))
    n_back = len(back.q_pip_rad) - 1  # back samples besides 0

    def joined(name: str) -> np.ndarray:
        """The back samples, then the forward sweep, in increasing flexion."""

        return np.concatenate((getattr(back, name)[n_back:0:-1], getattr(sweep, name)))

    unsafe = _unsafe_conditions(linkage, footprint, joined, sense, config)
    total = sweep.total_flexion_rad
    # The pads from extension up to the first other unsafe sample or the curl
    # (180 deg), as far as phi_max needs them; hyperextension is linkage range only.
    other = np.flatnonzero(np.any(np.stack(list(unsafe.values())), axis=0)[n_back:])
    end = int(other[0]) if other.size else len(total) - 1
    curled = np.flatnonzero(total >= math.pi)
    if curled.size:
        end = min(end, int(curled[0]))
    swept = slice(n_back, n_back + end + 1)
    margins = pad_margins(linkage, packaging, joined("b")[swept], joined("c")[swept],
                          joined("bp")[swept], config.mcp_samples_rad)
    pad = np.full(len(unsafe["closure"]), math.inf)
    pad[swept] = np.min(np.stack(list(margins.values())), axis=0)
    # The reported clearance leaves out pairs whose margin the design does not set.
    reported = [margin for pair, margin in margins.items() if pair not in fixed_margin_pairs(packaging)]
    reported_pad = np.full(len(unsafe["closure"]), math.inf)
    if reported:
        reported_pad[swept] = np.min(np.stack(reported), axis=0)
    unsafe["pad_collision"] = pad < 0.0
    unsafe["pad_collision"][n_back + end + 1:] = True  # past the pads' last sample nothing is known safe
    any_unsafe = np.any(np.stack(list(unsafe.values())), axis=0)

    # Hyperextension: the PIP must reach the stopper safely; short of it, the
    # violation is the PIP still missing.
    safe_back = 0
    while safe_back < n_back and not any_unsafe[n_back - 1 - safe_back]:
        safe_back += 1
    deepest = float(back.q_pip_rad[safe_back]) if not any_unsafe[n_back] else 0.0
    hyper_violation = max(0.0, math.degrees(config.hyperextension_rad + deepest))
    if hyper_violation < 1e-9:
        hyper_violation = 0.0

    # The safe motion from extension: up to the first unsafe sample (or the
    # sweep's end, where the four-bar stops closing).
    forward_unsafe = any_unsafe[n_back:]
    first_unsafe = int(np.flatnonzero(forward_unsafe)[0]) if forward_unsafe.any() else len(total)
    reach = float(min(total[first_unsafe - 1], math.pi)) if first_unsafe > 0 else 0.0
    stop_reason = (next(name for name in STOP_REASONS if unsafe[name][n_back + first_unsafe])
                   if first_unsafe < len(total) else ("closure" if len(total) < len(forward) else "horizon"))
    if reach >= math.pi - 1e-12:  # BC antiparallel to OA: nothing past it is needed
        stop_reason = "curl"
    collision_pair = ""
    if stop_reason == "pad_collision" and first_unsafe <= end:
        collision_pair = "/".join(next(pair for pair, margin in margins.items()
                                       if margin[first_unsafe] < 0.0))

    # [0, phi_req] on the grid: to the first sample at or past it (the whole
    # safe motion when it falls short).
    at = np.flatnonzero(total >= config.required_rom_rad - 1e-12)
    last = int(at[0]) if at.size else max(first_unsafe - 1, 0)
    window = np.zeros(len(total), dtype=bool)
    window[:last + 1] = True
    lever, ratio, travel = servo_lever(sweep, window)
    ratio_min = _least_torque_ratio(sweep, window)
    tip_x = sweep.c[:, 0]
    least_tip = float(tip_x[window].min())
    # phi_max: the first sample any check fails at, C_x included.
    failing = forward_unsafe | (tip_x < 0.0)
    first_failing = int(np.flatnonzero(failing)[0]) if failing.any() else len(total)
    phi_max = float(min(total[first_failing - 1], math.pi)) if first_failing > 0 else 0.0

    window_joined = np.concatenate((np.ones(n_back, dtype=bool), window))
    margin = joined("margin_at_bp_rad")[window_joined]
    toggle = joined("margin_at_b_rad")[window_joined]
    bp_range = joined("bp")[window_joined]
    a = np.array((linkage.a.x_mm, linkage.a.z_mm))
    link = footprint.distance_to_segments(np.broadcast_to(a, bp_range.shape), bp_range)
    if packaging.links_clear_pads:  # the links bent out of the way: each face less what the other hides
        free = fold_free_lengths(*contact_faces(linkage, packaging, sweep.b[window], sweep.c[window]))
    else:
        free = usable_pad_lengths(linkage, packaging, sweep.b[window], sweep.bp[window], sweep.c[window])

    constraints = {
        **static,
        "rom_deg": math.degrees(config.required_rom_rad - reach),
        "hyperextension_deg": hyper_violation,
        "curl_tip_x_mm": -least_tip,
        "pad_usable_mm": max(
            config.minimum_proximal_support_mm - min(float(free["proximal_pad"].min()), common["proximal_face_mm"]),
            config.minimum_distal_support_mm - min(float(free["distal_pad"].min()), common["distal_face_mm"])),
        "torque_ratio": 0.0 if config.torque_ratio_max is None else ratio - config.torque_ratio_max,
    }
    return LinkageEvaluation(
        parameters=parameters,
        constraints=MappingProxyType(constraints),
        worst_transmission_quality=float(np.sin(margin).min()),
        phi_safe_rad=reach,
        phi_max_rad=phi_max,
        stop_reason=stop_reason,
        collision_pair=collision_pair,
        usable_pip_rad=float(sweep.q_pip_rad[last]),
        usable_servo_rad=float(forward[last]),
        curl_tip_x_mm=float(tip_x[last]),
        least_tip_x_mm=least_tip,
        min_margin_rad=float(margin.min()),
        toggle_margin_rad=float(toggle.min()),
        hyperextension_phi_rad=float(back.total_flexion_rad[safe_back]),
        driver_link_servo_clearance_mm=float(link.min()) - packaging.body_clearance_mm,
        servo_lowest_z_mm=footprint.lowest_z_mm,
        closes_to_rad=float(sweep.q_pip_rad[-1]),
        lever_max_mm=lever,
        torque_ratio_max=ratio,
        torque_ratio_min=ratio_min,
        servo_travel_rad=travel,
        pad_clearance_mm=float(reported_pad[n_back:][window].min()),
        proximal_usable_length_mm=float(free["proximal_pad"].min()),
        distal_usable_length_mm=float(free["distal_pad"].min()),
        **common,
    )


def _static_constraints(linkage: FingerLinkage, footprint: ServoFootprint, config: LinkageConfig,
                        common: Mapping[str, float]) -> dict[str, float]:
    """Constraints of the reference geometry and the packaging alone, in three
    stages by cost: lengths and pins, then the servos, then the pads' polygons.
    A stage that fails leaves the later ones unchecked, at :data:`UNSWEPT`
    (a feasible design checks them all)."""

    packaging = config.packaging
    clearance = packaging.body_clearance_mm
    parameters = linkage.parameters
    radius = packaging.pivot_radius_mm

    # 1. Lengths, pins and the pads' room: closed forms.
    # Two pins on one body: A-B' (the driver link) and B'-B; their housings may touch.
    spacing = max(radius("a") + radius("bp") - linkage.a.distance_to(linkage.bp),
                  radius("bp") + radius("b") - linkage.bp.distance_to(linkage.b))
    length, length_low, length_high = common["finger_length_mm"], *config.extended_length_bounds_mm
    proximal_length, (proximal_low, proximal_high) = common["proximal_length_mm"], config.proximal_length_bounds_mm
    proximal_violations = [value for value in (
        None if proximal_low is None else proximal_low - proximal_length,
        None if proximal_high is None else proximal_length - proximal_high) if value is not None]
    spans = pad_spans(parameters, packaging)
    proximal, distal = spans["proximal_pad"], spans["distal_pad"]
    if packaging.pads_off_pins:  # each pad clear of its own body's pin housings, wherever it ends
        proximal_pad = proximal_pad_polygon(linkage, packaging)[None]
        b, c = np.array((linkage.b.x_mm, linkage.b.z_mm)), np.array((linkage.c.x_mm, linkage.c.z_mm))
        distal_pad = distal_pad_polygon(linkage, packaging, b, c)
        own = lambda pad, pins: min(float(_polygon_to_disk(pad, np.array([(point.x_mm, point.z_mm)]),  # noqa: E731
                                                            radius(name))[0]) for name, point in pins)
        topology = (-(own(proximal_pad, (("a", linkage.a), ("ap", linkage.ap))) - clearance),
                    -(own(distal_pad, (("b", linkage.b), ("bp", linkage.bp))) - clearance))
        icr_x = extended_icr_x(*((point.x_mm, point.z_mm) for point in (linkage.a, linkage.ap, linkage.b, linkage.bp)))
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
    # A clears the keep-out at the servo's shaft A'.
    pivot_pin = (0.0 if packaging.allow_collision_pip_servo_to_joint_a else
                 -(packaging.pivot_pin_gap_mm(linkage) - clearance))
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
    }
    servos, pads = ("mcp_servo_mm", "pip_servo_pad_mm"), ("pad_collision_mm",)
    if max(constraints.values()) > 0.0:
        return {**constraints, **{name: UNSWEPT for name in servos + pads}}

    # 2. The servos: the two bodies, A against the PIP servo, the PIP servo
    # and the proximal pad.
    mcp_angles = np.asarray(config.mcp_servo_angles_rad)
    # Footprints whose circumscribed circles are apart cannot touch.
    apart = math.hypot(*footprint.pivot) - footprint.reach_mm - packaging.mcp_footprint().reach_mm
    servo_gap = apart if apart > clearance else packaging.mcp_servo_gap_mm(footprint, mcp_angles)
    constraints["mcp_servo_mm"] = -(servo_gap - clearance)
    if packaging.a_pin_servo_check:
        a = np.array((linkage.a.x_mm, linkage.a.z_mm))
        constraints["pivot_pin_mm"] = max(pivot_pin, -(footprint.distance_to_point(a) - radius("a") - clearance))
    # The PIP servo and the proximal pad (one body, still kept apart), and the
    # older rule: the servo dorsal of the OA line.
    interface = -footprint.lowest_z_mm if packaging.pip_servo_above_oa else 0.0
    if packaging.pip_servo_pad_check:
        interface = max(interface, -(pip_servo_pad_gap_mm(linkage, packaging) - clearance))
    constraints["pip_servo_pad_mm"] = interface
    if max(constraints.values()) > 0.0:
        return {**constraints, **{name: UNSWEPT for name in pads}}

    # 3. The pads against every checked part, extended.
    reference = [np.array([[point.x_mm, point.z_mm]]) for point in (linkage.b, linkage.c, linkage.bp)]
    margins = pad_margins(linkage, packaging, *reference, config.mcp_samples_rad)
    constraints["pad_collision_mm"] = -min(float(margin.min()) for margin in margins.values())
    return constraints


def overhang_violations(linkage: FingerLinkage, config: LinkageConfig) -> tuple[float, float]:
    """How far (mm) each pad runs past its own body beyond ``goal.pads.*.overhang_max_mm``
    (0 within it, or without a limit), in the extended pose along the finger:
    the proximal pad's ``A''`` past the proximal body's front (``A``'s
    housing, the PIP servo's outline), the distal pad's ``B''`` behind the
    distal body's back (``B``'s and ``B'``'s housings)."""

    packaging, p = config.packaging, linkage.parameters
    radius = packaging.pivot_radius_mm
    front = max(p.a_x_mm + radius("a"), float(packaging.pip_footprint(linkage).outline[:, 0].max()))
    back = min(p.b_x_mm - radius("b"), p.bp.x_mm - radius("bp"))
    proximal, distal = config.proximal_overhang_max_mm, config.distal_overhang_max_mm
    return (0.0 if proximal is None else max(p.app_x_mm - front - proximal, 0.0),
            0.0 if distal is None else max(back - p.bpp_x_mm - distal, 0.0))


def _unsafe_conditions(linkage: FingerLinkage, footprint: ServoFootprint, joined, sense: float,
                       config: LinkageConfig) -> dict[str, np.ndarray]:
    """Per-sample unsafe flags along the joined sweep, in increasing flexion
    (the caller adds the pads). Backoffs are measured from the start of the
    sweep (deepest hyperextension); the servo angle is signed to rise with flexion."""

    packaging = config.packaging
    tolerance = config.dip_backoff_tolerance_rad
    low, high = config.transmission_angle_range_rad
    bp = joined("bp")
    angle = _angle_at(np.array((linkage.a.x_mm, linkage.a.z_mm)), bp, joined("b"))
    q_dip, q_pip, servo = joined("q_dip_rad"), joined("q_pip_rad"), sense * joined("servo_rad")
    return {
        "closure": np.zeros(len(angle), dtype=bool),
        "transmission": np.minimum(angle - low, high - angle) < 0.0,
        "dip_backoff": np.maximum.accumulate(q_dip) - q_dip > tolerance,
        # The PIP and the servo turn together.
        "pip_backoff": (np.maximum.accumulate(q_pip) - q_pip > tolerance)
        | (np.maximum.accumulate(servo) - servo > tolerance),
        # The B' pin may cross the servo body when they sit in different Y layers.
        "dip_pin": (np.zeros(len(angle), dtype=bool) if packaging.allow_collision_pip_servo_to_joint_bp
                    else footprint.distance_to_segments(bp, bp)
                    < packaging.pivot_radius_mm("bp") + packaging.body_clearance_mm),
    }


def _angle_at(a: np.ndarray, bp: np.ndarray, b: np.ndarray) -> np.ndarray:
    """The transmission angle at ``B'`` between ``B'->A`` and ``B'->B`` per sample, 0 to pi."""

    first, second = a - bp, b - bp
    cosine = np.sum(first * second, axis=1) / (np.linalg.norm(first, axis=1) * np.linalg.norm(second, axis=1))
    return np.arccos(np.clip(cosine, -1.0, 1.0))


def servo_lever(sweep: PipSweep, workspace: np.ndarray) -> tuple[float, float, float]:
    """Worst lever ``r``, worst torque ratio ``k`` and the servo travel over the workspace.

    A pad force ``F`` along the palmar normal ``n`` of ``BC`` at a contact
    point ``p`` on ``BC`` loads the PIP servo, which turns the coupler
    ``A'-B`` by ``theta``, with ``tau = F * |n . dp/dtheta|`` (virtual work, MCP
    held). ``r = |n . dp/dtheta|`` is linear along ``BC``, so its worst is at
    ``B`` or ``C``. It factors as ``r = k * d``: the linkage's torque ratio
    ``k = d(phi)/d(theta)`` times the contact's distance ``d`` along ``BC`` from
    the instantaneous centre. Returns ``(max r in mm, max k, |theta(end) - theta(start)|)``.
    """

    theta = sweep.servo_rad[workspace]
    if len(theta) < 3:
        return math.inf, math.inf, 0.0
    b, c = sweep.b[workspace], sweep.c[workspace]
    bc = (c - b) / np.linalg.norm(c - b, axis=1)[:, None]
    normal = np.stack((bc[:, 1], -bc[:, 0]), axis=1)  # palmar side of BC
    levers = [np.abs(np.sum(normal * np.gradient(point, theta, axis=0), axis=1)) for point in (b, c)]
    phi = sweep.total_flexion_rad[workspace]
    lever = np.max(np.maximum(*levers))
    ratio = np.max(np.abs(np.gradient(phi, theta)))
    return float(lever), float(ratio), float(abs(theta[-1] - theta[0]))


def _least_torque_ratio(sweep: PipSweep, workspace: np.ndarray) -> float:
    """Least ``k = d(phi)/d(theta_servo)`` over the workspace, as :func:`servo_lever` takes it."""

    theta = sweep.servo_rad[workspace]
    if len(theta) < 3:
        return math.nan
    return float(np.min(np.abs(np.gradient(sweep.total_flexion_rad[workspace], theta))))


def _pad_lengths(parameters: FingerLinkageParameters, packaging) -> dict[str, float]:
    """Each pad's mounting length and its face (less the chamfer)."""

    spans = pad_spans(parameters, packaging)
    values = {}
    for side in ("proximal", "distal"):
        length = spans[f"{side}_pad"].length_mm
        chamfer = pad_chamfer_mm(getattr(packaging, f"{side}_pad"), parameters, side)
        values[f"{side}_support_length_mm"] = length
        values[f"{side}_face_mm"] = max(length - min(chamfer, length), 0.0)
    return values


__all__ = [
    "CONSTRAINT_NAMES",
    "INVALID",
    "MOTION_CONSTRAINTS",
    "STATIC_CONSTRAINTS",
    "STOP_REASONS",
    "UNSWEPT",
    "LinkageEvaluation",
    "evaluate",
    "finger_length_mm",
    "servo_lever",
]
