"""Evaluate one Stage-0 linkage design (see ``docs/optimization/stage0.md``).

The PIP servo sits on ``A'`` and drives the coupler ``A'-B``, dorsal of the
``OA`` contact interface. ``phi = q_pip + q_dip`` is the distal link's
rotation relative to the proximal link (180 deg: ``BC`` antiparallel to ``OA``).
The contact pads hang palmar of ``O-A''`` (proximal body) and ``B''-C``
(distal body), solids whose geometry and collision pairs
``core.morphology.linkage.packaging`` owns.

**Safe motion.** From extension the PIP servo is turned in ``servo_step_rad``
steps, both ways. The motion is safe while (a) the four-bar closes, (b) the
transmission margin at ``B'`` (between ``B'->A`` and ``B'->B``) stays at least
``minimum_transmission_margin_rad``, (c) the DIP and (d) the PIP keep turning
with the servo, (e) the ``B'`` pin envelope (its bearing housing or
flange) clears the PIP servo body, and (f) the pads clear the
parts of other bodies in their Y layers (``pad_collision``; with
``pad_mcp_check`` also the MCP servo at every MCP flexion up to
``mcp_flexion_max_rad``). Hyperextension is only range for the linkage: (f)
is not checked there. The usable range ends
``flexion_stopper_margin_rad`` (of PIP) before the first unsafe sample, leaving
room past the flexion stopper, or at ``phi = 180 deg``, whichever comes first;
``phi_safe_max`` is ``phi`` there. The same conditions must hold
``hyperextension_rad`` into hyperextension, where the other stopper goes.

**Objectives** (all minimized; no weights):

1. ``k_max``, the worst torque ratio ``k = d(phi)/d(theta_servo)`` over the
   usable range (the worst lever and transmission quality are reported);
2. ``-(|a-b| proximal + |a-b| distal)``, the pads' lengths on ``O-A''`` and
   ``B''-C`` (less the MCP servo and the pad offsets);
3. ``180 deg - phi_safe_max``.

**Constraints** (``g <= 0``; degrees and mm): the static geometry and
packaging checks (pad topology ``A'_x >= A_x`` and ``B'_x <= B_x`` with
the minimum pad lengths, and a collision-free reference pose),
hyperextension safety and ``phi_safe_max >= minimum_flexion_rad``. A pad
collision past the minimum flexion only ends the usable range (objective 3). The curl-tip position and the fingertip to MCP servo
clearance are held (reported, enforced only when enabled in the config) until
their Stage-1 definitions; the OA-BC curl gap is left to Stage-1.
"""

from __future__ import annotations

import dataclasses
import functools
import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from core.morphology import Fingertip, FingertipParameters
from core.morphology.linkage import FingerLinkage, FingerLinkageParameters, PipSweep
from core.morphology.linkage.packaging import (
    ServoFootprint,
    fixed_margin_pairs,
    point_segment_distance,
    pad_margins,
    pad_spans,
    turn_points,
    usable_pad_lengths,
)

from core.optimization.stage0.config import FingerStudyConfig


STATIC_CONSTRAINTS: tuple[str, ...] = (
    "valid_reference",
    "mcp_servo_mm",
    "pivot_pin_mm",
    "pin_spacing_mm",
    "extended_length_mm",
    "oa_interface_mm",
    "proximal_support_mm",
    "distal_support_mm",
    "pad_collision_mm",
)
"""Constraints of the reference geometry alone, checked before any sweep."""
MOTION_CONSTRAINTS: tuple[str, ...] = (
    "hyperextension_deg",
    "minimum_flexion_deg",
    "middle_link_servo_mm",
    "servo_travel_deg",
    "torque_ratio_max",
    "curl_tip_x_mm",
    "fingertip_mcp_mm",
)
"""Constraints that need the PIP sweep (the last two are held unless enabled)."""
CONSTRAINT_NAMES: tuple[str, ...] = STATIC_CONSTRAINTS + MOTION_CONSTRAINTS
INVALID = 1.0e3
"""Violation assigned to every other constraint of an invalid reference design."""
UNSWEPT = 1.0e2
"""Violation assigned to the motion constraints of a design that already fails a
static one: such designs rank behind every design that passes them."""
STOP_REASONS = ("closure", "transmission", "dip_backoff", "pip_backoff", "dip_pin",
                "pad_collision")
"""The unsafe conditions (a)-(f), in the order they are reported."""


@dataclass(frozen=True)
class FingerEvaluation:
    """Constraints, objectives and diagnostics of one design."""

    parameters: FingerLinkageParameters
    constraints: Mapping[str, float]
    worst_transmission_quality: float
    ab_mm: float
    phi_safe_rad: float = 0.0
    """Distal rotation at the end of the usable range (at most pi)."""
    usable_pip_rad: float = 0.0
    """PIP flexion at the end of the usable range (the flexion stopper)."""
    usable_servo_rad: float = 0.0
    """PIP servo rotation from extension at the end of the usable range."""
    stop_reason: str = ""
    """``"curl"`` when ``phi`` reached 180 deg first, else the first unsafe condition
    (``"closure"``, ``"transmission"``, ``"dip_backoff"``, ``"pip_backoff"``,
    ``"dip_pin"``, ``"pad_collision"``) or ``"horizon"``."""
    collision_pair: str = ""
    """With ``"pad_collision"``, the first pair that collides, ``"first/second"``."""
    curl_pip_rad: float = math.nan
    curl_dip_rad: float = math.nan
    curl_tip_x_mm: float = math.nan
    """``x`` of the reference point (``C`` or ``B``) at the end of the usable range
    (the most curl) with the MCP at 0."""
    min_margin_rad: float = math.nan
    """Smallest transmission margin from hyperextension to the end of the usable range."""
    toggle_margin_rad: float = math.nan
    """Smallest margin from 0 or pi of the angle at ``B`` (the driven link
    ``A'-B`` and ``B-B'`` in line: the four-bar's limit position, where ``k``
    falls to 0) over the same range."""
    hyperextension_phi_rad: float = math.nan
    """``phi`` at the deepest safe hyperextension (negative: the distal link
    turned back past the proximal one, the passive DIP's share included)."""
    middle_link_servo_clearance_mm: float = math.nan
    """Middle link ``A-B'`` to the PIP servo body over the usable range, beyond the
    body clearance, in the plane (negative: they overlap in the side view)."""
    servo_lowest_z_mm: float = math.nan
    """Most palmar point of the PIP servo; below 0 it protrudes past the ``OA`` line."""
    closes_to_rad: float = math.nan
    lever_max_mm: float = math.nan
    """Worst PIP servo torque per unit pad force over the usable range (N mm / N)."""
    torque_ratio_max: float = math.nan
    """Worst ``k = d(phi_distal)/d(theta_servo)`` over the usable range: servo torque
    per unit moment on the distal link. Independent of where along ``BC`` a
    contact lands."""
    torque_ratio_min: float = math.nan
    """Least ``k`` over the usable range."""
    servo_travel_rad: float = math.nan
    """PIP servo rotation over the usable range."""
    pad_clearance_mm: float = math.nan
    """Least margin of the pads beyond the body clearance, over the checked
    pairs from extension to the end of the usable range, less
    ``packaging.FIXED_MARGIN_PAIRS`` (their margin does not depend on the design)."""
    proximal_support_length_mm: float = math.nan
    """The proximal pad's ``a-b`` (``packaging.pad_spans``)."""
    distal_support_length_mm: float = math.nan
    """The distal pad's ``a-b`` (``packaging.pad_spans``)."""
    proximal_usable_length_mm: float = math.nan
    distal_usable_length_mm: float = math.nan
    """Each pad's length the links leave free over the usable range, from its
    outer end (``packaging.usable_pad_lengths``)."""

    @property
    def support_length_mm(self) -> float:
        """The two pads' ``a-b`` lengths."""

        return self.proximal_support_length_mm + self.distal_support_length_mm

    @property
    def usable_support_length_mm(self) -> float:
        """The two pads' free lengths (reported only)."""

        return self.proximal_usable_length_mm + self.distal_usable_length_mm

    @property
    def phi_safe_deg(self) -> float:
        return math.degrees(self.phi_safe_rad)

    @property
    def torque_ratio_spread(self) -> float:
        """``k_max / k_min`` over the usable range: 1 when the torque ratio is flat."""

        return self.torque_ratio_max / self.torque_ratio_min

    @property
    def curl_shortfall_deg(self) -> float:
        """``180 deg - phi_safe_max``."""

        return 180.0 - math.degrees(self.phi_safe_rad)

    @property
    def g(self) -> tuple[float, ...]:
        return tuple(self.constraints[name] for name in CONSTRAINT_NAMES)

    @property
    def max_violation(self) -> float:
        return max(0.0, *self.g)

    @property
    def feasible(self) -> bool:
        return self.max_violation <= 0.0


def evaluate(parameters: FingerLinkageParameters, config: FingerStudyConfig) -> FingerEvaluation:
    """Evaluate one design.

    Static constraints come first; a design that fails one is not swept and
    gets :data:`UNSWEPT` for the motion constraints. An invalid reference
    design gets :data:`INVALID` for everything but ``valid_reference``.
    """

    ab_mm = math.hypot(parameters.b_x_mm - parameters.a_x_mm, parameters.b_z_mm - parameters.a_z_mm)
    try:
        linkage = FingerLinkage(parameters)
    except ValueError:
        # The reference pose is a dead point: no assembly branch to follow.
        constraints = {name: INVALID for name in CONSTRAINT_NAMES}
        constraints["valid_reference"] = 1.0
        return FingerEvaluation(parameters, MappingProxyType(constraints), 0.0, ab_mm,
                                **_pad_lengths(parameters, config.packaging))

    packaging = config.packaging
    footprint = packaging.pip_footprint(linkage)
    static = _static_constraints(linkage, footprint, config)
    if max(static.values()) > 0.0:
        constraints = {**static, **{name: UNSWEPT for name in MOTION_CONSTRAINTS}}
        return FingerEvaluation(parameters, MappingProxyType(constraints), 0.0, ab_mm,
                                servo_lowest_z_mm=footprint.lowest_z_mm,
                                **_pad_lengths(parameters, packaging))

    step = config.servo_step_rad
    # The servo sense that flexes the PIP.
    sense = 1.0 if _drive(linkage, packaging, [0.0, step]).q_pip_rad[-1] >= 0.0 else -1.0
    forward = np.asarray(config.servo_samples_rad)
    sweep = _drive(linkage, packaging, sense * forward)
    # The transmission angle is at the joint the driven link does not touch.
    transmission, limit_position = (("margin_at_b_rad", "margin_at_bp_rad") if packaging.pip_servo_on_a
                                    else ("margin_at_bp_rad", "margin_at_b_rad"))
    # Hyperextension is a PIP requirement: solve it by the PIP, to the stopper
    # exactly. Past a toggle at B the four-bar no longer closes.
    count = int(math.ceil(config.hyperextension_rad / step - 1e-9))
    requested_back = -np.minimum(np.arange(count + 1) * step, config.hyperextension_rad)
    back = linkage.sweep_pip(requested_back)
    if packaging.pip_servo_on_a:
        back = dataclasses.replace(back, servo_rad=back.q_pip_rad)
    n_back = len(back.q_pip_rad) - 1  # back samples besides 0

    # Joined, in increasing flexion: the back samples, then the forward sweep.
    def joined(name: str) -> np.ndarray:
        return np.concatenate((getattr(back, name)[n_back:0:-1], getattr(sweep, name)))

    unsafe = _unsafe_conditions(
        footprint, joined(transmission), joined("q_dip_rad"), joined("q_pip_rad"),
        sense * joined("servo_rad"), joined("bp"), config,
    )
    # (f) The pads, from extension (hyperextension is only linkage range, where
    # the pads may touch) up to where the usable range is already decided: the
    # first other unsafe sample, or the curl plus the stopper margin.
    other_unsafe = np.flatnonzero(np.any(np.stack(list(unsafe.values())), axis=0)[n_back:])
    end = int(other_unsafe[0]) if other_unsafe.size else len(sweep.q_pip_rad) - 1
    reached = np.flatnonzero(sweep.total_flexion_rad >= math.pi)
    if reached.size:
        rising = np.maximum.accumulate(sweep.q_pip_rad)
        past = rising[reached[0]] + config.flexion_stopper_margin_rad
        end = min(end, int(np.searchsorted(rising, past)))
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

    # Flexion: the first unsafe sample, else where the sweep stops closing or ends.
    q = sweep.q_pip_rad
    servo = forward[:len(q)]
    total = sweep.total_flexion_rad
    forward_unsafe = any_unsafe[n_back:]
    collision_pair = ""
    if forward_unsafe.any():
        safe_count = int(np.flatnonzero(forward_unsafe)[0])
        limit = float(q[safe_count])
        stop_reason = next(name for name in STOP_REASONS if unsafe[name][n_back + safe_count])
        if stop_reason == "pad_collision":
            collision_pair = "/".join(next(pair for pair, margin in margins.items()
                                           if margin[safe_count] < 0.0))
    else:
        safe_count = len(q)
        limit = float(q[-1])
        stop_reason = "closure" if len(q) < len(forward) else "horizon"
    usable_end = limit - config.flexion_stopper_margin_rad
    curl = curl_servo = math.nan
    reached_curl = np.flatnonzero(total[:safe_count] >= math.pi)
    if reached_curl.size and reached_curl[0] > 0:
        i = int(reached_curl[0])
        curl_servo, curl = _refine_curl(linkage, packaging, sense, float(servo[i - 1]), float(servo[i]))
        if curl <= usable_end:
            usable_end, stop_reason, collision_pair = curl, "curl", ""
    usable_end = max(usable_end, 0.0)
    if stop_reason == "curl":
        usable_servo, phi_safe = curl_servo, math.pi
    elif safe_count >= 2:
        # The PIP rises with the servo over the safe samples (up to the backoff tolerance).
        rising = np.maximum.accumulate(q[:safe_count])
        usable_servo = float(np.interp(usable_end, rising, servo[:safe_count]))
        phi_safe = min(float(np.interp(usable_servo, servo, total)), math.pi)
    else:
        usable_servo, phi_safe = 0.0, 0.0

    usable = servo <= usable_servo + 1e-12
    lever, ratio, travel = servo_lever(sweep, usable)
    free = usable_pad_lengths(linkage, packaging, sweep.b[usable], sweep.bp[usable], sweep.c[usable])
    ratio_min = _least_torque_ratio(sweep, usable)
    usable_joined = np.concatenate((np.ones(n_back, dtype=bool), usable))
    margin = joined(transmission)[usable_joined]
    toggle = joined(limit_position)[usable_joined]
    bp_range = joined("bp")[usable_joined]
    pad_clearance = float(reported_pad[n_back:][usable].min())

    clearance = packaging.body_clearance_mm
    a = np.array((linkage.a.x_mm, linkage.a.z_mm))
    # The middle link A-B' passes the servo side.
    link = footprint.distance_to_segments(np.broadcast_to(a, bp_range.shape), bp_range)
    middle_link_clearance = float(link.min()) - clearance

    curl_dip = math.nan
    if stop_reason == "curl":
        curl_dip = linkage.solve(0.0, curl).q_dip_rad
    # The tip at the deepest usable curl, the MCP at 0.
    end_pose = linkage.solve(0.0, usable_end)
    curl_tip_x = (end_pose.c if config.reference_point == "C" else end_pose.b).x_mm
    tip_violation = config.curl_tip_min_x_mm - curl_tip_x
    fingertip_violation = 0.0
    if config.enforce_fingertip_mcp:
        fingertip_violation = -(_fingertip_mcp_gap_mm(linkage, sweep, usable, config) - clearance)

    constraints = {
        **static,
        "hyperextension_deg": hyper_violation,
        "minimum_flexion_deg": math.degrees(config.minimum_flexion_rad - phi_safe),
        "middle_link_servo_mm": (-middle_link_clearance
                                 if config.require_middle_link_servo_clearance else 0.0),
        "torque_ratio_max": (ratio - config.torque_ratio_max
                             if config.torque_ratio_max is not None else 0.0),
        "servo_travel_deg": (math.degrees(travel - config.servo_travel_max_rad)
                             if config.servo_travel_max_rad is not None else 0.0),
        # Held until their Stage-1 definitions; enforced only when enabled.
        "curl_tip_x_mm": tip_violation if config.enforce_curl_tip else 0.0,
        "fingertip_mcp_mm": fingertip_violation,
    }
    return FingerEvaluation(
        parameters=parameters,
        constraints=MappingProxyType(constraints),
        worst_transmission_quality=float(np.sin(margin).min()),
        ab_mm=ab_mm,
        phi_safe_rad=phi_safe,
        usable_pip_rad=usable_end,
        stop_reason=stop_reason,
        collision_pair=collision_pair,
        usable_servo_rad=usable_servo,
        curl_pip_rad=curl if stop_reason == "curl" else math.nan,
        curl_dip_rad=curl_dip,
        curl_tip_x_mm=curl_tip_x,
        min_margin_rad=float(margin.min()),
        toggle_margin_rad=float(toggle.min()),
        hyperextension_phi_rad=float(back.total_flexion_rad[safe_back]),
        middle_link_servo_clearance_mm=middle_link_clearance,
        servo_lowest_z_mm=footprint.lowest_z_mm,
        closes_to_rad=float(q[-1]),
        lever_max_mm=lever,
        torque_ratio_max=ratio,
        torque_ratio_min=ratio_min,
        servo_travel_rad=travel,
        pad_clearance_mm=pad_clearance,
        **_pad_lengths(parameters, packaging),
        proximal_usable_length_mm=float(free["proximal_pad"].min()),
        distal_usable_length_mm=float(free["distal_pad"].min()),
    )


def _unsafe_conditions(
    footprint: ServoFootprint,
    margin: np.ndarray,
    q_dip: np.ndarray,
    q_pip: np.ndarray,
    servo: np.ndarray,
    bp: np.ndarray,
    config: FingerStudyConfig,
) -> dict[str, np.ndarray]:
    """Per-sample unsafe flags (b)-(e) along the joined sweep, in increasing
    flexion; (a) is the sweep's end and the caller adds (f). Backoffs are
    measured from the start of the sweep (deepest hyperextension); ``servo`` is
    signed to rise with flexion."""

    tolerance = config.dip_backoff_tolerance_rad
    return {
        "closure": np.zeros(len(margin), dtype=bool),
        "transmission": margin < config.minimum_transmission_margin_rad,
        "dip_backoff": np.maximum.accumulate(q_dip) - q_dip > tolerance,
        # The PIP and the servo turn together.
        "pip_backoff": (np.maximum.accumulate(q_pip) - q_pip > tolerance)
        | (np.maximum.accumulate(servo) - servo > tolerance),
        "dip_pin": footprint.distance_to_segments(bp, bp)
        < config.packaging.pin_radius_mm + config.packaging.body_clearance_mm,
    }


def servo_lever(sweep: PipSweep, workspace: np.ndarray) -> tuple[float, float, float]:
    """Worst lever ``r``, worst torque ratio ``k`` and the servo travel over the workspace.

    A pad force ``F`` along the palmar normal ``n`` of ``BC`` at a contact
    point ``p`` on ``BC`` loads the PIP servo, which turns the coupler
    ``A'-B`` by ``theta``, with ``tau = F * |n . dp/dtheta|`` (virtual work, MCP
    held). ``r = |n . dp/dtheta|`` is linear along ``BC``, so its worst is at
    ``B`` or ``C``. It factors as ``r = k * d``: the linkage's torque ratio
    ``k = d(phi)/d(theta)``, with ``phi = q_pip + q_dip`` the distal link's
    rotation, times the contact's distance ``d`` along ``BC`` from the
    instantaneous centre. Returns ``(max r in mm, max k, |theta(end) - theta(start)|)``.
    """

    theta = sweep.servo_rad[workspace]
    if len(theta) < 3:
        return math.inf, math.inf, 0.0
    b, c = sweep.b[workspace], sweep.c[workspace]
    bc = (c - b) / np.linalg.norm(c - b, axis=1)[:, None]
    normal = np.stack((bc[:, 1], -bc[:, 0]), axis=1)  # palmar side of BC
    levers = [np.abs(np.sum(normal * np.gradient(point, theta, axis=0), axis=1))
              for point in (b, c)]
    phi = sweep.total_flexion_rad[workspace]
    lever = np.max(np.maximum(*levers))
    ratio = np.max(np.abs(np.gradient(phi, theta)))
    return float(lever), float(ratio), float(abs(theta[-1] - theta[0]))


def _static_constraints(
    linkage: FingerLinkage, footprint: ServoFootprint, config: FingerStudyConfig
) -> dict[str, float]:
    """Constraints of the reference geometry and the packaging alone."""

    packaging = config.packaging
    clearance = packaging.body_clearance_mm
    parameters = linkage.parameters
    # The proximal pin without the servo hangs off its body on a bracket.
    other = linkage.ap if packaging.pip_servo_on_a else linkage.a
    other_pin = np.array((other.x_mm, other.z_mm))
    spacing = min(linkage.a.distance_to(linkage.bp), linkage.bp.distance_to(linkage.b))
    reach = parameters.bc_length_mm if config.reference_point == "C" else 0.0
    length = math.hypot(parameters.b_x_mm + reach, parameters.b_z_mm)
    length_low, length_high = config.extended_length_bounds_mm
    # From MCP hyperextension to extension; flexion only turns the PIP servo
    # away from the MCP servo's dorsal rear body.
    mcp_angles = np.linspace(-config.hyperextension_rad, 0.0, 3)
    # Footprints whose circumscribed circles are apart cannot touch.
    apart = math.hypot(*footprint.pivot) - footprint.reach_mm - packaging.mcp_footprint().reach_mm
    servo_gap = apart if apart > clearance else packaging.mcp_servo_gap_mm(footprint, mcp_angles)
    spans = pad_spans(parameters, packaging)
    proximal, distal = spans["proximal_pad"], spans["distal_pad"]
    reference = [np.array([[point.x_mm, point.z_mm]]) for point in (linkage.b, linkage.c, linkage.bp)]
    margins = pad_margins(linkage, packaging, *reference, config.mcp_samples_rad)
    return {
        "valid_reference": 0.0,
        "mcp_servo_mm": -(servo_gap - clearance),
        # The pivot off the servo (A, or A' with the servo on A) clears its body.
        "pivot_pin_mm": -(footprint.distance_to_point(other_pin) - packaging.pin_radius_mm - clearance),
        "pin_spacing_mm": packaging.pin_spacing_mm - spacing,
        "extended_length_mm": max(length_low - length, length - length_high),
        # The PIP servo stays dorsal of the OA contact interface.
        "oa_interface_mm": (0.0 if packaging.pip_servo_on_a
                            else config.oa_interface_clearance_mm - footprint.lowest_z_mm),
        # The pads' mounting lines run from the pivots outward, with room for
        # the pads: a positive length, at least the minimum, at least a fixed one.
        "proximal_support_mm": max(-math.inf if packaging.pip_servo_on_a
                                   else parameters.a_x_mm - parameters.ap.x_mm,
                                   proximal.length_mm - proximal.limit_mm,
                                   config.minimum_proximal_support_mm - proximal.length_mm,
                                   proximal.length_mm - proximal.room_mm),
        "distal_support_mm": max(parameters.bp.x_mm - parameters.b_x_mm,
                                 distal.length_mm - distal.limit_mm,
                                 config.minimum_distal_support_mm - distal.length_mm,
                                 distal.length_mm - distal.room_mm),
        "pad_collision_mm": -min(float(margin.min()) for margin in margins.values()),
    }


def _least_torque_ratio(sweep: PipSweep, workspace: np.ndarray) -> float:
    """Least ``k = d(phi)/d(theta_servo)`` over the workspace, as :func:`servo_lever` takes it."""

    theta = sweep.servo_rad[workspace]
    if len(theta) < 3:
        return math.nan
    return float(np.min(np.abs(np.gradient(sweep.total_flexion_rad[workspace], theta))))


def _pad_lengths(parameters: FingerLinkageParameters, packaging) -> dict[str, float]:
    spans = pad_spans(parameters, packaging)
    return {"proximal_support_length_mm": spans["proximal_pad"].length_mm,
            "distal_support_length_mm": spans["distal_pad"].length_mm}


def _fingertip_mcp_gap_mm(
    linkage: FingerLinkage, sweep: PipSweep, safe: np.ndarray, config: FingerStudyConfig
) -> float:
    """Least gap between the fingertip silhouette and the MCP servo body.

    Checked on a coarse grid: PIP every 5 deg over the flexion side of the
    range and MCP flexion every 15 deg from hyperextension to its maximum. The
    fingertip (default family, as long as ``BC``) rides on ``BC`` like in
    :func:`core.morphology.hand_layout.fingertip_attachment`. Only outline
    segments within the MCP servo's reach of ``O`` are checked at every MCP
    angle; the rest bound the gap from below.
    """

    outline = _fingertip_outline(linkage.parameters.bc_length_mm)

    stride = max(1, int(round(math.radians(5.0) / config.servo_step_rad)))
    b, c = sweep.b[safe][::stride], sweep.c[safe][::stride]
    angle = np.arctan2(c[:, 1] - b[:, 1], c[:, 0] - b[:, 0])
    cosine, sine = np.cos(angle)[:, None], np.sin(angle)[:, None]
    points = np.stack((b[:, :1] + cosine * outline[:, 0] - sine * outline[:, 1],
                       b[:, 1:] + sine * outline[:, 0] + cosine * outline[:, 1]), axis=-1)
    start = points.reshape(-1, 2)
    end = np.roll(points, -1, axis=1).reshape(-1, 2)

    packaging = config.packaging
    reach = packaging.mcp_footprint().reach_mm
    from_o = point_segment_distance(np.zeros(2), start, end)
    near = from_o <= reach + packaging.body_clearance_mm
    if not near.any():
        return float(from_o.min()) - reach
    angles = np.arange(-config.hyperextension_rad, config.mcp_flexion_max_rad + 1e-9, math.radians(15.0))
    # Turning the MCP servo by +q is turning the fingertip by -q about O.
    starts = np.concatenate([turn_points(start[near], -q) for q in angles])
    ends = np.concatenate([turn_points(end[near], -q) for q in angles])
    return float(packaging.mcp_footprint().distance_to_segments(starts, ends).min())


@functools.lru_cache(maxsize=8)
def _fingertip_outline(bc_length_mm: float) -> np.ndarray:
    """Fingertip side silhouette in the ``BC`` frame (``B`` at 0, ``BC`` along ``+X``)."""

    fingertip = Fingertip(FingertipParameters(total_length_mm=bc_length_mm))
    outline = np.array(fingertip.side_silhouette_mm(12))
    outline[:, 1] -= fingertip.rotation_axis_z_mm
    return outline


def _drive(linkage: FingerLinkage, packaging, servo_rad) -> PipSweep:
    """The four-bar at PIP servo angles: turning ``A'-B`` about ``A'``, or with the
    servo on ``A`` the middle link ``A-B'`` about ``A`` (its angle is the PIP's)."""

    if packaging.pip_servo_on_a:
        sweep = linkage.sweep_pip(servo_rad)
        return dataclasses.replace(sweep, servo_rad=sweep.q_pip_rad)
    return linkage.sweep_servo(servo_rad)


def _refine_curl(linkage: FingerLinkage, packaging, sense: float, low: float,
                 high: float) -> tuple[float, float]:
    """The servo angle and PIP where ``q_pip + q_dip`` reaches pi between two servo samples.

    One fine vectorized sweep of the bracket, then linear interpolation.
    """

    servo = np.linspace(low, high, 65)
    sweep = _drive(linkage, packaging, sense * servo)
    total = sweep.total_flexion_rad
    i = int(np.flatnonzero(total >= math.pi)[0]) if (total >= math.pi).any() else len(total) - 1
    if i == 0:
        return float(servo[0]), float(sweep.q_pip_rad[0])
    fraction = (math.pi - total[i - 1]) / (total[i] - total[i - 1])
    return (float(servo[i - 1] + fraction * (servo[i] - servo[i - 1])),
            float(sweep.q_pip_rad[i - 1] + fraction * (sweep.q_pip_rad[i] - sweep.q_pip_rad[i - 1])))


__all__ = [
    "CONSTRAINT_NAMES",
    "FingerEvaluation",
    "INVALID",
    "MOTION_CONSTRAINTS",
    "STATIC_CONSTRAINTS",
    "STOP_REASONS",
    "UNSWEPT",
    "evaluate",
    "servo_lever",
]
