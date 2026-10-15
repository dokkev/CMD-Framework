"""Evaluate one Stage-0 linkage design (see ``docs/optimization/stage0.md``).

The PIP servo sits on ``A'`` and drives the coupler ``A'-B``, dorsal of the
``OA`` contact interface. ``phi = q_pip + q_dip`` is the distal link's
rotation relative to the proximal link (180 deg: ``BC`` antiparallel to ``OA``).

**Safe motion.** From extension the PIP is swept in ``pip_step_rad`` steps. The
motion is safe while (a) the four-bar closes, (b) the transmission margin at
``B'`` (between ``B'->A`` and ``B'->B``) stays at least
``minimum_transmission_margin_rad``, (c) the DIP and (d) the driven coupler
``A'-B`` keep turning one way, and (e) the ``B'`` bushing housing clears the
PIP servo body. The usable range ends ``range_margin_rad`` (of PIP) before the
first unsafe sample, leaving room for a stopper, or at ``phi = 180 deg``,
whichever comes first; ``phi_safe_max`` is ``phi`` there. The same conditions
must hold ``range_margin_rad`` into hyperextension.

**Objectives** (all minimized; no weights):

1. ``k_max``, the worst torque ratio ``k = d(phi)/d(theta_servo)`` over the
   usable range (or, per ``config.objective``, the worst lever or the negated
   worst transmission quality);
2. ``-(OA + BC)``, the structural length of the two contact segments;
3. ``180 deg - phi_safe_max``.

**Constraints** (``g <= 0``; degrees and mm): the static geometry and
packaging checks, hyperextension safety and ``phi_safe_max >=
minimum_flexion_rad``. The curl-tip position and the fingertip to MCP servo
clearance are held (reported, enforced only when enabled in the config) until
their Stage-1 definitions; the OA-BC curl gap is left to Stage-1.
"""

from __future__ import annotations

import functools
import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from cmd.morphology import Fingertip, FingertipParameters
from cmd.morphology.linkage import FingerLinkage, FingerLinkageParameters, PipSweep
from cmd.morphology.linkage.packaging import ServoFootprint, point_segment_distance, turn_points

from .config import FingerStudyConfig


STATIC_CONSTRAINTS: tuple[str, ...] = (
    "valid_reference",
    "mcp_servo_mm",
    "pivot_pin_mm",
    "pin_spacing_mm",
    "extended_length_mm",
    "oa_interface_mm",
    "bc_to_oa",
)
"""Constraints of the reference geometry alone, checked before any sweep."""
MOTION_CONSTRAINTS: tuple[str, ...] = (
    "hyperextension_deg",
    "minimum_flexion_deg",
    "link_mm",
    "servo_travel_deg",
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
STOP_REASONS = ("closure", "transmission", "dip_backoff", "input_backoff", "dip_pin")
"""The unsafe conditions (a)-(e), in the order they are reported."""


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
    """PIP flexion at the end of the usable range."""
    stop_reason: str = ""
    """``"curl"`` when ``phi`` reached 180 deg first, else the first unsafe condition
    (``"closure"``, ``"transmission"``, ``"dip_backoff"``, ``"input_backoff"``,
    ``"dip_pin"``) or ``"horizon"``."""
    curl_pip_rad: float = math.nan
    curl_dip_rad: float = math.nan
    curl_tip_x_mm: float = math.nan
    """``x`` of the reference point (``C`` or ``B``) at the curl pose with the MCP at 0."""
    min_margin_rad: float = math.nan
    """Smallest transmission margin from hyperextension to the end of the usable range."""
    link_clearance_mm: float = math.nan
    """Undriven straight link to the PIP servo over the usable range (negative: interference)."""
    servo_lowest_z_mm: float = math.nan
    """Most palmar point of the PIP servo; below 0 it protrudes past the ``OA`` line."""
    closes_to_rad: float = math.nan
    lever_max_mm: float = math.nan
    """Worst PIP servo torque per unit pad force over the usable range (N mm / N)."""
    torque_ratio_max: float = math.nan
    """Worst ``k = d(phi_distal)/d(theta_servo)`` over the usable range: servo torque
    per unit moment on the distal link. Independent of where along ``BC`` a
    contact lands."""
    servo_travel_rad: float = math.nan
    """PIP servo rotation over the usable range."""

    @property
    def structural_length_mm(self) -> float:
        """``OA + BC``, the two contact-supporting segments."""

        return self.parameters.a_x_mm + self.parameters.bc_length_mm

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

    ab_mm = math.hypot(parameters.b_x_mm - parameters.a_x_mm, parameters.b_z_mm)
    try:
        linkage = FingerLinkage(parameters)
    except ValueError:
        # The reference pose is a dead point: no assembly branch to follow.
        constraints = {name: INVALID for name in CONSTRAINT_NAMES}
        constraints["valid_reference"] = 1.0
        return FingerEvaluation(parameters, MappingProxyType(constraints), 0.0, ab_mm)

    packaging = config.packaging
    footprint = packaging.pip_footprint(linkage)
    static = _static_constraints(linkage, footprint, config)
    if max(static.values()) > 0.0:
        constraints = {**static, **{name: UNSWEPT for name in MOTION_CONSTRAINTS}}
        return FingerEvaluation(parameters, MappingProxyType(constraints), 0.0, ab_mm,
                                servo_lowest_z_mm=footprint.lowest_z_mm)

    step = config.pip_step_rad
    requested = config.pip_samples_rad
    sweep = linkage.sweep_pip(requested)
    margin_steps = int(round(config.range_margin_rad / step))
    back = linkage.sweep_pip([-index * step for index in range(margin_steps + 1)])
    n_back = len(back.q_pip_rad) - 1  # hyperextension samples that close, besides 0

    # Joined, in increasing PIP: closed hyperextension samples, then the flexion sweep.
    def joined(name: str) -> np.ndarray:
        return np.concatenate((getattr(back, name)[:0:-1], getattr(sweep, name)))

    unsafe = _unsafe_conditions(
        parameters, footprint, joined("margin_at_bp_rad"), joined("q_dip_rad"),
        joined("b"), joined("bp"), n_back, config,
    )
    any_unsafe = np.any(np.stack(list(unsafe.values())), axis=0)

    # Hyperextension: every requested sample closes and is safe.
    hyper_violation = math.degrees(step) * (
        (margin_steps - n_back) + int(np.count_nonzero(any_unsafe[:n_back]))
    )

    # Flexion: the first unsafe sample, or the first one that does not close.
    q = sweep.q_pip_rad
    total = sweep.total_flexion_rad
    forward_unsafe = any_unsafe[n_back:]
    candidates = []
    if forward_unsafe.any():
        first = int(np.flatnonzero(forward_unsafe)[0])
        reason = next(name for name in STOP_REASONS if unsafe[name][n_back + first])
        candidates.append((float(q[first]) - config.range_margin_rad, reason))
    if len(q) < len(requested):
        candidates.append((float(requested[len(q)]) - config.range_margin_rad, "closure"))
    reached = np.flatnonzero(total >= math.pi)
    curl = math.nan
    if reached.size and reached[0] > 0:
        curl = _refine_curl(linkage, float(q[reached[0] - 1]), float(q[reached[0]]))
        candidates.append((curl, "curl"))
    candidates.append((float(q[-1]), "horizon"))
    usable_end, stop_reason = min(candidates, key=lambda item: item[0])
    usable_end = max(usable_end, 0.0)
    # The curl pose is refined to phi = 180 deg; the grid interpolation would fall just short.
    phi_safe = math.pi if stop_reason == "curl" else min(float(np.interp(usable_end, q, total)), math.pi)

    usable = q <= usable_end + 1e-12
    lever, ratio, travel = servo_lever(parameters, sweep, usable)
    usable_joined = np.concatenate((np.ones(n_back, dtype=bool), usable))
    margin = joined("margin_at_bp_rad")[usable_joined]
    bp_range = joined("bp")[usable_joined]

    clearance = packaging.body_clearance_mm
    a = np.array((linkage.a.x_mm, 0.0))
    # The middle link A-B' passes the servo side.
    link = footprint.distance_to_segments(np.broadcast_to(a, bp_range.shape), bp_range)
    link_clearance = float(link.min()) - clearance

    curl_dip = curl_tip_x = math.nan
    tip_violation = 0.0
    if stop_reason == "curl":
        pose = linkage.solve(0.0, curl)
        curl_dip = pose.q_dip_rad
        curl_tip_x = (pose.c if config.reference_point == "C" else pose.b).x_mm
        tip_violation = config.curl_tip_min_x_mm - curl_tip_x
    fingertip_violation = 0.0
    if config.enforce_fingertip_mcp:
        fingertip_violation = -(_fingertip_mcp_gap_mm(linkage, sweep, usable, config) - clearance)

    constraints = {
        **static,
        "hyperextension_deg": hyper_violation,
        "minimum_flexion_deg": math.degrees(config.minimum_flexion_rad - phi_safe),
        "link_mm": -link_clearance if config.link_clearance else 0.0,
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
        curl_pip_rad=curl if stop_reason == "curl" else math.nan,
        curl_dip_rad=curl_dip,
        curl_tip_x_mm=curl_tip_x,
        min_margin_rad=float(margin.min()),
        link_clearance_mm=link_clearance,
        servo_lowest_z_mm=footprint.lowest_z_mm,
        closes_to_rad=float(q[-1]),
        lever_max_mm=lever,
        torque_ratio_max=ratio,
        servo_travel_rad=travel,
    )


def _unsafe_conditions(
    parameters: FingerLinkageParameters,
    footprint: ServoFootprint,
    margin: np.ndarray,
    q_dip: np.ndarray,
    b: np.ndarray,
    bp: np.ndarray,
    extension_index: int,
    config: FingerStudyConfig,
) -> dict[str, np.ndarray]:
    """Per-sample unsafe flags (b)-(e) along the joined sweep; (a) is the sweep's end.

    Backoffs are measured from the start of the sweep (deepest hyperextension).
    The driven coupler's direction is the one it turns in leaving extension.
    """

    tolerance = config.dip_backoff_tolerance_rad
    to_b = b - np.array((parameters.ap.x_mm, parameters.ap.z_mm))
    turn = np.unwrap(np.arctan2(to_b[:, 1], to_b[:, 0]))
    i = min(extension_index, len(turn) - 2)
    turn = turn * (1.0 if turn[i + 1] >= turn[i] else -1.0)
    housing = footprint.distance_to_segments(bp, bp)
    packaging = config.packaging
    return {
        "closure": np.zeros(len(margin), dtype=bool),
        "transmission": margin < config.minimum_transmission_margin_rad,
        "dip_backoff": np.maximum.accumulate(q_dip) - q_dip > tolerance,
        "input_backoff": np.maximum.accumulate(turn) - turn > tolerance,
        "dip_pin": housing < packaging.housing_radius_mm + packaging.body_clearance_mm,
    }


def servo_lever(
    parameters: FingerLinkageParameters, sweep: PipSweep, workspace: np.ndarray
) -> tuple[float, float, float]:
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

    q = sweep.q_pip_rad[workspace]
    if len(q) < 3:
        return math.inf, math.inf, 0.0
    b, c = sweep.b[workspace], sweep.c[workspace]
    to_b = b - np.array((parameters.ap.x_mm, parameters.ap.z_mm))
    theta = np.unwrap(np.arctan2(to_b[:, 1], to_b[:, 0]))
    dtheta = np.gradient(theta, q)
    bc = (c - b) / np.linalg.norm(c - b, axis=1)[:, None]
    normal = np.stack((bc[:, 1], -bc[:, 0]), axis=1)  # palmar side of BC
    levers = [np.abs(np.sum(normal * np.gradient(point, q, axis=0), axis=1))
              for point in (b, c)]
    phi = sweep.total_flexion_rad[workspace]
    with np.errstate(divide="ignore"):
        lever = np.max(np.maximum(*levers) / np.abs(dtheta))
        ratio = np.max(np.abs(np.gradient(phi, q) / dtheta))
    return float(lever), float(ratio), float(abs(theta[-1] - theta[0]))


def _static_constraints(
    linkage: FingerLinkage, footprint: ServoFootprint, config: FingerStudyConfig
) -> dict[str, float]:
    """Constraints of the reference geometry and the packaging alone."""

    packaging = config.packaging
    clearance = packaging.body_clearance_mm
    parameters = linkage.parameters
    a = np.array((linkage.a.x_mm, 0.0))
    spacing = min(linkage.a.distance_to(linkage.bp), linkage.bp.distance_to(linkage.b))
    reach = parameters.bc_length_mm if config.reference_point == "C" else 0.0
    length = math.hypot(parameters.b_x_mm + reach, parameters.b_z_mm)
    length_low, length_high = config.extended_length_bounds_mm
    # From MCP hyperextension to extension; flexion only turns the PIP servo
    # away from the MCP servo's dorsal rear body.
    mcp_angles = np.linspace(-config.range_margin_rad, 0.0, 3)
    # Footprints whose circumscribed circles are apart cannot touch.
    apart = math.hypot(*footprint.pivot) - footprint.reach_mm - packaging.mcp_footprint().reach_mm
    servo_gap = apart if apart > clearance else packaging.mcp_servo_gap_mm(footprint, mcp_angles)
    return {
        "valid_reference": 0.0,
        "mcp_servo_mm": -(servo_gap - clearance),
        # The A pivot hangs off the servo body on a bracket: its housing clears the body.
        "pivot_pin_mm": -(footprint.distance_to_point(a) - packaging.housing_radius_mm - clearance),
        "pin_spacing_mm": packaging.pin_spacing_mm - spacing,
        "extended_length_mm": max(length_low - length, length - length_high),
        # The PIP servo stays dorsal of the OA contact interface.
        "oa_interface_mm": config.oa_interface_clearance_mm - footprint.lowest_z_mm,
        # BC no longer than OA (times the ratio): the curled tip stays near O.
        "bc_to_oa": (parameters.bc_length_mm - config.bc_to_oa_ratio_max * parameters.a_x_mm
                     if config.bc_to_oa_ratio_max is not None else 0.0),
    }


def _fingertip_mcp_gap_mm(
    linkage: FingerLinkage, sweep: PipSweep, safe: np.ndarray, config: FingerStudyConfig
) -> float:
    """Least gap between the fingertip silhouette and the MCP servo body.

    Checked on a coarse grid: PIP every 5 deg over the flexion side of the
    range and MCP flexion every 15 deg from hyperextension to its maximum. The
    fingertip (default family, as long as ``BC``) rides on ``BC`` like in
    :func:`cmd.morphology.hand_layout.fingertip_attachment`. Only outline
    segments within the MCP servo's reach of ``O`` are checked at every MCP
    angle; the rest bound the gap from below.
    """

    outline = _fingertip_outline(linkage.parameters.bc_length_mm)

    stride = max(1, int(round(math.radians(5.0) / config.pip_step_rad)))
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
    angles = np.arange(-config.range_margin_rad, config.mcp_flexion_max_rad + 1e-9, math.radians(15.0))
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


def _refine_curl(linkage: FingerLinkage, low: float, high: float) -> float:
    """The PIP angle where ``q_pip + q_dip`` reaches pi between two samples.

    One fine vectorized sweep of the bracket, then linear interpolation.
    """

    sweep = linkage.sweep_pip(np.linspace(low, high, 65))
    total = sweep.total_flexion_rad
    i = int(np.flatnonzero(total >= math.pi)[0]) if (total >= math.pi).any() else len(total) - 1
    if i == 0:
        return float(sweep.q_pip_rad[0])
    fraction = (math.pi - total[i - 1]) / (total[i] - total[i - 1])
    return float(sweep.q_pip_rad[i - 1] + fraction * (sweep.q_pip_rad[i] - sweep.q_pip_rad[i - 1]))


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
