"""Evaluate one 2-DoF finger design against the study requirements.

The PIP is swept in ``pip_step_rad`` steps. The curl pose is the PIP angle
where the distal link has turned 180 deg relative to the proximal link
(``q_pip + q_dip = pi``, ``BC`` antiparallel to ``OA``); there the ``BC``
line must sit ``curl_gap_mm`` palmar of the ``OA`` line (the two contact
interfaces; the fingertip is only drawn). The evaluated range is extension
to curl widened by ``range_margin_rad`` at both ends; over it the linkage
must close, keep the transmission margin, and turn the DIP and the driven
link one way. Objectives, in priority order of the study (the curl
requirement is a constraint):

1. maximize the worst transmission quality ``sin(margin)`` over the range;
2. minimize ``AB`` in the extended pose.

The PIP servo sits on ``A'`` and drives the coupler ``A'-B``, dorsal of the
``OA`` contact interface. The transmission margin is the four-bar's output
angle at ``B'`` (between ``B'->A`` and ``B'->B``), and the driven coupler
must turn one way over the range (no limit position).

Constraints are ``g <= 0``. Angles are in degrees and lengths in mm.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from cmd.morphology.linkage import FingerLinkage, FingerLinkageParameters
from cmd.morphology.linkage.packaging import BODY_CLEARANCE_MM, PIN_CLEARANCE_MM, ServoFootprint

from .config import FingerStudyConfig


CONSTRAINT_NAMES: tuple[str, ...] = (
    "valid_reference",
    "curl_reached_deg",
    "range_closure_deg",
    "transmission_deg",
    "dip_backoff_deg",
    "input_backoff_deg",
    "mcp_servo_mm",
    "pivot_pin_mm",
    "link_mm",
    "pin_spacing_mm",
    "extended_length_mm",
    "curl_gap_mm",
    "oa_interface_mm",
)
INVALID = 1.0e3
"""Violation assigned to every other constraint of an invalid reference design."""


@dataclass(frozen=True)
class FingerEvaluation:
    """Constraints, objectives and diagnostics of one design."""

    parameters: FingerLinkageParameters
    constraints: Mapping[str, float]
    worst_transmission_quality: float
    ab_mm: float
    curl_pip_rad: float = math.nan
    curl_dip_rad: float = math.nan
    curl_gap_mm: float = math.nan
    min_margin_rad: float = math.nan
    """Smallest transmission margin over the evaluated range."""
    link_clearance_mm: float = math.nan
    """Undriven straight link to the PIP servo over the safe range (negative: interference)."""
    servo_lowest_z_mm: float = math.nan
    """Most palmar point of the PIP servo; below 0 it protrudes past the ``OA`` line."""
    closes_to_rad: float = math.nan

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
    """Evaluate one design; invalid reference designs get :data:`INVALID` violations."""

    ab_mm = parameters.b_x_mm - parameters.a_x_mm
    try:
        linkage = FingerLinkage(parameters)
    except ValueError:
        violation = max(_crossing_violation(parameters), 1.0e-3)
        constraints = {name: INVALID for name in CONSTRAINT_NAMES}
        constraints["valid_reference"] = violation
        return FingerEvaluation(parameters, MappingProxyType(constraints), 0.0, ab_mm)

    sweep = linkage.sweep_pip(config.pip_samples_rad)
    q = sweep.q_pip_rad
    total = sweep.total_flexion_rad
    closes_to = float(q[-1])

    reached = np.flatnonzero(total >= math.pi)
    if reached.size:
        i = int(reached[0])
        curl = _refine_curl(linkage, float(q[i - 1]), float(q[i]))
        curl_shortfall = 0.0
        safe_end = curl + config.range_margin_rad
    else:
        curl = math.nan
        curl_shortfall = math.degrees(math.pi - float(total.max()))
        safe_end = closes_to
    safe = q <= safe_end + 1e-12

    # Sweep from extension into hyperextension and join it, in increasing PIP
    # order, to the flexion side up to the safe end.
    steps = int(round(config.range_margin_rad / config.pip_step_rad))
    back = linkage.sweep_pip([-index * config.pip_step_rad for index in range(steps + 1)])
    extension_shortfall = config.range_margin_rad + float(back.q_pip_rad[-1])

    def joined(field: str) -> np.ndarray:
        return np.concatenate((getattr(back, field)[:0:-1], getattr(sweep, field)[safe]))

    margin = joined("margin_at_bp_rad")
    q_dip = joined("q_dip_rad")
    backoff = float(np.max(np.maximum.accumulate(q_dip) - q_dip))
    b_range, bp_range = joined("b"), joined("bp")
    # The driven coupler A'-B must turn one way.
    to_b = b_range - np.array((parameters.ap.x_mm, parameters.ap.z_mm))
    turn = np.unwrap(np.arctan2(to_b[:, 1], to_b[:, 0]))
    turn = turn * (1.0 if turn[-1] >= turn[0] else -1.0)
    input_backoff = float(np.max(np.maximum.accumulate(turn) - turn))

    footprint = ServoFootprint.of(linkage)
    a = np.array((linkage.a.x_mm, 0.0))
    # The middle link A-B' passes the servo side.
    link = footprint.distance_to_segments(np.broadcast_to(a, bp_range.shape), bp_range)
    link_clearance = float(link.min()) - BODY_CLEARANCE_MM
    spacing = min(linkage.a.distance_to(linkage.bp), linkage.bp.distance_to(linkage.b))
    length = parameters.b_x_mm + parameters.bc_length_mm
    length_low, length_high = config.extended_length_bounds_mm

    curl_dip = curl_gap = math.nan
    gap_violation = 0.0
    if reached.size:
        pose = linkage.solve(0.0, curl)
        curl_dip = pose.q_dip_rad
        # BC is antiparallel to OA here: the gap is the distance between the two lines.
        curl_gap = -0.5 * (pose.b.z_mm + pose.c.z_mm)
        gap_violation = abs(curl_gap - config.curl_gap_mm) - config.curl_gap_tolerance_mm

    constraints = {
        "valid_reference": 0.0,
        "curl_reached_deg": curl_shortfall,
        "range_closure_deg": math.degrees(max(
            safe_end - closes_to if reached.size else 0.0, extension_shortfall
        )),
        "transmission_deg": math.degrees(config.minimum_transmission_margin_rad - float(margin.min())),
        "dip_backoff_deg": math.degrees(backoff - config.dip_backoff_tolerance_rad),
        "input_backoff_deg": math.degrees(input_backoff - config.dip_backoff_tolerance_rad),
        "mcp_servo_mm": -(footprint.distance_to_point(np.zeros(2))
                          - footprint.radius_mm - BODY_CLEARANCE_MM),
        # The PIP pin A clears the servo's shaft disk.
        "pivot_pin_mm": -(linkage.a.distance_to(linkage.ap) - footprint.radius_mm - PIN_CLEARANCE_MM),
        "link_mm": -link_clearance if config.link_clearance else 0.0,
        "pin_spacing_mm": config.minimum_pin_spacing_mm - spacing,
        "extended_length_mm": max(length_low - length, length - length_high),
        "curl_gap_mm": gap_violation,
        # The PIP servo stays dorsal of the OA contact interface.
        "oa_interface_mm": config.oa_interface_clearance_mm - footprint.lowest_z_mm,
    }
    return FingerEvaluation(
        parameters=parameters,
        constraints=MappingProxyType(constraints),
        worst_transmission_quality=float(np.sin(margin).min()),
        ab_mm=ab_mm,
        curl_pip_rad=curl,
        curl_dip_rad=curl_dip,
        curl_gap_mm=curl_gap,
        min_margin_rad=float(margin.min()),
        link_clearance_mm=link_clearance,
        servo_lowest_z_mm=footprint.lowest_z_mm,
        closes_to_rad=closes_to,
    )


def _refine_curl(linkage: FingerLinkage, low: float, high: float) -> float:
    """Bisect the PIP angle where ``q_pip + q_dip`` reaches pi between two samples."""

    for _ in range(15):
        middle = 0.5 * (low + high)
        pose = linkage.solve(0.0, middle)
        if pose.q_pip_rad + pose.q_dip_rad >= math.pi:
            high = middle
        else:
            low = middle
    return 0.5 * (low + high)


def _crossing_violation(parameters: FingerLinkageParameters) -> float:
    """How far the reference design is from a crossed four-bar with ``B`` beyond ``A``."""

    a = np.array((parameters.a_x_mm, 0.0))
    b = np.array((parameters.b_x_mm, 0.0))
    ap = np.array((parameters.ap.x_mm, parameters.ap.z_mm))
    bp = np.array((parameters.bp.x_mm, parameters.bp.z_mm))

    def cross(o, p, r):
        return (p[0] - o[0]) * (r[1] - o[1]) - (p[1] - o[1]) * (r[0] - o[0])

    # A-B' and A'-B cross when each segment's ends lie on opposite sides of the other.
    side_1 = cross(a, bp, ap) * cross(a, bp, b)
    side_2 = cross(ap, b, a) * cross(ap, b, bp)
    return (max(side_1, 0.0) + max(side_2, 0.0)) / 1.0e4 + max(0.0, a[0] - b[0])


__all__ = ["CONSTRAINT_NAMES", "FingerEvaluation", "INVALID", "evaluate"]
