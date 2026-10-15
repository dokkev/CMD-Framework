"""Evaluate one 2-DoF finger design against the study requirements.

The PIP is swept in ``pip_step_rad`` steps. The curl pose is the PIP angle
where the distal link has turned 180 deg relative to the proximal link
(``q_pip + q_dip = pi``, ``BC`` antiparallel to ``OA``); there the ``BC``
line must sit at least ``curl_gap_mm`` palmar of the ``OA`` line (the two
contact interfaces). The fingertip's side silhouette, carried by ``BC``, must
clear the MCP servo over the range and MCP flexion. The evaluated range is extension
to curl widened by ``range_margin_rad`` at both ends; over it the linkage
must close, keep the transmission margin, and turn the DIP and the driven
link one way. Objectives, in priority order of the study (the curl
requirement is a constraint):

1. maximize the worst transmission quality ``sin(margin)`` over the range;
2. minimize the distance ``AB`` in the extended pose.

The PIP servo sits on ``A'`` and drives the coupler ``A'-B``, dorsal of the
``OA`` contact interface. The transmission margin is the four-bar's output
angle at ``B'`` (between ``B'->A`` and ``B'->B``), and the driven coupler
must turn one way over the range (no limit position).

Constraints are ``g <= 0``. Angles are in degrees and lengths in mm.
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
)
"""Constraints of the reference geometry alone, checked before any sweep."""
MOTION_CONSTRAINTS: tuple[str, ...] = (
    "curl_reached_deg",
    "range_closure_deg",
    "transmission_deg",
    "dip_backoff_deg",
    "input_backoff_deg",
    "dip_pin_mm",
    "link_mm",
    "curl_gap_mm",
    "fingertip_mcp_mm",
)
"""Constraints that need the PIP sweep."""
CONSTRAINT_NAMES: tuple[str, ...] = STATIC_CONSTRAINTS + MOTION_CONSTRAINTS
INVALID = 1.0e3
"""Violation assigned to every other constraint of an invalid reference design."""
UNSWEPT = 1.0e2
"""Violation assigned to the motion constraints of a design that already fails a
static one: such designs rank behind every design that passes them."""


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

    def joined(name: str) -> np.ndarray:
        return np.concatenate((getattr(back, name)[:0:-1], getattr(sweep, name)[safe]))

    margin = joined("margin_at_bp_rad")
    q_dip = joined("q_dip_rad")
    backoff = float(np.max(np.maximum.accumulate(q_dip) - q_dip))
    b_range, bp_range = joined("b"), joined("bp")
    # The driven coupler A'-B must turn one way.
    to_b = b_range - np.array((parameters.ap.x_mm, parameters.ap.z_mm))
    turn = np.unwrap(np.arctan2(to_b[:, 1], to_b[:, 0]))
    turn = turn * (1.0 if turn[-1] >= turn[0] else -1.0)
    input_backoff = float(np.max(np.maximum.accumulate(turn) - turn))

    clearance = packaging.body_clearance_mm
    a = np.array((linkage.a.x_mm, 0.0))
    # The middle link A-B' passes the servo side.
    link = footprint.distance_to_segments(np.broadcast_to(a, bp_range.shape), bp_range)
    link_clearance = float(link.min()) - clearance

    curl_dip = curl_gap = math.nan
    gap_violation = 0.0
    if reached.size:
        pose = linkage.solve(0.0, curl)
        curl_dip = pose.q_dip_rad
        # BC is antiparallel to OA here: the gap is the distance between the two lines.
        curl_gap = -0.5 * (pose.b.z_mm + pose.c.z_mm)
        gap_violation = config.curl_gap_mm - curl_gap

    constraints = {
        **static,
        "curl_reached_deg": curl_shortfall,
        "range_closure_deg": math.degrees(max(
            safe_end - closes_to if reached.size else 0.0, extension_shortfall
        )),
        "transmission_deg": math.degrees(config.minimum_transmission_margin_rad - float(margin.min())),
        "dip_backoff_deg": math.degrees(backoff - config.dip_backoff_tolerance_rad),
        "input_backoff_deg": math.degrees(input_backoff - config.dip_backoff_tolerance_rad),
        # The DIP pin B' moves past the servo; its housing clears the body throughout.
        "dip_pin_mm": -(float(footprint.distance_to_segments(bp_range, bp_range).min())
                        - packaging.housing_radius_mm - clearance),
        "link_mm": -link_clearance if config.link_clearance else 0.0,
        "curl_gap_mm": gap_violation,
        "fingertip_mcp_mm": -(_fingertip_mcp_gap_mm(linkage, sweep, safe, config) - clearance),
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


def _static_constraints(
    linkage: FingerLinkage, footprint: ServoFootprint, config: FingerStudyConfig
) -> dict[str, float]:
    """Constraints of the reference geometry and the packaging alone."""

    packaging = config.packaging
    clearance = packaging.body_clearance_mm
    parameters = linkage.parameters
    a = np.array((linkage.a.x_mm, 0.0))
    spacing = min(linkage.a.distance_to(linkage.bp), linkage.bp.distance_to(linkage.b))
    length = math.hypot(parameters.b_x_mm + parameters.bc_length_mm, parameters.b_z_mm)
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
    "UNSWEPT",
    "evaluate",
]
