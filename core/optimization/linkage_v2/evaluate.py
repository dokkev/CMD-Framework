"""Evaluate one linkage design in formulation v2.

The linkage, its packaging and every check are formulation v1's
(``core.optimization.linkage.evaluate``); what changes is the interval and
what is optimized:

- **ROM.** The motion must be danger-free on ``[0, phi_req + margin]``
  (``target``; the margin is ``flexion_stopper_margin_rad``):
  no closure loss, transmission-margin loss, DIP or PIP backoff, or pad
  collision before ``phi`` reaches it. The linkage validity (assembly,
  transmission margin, backoffs) also holds from the PIP hyperextension, as
  in v1, where the pads are not checked. Flexion past the target does not
  matter: the finger is stopped there.
- **On ``[0, phi_req]``.** ``k_max`` (objective) and the least pad gap
  (reported). The least ``C_x`` with the MCP at 0 (``C_x >= 0``) is taken
  over the danger-free interval ``[0, phi_req + margin]``, as the pads.
- **Objectives.** ``k_max`` and ``L_finger = |OC|`` in the extended pose, with
  ``C = (B_x + BC, B_z)`` and ``B`` as assembled (on the machining grid).

On the servo grid the sample interval is taken to the first sample at or past
the target, so a design is judged on slightly more than ``[0, target]``.
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from core.morphology.linkage import FingerLinkage, FingerLinkageParameters
from core.morphology.linkage.packaging import fixed_margin_pairs, pad_margins, pad_spans

from core.optimization.linkage.evaluate import (
    INVALID,
    STATIC_CONSTRAINTS,
    STOP_REASONS,
    UNSWEPT,
    FingerEvaluation,
    _drive,
    _fingertip_mcp_gap_mm,
    _least_torque_ratio,
    _pad_lengths,
    _static_constraints,
    _unsafe_conditions,
    servo_lever,
)
from core.optimization.linkage_v2.config import LinkageV2Config


MOTION_CONSTRAINTS = (
    "rom_deg",
    "hyperextension_deg",
    "curl_tip_x_mm",
    "middle_link_servo_mm",
    "servo_travel_deg",
    "torque_ratio_max",
    "fingertip_mcp_mm",
)
"""``rom_deg``: how far short of the target the safe motion stops (``<= 0``: it
reaches it). The last four are v1's, held unless enabled."""
CONSTRAINT_NAMES = STATIC_CONSTRAINTS + MOTION_CONSTRAINTS


@dataclass(frozen=True)
class LinkageV2Evaluation(FingerEvaluation):
    """v1's evaluation record, measured on ``[0, phi_req]``, and these."""

    finger_length_mm: float = math.nan
    """``|OC|`` in the extended pose: the second objective."""
    required_rom_rad: float = math.nan
    phi_max_rad: float = math.nan
    """The most flexion every constraint would still allow (``C_x`` included),
    at most 180 deg: information only."""
    least_tip_x_mm: float = math.nan
    """Least ``x`` of ``C`` on ``[0, phi_req + margin]``, the MCP at 0."""
    required_proximal_mm: float = math.nan
    required_distal_mm: float = math.nan
    """``L_p`` and ``L_d`` the design was asked to accommodate."""
    proximal_face_mm: float = math.nan
    distal_face_mm: float = math.nan
    """Each pad's palmar face, its ``a-b`` less the chamfer."""
    bound_hits: tuple[str, ...] = ()
    """Search limits the design sits at (set by the mechanism, which knows the box)."""

    @property
    def g(self) -> tuple[float, ...]:
        return tuple(self.constraints[name] for name in CONSTRAINT_NAMES)


def finger_length_mm(parameters: FingerLinkageParameters) -> float:
    """``|OC|`` in the extended pose, ``C = (B_x + BC, B_z)``."""

    return math.hypot(parameters.b_x_mm + parameters.bc_length_mm, parameters.b_z_mm)


def _faces(parameters: FingerLinkageParameters, packaging) -> dict[str, float]:
    spans = pad_spans(parameters, packaging)
    return {f"{name}_face_mm": max(spans[f"{name}_pad"].length_mm
                                   - min(getattr(packaging, f"{name}_pad").chamfer_mm,
                                         spans[f"{name}_pad"].length_mm), 0.0)
            for name in ("proximal", "distal")}


def evaluate(parameters: FingerLinkageParameters, config: LinkageV2Config) -> LinkageV2Evaluation:
    """Evaluate one design (static checks first, as in v1)."""

    ab_mm = math.hypot(parameters.b_x_mm - parameters.a_x_mm, parameters.b_z_mm - parameters.a_z_mm)
    packaging = config.packaging
    common = dict(finger_length_mm=finger_length_mm(parameters), required_rom_rad=config.required_rom_rad,
                  required_proximal_mm=config.minimum_proximal_support_mm,
                  required_distal_mm=config.minimum_distal_support_mm,
                  **_pad_lengths(parameters, packaging), **_faces(parameters, packaging))
    try:
        linkage = FingerLinkage(parameters)
    except ValueError:  # the reference pose is a dead point
        constraints = {name: INVALID for name in CONSTRAINT_NAMES}
        constraints["valid_reference"] = 1.0
        return LinkageV2Evaluation(parameters, MappingProxyType(constraints), 0.0, ab_mm, **common)

    footprint = packaging.pip_footprint(linkage)
    static = _static_constraints(linkage, footprint, config)
    if max(static.values()) > 0.0:
        constraints = {**static, **{name: UNSWEPT for name in MOTION_CONSTRAINTS}}
        return LinkageV2Evaluation(parameters, MappingProxyType(constraints), 0.0, ab_mm,
                                   servo_lowest_z_mm=footprint.lowest_z_mm, **common)

    step = config.servo_step_rad
    sense = 1.0 if _drive(linkage, packaging, [0.0, step]).q_pip_rad[-1] >= 0.0 else -1.0
    forward = np.asarray(config.servo_samples_rad)
    sweep = _drive(linkage, packaging, sense * forward)
    transmission, limit_position = (("margin_at_b_rad", "margin_at_bp_rad") if packaging.pip_servo_on_a
                                    else ("margin_at_bp_rad", "margin_at_b_rad"))
    count = int(math.ceil(config.hyperextension_rad / step - 1e-9))
    back = linkage.sweep_pip(-np.minimum(np.arange(count + 1) * step, config.hyperextension_rad))
    if packaging.pip_servo_on_a:
        back = dataclasses.replace(back, servo_rad=back.q_pip_rad)
    n_back = len(back.q_pip_rad) - 1

    def joined(name: str) -> np.ndarray:
        return np.concatenate((getattr(back, name)[n_back:0:-1], getattr(sweep, name)))

    unsafe = _unsafe_conditions(
        footprint, joined(transmission), joined("q_dip_rad"), joined("q_pip_rad"),
        sense * joined("servo_rad"), joined("bp"), config,
    )
    total = sweep.total_flexion_rad
    # The pads from extension up to the first other unsafe sample or the curl
    # (180 deg), as far as phi_max needs them.
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
    reported = [margin for pair, margin in margins.items() if pair not in fixed_margin_pairs(packaging)]
    reported_pad = np.full(len(unsafe["closure"]), math.inf)
    if reported:
        reported_pad[swept] = np.min(np.stack(reported), axis=0)
    unsafe["pad_collision"] = pad < 0.0
    # Past the pads' last sample nothing is known to be safe.
    unsafe["pad_collision"][n_back + end + 1:] = True
    any_unsafe = np.any(np.stack(list(unsafe.values())), axis=0)

    # Hyperextension, as in v1.
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
    rom_violation = math.degrees(config.target_rad - reach)

    def up_to(phi: float) -> tuple[int, np.ndarray]:
        """``[0, phi]`` on the grid: to the first sample at or past ``phi`` (the
        whole safe motion when it falls short)."""

        at = np.flatnonzero(total >= phi - 1e-12)
        last = int(at[0]) if at.size else max(first_unsafe - 1, 0)
        mask = np.zeros(len(total), dtype=bool)
        mask[:last + 1] = True
        return last, mask

    last, window = up_to(config.required_rom_rad)  # k_max and the reported gaps
    _, danger_free = up_to(config.target_rad)       # C_x, as the pads and the validity
    lever, ratio, travel = servo_lever(sweep, window)
    ratio_min = _least_torque_ratio(sweep, window)
    tip_x = sweep.c[:, 0]
    least_tip = float(tip_x[danger_free].min())
    tip_violation = config.curl_tip_min_x_mm - least_tip if config.enforce_curl_tip else 0.0
    # phi_max: the first sample any constraint fails at, C_x included.
    failing = forward_unsafe.copy()
    if config.enforce_curl_tip:
        failing |= tip_x < config.curl_tip_min_x_mm
    first_failing = int(np.flatnonzero(failing)[0]) if failing.any() else len(total)
    phi_max = float(min(total[first_failing - 1], math.pi)) if first_failing > 0 else 0.0

    window_joined = np.concatenate((np.ones(n_back, dtype=bool), window))
    margin = joined(transmission)[window_joined]
    toggle = joined(limit_position)[window_joined]
    bp_range = joined("bp")[window_joined]
    pad_clearance = float(reported_pad[n_back:][window].min())
    clearance = packaging.body_clearance_mm
    a = np.array((linkage.a.x_mm, linkage.a.z_mm))
    link = footprint.distance_to_segments(np.broadcast_to(a, bp_range.shape), bp_range)
    middle_link_clearance = float(link.min()) - clearance
    fingertip_violation = 0.0
    if config.enforce_fingertip_mcp:
        fingertip_violation = -(_fingertip_mcp_gap_mm(linkage, sweep, window, config) - clearance)

    constraints = {
        **static,
        "rom_deg": rom_violation,
        "hyperextension_deg": hyper_violation,
        "curl_tip_x_mm": tip_violation,
        "middle_link_servo_mm": -middle_link_clearance if config.require_middle_link_servo_clearance else 0.0,
        "servo_travel_deg": (math.degrees(travel - config.servo_travel_max_rad)
                             if config.servo_travel_max_rad is not None else 0.0),
        "torque_ratio_max": ratio - config.torque_ratio_max if config.torque_ratio_max is not None else 0.0,
        "fingertip_mcp_mm": fingertip_violation,
    }
    return LinkageV2Evaluation(
        parameters=parameters,
        constraints=MappingProxyType(constraints),
        worst_transmission_quality=float(np.sin(margin).min()),
        ab_mm=ab_mm,
        phi_safe_rad=reach,
        usable_pip_rad=float(sweep.q_pip_rad[last]),
        usable_servo_rad=float(forward[last]),
        stop_reason=stop_reason,
        collision_pair=collision_pair,
        curl_tip_x_mm=float(tip_x[last]),
        min_margin_rad=float(margin.min()),
        toggle_margin_rad=float(toggle.min()),
        hyperextension_phi_rad=float(back.total_flexion_rad[safe_back]),
        middle_link_servo_clearance_mm=middle_link_clearance,
        servo_lowest_z_mm=footprint.lowest_z_mm,
        closes_to_rad=float(sweep.q_pip_rad[-1]),
        lever_max_mm=lever,
        torque_ratio_max=ratio,
        torque_ratio_min=ratio_min,
        servo_travel_rad=travel,
        pad_clearance_mm=pad_clearance,
        phi_max_rad=phi_max,
        least_tip_x_mm=least_tip,
        **common,
    )


__all__ = ["CONSTRAINT_NAMES", "MOTION_CONSTRAINTS", "LinkageV2Evaluation", "evaluate", "finger_length_mm"]
