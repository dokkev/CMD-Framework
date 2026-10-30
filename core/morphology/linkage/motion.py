"""The linkage finger's motion: swept by its PIP servo from extension (and by
the PIP into hyperextension), and what holds along it.

The motion is safe while the four-bar closes, the transmission angle at
``B'`` stays in its range, the DIP and the PIP keep turning with the servo,
the ``B'`` pin clears the PIP servo (when checked) and, from extension on,
the pads clear the parts of other bodies (the MCP servo at every MCP
flexion sampled). :func:`sweep_motion` measures how far it stays safe, what
ends it, and over ``[0, phi]`` (``phi`` given) the torque ratio ``k``, the
fingertip's travel and each pad's face the links leave free.

On the servo grid an interval is taken to the first sample at or past its
end, so a measure is over slightly more than ``[0, phi]``.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np

from core.morphology.linkage.model import FingerLinkage, PipSweep
from core.morphology.linkage.packaging import (
    LinkagePackaging,
    assembly,
    contact_faces,
    driver_link_servo_gaps,
    fixed_margin_pairs,
    layout,
    link_capsule_margins,
    pad_margins,
    pip_footprint,
    poses,
    support_margins,
    usable_pad_lengths,
)
from core.morphology.packaging.assembly import margins
from core.morphology.packaging.faces import fold_free_lengths
from core.morphology.packaging.shapes import ServoFootprint

STOP_REASONS = ("closure", "transmission", "dip_backoff", "pip_backoff", "dip_pin", "pad_collision")
"""The unsafe conditions, in the order they are reported."""


@dataclass(frozen=True)
class MotionSettings:
    """How the motion is swept and what is checked along it. Angles in radians."""

    servo_samples_rad: Sequence[float]
    """The PIP servo's samples from extension (its sense of flexion found by the sweep)."""
    servo_step_rad: float
    hyperextension_rad: float
    """How far the PIP is swept back into hyperextension (to the stopper)."""
    mcp_samples_rad: Sequence[float]
    """MCP flexions at which the pads must clear the MCP servo."""
    transmission_angle_range_rad: tuple[float, float]
    dip_backoff_tolerance_rad: float
    link_radius_mm: float | None = None
    """The links and bars as capsules of this radius against the pads they are
    not mounted on, in the side view (``None``: not checked)."""
    proximal_carrier_height_mm: float | None = None
    """The proximal support as a solid plate (``packaging.proximal_carrier_polygon``)
    checked against the other segment (``None``: not checked)."""


@dataclass(frozen=True)
class FingerMotion:
    """The swept motion of one linkage (see :func:`sweep_motion`)."""

    phi_safe_rad: float
    """How far the safe motion reaches (at most pi), ``C_x`` aside."""
    phi_max_rad: float
    """The most flexion every check still allows, ``C`` ahead of ``O`` included (at most pi)."""
    stop_reason: str
    """What ends the safe motion: ``"curl"`` at 180 deg, an unsafe condition
    (:data:`STOP_REASONS`), ``"closure"`` or ``"horizon"``."""
    collision_pair: str
    """With ``"pad_collision"``, the first pair that collides, ``"first/second"``."""
    deepest_pip_rad: float
    """The PIP's angle as far back as the safe hyperextension reaches (0 or less)."""
    hyperextension_phi_rad: float
    usable_pip_rad: float
    usable_servo_rad: float
    """PIP and servo rotation at ``phi`` (the end of the measured interval)."""
    curl_tip_x_mm: float
    least_tip_x_mm: float
    """``x`` of ``C`` at ``phi``, and its least on ``[0, phi]``, MCP at 0."""
    worst_transmission_quality: float
    min_margin_rad: float
    """Least transmission margin (from 0 or 180 deg) at ``B'``, hyperextension to ``phi``."""
    toggle_margin_rad: float
    """Least margin from 0 or 180 deg of the angle at ``B`` (the limit position) over the same range."""
    driver_link_servo_clearance_mm: float
    closes_to_rad: float
    lever_max_mm: float
    torque_ratio_max: float
    """``k_max``: the worst ``d(phi)/d(theta_servo)`` on ``[0, phi]``."""
    torque_ratio_min: float
    servo_travel_rad: float
    pad_clearance_mm: float
    """Least pad margin on ``[0, phi]``, less the pairs set by definition."""
    fold_free: dict[str, float]
    """Each pad's length the links leave free on ``[0, phi]``
    (``packaging.usable_pad_lengths``), or with the links bent around the pads
    (``packaging.links_clear_pads``) each face less what the other hides."""
    integrity_free: dict[str, float] | None = None
    """Each face's length no solid comes within the integrity depth of on
    ``[0, phi]`` (``contact.loss.integrity_free``; ``None``: not asked)."""


def sweep_motion(linkage: FingerLinkage, packaging: LinkagePackaging, settings: MotionSettings, phi_rad: float,
                 footprint: ServoFootprint | None = None, integrity_depth_mm: float | None = None) -> FingerMotion:
    """Sweep ``linkage`` (see the module docstring) and measure it on ``[0, phi_rad]``."""

    footprint = pip_footprint(packaging, linkage) if footprint is None else footprint
    step = settings.servo_step_rad
    sense = servo_sense(linkage, step)
    forward = np.asarray(settings.servo_samples_rad)
    sweep = linkage.sweep_servo(sense * forward)
    # Hyperextension is a PIP requirement: solved by the PIP, to the stopper exactly.
    count = int(math.ceil(settings.hyperextension_rad / step - 1e-9))
    back = linkage.sweep_pip(-np.minimum(np.arange(count + 1) * step, settings.hyperextension_rad))
    n_back = len(back.q_pip_rad) - 1  # back samples besides 0

    def joined(name: str) -> np.ndarray:
        """The back samples, then the forward sweep, in increasing flexion."""

        return np.concatenate((getattr(back, name)[n_back:0:-1], getattr(sweep, name)))

    unsafe = unsafe_conditions(linkage, footprint, joined, sense, packaging, settings)
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
                          joined("bp")[swept], settings.mcp_samples_rad)
    if settings.link_radius_mm is not None:
        margins.update(link_capsule_margins(linkage, packaging, joined("b")[swept], joined("c")[swept],
                                            joined("bp")[swept], settings.link_radius_mm))
    if settings.proximal_carrier_height_mm is not None:
        margins.update(support_margins(linkage, packaging, joined("b")[swept], joined("c")[swept],
                                       settings.proximal_carrier_height_mm))
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

    # Hyperextension: how far back the PIP goes safely.
    safe_back = 0
    while safe_back < n_back and not any_unsafe[n_back - 1 - safe_back]:
        safe_back += 1
    deepest = float(back.q_pip_rad[safe_back]) if not any_unsafe[n_back] else 0.0

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

    # [0, phi] on the grid: to the first sample at or past it (the whole
    # safe motion when it falls short).
    at = np.flatnonzero(total >= phi_rad - 1e-12)
    last = int(at[0]) if at.size else max(first_unsafe - 1, 0)
    window = np.zeros(len(total), dtype=bool)
    window[:last + 1] = True
    lever, ratio, travel = servo_lever(sweep, window)
    ratio_min = least_torque_ratio(sweep, window)
    tip_x = sweep.c[:, 0]
    least_tip = float(tip_x[window].min())
    # phi_max: the first sample any check fails at, C_x included.
    failing = forward_unsafe | (tip_x < 0.0)
    first_failing = int(np.flatnonzero(failing)[0]) if failing.any() else len(total)
    phi_max = float(min(total[first_failing - 1], math.pi)) if first_failing > 0 else 0.0

    window_joined = np.concatenate((np.ones(n_back, dtype=bool), window))
    margin = joined("margin_at_bp_rad")[window_joined]
    toggle = joined("margin_at_b_rad")[window_joined]
    link = driver_link_servo_gaps(linkage, footprint, joined("bp")[window_joined])
    if packaging.links_clear_pads:  # the links bent out of the way: each face less what the other hides
        free = fold_free_lengths(*contact_faces(linkage, packaging, sweep.b[window], sweep.c[window]))
    else:
        free = usable_pad_lengths(linkage, packaging, sweep.b[window], sweep.bp[window], sweep.c[window])
    kept = None
    if integrity_depth_mm is not None:
        # Only here: the loss module (and the other mechanisms' solids it
        # reads) stays out of every evaluation that asks no integrity.
        from core.morphology.contact.loss import integrity_free, linkage_scene

        kept = integrity_free(linkage_scene(linkage, packaging, sweep.b[window], sweep.c[window], sweep.bp[window],
                                            np.degrees(total[window])), integrity_depth_mm)
    return FingerMotion(
        phi_safe_rad=reach,
        phi_max_rad=phi_max,
        stop_reason=stop_reason,
        collision_pair=collision_pair,
        deepest_pip_rad=deepest,
        hyperextension_phi_rad=float(back.total_flexion_rad[safe_back]),
        usable_pip_rad=float(sweep.q_pip_rad[last]),
        usable_servo_rad=float(forward[last]),
        curl_tip_x_mm=float(tip_x[last]),
        least_tip_x_mm=least_tip,
        worst_transmission_quality=float(np.sin(margin).min()),
        min_margin_rad=float(margin.min()),
        toggle_margin_rad=float(toggle.min()),
        driver_link_servo_clearance_mm=float(link.min()) - packaging.body_clearance_mm,
        closes_to_rad=float(sweep.q_pip_rad[-1]),
        lever_max_mm=lever,
        torque_ratio_max=ratio,
        torque_ratio_min=ratio_min,
        servo_travel_rad=travel,
        pad_clearance_mm=float(reported_pad[n_back:][window].min()),
        fold_free={side: float(lengths.min()) for side, lengths in free.items()},
        integrity_free=kept,
    )


def servo_sense(linkage: FingerLinkage, step_rad: float) -> float:
    """The sign of the PIP servo's rotation that flexes the PIP (``+1`` or ``-1``)."""

    return 1.0 if linkage.sweep_servo([0.0, step_rad]).q_pip_rad[-1] >= 0.0 else -1.0


def unsafe_conditions(linkage: FingerLinkage, footprint: ServoFootprint, joined: Callable[[str], np.ndarray],
                      sense: float, packaging: LinkagePackaging, settings: MotionSettings) -> dict[str, np.ndarray]:
    """Per-sample unsafe flags along the joined sweep, in increasing flexion
    (the caller adds the pads). Backoffs are measured from the start of the
    sweep (deepest hyperextension); the servo angle is signed to rise with flexion."""

    tolerance = settings.dip_backoff_tolerance_rad
    dip_pin = layout(packaging).pair("bp_pin", "pip_servo")  # unless B' may pass the servo
    low, high = settings.transmission_angle_range_rad
    bp = joined("bp")
    angle = transmission_angle(np.array((linkage.a.x_mm, linkage.a.z_mm)), bp, joined("b"))
    q_dip, q_pip, servo = joined("q_dip_rad"), joined("q_pip_rad"), sense * joined("servo_rad")
    return {
        "closure": np.zeros(len(angle), dtype=bool),
        "transmission": np.minimum(angle - low, high - angle) < 0.0,
        "dip_backoff": np.maximum.accumulate(q_dip) - q_dip > tolerance,
        # The PIP and the servo turn together.
        "pip_backoff": (np.maximum.accumulate(q_pip) - q_pip > tolerance)
        | (np.maximum.accumulate(servo) - servo > tolerance),
        # The B' pin may cross the servo body when they sit in different Y layers.
        "dip_pin": (np.zeros(len(angle), dtype=bool) if dip_pin is None
                    else margins(assembly(linkage, packaging), poses(linkage, joined("b"), joined("c"), bp), packaging,
                                 [dip_pin])[dip_pin.key] < 0.0),
    }


def transmission_angle(a: np.ndarray, bp: np.ndarray, b: np.ndarray) -> np.ndarray:
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


def least_torque_ratio(sweep: PipSweep, workspace: np.ndarray) -> float:
    """Least ``k = d(phi)/d(theta_servo)`` over the workspace, as :func:`servo_lever` takes it."""

    theta = sweep.servo_rad[workspace]
    if len(theta) < 3:
        return math.nan
    return float(np.min(np.abs(np.gradient(sweep.total_flexion_rad[workspace], theta))))


__all__ = [
    "STOP_REASONS",
    "FingerMotion",
    "MotionSettings",
    "least_torque_ratio",
    "servo_lever",
    "servo_sense",
    "sweep_motion",
    "transmission_angle",
    "unsafe_conditions",
]
