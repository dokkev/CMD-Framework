"""Evaluate one serial 2R driven through a parallel four-bar against the
linkage study's goal.

The distal segment ``A-C`` turns about the pin ``A`` by ``phi``, as the 2R's.
The servo's shaft is on ``A' = A + (dx, dz)``, its body laid back along
``-X`` tilted dorsally by the goal's PIP servo tilt (as the linkage's); it
turns the crank ``A'-Q``, and the coupler ``Q-R`` (parallel and equal to
``A'-A``) turns the distal crank ``A-R``, fixed to the distal segment, by the
same angle: ``phi = theta``, so ``k = 1``. A parallelogram reaches its dead
centre where the cranks lie along ``A'-A``: the transmission angle (between
the coupler and the cranks) must stay within the goal's range, as the
linkage's at ``B'``; outside it the motion is unsafe.

Every other rule is the idealized 2R's (``core.optimization.revolute``,
``joint_servo=False``) and the linkage's: the pads hang ``A``'s pin radius
below their segment lines and are checked against each other, ``A``'s pin and
the MCP servo; the servo against the distal pad, the proximal pad and the
MCP servo; the pins ``Q`` and ``R`` against the pads they do not belong to.
The links are in other layers (unchecked), but where one passes through a
pad's block in the side view it runs across the contact interface: each
pad's effective length is its face up to the first link crossing it, the
least on ``[0, phi_req]`` (``packaging.usable_pad_lengths``'s rule).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from core.morphology.linkage.packaging import (
    ServoFootprint,
    _block_extent,
    _polygon_to_disk,
    _trimmed,
    footprint_gaps,
)

from core.optimization.revolute.evaluate import (
    UNSWEPT,
    RevoluteEvaluation,
    RevoluteGeometry,
    _between_samples,
    _margins as revolute_margins,
    geometry as revolute_geometry,
)
from core.optimization.revolute.config import VARIABLE_NAMES as REVOLUTE_NAMES
from core.optimization.revolute_parallel.config import VARIABLE_NAMES, ParallelConfig


STATIC_CONSTRAINTS = ("extended_length_mm", "proximal_length_mm", "mcp_servo_mm", "pip_servo_pad_mm",
                      "proximal_support_mm", "distal_support_mm", "pin_pad_mm", "pad_collision_mm")
MOTION_CONSTRAINTS = ("rom_deg", "curl_tip_x_mm", "pad_usable_mm")
CONSTRAINT_NAMES = STATIC_CONSTRAINTS + MOTION_CONSTRAINTS


@dataclass(frozen=True)
class ParallelGeometry:
    """The 2R's solids (``joint``) with the servo on ``A'`` and the parallelogram."""

    joint: RevoluteGeometry
    ap: np.ndarray
    crank_mm: float
    crank_angle_deg: float
    servo: ServoFootprint

    def __getattr__(self, name: str):  # the 2R's solids and poses
        return getattr(self.joint, name)

    @property
    def a(self) -> np.ndarray:
        return np.array((self.joint.a_x_mm, 0.0))

    def crank(self, phi_rad) -> np.ndarray:
        """The cranks' direction at distal flexions ``phi``, ``(n, 2)``."""

        angle = math.radians(self.crank_angle_deg) - np.atleast_1d(np.asarray(phi_rad, dtype=float))
        return np.stack((np.cos(angle), np.sin(angle)), axis=1)

    def q(self, phi_rad) -> np.ndarray:
        return self.ap + self.crank_mm * self.crank(phi_rad)

    def r(self, phi_rad) -> np.ndarray:
        return self.a + self.crank_mm * self.crank(phi_rad)

    def transmission_angle_rad(self, phi_rad) -> np.ndarray:
        """Between the coupler ``Q-R`` (along ``A'->A``) and the cranks, in ``[0, pi]``."""

        coupler = (self.a - self.ap) / np.linalg.norm(self.a - self.ap)
        return np.arccos(np.clip(self.crank(phi_rad) @ coupler, -1.0, 1.0))

    @property
    def distal_reach_mm(self) -> float:
        """How fast a moving point goes per radian of ``phi``: the distal pad
        about ``A``, or a crank's end."""

        return max(self.joint.distal_reach_mm, self.crank_mm)


def geometry(x: Sequence[float], config: ParallelConfig) -> ParallelGeometry:
    v = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    joint = revolute_geometry(tuple(x[:len(REVOLUTE_NAMES)]), config.base)
    ap = np.array((joint.a_x_mm + v["ap_from_a_x_mm"], v["ap_from_a_z_mm"]))
    servo = config.goal.packaging.pip_servo
    tilt = config.tilt_rad
    footprint = ServoFootprint((float(ap[0]), float(ap[1])), (-math.cos(tilt), math.sin(tilt)),
                               servo.radius_mm, servo.rear_extent_mm)
    return ParallelGeometry(joint, ap, v["crank_mm"], v["crank_angle_deg"], footprint)


@dataclass(frozen=True)
class ParallelEvaluation(RevoluteEvaluation):
    """Constraints, objectives and diagnostics, named as the linkage's and the 2R's."""

    proximal_usable_length_mm: float = math.nan
    distal_usable_length_mm: float = math.nan
    """Each pad's face the links leave free from its outer end, the least on ``[0, phi_req]``."""
    min_transmission_deg: float = math.nan
    """The least transmission angle's margin from 0 or 180 deg on ``[0, phi_req]``."""

    @property
    def g(self) -> tuple[float, ...]:
        return tuple(self.constraints[name] for name in CONSTRAINT_NAMES)

    @property
    def proximal_effective_mm(self) -> float:
        return min(self.proximal_face_mm, self.proximal_usable_length_mm)

    @property
    def distal_effective_mm(self) -> float:
        return min(self.distal_face_mm, self.distal_usable_length_mm)


def evaluate(x: Sequence[float], config: ParallelConfig) -> ParallelEvaluation:
    """Evaluate one design (static checks first; a design failing one is not swept)."""

    goal = config.goal
    packaging = goal.packaging
    clearance = packaging.body_clearance_mm
    geo = geometry(x, config)
    joint = geo.joint
    radius = packaging.pivot_radius_mm
    length_low, length_high = goal.extended_length_bounds_mm
    oa_low, oa_high = goal.proximal_length_bounds_mm
    oa = [value for value in (None if oa_low is None else oa_low - joint.a_x_mm,
                              None if oa_high is None else joint.a_x_mm - oa_high) if value is not None]
    static = {
        "extended_length_mm": max(length_low - joint.finger_length_mm, joint.finger_length_mm - length_high),
        "proximal_length_mm": max(oa, default=0.0),
        "proximal_support_mm": max(goal.minimum_proximal_support_mm - joint.proximal_length_mm,
                                   -joint.proximal_room_mm,
                                   x[VARIABLE_NAMES.index("app_x_mm")] - (joint.a_x_mm - config.joint_radius_mm)),
        "distal_support_mm": max(goal.minimum_distal_support_mm - joint.distal_length_mm, -joint.distal_room_mm,
                                 config.joint_radius_mm - x[VARIABLE_NAMES.index("bpp_from_a_mm")]),
    }
    if max(static.values()) <= 0.0:
        mcp_angles = np.asarray(goal.mcp_servo_angles_rad)
        static["mcp_servo_mm"] = -(packaging.mcp_servo_gap_mm(geo.servo, mcp_angles) - clearance)
        static["pip_servo_pad_mm"] = (-(float(footprint_gaps(joint.proximal_pad[None], geo.servo, packaging)[0])
                                        - clearance) if packaging.pip_servo_pad_check else 0.0)
        # R's pin on its own distal segment: touching its pad, not inside it.
        r_local = np.array(((geo.crank_mm * math.cos(math.radians(geo.crank_angle_deg)),
                             geo.crank_mm * math.sin(math.radians(geo.crank_angle_deg))),))
        gap = float(_polygon_to_disk(joint.distal_pad_local[None], r_local, radius("b"))[0])
        static["pin_pad_mm"] = 0.0 if gap >= -1e-9 else -gap
    if max(static.values()) <= 0.0:
        margins_at_rest = _margins(geo, np.zeros(1), config)
        static["pad_collision_mm"] = -min(float(margin[0]) for margin in margins_at_rest.values())
    for name in STATIC_CONSTRAINTS:
        static.setdefault(name, UNSWEPT)
    common = dict(x=tuple(float(value) for value in x), finger_length_mm=joint.finger_length_mm,
                  proximal_length_mm=joint.a_x_mm, required_rom_rad=goal.required_rom_rad,
                  proximal_support_length_mm=joint.proximal_length_mm,
                  distal_support_length_mm=joint.distal_length_mm,
                  proximal_face_mm=joint.proximal_face_mm, distal_face_mm=joint.distal_face_mm,
                  proximal_depth_mm=joint.proximal_drop_mm + packaging.proximal_pad.height_mm,
                  distal_depth_mm=joint.distal_drop_mm + packaging.distal_pad.height_mm)
    if max(static.values()) > 0.0:
        constraints = {**static, **{name: UNSWEPT for name in MOTION_CONSTRAINTS}}
        return ParallelEvaluation(constraints=MappingProxyType(constraints), **common)

    phi = np.asarray(goal.servo_samples_rad)
    margins = _between_samples(geo, phi, _margins(geo, phi, config, until_unsafe=True), config, _margins)
    least = np.min(np.stack(list(margins.values())), axis=0)
    low, high = goal.transmission_angle_range_rad
    mu = geo.transmission_angle_rad(phi)
    dead = (mu < low - 1e-12) | (mu > high + 1e-12)
    unsafe = (least < 0.0) | dead
    first_unsafe = int(np.flatnonzero(unsafe)[0]) if unsafe.any() else len(phi)
    reach = float(min(phi[first_unsafe - 1], math.pi)) if first_unsafe > 0 else 0.0
    stop_reason, collision_pair = ("curl" if reach >= math.pi - 1e-12 else "horizon"), ""
    if first_unsafe < len(phi) and reach < math.pi - 1e-12:
        if dead[first_unsafe]:
            stop_reason = "transmission"
        else:
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
    free = usable_pad_lengths(geo, phi[window], config)
    proximal_usable, distal_usable = float(free["proximal_pad"].min()), float(free["distal_pad"].min())
    reported = [margin for pair, margin in margins.items() if pair != ("mcp_servo", "proximal_pad")]
    constraints = {
        **static,
        "rom_deg": math.degrees(goal.required_rom_rad - reach),
        "curl_tip_x_mm": -float(tip_x[window].min()),
        "pad_usable_mm": max(goal.minimum_proximal_support_mm - min(joint.proximal_face_mm, proximal_usable),
                             goal.minimum_distal_support_mm - min(joint.distal_face_mm, distal_usable)),
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
        min_transmission_deg=float(np.degrees(np.minimum(mu[window], math.pi - mu[window]).min())),
        **common,
    )


def _margins(geo: ParallelGeometry, phi: np.ndarray, config: ParallelConfig,
             until_unsafe: bool = False) -> dict[tuple[str, str], np.ndarray]:
    """The idealized 2R's pairs (the pads, ``A``'s pin, the MCP servo), then the
    servo and the parallelogram's pins against the pads."""

    packaging = config.goal.packaging
    clearance = packaging.body_clearance_mm
    radius = packaging.pivot_radius_mm
    count = len(phi)
    distal = geo.joint.distal_pad(phi)
    proximal = np.broadcast_to(geo.joint.proximal_pad, (count, *geo.joint.proximal_pad.shape))
    margins = {
        ("distal_pad", "pip_servo"): footprint_gaps(distal, geo.servo, packaging) - clearance,
        ("q_pin", "proximal_pad"): _polygon_to_disk(proximal, geo.q(phi), radius("bp")) - clearance,
        ("q_pin", "distal_pad"): _polygon_to_disk(distal, geo.q(phi), radius("bp")) - clearance,
        ("r_pin", "proximal_pad"): _polygon_to_disk(proximal, geo.r(phi), radius("b")) - clearance,
    }
    margins = {pair: np.where(np.abs(margin) < 1e-9, 0.0, margin) for pair, margin in margins.items()}
    return {**revolute_margins(geo.joint, phi, config.base, until_unsafe=until_unsafe), **margins}


def usable_pad_lengths(geo: ParallelGeometry, phi: np.ndarray, config: ParallelConfig) -> dict[str, np.ndarray]:
    """Per-sample length of each pad's face the links (``A'-Q``, ``Q-R``,
    ``A-R``, each less its pins) leave free from its outer end."""

    packaging = config.goal.packaging
    radius = packaging.pivot_radius_mm
    count = len(phi)
    a = np.broadcast_to(geo.a, (count, 2))
    ap = np.broadcast_to(geo.ap, (count, 2))
    q, r = geo.q(phi), geo.r(phi)
    links = [_trimmed(ap, q, radius("ap"), radius("bp")), _trimmed(q, r, radius("bp"), radius("b")),
             _trimmed(a, r, radius("a"), radius("b"))]

    joint = geo.joint
    pad = joint.proximal_pad  # a block on its top face, in the proximal frame
    top, start, end = pad[:, 1].max(), pad[:, 0].min(), pad[:, 0].max()
    height = top - pad[:, 1].min()
    near = np.full(count, np.inf)
    for first, second in links:
        low, _ = _block_extent(first - (0.0, top), second - (0.0, top), start, end, height)
        near = np.fmin(near, low)
    proximal_free = np.where(np.isfinite(near), near - start, end - start)

    local_pad = joint.distal_pad_local  # u from A along A->C, v dorsal
    top, start, end = local_pad[:, 1].max(), local_pad[:, 0].min(), local_pad[:, 0].max()
    height = top - local_pad[:, 1].min()
    e = np.stack((np.cos(phi), -np.sin(phi)), axis=1)
    n = np.stack((-e[:, 1], e[:, 0]), axis=1)

    def local(points: np.ndarray) -> np.ndarray:  # v from the pad's top face
        rel = points - a
        return np.stack((np.sum(rel * e, axis=1), np.sum(rel * n, axis=1) - top), axis=1)

    far = np.full(count, -np.inf)
    for first, second in links:
        _, high = _block_extent(local(first), local(second), start, end, height)
        far = np.fmax(far, high)
    distal_free = np.where(np.isfinite(far), end - far, end - start)
    return {"proximal_pad": np.maximum(proximal_free, 0.0), "distal_pad": np.maximum(distal_free, 0.0)}


__all__ = ["CONSTRAINT_NAMES", "ParallelEvaluation", "ParallelGeometry", "evaluate", "geometry",
           "usable_pad_lengths"]
