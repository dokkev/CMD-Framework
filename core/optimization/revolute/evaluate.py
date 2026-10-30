"""Evaluate one serial 2R (revolute) finger against the linkage study's goal.

Its solids and their gaps are the morphology's (``core.morphology.revolute``);
this module holds them to the goal's bounds as constraints.

The MCP is at ``O`` and the distal joint at ``A = (|OA|, 0)``; the distal
segment ``A-C`` turns by ``phi`` about ``A`` (positive flexion turns ``+X``
toward ``-Z``), so ``phi`` is the same rotation of the distal segment
relative to the proximal one as the linkage's ``q_pip + q_dip``. The distal
servo's shaft is on ``A``, its body fixed to the proximal segment and laid
back along ``-X`` tilted dorsally by ``servo_tilt_deg``; it drives the joint
directly, so ``k = 1``.

Every rule is the linkage's goal (``core.optimization.linkage.config``),
but for where the pads sit: each on its own segment and off the joint's
housing (a fixed pivot's crease, padless in practice), the servo's radius
(the idealized 2R's: its pin's) either side of ``A``. The proximal pad hangs
a searched drop below ``O-A`` (``proximal_drop_mm``), laid from the MCP
servo to ``A''``, at most ``A_x`` less the housing's radius; the distal pad
a searched drop below ``A-C`` (``distal_drop_mm``), laid from ``C`` to
``B''``, at least that radius past ``A``. The distal phalanx is a truss of
links from ``A`` to the pad's two ends, in other layers than the servo (as
the linkage's links), so only the pad is checked. No clearance, overlaps by depth; checked over
``[0, 180 deg]`` from extension: the distal pad against the proximal pad,
the distal servo and the MCP servo (MCP flexion up to its max); statically
the proximal pad against the distal servo and the MCP servo, and the two
servos (MCP hyperextension to extension). No link crosses a pad, so a pad's
effective length is its face less what the other face hides in the fold
(``packaging.fold_free_lengths``, as the linkage's). As the linkage's, the
distal pad sits on a carrier plate (``goal.collision.param.distal_carrier_height_mm``)
checked against the joint's servo, and the joint must hyperextend
(``goal.motion.hyperextension_deg``) with every pair clear.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from core.morphology.contact.loss import dorsal_space_mm, integrity_free, joint_scene
from core.morphology.packaging.gaps import footprint_gaps
from core.morphology.packaging.shapes import polygon_to_disk
from core.morphology.revolute import solids
from core.morphology.revolute.solids import (
    JointChecks,
    RevoluteGeometry,
    between_samples,
    fold_free,
    hyperextension_margin,
)

from core.optimization.revolute.config import VARIABLE_NAMES, RevoluteConfig, pad_reach_mm


STATIC_CONSTRAINTS = ("extended_length_mm", "proximal_length_mm", "mcp_servo_mm", "pip_servo_pad_mm",
                      "proximal_support_mm", "distal_support_mm", "dorsal_space_mm", "pad_collision_mm")
MOTION_CONSTRAINTS = ("rom_deg", "hyperextension_mm", "curl_tip_x_mm", "pad_usable_mm")
CONSTRAINT_NAMES = STATIC_CONSTRAINTS + MOTION_CONSTRAINTS
UNSWEPT = 1.0e2


def geometry(x: Sequence[float], config: RevoluteConfig) -> RevoluteGeometry:
    """The solids of a design vector, with the study's packaging and supports."""

    goal = config.goal
    return solids.geometry(x, goal.packaging, goal.proximal_carrier_height_mm)


def joint_checks(config: RevoluteConfig) -> JointChecks:
    """What the study checks a 2R's motion against."""

    goal = config.goal
    return JointChecks(goal.packaging, goal.mcp_samples_rad, config.joint_servo, config.joint_radius_mm,
                       goal.link_radius_mm)


def dorsal_space_violation(geo, goal, mechanism: str) -> float:
    """How far (mm) the extended finger rises past ``goal.link.dorsal_space_mm`` above ``O``'s line (0: none set)."""

    if goal.dorsal_space_mm is None:
        return 0.0
    return (dorsal_space_mm(joint_scene(mechanism, geo, goal.packaging, [0.0]), goal.link_radius_mm or 0.0)
            - goal.dorsal_space_mm)


def usable_lengths(geo, phi_rad: np.ndarray, goal, mechanism: str) -> tuple[float, float]:
    """Each face's length kept through the flexions ``phi`` (the window to
    the required flexion): less the fold at each, or with
    ``goal.integrity_depth_mm`` the integrity rule
    (``core.morphology.contact.loss.integrity_free``) over the
    ``mechanism``'s solids."""

    if goal.integrity_depth_mm is None and not goal.count_fold:  # a module clear through the motion counts whole
        return math.inf, math.inf
    if goal.integrity_depth_mm is None:
        free = fold_free(getattr(geo, "joint", geo), phi_rad)
        return float(free["proximal_pad"].min()), float(free["distal_pad"].min())
    kept = integrity_free(joint_scene(mechanism, geo, goal.packaging, np.degrees(phi_rad)), goal.integrity_depth_mm)
    return kept["proximal_pad"], kept["distal_pad"]


def hyperextension_violation(geo, config, margins_of) -> float:
    """How deep (mm) any checked pair overlaps while the joint hyperextends
    to ``goal.motion.hyperextension_deg`` (0: all clear), on the servo grid."""

    goal = config.goal
    if goal.hyperextension_rad <= 0.0:
        return 0.0
    least = hyperextension_margin(geo, goal.hyperextension_rad, goal.servo_step_rad,
                                  lambda geo, phi: margins_of(geo, phi, config))
    return max(-least, 0.0)


@dataclass(frozen=True)
class RevoluteEvaluation:
    """Constraints, objectives and diagnostics, named as the linkage's."""

    x: tuple[float, ...]
    constraints: Mapping[str, float]
    finger_length_mm: float
    proximal_length_mm: float
    required_rom_rad: float
    proximal_support_length_mm: float
    distal_support_length_mm: float
    proximal_face_mm: float
    distal_face_mm: float
    proximal_room_mm: float = math.nan
    distal_room_mm: float = math.nan
    """Each support's length, from the pad's outer end to ``A''`` / ``B''``."""
    proximal_depth_mm: float = math.nan
    distal_depth_mm: float = math.nan
    """How far each pad's contact face lies palmar of its segment's line
    through the joints (the centres of rotation): the finger's thickness there."""
    phi_safe_rad: float = 0.0
    phi_max_rad: float = math.nan
    stop_reason: str = ""
    collision_pair: str = ""
    least_tip_x_mm: float = math.nan
    pad_clearance_mm: float = math.nan
    torque_ratio_max: float = 1.0
    """Direct drive: one unit of servo torque per unit of distal moment."""
    bound_hits: tuple[str, ...] = ()
    proximal_usable_length_mm: float = math.inf
    distal_usable_length_mm: float = math.inf
    """Each face's length the other face leaves unhidden in the fold, the least on ``[0, phi_req]``."""

    @property
    def g(self) -> tuple[float, ...]:
        return tuple(self.constraints[name] for name in CONSTRAINT_NAMES)

    @property
    def feasible(self) -> bool:
        return max(self.g) <= 0.0

    @property
    def support_balance_mm(self) -> float:
        """How much longer than its module the shorter-spare support is:
        ``min(S_p - l_p, S_d - l_d)`` (the accommodation study's reference layouts)."""

        return min(self.proximal_room_mm - self.proximal_support_length_mm,
                   self.distal_room_mm - self.distal_support_length_mm)

    @property
    def proximal_effective_mm(self) -> float:
        return min(self.proximal_face_mm, self.proximal_usable_length_mm)

    @property
    def distal_effective_mm(self) -> float:
        return min(self.distal_face_mm, self.distal_usable_length_mm)

    @property
    def pad_coverage(self) -> float:
        return (self.proximal_effective_mm + self.distal_effective_mm) / self.finger_length_mm

    @property
    def mounted_coverage(self) -> float:
        return (self.proximal_support_length_mm + self.distal_support_length_mm) / self.finger_length_mm

    @property
    def curl_deg(self) -> float:
        return 180.0 - math.degrees(self.phi_max_rad)

    @property
    def phi_max_stop(self) -> str:
        return "tip_x" if self.phi_max_rad < self.phi_safe_rad - 1e-12 else self.stop_reason


def evaluate(x: Sequence[float], config: RevoluteConfig) -> RevoluteEvaluation:
    """Evaluate one design (static checks first; a design failing one is not swept)."""

    goal = config.goal
    packaging = goal.packaging
    clearance = packaging.body_clearance_mm
    geo = geometry(x, config)
    length_low, length_high = goal.extended_length_bounds_mm
    oa_low, oa_high = goal.proximal_length_bounds_mm
    oa = [value for value in (None if oa_low is None else oa_low - geo.a_x_mm,
                              None if oa_high is None else geo.a_x_mm - oa_high) if value is not None]
    # |OA| and BC (|AC|: no span) as shares of |OC|, as the linkage's.
    oa += goal.length_ratio_violations(geo.a_x_mm, geo.ac_length_mm, 0.0, geo.finger_length_mm)
    oa += goal.icr0_violations(geo.a_x_mm, geo.finger_length_mm)  # the pin A
    reach = pad_reach_mm(goal, config.joint_radius_mm)
    # In stages by cost (as the linkage's): lengths and the pads' room, then
    # the servos, then the pads' polygons; a failing stage leaves the rest unchecked.
    static = {
        "extended_length_mm": max(length_low - geo.finger_length_mm, geo.finger_length_mm - length_high),
        "proximal_length_mm": max(oa, default=0.0),
        # At least its min, reaching past the joint's housing by its overhang at
        # most (v6: no setback; the housing and the other pad are collisions).
        "proximal_support_mm": max(goal.minimum_proximal_support_mm - geo.proximal_length_mm,
                                   -geo.proximal_room_mm, geo.proximal_length_mm - geo.proximal_room_mm,
                                   x[VARIABLE_NAMES.index("app_x_mm")] - (geo.a_x_mm + reach[0])),
        "distal_support_mm": max(goal.minimum_distal_support_mm - geo.distal_length_mm, -geo.distal_room_mm,
                                 geo.distal_length_mm - geo.distal_room_mm,
                                 -reach[1] - x[VARIABLE_NAMES.index("bpp_from_a_mm")]),
        "dorsal_space_mm": dorsal_space_violation(geo, goal,
                                                  "revolute" if config.joint_servo else "revolute_idealized"),
    }
    stages = (("mcp_servo_mm", "pip_servo_pad_mm"), ("pad_collision_mm",))
    if max(static.values()) <= 0.0 and config.joint_servo:
        mcp_angles = np.asarray(goal.mcp_servo_angles_rad)
        static["mcp_servo_mm"] = -(packaging.mcp_servo_gap_mm(geo.distal_servo, mcp_angles) - clearance)
        static["pip_servo_pad_mm"] = (-(float(footprint_gaps(geo.proximal_pad[None], geo.distal_servo, packaging)[0])
                                        - clearance) if packaging.pip_servo_pad_check else 0.0)
    elif max(static.values()) <= 0.0:  # idealized: only a pin at A, the proximal pad off its housing
        static["mcp_servo_mm"] = 0.0
        a = np.array([[geo.a_x_mm, 0.0]])
        static["pip_servo_pad_mm"] = -(float(polygon_to_disk(geo.proximal_pad[None], a, config.joint_radius_mm)[0])
                                       - clearance)
    if max(static.values()) <= 0.0:
        margins_at_rest = _margins(geo, np.zeros(1), config)
        static["pad_collision_mm"] = -min(float(margin[0]) for margin in margins_at_rest.values())
    for name in stages[0] + stages[1]:
        static.setdefault(name, UNSWEPT)
    common = dict(x=tuple(float(value) for value in x), finger_length_mm=geo.finger_length_mm,
                  proximal_length_mm=geo.a_x_mm, required_rom_rad=goal.required_rom_rad,
                  proximal_support_length_mm=geo.proximal_length_mm,
                  distal_support_length_mm=geo.distal_length_mm,
                  proximal_face_mm=geo.proximal_face_mm, distal_face_mm=geo.distal_face_mm,
                  proximal_room_mm=geo.proximal_room_mm, distal_room_mm=geo.distal_room_mm,
                  proximal_depth_mm=geo.proximal_drop_mm + packaging.proximal_pad.height_mm,
                  distal_depth_mm=geo.distal_drop_mm + packaging.distal_pad.height_mm)
    if max(static.values()) > 0.0:
        constraints = {**static, **{name: UNSWEPT for name in MOTION_CONSTRAINTS}}
        return RevoluteEvaluation(constraints=MappingProxyType(constraints), **common)

    # The joint turns with the servo: the sweep is the servo grid, from extension.
    phi = np.asarray(goal.servo_samples_rad)
    margins = _between_samples(geo, phi, _margins(geo, phi, config, until_unsafe=True), config)
    least = np.min(np.stack(list(margins.values())), axis=0)
    unsafe = least < 0.0
    first_unsafe = int(np.flatnonzero(unsafe)[0]) if unsafe.any() else len(phi)
    reach = float(min(phi[first_unsafe - 1], math.pi)) if first_unsafe > 0 else 0.0
    stop_reason, collision_pair = ("curl" if reach >= math.pi - 1e-12 else "horizon"), ""
    if first_unsafe < len(phi) and reach < math.pi - 1e-12:
        stop_reason = "pad_collision"
        collision_pair = "/".join(next(pair for pair, margin in margins.items() if margin[first_unsafe] < 0.0))
    at = np.flatnonzero(phi >= goal.required_rom_rad - 1e-12)
    last = int(at[0]) if at.size else max(first_unsafe - 1, 0)
    window = np.zeros(len(phi), dtype=bool)
    window[:last + 1] = True
    tip_x = geo.tip(phi)[:, 0]
    failing = unsafe | (tip_x < 0.0)
    first_failing = int(np.flatnonzero(failing)[0]) if failing.any() else len(phi)
    phi_max = float(min(phi[first_failing - 1], math.pi)) if first_failing > 0 else 0.0
    reported = [margin for pair, margin in margins.items() if pair != ("mcp_servo", "proximal_pad")]
    proximal_usable, distal_usable = usable_lengths(
        geo, phi[window], goal, "revolute" if config.joint_servo else "revolute_idealized")
    constraints = {
        **static,
        "rom_deg": math.degrees(goal.required_rom_rad - reach),
        "hyperextension_mm": hyperextension_violation(geo, config, _margins),
        "curl_tip_x_mm": -float(tip_x[window].min()),
        # No link crosses a 2R pad: its effective length is its face less the fold (as the linkage's).
        "pad_usable_mm": max(goal.minimum_proximal_support_mm - min(geo.proximal_face_mm, proximal_usable),
                             goal.minimum_distal_support_mm - min(geo.distal_face_mm, distal_usable)),
    }
    return RevoluteEvaluation(
        proximal_usable_length_mm=proximal_usable,
        distal_usable_length_mm=distal_usable,
        constraints=MappingProxyType(constraints),
        phi_safe_rad=reach,
        phi_max_rad=phi_max,
        stop_reason=stop_reason,
        collision_pair=collision_pair,
        least_tip_x_mm=float(tip_x[window].min()),
        pad_clearance_mm=float(np.min(np.stack(reported), axis=0)[window].min()) if reported else math.inf,
        **common,
    )


def _margins(geo: RevoluteGeometry, phi: np.ndarray, config: RevoluteConfig,
             until_unsafe: bool = False) -> dict[tuple[str, str], np.ndarray]:
    """Per-sample margin of every checked pair (``solids.margins``, the study's checks)."""

    return solids.margins(geo, phi, joint_checks(config), until_unsafe)


def _between_samples(geo, phi: np.ndarray, margins: dict[tuple[str, str], np.ndarray], config,
                     margins_of=None) -> dict[tuple[str, str], np.ndarray]:
    """``margins`` resampled between samples where a collision could hide
    (``solids.between_samples``, on the study's verification grid);
    ``margins_of``: another mechanism's ``_margins``."""

    of = margins_of or _margins
    return between_samples(geo, phi, margins, config.goal.verify_step_rad, lambda geo, phi: of(geo, phi, config))


__all__ = ["CONSTRAINT_NAMES", "RevoluteEvaluation", "evaluate", "geometry", "hyperextension_violation",
           "joint_checks", "usable_lengths"]
