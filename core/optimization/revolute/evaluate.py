"""Evaluate one serial 2R (revolute) finger against the linkage study's goal.

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

from core.morphology.linkage.packaging import (
    PadSpan,
    ServoFootprint,
    _polygon_to_disk,
    carrier_outline,
    fold_free_lengths,
    footprint_gaps,
    mcp_servo_gaps,
    pad_outline,
    polygon_gaps,
    proximal_pad_mcp_gap,
    segment_polygon_gaps,
)

from core.optimization.revolute.config import VARIABLE_NAMES, RevoluteConfig, pad_reach_mm, servo_rear


STATIC_CONSTRAINTS = ("extended_length_mm", "proximal_length_mm", "mcp_servo_mm", "pip_servo_pad_mm",
                      "proximal_support_mm", "distal_support_mm", "dorsal_space_mm", "pad_collision_mm")
MOTION_CONSTRAINTS = ("rom_deg", "hyperextension_mm", "curl_tip_x_mm", "pad_usable_mm")
CONSTRAINT_NAMES = STATIC_CONSTRAINTS + MOTION_CONSTRAINTS
UNSWEPT = 1.0e2


@dataclass(frozen=True)
class RevoluteGeometry:
    """A design's solids in the reference pose: the proximal pad and servos in
    the proximal frame, the distal pad in the distal frame (``u`` from ``A``
    along ``A->C``, ``v`` dorsal)."""

    a_x_mm: float
    ac_length_mm: float
    proximal_pad: np.ndarray
    distal_pad_local: np.ndarray
    distal_drop_mm: float
    proximal_drop_mm: float
    distal_servo: ServoFootprint
    proximal_length_mm: float
    distal_length_mm: float
    proximal_face_mm: float
    distal_face_mm: float
    proximal_room_mm: float
    distal_room_mm: float
    distal_carrier_local: np.ndarray | None = None
    """The distal pad's carrier plate in the distal frame (``None``: none)."""
    distal_mount_local: np.ndarray | None = None
    """The distal mounting line, ``B''`` then the fingertip end, in the distal
    frame (``None``: the pad's top face): the support, there whatever module it carries."""

    @property
    def finger_length_mm(self) -> float:
        return self.a_x_mm + self.ac_length_mm

    def distal_pad(self, phi_rad: np.ndarray) -> np.ndarray:
        """The distal pad at distal flexions ``phi`` (``(n,)``), ``(n, m, 2)`` in the proximal frame."""

        return self.posed(self.distal_pad_local, phi_rad)

    def posed(self, local: np.ndarray, phi_rad: np.ndarray) -> np.ndarray:
        """Points ``(m, 2)`` of the distal frame at ``phi``, ``(n, m, 2)`` in the proximal frame."""

        phi = np.atleast_1d(np.asarray(phi_rad, dtype=float))
        e = np.stack((np.cos(phi), -np.sin(phi)), axis=1)
        n = np.stack((-e[:, 1], e[:, 0]), axis=1)
        a = np.array((self.a_x_mm, 0.0))
        local = np.asarray(local, dtype=float)
        return a + local[None, :, :1] * e[:, None] + local[None, :, 1:] * n[:, None]

    @property
    def distal_reach_mm(self) -> float:
        """The distal pad's farthest point from ``A``: how fast it moves per
        radian of ``phi``, at most."""

        solids = self.distal_pad_local if self.distal_carrier_local is None else np.vstack(
            (self.distal_pad_local, self.distal_carrier_local))
        return float(np.hypot(solids[:, 0], solids[:, 1]).max())

    @property
    def distal_mount(self) -> np.ndarray:
        """The distal pad's mounting line (its top face), ``B''`` then ``C``'s
        end, in the distal frame: the truss's two pad-side nodes."""

        if self.distal_mount_local is not None:
            return self.distal_mount_local
        top = self.distal_pad_local[:, 1].max()
        return np.array(((self.distal_pad_local[:, 0].min(), top), (self.distal_pad_local[:, 0].max(), top)))

    def tip(self, phi_rad: np.ndarray) -> np.ndarray:
        """``C`` at distal flexions ``phi``, ``(n, 2)``."""

        phi = np.atleast_1d(np.asarray(phi_rad, dtype=float))
        return np.stack((self.a_x_mm + self.ac_length_mm * np.cos(phi), -self.ac_length_mm * np.sin(phi)), axis=1)


def geometry(x: Sequence[float], config: RevoluteConfig) -> RevoluteGeometry:
    """The solids of a design vector."""

    v = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    packaging = config.goal.packaging
    proximal, distal = packaging.proximal_pad, packaging.distal_pad
    start = packaging.proximal_pad_start_mm
    proximal_room = v["app_x_mm"] - start
    distal_room = v["ac_length_mm"] - distal.offset_mm - v["bpp_from_a_mm"]
    # A module of a set length (a fixed finger's module sweep), else the room's, at most its max.
    proximal_length = proximal.length_mm or min(proximal_room, proximal.max_length_mm or math.inf)
    distal_length = distal.length_mm or min(distal_room, distal.max_length_mm or math.inf)
    chamfer = {side: pad.chamfer_mm for side, pad in (("proximal", proximal), ("distal", distal))}  # 0: blocks
    outer = v["ac_length_mm"] - distal.offset_mm
    proximal_outline = pad_outline(PadSpan(start, start + max(proximal_length, 1e-6), proximal_room),
                                   proximal, "end", chamfer["proximal"]) - (0.0, v["proximal_drop_mm"])
    distal_span = PadSpan(outer - max(distal_length, 1e-6), outer, distal_room)
    distal_outline = pad_outline(distal_span, distal, "start", chamfer["distal"]) - (0.0, v["distal_drop_mm"])
    servo = ServoFootprint((v["a_x_mm"], 0.0), servo_rear(v["servo_tilt_deg"]),
                           packaging.pip_servo.radius_mm, packaging.pip_servo.rear_extent_mm)
    face = lambda length, k: max(length - min(k, max(length, 0.0)), 0.0)  # noqa: E731
    # The support: the mounting line from B'' to the fingertip end, whatever the module.
    support = pad_outline(PadSpan(outer - max(distal_room, 1e-6), outer, distal_room), distal, "start", 0.0) \
        - (0.0, v["distal_drop_mm"])
    mount = np.array(((outer - distal_room, -v["distal_drop_mm"]), (outer, -v["distal_drop_mm"])))
    return RevoluteGeometry(v["a_x_mm"], v["ac_length_mm"], proximal_outline, distal_outline, v["distal_drop_mm"],
                            v["proximal_drop_mm"], servo,
                            proximal_length, distal_length, face(proximal_length, chamfer["proximal"]),
                            face(distal_length, chamfer["distal"]), proximal_room, distal_room,
                            carrier_local(support, v["distal_drop_mm"], packaging), mount)


def carrier_local(distal_pad_local: np.ndarray, drop_mm: float, packaging) -> np.ndarray | None:
    """The distal pad's carrier plate (``packaging.distal_carrier_height_mm``
    dorsal of the pad's top face, over its span) in the distal frame, as the
    linkage's (``packaging.distal_carrier_polygon``); ``None`` without one."""

    if packaging.distal_carrier_height_mm <= 0.0:
        return None
    return carrier_outline(distal_pad_local + (0.0, drop_mm), packaging.distal_carrier_height_mm) - (0.0, drop_mm)


def contact_faces(geo, phi_rad: np.ndarray):
    """The two contact faces as ``packaging.fold_free_lengths`` takes them:
    each pad's palmar face, outer end first, at distal flexions ``phi``
    (any geometry with ``proximal_pad``, ``distal_pad_local`` and ``posed``)."""

    proximal = geo.proximal_pad
    z = float(proximal[:, 1].min())
    proximal_face = np.array(((proximal[:, 0].min(), z), (proximal[:, 0].max(), z)))
    local = geo.distal_pad_local
    v = float(local[:, 1].min())
    distal_face = geo.posed(np.array(((local[:, 0].max(), v), (local[:, 0].min(), v))), phi_rad)
    origin, palmar = np.moveaxis(geo.posed(np.array(((0.0, 0.0), (0.0, -1.0))), phi_rad), 1, 0)
    return proximal_face, np.array((0.0, -1.0)), distal_face, palmar - origin


def fold_free(geo, phi_rad: np.ndarray) -> dict[str, np.ndarray]:
    """Each face's length the other leaves unhidden at ``phi`` (``(n,)`` each)."""

    return fold_free_lengths(*contact_faces(geo, np.atleast_1d(phi_rad)))


def dorsal_space_violation(geo, goal, mechanism: str) -> float:
    """How far (mm) the extended finger rises past ``goal.link.dorsal_space_mm`` above ``O``'s line (0: none set)."""

    if goal.dorsal_space_mm is None:
        return 0.0
    from core.optimization.analysis.loss import dorsal_space_mm, joint_scene

    return dorsal_space_mm(joint_scene(mechanism, geo, goal.packaging, [0.0])) - goal.dorsal_space_mm


def usable_lengths(geo, phi_rad: np.ndarray, goal, mechanism: str) -> tuple[float, float]:
    """Each face's length kept through the flexions ``phi`` (the window to
    the required flexion): less the fold at each, or with
    ``goal.integrity_depth_mm`` the integrity rule
    (``core.optimization.analysis.loss.integrity_free``) over the
    ``mechanism``'s solids."""

    if goal.integrity_depth_mm is None and not goal.count_fold:  # a module clear through the motion counts whole
        return math.inf, math.inf
    if goal.integrity_depth_mm is None:
        free = fold_free(getattr(geo, "joint", geo), phi_rad)
        return float(free["proximal_pad"].min()), float(free["distal_pad"].min())
    from core.optimization.analysis.loss import integrity_free, joint_scene

    kept = integrity_free(joint_scene(mechanism, geo, goal.packaging, np.degrees(phi_rad)), goal.integrity_depth_mm)
    return kept["proximal_pad"], kept["distal_pad"]


def hyperextension_violation(geo, config, margins_of) -> float:
    """How deep (mm) any checked pair overlaps while the joint hyperextends
    to ``goal.motion.hyperextension_deg`` (0: all clear), on the servo grid."""

    goal = config.goal
    if goal.hyperextension_rad <= 0.0:
        return 0.0
    step = goal.servo_step_rad
    back = -np.unique(np.append(np.arange(step, goal.hyperextension_rad, step), goal.hyperextension_rad))
    least = min(float(margin.min()) for margin in margins_of(geo, back, config).values())
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
        static["pip_servo_pad_mm"] = -(float(_polygon_to_disk(geo.proximal_pad[None], a, config.joint_radius_mm)[0])
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
    """Per-sample margin of every checked pair beyond the clearance.

    ``until_unsafe``: the distal pad is checked against the MCP servo (the
    costly pair: every MCP angle) only up to the first sample another pair
    fails at; past it the motion is unsafe anyway, and its margin is ``inf``.
    """

    goal = config.goal
    packaging = goal.packaging
    clearance = packaging.body_clearance_mm
    count = len(phi)
    distal = geo.distal_pad(phi)
    proximal = np.broadcast_to(geo.proximal_pad, (count, *geo.proximal_pad.shape))
    moving = {"distal_pad": distal}
    margins = {}
    if config.joint_servo and geo.distal_carrier_local is not None:
        margins[("distal_carrier", "pip_servo")] = (
            footprint_gaps(geo.posed(geo.distal_carrier_local, phi), geo.distal_servo, packaging) - clearance)
    for name, solid in moving.items():
        if config.joint_servo:
            margins[(name, "pip_servo")] = footprint_gaps(solid, geo.distal_servo, packaging) - clearance
        else:
            a = np.broadcast_to((geo.a_x_mm, 0.0), (count, 2))
            margins[(name, "a_pin")] = _polygon_to_disk(solid, a, config.joint_radius_mm) - clearance
        if packaging.pad_pad_check:
            margins[(name, "proximal_pad")] = polygon_gaps(solid, proximal, packaging) - clearance
    if goal.link_radius_mm is not None:  # the comparison model: the bars as capsules, in the side view
        margins.update(bar_capsule_margins(geo, phi, goal.link_radius_mm, clearance))
    if packaging.pad_checks_mcp("distal_pad"):
        end = count
        if until_unsafe:
            failing = np.flatnonzero(np.min(np.stack(list(margins.values())), axis=0) < -1e-9)
            end = int(failing[0]) + 1 if failing.size else count
        for name, solid in moving.items():
            margin = np.full(count, np.inf)
            margin[:end] = (mcp_servo_gaps(solid[:end], packaging, goal.mcp_samples_rad)
                            - packaging.mcp_pad_clearance_mm)
            margins[(name, "mcp_servo")] = margin
    if packaging.pad_checks_mcp("proximal_pad"):  # fixed in the proximal frame
        gap = proximal_pad_mcp_gap(geo.proximal_pad, packaging, goal.mcp_samples_rad)
        margins[("mcp_servo", "proximal_pad")] = np.full(count, gap - packaging.mcp_pad_clearance_mm)
    return {pair: np.where(np.abs(margin) < 1e-9, 0.0, margin) for pair, margin in margins.items()}


def bar_capsule_margins(geo, phi: np.ndarray, radius_mm: float, clearance: float) -> dict[tuple[str, str], np.ndarray]:
    """Per-sample margin (mm) of each pad against the bars it is not mounted
    on, capsules of ``radius_mm``, in the side view: the proximal pad against
    the distal truss (``A`` to the distal pad's mounting ends, and between
    them), the distal pad against the proximal bar ``O-A``."""

    count = len(phi)
    a = np.broadcast_to((geo.a_x_mm, 0.0), (count, 2))
    near, far = np.moveaxis(geo.posed(geo.distal_mount, phi), 1, 0)
    proximal = np.broadcast_to(geo.proximal_pad, (count, *geo.proximal_pad.shape))
    margins = {("proximal_pad", f"link_{name}"): segment_polygon_gaps(proximal, start, end) - radius_mm - clearance
               for name, (start, end) in (("a_near", (a, near)), ("a_far", (a, far)), ("near_far", (near, far)))}
    margins[("distal_pad", "link_o_a")] = (segment_polygon_gaps(geo.distal_pad(phi), np.zeros((count, 2)), a)
                                           - radius_mm - clearance)
    return margins


def _between_samples(geo: RevoluteGeometry, phi: np.ndarray, margins: dict[tuple[str, str], np.ndarray],
                     config: RevoluteConfig, margins_of=None) -> dict[tuple[str, str], np.ndarray]:
    """``margins`` with the motion between samples checked on the verification
    grid where a collision could hide there.

    The distal solids turn rigidly about ``A``, so no point of them moves more
    than ``distal_reach_mm`` per radian: a margin falls at most that fast, and
    two neighbouring clear samples whose margins sum to less than the reach
    times their step may pass through a collision between them. Those steps
    are resampled on ``verify_step_rad``, up to the first unsafe sample; the
    first that collides inside lowers the margins at its end to the least
    inside. (On the verification grid itself nothing is resampled.)
    ``margins_of`` (``_margins``'s signature) resamples another mechanism whose
    ``geo`` bounds its speed by ``distal_reach_mm`` too.
    """

    fine = config.goal.verify_step_rad
    least = np.min(np.stack(list(margins.values())), axis=0)
    unsafe = np.flatnonzero(least < 0.0)
    end = int(unsafe[0]) if unsafe.size else len(phi) - 1
    step = np.diff(phi[:end + 1])
    clear = least[:end]
    suspect = np.flatnonzero((clear + least[1:end + 1] < geo.distal_reach_mm * step)
                             & (step > fine * (1.0 + 1e-9)))
    if not suspect.size:
        return margins
    counts = np.ceil(step[suspect] / fine - 1e-9).astype(int)
    inner = np.concatenate([np.linspace(phi[i], phi[i + 1], count + 1)[1:-1] for i, count in zip(suspect, counts)])
    owner = np.repeat(suspect, counts - 1)
    fine_margins = (margins_of or _margins)(geo, inner, config)
    fine_least = np.min(np.stack(list(fine_margins.values())), axis=0)
    hit = np.flatnonzero(fine_least < 0.0)
    if not hit.size:
        return margins
    interval = owner[hit[0]]
    inside = owner == interval
    result = {pair: margin.copy() for pair, margin in margins.items()}
    for pair, margin in fine_margins.items():
        result[pair][interval + 1] = min(result[pair][interval + 1], float(margin[inside].min()))
    return result


__all__ = ["CONSTRAINT_NAMES", "RevoluteEvaluation", "RevoluteGeometry", "carrier_local", "contact_faces",
           "evaluate", "fold_free", "geometry", "hyperextension_violation", "usable_lengths"]
