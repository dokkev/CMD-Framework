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
effective length is its face.
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
    footprint_gaps,
    mcp_servo_gaps,
    pad_outline,
    polygon_gaps,
    proximal_pad_mcp_gap,
)

from core.optimization.revolute.config import VARIABLE_NAMES, RevoluteConfig, servo_rear


STATIC_CONSTRAINTS = ("extended_length_mm", "proximal_length_mm", "mcp_servo_mm", "pip_servo_pad_mm",
                      "proximal_support_mm", "distal_support_mm", "pad_collision_mm")
MOTION_CONSTRAINTS = ("rom_deg", "curl_tip_x_mm", "pad_usable_mm")
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

        return float(np.hypot(self.distal_pad_local[:, 0], self.distal_pad_local[:, 1]).max())

    @property
    def distal_mount(self) -> np.ndarray:
        """The distal pad's mounting line (its top face), ``B''`` then ``C``'s
        end, in the distal frame: the truss's two pad-side nodes."""

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
    proximal_length = min(proximal_room, proximal.max_length_mm or math.inf)
    distal_length = min(distal_room, distal.max_length_mm or math.inf)
    chamfer = {side: pad.chamfer_mm for side, pad in (("proximal", proximal), ("distal", distal))}  # 0: blocks
    outer = v["ac_length_mm"] - distal.offset_mm
    proximal_outline = pad_outline(PadSpan(start, start + max(proximal_length, 1e-6), proximal_room),
                                   proximal, "end", chamfer["proximal"]) - (0.0, v["proximal_drop_mm"])
    distal_span = PadSpan(outer - max(distal_length, 1e-6), outer, distal_room)
    distal_outline = pad_outline(distal_span, distal, "start", chamfer["distal"]) - (0.0, v["distal_drop_mm"])
    servo = ServoFootprint((v["a_x_mm"], 0.0), servo_rear(v["servo_tilt_deg"]),
                           packaging.pip_servo.radius_mm, packaging.pip_servo.rear_extent_mm)
    face = lambda length, k: max(length - min(k, max(length, 0.0)), 0.0)  # noqa: E731
    return RevoluteGeometry(v["a_x_mm"], v["ac_length_mm"], proximal_outline, distal_outline, v["distal_drop_mm"],
                            v["proximal_drop_mm"], servo,
                            proximal_length, distal_length, face(proximal_length, chamfer["proximal"]),
                            face(distal_length, chamfer["distal"]), proximal_room, distal_room)


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

    @property
    def g(self) -> tuple[float, ...]:
        return tuple(self.constraints[name] for name in CONSTRAINT_NAMES)

    @property
    def feasible(self) -> bool:
        return max(self.g) <= 0.0

    @property
    def proximal_effective_mm(self) -> float:
        return self.proximal_face_mm

    @property
    def distal_effective_mm(self) -> float:
        return self.distal_face_mm

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
    # In stages by cost (as the linkage's): lengths and the pads' room, then
    # the servos, then the pads' polygons; a failing stage leaves the rest unchecked.
    static = {
        "extended_length_mm": max(length_low - geo.finger_length_mm, geo.finger_length_mm - length_high),
        "proximal_length_mm": max(oa, default=0.0),
        # At least its min, and on its own segment, off the joint's housing.
        "proximal_support_mm": max(goal.minimum_proximal_support_mm - geo.proximal_length_mm,
                                   -geo.proximal_room_mm,
                                   x[VARIABLE_NAMES.index("app_x_mm")] - (geo.a_x_mm - config.joint_radius_mm)),
        "distal_support_mm": max(goal.minimum_distal_support_mm - geo.distal_length_mm, -geo.distal_room_mm,
                                 config.joint_radius_mm - x[VARIABLE_NAMES.index("bpp_from_a_mm")]),
    }
    stages = (("mcp_servo_mm", "pip_servo_pad_mm"), ("pad_collision_mm",))
    if max(static.values()) <= 0.0 and config.joint_servo:
        mcp_angles = np.asarray(goal.mcp_servo_angles_rad)
        static["mcp_servo_mm"] = -(packaging.mcp_servo_gap_mm(geo.distal_servo, mcp_angles) - clearance)
        static["pip_servo_pad_mm"] = (-(float(footprint_gaps(geo.proximal_pad[None], geo.distal_servo, packaging)[0])
                                        - clearance) if packaging.pip_servo_pad_check else 0.0)
    elif max(static.values()) <= 0.0:  # idealized: only a pin at A, the pads off its housing
        static["mcp_servo_mm"] = 0.0
        static["pip_servo_pad_mm"] = 0.0
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
    constraints = {
        **static,
        "rom_deg": math.degrees(goal.required_rom_rad - reach),
        "curl_tip_x_mm": -float(tip_x[window].min()),
        # No link crosses a 2R pad: its effective length is its face (as the linkage's).
        "pad_usable_mm": max(goal.minimum_proximal_support_mm - geo.proximal_face_mm,
                             goal.minimum_distal_support_mm - geo.distal_face_mm),
    }
    return RevoluteEvaluation(
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
    for name, solid in moving.items():
        if config.joint_servo:
            margins[(name, "pip_servo")] = footprint_gaps(solid, geo.distal_servo, packaging) - clearance
        else:
            a = np.broadcast_to((geo.a_x_mm, 0.0), (count, 2))
            margins[(name, "a_pin")] = _polygon_to_disk(solid, a, config.joint_radius_mm) - clearance
        if packaging.pad_pad_check:
            margins[(name, "proximal_pad")] = polygon_gaps(solid, proximal, packaging) - clearance
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


__all__ = ["CONSTRAINT_NAMES", "RevoluteEvaluation", "RevoluteGeometry", "evaluate", "geometry"]
