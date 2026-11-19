"""Evaluate one revolute-family finger (2R-DD here; 2R-remote and RCJ reuse
:func:`measure_motion`): the prototype formulation's constraints and
objective, as the CFL's (``design/SSoT/problem_formulation.md`` A.4-A.5).

The MCP is at ``O`` and the joint at ``A = (|OA|, 0)``; the distal segment
``A-C`` turns by ``q_2`` about ``A``. The 2R-DD's servo sits on ``A`` and
drives the joint directly (``MA = 1``). Its solids and their gaps are the
morphology's (``core.morphology.mechanism.revolute``).

- **Static checks first**: the morphology bounds (``bounds_*``) and
  ``F_static`` (``static_*``) in the reference pose.
- **The motion**: ``q_2`` on the uniform grid; a pose is valid when every
  checked pair clears (resampled between samples where a collision could
  hide) and the fingertip stays ahead of the MCP. ``F_motion`` asks it on
  ``[0, q_req]``.
- **The objective**: the mean cylinder-based coverage over ``Q_eval``.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from core.evaluation import (
    MotionSamples,
    PoseCoverage,
    common_length_mm,
    evaluation_poses_rad,
    mean_coverage,
    posed_finger_length_mm,
    worst_mechanical_advantage,
)
from core.geometry.intervals import IntervalSet
from core.morphology.contact.loss import dorsal_space_mm, joint_scene
from core.morphology.packaging.gaps import footprint_gaps
from core.morphology.packaging.hardware import MARGIN_TOLERANCE_MM
from core.morphology.packaging.shapes import polygon_to_disk
from core.morphology.mechanism.revolute import solids
from core.morphology.mechanism.revolute.solids import JointChecks, RevoluteGeometry, between_samples, contact_faces

from core.optimization.revolute.config import VARIABLE_NAMES, RevoluteConfig, pad_reach_mm


STATIC_CONSTRAINTS = ("bounds_finger_length_mm", "bounds_proportions_mm", "bounds_proximal_pad_mm",
                      "bounds_distal_pad_mm", "bounds_mount_depth_mm", "bounds_dorsal_space_mm",
                      "static_mcp_servo_mm", "static_pip_servo_pad_mm", "static_pad_collision_mm")
"""The morphology bounds (``bounds_*``) and ``F_static`` (``static_*``), as the CFL's."""
MOTION_CONSTRAINTS = ("f_motion_deg", "gamma_range_deg", "ma_min")
"""As the CFL's; the transmission angle and ``MA_min`` hold for the CFL only (0 here)."""
CONSTRAINT_NAMES = STATIC_CONSTRAINTS + MOTION_CONSTRAINTS
UNSWEPT = 1.0e2
TOUCH_MM = MARGIN_TOLERANCE_MM
"""Gaps above ``-TOUCH_MM`` count as touching (``code_formulation.md`` §2)."""


def segment_lengths(x: Sequence[float]) -> tuple[float, float]:
    """The 2R's segments, as the linkage's: ``|OA''|`` (``O`` to where the
    proximal body ends) and ``|B''C|`` (where the distal body starts to ``C``)."""

    v = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    return v["app_x_mm"], v["ac_length_mm"] - v["bpp_from_a_mm"]


def geometry(x: Sequence[float], config: RevoluteConfig) -> RevoluteGeometry:
    """The solids of a design vector, with the study's packaging, each pad
    mounted its drop (its searched offset) below its segment's line."""

    return solids.geometry(x, config.goal.packaging)


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


@dataclass(frozen=True)
class RevoluteEvaluation:
    """Constraints, objective and diagnostics of a revolute-family design
    (2R-DD, 2R-remote, RCJ), with the CFL's fields (``mechanism.evaluation_report``)."""

    x: tuple[float, ...]
    constraints: Mapping[str, float]
    finger_length_mm: float
    proximal_length_mm: float
    required_rom_rad: float
    proximal_support_length_mm: float
    distal_support_length_mm: float
    proximal_room_mm: float = math.nan
    distal_room_mm: float = math.nan
    required_proximal_mm: float = math.nan
    required_distal_mm: float = math.nan
    proximal_depth_mm: float = math.nan
    distal_depth_mm: float = math.nan
    """How far each pad's contact face lies palmar of its segment's line through the joints."""
    q_max_rad: float = 0.0
    valid_reach_rad: float = 0.0
    stop_reason: str = ""
    collision_pair: str = ""
    ma_wc: float = math.nan
    gamma_min_rad: float = math.nan
    gamma_max_rad: float = math.nan
    least_tip_x_mm: float = math.nan
    pad_clearance_mm: float = math.nan
    poses: tuple[PoseCoverage, ...] = ()
    bound_hits: tuple[str, ...] = ()
    proximal_ratio_shortfall_mm: float = 0.0
    distal_ratio_shortfall_mm: float = 0.0

    constraint_names = CONSTRAINT_NAMES

    @property
    def g(self) -> tuple[float, ...]:
        return tuple(self.constraints[name] for name in self.constraint_names)

    @property
    def feasible(self) -> bool:
        return max(self.g) <= 0.0

    @property
    def c_pad_mean(self) -> float:
        return mean_coverage(self.poses)

    @property
    def q_max_deg(self) -> float:
        return math.degrees(self.q_max_rad)


def joint_coverage(joint, poses_rad, l1_mm: float, l2_mm: float, distal_start_u_mm: float,
                   hidden=None) -> tuple[PoseCoverage, ...]:
    """Cylinder-based coverage of a revolute-family joint at each pose.

    ``joint`` has ``proximal_pad``, ``distal_pad_local`` and ``posed``
    (``solids.contact_faces``); ``distal_start_u_mm`` is ``B''`` along the
    distal frame's axis. ``hidden(q, proximal_face, proximal_normal,
    distal_face, distal_normal)`` gives what an occluder hides on each face
    (``None``: nothing).
    """

    found = []
    for q in poses_rad:
        proximal_face, proximal_normal, distal_faces, distal_normals = contact_faces(joint, np.array([q]))
        cut = ((IntervalSet(), IntervalSet()) if hidden is None
               else hidden(q, proximal_face, proximal_normal, distal_faces[0], distal_normals[0]))
        distal_start = joint.posed(np.array(((distal_start_u_mm, 0.0),)), np.array([q]))[0, 0]
        found.append(PoseCoverage(float(q), common_length_mm(proximal_face, distal_faces[0], cut),
                                  posed_finger_length_mm(l1_mm, l2_mm, (l1_mm, 0.0), distal_start)))
    return tuple(found)


def measure_motion(geo, joint, config, margins_of, *, ma: float, l1_mm: float, l2_mm: float,
                   distal_start_u_mm: float, hidden=None) -> tuple[dict, dict]:
    """The motion and coverage of a revolute-family design that passed its static checks.

    The flexion ``q_2`` on the uniform grid (``goal.flexion_samples_rad``),
    each pose valid when every checked pair clears (``margins_of``, resampled
    between samples where a collision could hide) and the fingertip stays
    ahead of the MCP. ``ma`` is the joint's constant mechanical advantage
    (``theta = ma * q_2``). Returns the motion constraints and the
    evaluation's fields.
    """

    goal = config.goal
    q_req = goal.required_rom_rad
    phi = np.asarray(goal.flexion_samples_rad)
    margins = _between_samples(geo, phi, margins_of(geo, phi, config, until_unsafe=True, include_static_pairs=False),
                               config, margins_of)
    least = np.min(np.stack(list(margins.values())), axis=0)
    tip_x = joint.tip(phi)[:, 0]
    samples = MotionSamples(phi, (least >= -TOUCH_MM) & (tip_x >= -TOUCH_MM), ma * phi)
    bad = np.flatnonzero(~samples.valid)
    stop_reason = "curl" if not bad.size else ("collision" if least[int(bad[0])] < -TOUCH_MM else "tip")
    overlapping = [(int(np.flatnonzero(gap < -TOUCH_MM)[0]), "/".join(pair))
                   for pair, gap in margins.items() if np.any(gap < -TOUCH_MM)]
    collision_pair = min(overlapping)[1] if overlapping else ""
    window = samples.window(q_req)
    reported = [gap for pair, gap in margins.items() if pair != ("mcp_servo", "proximal_pad")]
    constraints = {"f_motion_deg": samples.f_motion_violation_deg(q_req), "gamma_range_deg": 0.0, "ma_min": 0.0}
    fields = dict(
        q_max_rad=samples.q_max_rad(),
        valid_reach_rad=samples.valid_reach_rad(),
        stop_reason=stop_reason,
        collision_pair=collision_pair,
        ma_wc=worst_mechanical_advantage(samples, q_req),
        least_tip_x_mm=float(tip_x[window].min()),
        pad_clearance_mm=float(np.min(np.stack(reported), axis=0)[window].min()) if reported else math.inf,
        poses=joint_coverage(joint, evaluation_poses_rad(q_req), l1_mm, l2_mm, distal_start_u_mm, hidden),
    )
    return constraints, fields


def mount_depth_violation(x: Sequence[float], config: RevoluteConfig) -> float:
    """How far the drops are from the mounting policy's (mm; 0 when they are searched)."""

    if config.goal.mount_policy == "searched":
        return 0.0
    from core.optimization.mounting import depth_violation_mm, revolute_mounting

    v = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    return depth_violation_mm(revolute_mounting(x, config), (v["proximal_drop_mm"], v["distal_drop_mm"]), config.goal)

def evaluate(x: Sequence[float], config: RevoluteConfig) -> RevoluteEvaluation:
    """Evaluate one design (static checks first; a design failing one is not swept)."""

    goal = config.goal
    packaging = goal.packaging
    clearance = 0.0
    geo = geometry(x, config)
    length_low, length_high = goal.extended_length_bounds_mm
    oa_low, oa_high = goal.proximal_length_bounds_mm
    oa = [value for value in (None if oa_low is None else oa_low - geo.a_x_mm,
                              None if oa_high is None else geo.a_x_mm - oa_high) if value is not None]
    # The segments |OA''| and |B''C| (no span) as shares of |OC|, as the linkage's.
    oa += goal.length_ratio_violations(*segment_lengths(x), 0.0, geo.finger_length_mm)
    oa += goal.icr0_violations(geo.a_x_mm, geo.finger_length_mm)  # the pin A
    reach = pad_reach_mm(goal, config.joint_radius_mm)
    # In stages by cost (as the linkage's): lengths and the pads' room, then
    # the servos, then the pads' polygons; a failing stage leaves the rest unchecked.
    static = {
        "bounds_finger_length_mm": max(length_low - geo.finger_length_mm, geo.finger_length_mm - length_high),
        "bounds_proportions_mm": max(oa, default=0.0),
        # At least its min, reaching past the joint's housing by its overhang at
        # most (v6: no setback; the housing and the other pad are collisions).
        "bounds_proximal_pad_mm": max(goal.minimum_proximal_support_mm - geo.proximal_length_mm,
                                   -geo.proximal_room_mm, geo.proximal_length_mm - geo.proximal_room_mm,
                                   x[VARIABLE_NAMES.index("app_x_mm")] - (geo.a_x_mm + reach[0])),
        "bounds_distal_pad_mm": max(goal.minimum_distal_support_mm - geo.distal_length_mm, -geo.distal_room_mm,
                                 geo.distal_length_mm - geo.distal_room_mm,
                                 -reach[1] - x[VARIABLE_NAMES.index("bpp_from_a_mm")]),
        "bounds_dorsal_space_mm": dorsal_space_violation(geo, goal,
                                                  "revolute" if config.joint_servo else "revolute_idealized"),
        "bounds_mount_depth_mm": mount_depth_violation(x, config),
    }
    stages = (("static_mcp_servo_mm", "static_pip_servo_pad_mm"), ("static_pad_collision_mm",))
    if max(static.values()) <= 0.0 and config.joint_servo:
        mcp_angles = np.asarray(goal.mcp_servo_angles_rad)
        static["static_mcp_servo_mm"] = -(packaging.mcp_servo_gap_mm(geo.distal_servo, mcp_angles) - clearance)
        static["static_pip_servo_pad_mm"] = (-(float(footprint_gaps(geo.proximal_pad[None], geo.distal_servo, packaging)[0])
                                        - clearance) if packaging.pip_servo_pad_check else 0.0)
    elif max(static.values()) <= 0.0:  # idealized: only a pin at A, the proximal pad off its housing
        static["static_mcp_servo_mm"] = 0.0
        a = np.array([[geo.a_x_mm, 0.0]])
        static["static_pip_servo_pad_mm"] = -(float(polygon_to_disk(geo.proximal_pad[None], a, config.joint_radius_mm)[0])
                                       - clearance)
    if max(static.values()) <= 0.0:
        margins_at_rest = _margins(geo, np.zeros(1), config)
        static["static_pad_collision_mm"] = -min(float(margin[0]) for margin in margins_at_rest.values())
    for name in stages[0] + stages[1]:
        static.setdefault(name, UNSWEPT)
    shortfalls = goal.length_ratio_shortfalls(*segment_lengths(x), geo.finger_length_mm)
    common = dict(x=tuple(float(value) for value in x), finger_length_mm=geo.finger_length_mm,
                  proximal_ratio_shortfall_mm=shortfalls[0], distal_ratio_shortfall_mm=shortfalls[1],
                  proximal_length_mm=geo.a_x_mm, required_rom_rad=goal.required_rom_rad,
                  proximal_support_length_mm=geo.proximal_length_mm,
                  distal_support_length_mm=geo.distal_length_mm,
                  proximal_room_mm=geo.proximal_room_mm, distal_room_mm=geo.distal_room_mm,
                  proximal_depth_mm=geo.proximal_drop_mm + packaging.proximal_pad.height_mm,
                  distal_depth_mm=geo.distal_drop_mm + packaging.distal_pad.height_mm,
                  required_proximal_mm=goal.minimum_proximal_support_mm,
                  required_distal_mm=goal.minimum_distal_support_mm)
    if max(static.values()) > 0.0:
        constraints = {**static, **{name: UNSWEPT for name in MOTION_CONSTRAINTS}}
        return RevoluteEvaluation(constraints=MappingProxyType(constraints), **common)

    v = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    motion, fields = measure_motion(geo, geo, config, _margins, ma=1.0, l1_mm=v["app_x_mm"],
                                    l2_mm=v["ac_length_mm"] - v["bpp_from_a_mm"], distal_start_u_mm=v["bpp_from_a_mm"])
    return RevoluteEvaluation(constraints=MappingProxyType({**static, **motion}), **fields, **common)


def _margins(geo: RevoluteGeometry, phi: np.ndarray, config: RevoluteConfig,
             until_unsafe: bool = False, *, include_static_pairs: bool = True) -> dict[tuple[str, str], np.ndarray]:
    """Per-sample margin of every checked pair (``solids.margins``, the study's checks)."""

    return solids.margins(geo, phi, joint_checks(config), until_unsafe, include_static_pairs=include_static_pairs)


def _between_samples(geo, phi: np.ndarray, margins: dict[tuple[str, str], np.ndarray], config,
                     margins_of=None) -> dict[tuple[str, str], np.ndarray]:
    """``margins`` resampled between samples where a collision could hide
    (``solids.between_samples``, on the study's verification grid);
    ``margins_of``: another mechanism's ``_margins``."""

    of = margins_of or _margins
    return between_samples(
        geo,
        phi,
        margins,
        config.goal.verify_step_rad,
        lambda geo, phi: of(geo, phi, config, include_static_pairs=False),
    )


__all__ = ["CONSTRAINT_NAMES", "MOTION_CONSTRAINTS", "STATIC_CONSTRAINTS", "TOUCH_MM", "UNSWEPT",
           "RevoluteEvaluation", "evaluate", "geometry", "joint_checks", "joint_coverage", "measure_motion",
           "segment_lengths"]
