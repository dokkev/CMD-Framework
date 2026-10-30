"""Contact accommodation of a fixed finger layout: whether a proximal and a
distal contact module both fit and stay clear from extension to a required
flexion, the mechanism never changed.

A geometry-only check, apart from the optimization's design requirements
(no torque, transmission-angle, dorsal-space or fingertip limits): only
mounting on the fixed supports, the motion's closure and solid penetration.
For a layout (a complete, fixed design vector ``x`` and its comparison
config, ``tradeoff.load``) and modules (``tradeoff.Modules``):

1. the modules on the fixed supports (``tradeoff.with_modules``: the design
   vector, the pivots, the servo, ``A''``, ``B''`` and both supports' plates
   unchanged); a module longer than its support's room is ``support_exceeded``;
2. the motion: the 2R turning about ``A``; CoDe solved by its four-bar
   (``FingerLinkage.sweep_servo``), ``phi = q_pip + q_dip`` the distal
   carrier's rotation relative to the proximal one (not the servo's turn),
   sampled to the first pose at or past ``phi_req`` (``kinematic_unreachable``
   if the four-bar stops closing or turns back short of it);
3. every checked pair's signed margin over those poses (``overlap_depth``:
   an overlap negative by its depth, touching 0, clearance 0), feasible when
   none falls below ``-TOLERANCE_MM``: the evaluators' own pairs
   (``packaging.pad_margins`` / ``revolute.evaluate._margins``), the bars as
   capsules (``link_radius_mm``), the supports (``proximal_carrier_height_mm``),
   and each pad against its own segment's pin housings (CoDe: A and A'
   proximal, B and B' distal; the 2R: the servo at A, the proximal pad).

The fixed structure itself is checked once (:func:`layout_violations`: the
PIP servo against the MCP servo, CoDe's pin A against the servo on A', two
pins on one body apart): an unbuildable layout is ``invalid_fixed_layout``,
whatever the modules. Between motion samples a collision is not assumed
away: where two neighbouring samples' least margin leaves room for the
moving solids to touch in between (their travel bound), the interval is
resampled until each piece moves at most ``REFINE_MM`` (the 2R:
``revolute.evaluate._between_samples`` on the 0.05 deg verification grid,
about as fine). The first failure's pair names the reason (``REASONS``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from core.morphology.linkage import FingerLinkage
from core.morphology.linkage.packaging import distal_drop, distal_pad_polygon, pad_margins, pad_spans, pip_footprint, proximal_drop, proximal_pad_polygon
from core.morphology.packaging.gaps import footprint_gaps
from core.morphology.packaging.shapes import polygon_to_disk
from core.optimization.analysis.tradeoff import Modules, goal_of, with_modules

TOLERANCE_MM = 1e-6
"""Numerical only: a margin this far below 0 still counts as touching (not an assembly clearance)."""
LINKAGE_SERVO_STEP_DEG = 0.1
"""CoDe's servo sampled this finely (its flexion moves at most ``k_max`` times as fast)."""
REVOLUTE_STEP_DEG = 0.25
"""The 2R's flexion sampled this finely, ``phi_req`` itself included."""
REFINE_MM = 0.05
"""Between samples, how far a moving solid may travel between the poses checked (CoDe's refinement)."""
COLLISION_POLICY = {
    "model": "idealized 2.5D: solids in the XZ side view, with assumed Y-layer separation between "
             "the links and the actuators; no 3D or stack-up check (the layers are the prototype's to validate)",
    "checked_in_xz_projection": [
        "proximal module - distal module",
        "module - servo body (the PIP servo, the MCP servo)",
        "module - joint pin housings (its own segment's and the other's)",
        "module - links and bars of the other body (capsules, link_radius_mm)",
        "module - the other segment's support; distal support - proximal module and support",
        "proximal support - the servo body on its own segment",
        "distal carrier - PIP servo",
    ],
    "not_checked": {
        "link or bar - servo body": "Y-layer separation assumed (the links beside the actuators), for CoDe and the 2R alike",
        "link - link": "Y-layer separation assumed",
        "driven link at its actuator horn (CoDe A'-B at A'; the 2R's distal truss at A)": "the intended drive connection",
        "module or support - its own body's links and pins at its mounting": "the intended attachment",
    },
}
"""Which solids are checked against which (``metadata.json`` records it)."""
REASONS = ("feasible", "support_exceeded", "kinematic_unreachable", "pad_pad_collision", "pad_servo_collision",
           "pad_joint_collision", "pad_linkage_collision", "pad_support_collision", "structure_collision",
           "invalid_fixed_layout")


@dataclass(frozen=True)
class FitResult:
    feasible: bool
    reason: str
    first_failure_deg: float | None = None
    collision_pair: str | None = None


@dataclass(frozen=True)
class Motion:
    """A fixed layout's poses from extension to the first at or past
    ``phi_req``: their flexions and, for CoDe, the posed ``B``, ``C``, ``B'``."""

    phi_deg: np.ndarray
    reached: bool
    b: np.ndarray | None = None
    c: np.ndarray | None = None
    bp: np.ndarray | None = None
    linkage: FingerLinkage | None = None
    servo_rad: np.ndarray | None = None
    """CoDe's servo angle at each pose (to resample between them)."""


def motion(mechanism: str, x, config, phi_req_deg: float) -> Motion:
    """The layout's motion to ``phi_req_deg`` (the modules do not change it)."""

    if mechanism == "revolute":
        phi = np.append(np.arange(0.0, phi_req_deg, REVOLUTE_STEP_DEG), phi_req_deg)
        return Motion(phi, True)
    from core.morphology.linkage.design import decode

    linkage = FingerLinkage(decode(x, config.link_length_resolution_mm))
    step = math.radians(LINKAGE_SERVO_STEP_DEG)
    sense = 1.0 if linkage.sweep_servo([0.0, step]).q_pip_rad[-1] >= 0.0 else -1.0
    sweep = linkage.sweep_servo(sense * np.arange(0.0, 2.0 * math.pi, step))
    phi = np.degrees(sweep.total_flexion_rad)
    reached = np.maximum.accumulate(phi)
    at = np.flatnonzero(reached >= phi_req_deg - 1e-9)
    # Past the four-bar's closure, or turning back short of phi_req: not reached.
    last = int(at[0]) if at.size else len(phi) - 1
    ok = bool(at.size) and bool(np.all(np.diff(phi[:last + 1]) >= -1e-9))
    window = slice(0, last + 1)
    return Motion(phi[window], ok, sweep.b[window], sweep.c[window], sweep.bp[window], linkage,
                  (sense * np.arange(0.0, 2.0 * math.pi, step))[:len(phi)][window])


def reason_of(pair: tuple[str, str]) -> str:
    names = set(pair)
    if not names & {"proximal_pad", "distal_pad"}:
        return "structure_collision"
    if {"proximal_pad", "distal_pad"} <= names:
        return "pad_pad_collision"
    if any(name.startswith("link_") for name in names):
        return "pad_linkage_collision"
    if any("carrier" in name for name in names):
        return "pad_support_collision"
    if any("servo" in name for name in names):
        return "pad_servo_collision"
    if any("pin" in name for name in names):
        return "pad_joint_collision"
    return "structure_collision"


def layout_violations(mechanism: str, x, config) -> dict[str, float]:
    """The fixed structure's own margins (mm, negative: overlap), whatever the
    modules, as the optimization's static checks: the PIP servo against the
    MCP servo; CoDe's pin A against the servo on A' (``a_pin_servo_check``)
    and the two pins on each body (A and B' on the driver link, B' and B on
    the distal one) apart, their housings at most touching."""

    packaging = goal_of(config).packaging
    clearance = packaging.body_clearance_mm
    if mechanism == "revolute":
        from core.optimization.revolute.evaluate import geometry

        servo = geometry(x, config).distal_servo
        return {"pip_servo/mcp_servo": packaging.mcp_servo_gap_mm(servo, (0.0,)) - clearance}
    from core.morphology.linkage.design import decode

    linkage = FingerLinkage(decode(x, config.link_length_resolution_mm))
    footprint, radius = pip_footprint(packaging, linkage), packaging.pivot_radius_mm
    found = {"pip_servo/mcp_servo": packaging.mcp_servo_gap_mm(footprint, (0.0,)) - clearance,
             "a_pin/bp_pin": linkage.a.distance_to(linkage.bp) - radius("a") - radius("bp"),
             "bp_pin/b_pin": linkage.bp.distance_to(linkage.b) - radius("bp") - radius("b")}
    if packaging.a_pin_servo_check:
        found["a_pin/pip_servo"] = (footprint.distance_to_point(np.array((linkage.a.x_mm, linkage.a.z_mm)))
                                    - radius("a") - clearance)
    return found


def first_dip(params: np.ndarray, least: np.ndarray, travel: np.ndarray, least_at, refine_mm: float = REFINE_MM):
    """Where between samples the least margin may dip below 0 unseen: the first
    interval (up to the first failing sample) whose two margins sum to less
    than ``travel`` (the most a moving solid moves across it), resampled by
    ``least_at(params) -> least margins`` into pieces moving at most
    ``refine_mm``; returns the first failing parameter found, else ``None``."""

    failing = np.flatnonzero(least < -TOLERANCE_MM)
    end = int(failing[0]) if failing.size else len(params) - 1
    for i in range(end):
        if least[i] + least[i + 1] >= travel[i]:
            continue  # neither can reach 0 before meeting the other
        count = min(int(math.ceil(travel[i] / refine_mm)), 400)
        inner = np.linspace(params[i], params[i + 1], count + 1)[1:-1]
        if not inner.size:
            continue
        dips = np.flatnonzero(least_at(inner) < -TOLERANCE_MM)
        if dips.size:
            return float(inner[dips[0]])
    return None


def _linkage_margins(found_motion: Motion, config) -> dict[tuple[str, str], np.ndarray]:
    from core.morphology.linkage.packaging import link_capsule_margins, support_margins

    linkage, packaging = found_motion.linkage, config.packaging
    b, c, bp = found_motion.b, found_motion.c, found_motion.bp
    margins = dict(pad_margins(linkage, packaging, b, c, bp, (0.0,)))
    if config.link_radius_mm is not None:
        margins.update(link_capsule_margins(linkage, packaging, b, c, bp, config.link_radius_mm))
    if config.proximal_carrier_height_mm is not None:
        margins.update(support_margins(linkage, packaging, b, c, config.proximal_carrier_height_mm))
    # Each pad against its own segment's pin housings (one body: unchecked by pad_margins).
    count = len(b)
    proximal = np.broadcast_to(proximal_pad_polygon(linkage, packaging), (count, 4, 2))
    distal = distal_pad_polygon(linkage, packaging, b, c)
    radius = packaging.pivot_radius_mm
    # The proximal pad against the PIP servo on A' (one body with it).
    margins[("proximal_pad", "pip_servo")] = footprint_gaps(proximal, pip_footprint(packaging, linkage), packaging) \
        - packaging.body_clearance_mm
    for pad, polygon, pins in (("proximal_pad", proximal, (("a", (linkage.a.x_mm, linkage.a.z_mm)),
                                                            ("ap", (linkage.ap.x_mm, linkage.ap.z_mm)))),
                               ("distal_pad", distal, (("b", b), ("bp", bp)))):
        for name, centre in pins:
            centre = np.broadcast_to(np.asarray(centre, dtype=float), (count, 2))
            margins[(pad, f"{name}_pin")] = polygon_to_disk(polygon, centre, radius(name)) - packaging.body_clearance_mm
    return margins


def _revolute_margins(phi_deg: np.ndarray, x, config) -> dict[tuple[str, str], np.ndarray]:
    from core.optimization.revolute.evaluate import _margins, geometry

    geo = geometry(x, config)
    margins = dict(_margins(geo, np.radians(phi_deg), config))
    packaging = config.goal.packaging
    count = len(phi_deg)
    proximal = np.broadcast_to(geo.proximal_pad, (count, *geo.proximal_pad.shape))
    # The proximal pad against the joint's housing it shares a body with (the servo, fixed on the proximal segment).
    if config.joint_servo:
        margins[("proximal_pad", "pip_servo")] = footprint_gaps(proximal, geo.distal_servo, packaging) \
            - packaging.body_clearance_mm
    return margins


def _linkage_dip(found_motion: Motion, config, margins) -> FitResult | None:
    """A collision between CoDe's motion samples (:func:`first_dip`): the
    distal body's points travel at most ``|dB| + |d phi| R`` (``R``: its
    reach from ``B``), the crossing links' ends ``|dB|`` and ``|dB'|``."""

    from core.morphology.linkage.packaging import distal_carrier_polygon

    linkage, packaging = found_motion.linkage, config.packaging
    b, c, bp = found_motion.b, found_motion.c, found_motion.bp
    solids = [distal_pad_polygon(linkage, packaging, b[:1], c[:1])[0], c[:1], bp[:1]]
    if packaging.distal_carrier_height_mm > 0.0:
        solids.append(distal_carrier_polygon(linkage, packaging, b[:1], c[:1])[0])
    reach = max(float(np.linalg.norm(np.asarray(solid) - b[0], axis=-1).max()) for solid in solids)
    turn = np.abs(np.diff(np.radians(found_motion.phi_deg)))
    travel = 1.05 * np.maximum(np.linalg.norm(np.diff(b, axis=0), axis=1) + turn * reach,
                               np.linalg.norm(np.diff(bp, axis=0), axis=1))
    shape = found_motion.phi_deg.shape
    least = np.min(np.stack([np.broadcast_to(m, shape) for m in margins.values()]), axis=0)

    def pose(servo: np.ndarray) -> tuple[Motion, dict]:
        sweep = linkage.sweep_servo(servo)
        sub = Motion(np.degrees(sweep.total_flexion_rad), True, sweep.b, sweep.c, sweep.bp, linkage, servo)
        return sub, _linkage_margins(sub, config)

    def least_at(servo: np.ndarray) -> np.ndarray:
        sub, found = pose(servo)
        return np.min(np.stack([np.broadcast_to(m, sub.phi_deg.shape) for m in found.values()]), axis=0)

    servo = first_dip(found_motion.servo_rad, least, travel, least_at)
    if servo is None:
        return None
    sub, found = pose(np.array([servo]))
    pair = min(found, key=lambda p: float(np.broadcast_to(found[p], (1,))[0]))
    return FitResult(False, reason_of(pair), float(sub.phi_deg[0]), "/".join(pair))


def support_rooms(mechanism: str, x, config) -> tuple[float, float]:
    """Each support's room for a module (mm): from the module's outer end to ``A''`` / ``B''``."""

    if mechanism == "revolute":
        from core.optimization.revolute.evaluate import geometry

        geo = geometry(x, config)
        return geo.proximal_room_mm, geo.distal_room_mm
    from core.morphology.linkage.design import decode

    spans = pad_spans(decode(x, config.link_length_resolution_mm), config.packaging)
    return spans["proximal_pad"].room_mm, spans["distal_pad"].room_mm


def evaluate_fit(mechanism: str, x, base_config, modules: Modules, phi_req_deg: float,
                 found_motion: Motion | None = None) -> FitResult:
    """Whether the fixed layout ``x`` carries ``modules`` from extension to
    ``phi_req_deg`` (``found_motion``: the layout's :func:`motion`, reused)."""

    config = with_modules(base_config, modules)
    try:
        proximal_room, distal_room = support_rooms(mechanism, x, config)
        found_motion = found_motion or motion(mechanism, x, config, phi_req_deg)
    except ValueError:  # the design vector describes no closed four-bar
        return FitResult(False, "invalid_fixed_layout")
    try:
        structure = layout_violations(mechanism, x, config)
    except ValueError:
        return FitResult(False, "invalid_fixed_layout")
    broken = min(structure, key=structure.get)
    if structure[broken] < -TOLERANCE_MM:
        return FitResult(False, "invalid_fixed_layout", collision_pair=broken)
    if (modules.proximal_length_mm > proximal_room + TOLERANCE_MM
            or modules.distal_length_mm > distal_room + TOLERANCE_MM):
        return FitResult(False, "support_exceeded")
    shape = found_motion.phi_deg.shape
    if mechanism == "revolute":
        from core.optimization.revolute.evaluate import _between_samples, geometry

        margins = _revolute_margins(found_motion.phi_deg, x, config)
        # Between samples: the evaluator's own resampling on the verification grid.
        moving = {p: np.broadcast_to(m, shape).copy() for p, m in margins.items() if p != ("proximal_pad", "pip_servo")}
        margins = {**margins, **_between_samples(geometry(x, config), np.radians(found_motion.phi_deg), moving, config)}
    else:
        margins = _linkage_margins(found_motion, config)
        dip = _linkage_dip(found_motion, config, margins)
        if dip is not None:
            return dip
    stacked = np.stack([np.broadcast_to(m, shape) for m in margins.values()])
    failing = np.flatnonzero((stacked < -TOLERANCE_MM).any(axis=0))
    if failing.size:
        index = int(failing[0])
        pair = min(margins, key=lambda p: float(np.broadcast_to(margins[p], shape)[index]))
        return FitResult(False, reason_of(pair), float(found_motion.phi_deg[index]), "/".join(pair))
    if not found_motion.reached:
        return FitResult(False, "kinematic_unreachable", float(found_motion.phi_deg.max()))
    return FitResult(True, "feasible")


def max_flexion_deg(mechanism: str, x, base_config, modules: Modules, limit_deg: float = 180.0) -> tuple[float, FitResult]:
    """How far the fixed layout flexes with ``modules``: ``limit_deg`` when it
    fits all through, else the flexion it fails at (the first colliding pose,
    or the most the four-bar reaches), with the failure."""

    fit = evaluate_fit(mechanism, x, base_config, modules, limit_deg)
    return (limit_deg if fit.feasible else fit.first_failure_deg if fit.first_failure_deg is not None else 0.0), fit


def layout_summary(mechanism: str, x, base_config, modules: Modules) -> dict:
    """A fixed layout as it is audited: its design vector, length, active and
    passive joints, supports (their room for a module, their ends ``A''`` and
    ``B''``) and the modules' mounting offsets (drops) and outer ends."""

    from core.optimization.mechanism import mechanism as mechanism_of

    config = with_modules(base_config, modules)
    goal = goal_of(config)
    rooms = support_rooms(mechanism, x, config)
    v = dict(zip(mechanism_of(mechanism).variable_names, (float(value) for value in x)))
    info = {"design_vector": [float(value) for value in x], "variables": v,
            "support_room_mm": {"proximal": rooms[0], "distal": rooms[1]},
            "proximal_module_outer_end_x_mm": goal.packaging.proximal_pad_start_mm,
            "pip_servo": {"model_radius_mm": goal.packaging.pip_servo.radius_mm,
                          "rear_extent_mm": goal.packaging.pip_servo.rear_extent_mm}}
    if mechanism == "linkage":
        from core.morphology.linkage.design import decode

        p = decode(x, config.link_length_resolution_mm)
        info.update({"finger_length_mm": math.hypot(p.b_x_mm + p.bc_length_mm, p.b_z_mm),
                     "active_joints": {"O": [0.0, 0.0], "A'": [p.ap.x_mm, p.ap.z_mm]},
                     "passive_joints": {"A": [p.a_x_mm, p.a_z_mm], "B": [p.b_x_mm, p.b_z_mm],
                                        "B'": [p.bp.x_mm, p.bp.z_mm]},
                     "pip_servo_tilt_deg": math.degrees(p.pip_servo_tilt_rad),
                     "supports_end_x_mm": {"A''": p.app_x_mm, "B''": p.bpp_x_mm},
                     "pad_drops_mm": {"proximal": proximal_drop(goal.packaging, p),
                                      "distal": distal_drop(goal.packaging, p)}})
    else:
        info.update({"finger_length_mm": v["a_x_mm"] + v["ac_length_mm"],
                     "active_joints": {"O": [0.0, 0.0], "A": [v["a_x_mm"], 0.0]}, "passive_joints": {},
                     "pip_servo_tilt_deg": v["servo_tilt_deg"],
                     "supports_end_x_mm": {"A''": v["app_x_mm"], "B''": v["a_x_mm"] + v["bpp_from_a_mm"]},
                     "pad_drops_mm": {"proximal": v["proximal_drop_mm"], "distal": v["distal_drop_mm"]}})
    return info


def sweep(layouts: dict, reference: Modules, lengths_mm, heights_mm, phi_req_deg: float) -> list[dict]:
    """Experiments A (the proximal module over ``lengths_mm`` x ``heights_mm``,
    the distal at ``reference``) and B (the reverse) for each layout
    ``{mechanism: (x, base_config)}``: one record per mechanism, experiment and
    module pair; ``layout_reaches``: whether the layout carries the reference
    pair itself through ``phi_req`` (if not, the layout falls short of the
    flexion, whatever a module's record says)."""

    records = []
    for mechanism, (x, base_config) in layouts.items():
        found_motion = motion(mechanism, x, with_modules(base_config, reference), phi_req_deg)
        # The layout itself (its reference modules) through phi_req: else no module "fits" at it.
        reaches = evaluate_fit(mechanism, x, base_config, reference, phi_req_deg, found_motion).feasible
        for experiment in ("A", "B"):
            for length in lengths_mm:
                for height in heights_mm:
                    modules = (Modules(float(length), float(height), reference.distal_length_mm,
                                       reference.distal_height_mm) if experiment == "A" else
                               Modules(reference.proximal_length_mm, reference.proximal_height_mm, float(length),
                                       float(height)))
                    fit = evaluate_fit(mechanism, x, base_config, modules, phi_req_deg, found_motion)
                    records.append({"mechanism": mechanism, "experiment": experiment,
                                    "proximal_length_mm": modules.proximal_length_mm,
                                    "proximal_height_mm": modules.proximal_height_mm,
                                    "distal_length_mm": modules.distal_length_mm,
                                    "distal_height_mm": modules.distal_height_mm,
                                    "required_flexion_deg": phi_req_deg, "layout_reaches": reaches,
                                    "feasible": fit.feasible,
                                    "reason": fit.reason,
                                    "first_failure_deg": "" if fit.first_failure_deg is None else
                                    round(fit.first_failure_deg, 3),
                                    "collision_pair": fit.collision_pair or ""})
    return records


__all__ = ["COLLISION_POLICY", "FitResult", "LINKAGE_SERVO_STEP_DEG", "Motion", "REASONS", "REFINE_MM",
           "REVOLUTE_STEP_DEG", "TOLERANCE_MM", "evaluate_fit", "first_dip", "layout_violations", "layout_summary", "max_flexion_deg", "motion", "reason_of", "support_rooms", "sweep"]
