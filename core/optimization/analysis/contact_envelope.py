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

The first failure's pair names the reason (``REASONS``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from core.morphology.linkage import FingerLinkage
from core.morphology.linkage.packaging import (
    _polygon_to_disk,
    distal_pad_polygon,
    footprint_gaps,
    pad_margins,
    pad_spans,
    proximal_pad_polygon,
)
from core.optimization.analysis.tradeoff import Modules, goal_of, with_modules

TOLERANCE_MM = 1e-6
"""Numerical only: a margin this far below 0 still counts as touching (not an assembly clearance)."""
LINKAGE_SERVO_STEP_DEG = 0.1
"""CoDe's servo sampled this finely (its flexion moves at most ``k_max`` times as fast)."""
REVOLUTE_STEP_DEG = 0.25
"""The 2R's flexion sampled this finely, ``phi_req`` itself included."""
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


def motion(mechanism: str, x, config, phi_req_deg: float) -> Motion:
    """The layout's motion to ``phi_req_deg`` (the modules do not change it)."""

    if mechanism == "revolute":
        phi = np.append(np.arange(0.0, phi_req_deg, REVOLUTE_STEP_DEG), phi_req_deg)
        return Motion(phi, True)
    from core.optimization.linkage.design import decode

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
    return Motion(phi[window], ok, sweep.b[window], sweep.c[window], sweep.bp[window], linkage)


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


def _linkage_margins(found_motion: Motion, config) -> dict[tuple[str, str], np.ndarray]:
    from core.optimization.linkage.evaluate import link_capsule_margins, support_margins

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
    margins[("proximal_pad", "pip_servo")] = footprint_gaps(proximal, packaging.pip_footprint(linkage), packaging) \
        - packaging.body_clearance_mm
    for pad, polygon, pins in (("proximal_pad", proximal, (("a", (linkage.a.x_mm, linkage.a.z_mm)),
                                                            ("ap", (linkage.ap.x_mm, linkage.ap.z_mm)))),
                               ("distal_pad", distal, (("b", b), ("bp", bp)))):
        for name, centre in pins:
            centre = np.broadcast_to(np.asarray(centre, dtype=float), (count, 2))
            margins[(pad, f"{name}_pin")] = _polygon_to_disk(polygon, centre, radius(name)) - packaging.body_clearance_mm
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


def support_rooms(mechanism: str, x, config) -> tuple[float, float]:
    """Each support's room for a module (mm): from the module's outer end to ``A''`` / ``B''``."""

    if mechanism == "revolute":
        from core.optimization.revolute.evaluate import geometry

        geo = geometry(x, config)
        return geo.proximal_room_mm, geo.distal_room_mm
    from core.optimization.linkage.design import decode

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
    if (modules.proximal_length_mm > proximal_room + TOLERANCE_MM
            or modules.distal_length_mm > distal_room + TOLERANCE_MM):
        return FitResult(False, "support_exceeded")
    margins = (_revolute_margins(found_motion.phi_deg, x, config) if mechanism == "revolute"
               else _linkage_margins(found_motion, config))
    stacked = np.stack([np.broadcast_to(m, found_motion.phi_deg.shape) for m in margins.values()])
    failing = np.flatnonzero((stacked < -TOLERANCE_MM).any(axis=0))
    if failing.size:
        index = int(failing[0])
        pair = min(margins, key=lambda p: float(np.broadcast_to(margins[p], found_motion.phi_deg.shape)[index]))
        return FitResult(False, reason_of(pair), float(found_motion.phi_deg[index]), "/".join(pair))
    if not found_motion.reached:
        return FitResult(False, "kinematic_unreachable", float(found_motion.phi_deg.max()))
    return FitResult(True, "feasible")


def sweep(layouts: dict, reference: Modules, lengths_mm, heights_mm, phi_req_deg: float) -> list[dict]:
    """Experiments A (the proximal module over ``lengths_mm`` x ``heights_mm``,
    the distal at ``reference``) and B (the reverse) for each layout
    ``{mechanism: (x, base_config)}``: one record per mechanism, experiment and module pair."""

    records = []
    for mechanism, (x, base_config) in layouts.items():
        found_motion = motion(mechanism, x, with_modules(base_config, reference), phi_req_deg)
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
                                    "required_flexion_deg": phi_req_deg, "feasible": fit.feasible,
                                    "reason": fit.reason,
                                    "first_failure_deg": "" if fit.first_failure_deg is None else
                                    round(fit.first_failure_deg, 3),
                                    "collision_pair": fit.collision_pair or ""})
    return records


__all__ = ["FitResult", "LINKAGE_SERVO_STEP_DEG", "Motion", "REASONS", "REVOLUTE_STEP_DEG", "TOLERANCE_MM",
           "evaluate_fit", "motion", "reason_of", "support_rooms", "sweep"]
