"""Study 2 (problem formulation A.6.3): a finger redesigned for a thicker
interface with the relative contact trajectory as its target.

The relative contact trajectory of a finger is the path of its distal contact
face seen from its proximal contact face, as a function of the flexion (not of
time; the transmission from servo to flexion is not part of it). The faces turn against each other by
the flexion ``q`` whatever the design, so the path of one point of the distal
face, taken in the proximal face's frame, is the whole of it
(:func:`contact_path`). A redesign is searched in the mechanism's own search
space for two objectives at once (:class:`ContactMechanism`): the least change
of that path from a reference finger's, and the most pad coverage (the study's
own objective). Its front says what coverage an architecture keeps at each
change of the contact's motion.
"""

from __future__ import annotations

import dataclasses
import importlib
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from core.optimization.analysis.tradeoff import goal_of
from core.optimization.mechanism import Mechanism, Objective
from core.optimization.optimizer import OptimizerSettings

PATH_SAMPLES = 61
"""Flexions the paths are compared at, evenly over ``[0, q_req]``."""
INVALID = 1.0e6


def tip_path(mechanism_name: str, x: Sequence[float], config, q_rad: np.ndarray) -> np.ndarray:
    """The fingertip ``C`` (``(n, 2)``, mm, proximal frame) at the distal
    flexions ``q_rad``, MCP at 0; fewer rows where the joint does not pose them all."""

    from core.morphology.mechanism import base_mechanism

    base = base_mechanism(mechanism_name)
    if base == "linkage":
        from core.morphology.mechanism.linkage.design import decode

        return np.asarray(decode(x, config.link_length_resolution_mm).sweep_pip(q_rad).tip, dtype=float)
    module = importlib.import_module(f"core.optimization.{mechanism_name}.evaluate"
                                     if mechanism_name == "revolute_parallel" else f"core.optimization.{base}.evaluate")
    return np.asarray(module.geometry(x, config).tip(q_rad), dtype=float)


def contact_path(tips: np.ndarray, q_rad: np.ndarray, proximal_depth_mm: float, distal_depth_mm: float) -> np.ndarray:
    """The distal contact face's point under the fingertip, from the proximal
    contact face's line: the tip, the face's depth below it in the distal
    frame (turned by ``-q``: flexion is toward the palm), and the proximal
    face's depth back."""

    q = np.asarray(q_rad, dtype=float)[:len(tips)]
    below = np.stack((-distal_depth_mm * np.sin(q), -distal_depth_mm * np.cos(q)), axis=1)
    return np.asarray(tips, dtype=float) + below + np.array((0.0, proximal_depth_mm))


def path_change(path, reference, scale_mm: float) -> float:
    """The largest distance between two paths over the flexion, per ``scale_mm`` (INVALID: no path)."""

    if path is None or reference is None or len(path) != len(reference) or not np.all(np.isfinite(path)):
        return INVALID
    return float(np.max(np.linalg.norm(np.asarray(path) - np.asarray(reference), axis=1)) / scale_mm)


def paths(base: Mechanism, x: Sequence[float], config, evaluation=None, samples: int = PATH_SAMPLES):
    """A design's skeleton path (the fingertip's) and contact path over ``[0, q_req]``
    (``None``: it has no evaluation or does not pose them all)."""

    if evaluation is None:
        try:
            evaluation = base.evaluate(x, config)
        except ValueError:
            return None
    if evaluation is None:
        return None
    report = base.report(evaluation)
    q = np.linspace(0.0, float(goal_of(config).required_rom_rad), int(samples))
    try:
        tips = tip_path(base.name, x, config, q)
    except (ValueError, ZeroDivisionError):
        return None
    depths = report.get("proximal_depth_mm"), report.get("distal_depth_mm")
    if len(tips) != len(q) or not np.all(np.isfinite(tips)) or None in depths or "" in depths:
        return None
    return {"q": q, "tip": tips, "contact": contact_path(tips, q, float(depths[0]), float(depths[1]))}


@dataclass(frozen=True)
class Redesigned:
    """A design's own evaluation (``base``), its contact path's change from the
    reference finger's and its pad coverage."""

    base: Any
    contact_change: float
    c_pad_mean: float
    constraints: Mapping[str, float] = field(default_factory=dict)

    @property
    def g(self) -> tuple[float, ...]:
        return tuple(self.base.g)

    @property
    def feasible(self) -> bool:
        return bool(self.base.feasible)


def _picklable(evaluation):
    if isinstance(getattr(evaluation, "constraints", None), Mapping):
        return dataclasses.replace(evaluation, constraints=dict(evaluation.constraints))
    return evaluation


class ContactMechanism(Mechanism):
    """``base`` searched in its own coordinates for the least change of the
    contact path from ``reference`` (``(samples, 2)``, per ``scale_mm``) and
    the most pad coverage."""

    def __init__(self, base: Mechanism, reference: np.ndarray, scale_mm: float) -> None:
        self.base, self.reference, self.scale_mm = base, np.asarray(reference, dtype=float), float(scale_mm)
        self.variable_names = base.variable_names
        self.name, self.formulation = f"contact:{base.name}", f"{base.formulation}+contact-v1"
        self.constraint_names = tuple(base.constraint_names)
        self.objectives = {
            "contact_change": Objective(lambda e: e.contact_change, "contact_change", "D_c", "least_contact_change",
                                        False),
            "c_pad_mean": Objective(lambda e: -e.c_pad_mean, "c_pad_mean", "C_pad", "most_coverage", True)}

    def search_names(self, config):
        return self.base.search_names(config)

    def search_box(self, config):
        return self.base.search_box(config)

    def search_steps(self, config):
        return self.base.search_steps(config)

    def to_design(self, coordinates, config):
        return self.base.to_design(coordinates, config)

    def from_design(self, x, config):
        return self.base.from_design(x, config)

    def prepare(self, config) -> None:
        self.base.prepare(config)

    def active_objectives(self, config) -> tuple[str, ...]:
        return tuple(self.objectives)

    def config_from_mapping(self, values):
        return self.base.config_from_mapping(values)

    def config_to_json(self, config):
        return self.base.config_to_json(config)

    def config_from_json(self, values):
        return self.base.config_from_json(values)

    def evaluate(self, x: Sequence[float], config) -> Redesigned | None:
        try:
            base = self.base.evaluate(x, config)
        except ValueError:
            return None
        if base is None:
            return None
        base = _picklable(base)
        change, coverage = INVALID, 0.0
        if base.feasible:
            found = paths(self.base, x, config, base, len(self.reference))
            change = INVALID if found is None else path_change(found["contact"], self.reference, self.scale_mm)
            value = self.base.report(base).get("c_pad_mean")
            coverage = float(value) if value not in (None, "") and math.isfinite(float(value)) else 0.0
        return Redesigned(base, change, coverage, dict(base.constraints))

    def report(self, evaluation) -> dict[str, Any]:
        return {"contact_change": evaluation.contact_change, "c_pad_mean": evaluation.c_pad_mean}


def front(points: Sequence[tuple[float, float]]) -> list[int]:
    """The indices of the points no other beats (less change, no less coverage), by rising change."""

    order = sorted(range(len(points)), key=lambda i: (points[i][0], -points[i][1]))
    kept, best = [], -math.inf
    for i in order:
        if points[i][1] > best + 1e-12:
            kept.append(i)
            best = points[i][1]
    return kept


def redesign(base: Mechanism, config, reference: np.ndarray, scale_mm: float, settings: OptimizerSettings,
             warm: Sequence[Sequence[float] | None] = (), workers: int = 1):
    """The front of the redesigns of ``config``'s interface: ``[(x, Redesigned)]``
    by rising change of the contact path, and the evaluations it took. The
    starts (``warm``) that are designs compete too."""

    from core.optimization.study import run_study

    mechanism = ContactMechanism(base, reference, scale_mm)
    starts = []
    for x in warm:
        if x is None:
            continue
        try:
            values = base.from_design(x, config)
        except (ValueError, KeyError, ZeroDivisionError):
            continue
        if all(math.isfinite(float(v)) for v in values):
            starts.append(tuple(x))
    result = run_study(mechanism, config, settings, initial_designs=starts, workers=workers)
    found = [(tuple(c.x), c.evaluation) for c in result.pareto if c.evaluation.feasible]
    for x in starts:
        evaluation = mechanism.evaluate(x, config)
        if evaluation is not None and evaluation.feasible:
            found.append((x, evaluation))
    found = [f for f in found if f[1].contact_change < INVALID]
    keep = front([(e.contact_change, e.c_pad_mean) for _, e in found])
    return [found[i] for i in keep], result.evaluation_count


__all__ = ["INVALID", "PATH_SAMPLES", "ContactMechanism", "Redesigned", "contact_path", "front", "path_change",
           "paths", "redesign", "tip_path"]
