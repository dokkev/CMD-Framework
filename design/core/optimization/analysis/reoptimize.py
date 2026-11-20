"""Study 2 (problem formulation A.6.3): a morphology re-optimized for a given
contact interface.

For an interface ``c`` (the two pads' lengths and thicknesses), a mechanism's
design is searched again in its own search space (as its native design was)
for the pad coverage, with the pads fixed to ``c`` and the finger's length in
a set range (:func:`interface_config`). Where the optimum is flat the optimal
morphology is not one design, so among the designs within ``delta`` of the
best coverage the one whose external morphology is closest to the reference's
is reported (:class:`ClosestMechanism`): closest in its motion (the fingertip's
path over the required flexion, :func:`tip_path`; A.6.3) or in its external
morphology.
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from core.optimization.analysis.adaptation import Partition, change, config_resolution, with_finger_lengths
from core.optimization.analysis.tradeoff import Modules, with_modules
from core.optimization.mechanism import Mechanism, Objective
from core.optimization.optimizer import OptimizerSettings

TIP_SAMPLES = 61
"""Flexions the fingertip's path is compared at, evenly over ``[0, q_req]``."""
OBJECTIVES = ("kinematic_cost", "external_cost")
FLOOR = "coverage_floor"
"""The constraint holding the coverage within ``delta`` of the best found."""
INVALID = 1.0e6


def tip_path(mechanism_name: str, x: Sequence[float], config, q_req_rad: float,
             samples: int = TIP_SAMPLES):
    """The fingertip ``C`` (``(samples, 2)``, mm, proximal frame) at the distal
    flexions ``0 .. q_req``, MCP at 0 (``None``: the joint does not pose them all).

    At one flexion the distal phalanx has one orientation whatever the
    mechanism, so the tip's position is its whole pose: two designs whose tip
    paths agree move their distal phalanx alike."""

    import numpy as np

    from core.morphology.mechanism import base_mechanism

    q = np.linspace(0.0, float(q_req_rad), int(samples))
    base = base_mechanism(mechanism_name)
    try:
        if base == "linkage":
            from core.morphology.mechanism.linkage.design import decode

            sweep = decode(x, config.link_length_resolution_mm).sweep_pip(q)
            tips = np.asarray(sweep.tip, dtype=float)
        else:
            import importlib

            module = importlib.import_module(f"core.optimization.{mechanism_name}.evaluate"
                                             if mechanism_name == "revolute_parallel"
                                             else f"core.optimization.{base}.evaluate")
            tips = np.asarray(module.geometry(x, config).tip(q), dtype=float)
    except (ValueError, ZeroDivisionError):
        return None
    if tips.shape != (len(q), 2) or not np.all(np.isfinite(tips)):
        return None
    return tips


def path_change(path, reference, scale_mm: float) -> float:
    """The largest distance between two tip paths over the flexion, per ``scale_mm`` (INVALID: none)."""

    import numpy as np

    if path is None or reference is None:
        return INVALID
    return float(np.max(np.linalg.norm(np.asarray(path) - np.asarray(reference), axis=1)) / scale_mm)


def interface_config(config, modules: Modules, finger_length_mm: tuple[float, float] | None = None):
    """``config`` asking for the interface ``modules`` (each pad of its set
    length and thickness, its least support its own length), the finger's
    length within ``finger_length_mm`` (``None``: as the config has it)."""

    carried = with_modules(config, modules)
    return carried if finger_length_mm is None else with_finger_lengths(carried, *finger_length_mm)


@dataclass(frozen=True)
class ClosestEvaluation:
    """A design's own evaluation (``base``), its morphology and its change from the reference."""

    base: Any
    external: Mapping[str, float]
    mechanism: Mapping[str, float]
    external_cost: float
    mechanism_cost: float
    kinematic_cost: float = INVALID
    """The fingertip path's largest change from the reference's, per the scale (:func:`path_change`)."""
    floor: float = 0.0
    """The coverage floor less the design's coverage (``<= 0``: within ``delta`` of the best)."""
    constraints: Mapping[str, float] = field(default_factory=dict)

    @property
    def g(self) -> tuple[float, ...]:
        return (*self.base.g, self.floor)

    @property
    def feasible(self) -> bool:
        return bool(self.base.feasible) and self.floor <= 0.0

    @property
    def c_pad_mean(self) -> float:
        return float(self.base.c_pad_mean)


class ClosestMechanism(Mechanism):
    """``base`` searched in its own coordinates for the design closest to a
    reference's, its coverage held at ``floor`` or more: closest in its motion
    (``kinematic_cost``, the fingertip path; needs ``reference_path``) or in its
    external morphology (``external_cost``)."""

    def __init__(self, base: Mechanism, partition: Partition, reference_external: Mapping[str, float],
                 reference_mechanism: Mapping[str, float], scale_mm: float, floor: float = -math.inf,
                 reference_path=None, objective: str = "kinematic_cost") -> None:
        if objective not in OBJECTIVES:
            raise ValueError(f"objective must be one of {OBJECTIVES}")
        if objective == "kinematic_cost" and reference_path is None:
            raise ValueError("closest in motion needs the reference's tip path")
        self.base, self.partition, self.scale_mm, self.floor = base, partition, float(scale_mm), float(floor)
        self.reference_external, self.reference_mechanism = dict(reference_external), dict(reference_mechanism)
        self.reference_path, self.objective = reference_path, objective
        self.name = f"closest:{base.name}"
        self.formulation = f"{base.formulation}+closest-v2"
        self.variable_names = base.variable_names
        self.constraint_names = (*base.constraint_names, FLOOR)
        self.objectives = {
            "kinematic_cost": Objective(lambda e: e.kinematic_cost, "kinematic_cost", "eps_kin", "least_motion",
                                        False),
            "external_cost": Objective(lambda e: e.external_cost, "external_cost", "eps_d", "least_external", False),
        }

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
        return (self.objective,)

    def evaluate(self, x: Sequence[float], config) -> ClosestEvaluation | None:
        try:
            base = self.base.evaluate(x, config)
        except ValueError:
            return None
        if base is None:
            return None
        if isinstance(getattr(base, "constraints", None), Mapping):  # picklable across processes
            base = dataclasses.replace(base, constraints=dict(base.constraints))
        try:
            external, mechanism = self.partition.external(x, config), self.partition.mechanism(x, config)
            external_cost = change(external, self.reference_external, self.scale_mm)
            mechanism_cost = change(mechanism, self.reference_mechanism, self.scale_mm)
        except (ValueError, KeyError, ZeroDivisionError):
            external, mechanism, external_cost, mechanism_cost = {}, {}, INVALID, INVALID
        if not all(math.isfinite(v) for v in (external_cost, mechanism_cost)):
            external_cost = mechanism_cost = INVALID
        kinematic_cost = INVALID
        if self.reference_path is not None and base.feasible:  # the motion of a feasible design only
            q_req = float(getattr(config, "required_rom_rad", None) or goal_rom(config))
            kinematic_cost = path_change(tip_path(self.base.name, x, config, q_req), self.reference_path,
                                         self.scale_mm)
        coverage = float(getattr(base, "c_pad_mean", 0.0) or 0.0)
        floor = self.floor - coverage if math.isfinite(self.floor) else 0.0
        return ClosestEvaluation(base, external, mechanism, external_cost, mechanism_cost, kinematic_cost, floor,
                                 {**dict(base.constraints), FLOOR: floor})

    def report(self, evaluation: ClosestEvaluation) -> dict[str, Any]:
        return {**self.base.report(evaluation.base), "external_cost": evaluation.external_cost,
                "mechanism_cost": evaluation.mechanism_cost, "kinematic_cost": evaluation.kinematic_cost}

    def config_from_mapping(self, values):
        return self.base.config_from_mapping(values)

    def config_to_json(self, config):
        return self.base.config_to_json(config)

    def config_from_json(self, values):
        return self.base.config_from_json(values)


@dataclass(frozen=True)
class Design:
    """A design found for an interface: its coverage, morphology and change from the reference."""

    x: tuple[float, ...]
    evaluation: ClosestEvaluation

    @property
    def coverage(self) -> float:
        return self.evaluation.c_pad_mean


def goal_rom(config) -> float:
    """The required flexion (rad) a study config asks (its goal's)."""

    from core.optimization.analysis.tradeoff import goal_of

    return float(goal_of(config).required_rom_rad)


def measure(base: Mechanism, partition: Partition, x: Sequence[float], config, reference_external,
            reference_mechanism, scale_mm: float, reference_path=None) -> Design | None:
    """``x`` evaluated under ``config`` with its change from the reference (``None``: no design)."""

    found = ClosestMechanism(base, partition, reference_external, reference_mechanism, scale_mm,
                             reference_path=reference_path,
                             objective="kinematic_cost" if reference_path is not None else "external_cost"
                             ).evaluate(x, config)
    return None if found is None else Design(tuple(float(v) for v in x), found)


def _warm(base: Mechanism, config, designs: Sequence[Sequence[float] | None]) -> list:
    """The designs a search of ``config`` can start from."""

    found = []
    for x in designs:
        if x is None:
            continue
        try:
            values = base.from_design(x, config)
        except (ValueError, KeyError, ZeroDivisionError):
            continue
        if all(math.isfinite(float(v)) for v in values):
            found.append(tuple(x))
    return found


def best_coverage(base: Mechanism, partition: Partition, config, settings: OptimizerSettings, reference_external,
                  reference_mechanism, scale_mm: float, warm: Sequence[Sequence[float] | None] = (),
                  workers: int = 1, reference_path=None) -> tuple[Design | None, int]:
    """The design of the most coverage a search of ``config`` finds (the
    mechanism's own search and objective), and the evaluations it took."""

    from core.optimization.study import run_study

    result = run_study(base, config, settings, initial_designs=_warm(base, config, warm), workers=workers)
    feasible = [c for c in result.pareto if c.evaluation.feasible]
    if not feasible:
        return None, result.evaluation_count
    top = max(feasible, key=lambda c: float(c.evaluation.c_pad_mean))
    return (measure(base, partition, top.x, config, reference_external, reference_mechanism, scale_mm,
                    reference_path), result.evaluation_count)


def closest(base: Mechanism, partition: Partition, config, settings: OptimizerSettings, reference_external,
            reference_mechanism, scale_mm: float, floor: float, warm: Sequence[Sequence[float] | None] = (),
            workers: int = 1, reference_path=None, objective: str = "kinematic_cost") -> tuple[Design | None, int]:
    """Among the designs of coverage ``floor`` or more, the one closest to the
    reference's: in its motion (``kinematic_cost``) or its external morphology
    (``external_cost``); ties by the other changes."""

    from core.optimization.study import run_study

    mechanism = ClosestMechanism(base, partition, reference_external, reference_mechanism, scale_mm, floor,
                                 reference_path, objective)
    result = run_study(mechanism, config, settings, initial_designs=_warm(base, config, warm), workers=workers)
    found = [Design(tuple(c.x), c.evaluation) for c in result.pareto if c.evaluation.feasible]
    # A warm design that already meets the floor competes too (the search keeps only its front).
    for x in _warm(base, config, warm):
        design = mechanism.evaluate(x, config)
        if design is not None and design.feasible:
            found.append(Design(tuple(x), design))
    if not found:
        return None, result.evaluation_count
    order = ("kinematic_cost", "external_cost", "mechanism_cost") if objective == "kinematic_cost" else \
        ("external_cost", "mechanism_cost", "kinematic_cost")
    return (min(found, key=lambda d: tuple(getattr(d.evaluation, name) for name in order)),
            result.evaluation_count)


__all__ = ["FLOOR", "OBJECTIVES", "TIP_SAMPLES", "ClosestEvaluation", "ClosestMechanism", "Design", "best_coverage",
           "closest", "config_resolution", "interface_config", "measure", "path_change", "tip_path"]
