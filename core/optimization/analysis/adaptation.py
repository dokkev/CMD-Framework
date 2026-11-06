"""Morphology adaptation for contact accommodation (Study 2): how far a
reference hand must change to carry a given contact interface.

A hand is three groups of parameters (:class:`Partition`, one per
mechanism, mapping its own design vector onto them):

- ``E``, external morphology, as the common descriptor
  :data:`DESCRIPTORS`: the finger's length, its proximal and distal
  contact-bearing spans (the segments ``|OA''|`` and ``|B''C|``, as every
  evaluator measures them) and its articulation station (the extended centre
  of rotation, ``core.morphology.icr``);
- ``Q``, mechanism geometry, the mechanism's own (CoDe's pivots, the RCJ's
  roller; the 2R has none once ``E`` is fixed);
- ``A``, actuation, held fixed throughout (the servo model, its mount, tilt).

The contact interface (the modules' lengths and thicknesses) is fixed per
target (``tradeoff.with_modules``), and each pad's mounting depth is set by
the goal's mounting policy for that interface (``core.optimization.mounting``),
never searched.

Each change is measured as the largest change of its group, normalized by
one engineering scale (the reference finger's length):
``eps_E = max_i |e_i - e0_i| / s`` and ``eps_Q = max_j |q_j - q0_j| / s``.
The two are reported separately, never summed.

The hierarchy (:func:`adapt`), each step a single-objective constrained
search with the existing optimizer (``core.optimization.study.run_study``)
on the existing evaluator (``Mechanism.evaluate``), which alone decides
feasibility (mounting, collisions, the required flexion, the modules usable):

1. the reference as it is (``E``, ``Q``, ``A`` fixed): feasible, stop;
2. Level 1: ``min eps_Q`` with ``E = E0``, ``A = A0``;
3. Level 2, only if Level 1 found nothing: ``min eps_E`` over ``E`` and
   ``Q`` (``A = A0``), then ``min eps_Q`` with ``eps_E <= eps_E* + delta``
   (lexicographic: the external change first).

"Not found" means the optimizer found no feasible design in its range and
budget, not that none exists.
"""

from __future__ import annotations

import dataclasses
import math
from abc import ABC, abstractmethod
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from core.optimization.mechanism import INVALID, Mechanism, Objective
from core.optimization.optimizer import OptimizerSettings

DESCRIPTORS = ("finger_length_mm", "proximal_span_mm", "distal_span_mm", "articulation_mm")
"""The external morphology every mechanism is described by (``E``)."""
OFFSETS = tuple(f"d_{name}" for name in DESCRIPTORS)
"""Level 2 searches ``E`` as offsets from the reference (0 on the grid, whatever ``E0`` is)."""
COSTS = ("mechanism_cost", "external_cost")
CAP = "external_cap"
"""The constraint holding ``eps_E`` to Level 2's least (``eps_E* + delta``)."""


class Partition(ABC):
    """A mechanism's design vector split into ``E``, ``Q`` and ``A``.

    The search moves ``E`` (as :data:`DESCRIPTORS`) and the mechanism
    coordinates :attr:`coordinates`; :meth:`compose` builds the design from
    them, the actuation and anything else taken from a template design, and
    the mounting depths from the policy. ``Q`` is measured by
    :meth:`mechanism`, physical values in mm (pivot positions, roller size).
    """

    mechanisms: tuple[str, ...]
    """The registered mechanisms it maps."""
    coordinates: tuple[str, ...]
    """The mechanism coordinates searched (``Q``'s freedom with ``E`` fixed)."""
    actuation: tuple[str, ...]
    """Design variables held from the template (``A``)."""

    @abstractmethod
    def external(self, x: Sequence[float], config) -> dict[str, float]:
        """``E`` of a design (:data:`DESCRIPTORS`, mm)."""

    @abstractmethod
    def mechanism(self, x: Sequence[float], config) -> dict[str, float]:
        """``Q`` of a design, its physical values (mm)."""

    @abstractmethod
    def coordinate_values(self, x: Sequence[float], config) -> dict[str, float]:
        """The search coordinates of a design (:attr:`coordinates`)."""

    @abstractmethod
    def coordinate_bounds(self, config) -> dict[str, tuple[float, float]]:
        """Each coordinate's range: the mechanism's valid design bounds."""

    @abstractmethod
    def compose(self, external: Mapping[str, float], coordinates: Mapping[str, float],
                template: Sequence[float], config) -> tuple[float, ...] | None:
        """The design with ``external``, ``coordinates`` and ``template``'s
        actuation, its depths set by the mounting policy (``None``: no design,
        or one its supports cannot carry)."""


def change(values: Mapping[str, float], reference: Mapping[str, float], scale: float) -> float:
    """The largest normalized change, ``max |v - v0| / scale`` (0 with none)."""

    return max((abs(values[name] - reference[name]) / scale for name in reference), default=0.0)


@dataclass(frozen=True)
class AdaptationEvaluation:
    """A design's own evaluation (``base``) and its changes from the reference."""

    base: Any
    external: Mapping[str, float]
    mechanism: Mapping[str, float]
    external_cost: float
    mechanism_cost: float
    cap: float = 0.0
    """``eps_E`` less its allowed most (``<= 0``: within it)."""
    constraints: Mapping[str, float] = field(default_factory=dict)

    @property
    def g(self) -> tuple[float, ...]:
        return (*self.base.g, self.cap)

    @property
    def feasible(self) -> bool:
        return bool(self.base.feasible) and self.cap <= 0.0


class AdaptationMechanism(Mechanism):
    """A mechanism searched only where a level lets it: its design built by a
    :class:`Partition` from the free groups, the rest the reference's;
    minimizing one change (``objective``), evaluated by the mechanism itself."""

    def __init__(self, base: Mechanism, partition: Partition, reference: Sequence[float], config,
                 free: str, objective: str, scale_mm: float, external_range_mm: float = 0.0,
                 external_cap: float | None = None) -> None:
        if free not in ("mechanism", "external"):
            raise ValueError("free must be 'mechanism' (Level 1) or 'external' (Level 2: E and Q)")
        if objective not in COSTS:
            raise ValueError(f"objective must be one of {COSTS}")
        self.base, self.partition, self.free, self.objective = base, partition, free, objective
        self.reference = tuple(float(v) for v in reference)
        self.scale_mm, self.external_range_mm, self.external_cap = scale_mm, external_range_mm, external_cap
        self.reference_external = partition.external(self.reference, config)
        self.reference_mechanism = partition.mechanism(self.reference, config)
        self.reference_coordinates = partition.coordinate_values(self.reference, config)
        self.name = f"adaptation:{base.name}"
        self.formulation = f"{base.formulation}+adaptation-v1"
        self.variable_names = base.variable_names
        self.constraint_names = (*base.constraint_names, CAP)
        self.objectives = {
            "mechanism_cost": Objective(lambda e: e.mechanism_cost, "mechanism_cost", "eps_Q", "least_mechanism",
                                        False),
            "external_cost": Objective(lambda e: e.external_cost, "external_cost", "eps_E", "least_external",
                                       False),
        }

    # The search: the free groups only.

    def search_names(self, config) -> tuple[str, ...]:
        coordinates = tuple(self.partition.coordinates)
        return (*OFFSETS, *coordinates) if self.free == "external" else coordinates

    def search_box(self, config):
        bounds = self.partition.coordinate_bounds(config)
        box = {name: bounds[name] for name in self.partition.coordinates}
        if self.free == "external":
            box |= {name: (-self.external_range_mm, self.external_range_mm) for name in OFFSETS}
        names = self.search_names(config)
        return tuple(box[name][0] for name in names), tuple(box[name][1] for name in names)

    def search_steps(self, config) -> tuple[float, ...]:
        return (config_resolution(config),) * len(self.search_names(config))

    def to_design(self, coordinates: Sequence[float], config) -> tuple[float, ...] | None:
        values = dict(zip(self.search_names(config), (float(c) for c in coordinates)))
        external = {name: self.reference_external[name] + values.get(f"d_{name}", 0.0) for name in DESCRIPTORS}
        moved = {name: values.get(name, self.reference_coordinates[name]) for name in self.partition.coordinates}
        if self.free == "mechanism":
            external = dict(self.reference_external)  # E fixed, exactly
        return self.partition.compose(external, moved, self.reference, config)

    def from_design(self, x: Sequence[float], config) -> tuple[float, ...]:
        external = self.partition.external(x, config)
        values = {**{f"d_{name}": external[name] - self.reference_external[name] for name in DESCRIPTORS},
                  **self.partition.coordinate_values(x, config)}
        return tuple(values[name] for name in self.search_names(config))

    # Designs: the mechanism's own evaluation, and the changes.

    def evaluate(self, x: Sequence[float], config) -> AdaptationEvaluation | None:
        try:
            base = self.base.evaluate(x, config)
        except ValueError:  # a composed design that builds no mechanism (CoDe's bars not closing)
            return None
        if base is None:
            return None
        if isinstance(getattr(base, "constraints", None), Mapping):  # picklable across processes
            base = dataclasses.replace(base, constraints=dict(base.constraints))
        external = self.partition.external(x, config)
        mechanism = self.partition.mechanism(x, config)
        external_cost = change(external, self.reference_external, self.scale_mm)
        mechanism_cost = change(mechanism, self.reference_mechanism, self.scale_mm)
        if not all(math.isfinite(v) for v in (external_cost, mechanism_cost)):
            external_cost = mechanism_cost = INVALID
        cap = 0.0 if self.external_cap is None else external_cost - self.external_cap
        return AdaptationEvaluation(base, external, mechanism, external_cost, mechanism_cost, cap,
                                    {**dict(base.constraints), CAP: cap})

    def active_objectives(self, config) -> tuple[str, ...]:
        return (self.objective,)

    def report(self, evaluation: AdaptationEvaluation) -> dict[str, Any]:
        return {**self.base.report(evaluation.base), "external_cost": evaluation.external_cost,
                "mechanism_cost": evaluation.mechanism_cost,
                **{f"e_{name}": value for name, value in evaluation.external.items()},
                **{f"q_{name}": value for name, value in evaluation.mechanism.items()}}

    def config_from_mapping(self, values):  # the base mechanism's config is used as is
        return self.base.config_from_mapping(values)

    def config_to_json(self, config):
        return self.base.config_to_json(config)

    def config_from_json(self, values):
        return self.base.config_from_json(values)

    def prepare(self, config) -> None:
        self.base.prepare(config)


def with_finger_lengths(config, low_mm: float, high_mm: float):
    """``config`` with the finger's length allowed in ``[low_mm, high_mm]`` (Level 2 moves it)."""

    from core.optimization.analysis.tradeoff import goal_of, with_goal

    return with_goal(config, dataclasses.replace(goal_of(config), extended_length_bounds_mm=(float(low_mm),
                                                                                          float(high_mm))))


def config_resolution(config) -> float:
    return float(getattr(config, "resolution_mm", None) or getattr(config, "goal").resolution_mm)


# The hierarchy --------------------------------------------------------------

LEVELS = ("frozen", "level1", "level2_external", "level2_mechanism")


@dataclass(frozen=True)
class Step:
    """One step of :func:`adapt`: its status and the best design it found."""

    level: str
    status: str
    """``feasible`` (the reference itself), ``found``, ``not_found`` or
    ``no_freedom`` (Level 1 of a mechanism with no ``Q`` left once ``E`` is fixed)."""
    x: tuple[float, ...] | None = None
    evaluation: AdaptationEvaluation | None = None
    evaluations: int = 0
    result: Any = None
    """The optimizer's :class:`~core.optimization.study.StudyResult` (``None``: no search)."""


@dataclass(frozen=True)
class AdaptationResult:
    """The hierarchy's outcome for one reference and contact interface."""

    status: str
    """``frozen_feasible``, ``level1``, ``level2`` or ``not_found``."""
    steps: tuple[Step, ...]
    final: Step | None

    @property
    def mechanism_cost(self) -> float:
        return math.nan if self.final is None else self.final.evaluation.mechanism_cost

    @property
    def external_cost(self) -> float:
        return math.nan if self.final is None else self.final.evaluation.external_cost

    def step(self, level: str) -> Step | None:
        return next((step for step in self.steps if step.level == level), None)


def _best(result, mechanism: AdaptationMechanism, objective: str):
    """The feasible design least in ``objective`` (``None``: none feasible)."""

    feasible = [c for c in result.pareto if c.evaluation.feasible]
    if not feasible:
        return None
    return min(feasible, key=lambda c: (getattr(c.evaluation, objective),
                                        c.evaluation.mechanism_cost if objective == "external_cost"
                                        else c.evaluation.external_cost))


def _search(base: Mechanism, partition: Partition, reference, config, settings: OptimizerSettings, level: str,
            free: str, objective: str, scale_mm: float, external_range_mm: float = 0.0,
            external_cap: float | None = None, warm: Sequence[Sequence[float]] = (), workers: int = 1) -> Step:
    from core.optimization.study import run_study

    adapted = AdaptationMechanism(base, partition, reference, config, free, objective, scale_mm,
                                  external_range_mm, external_cap)
    if not adapted.search_names(config):
        return Step(level, "no_freedom")
    result = run_study(adapted, config, settings, initial_designs=[w for w in warm if w is not None], workers=workers)
    best = _best(result, adapted, objective)
    if best is None:
        return Step(level, "not_found", evaluations=result.evaluation_count, result=result)
    return Step(level, "found", best.x, best.evaluation, result.evaluation_count, result)


def adapt(base: Mechanism, partition: Partition, reference: Sequence[float], config,
          settings: OptimizerSettings, scale_mm: float, external_range_mm: float,
          workers: int = 1) -> AdaptationResult:
    """The hierarchy for one reference ``reference`` (a design vector) and the
    contact interface ``config`` carries (``tradeoff.with_modules``)."""

    reference = tuple(float(v) for v in reference)
    frozen_mechanism = AdaptationMechanism(base, partition, reference, config, "mechanism", "mechanism_cost",
                                           scale_mm)
    # The reference itself, its depths reset by the policy for this interface.
    frozen_x = partition.compose(frozen_mechanism.reference_external, frozen_mechanism.reference_coordinates,
                                 reference, config)
    frozen_eval = None if frozen_x is None else frozen_mechanism.evaluate(frozen_x, config)
    if frozen_eval is not None and frozen_eval.feasible:
        frozen = Step("frozen", "feasible", frozen_x, frozen_eval, 1)
        return AdaptationResult("frozen_feasible", (frozen,), frozen)
    frozen = Step("frozen", "infeasible", frozen_x, frozen_eval, 1)

    level1 = _search(base, partition, reference, config, settings, "level1", "mechanism", "mechanism_cost",
                     scale_mm, warm=[frozen_x], workers=workers)
    if level1.status == "found":
        return AdaptationResult("level1", (frozen, level1), level1)

    length = frozen_mechanism.reference_external["finger_length_mm"]
    free_config = with_finger_lengths(config, length - external_range_mm, length + external_range_mm)
    first = _search(base, partition, reference, free_config, settings, "level2_external", "external",
                    "external_cost", scale_mm, external_range_mm, warm=[frozen_x], workers=workers)
    if first.status != "found":
        return AdaptationResult("not_found", (frozen, level1, first), None)
    # Lexicographic: among the least external changes, the least mechanism change.
    delta = config_resolution(config) / scale_mm
    second = _search(base, partition, reference, free_config, settings, "level2_mechanism", "external",
                     "mechanism_cost",
                     scale_mm, external_range_mm, external_cap=first.evaluation.external_cost + delta,
                     warm=[first.x], workers=workers)
    final = second if second.status == "found" else first
    return AdaptationResult("level2", (frozen, level1, first, second), final)


__all__ = ["CAP", "COSTS", "DESCRIPTORS", "LEVELS", "OFFSETS", "with_finger_lengths", "AdaptationEvaluation", "AdaptationMechanism",
           "AdaptationResult", "Partition", "Step", "adapt", "change"]
