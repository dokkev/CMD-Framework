"""The interface a finger mechanism plugs into the optimizer with.

A mechanism (the linkage, an open chain, an RCJ, ...) brings its design
vector, its config, its evaluation and its objectives; ``core.optimization.study``
runs NSGA-II on any of them and saves, reloads and compares their runs the
same way. Objectives that mean the same thing across mechanisms share a name
(``transmission``, ``curl``), so their fronts can be put side by side.

To add one: subclass :class:`Mechanism` in its own package and list it in
:data:`MECHANISMS`.
"""

from __future__ import annotations

import importlib
import math
from abc import ABC, abstractmethod
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np
import yaml


INVALID = 1.0e3
"""Objective and constraint value of a design that cannot be evaluated."""


@dataclass(frozen=True)
class Objective:
    """One objective, minimized by ``value``.

    ``column`` is the reported field people read for it (in ``pareto.csv`` and
    ``seeds.csv``), better when larger if ``larger_is_better``; ``role`` names
    the seed that is best at it.
    """

    value: Callable[[Any], float]
    column: str
    label: str
    role: str
    larger_is_better: bool
    default: bool = True
    """Optimized when the config does not list its objectives."""
    column_minimized: Callable[[float], float] | None = None
    """The minimized value from the reported ``column`` (default: the value,
    negated when ``larger_is_better``)."""

    def minimized(self, column_value: float) -> float:
        if self.column_minimized is not None:
            return self.column_minimized(column_value)
        return -column_value if self.larger_is_better else column_value


@dataclass(frozen=True)
class SummaryColumn:
    """One column of the representative-seed table printed after a run."""

    header: str
    width: int
    text: Callable[[Mapping[str, Any]], str]
    """The cell, from a candidate's reported values (:meth:`Mechanism.report`)."""


class StudyConfig(Protocol):
    """What the optimizer reads from every mechanism's config."""

    resolution_mm: float
    """Grid step of the design variables."""
    objectives: tuple[str, ...] | None


class Evaluation(Protocol):
    g: Sequence[float]
    """Constraints, satisfied when ``<= 0``."""
    feasible: bool


class Mechanism(ABC):
    """A finger mechanism the optimizer can search."""

    name: str
    """Registry key, also the ``output/<name>`` and ``scripts/optimize/<name>`` folder."""
    formulation: str
    """Geometry and feasibility rules of its evaluation, recorded in ``run.json``;
    runs of different formulations are not comparable."""
    legacy_formulation: str | None = None
    """The formulation of runs that predate recording it."""
    legacy_objectives: tuple[str, ...] | None = None
    """The objectives of runs that predate recording them."""
    variable_names: tuple[str, ...]
    constraint_names: tuple[str, ...]
    objectives: Mapping[str, Objective]
    summary: tuple[SummaryColumn, ...] = ()

    # Config.

    @abstractmethod
    def config_from_mapping(self, values: Mapping[str, Any]) -> StudyConfig:
        """Build a config from plain values, as written in a config file."""

    def load_yaml(self, path: str | Path) -> StudyConfig:
        """The config of a config file, less its optimizer settings
        (``core.optimization.study.load_yaml`` reads both)."""

        return self.config_from_mapping(self.split_settings(read_config_file(path))[0])

    def split_settings(self, values: Mapping[str, Any]) -> tuple[dict[str, Any], Any]:
        """A config file's values less its optimizer settings, and those
        (``core.optimization.optimizer.split_settings``: its ``optimizer`` table)."""

        from core.optimization.optimizer import split_settings

        return split_settings(values)

    @abstractmethod
    def config_to_json(self, config: StudyConfig) -> dict[str, Any]:
        """The config as JSON values, for ``run.json``."""

    @abstractmethod
    def config_from_json(self, values: dict[str, Any]) -> StudyConfig:
        """Rebuild a config recorded by :meth:`config_to_json`, in this or an earlier version."""

    @abstractmethod
    def search_box(self, config: StudyConfig) -> tuple[Sequence[float], Sequence[float]]:
        """Lower and upper bounds of the search coordinates (:meth:`search_names`;
        by default the design vector, in :attr:`variable_names` order)."""

    # The coordinates the optimizer searches. By default the design vector
    # itself; a mechanism may search others (relative positions, shares of a
    # range) that keep its simplest constraints by construction.

    def search_names(self, config: StudyConfig) -> tuple[str, ...]:
        return tuple(self.variable_names)

    def search_steps(self, config: StudyConfig) -> tuple[float, ...]:
        """Grid step of each search coordinate."""

        return (config.resolution_mm,) * len(self.search_names(config))

    def to_design(self, coordinates: Sequence[float], config: StudyConfig) -> tuple[float, ...] | None:
        """The design vector of search coordinates (``None``: no design)."""

        return tuple(coordinates)

    def from_design(self, x: Sequence[float], config: StudyConfig) -> tuple[float, ...]:
        """Search coordinates of a design vector (to warm-start from it)."""

        return tuple(x)

    def prepare(self, config: StudyConfig) -> None:
        """Compute once what every evaluation of the study reuses (cached
        tables), before the evaluating processes fork and inherit it."""

    def refine(self, config: StudyConfig, step_rad: float) -> StudyConfig:
        """The config to verify candidates with: its motion sampled every ``step_rad``."""

        return config

    # Designs.

    @abstractmethod
    def evaluate(self, x: Sequence[float], config: StudyConfig) -> Evaluation | None:
        """Evaluate a design vector; ``None`` when it describes no mechanism."""

    @abstractmethod
    def report(self, evaluation: Evaluation) -> dict[str, Any]:
        """The values written for a candidate, by column (numbers and strings)."""

    def read_design(self, row: Mapping[str, str]) -> list[float]:
        """Design vector of a CSV row."""

        return [float(row[name]) for name in self.variable_names]

    # Objectives.

    def active_objectives(self, config: StudyConfig) -> tuple[str, ...]:
        """The objectives a study optimizes, in :attr:`objectives` order: the ones
        its config lists, or else the default ones."""

        if config.objectives is not None:
            unknown = set(config.objectives) - set(self.objectives)
            if unknown:
                raise ValueError(f"unknown objectives {sorted(unknown)}; known: {list(self.objectives)}")
            return tuple(name for name in self.objectives if name in config.objectives)
        return tuple(name for name, objective in self.objectives.items() if objective.default)

    def objective_values(self, evaluation: Evaluation, names: Sequence[str]) -> tuple[float, ...]:
        """The named objectives of one evaluation, as the optimizer minimizes them."""

        def finite(value: float) -> float:
            # A design without a value (e.g. never swept) ranks behind every other.
            return value if math.isfinite(value) else INVALID

        return tuple(finite(self.objectives[name].value(evaluation)) for name in names)

    def seed_roles(
        self,
        objectives: Sequence[Sequence[float]],
        columns: Sequence[Mapping[str, Any]],
        names: Sequence[str],
    ) -> dict[str, int]:
        """The representative seeds of a front: role -> candidate index.

        ``objectives`` are the candidates' minimized objectives (``names``) and
        ``columns`` their reported values. Each objective's ``role`` goes to the
        candidate best at it (ties to the better first objective); ``balanced``
        to the one closest to the ideal point with each objective scaled to
        ``[0, 1]`` over the front.
        """

        if not objectives:
            return {}
        scores = np.asarray(objectives, dtype=float)
        low, high = scores.min(axis=0), scores.max(axis=0)
        scaled = (scores - low) / np.where(high > low, high - low, 1.0)
        roles = {}
        for name in names:
            objective = self.objectives[name]
            sign = 1.0 if objective.larger_is_better else -1.0
            roles[objective.role] = max(range(len(columns)), key=lambda i: (
                sign * float(columns[i][objective.column]), -scores[i, 0]))
        roles["balanced"] = int(np.argmin(np.linalg.norm(scaled, axis=1)))
        return roles

    def role_order(self, names: Sequence[str]) -> list[str]:
        """The seed roles of a study, in the order they are listed."""

        return [self.objectives[name].role for name in names] + ["balanced"]

    def summary_line(self, row: Mapping[str, Any]) -> str:
        return " ".join(f"{column.text(row):>{column.width}s}" for column in self.summary)

    def summary_header(self) -> str:
        return " ".join(f"{column.header:>{column.width}s}" for column in self.summary)


MECHANISMS = {
    "linkage": "core.optimization.linkage.mechanism",
    "linkage_v3": "core.optimization.linkage.mechanism",  # runs recorded under that name (2026-10-23 to 26)
    "open_chain_idealized": "core.optimization.open_chain.mechanism:IDEALIZED",
    "open_chain_packaged": "core.optimization.open_chain.mechanism:PACKAGED",
    "revolute": "core.optimization.revolute.mechanism",
    "rolling_contact_joint": "core.optimization.rolling_contact_joint.mechanism",
}
"""Every mechanism, by name: ``module`` (its ``MECHANISM``) or ``module:NAME``."""


def mechanism(name: str) -> Mechanism:
    """The registered mechanism of this name."""

    if name not in MECHANISMS:
        raise ValueError(f"unknown mechanism {name!r}; known: {sorted(MECHANISMS)}")
    module, _, attribute = MECHANISMS[name].partition(":")
    return getattr(importlib.import_module(module), attribute or "MECHANISM")


def read_config_file(path: str | Path, overrides: Sequence[str] = ()) -> dict[str, Any]:
    """A config file's values. A ``base`` entry names a file (relative to this
    one) whose values it extends, table by table; ``overrides`` are
    ``key.subkey=value`` (YAML values) applied last."""

    path = Path(path)
    with path.open(encoding="utf-8") as stream:
        values = yaml.safe_load(stream) or {}
    base = values.pop("base", None)
    if base is not None:
        values = _merged(read_config_file(path.parent / base), values)
    for override in overrides:
        key, separator, text = override.partition("=")
        if not separator:
            raise ValueError(f"override {override!r} is not key=value")
        *tables, last = key.strip().split(".")
        target = values
        for table in tables:
            target = target.setdefault(table, {})
        target[last] = yaml.safe_load(text)
    return values


def _merged(base: Mapping[str, Any], values: Mapping[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in values.items():
        if isinstance(value, Mapping) and isinstance(merged.get(key), Mapping):
            merged[key] = _merged(merged[key], value)
        else:
            merged[key] = value
    return merged


__all__ = [
    "INVALID",
    "MECHANISMS",
    "Evaluation",
    "Mechanism",
    "Objective",
    "StudyConfig",
    "SummaryColumn",
    "mechanism",
    "read_config_file",
]
